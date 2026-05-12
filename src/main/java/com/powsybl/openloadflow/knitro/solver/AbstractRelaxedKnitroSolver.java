/**
 * Copyright (c) 2025, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.artelys.knitro.api.*;
import com.artelys.knitro.api.callbacks.KNEvalGACallback;
import com.powsybl.commons.PowsyblException;
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.equations.*;
import com.powsybl.openloadflow.network.LfBus;
import com.powsybl.openloadflow.network.LfNetwork;
import com.powsybl.openloadflow.util.PerUnit;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * Abstract class for relaxed Knitro solvers, solving the open load flow equation system by minimizing constraint violations through relaxation.
 * It provides common functionality, including:
 *      - Post-processing of the computed solutions, including the reporting of relaxation variables.
 *      - An extended optimization problem formulation dedicated to solving the open load flow equation system, including relaxations.
 * This class can be extended to add custom behavior to any of these features (e.g., in {@link com.powsybl.openloadflow.knitro.solver.UseReactiveLimitsKnitroSolver}).
 * For example, if you modify the optimization problem, you may also need to update the solution-processing logic.
 *
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 */
public abstract class AbstractRelaxedKnitroSolver extends AbstractKnitroSolver {

    private static final Logger LOGGER = LoggerFactory.getLogger(AbstractRelaxedKnitroSolver.class);
    private static final double BASE_100MVA = 100.0;    // Penalty weights in the objective function
    protected static double WEIGHT_P_1;
    protected static final double WEIGHT_Q_1 = 1.0;
    protected static double WEIGHT_P_2;
    protected static double WEIGHT_Q_2;

    protected static final double P_THRESHOLD = 100.0; // MW
    protected static final double Q_THRESHOLD = 100.0; // MW
    protected static final double GAMMA_FACTOR = 0.1;

    // Total number of variables (including power flow and slack variables)
    protected int numSlackVariables;

    // Number of equations for active power (P), reactive power (Q), and voltage magnitude (V)
    protected final int numPEquations;
    protected final int numQEquations;
    protected final int numVEquations;

    // Starting indices for slack variables in the variable vector
    protected final int slackPStartIndex;
    protected final int slackQStartIndex;
    protected final int slackVStartIndex;

    // Mappings from global equation indices to local indices by equation type
    protected final Map<Integer, Integer> pEquationLocalIds;
    protected final Map<Integer, Integer> qEquationLocalIds;
    protected final Map<Integer, Integer> vEquationLocalIds;

    // Mapping of gamma : each Voltage Level is assign to a gamma depending on its nominal voltage
    protected HashMap<Double, Double> voltageLevelGammaMap;
    protected HashMap<Integer, Double> omegaVMap;

    protected AbstractRelaxedKnitroSolver(LfNetwork network, KnitroSolverParameters knitroParameters, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                          JacobianMatrix<AcVariableType, AcEquationType> j, TargetVector<AcVariableType, AcEquationType> targetVector,
                                          EquationVector<AcVariableType, AcEquationType> equationVector, boolean detailedReport) {
        super(network, knitroParameters, equationSystem, j, targetVector, equationVector, detailedReport);

        List<SingleEquation<AcVariableType, AcEquationType>> sortedEquations = equationSystem.getIndex().getSortedSingleEquationsToSolve();
        // Count number of equations by type
        this.numPEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_P).count();
        this.numQEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_Q).count();
        this.numVEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_V).count();

        this.numSlackVariables = 2 * (numPEquations + numQEquations + numVEquations);

        // the slack variables start after power flow variables
        this.slackPStartIndex = equationSystem.getIndex().getSortedVariablesToFind().size();
        this.slackQStartIndex = slackPStartIndex + 2 * numPEquations;
        this.slackVStartIndex = slackQStartIndex + 2 * numQEquations;

        // Map equations to local indices
        this.pEquationLocalIds = new HashMap<>();
        this.qEquationLocalIds = new HashMap<>();
        this.vEquationLocalIds = new HashMap<>();

        int pCounter = 0;
        int qCounter = 0;
        int vCounter = 0;

        // MAP GAMMA : VL in kV and the corresponding gamma
        voltageLevelGammaMap = new HashMap<>();
        voltageLevelGammaMap.put(72.5, 39.55);
        voltageLevelGammaMap.put(145.0, 100.45);
        voltageLevelGammaMap.put(245.0, 212.18);
        voltageLevelGammaMap.put(420.0, 458.30);

        omegaVMap = new HashMap<Integer, Double>();

        for (int i = 0; i < sortedEquations.size(); i++) {
            AcEquationType type = sortedEquations.get(i).getType();
            double gammaValue = 0.0;

            switch (type) {
                case BUS_TARGET_P -> pEquationLocalIds.put(i, pCounter++);
                case BUS_TARGET_Q -> qEquationLocalIds.put(i, qCounter++);
                case BUS_TARGET_V -> {
                    // Set WEIGHT_V_1 based on the nominal voltage of the bus and the corresponding gamma
                    LfBus vlInfo = network.getBus(sortedEquations.get(i).getElementNum());
                    gammaValue = getGammaValues(vlInfo);
                    if (gammaValue == 0.0 || Double.isNaN(gammaValue) || Double.isInfinite(gammaValue)) {
                        throw new PowsyblException("Gamma value is not define for bus " + vlInfo.getId() + " with nominal voltage " + vlInfo.getNominalV() + " kV. Please check the voltage level gamma mapping.");
                    }
                    omegaVMap.put(vCounter, gammaValue); // for each index of V  I have the corresponding gamma
                    vEquationLocalIds.put(i, vCounter++);
                }
                default -> {
                    // Other equation types don't require slack variables
                }
            }
        }

        // Weight P
        double activeGeneration = computeActiveGeneration(network);
        double deltaP = computeDeltaP(network, activeGeneration, this.knitroParameters.getLosses());
        if (deltaP == 0 || Double.isNaN(deltaP)) {
            throw new PowsyblException("DIVIDED BY ZERO: DeltaP is equal to 0, cannot compute WEIGHT_P_1. Please check that the network has non-zero active power generation and load, and/or adjust the losses parameter.");
        }

        // Weight P
        WEIGHT_P_1 = getWeightP1(activeGeneration, deltaP);
        WEIGHT_P_2 = getWeightP2();

        // Weight Q
        WEIGHT_Q_2 = getWeightQ2();
    }

    protected double getGammaValues(LfBus vlInfo) {
        double gamma = 0.0;
        if (vlInfo.getNominalV() <= 85.0) {
            gamma = voltageLevelGammaMap.get(72.5);
        } else if (vlInfo.getNominalV() > 85.0 && vlInfo.getNominalV() <= 200.0) {
            gamma = voltageLevelGammaMap.get(145.0);
        } else if (vlInfo.getNominalV() > 200.0 && vlInfo.getNominalV() <= 350.0) {
            gamma = voltageLevelGammaMap.get(245.0);
        } else if (vlInfo.getNominalV() > 350.0) {
            gamma = voltageLevelGammaMap.get(420.0);
        }
        return gamma;
    }

    private static double getWeightP1(double activeGeneration, double deltaP) {
        return WEIGHT_P_1 = activeGeneration / (10 * deltaP);
    }

    private static double getWeightP2() {
        return WEIGHT_P_1 * BASE_100MVA / (2 * P_THRESHOLD);
    }

    private static double getWeightQ2() {
        return WEIGHT_Q_1 * BASE_100MVA / (2 * Q_THRESHOLD);
    }

    @Override
    protected void processSolution(KNSolver solver, KNSolution solution, KNProblem problemInstance) {
        super.processSolution(solver, solution, problemInstance);

        List<Double> x = solution.getX();

        // ========== Slack Logging ==========
        logSlackValues("P", slackPStartIndex, numPEquations, x);
        logSlackValues("Q", slackQStartIndex, numQEquations, x);
        logSlackValues("V", slackVStartIndex, numVEquations, x);

        // ========== Penalty Computation ==========
        double penaltyP = computeSlackPenalty(x, slackPStartIndex, numPEquations, WEIGHT_P_1, WEIGHT_P_2);
        double penaltyQ = computeSlackPenalty(x, slackQStartIndex, numQEquations, WEIGHT_Q_1, WEIGHT_Q_2);
        double penaltyV = computeSlackPenaltyTypeV(x, slackVStartIndex, numVEquations, omegaVMap);
        double totalPenalty = penaltyP + penaltyQ + penaltyV;

        LOGGER.info("==== Slack penalty details ====");
        LOGGER.info("Penalty P = {}", penaltyP);
        LOGGER.info("Penalty Q = {}", penaltyQ);
        LOGGER.info("Penalty V = {}", penaltyV);
        LOGGER.info("Total penalty = {}", totalPenalty);

        // Weight use in the objective function
        LOGGER.info("Total LOSSES DC (ABSTRACT) =  {} MW", this.knitroParameters.getLosses());
        LOGGER.info("Weight P1 = {}", WEIGHT_P_1);
        LOGGER.info("Weight P2 = {}", WEIGHT_P_2);
        LOGGER.info("Weight Q1 = {}", WEIGHT_Q_1);
        LOGGER.info("Weight Q2 = {}", WEIGHT_Q_2);
        LOGGER.info("Gamma values :" + omegaVMap.entrySet().stream()
                .collect(Collectors.groupingBy(Map.Entry::getValue, Collectors.counting()))
                .entrySet().stream()
                .map(e -> e.getValue() + " x " + e.getKey())
                .collect(Collectors.joining(", ")));
    }

    /**
     * Logs information like bus name and slack value for most significant slack variables.
     *
     * @param type The slack variable type.
     * @param startIndex The start index of slack variables associated to the given type.
     * @param count The maximum number of slack variables associated to the given type.
     * @param x The variable values as returned by solver.
     */
    protected void logSlackValues(String type, int startIndex, int count, List<Double> x) {
        LOGGER.debug("==== Slack diagnostics for {} (p.u. and physical units) ====", type);
        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            double epsilon = sp - sm;

            // Get significant slack values above threshold
            boolean shouldSkip = Math.abs(epsilon) <= knitroParameters.getSlackThreshold();
            String name = null;
            String interpretation = null;

            if (!shouldSkip) {
                name = getSlackVariableBusName(i, type);

                switch (type) {
                    case "P" -> {
                        interpretation = String.format("ΔP = %.4f p.u. (%.1f MW)", epsilon, epsilon * PerUnit.SB);
                    }
                    case "Q" -> interpretation = String.format("ΔQ = %.4f p.u. (%.1f MVAr)", epsilon, epsilon * PerUnit.SB);
                    case "V" -> {
                        var bus = network.getBusById(name);
                        if (bus == null) {
                            LOGGER.warn("Bus {} not found while logging V slack.", name);
                            shouldSkip = true;
                        } else {
                            interpretation = String.format("ΔV = %.4f p.u. (%.1f kV)", epsilon, epsilon * bus.getNominalV());
                        }
                    }
                    default -> interpretation = "Unknown slack type";
                }
            }

            if (shouldSkip) {
                continue;
            }

            String msg = String.format("Slack %s[ %s ] → Sm = %.4f, Sp = %.4f → %s", type, name, sm, sp, interpretation);
            LOGGER.debug(msg);
        }
    }

    /**
     * Finds the bus associated to a slack variable.
     *
     * @param index The index of the slack variable
     * @param type The slack variable type.
     * @return The id of the bus associated to the slack variable.
     */
    private String getSlackVariableBusName(Integer index, String type) {
        Set<Map.Entry<Integer, Integer>> equationSet = switch (type) {
            case "P" -> pEquationLocalIds.entrySet();
            case "Q" -> qEquationLocalIds.entrySet();
            case "V" -> vEquationLocalIds.entrySet();
            default -> throw new IllegalStateException("Unexpected variable type: " + type);
        };

        Optional<Integer> varIndexOptional = equationSet.stream()
                .filter(entry -> index.equals(entry.getValue()))
                .map(Map.Entry::getKey)
                .findAny();

        int varIndex;
        if (varIndexOptional.isPresent()) {
            varIndex = varIndexOptional.get();
        } else {
            throw new PowsyblException("Variable index associated with slack variable " + type + " was not found");
        }

        LfBus bus = network.getBus(equationSystem.getIndex().getSortedSingleEquationsToSolve().get(varIndex).getElementNum());

        return bus.getId();
    }

    /**
     * Calculates the total loss associated to a slack variable type
     *
     * @param x          The variable values as returned by solver.
     * @param startIndex The start index of slack variables associated to the given type.
     * @param count      The maximum number of slack variables associated to the given type.
     * @param weight1    The weight in front of the given slack variables terms (L1)
     * @param weight2    The weight in front of the given slack variables terms (L2)
     * @return The total penalty associated to the slack variables type.
     */
    double computeSlackPenalty(List<Double> x, int startIndex, int count, double weight1, double weight2) {
        double penalty = 0.0;
        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            double diff = sp - sm;
            penalty += weight2 * (diff * diff); // Quadratic terms
            penalty += weight1 * (sp + sm); // Linear terms
        }
        return penalty;
    }

    /**
     * Calculates the total loss associated to a slack variable of type V
     *
     * @param x          The variable values as returned by solver.
     * @param startIndex The start index of slack variables associated to the given type.
     * @param count      The maximum number of slack variables associated to the given type.
     * @param weight    The weight in front of the given slack variables terms : omegaV depending on the coltage level of the bus
     * @return The total penalty associated to the slack variables type.
     */
    double computeSlackPenaltyTypeV(List<Double> x, int startIndex, int count, HashMap<Integer, Double> weight) {
        double penalty = 0.0;
        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            double diff = sp - sm;
            penalty += weight.get(i) * (diff * diff) / 2; // Quadratic terms
            penalty += GAMMA_FACTOR * weight.get(i) * (sp + sm); // Linear terms
        }
        return penalty;
    }

    /**
     * Calculates Delta P = |Pgen - Pload - Losses|
     *
     * @param network           LfNetwork
     * @param activeGeneration  The total active power genreation in the network
     * @param losses            The approximated losses computed by a DC LoadF
     * @return Delta P
     */
    private double computeDeltaP(LfNetwork network, double activeGeneration, double losses) {
        double activeLoad = (double) 0.0;

        for (LfBus b : network.getBuses()) {
            activeLoad += b.getLoadTargetP() * (double) 100.0;
        }
        return Math.abs(activeGeneration - activeLoad - losses); //minus total Losses
    }

    /**
     * Calculates Active Generation
     *
     * @param network           LfNetwork
     * @return The total active power generation in the network
     */
    private double computeActiveGeneration(LfNetwork network) {
        double activeGeneration = (double) 0.0;

        for (LfBus b : network.getBuses()) {
            activeGeneration += b.getGenerationTargetP() * (double) 100.0;
        }
        return activeGeneration;
    }

    /**
     * Optimization problem-solving the open load flow equation system by minimizing constraint violations through relaxation.
     */
    public abstract class AbstractRelaxedKnitroProblem extends AbstractKnitroProblem {

        /**
         * Relaxed Knitro problem definition including:
         * - initialization of variables (types, bounds, initial state)
         * - definition of linear constraints
         * - definition of non-linear constraints, evaluated in extended the callback function
         * - definition of the extended Jacobian matrix passed to Knitro to solve the problem
         * - definition of the objective function to be minimized (equation system violations)
         */
        protected AbstractRelaxedKnitroProblem(LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                               TargetVector<AcVariableType, AcEquationType> targetVector, JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                               KnitroSolverParameters knitroParameters, int numAdditionalVariables, int numAdditionalConstraints) {
            super(network, equationSystem, targetVector, jacobianMatrix, knitroParameters, numAdditionalVariables, numAdditionalConstraints);
        }

        void addObjectiveFunction(int numPEquations, int slackPStartIndex, int numQEquations, int slackQStartIndex,
                                  int numVEquations, int slackVStartIndex) throws KNException {
            // initialise lists to track quadratic objective function terms of the form: a * x1 * x2
            List<Integer> quadRows = new ArrayList<>(); // list of indexes of the first variable x1
            List<Integer> quadCols = new ArrayList<>(); // list of indexes of the second variable x2
            List<Double> quadCoefs = new ArrayList<>(); // list of indexes of the coefficient a

            // initialise lists to track linear objective function terms of the form: a * x
            List<Integer> linIndexes = new ArrayList<>(); // list of indexes of the variable x
            List<Double> linCoefs = new ArrayList<>(); // list of indexes of the coefficient a

            // add slack penalty terms, for each slack type, of the form: (Sp - Sm)^2 = Sp^2 + Sm^2 - 2*Sp*Sm + linear terms from the absolute value
            addSlackObjectiveTerms(numPEquations, slackPStartIndex, AbstractRelaxedKnitroSolver.WEIGHT_P_2, AbstractRelaxedKnitroSolver.WEIGHT_P_1, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);
            addSlackObjectiveTerms(numQEquations, slackQStartIndex, AbstractRelaxedKnitroSolver.WEIGHT_Q_2, AbstractRelaxedKnitroSolver.WEIGHT_Q_1, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);
            addSlackObjectiveTermTypeV(numVEquations, slackVStartIndex, omegaVMap, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);

            setObjectiveQuadraticPart(quadRows, quadCols, quadCoefs);
            setObjectiveLinearPart(linIndexes, linCoefs);
        }

        /**
         * Adds quadratic and linear terms related to slack variables of type P and Q  to the objective function.
         */
        void addSlackObjectiveTerms(int numEquations, int slackStartIdx, double weight2, double weight1,
                                    List<Integer> quadRows, List<Integer> quadCols, List<Double> quadCoefs,
                                    List<Integer> linIndexes, List<Double> linCoefs) {
            for (int i = 0; i < numEquations; i++) {
                int idxSm = slackStartIdx + 2 * i; // negative slack variable index
                int idxSp = slackStartIdx + 2 * i + 1; // positive slack variable index

                // Add quadratic terms: weight2 * (sp^2 + sm^2 - 2 * sp * sm)

                // add first quadratic term : weight2 * sp^2
                quadRows.add(idxSp);
                quadCols.add(idxSp);
                quadCoefs.add(weight2);

                // add second quadratic term : weight2 * sm^2
                quadRows.add(idxSm);
                quadCols.add(idxSm);
                quadCoefs.add(weight2);

                // add third quadratic term : weight2 * (- 2 * sp * sm)
                quadRows.add(idxSp);
                quadCols.add(idxSm);
                quadCoefs.add(-2 * weight2);

                // Add linear terms: weight1 * (sp + sm)

                // add first linear term : weight1 * sp
                linIndexes.add(idxSp);
                linCoefs.add(weight1);

                // add second linear term : weight1 * sm
                linIndexes.add(idxSm);
                linCoefs.add(weight1);
            }
        }

        /**
         * Adds quadratic and linear terms related to slack variables of type V to the objective function.
         */
        void addSlackObjectiveTermTypeV(int numEquations, int slackStartIdx, HashMap weight,
                                        List<Integer> quadRows, List<Integer> quadCols, List<Double> quadCoefs,
                                        List<Integer> linIndexes, List<Double> linCoefs) {

            for (int i = 0; i < numEquations; i++) {
                int idxSm = slackStartIdx + 2 * i; // negative slack variable index
                int idxSp = slackStartIdx + 2 * i + 1; // positive slack variable index

                // Add quadratic terms: weight * (sp^2 + sm^2 - 2 * sp * sm)

                // add first quadratic term : weight * sp^2
                quadRows.add(idxSp);
                quadCols.add(idxSp);
                quadCoefs.add((double) weight.get(i) / 2);

                // add second quadratic term : weight * sm^2
                quadRows.add(idxSm);
                quadCols.add(idxSm);
                quadCoefs.add((double) weight.get(i) / 2);

                // add third quadratic term : weight * (- 2 * sp * sm)
                quadRows.add(idxSp);
                quadCols.add(idxSm);
                quadCoefs.add(-2 * (double) weight.get(i) / 2);

                // Add linear terms: weight * (sp + sm)

                // add first linear term : weight * sp
                linIndexes.add(idxSp);
                linCoefs.add((double) weight.get(i) * GAMMA_FACTOR);

                // add second linear term : weight  * sm
                linIndexes.add(idxSm);
                linCoefs.add((double) weight.get(i) * GAMMA_FACTOR);
            }
        }

        @Override
        protected void initializeCustomizedVariables(List<Double> lowerBounds, List<Double> upperBounds,
                                                     List<Double> initialValues) {
            // set a lower bound to slack variables (>= 0)
            // initial values have already been set to 0
            for (int i = numberOfPowerFlowVariables; i < numTotalVariables; i++) {
                lowerBounds.set(i, 0.0);
            }
        }

        @Override
        protected void addAdditionalConstraintVariables(int equationId, AcEquationType equationType,
                                                        List<Integer> varIndices, List<Double> coefficients) {
            // Add slack variables if applicable
            int slackBase = getSlackIndexBase(equationType, equationId);
            if (slackBase >= 0) {
                varIndices.add(slackBase);       // Sm
                varIndices.add(slackBase + 1);   // Sp
                coefficients.add(1.0);
                coefficients.add(-1.0);
            }
        }

        @Override
        protected void addAdditionalJacobianVariables(int constraintIndex,
                                                      SingleEquation<AcVariableType, AcEquationType> equation,
                                                      List<Integer> variableIndices) {
            AcEquationType equationType = equation.getType();
            // get slack variable local index (within its equation type)
            int slackStart = switch (equationType) {
                case BUS_TARGET_P -> pEquationLocalIds.getOrDefault(constraintIndex, -1);
                case BUS_TARGET_Q -> qEquationLocalIds.getOrDefault(constraintIndex, -1);
                case BUS_TARGET_V -> vEquationLocalIds.getOrDefault(constraintIndex, -1);
                default -> -1;
            };

            if (slackStart >= 0) {
                // get slack variable type starting index (within total variables' indexes)
                int slackBaseIndex = switch (equationType) {
                    case BUS_TARGET_P -> slackPStartIndex;
                    case BUS_TARGET_Q -> slackQStartIndex;
                    case BUS_TARGET_V -> slackVStartIndex;
                    default -> throw new IllegalStateException("Unexpected constraint type: " + equationType);
                };
                // get slack variables Sm and Sp indexes
                variableIndices.add(slackBaseIndex + 2 * slackStart);     // Sm
                variableIndices.add(slackBaseIndex + 2 * slackStart + 1); // Sp
            }
        }

        /**
         * Returns the base index of the slack variable associated with a given equation type and ID.
         *
         * @param equationType Type of the equation (P, Q, or V).
         * @param equationId   Index of the equation.
         * @return Base index of the corresponding slack variable, or -1 if not applicable.
         */
        protected int getSlackIndexBase(AcEquationType equationType, int equationId) {
            return switch (equationType) {
                case BUS_TARGET_P -> pEquationLocalIds.getOrDefault(equationId, -1) >= 0
                        ? slackPStartIndex + 2 * pEquationLocalIds.get(equationId) : -1;
                case BUS_TARGET_Q -> qEquationLocalIds.getOrDefault(equationId, -1) >= 0
                        ? slackQStartIndex + 2 * qEquationLocalIds.get(equationId) : -1;
                case BUS_TARGET_V -> vEquationLocalIds.getOrDefault(equationId, -1) >= 0
                        ? slackVStartIndex + 2 * vEquationLocalIds.get(equationId) : -1;
                default -> -1;
            };
        }

        @Override
        protected KNEvalGACallback createGradientCallback(JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                                          List<Integer> listNonZerosCtsDense, List<Integer> listNonZerosVarsDense,
                                                          List<Integer> listNonZerosCtsSparse, List<Integer> listNonZerosVarsSparse) {

            return new AbstractRelaxedKnitroProblem.RelaxedCallbackEvalG(jacobianMatrix, listNonZerosCtsDense, listNonZerosVarsDense,
                    listNonZerosCtsSparse, listNonZerosVarsSparse, network,
                    equationSystem, knitroParameters, numberOfPowerFlowVariables);
        }

        /**
         * Callback used by Knitro to evaluate the non-linear parts of the objective and constraint functions.
         */
        public static class RelaxedCallbackEvalFC extends KnitroCallbacks.BaseCallbackEvalFC {

            private final AbstractRelaxedKnitroProblem problemInstance;

            RelaxedCallbackEvalFC(AbstractRelaxedKnitroProblem problemInstance,
                                  List<SingleEquation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                                  List<Integer> nonLinearConstraintIds) {
                super(sortedEquationsToSolve, nonLinearConstraintIds);
                this.problemInstance = problemInstance;
            }

            @Override
            protected double addModificationOfNonLinearConstraints(int equationId, AcEquationType equationType,
                                                                   List<Double> x) {
                int slackIndexBase = problemInstance.getSlackIndexBase(equationType, equationId);
                if (slackIndexBase >= 0) {
                    double sm = x.get(slackIndexBase);        // negative slack
                    double sp = x.get(slackIndexBase + 1);    // positive slack
                    return sp - sm;              // add slack contribution
                }
                return 0;
            }
        }

        /**
         * Callback used by Knitro to evaluate the gradient (Jacobian matrix) of the constraints.
         * Only constraints (no objective) are handled here.
         */
        public static class RelaxedCallbackEvalG extends KnitroCallbacks.BaseCallbackEvalG {

            RelaxedCallbackEvalG(JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                 List<Integer> denseConstraintIndices, List<Integer> denseVariableIndices,
                                 List<Integer> sparseConstraintIndices, List<Integer> sparseVariableIndices,
                                 LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                 KnitroSolverParameters knitroParameters, int numLFVariables) {

                super(jacobianMatrix, denseConstraintIndices, denseVariableIndices, sparseConstraintIndices, sparseVariableIndices,
                        network, equationSystem, knitroParameters, numLFVariables);
            }

            @Override
            protected double computeModifiedJacobianValue(int variableIndex) {
                if (((variableIndex - numLFVariables) & 1) == 0) {
                    // set Jacobian entry to -1.0 if slack variable is Sm
                    return -1.0;
                } else {
                    // set Jacobian entry to +1.0 if slack variable is Sp
                    return 1.0;
                }
            }
        }
    }
}

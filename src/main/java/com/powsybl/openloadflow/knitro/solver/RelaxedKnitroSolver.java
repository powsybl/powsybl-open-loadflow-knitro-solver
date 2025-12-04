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
import com.powsybl.openloadflow.network.util.VoltageInitializer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public class RelaxedKnitroSolver extends AbstractKnitroSolver {

    private static final Logger LOGGER = LoggerFactory.getLogger(RelaxedKnitroSolver.class);

    // Penalty weights in the objective function
    private static final double WEIGHT_P_PENAL = 1.0;
    private static final double WEIGHT_Q_PENAL = 1.0;
    private static final double WEIGHT_V_PENAL = 1.0;

    // Weights of the linear in the objective function
    private static final double WEIGHT_ABSOLUTE_PENAL = 3.0;

    // Total number of variables including slack variables
    private final int numTotalVariables;

    // Number of equations for active power (P), reactive power (Q), and voltage magnitude (V)
    private final int numPEquations;
    private final int numQEquations;
    private final int numVEquations;

    // Starting indices for slack variables in the variable vector
    private final int slackStartIndex;
    private final int slackPStartIndex;
    private final int slackQStartIndex;
    private final int slackVStartIndex;

    // Mappings from global equation indices to local indices by equation type
    private final Map<Integer, Integer> pEquationLocalIds;
    private final Map<Integer, Integer> qEquationLocalIds;
    private final Map<Integer, Integer> vEquationLocalIds;

    public RelaxedKnitroSolver(
            LfNetwork network,
            KnitroSolverParameters knitroParameters,
            EquationSystem<AcVariableType, AcEquationType> equationSystem,
            JacobianMatrix<AcVariableType, AcEquationType> j,
            TargetVector<AcVariableType, AcEquationType> targetVector,
            EquationVector<AcVariableType, AcEquationType> equationVector,
            boolean detailedReport) {

        super(network, knitroParameters, equationSystem, j, targetVector, equationVector, detailedReport);

        // Number of variables in the equations system of open load flow
        int numberOfPowerFlowVariables = equationSystem.getIndex().getSortedVariablesToFind().size();

        List<Equation<AcVariableType, AcEquationType>> sortedEquations = equationSystem.getIndex().getSortedEquationsToSolve();

        // Count number of equations by type
        this.numPEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_P).count();
        this.numQEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_Q).count();
        this.numVEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_V).count();

        int numSlackVariables = 2 * (numPEquations + numQEquations + numVEquations);
        this.numTotalVariables = numberOfPowerFlowVariables + numSlackVariables;

        this.slackStartIndex = numberOfPowerFlowVariables;
        this.slackPStartIndex = slackStartIndex;
        this.slackQStartIndex = slackPStartIndex + 2 * numPEquations;
        this.slackVStartIndex = slackQStartIndex + 2 * numQEquations;

        // Map equations to local indices
        this.pEquationLocalIds = new HashMap<>();
        this.qEquationLocalIds = new HashMap<>();
        this.vEquationLocalIds = new HashMap<>();

        int pCounter = 0;
        int qCounter = 0;
        int vCounter = 0;

        for (int i = 0; i < sortedEquations.size(); i++) {
            AcEquationType type = sortedEquations.get(i).getType();

            switch (type) {
                case BUS_TARGET_P -> pEquationLocalIds.put(i, pCounter++);
                case BUS_TARGET_Q -> qEquationLocalIds.put(i, qCounter++);
                case BUS_TARGET_V -> vEquationLocalIds.put(i, vCounter++);
                default -> {} // no slack to add
            }
        }
    }

    @Override
    public String getName() {
        return "Relaxed Knitro Solver";
    }

    @Override
    protected KNProblem createKnitroProblem(VoltageInitializer voltageInitializer) {
        try {
            return new RelaxedKnitroProblem(network, equationSystem, targetVector, j, voltageInitializer, knitroParameters);
        } catch (KNException e) {
            throw new PowsyblException("Failed to create relaxed Knitro problem", e);
        }
    }

    @Override
    protected KNSolution getSolution(KNSolver solver) {
        return solver.getBestFeasibleIterate();
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
        double penaltyP = computeSlackPenalty(x, slackPStartIndex, numPEquations, WEIGHT_P_PENAL, WEIGHT_ABSOLUTE_PENAL);
        double penaltyQ = computeSlackPenalty(x, slackQStartIndex, numQEquations, WEIGHT_Q_PENAL, WEIGHT_ABSOLUTE_PENAL);
        double penaltyV = computeSlackPenalty(x, slackVStartIndex, numVEquations, WEIGHT_V_PENAL, WEIGHT_ABSOLUTE_PENAL);
        double totalPenalty = penaltyP + penaltyQ + penaltyV;

        LOGGER.info("==== Slack penalty details ====");
        LOGGER.info("Penalty P = {}", penaltyP);
        LOGGER.info("Penalty Q = {}", penaltyQ);
        LOGGER.info("Penalty V = {}", penaltyV);
        LOGGER.info("Total penalty = {}", totalPenalty);
    }

    private void logSlackValues(String type, int startIndex, int count, List<Double> x) {
        final double sbase = 100.0;     // Base power in MVA

        LOGGER.debug("==== Slack diagnostics for {} (p.u. and physical units) ====", type);

        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            double epsilon = sp - sm;

            // Get significant slack values below threshold
            if (Math.abs(epsilon) <= knitroParameters.getSlackThreshold()) {
                continue;
            }

            String name = getSlackVariableBusName(i, type);
            String interpretation;

            switch (type) {
                case "P" -> interpretation = String.format("ΔP = %.4f p.u. (%.1f MW)", epsilon, epsilon * sbase);
                case "Q" -> interpretation = String.format("ΔQ = %.4f p.u. (%.1f MVAr)", epsilon, epsilon * sbase);
                case "V" -> {
                    var bus = network.getBusById(name);
                    if (bus == null) {
                        LOGGER.warn("Bus {} not found while logging V slack.", name);
                        continue;
                    }
                    interpretation = String.format("ΔV = %.4f p.u. (%.1f kV)", epsilon, epsilon * bus.getNominalV());
                }
                default -> interpretation = "Unknown slack type";
            }

            String msg = String.format("Slack %s[ %s ] → Sm = %.4f, Sp = %.4f → %s", type, name, sm, sp, interpretation);
            LOGGER.debug(msg);
        }
    }

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

        LfBus bus = network.getBus(equationSystem.getIndex().getSortedEquationsToSolve().get(varIndex).getElementNum());

        return bus.getId();
    }

    private double computeSlackPenalty(List<Double> x, int startIndex, int count, double weight, double lambda) {
        double penalty = 0.0;
        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            double diff = sp - sm;
            penalty += weight * (diff * diff); // Quadratic terms
            penalty += weight * lambda * (sp + sm); // Linear terms
        }
        return penalty;
    }

    private final class RelaxedKnitroProblem extends AbstractKnitroProblem {

        /**
         * Relaxed Knitro problem definition including:
         * - Initialization of variables (types, bounds, initial state)
         * - Definition of linear and non-linear constraints
         * - Objective function setup
         * - Jacobian matrix setup for Knitro
         */
        private RelaxedKnitroProblem(
                LfNetwork network,
                EquationSystem<AcVariableType, AcEquationType> equationSystem,
                TargetVector<AcVariableType, AcEquationType> targetVector,
                JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                VoltageInitializer voltageInitializer,
                KnitroSolverParameters parameters) throws KNException {

            super(network, equationSystem, targetVector, jacobianMatrix, parameters, numTotalVariables);
            LOGGER.info("Defining {} variables", numTotalVariables);

            // Initialize variables (base class handles LF variables, we customize for slack)
            initializeVariables(voltageInitializer, numTotalVariables);
            LOGGER.info("Voltage initialization strategy: {}", voltageInitializer);

            // =============== Constraint Setup ===============
            setupConstraints();
            List<Equation<AcVariableType, AcEquationType>> activeConstraints = equationSystem.getIndex().getSortedEquationsToSolve();
            List<Integer> nonlinearConstraintIndexes = new ArrayList<>();
            for (int i = 0; i < activeConstraints.size(); i++) {
                Equation<AcVariableType, AcEquationType> eq = activeConstraints.get(i);
                if (!NonLinearExternalSolverUtils.isLinear(eq.getType(), eq.getTerms())) {
                    nonlinearConstraintIndexes.add(i);
                }
            }

            // =============== Objective Function ===============
            List<Integer> quadRows = new ArrayList<>();
            List<Integer> quadCols = new ArrayList<>();
            List<Double> quadCoefs = new ArrayList<>();

            List<Integer> linIndexes = new ArrayList<>();
            List<Double> linCoefs = new ArrayList<>();

            // Slack penalty terms: (Sp - Sm)^2 = Sp^2 + Sm^2 - 2*Sp*Sm + linear terms from the absolute value
            addSlackObjectiveTerms(numPEquations, slackPStartIndex, WEIGHT_P_PENAL, WEIGHT_ABSOLUTE_PENAL, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);
            addSlackObjectiveTerms(numQEquations, slackQStartIndex, WEIGHT_Q_PENAL, WEIGHT_ABSOLUTE_PENAL, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);
            addSlackObjectiveTerms(numVEquations, slackVStartIndex, WEIGHT_V_PENAL, WEIGHT_ABSOLUTE_PENAL, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);

            setObjectiveQuadraticPart(quadRows, quadCols, quadCoefs);
            setObjectiveLinearPart(linIndexes, linCoefs);

            // =============== Callbacks and Jacobian ===============
            setObjEvalCallback(new RelaxedCallbackEvalFC(this, activeConstraints, nonlinearConstraintIndexes));

            List<Integer> jacCstDense = new ArrayList<>();
            List<Integer> jacVarDense = new ArrayList<>();
            List<Integer> jacCstSparse = new ArrayList<>();
            List<Integer> jacVarSparse = new ArrayList<>();

            setJacobianMatrix(activeConstraints, nonlinearConstraintIndexes,
                    jacCstDense, jacVarDense, jacCstSparse, jacVarSparse);

            AbstractMap.SimpleEntry<List<Integer>, List<Integer>> hessNnz = getHessNnzRowsAndCols(nonlinearConstraintIndexes);
            setHessNnzPattern(hessNnz.getKey(), hessNnz.getValue());
        }

        /**
         * Returns the sparsity pattern of the hessian matrix associated with the problem.
         *
         * @param nonlinearConstraintIndexes A list of the indexes of non-linear equations.
         * @return row and column coordinates of non-zero entries in the hessian matrix.
         */
        private AbstractMap.SimpleEntry<List<Integer>, List<Integer>> getHessNnzRowsAndCols(List<Integer> nonlinearConstraintIndexes) {
            record NnzCoordinates(int iRow, int iCol) {
            }

            Set<NnzCoordinates> hessianEntries = new LinkedHashSet<>();

            // Non-linear constraints contributions in the hessian matrix
            for (int index : nonlinearConstraintIndexes) {
                Equation<AcVariableType, AcEquationType> equation = equationSystem.getIndex().getSortedEquationsToSolve().get(index);
                for (EquationTerm<AcVariableType, AcEquationType> term : equation.getTerms()) {
                    for (Variable<AcVariableType> var1 : term.getVariables()) {
                        int i = var1.getRow();
                        for (Variable<AcVariableType> var2 : term.getVariables()) {
                            int j = var2.getRow();
                            if (j >= i) {
                                hessianEntries.add(new NnzCoordinates(i, j));
                            }
                        }
                    }
                }
            }

            // Slacks variables contributions in the objective function
            for (int iSlack = slackStartIndex; iSlack < numTotalVariables; iSlack++) {
                hessianEntries.add(new NnzCoordinates(iSlack, iSlack));
                if (((iSlack - slackStartIndex) & 1) == 0) {
                    hessianEntries.add(new NnzCoordinates(iSlack, iSlack + 1));
                }
            }

            // Sort the entries by row and column indices
            hessianEntries = hessianEntries.stream()
                    .sorted(Comparator.comparingInt(NnzCoordinates::iRow).thenComparingInt(NnzCoordinates::iCol))
                    .collect(Collectors.toCollection(LinkedHashSet::new));

            List<Integer> hessRows = new ArrayList<>();
            List<Integer> hessCols = new ArrayList<>();
            for (NnzCoordinates entry : hessianEntries) {
                hessRows.add(entry.iRow());
                hessCols.add(entry.iCol());
            }

            return new AbstractMap.SimpleEntry<>(hessRows, hessCols);
        }

        /**
         * Adds quadratic and linear terms related to slack variables to the objective function.
         */
        private void addSlackObjectiveTerms(int numEquations, int slackStartIdx, double weight, double lambda,
                                            List<Integer> quadRows, List<Integer> quadCols, List<Double> quadCoefs,
                                            List<Integer> linIndexes, List<Double> linCoefs) {

            for (int i = 0; i < numEquations; i++) {
                int idxSm = slackStartIdx + 2 * i;
                int idxSp = slackStartIdx + 2 * i + 1;

                // Quadratic terms: weight * (sp^2 + sm^2 - 2 * sp * sm)
                quadRows.add(idxSp);
                quadCols.add(idxSp);
                quadCoefs.add(weight);

                quadRows.add(idxSm);
                quadCols.add(idxSm);
                quadCoefs.add(weight);

                quadRows.add(idxSp);
                quadCols.add(idxSm);
                quadCoefs.add(-2 * weight);

                // Linear terms: weight * lambda * (sp + sm)
                linIndexes.add(idxSp);
                linCoefs.add(lambda * weight);

                linIndexes.add(idxSm);
                linCoefs.add(lambda * weight);
            }
        }

        @Override
        protected void customizeVariableBounds(List<Double> lowerBounds, List<Double> upperBounds,
                                              List<Double> initialValues, int numTotalVariables) {
            // Initialize slack variables (≥ 0, initial value = 0)
            for (int i = slackStartIndex; i < numTotalVariables; i++) {
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
                                                      Equation<AcVariableType, AcEquationType> equation,
                                                      List<Integer> variableIndices) {
            AcEquationType equationType = equation.getType();
            int slackStart = switch (equationType) {
                case BUS_TARGET_P -> pEquationLocalIds.getOrDefault(constraintIndex, -1);
                case BUS_TARGET_Q -> qEquationLocalIds.getOrDefault(constraintIndex, -1);
                case BUS_TARGET_V -> vEquationLocalIds.getOrDefault(constraintIndex, -1);
                default -> -1;
            };

            if (slackStart >= 0) {
                int slackBaseIndex = switch (equationType) {
                    case BUS_TARGET_P -> slackPStartIndex;
                    case BUS_TARGET_Q -> slackQStartIndex;
                    case BUS_TARGET_V -> slackVStartIndex;
                    default -> throw new IllegalStateException("Unexpected constraint type: " + equationType);
                };
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
        private int getSlackIndexBase(AcEquationType equationType, int equationId) {
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

            return new RelaxedCallbackEvalG(jacobianMatrix, listNonZerosCtsDense, listNonZerosVarsDense,
                    listNonZerosCtsSparse, listNonZerosVarsSparse, network,
                    equationSystem, knitroParameters, numLFVariables);
        }

        /**
         * Callback used by Knitro to evaluate the non-linear parts of the objective and constraint functions.
         */
        private static final class RelaxedCallbackEvalFC extends KnitroCallbacks.BaseCallbackEvalFC {

            private final RelaxedKnitroProblem problemInstance;

            private RelaxedCallbackEvalFC(RelaxedKnitroProblem problemInstance,
                                          List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
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
        private static final class RelaxedCallbackEvalG extends KnitroCallbacks.BaseCallbackEvalG {

            private RelaxedCallbackEvalG(JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                         List<Integer> denseConstraintIndices, List<Integer> denseVariableIndices,
                                         List<Integer> sparseConstraintIndices, List<Integer> sparseVariableIndices,
                                         LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                         KnitroSolverParameters knitroParameters, int numLFVariables) {

                super(jacobianMatrix, denseConstraintIndices, denseVariableIndices, sparseConstraintIndices, sparseVariableIndices,
                        network, equationSystem, knitroParameters, numLFVariables);
            }

            @Override
            protected double computeModifiedJacobianValue(int variableIndex, int constraintIndex) {
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

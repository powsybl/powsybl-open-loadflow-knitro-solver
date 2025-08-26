/**
 * Copyright (c) 2025, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */

package com.powsybl.openloadflow.knitro.solver;

import com.artelys.knitro.api.*;
import com.artelys.knitro.api.callbacks.KNEvalFCCallback;
import com.artelys.knitro.api.callbacks.KNEvalGACallback;
import com.powsybl.commons.PowsyblException;
import com.powsybl.commons.report.ReportNode;
import com.powsybl.math.matrix.SparseMatrix;
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.ac.solver.AbstractAcSolver;
import com.powsybl.openloadflow.ac.solver.AcSolverResult;
import com.powsybl.openloadflow.ac.solver.AcSolverStatus;
import com.powsybl.openloadflow.ac.solver.AcSolverUtil;
import com.powsybl.openloadflow.equations.*;
import com.powsybl.openloadflow.network.ElementType;
import com.powsybl.openloadflow.network.LfBus;
import com.powsybl.openloadflow.network.LfNetwork;
import com.powsybl.openloadflow.network.util.VoltageInitializer;
import org.apache.commons.lang3.Range;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import static com.google.common.primitives.Doubles.toArray;
import static com.powsybl.openloadflow.ac.equations.AcEquationType.*;


/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Yoann Anezin {@literal <yoann.anezin at artelys.com>}
 */
public class KnitroSolverReacLim extends AbstractAcSolver {

    private static final Logger LOGGER = LoggerFactory.getLogger(KnitroSolverReacLim.class);

    // Penalty weights in the objective function
    private final double wK = 1.0;
    private final double wP = 1.0;
    private final double wQ = 1.0;
    private final double wV = 10.0;

    // Lambda
    private final double lambda = 3.0;

    // Number of Load Flows (LF) variables in the system
    private final int numLFVariables;

    // Number of variables including slack variables
    private final int numLFandSlackVariables;
    // Total number of variables including complementarity constraints' ones
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
    private final int compVarIndex;

    // Mappings from global equation indices to local indices by equation type
    private final Map<Integer, Integer> pEquationLocalIds;
    private final Map<Integer, Integer> qEquationLocalIds;
    private final Map<Integer, Integer> vEquationLocalIds;
    private static final Map<Integer, Equation<AcVariableType, AcEquationType>> indEqUnactiveQ = new HashMap<>();

    protected KnitroSolverParameters knitroParameters;

    public KnitroSolverReacLim(
            LfNetwork network,
            KnitroSolverParameters knitroParameters,
            EquationSystem<AcVariableType, AcEquationType> equationSystem,
            JacobianMatrix<AcVariableType, AcEquationType> jacobian,
            TargetVector<AcVariableType, AcEquationType> targetVector,
            EquationVector<AcVariableType, AcEquationType> equationVector,
            boolean detailedReport) {

        super(network, equationSystem, jacobian, targetVector, equationVector, detailedReport);
        this.knitroParameters = knitroParameters;

        this.numLFVariables = equationSystem.getIndex().getSortedVariablesToFind().size();
        List<Equation<AcVariableType, AcEquationType>> sortedEquations = equationSystem.getIndex().getSortedEquationsToSolve();

        // Count the number of slack buses by their type
        List<Integer> slackBuses = network.getSlackBuses().stream()
                .map(LfBus::getNum)
                .toList();

        Map<AcEquationType, Long> slackBusCounts = slackBuses.stream()
                .map(slackBusID -> equationSystem.getIndex().getSortedEquationsToSolve().stream()
                        .filter(e -> e.getElementNum() == slackBusID)
                        .findAny()
                        .orElseThrow())
                .collect(Collectors.groupingBy(Equation::getType, Collectors.counting()));

        long numSlackBusesPQ = slackBusCounts.getOrDefault(AcEquationType.BUS_TARGET_Q, 0L);
        long numSlackBusesPV = slackBusCounts.getOrDefault(AcEquationType.BUS_TARGET_V, 0L);

        // Count number of classic LF equations by type
        this.numPEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_P).count();
        this.numQEquations = (int) (sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_Q).count());
        this.numVEquations = (int) (sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_V).count());

        int numSlackVariables = 2 * (numPEquations + numQEquations + numVEquations);
        int complConstVariables = 5 * numVEquations;
        this.numLFandSlackVariables = numLFVariables + numSlackVariables;
        this.numTotalVariables = numLFandSlackVariables + complConstVariables;

        this.slackStartIndex = numLFVariables;
        this.slackPStartIndex = slackStartIndex;
        this.slackQStartIndex = slackPStartIndex + 2 * numPEquations;
        this.slackVStartIndex = slackQStartIndex + 2 * numQEquations;
        this.compVarIndex = slackVStartIndex + 2 * numVEquations;

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
            }
        }
    }

    /**
     * Returns the name of the solver.
     */
    @Override
    public String getName() {
        return "Knitro Reactive Limits Solver";
    }

    /**
     * Logs the Knitro status and its corresponding AcSolverStatus.
     *
     * @param status the KnitroStatus to log
     */
    private void logKnitroStatus(KnitroStatus status) {
        LOGGER.info("Knitro Status: {}", status);
    }

    /**
     * Builds the row and column index lists corresponding to the dense Jacobian structure,
     * assuming each non-linear constraint is derived with respect to every variable.
     *
     * @param numVars               Total number of variables.
     * @param listNonLinearConsts   List of non-linear constraint indices.
     * @param listNonZerosCtsDense  Output list to receive constraint indices for non-zero Jacobian entries.
     * @param listNonZerosVarsDense Output list to receive variable indices for non-zero Jacobian entries.
     */
    public void buildDenseJacobianMatrix(
            int numVars,
            List<Integer> listNonLinearConsts,
            List<Integer> listNonZerosCtsDense,
            List<Integer> listNonZerosVarsDense) {

        // Each non-linear constraint will have a partial derivative with respect to every variable
        for (Integer constraintId : listNonLinearConsts) {
            for (int varIndex = 0; varIndex < numVars; varIndex++) {
                listNonZerosCtsDense.add(constraintId);
            }
        }

        // We repeat the full list of variables for each non-linear constraint
        List<Integer> variableIndices = IntStream.range(0, numVars).boxed().toList();
        for (int i = 0; i < listNonLinearConsts.size(); i++) {
            listNonZerosVarsDense.addAll(variableIndices);
        }
    }

    /**
     * Builds the sparse Jacobian matrix by identifying non-zero entries
     * for each non-linear constraint, including contributions from slack variables.
     *
     * @param sortedEquationsToSolve Ordered list of equations to solve.
     * @param nonLinearConstraintIds Indices of non-linear constraints within the sorted equation list.
     * @param jacobianRowIndices     Output: row indices (constraints) of non-zero Jacobian entries.
     * @param jacobianColumnIndices  Output: column indices (variables) of non-zero Jacobian entries.
     */
    public void buildSparseJacobianMatrix(
            List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
            List<Integer> nonLinearConstraintIds,
            List<Integer> jacobianRowIndices,
            List<Integer> jacobianColumnIndices) {

        int nbreVEq = sortedEquationsToSolve.stream().filter(e -> e.getType()==BUS_TARGET_V).toList().size()/2;
        int nbreLFEq = sortedEquationsToSolve.size() - 3*nbreVEq;
        int nbreBlow = 0;
        int nbreBup = 0;

        for (Integer constraintIndex : nonLinearConstraintIds) {
            Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(constraintIndex);
            AcEquationType equationType = equation.getType();

            // Collect all variable indices involved in the current equation
            Set<Integer> involvedVariables = equation.getTerms().stream()
                    .flatMap(term -> term.getVariables().stream())
                    .map(Variable::getRow)
                    .collect(Collectors.toCollection(TreeSet::new));

            // Add slack variables if the constraint type has them
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
                involvedVariables.add(slackBaseIndex + 2 * slackStart);     // Sm
                involvedVariables.add(slackBaseIndex + 2 * slackStart + 1); // Sp
            }

            // Add complementarity constraints' variables if the constraint type has them
            int compVarStart;
            // Case of inactive Q equations
            if (equationType == BUS_TARGET_Q && !equation.isActive()) {
                compVarStart = vEquationLocalIds.getOrDefault(equationSystem.getIndex().getSortedEquationsToSolve()
                        .indexOf(equationSystem.getEquations(ElementType.BUS, equation.getElementNum()).stream()
                                .filter(e -> e.getType() == BUS_TARGET_V).toList()
                                .get(0)), -1);
                if (constraintIndex < nbreLFEq + 2 * nbreVEq) {
                    involvedVariables.add(compVarIndex + 5 * compVarStart + 2); // b_low
                    nbreBlow += 1;
                } else {
                    involvedVariables.add(compVarIndex + 5 * compVarStart + 3); // b_up
                    nbreBup += 1;
                }
            }

            // Add one entry for each non-zero (constraintIndex, variableIndex)
            jacobianColumnIndices.addAll(involvedVariables);
            jacobianRowIndices.addAll(Collections.nCopies(involvedVariables.size(), constraintIndex));
        }
    }

    /**
     * Sets Knitro solver parameters based on the provided KnitroSolverParameters object.
     *
     * @param solver The Knitro solver instance to configure.
     * @throws KNException if Knitro fails to accept a parameter.
     */
    private void setSolverParameters(KNSolver solver) throws KNException {
        LOGGER.info("Configuring Knitro solver parameters...");

        solver.setParam(KNConstants.KN_PARAM_GRADOPT, knitroParameters.getGradientComputationMode());
        solver.setParam(KNConstants.KN_PARAM_FEASTOL, knitroParameters.getConvEps());
        solver.setParam(KNConstants.KN_PARAM_MAXIT, knitroParameters.getMaxIterations());
        solver.setParam(KNConstants.KN_PARAM_HESSOPT, knitroParameters.getHessianComputationMode());

        LOGGER.info("Knitro parameters set: GRADOPT={}, HESSOPT={}, FEASTOL={}, MAXIT={}",
                knitroParameters.getGradientComputationMode(),
                knitroParameters.getHessianComputationMode(),
                knitroParameters.getConvEps(),
                knitroParameters.getMaxIterations());
    }

    @Override
    public AcSolverResult run(VoltageInitializer voltageInitializer, ReportNode reportNode) {
        int nbIterations;
        AcSolverStatus solverStatus;
        ResilientReacLimKnitroProblem problemInstance;

        try {
            problemInstance = new ResilientReacLimKnitroProblem(network, equationSystem, targetVector, j, voltageInitializer);
        } catch (KNException e) {
            throw new PowsyblException("Exception while building Knitro problem", e);
        }

        try  {
            KNSolver solver = new KNSolver(problemInstance);
            solver.initProblem();
            setSolverParameters(solver);
            solver.solve();

            KNSolution solution = solver.getBestFeasibleIterate();
            List<Double> constraintValues = solver.getConstraintValues();
            List<Double> x = solution.getX();
            List<Double> lambda2 = solution.getLambda();

            solverStatus = KnitroStatus.fromStatusCode(solution.getStatus()).toAcSolverStatus();
            logKnitroStatus(KnitroStatus.fromStatusCode(solution.getStatus()));
            nbIterations = solver.getNumberIters();

            LOGGER.info("==== Solution Summary ====");
            LOGGER.info("Objective value            = {}", solution.getObjValue());
            LOGGER.info("Feasibility violation      = {}", solution.getFeasError());
            LOGGER.info("Optimality violation       = {}", solver.getAbsOptError());

            // Log primal solution
            LOGGER.debug("==== Optimal variables ====");
            for (int i = 0; i < x.size(); i++) {
                LOGGER.debug(" x[{}] = {}", i, x.get(i));
            }

            LOGGER.debug("==== Constraint values ====");
            for (int i = 0; i < problemInstance.getNumCons(); i++) {
                LOGGER.debug(" c[{}] = {} (λ = {})", i, constraintValues.get(i), lambda2.get(i));
            }

            LOGGER.debug("==== Constraint violations ====");
            for (int i = 0; i < problemInstance.getNumCons(); i++) {
                LOGGER.debug(" violation[{}] = {}", i, solver.getConViol(i));
            }

            // ========== Slack Logging ==========
            logSlackValues("P", slackPStartIndex, numPEquations, x);
            logSlackValues("Q", slackQStartIndex, numQEquations, x);
            logSlackValues("V", slackVStartIndex, numVEquations, x);

            // ========== Penalty Computation ==========
            double penaltyP = computeSlackPenalty(x, slackPStartIndex, numPEquations, wK * wP, lambda);
            double penaltyQ = computeSlackPenalty(x, slackQStartIndex, numQEquations, wK * wQ, lambda);
            double penaltyV = computeSlackPenalty(x, slackVStartIndex, numVEquations, wV, lambda);
            double totalPenalty = penaltyP + penaltyQ + penaltyV;

            LOGGER.info("==== Slack penalty details ====");
            LOGGER.info("Penalty P = {}", penaltyP);
            LOGGER.info("Penalty Q = {}", penaltyQ);
            LOGGER.info("Penalty V = {}", penaltyV);
            LOGGER.info("Total penalty = {}", totalPenalty);

            LOGGER.info("=== Switches Done===");
            checkSwitchesDone(x, compVarIndex, numVEquations);

            // ========== Network Update ==========
            if (solverStatus == AcSolverStatus.CONVERGED || knitroParameters.isAlwaysUpdateNetwork()) {
                equationSystem.getStateVector().set(toArray(x));
                for (Equation<AcVariableType, AcEquationType> equation : equationSystem.getEquations()) {
                    for (EquationTerm<AcVariableType, AcEquationType> term : equation.getTerms()) {
                        term.setStateVector(equationSystem.getStateVector());
                    }
                }
                AcSolverUtil.updateNetwork(network, equationSystem);
            }

        } catch (KNException e) {
            throw new PowsyblException("Exception while solving with Knitro", e);
        }

        double slackBusMismatch = network.getSlackBuses().stream()
                .mapToDouble(LfBus::getMismatchP)
                .sum();

        return new AcSolverResult(solverStatus, nbIterations, slackBusMismatch);
    }

    private void logSlackValues(String type, int startIndex, int count, List<Double> x) {
        final double threshold = 1e-3;  // Threshold for significant slack values
        final double sbase = 100.0;     // Base power in MVA

        LOGGER.info("==== Slack diagnostics for {} (p.u. and physical units) ====", type);

        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            double epsilon = sp - sm;
            double absEpsilon = Math.abs(epsilon);
            String name = getSlackVariableBusName(i, type);

            if (absEpsilon > threshold) {
                String interpretation;
                switch (type) {
                    case "P" -> interpretation = String.format("ΔP = %.4f p.u. (%.1f MW)", epsilon, epsilon * sbase);
                    case "Q" -> interpretation = String.format("ΔQ = %.4f p.u. (%.1f MVAr)", epsilon, epsilon * sbase);
                    case "V" -> interpretation = String.format("ΔV = %.4f p.u.", epsilon);
                    default -> interpretation = String.format("Δ = %.4f p.u.", epsilon);
                }

                String msg = String.format("Slack %s[ %s ] → Sm = %.4f, Sp = %.4f → %s", type, name, sm, sp, interpretation);
                LOGGER.info(msg);
            }
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
            throw new RuntimeException("Variable index associated with slack variable " + type + "was not found");
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

    /**
     * Inform all switches PV -> PQ done in the solution found
     * @param x             current network's estate
     * @param startIndex    first index of complementarity constraints variables in x
     * @param count         number of b_low / b_up different variables
     */
    private void checkSwitchesDone(List<Double> x, int startIndex, int count) {
        for (int i = 0; i < count; i++) {
            double blow = x.get(startIndex + 5 * i + 2);
            double bup = x.get(startIndex + 5 * i + 3);
            if (Math.abs(blow) < 1E-3) {
                LOGGER.info("Switch PV -> PQ on bus {}, Q set at Qmin", equationSystem.getIndex()
                        .getSortedEquationsToSolve().stream().filter(e ->
                                e.getType() == BUS_TARGET_V).toList().get(i).getElement(network).get().getId());
            } else if (Math.abs(bup) < 1E-3) {
                LOGGER.info("Switch PV -> PQ on bus {}, Q set at Qmax", equationSystem.getIndex()
                        .getSortedEquationsToSolve().stream().filter(e ->
                                e.getType() == BUS_TARGET_V).toList().get(i).getElement(network).get().getId());
            }
        }
    }

    private AbstractMap.SimpleEntry<List<Integer>, List<Integer>> getHessNnzRowsAndCols() {
        return new AbstractMap.SimpleEntry<>(
                IntStream.range(slackStartIndex, numLFandSlackVariables).boxed().toList(),
                IntStream.range(slackStartIndex, numLFandSlackVariables).boxed().toList()
        );
    }

    /**
     * Enum representing specific status codes returned by the Knitro solver,
     * grouped either individually or by ranges, and mapped to corresponding {@link AcSolverStatus} values.
     * This mapping allows a more fine-grained interpretation of solver termination reasons,
     * distinguishing between convergence, infeasibility, modeling errors, evaluation issues, etc...
     */
    public enum KnitroStatus {

        /**
         * Successful convergence to a local optimum.
         */
        CONVERGED_TO_LOCAL_OPTIMUM(0, 0, AcSolverStatus.CONVERGED),

        /**
         * Converged to a feasible but not necessarily optimal solution.
         */
        CONVERGED_TO_FEASIBLE_APPROXIMATE_SOLUTION(-199, -100, AcSolverStatus.CONVERGED),

        /**
         * Solver terminated at an infeasible point.
         */
        TERMINATED_AT_INFEASIBLE_POINT(-299, -200, AcSolverStatus.SOLVER_FAILED),

        /**
         * The problem was detected as unbounded.
         */
        PROBLEM_UNBOUNDED(-399, -300, AcSolverStatus.SOLVER_FAILED),

        /**
         * Optimization stopped due to reaching iteration or time limits.
         */
        TERMINATED_DUE_TO_PRE_DEFINED_LIMIT(-499, -400, AcSolverStatus.MAX_ITERATION_REACHED),

        /**
         * Failure in a user-defined callback function.
         */
        CALLBACK_ERROR(-500, -500, AcSolverStatus.SOLVER_FAILED),

        /**
         * Internal LP solver failure in active-set method.
         */
        LP_SOLVER_ERROR(-501, -501, AcSolverStatus.SOLVER_FAILED),

        /**
         * Evaluation failure (e.g., division by zero or invalid sqrt).
         */
        EVALUATION_ERROR(-502, -502, AcSolverStatus.SOLVER_FAILED),

        /**
         * Insufficient memory to solve the problem.
         */
        OUT_OF_MEMORY(-503, -503, AcSolverStatus.SOLVER_FAILED),

        /**
         * Solver was stopped manually by the user.
         */
        USER_TERMINATION(-504, -504, AcSolverStatus.SOLVER_FAILED),

        /**
         * File open error when trying to read input.
         */
        INPUT_FILE_ERROR(-505, -505, AcSolverStatus.SOLVER_FAILED),

        /**
         * Modeling error: invalid variable/constraint setup.
         */
        MODEL_DEFINITION_ERROR(-530, -506, AcSolverStatus.SOLVER_FAILED),

        /**
         * Internal Knitro error – contact support.
         */
        INTERNAL_ERROR(-600, -600, AcSolverStatus.SOLVER_FAILED),

        /**
         * Fallback for unknown status codes.
         */
        UNKNOWN_STATUS(Integer.MIN_VALUE, Integer.MAX_VALUE, AcSolverStatus.SOLVER_FAILED);

        private final Range<Integer> codeRange;
        private final AcSolverStatus mappedStatus;

        /**
         * Constructs a KnitroStatus with a range of status codes and the associated AcSolverStatus.
         *
         * @param min          The minimum status code value (inclusive).
         * @param max          The maximum status code value (inclusive).
         * @param mappedStatus The corresponding AcSolverStatus.
         */
        KnitroStatus(int min, int max, AcSolverStatus mappedStatus) {
            this.codeRange = Range.of(min, max);
            this.mappedStatus = mappedStatus;
        }

        /**
         * Returns the KnitroStatus corresponding to the given status code.
         *
         * @param statusCode the status code returned by Knitro
         * @return the matching KnitroStatus enum constant, or UNKNOWN_STATUS if unknown
         */
        public static KnitroStatus fromStatusCode(int statusCode) {
            return Arrays.stream(KnitroStatus.values())
                    .filter(status -> status.codeRange.contains(statusCode))
                    .findFirst()
                    .orElse(UNKNOWN_STATUS);
        }

        /**
         * Returns the {@link AcSolverStatus} associated with this KnitroStatus.
         *
         * @return the corresponding AcSolverStatus
         */
        public AcSolverStatus toAcSolverStatus() {
            return mappedStatus;
        }
    }

    private final class ResilientReacLimKnitroProblem extends KNProblem {

        /**
         * Knitro problem definition including:
         * - Initialization of variables (types, bounds, initial state)
         * - Definition of linear and non-linear constraints
         * - Objective function setup
         * - Jacobian matrix setup for Knitro
         */
        private ResilientReacLimKnitroProblem(
                LfNetwork network,
                EquationSystem<AcVariableType, AcEquationType> equationSystem,
                TargetVector<AcVariableType, AcEquationType> targetVector,
                JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                VoltageInitializer voltageInitializer) throws KNException {
            // =============== Variable Initialization ===============
            super(numTotalVariables, equationSystem.getIndex().getSortedEquationsToSolve().size() + 3*((int)
                    equationSystem.getIndex().getSortedEquationsToSolve().stream().filter(
                            e -> e.getType() == AcEquationType.BUS_TARGET_V).count()));
            LOGGER.info("Defining {} variables", numTotalVariables);

            // Variable types (all continuous), bounds, and initial values
            List<Integer> variableTypes = new ArrayList<>(Collections.nCopies(numTotalVariables, KNConstants.KN_VARTYPE_CONTINUOUS));
            List<Double> lowerBounds = new ArrayList<>(Collections.nCopies(numTotalVariables, -KNConstants.KN_INFINITY));
            List<Double> upperBounds = new ArrayList<>(Collections.nCopies(numTotalVariables, KNConstants.KN_INFINITY));
            List<Double> scalingFactors = new ArrayList<>(Collections.nCopies(numTotalVariables, 1.0));
            List<Double> scalingCenters = new ArrayList<>(Collections.nCopies(numTotalVariables, 0.0));
            List<Double> initialValues = new ArrayList<>(Collections.nCopies(numTotalVariables, 0.0));

            setVarTypes(variableTypes);

            // Compute initial voltage state using the given initializer
            AcSolverUtil.initStateVector(network, equationSystem, voltageInitializer);
            for (int i = 0; i < numLFVariables; i++) {
                initialValues.set(i, equationSystem.getStateVector().get(i));
            }

            // Initialize slack variables (≥ 0, initial value = 0), scale P and Q slacks
            for (int i = slackStartIndex; i < numLFandSlackVariables; i++) {
                lowerBounds.set(i, 0.0);
//                upperBounds.set(i, 0.0);
//                if (i >= slackQStartIndex) {
//                    upperBounds.set(i, 0.0);
//                }

                if (i < slackVStartIndex) {
                    scalingFactors.set(i, 1e-2);
                }
            }

            // Set bounds for voltage variables based on Knitro parameters
            for (int i = 0; i < numLFVariables; i++) {
                if (equationSystem.getIndex().getSortedVariablesToFind().get(i).getType() == AcVariableType.BUS_V) {
                    lowerBounds.set(i, knitroParameters.getLowerVoltageBound());
                    upperBounds.set(i, knitroParameters.getUpperVoltageBound());
                }
            }

            // Set bounds for complementarity variables (≥ 0)
            for (int i = 0; i < numVEquations; i++) {
                lowerBounds.set(compVarIndex + 5*i, 0.0);
                lowerBounds.set(compVarIndex + 5*i + 1, 0.0);
                lowerBounds.set(compVarIndex + 5*i + 2, 0.0);
                lowerBounds.set(compVarIndex + 5*i + 3, 0.0);
                lowerBounds.set(compVarIndex + 5*i + 4, 0.0);
//                upperBounds.set(compVarIndex + 5*i + 4, 0.0);
            }

            LOGGER.info("Voltage initialization strategy: {}", voltageInitializer);

            int N = numTotalVariables;
            ArrayList<Integer> list = new ArrayList<>(N);
            for (int i = 0; i < N; i++) {
                list.add(i);
            }

            // Set bounds and initial state
            setVarLoBnds(lowerBounds);
            setVarUpBnds(upperBounds);
            setVarScaleFactors(new KNSparseVector<>(list, scalingFactors));
            setVarScaleCenters(new KNSparseVector<>(list, scalingCenters));
            setXInitial(initialValues);
            LOGGER.info("Variables initialization complete!");

            // =============== Constraint Setup ===============
            List<Equation<AcVariableType, AcEquationType>> activeConstraints = equationSystem.getIndex().getSortedEquationsToSolve();
            int maxColumn = activeConstraints.stream().map(Equation::getColumn).max(Integer::compare).get();
            // Build sorted list of buses' indexes for buses with an equation setting V in order to pick non-acitvated Q equations
            List<Integer> listBusesWithVEq = activeConstraints.stream().filter(
                            e->e.getType() == AcEquationType.BUS_TARGET_V)
                    .map(e -> e.getTerms().get(0).getElementNum())
                    .sorted().toList();

            // Picking non-activated Q equations
            List<Equation<AcVariableType, AcEquationType>> equationQBusV = new ArrayList<>();
            for (int elementNum : listBusesWithVEq) {
                equationQBusV.addAll(equationSystem.getEquations(ElementType.BUS,
                        elementNum).stream().filter(e ->
                        e.getType() == BUS_TARGET_Q).toList());
            }
            List<Equation<AcVariableType, AcEquationType>> equationsQBusV = Stream.concat(equationQBusV.stream(),
                    equationQBusV.stream()).toList(); //Duplication to get b_low and b_up eq

            // Linear and nonlinear constraints (the latter are deferred to callback)
            NonLinearExternalSolverUtils solverUtils = new NonLinearExternalSolverUtils();

            List<Integer> nonlinearConstraintIndexes = new ArrayList<>(); // contains the indexes of all non-linear constraints
            List<Equation<AcVariableType, AcEquationType>> completeEquationsToSolve = new ArrayList<>(activeConstraints);
            List<Double> wholeTargetVector = new ArrayList<>(Arrays.stream(targetVector.getArray()).boxed().toList());
            for (int equationId = 0; equationId < activeConstraints.size(); equationId++) {
                addActivatedConstraints(equationId, activeConstraints, solverUtils, nonlinearConstraintIndexes,
                        completeEquationsToSolve, targetVector,wholeTargetVector); // Add Linear constraints, index nonLinear ones and get target values
            }
            int totalActiveConstraints = completeEquationsToSolve.size();
            completeEquationsToSolve.addAll(equationsQBusV);

            // Set Target Q on the unactive equations added
            for (int equationId = totalActiveConstraints; equationId < completeEquationsToSolve.size(); equationId++) {
                Equation<AcVariableType, AcEquationType> equation = completeEquationsToSolve.get(equationId);
                LfBus b = network.getBus(equation.getElementNum());
                if (equationId-totalActiveConstraints < equationQBusV.size()) {
                    wholeTargetVector.add(b.getMinQ() - b.getLoadTargetQ());
                } else {
                    wholeTargetVector.add(b.getMaxQ() - b.getLoadTargetQ());
                }
                nonlinearConstraintIndexes.add(equationId);
                indEqUnactiveQ.put(equationId,equation);
            }

            int numConstraints = completeEquationsToSolve.size();
            LOGGER.info("Defined {} constraints", numConstraints);
            setMainCallbackCstIndexes(nonlinearConstraintIndexes);
            setConEqBnds(wholeTargetVector);

            // =============== Declaration of Complementarity Constraints ===============
            List<Integer> listTypeVar = new ArrayList<>(Collections.nCopies(2*numVEquations, KNConstants.KN_CCTYPE_VARVAR));
            List<Integer> bVarList = new ArrayList<>(); // b_up, b_low
            List<Integer> vInfSuppList = new ArrayList<>(); // V_inf, V_sup

            for (int i = 0; i < numVEquations; i++) {
                vInfSuppList.add(compVarIndex + 5*i); //Vinf
                vInfSuppList.add(compVarIndex + 5*i + 1); //Vsup
                bVarList.add(compVarIndex + 5*i + 3); // bup
                bVarList.add(compVarIndex + 5*i + 2); // blow
            }

            setCompConstraintsTypes(listTypeVar);
            setCompConstraintsParts(bVarList, vInfSuppList); // b_low compl with V_sup  and  b_up compl with V_inf

            // =============== Objective Function ===============
            List<Integer> quadRows = new ArrayList<>();
            List<Integer> quadCols = new ArrayList<>();
            List<Double> quadCoefs = new ArrayList<>();

            List<Integer> linIndexes = new ArrayList<>();
            List<Double> linCoefs = new ArrayList<>();

            // Slack penalty terms: (Sp - Sm)^2 = Sp^2 + Sm^2 - 2*Sp*Sm + linear terms from the absolute value
            addSlackObjectiveTerms(numPEquations, slackPStartIndex, wK * wP, lambda, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);
            addSlackObjectiveTerms(numQEquations, slackQStartIndex, wK * wQ, lambda, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);
            addSlackObjectiveTerms(numVEquations, slackVStartIndex, wV, lambda, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);

            setObjectiveQuadraticPart(quadRows, quadCols, quadCoefs);
            setObjectiveLinearPart(linIndexes, linCoefs);

            // =============== Callbacks and Jacobian ===============
            setObjEvalCallback(new CallbackEvalFC(this, completeEquationsToSolve, nonlinearConstraintIndexes));

            List<Integer> jacCstDense = new ArrayList<>();
            List<Integer> jacVarDense = new ArrayList<>();
            List<Integer> jacCstSparse = new ArrayList<>();
            List<Integer> jacVarSparse = new ArrayList<>();

            setJacobianMatrix(
                    network, jacobianMatrix, completeEquationsToSolve, nonlinearConstraintIndexes,
                    jacCstDense, jacVarDense, jacCstSparse, jacVarSparse
            );

            AbstractMap.SimpleEntry<List<Integer>, List<Integer>> hessNnz = getHessNnzRowsAndCols();
            setHessNnzPattern(hessNnz.getKey(), hessNnz.getValue());
        }

        /**
         * Adds quadratic and linear terms related to slack variables to the objective function.
         */
        private void addSlackObjectiveTerms(
                int numEquations,
                int slackStartIdx,
                double weight,
                double lambda,
                List<Integer> quadRows,
                List<Integer> quadCols,
                List<Double> quadCoefs,
                List<Integer> linIndexes,
                List<Double> linCoefs) {

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

        /**
         * Adds a single constraint to the Knitro problem.
         * Linear constraints are directly encoded; non-linear ones are delegated to the callback.
         *
         * @param equationId             Index of the equation in the list.
         * @param equationsToSolve       Base list of all equations to solve.
         * @param solverUtils            Utilities to extract linear constraint components.
         * @param nonLinearConstraintIds Output list of non-linear constraint indices.
         */
        private void addActivatedConstraints(
                int equationId,
                List<Equation<AcVariableType, AcEquationType>> equationsToSolve,
                NonLinearExternalSolverUtils solverUtils,
                List<Integer> nonLinearConstraintIds,
                List<Equation<AcVariableType, AcEquationType>> completeEquationsToSolve,
                TargetVector<AcVariableType, AcEquationType> targetVector,
                List<Double> wholeTargetVector) {

            Equation<AcVariableType, AcEquationType> equation = equationsToSolve.get(equationId);
            AcEquationType equationType = equation.getType();
            List<EquationTerm<AcVariableType, AcEquationType>> terms = equation.getTerms();

            if (equationType == BUS_TARGET_V) {
                if (NonLinearExternalSolverUtils.isLinear(equationType, terms)) {
                    try {
                        int compVarBaseIndex = compVarIndex + 5 * vEquationLocalIds.get(equationId);
                        // Extract linear constraint components
                        var linearConstraint = solverUtils.getLinearConstraint(equationType, terms);
                        List<Integer> varVInfIndices = new ArrayList<>(linearConstraint.listIdVar());
                        List<Double> coefficientsVInf = new ArrayList<>(linearConstraint.listCoef());

                        // To add complementarity conditions, Knitro requires that they be written as two variables
                        // that complement each other. That is why we are introducing new variables that will play this role.
                        // We call them V_inf, V_sup, b_low and b_up. The two lasts appear in non-linear constraints
                        // Equations on V are duplicated, we add V_inf to one and V_sup to the other.
                        // We are also adding a variable V_aux that allows us to perform the PV -> PQ switch.

                        // ---- V_inf Equation ----
                        varVInfIndices.add(compVarBaseIndex); // V_inf
                        varVInfIndices.add(compVarBaseIndex + 4); // V_aux
                        coefficientsVInf.add(1.0);
                        coefficientsVInf.add(-1.0);

                        // Add slack variables if applicable
                        int slackBase = getSlackIndexBase(equationType, equationId);
                        if (slackBase >= 0) {
                            varVInfIndices.add(slackBase);       // Sm
                            varVInfIndices.add(slackBase + 1);   // Sp
                            coefficientsVInf.add(-1.0);
                            coefficientsVInf.add(1.0);
                        }

                        for (int i = 0; i < varVInfIndices.size(); i++) {
                            this.addConstraintLinearPart(equationId, varVInfIndices.get(i), coefficientsVInf.get(i));
                        }

                        LOGGER.trace("Added linear constraint #{} of type {}", equationId, equationType);

                        // ---- V_sup Equation ----

                        List<Integer> varVSupIndices = new ArrayList<>(linearConstraint.listIdVar());
                        List<Double> coefficientsVSup = new ArrayList<>(linearConstraint.listCoef());

                        // Add complementarity constraints' variables
                        varVSupIndices.add(compVarBaseIndex + 1); // V_sup
                        varVSupIndices.add(compVarBaseIndex + 4); // V_aux
                        coefficientsVSup.add(-1.0);
                        coefficientsVSup.add(1.0);

                        // Add slack variables if applicable
                        if (slackBase >= 0) {
                            varVSupIndices.add(slackBase);       // Sm
                            varVSupIndices.add(slackBase + 1);   // Sp
                            coefficientsVSup.add(-1.0);
                            coefficientsVSup.add(1.0);
                        }

                        for (int i = 0; i < varVSupIndices.size(); i++) {
                            this.addConstraintLinearPart(equationsToSolve.size() + vEquationLocalIds
                                    .get(equationId), varVSupIndices.get(i), coefficientsVSup.get(i));
                        }


                    } catch (UnsupportedOperationException e) {
                        throw new PowsyblException("Failed to process linear constraint for equation #" + equationId, e);
                    }

                } else {
                    nonLinearConstraintIds.add(equationId);
                    nonLinearConstraintIds.add(equationsToSolve.size() + vEquationLocalIds.get(equationId));
                }

                // Add the duplicated equation (ie V_sup eq) to the list of eq to solve and its target
                Equation<AcVariableType, AcEquationType> VEq = equationsToSolve.get(equationId);
                completeEquationsToSolve.add(VEq);

                wholeTargetVector.add(Arrays.stream(targetVector.getArray()).boxed().toList().get(equationId));

            } else {
                if (NonLinearExternalSolverUtils.isLinear(equationType, terms)) {
                    try {
                        // Extract linear constraint components
                        var linearConstraint = solverUtils.getLinearConstraint(equationType, terms);
                        List<Integer> varIndices = new ArrayList<>(linearConstraint.listIdVar());
                        List<Double> coefficients = new ArrayList<>(linearConstraint.listCoef());

                        // Add slack variables if applicable
                        int slackBase = getSlackIndexBase(equationType, equationId);
                        if (slackBase >= 0) {
                            varIndices.add(slackBase);       // Sm
                            varIndices.add(slackBase + 1);   // Sp
                            coefficients.add(-1.0);
                            coefficients.add(+1.0);
                        }

                        for (int i = 0; i < varIndices.size(); i++) {
                            this.addConstraintLinearPart(equationId, varIndices.get(i), coefficients.get(i));
                        }

                        LOGGER.trace("Added linear constraint #{} of type {}", equationId, equationType);
                    } catch (UnsupportedOperationException e) {
                        throw new PowsyblException("Failed to process linear constraint for equation #" + equationId, e);
                    }
                } else {
                    nonLinearConstraintIds.add(equationId);
                }
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

        private int getcompVarBaseIndex(int equationId){
            return compVarIndex + 5 * vEquationLocalIds.get(equationId);
        }

        /**
         * Configures the Jacobian matrix for the Knitro problem, using either a dense or sparse representation.
         *
         * @param lfNetwork              The PowSyBl network.
         * @param jacobianMatrix         The PowSyBl Jacobian matrix.
         * @param sortedEquationsToSolve The list of equations to solve.
         * @param listNonLinearConsts    The list of non-linear constraint ids.
         * @param listNonZerosCtsDense   Dense non-zero constraints.
         * @param listNonZerosVarsDense  Dense non-zero variables.
         * @param listNonZerosCtsSparse  Sparse non-zero constraints.
         * @param listNonZerosVarsSparse Sparse non-zero variables.
         * @throws KNException If an error occurs in Knitro operations.
         */
        private void setJacobianMatrix(LfNetwork lfNetwork, JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                       List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve, List<Integer> listNonLinearConsts,
                                       List<Integer> listNonZerosCtsDense, List<Integer> listNonZerosVarsDense,
                                       List<Integer> listNonZerosCtsSparse, List<Integer> listNonZerosVarsSparse) throws KNException {

            int numVar = equationSystem.getIndex().getSortedVariablesToFind().size();
            if (knitroParameters.getGradientComputationMode() == 1) { // User routine to compute the Jacobian
                if (knitroParameters.getGradientUserRoutine() == 1) {
                    // Dense method: all non-linear constraints are considered as a function of all variables.
                    buildDenseJacobianMatrix(numVar, listNonLinearConsts, listNonZerosCtsDense, listNonZerosVarsDense);
                    this.setJacNnzPattern(listNonZerosCtsDense, listNonZerosVarsDense);
                } else if (knitroParameters.getGradientUserRoutine() == 2) {
                    // Sparse method: compute Jacobian only for variables the constraints depend on.
                    buildSparseJacobianMatrix(sortedEquationsToSolve, listNonLinearConsts, listNonZerosCtsSparse, listNonZerosVarsSparse);
                    this.setJacNnzPattern(listNonZerosCtsSparse, listNonZerosVarsSparse);
                }
                // Set the callback for gradient evaluations if the user directly passes the Jacobian to the solver.
                this.setGradEvalCallback(new CallbackEvalG(
                        jacobianMatrix,
                        listNonZerosCtsDense,
                        listNonZerosVarsDense,
                        listNonZerosCtsSparse,
                        listNonZerosVarsSparse,
                        lfNetwork,
                        equationSystem,
                        knitroParameters
                ));
            }
        }

        /**
         * Callback used by Knitro to evaluate the non-linear parts of the objective and constraint functions.
         */
        private static final class CallbackEvalFC extends KNEvalFCCallback {

            private final List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve;
            private final List<Integer> nonLinearConstraintIds;
            private final ResilientReacLimKnitroProblem problemInstance;

            private CallbackEvalFC(ResilientReacLimKnitroProblem problemInstance, List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve, List<Integer> nonLinearConstraintIds) {
                this.problemInstance = problemInstance;
                this.sortedEquationsToSolve = sortedEquationsToSolve;
                this.nonLinearConstraintIds = nonLinearConstraintIds;
            }

            /**
             * Knitro callback function that evaluates the non-linear constraints at the current point.
             *
             * @param x   Current point (primal variables).
             * @param obj Output objective value (unused here).
             * @param c   Output constraint values.
             */
            @Override
            public void evaluateFC(final List<Double> x, final List<Double> obj, final List<Double> c) {
                LOGGER.trace("============ Knitro evaluating callback function ============");

                StateVector currentState = new StateVector(toArray(x));
                LOGGER.trace("Current state vector: {}", currentState.get());
                LOGGER.trace("Evaluating {} non-linear constraints", nonLinearConstraintIds.size());

                int callbackConstraintIndex = 0;

                for (int equationId : nonLinearConstraintIds) {
                    Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(equationId);
                    AcEquationType type = equation.getType();

                    // Ensure the constraint is non-linear
                    if (NonLinearExternalSolverUtils.isLinear(type, equation.getTerms())) {
                        throw new IllegalArgumentException(
                                "Equation of type " + type + " is linear but passed to the non-linear callback");
                    }

                    // Evaluate equation using the current state
                    double constraintValue = 0.0;
                    for (EquationTerm<AcVariableType, AcEquationType> term : equation.getTerms()) {
                        term.setStateVector(currentState);
                        if (term.isActive()) {
                            constraintValue += term.eval();
                        }
                    }

                    if (equation.isActive()) { // add slack variables
                        int slackIndexBase = problemInstance.getSlackIndexBase(type, equationId);
                        if (slackIndexBase >= 0) {
                            double sm = x.get(slackIndexBase);        // negative slack
                            double sp = x.get(slackIndexBase + 1);    // positive slack
                            constraintValue += sp - sm;               // add s
                            // slack contribution
                        }
                    } else { // add blow / bup depending on the constraint
                        int elemNum = equation.getElementNum();
                        Equation<AcVariableType, AcEquationType> equationV = sortedEquationsToSolve.stream().filter(
                                e -> e.getElementNum() == elemNum).filter(
                                e->e.getType()==BUS_TARGET_V).toList().get(0);
                        int equationVId = sortedEquationsToSolve.indexOf(equationV);

                        int nmbreEqUnactivated = sortedEquationsToSolve.stream().filter
                                (e -> !e.isActive()).toList().size();
                        int compVarBaseIndex = problemInstance.getcompVarBaseIndex(equationVId);
                        if (equationId - sortedEquationsToSolve.size() + nmbreEqUnactivated/2 < 0) { // Q_low Constraint
                            double b_low = x.get(compVarBaseIndex + 2);
                            constraintValue -= b_low;
                        } else {    // Q_up Constraint
                            double b_up = x.get(compVarBaseIndex + 3);
                            constraintValue += b_up;
                        }
                    }
                    try {
                        c.set(callbackConstraintIndex, constraintValue);
                        LOGGER.trace("Added non-linear constraint #{} (type: {}) = {}", equationId, type, constraintValue);
                    } catch (Exception e) {
                        throw new PowsyblException("Error while adding non-linear constraint #" + equationId, e);
                    }

                    callbackConstraintIndex++;
                }
            }
        }

        /**
         * Callback used by Knitro to evaluate the gradient (Jacobian matrix) of the constraints.
         * Only constraints (no objective) are handled here.
         */
        private static final class CallbackEvalG extends KNEvalGACallback {

            private final JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix;
            private final List<Integer> denseConstraintIndices;
            private final List<Integer> denseVariableIndices;
            private final List<Integer> sparseConstraintIndices;
            private final List<Integer> sparseVariableIndices;

            private final LfNetwork network;
            private final EquationSystem<AcVariableType, AcEquationType> equationSystem;
            private final KnitroSolverParameters knitroParameters;

            private CallbackEvalG(
                    JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                    List<Integer> denseConstraintIndices,
                    List<Integer> denseVariableIndices,
                    List<Integer> sparseConstraintIndices,
                    List<Integer> sparseVariableIndices,
                    LfNetwork network,
                    EquationSystem<AcVariableType, AcEquationType> equationSystem,
                    KnitroSolverParameters knitroParameters) {

                this.jacobianMatrix = jacobianMatrix;
                this.denseConstraintIndices = denseConstraintIndices;
                this.denseVariableIndices = denseVariableIndices;
                this.sparseConstraintIndices = sparseConstraintIndices;
                this.sparseVariableIndices = sparseVariableIndices;
                this.network = network;
                this.equationSystem = equationSystem;
                this.knitroParameters = knitroParameters;
            }

            @Override
            public void evaluateGA(final List<Double> x, final List<Double> objGrad, final List<Double> jac) {
                // Update internal state and Jacobian
                LOGGER.debug("Entering in Callback");
                equationSystem.getStateVector().set(toArray(x));
                AcSolverUtil.updateNetwork(network, equationSystem);
                jacobianMatrix.forceUpdate();

                // Get sparse matrix representation
                SparseMatrix sparseMatrix = jacobianMatrix.getMatrix().toSparse();
                int[] columnStart = sparseMatrix.getColumnStart();
                int[] rowIndices = sparseMatrix.getRowIndices();
                double[] values = sparseMatrix.getValues();

                // Determine which list to use based on Knitro settings
                List<Integer> constraintIndices;
                List<Integer> variableIndices;

                int routineType = knitroParameters.getGradientUserRoutine();
                if (routineType == 1) {
                    constraintIndices = denseConstraintIndices;
                    variableIndices = denseVariableIndices;
                } else if (routineType == 2) {
                    constraintIndices = sparseConstraintIndices;
                    variableIndices = sparseVariableIndices;
                } else {
                    throw new IllegalArgumentException("Unsupported gradientUserRoutine value: " + routineType);
                }

                // Fill Jacobian values
                for (int index = 0; index < constraintIndices.size(); index++) {
                    try {
                        int ct = constraintIndices.get(index);
                        int var = variableIndices.get(index);

                        double value = 0.0;

                        int numLFVar = equationSystem.getIndex().getSortedVariablesToFind().size();
                        int numVEq = equationSystem.getIndex().getSortedEquationsToSolve().stream().filter(
                                e -> e.getType() == BUS_TARGET_V).toList().size();
                        int numPQEq = equationSystem.getIndex().getSortedEquationsToSolve().stream().filter(
                                e -> e.getType() == BUS_TARGET_Q ||
                                        e.getType() == BUS_TARGET_P).toList().size();

                        if (ct < Arrays.stream(columnStart).count()) {
                            // Find matching (var, ct) entry in sparse column
                            int colStart = columnStart[ct];
                            int colEnd = columnStart[ct + 1];

                            boolean found = false;
                            for (int i = colStart; i < colEnd; i++) {
                                if (rowIndices[i] == var) {
                                    value = values[i];
                                    found = true;
                                    break;
                                }
                            }

                            // Check if var is a slack variable (i.e. outside the main variable range)

                            if (var >= numLFVar && var < numLFVar + 2 * numPQEq) {
                                if (((var - numLFVar) % 2) == 0) {
                                    // set Jacobian entry to 1.0 if slack variable is Sm
                                    value = 1.0;
                                } else {
                                    // set Jacobian entry to -1.0 if slack variable is Sp
                                    value = -1.0;
                                }
                            } else if (var >= numLFVar + 2 * numPQEq && var < numLFVar + 2 * (numPQEq + numVEq)) {
                                if (((var - numLFVar) % 2) == 0) {
                                    // set Jacobian entry to -1.0 if slack variable is Sm
                                    value = -1.0;
                                } else {
                                    // set Jacobian entry to 1.0 if slack variable is Sp
                                    value = 1.0;
                                }

                            // Check if var is a b_low or b_up var
                            } else if (var >= numLFVar + 2 * (numPQEq + numVEq)) {
                                int rest = (var - numLFVar - 2 * (numPQEq + numVEq)) % 5;
                                if (rest == 2) {
                                    // set Jacobian entry to -1.0 if variable is b_low
                                    value = -1.0;
                                } else if (rest == 3) {
                                    // set Jacobian entry to 1.0 if variable is b_up
                                    value = 1.0;
                                }
                            }
                            if (!found && knitroParameters.getGradientUserRoutine() == 1) {
                                LOGGER.warn("Dense Jacobian entry not found for constraint {} variable {}", ct, var);
                            }
                            jac.set(index, value);
                        } else {
                            // If var is a LF variable : derivate non-activated equations
                            value = 0.0;
                            Equation<AcVariableType, AcEquationType> equation = indEqUnactiveQ.get(ct);
                            if (var < numLFVar) {
                                // add non-linear terms
                                for (Map.Entry<Variable<AcVariableType>, List<EquationTerm<AcVariableType, AcEquationType>>> e : equation.getTermsByVariable().entrySet()) {
                                    for (EquationTerm<AcVariableType, AcEquationType> term : e.getValue()) {
                                        Variable<AcVariableType> v = e.getKey();
                                        if (equationSystem.getVariableSet().getVariables().stream().filter(va -> va.getRow() == var ).toList().get(0) == v) {
                                            value += term.isActive() ? term.der(v) : 0;
                                        }

                                    }
                                }
                            }
                            // Check if var is a b_low or b_up var
                            if (var >= numLFVar + 2 * (numPQEq + numVEq)) {
                                int rest = (var - numLFVar - 2 * (numPQEq + numVEq)) % 5;
                                if (rest == 2) {
                                    // set Jacobian entry to -1.0 if variable is b_low
                                    value = -1.0;
                                } else if (rest == 3) {
                                    // set Jacobian entry to 1.0 if variable is b_up
                                    value = 1.0;
                                }
                            }
                            jac.set(index, value);
                        }

                    } catch (Exception e) {
                        int varId = routineType == 1 ? denseVariableIndices.get(index) : sparseVariableIndices.get(index);
                        int ctId = routineType == 1 ? denseConstraintIndices.get(index) : sparseConstraintIndices.get(index);
                        LOGGER.error("Error while filling Jacobian term at var {} and constraint {}", varId, ctId, e);
                    }
                }
            }


        }
    }
}

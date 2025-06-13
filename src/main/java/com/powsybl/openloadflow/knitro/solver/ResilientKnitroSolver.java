/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
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
import com.powsybl.openloadflow.network.LfBus;
import com.powsybl.openloadflow.network.LfNetwork;
import com.powsybl.openloadflow.network.util.VoltageInitializer;
import org.apache.commons.lang3.Range;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static com.google.common.primitives.Doubles.toArray;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public class ResilientKnitroSolver extends AbstractAcSolver {

    private static final Logger LOGGER = LoggerFactory.getLogger(ResilientKnitroSolver.class);

    // Penalty weights in the objective function
    private final double wK = 1.0;
    private final double wP = 1.0;
    private final double wQ = 1.0;
    private final double wV = 1.0;

    // Lambda
    private final double lambda = 3.0;

    // Number of Load Flows (LF) variables in the system
    private final int numLFVariables;

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

    protected KnitroSolverParameters knitroParameters;

    public ResilientKnitroSolver(
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

        // Count number of equations by type
        this.numPEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_P).count();
        this.numQEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_Q).count();
        this.numVEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_V).count();

        int numSlackVariables = 2 * (numPEquations + numQEquations + numVEquations);
        this.numTotalVariables = numLFVariables + numSlackVariables;

        this.slackStartIndex = numLFVariables;
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
            }
        }
    }

    /**
     * Returns the name of the solver.
     */
    @Override
    public String getName() {
        return "Knitro Resilient Solver";
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
        ResilientKnitroProblem problemInstance;

        try {
            problemInstance = new ResilientKnitroProblem(network, equationSystem, targetVector, j, voltageInitializer);
        } catch (KNException e) {
            throw new PowsyblException("Exception while building Knitro problem", e);
        }

        try (KNSolver solver = new KNSolver(problemInstance)) {
            solver.initProblem();
            setSolverParameters(solver);
            solver.solve();

            KNSolution solution = solver.getBestFeasibleIterate();
            List<Double> x = solution.getX();

            solverStatus = KnitroStatus.fromStatusCode(solution.getStatus()).toAcSolverStatus();
            logKnitroStatus(KnitroStatus.fromStatusCode(solution.getStatus()));
            nbIterations = solver.getNumberIters();

            LOGGER.info("==== Solution Summary ====");
            LOGGER.info("Objective value            = {}", solution.getObjValue());
            LOGGER.info("Feasibility violation      = {}", solver.getAbsFeasError());
            LOGGER.info("Optimality violation       = {}", solver.getAbsOptError());

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

    private AbstractMap.SimpleEntry<List<Integer>, List<Integer>> getHessNnzRowsAndCols() {
        return new AbstractMap.SimpleEntry<>(
                IntStream.range(slackStartIndex, numTotalVariables).boxed().toList(),
                IntStream.range(slackStartIndex, numTotalVariables).boxed().toList()
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

    private final class ResilientKnitroProblem extends KNProblem {

        /**
         * Knitro problem definition including:
         * - Initialization of variables (types, bounds, initial state)
         * - Definition of linear and non-linear constraints
         * - Objective function setup
         * - Jacobian matrix setup for Knitro
         */
        private ResilientKnitroProblem(
                LfNetwork network,
                EquationSystem<AcVariableType, AcEquationType> equationSystem,
                TargetVector<AcVariableType, AcEquationType> targetVector,
                JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                VoltageInitializer voltageInitializer) throws KNException {

            // =============== Variable Initialization ===============
            super(numTotalVariables, equationSystem.getIndex().getSortedEquationsToSolve().size());
            LOGGER.info("Defining {} variables", numTotalVariables);

            // Variable types (all continuous), bounds, and initial values
            List<Integer> variableTypes = new ArrayList<>(Collections.nCopies(numTotalVariables, KNConstants.KN_VARTYPE_CONTINUOUS));
            List<Double> lowerBounds = new ArrayList<>(Collections.nCopies(numTotalVariables, -KNConstants.KN_INFINITY));
            List<Double> upperBounds = new ArrayList<>(Collections.nCopies(numTotalVariables, KNConstants.KN_INFINITY));
            List<Double> initialValues = new ArrayList<>(Collections.nCopies(numTotalVariables, 0.0));

            setVarTypes(variableTypes);

            // Compute initial state (V, Theta) using the given initializer
            AcSolverUtil.initStateVector(network, equationSystem, voltageInitializer);
            for (int i = 0; i < numLFVariables; i++) {
                initialValues.set(i, equationSystem.getStateVector().get(i));
            }

            // Initialize slack variables (≥ 0, initial value = 0)
            for (int i = slackStartIndex; i < numTotalVariables; i++) {
                lowerBounds.set(i, 0.0);
            }

            // Set bounds for voltage variables based on Knitro parameters
            for (int i = 0; i < numLFVariables; i++) {
                if (equationSystem.getIndex().getSortedVariablesToFind().get(i).getType() == AcVariableType.BUS_V) {
                    lowerBounds.set(i, knitroParameters.getLowerVoltageBound());
                    upperBounds.set(i, knitroParameters.getUpperVoltageBound());
                }
            }

            LOGGER.info("Voltage initialization strategy: {}", voltageInitializer);

            // Set bounds and initial state
            setVarLoBnds(lowerBounds);
            setVarUpBnds(upperBounds);
            setXInitial(initialValues);
            LOGGER.info("Variables initialization complete!");

            // =============== Constraint Setup ===============
            List<Equation<AcVariableType, AcEquationType>> activeConstraints = equationSystem.getIndex().getSortedEquationsToSolve();
            int numConstraints = activeConstraints.size();
            List<Integer> nonlinearConstraintIndexes = new ArrayList<>();

            LOGGER.info("Defining {} active constraints", numConstraints);

            // Linear and nonlinear constraints (the latter are deferred to callback)
            NonLinearExternalSolverUtils solverUtils = new NonLinearExternalSolverUtils();
            addLinearConstraints(activeConstraints, solverUtils, nonlinearConstraintIndexes);
            setMainCallbackCstIndexes(nonlinearConstraintIndexes);
            setConEqBnds(Arrays.stream(targetVector.getArray()).boxed().toList());

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
            setObjEvalCallback(new CallbackEvalFC(this, activeConstraints, nonlinearConstraintIndexes));

            List<Integer> jacCstDense = new ArrayList<>();
            List<Integer> jacVarDense = new ArrayList<>();
            List<Integer> jacCstSparse = new ArrayList<>();
            List<Integer> jacVarSparse = new ArrayList<>();

            setJacobianMatrix(
                    network, jacobianMatrix, activeConstraints, nonlinearConstraintIndexes,
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
         * Adds all active constraints to the Knitro problem, classifying each as linear or non-linear.
         *
         * @param sortedEquationsToSolve Sorted list of equations to solve.
         * @param solverUtils            Utilities to extract linear constraints.
         * @param nonLinearConstraintIds Output list of indices of non-linear constraints.
         */
        private void addLinearConstraints(
                List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                NonLinearExternalSolverUtils solverUtils,
                List<Integer> nonLinearConstraintIds) {

            for (int equationId = 0; equationId < sortedEquationsToSolve.size(); equationId++) {
                addConstraint(equationId, sortedEquationsToSolve, solverUtils, nonLinearConstraintIds);
            }
        }

        /**
         * Adds a single constraint to the Knitro problem.
         * Linear constraints are directly encoded; non-linear ones are delegated to the callback.
         *
         * @param equationId             Index of the equation in the list.
         * @param sortedEquationsToSolve List of all equations to solve.
         * @param solverUtils            Utilities to extract linear constraint components.
         * @param nonLinearConstraintIds Output list of non-linear constraint indices.
         */
        private void addConstraint(
                int equationId,
                List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                NonLinearExternalSolverUtils solverUtils,
                List<Integer> nonLinearConstraintIds) {

            Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(equationId);
            AcEquationType equationType = equation.getType();
            List<EquationTerm<AcVariableType, AcEquationType>> terms = equation.getTerms();

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
                        coefficients.add(1.0);
                        coefficients.add(-1.0);
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
            private final ResilientKnitroProblem problemInstance;

            private CallbackEvalFC(ResilientKnitroProblem problemInstance, List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve, List<Integer> nonLinearConstraintIds) {
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

                    int slackIndexBase = problemInstance.getSlackIndexBase(type, equationId);
                    if (slackIndexBase >= 0) {
                        double sm = x.get(slackIndexBase);        // negative slack
                        double sp = x.get(slackIndexBase + 1);    // positive slack
                        constraintValue += -sm + sp;              // add slack contribution
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
                equationSystem.getStateVector().set(toArray(x));
                AcSolverUtil.updateNetwork(network, equationSystem);
                jacobianMatrix.forceUpdate();

                // Get sparse matrix representation
                SparseMatrix sparseMatrix = jacobianMatrix.getMatrix().toSparse();
                int[] columnStart = sparseMatrix.getColumnStart();
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
                boolean firstIteration = true;
                int iRowIndices = 0;
                int currentConstraint = -1;
                int numVariables = equationSystem.getIndex().getSortedVariablesToFind().size();
                for (int index = 0; index < constraintIndices.size(); index++) {
                    try {
                        if (firstIteration) {
                            currentConstraint = constraintIndices.get(index);
                        }

                        int ct = constraintIndices.get(index);
                        int var = variableIndices.get(index);

                        double value;

                        // Find matching (var, ct) entry in sparse column
                        int colStart = columnStart[ct];

                        if (!firstIteration) {
                            if (currentConstraint != ct) {
                                iRowIndices = 0;
                                currentConstraint = ct;
                            }
                        }

                        if (var >= numVariables) {
                            if ((var & 1) == 0) {
                                // set Jacobian entry to -1.0 if slack variable is Sm
                                value = -1.0;
                            } else {
                                // set Jacobian entry to +1.0 if slack variable is Sp
                                value = 1.0;
                            }
                        } else {
                            value = values[colStart + iRowIndices++];
                        }
                        jac.set(index, value);
                        if (firstIteration) {
                            firstIteration = false;
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

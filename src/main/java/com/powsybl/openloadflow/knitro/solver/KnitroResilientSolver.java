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
 */
public class KnitroResilientSolver extends AbstractAcSolver {

    private static final Logger LOGGER = LoggerFactory.getLogger(KnitroResilientSolver.class);

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

    public KnitroResilientSolver(
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

        // Debug logs
        LOGGER.info("P eq count = {}, Q eq count = {}, V eq count = {}", numPEquations, numQEquations, numVEquations);
    }

    /**
     * Returns the name of the solver.
     */
    @Override
    public String getName() {
        return "Knitro Resilient Solver";
    }

    /**
     * Enum representing possible status codes returned by Knitro,
     * grouped by ranges and associated with a corresponding AcSolverStatus.
     */
    public enum KnitroStatus {

        CONVERGED_TO_LOCAL_OPTIMUM(0, 0, AcSolverStatus.CONVERGED),
        CONVERGED_TO_FEASIBLE_APPROXIMATE_SOLUTION(-199, -100, AcSolverStatus.CONVERGED),
        TERMINATED_AT_INFEASIBLE_POINT(-299, -200, AcSolverStatus.SOLVER_FAILED),
        PROBLEM_UNBOUNDED(-399, -300, AcSolverStatus.SOLVER_FAILED),
        TERMINATED_DUE_TO_PRE_DEFINED_LIMIT(-499, -400, AcSolverStatus.MAX_ITERATION_REACHED),
        INPUT_OR_NON_STANDARD_ERROR(-599, -500, AcSolverStatus.SOLVER_FAILED);

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
         * @return the matching KnitroStatus enum constant
         * @throws IllegalArgumentException if the status code does not match any known range
         */
        public static KnitroStatus fromStatusCode(int statusCode) {
            return Arrays.stream(KnitroStatus.values())
                    .filter(status -> status.codeRange.contains(statusCode))
                    .findFirst()
                    .orElseThrow(() -> new IllegalArgumentException("Unknown Knitro status code: " + statusCode));
        }

        /**
         * Returns the AcSolverStatus associated with this KnitroStatus.
         *
         * @return the corresponding AcSolverStatus
         */
        public AcSolverStatus toAcSolverStatus() {
            return mappedStatus;
        }
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
                    .collect(Collectors.toCollection(TreeSet::new)); // TreeSet = sorted + unique

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

    private final class ResilientKnitroProblem extends KNProblem {

        /**
         * Callback used by Knitro to evaluate the non-linear parts of the objective and constraint functions.
         */
        private static final class CallbackEvalFC extends KNEvalFCCallback {

            private final List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve;
            private final List<Integer> nonLinearConstraintIds;

            private CallbackEvalFC(List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                                   List<Integer> nonLinearConstraintIds) {
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

                        // Find matching (var, ct) entry in sparse column
                        int colStart = columnStart[ct];
                        int colEnd = columnStart[ct + 1];

                        for (int i = colStart; i < colEnd; i++) {
                            if (rowIndices[i] == var) {
                                value = values[i];
                                break;
                            }
                        }

                        jac.set(index, value);

                    } catch (Exception e) {
                        int varId = routineType == 1 ? denseVariableIndices.get(index) : sparseVariableIndices.get(index);
                        int ctId = routineType == 1 ? denseConstraintIndices.get(index) : sparseConstraintIndices.get(index);
                        LOGGER.error("Error while filling Jacobian term at var {} and constraint {}", varId, ctId, e);
                    }
                }
            }
        }

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

            // Compute initial voltage state using the given initializer
            AcSolverUtil.initStateVector(network, equationSystem, voltageInitializer);
            for (int i = 0; i < numLFVariables; i++) {
                initialValues.set(i, equationSystem.getStateVector().get(i));
            }

            // Initialize slack variables (≥ 0, initial value = 0)
            for (int i = slackStartIndex; i < numTotalVariables; i++) {
                lowerBounds.set(i, 0.0);
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
            double wK = 1.0;
            double wP = 1000.0;
            double wQ = 10.0;
            double wV = 1.0;

            List<Integer> quadRows = new ArrayList<>();
            List<Integer> quadCols = new ArrayList<>();
            List<Double> quadCoefs = new ArrayList<>();

            List<Integer> linIndexes = new ArrayList<>();
            List<Double> linCoefs = new ArrayList<>();

            // Slack penalty terms: (Sp - Sm)^2 = Sp^2 + Sm^2 - 2*Sp*Sm + linear terms from the absolute value
            addSlackObjectiveTerms(numPEquations, slackPStartIndex, wK * wP, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);
            addSlackObjectiveTerms(numQEquations, slackQStartIndex, wK * wQ, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);
            addSlackObjectiveTerms(numVEquations, slackVStartIndex, wV, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);

            setObjectiveQuadraticPart(quadRows, quadCols, quadCoefs);
            setObjectiveLinearPart(linIndexes, linCoefs);

            // =============== Callbacks and Jacobian ===============
            setObjEvalCallback(new CallbackEvalFC(activeConstraints, nonlinearConstraintIndexes));

            List<Integer> jacCstDense = new ArrayList<>();
            List<Integer> jacVarDense = new ArrayList<>();
            List<Integer> jacCstSparse = new ArrayList<>();
            List<Integer> jacVarSparse = new ArrayList<>();

            setJacobianMatrix(
                    network, jacobianMatrix, activeConstraints, nonlinearConstraintIndexes,
                    jacCstDense, jacVarDense, jacCstSparse, jacVarSparse
            );
        }

        /**
         * Adds quadratic and linear terms related to slack variables to the objective function.
         */
        private void addSlackObjectiveTerms(
                int numEquations,
                int slackStartIdx,
                double weight,
                List<Integer> quadRows,
                List<Integer> quadCols,
                List<Double> quadCoefs,
                List<Integer> linIndexes,
                List<Double> linCoefs) {

            for (int i = 0; i < numEquations; i++) {
                int idxSm = slackStartIdx + 2 * i;
                int idxSp = slackStartIdx + 2 * i + 1;

                // Quadratic terms
                quadRows.add(idxSp);
                quadCols.add(idxSp);
                quadCoefs.add(weight);
                quadRows.add(idxSm);
                quadCols.add(idxSm);
                quadCoefs.add(weight);
                quadRows.add(idxSp);
                quadCols.add(idxSm);
                quadCoefs.add(-2 * weight);

                // Linear terms
                linIndexes.add(idxSp);
                linCoefs.add(weight);
                linIndexes.add(idxSm);
                linCoefs.add(weight);
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
                this.setGradEvalCallback(new KnitroResilientSolver.ResilientKnitroProblem.CallbackEvalG(
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

        LOGGER.info("Knitro parameters set: GRADOPT={}, FEASTOL={}, MAXIT={}",
                knitroParameters.getGradientComputationMode(),
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

            KNSolution solution = solver.getSolution();
            List<Double> constraintValues = solver.getConstraintValues();
            List<Double> x = solution.getX();
            List<Double> lambda = solution.getLambda();

            solverStatus = KnitroStatus.fromStatusCode(solution.getStatus()).toAcSolverStatus();
            logKnitroStatus(KnitroStatus.fromStatusCode(solution.getStatus()));
            nbIterations = solver.getNumberIters();

            LOGGER.info("==== Solution Summary ====");
            LOGGER.info("Objective value            = {}", solution.getObjValue());
            LOGGER.info("Feasibility violation      = {}", solver.getAbsFeasError());
            LOGGER.info("Optimality violation       = {}", solver.getAbsOptError());

            // Log primal solution
            LOGGER.debug("==== Optimal variables ====");
            for (int i = 0; i < x.size(); i++) {
                LOGGER.debug(" x[{}] = {}", i, x.get(i));
            }

            LOGGER.debug("==== Constraint values ====");
            for (int i = 0; i < problemInstance.getNumCons(); i++) {
                LOGGER.debug(" c[{}] = {} (λ = {})", i, constraintValues.get(i), lambda.get(i));
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
            double wK = 1.0;
            double wP = 1000.0;
            double wQ = 10.0;
            double wV = 1.0;

            double penaltyP = computeSlackPenalty(x, slackPStartIndex, numPEquations, wK * wP);
            double penaltyQ = computeSlackPenalty(x, slackQStartIndex, numQEquations, wK * wQ);
            double penaltyV = computeSlackPenalty(x, slackVStartIndex, numVEquations, wV);
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
        LOGGER.debug("==== Slack {} ====", type);
        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            LOGGER.debug("Slack {}[{}] -> Sm = {}, Sp = {}", type, i, sm, sp);
        }
    }

    private double computeSlackPenalty(List<Double> x, int startIndex, int count, double weight) {
        double penalty = 0.0;
        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            double diff = sp - sm;
            penalty += weight * (diff * diff + sp + sm);
        }
        return penalty;
    }
}

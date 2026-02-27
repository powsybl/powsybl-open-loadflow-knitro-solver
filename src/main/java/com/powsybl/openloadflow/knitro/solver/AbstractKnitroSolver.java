/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.artelys.knitro.api.*;
import com.artelys.knitro.api.callbacks.KNEvalGACallback;
import com.powsybl.commons.PowsyblException;
import com.powsybl.commons.report.ReportNode;
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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.IntStream;
import static com.google.common.primitives.Doubles.toArray;

/**
 * Abstract base class for Knitro solvers dedicated to solving the open load-flow equation system.
 * It provides common functionality, including:
 *      - Configuration of the external Knitro solver.
 *      - Creation of the optimization problem (extending {@link AbstractKnitroProblem}).
 *      - Processing of the obtained solution.
 * This class can be extended to add custom behavior to any of these features (e.g., in {@link AbstractRelaxedKnitroSolver}).
 * For example, if you modify the optimization problem, you may also need to update the solution-processing logic.
 *
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public abstract class AbstractKnitroSolver extends AbstractAcSolver {

    protected static final Logger LOGGER = LoggerFactory.getLogger(AbstractKnitroSolver.class);

    protected KnitroSolverParameters knitroParameters;

    protected AbstractKnitroSolver(LfNetwork network, KnitroSolverParameters knitroParameters,
                                EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                JacobianMatrix<AcVariableType, AcEquationType> j, TargetVector<AcVariableType, AcEquationType> targetVector,
                                EquationVector<AcVariableType, AcEquationType> equationVector,
                                boolean detailedReport) {

        super(network, equationSystem, j, targetVector, equationVector, detailedReport);
        this.knitroParameters = knitroParameters;
    }

    /**
     * Sets Knitro solver parameters based on the provided KnitroSolverParameters object.
     * Uses the improved parameter set from RelaxedKnitroSolver.
     *
     * @param solver The Knitro solver instance to configure.
     * @throws KNException if Knitro fails to accept a parameter.
     */
    protected void setSolverParameters(KNSolver solver) throws KNException {
        LOGGER.info("Configuring Knitro solver parameters...");

        solver.setParam(KNConstants.KN_PARAM_GRADOPT, knitroParameters.getGradientComputationMode());
        solver.setParam(KNConstants.KN_PARAM_FEASTOL, knitroParameters.getRelConvEps());
        solver.setParam(KNConstants.KN_PARAM_FEASTOLABS, knitroParameters.getAbsConvEps());
        solver.setParam(KNConstants.KN_PARAM_OPTTOL, knitroParameters.getRelOptEps());
        solver.setParam(KNConstants.KN_PARAM_OPTTOLABS, knitroParameters.getAbsOptEps());
        solver.setParam(KNConstants.KN_PARAM_MAXIT, knitroParameters.getMaxIterations());
        solver.setParam(KNConstants.KN_PARAM_HESSOPT, knitroParameters.getHessianComputationMode());
        solver.setParam(KNConstants.KN_PARAM_SOLTYPE, KNConstants.KN_SOLTYPE_BESTFEAS);
        solver.setParam(KNConstants.KN_PARAM_OUTLEV, 3);
        solver.setParam(KNConstants.KN_PARAM_NUMTHREADS, knitroParameters.getThreadNumber());

        LOGGER.info("Knitro parameters set: GRADOPT={}, HESSOPT={}, FEASTOL={}, OPTTOL={}, MAXIT={}",
                knitroParameters.getGradientComputationMode(),
                knitroParameters.getHessianComputationMode(),
                knitroParameters.getRelConvEps(),
                knitroParameters.getRelOptEps(),
                knitroParameters.getMaxIterations());
    }

    /**
     * Creates the Knitro problem instance.
     * This must be implemented by subclasses.
     *
     * @param voltageInitializer The voltage initializer to use. This initializes the optimization.
     * @return The created Knitro problem instance.
     */
    protected abstract KNProblem createKnitroProblem(VoltageInitializer voltageInitializer);

    /**
     * Processes the solution after solving.
     * Can be overridden by subclasses for additional processing.
     *
     * @param solver The Knitro solver instance.
     * @param solution The solution obtained from Knitro.
     * @param problemInstance The problem instance.
     */
    protected void processSolution(KNSolver solver, KNSolution solution, KNProblem problemInstance) {
        // log optimization solution details
        LOGGER.info("==== Solution Summary ====");
        LOGGER.info("Optimal objective value  = {}", solution.getObjValue());
        try {
            LOGGER.info("Feasibility violation    = {}", solver.getAbsFeasError());
            LOGGER.info("Optimality violation     = {}", solver.getAbsOptError());

            LOGGER.debug("Optimal x");
            for (int i = 0; i < solution.getX().size(); i++) {
                LOGGER.debug(" x[{}] = {}", i, solution.getX().get(i));
            }
            LOGGER.debug("Optimal constraint values (with corresponding multiplier)");
            List<Double> constraintValues = solver.getConstraintValues();
            for (int i = 0; i < problemInstance.getNumCons(); i++) {
                LOGGER.debug(" c[{}] = {} (lambda = {} )", i, constraintValues.get(i), solution.getLambda().get(i));
            }
            LOGGER.debug("Constraint violation");
            for (int i = 0; i < problemInstance.getNumCons(); i++) {
                LOGGER.debug(" violation[{}] = {} ", i, solver.getConViol(i));
            }
        } catch (KNException e) {
            LOGGER.warn("Failed to get some solution details", e);
        }
    }

    public static void logKnitroStatus(KnitroStatus status) {
        LOGGER.info("Knitro Status: {}", status);
    }

    /**
     * Gets the solution from the solver.
     * Can be overridden by subclasses.
     *
     * @param solver The Knitro solver instance.
     * @return The solution.
     */
    protected KNSolution getSolution(KNSolver solver) throws KNException {
        return solver.getSolution();
    }

    /**
     * Updates the network with the solution if required.
     *
     * @param solution The solution to apply.
     * @param solverStatus The solver status.
     */
    protected void updateNetworkIfRequired(KNSolution solution, AcSolverStatus solverStatus) {
        if (solverStatus == AcSolverStatus.CONVERGED || knitroParameters.isAlwaysUpdateNetwork()) {
            equationSystem.getStateVector().set(toArray(solution.getX()));
            for (Equation<AcVariableType, AcEquationType> equation : equationSystem.getEquations()) {
                for (EquationTerm<AcVariableType, AcEquationType> term : equation.getTerms()) {
                    term.setStateVector(equationSystem.getStateVector());
                }
            }
            AcSolverUtil.updateNetwork(network, equationSystem);
        }
    }

    @Override
    public AcSolverResult run(VoltageInitializer voltageInitializer, ReportNode reportNode) {
        int nbIter;
        AcSolverStatus acStatus;
        KNProblem instance;

        try {
            instance = createKnitroProblem(voltageInitializer);
        } catch (Exception e) {
            throw new PowsyblException("Exception while trying to build Knitro Problem", e);
        }

        try (KNSolver solver = new KNSolver(instance)) {
            solver.initProblem();
            setSolverParameters(solver);
            solver.solve();

            KNSolution solution = getSolution(solver);
            acStatus = KnitroStatus.fromStatusCode(solution.getStatus()).toAcSolverStatus();
            logKnitroStatus(KnitroStatus.fromStatusCode(solution.getStatus()));
            nbIter = solver.getNumberIters();

            processSolution(solver, solution, instance);
            updateNetworkIfRequired(solution, acStatus);

        } catch (KNException e) {
            throw new PowsyblException("Exception while trying to solve with Knitro", e);
        }

        double slackBusActivePowerMismatch = network.getSlackBuses().stream()
                .mapToDouble(LfBus::getMismatchP)
                .sum();
        return new AcSolverResult(acStatus, nbIter, slackBusActivePowerMismatch);
    }

    public abstract class AbstractKnitroProblem extends KNProblem {

        protected static final Logger LOGGER = LoggerFactory.getLogger(AbstractKnitroProblem.class);

        protected final LfNetwork network;
        protected final EquationSystem<AcVariableType, AcEquationType> equationSystem;
        protected final TargetVector<AcVariableType, AcEquationType> targetVector;
        protected final JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix;
        protected final KnitroSolverParameters knitroParameters;
        protected final int numberOfPowerFlowVariables;
        protected List<Equation<AcVariableType, AcEquationType>> activeConstraints = new ArrayList<>();
        protected final List<Integer> nonlinearConstraintIndexes = new ArrayList<>();
        protected final int numTotalVariables;

        protected AbstractKnitroProblem(LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                        TargetVector<AcVariableType, AcEquationType> targetVector, JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                        KnitroSolverParameters knitroParameters) {
            this(network, equationSystem, targetVector, jacobianMatrix, knitroParameters, 0, 0);
        }

        protected AbstractKnitroProblem(LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                        TargetVector<AcVariableType, AcEquationType> targetVector, JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                        KnitroSolverParameters knitroParameters, int numAdditionalVariables, int numAdditionalConstraints) {
            super(equationSystem.getIndex().getSortedVariablesToFind().size() + numAdditionalVariables,
                    equationSystem.getIndex().getSortedEquationsToSolve().size() + numAdditionalConstraints);
            this.numberOfPowerFlowVariables = equationSystem.getIndex().getSortedVariablesToFind().size();
            this.numTotalVariables = numberOfPowerFlowVariables + numAdditionalVariables;
            this.network = network;
            this.equationSystem = equationSystem;
            this.targetVector = targetVector;
            this.jacobianMatrix = jacobianMatrix;
            this.knitroParameters = knitroParameters;
        }

        /**
         * Initializes variables (types, bounds, initial values).
         * Can be overridden by subclasses to add additional variables (e.g., slack variables).
         *
         * @param voltageInitializer The voltage initializer to use.
         */
        protected void initializeVariables(VoltageInitializer voltageInitializer) throws KNException {
            List<Integer> variableTypes = new ArrayList<>(Collections.nCopies(numTotalVariables, KNConstants.KN_VARTYPE_CONTINUOUS));
            List<Double> lowerBounds = new ArrayList<>(Collections.nCopies(numTotalVariables, -KNConstants.KN_INFINITY));
            List<Double> upperBounds = new ArrayList<>(Collections.nCopies(numTotalVariables, KNConstants.KN_INFINITY));
            List<Double> initialValues = new ArrayList<>(Collections.nCopies(numTotalVariables, 0.0));

            setVarTypes(variableTypes);

            // Compute initial state (V, Theta) using the given initializer
            AcSolverUtil.initStateVector(network, equationSystem, voltageInitializer);
            for (int i = 0; i < numberOfPowerFlowVariables; i++) {
                initialValues.set(i, equationSystem.getStateVector().get(i));
            }

            // Set bounds for voltage variables based on Knitro parameters
            List<Variable<AcVariableType>> sortedVariables = equationSystem.getIndex().getSortedVariablesToFind();
            for (int i = 0; i < numberOfPowerFlowVariables; i++) {
                if (sortedVariables.get(i).getType() == AcVariableType.BUS_V) {
                    lowerBounds.set(i, knitroParameters.getLowerVoltageBound());
                    upperBounds.set(i, knitroParameters.getUpperVoltageBound());
                }
            }

            // Allow subclasses to modify bounds and initial values (e.g., for slack variables)
            initializeCustomizedVariables(lowerBounds, upperBounds, initialValues);

            // Set up scaling factors
            setUpScalingFactors();

            setVarLoBnds(lowerBounds);
            setVarUpBnds(upperBounds);
            setXInitial(initialValues);
            LOGGER.info("Variables initialization complete!");
        }

        /**
         * Allows subclasses to initialize additional variables (bounds, initial values).
         * Default implementation does nothing.
         *
         * @param lowerBounds Lower bounds list to modify.
         * @param upperBounds Upper bounds list to modify.
         * @param initialValues Initial values list to modify.
         */
        protected void initializeCustomizedVariables(List<Double> lowerBounds, List<Double> upperBounds,
                                                     List<Double> initialValues) {
            // no customization by default
        }

        /**
         * Allows subclasses to utilize scaling.
         * Default implementation does nothing.
         *
         */
        protected void setUpScalingFactors() throws KNException {
            // no customization by default
        }

        /**
         * Sets up constraints of the optimization problem.
         * Separate linear and nonlinear constraints to determine which ones
         * will be evaluated through callback functions.
         */
        protected void setupConstraints() throws KNException {
            activeConstraints = equationSystem.getIndex().getSortedEquationsToSolve();
            int numConstraints = activeConstraints.size();

            LOGGER.info("Defining {} active constraints", numConstraints);

            NonLinearExternalSolverUtils solverUtils = new NonLinearExternalSolverUtils();

            // add linear constraints and fill the list of non-linear constraints
            addLinearConstraints(activeConstraints, solverUtils);

            // pass to Knitro the indexes of non-linear constraints, that will be evaluated in the callback function
            setMainCallbackCstIndexes(nonlinearConstraintIndexes);

            // right hand side (targets)
            setConEqBnds(Arrays.stream(targetVector.getArray()).boxed().toList());
        }

        /**
         * Adds all active constraints to the Knitro problem, classifying each as linear or non-linear.
         *
         * @param sortedEquationsToSolve Sorted list of equations to solve.
         * @param solverUtils Utilities to extract linear constraints.
         */
        protected void addLinearConstraints(List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                                            NonLinearExternalSolverUtils solverUtils) {

            for (int equationId = 0; equationId < sortedEquationsToSolve.size(); equationId++) {
                addConstraint(equationId, sortedEquationsToSolve, solverUtils);
            }
        }

        /**
         * Adds a single constraint to the Knitro problem.
         * Linear constraints are directly encoded; non-linear ones are delegated to the callback.
         *
         * @param equationId Index of the equation in the list.
         * @param sortedEquationsToSolve List of all equations to solve.
         * @param solverUtils Utilities to extract linear constraint components.
         */
        protected void addConstraint(int equationId, List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                                     NonLinearExternalSolverUtils solverUtils) {

            Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(equationId);
            AcEquationType equationType = equation.getType();
            List<EquationTerm<AcVariableType, AcEquationType>> terms = equation.getTerms();

            if (NonLinearExternalSolverUtils.isLinear(equationType, terms)) {
                try {
                    var linearConstraint = solverUtils.getLinearConstraint(equationType, terms);
                    List<Integer> varIndices = new ArrayList<>(linearConstraint.listIdVar());
                    List<Double> coefficients = new ArrayList<>(linearConstraint.listCoef());

                    // Allow subclasses to add additional variables (e.g., slack variables)
                    addAdditionalConstraintVariables(equationId, equationType, varIndices, coefficients);

                    for (int i = 0; i < varIndices.size(); i++) {
                        this.addConstraintLinearPart(equationId, varIndices.get(i), coefficients.get(i));
                    }

                    LOGGER.trace("Added linear constraint #{} of type {}", equationId, equationType);
                } catch (UnsupportedOperationException e) {
                    throw new PowsyblException("Failed to process linear constraint for equation #" + equationId, e);
                }
            } else {
                nonlinearConstraintIndexes.add(equationId);
            }
        }

        /**
         * Allows subclasses to add additional variables to a linear constraint (e.g., slack variables).
         * Default implementation does nothing.
         *
         * @param equationId The equation ID.
         * @param equationType The equation type.
         * @param varIndices Variable indices list to modify.
         * @param coefficients Coefficients list to modify.
         */
        protected void addAdditionalConstraintVariables(int equationId, AcEquationType equationType,
                                                        List<Integer> varIndices, List<Double> coefficients) {
            // no additional variables
        }

        /**
         * Configures the Jacobian matrix for the Knitro problem, using either a dense or sparse representation.
         *
         * @param sortedEquationsToSolve The list of equations to solve.
         * @param listNonLinearConsts The list of non-linear constraint ids.
         */
        protected void setJacobianMatrix(List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve, List<Integer> listNonLinearConsts) {
            // Non-zero pattern : for each constraint, we detail the variables of which the constraint is a function of.
            List<Integer> listNonZerosCtsDense = new ArrayList<>();
            List<Integer> listNonZerosVarsDense = new ArrayList<>();
            List<Integer> listNonZerosCtsSparse = new ArrayList<>();
            List<Integer> listNonZerosVarsSparse = new ArrayList<>();
            if (knitroParameters.getGradientComputationMode() == 1) { // User routine to compute the Jacobian
                try {
                    if (knitroParameters.getGradientUserRoutine() == 1) {
                        // Dense method: all non-linear constraints are considered as a function of all variables.
                        buildDenseJacobianMatrix(numberOfPowerFlowVariables, listNonLinearConsts,
                                listNonZerosCtsDense, listNonZerosVarsDense);
                        this.setJacNnzPattern(listNonZerosCtsDense, listNonZerosVarsDense);
                    } else if (knitroParameters.getGradientUserRoutine() == 2) {
                        // Sparse method: compute Jacobian only for variables the constraints depend on.
                        buildSparseJacobianMatrix(sortedEquationsToSolve, listNonLinearConsts,
                                listNonZerosCtsSparse, listNonZerosVarsSparse);
                        this.setJacNnzPattern(listNonZerosCtsSparse, listNonZerosVarsSparse);
                    }
                } catch (KNException e) {
                    throw new PowsyblException("Failed to set Jacobian pattern in Knitro problem", e);
                }
                // Set the callback for gradient evaluations if the user directly passes the Jacobian to the solver.
                this.setGradEvalCallback(createGradientCallback(jacobianMatrix, listNonZerosCtsDense, listNonZerosVarsDense,
                        listNonZerosCtsSparse, listNonZerosVarsSparse));
            }
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
         * Builds the sparse Jacobian matrix by identifying non-zero entries for each non-linear constraint.
         * Can be overridden by subclasses to include additional variables (e.g., slack variables).
         *
         * @param sortedEquationsToSolve Ordered list of equations to solve.
         * @param nonLinearConstraintIds Indices of non-linear constraints within the sorted equation list.
         * @param jacobianRowIndices Output: row indices (constraints) of non-zero Jacobian entries.
         * @param jacobianColumnIndices Output: column indices (variables) of non-zero Jacobian entries.
         */
        protected void buildSparseJacobianMatrix(List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve, List<Integer> nonLinearConstraintIds,
                                                 List<Integer> jacobianRowIndices, List<Integer> jacobianColumnIndices) {

            for (Integer constraintIndex : nonLinearConstraintIds) {
                Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(constraintIndex);
                List<EquationTerm<AcVariableType, AcEquationType>> terms = equation.getTerms();
                List<Integer> listNonZerosVarsCurrentCt = new ArrayList<>();

                for (EquationTerm<AcVariableType, AcEquationType> term : terms) {
                    for (Variable<AcVariableType> variable : term.getVariables()) {
                        listNonZerosVarsCurrentCt.add(variable.getRow());
                    }
                }
                List<Integer> uniqueListVarsCurrentCt = new ArrayList<>(
                        listNonZerosVarsCurrentCt.stream()
                                .distinct()
                                .sorted()
                                .toList());

                // Allow subclasses to add additional variables (e.g., slack variables)
                addAdditionalJacobianVariables(constraintIndex, equation, uniqueListVarsCurrentCt);

                jacobianColumnIndices.addAll(uniqueListVarsCurrentCt);
                jacobianRowIndices.addAll(Collections.nCopies(uniqueListVarsCurrentCt.size(), constraintIndex));
            }
        }

        /**
         * Allows subclasses to add additional variables to the Jacobian pattern (e.g., slack variables).
         * Default implementation does nothing.
         *
         * @param constraintIndex The constraint index.
         * @param equation The equation.
         * @param variableIndices The variable indices list to modify.
         */
        protected void addAdditionalJacobianVariables(int constraintIndex, Equation<AcVariableType, AcEquationType> equation,
                                                      List<Integer> variableIndices) {
            // no additional variables by default
        }

        /**
         * Creates the gradient callback. Must be implemented by subclasses.
         *
         * @param jacobianMatrix The Jacobian matrix.
         * @param listNonZerosCtsDense Dense constraint indices.
         * @param listNonZerosVarsDense Dense variable indices.
         * @param listNonZerosCtsSparse Sparse constraint indices.
         * @param listNonZerosVarsSparse Sparse variable indices.
         * @return The gradient callback instance.
         */
        protected abstract KNEvalGACallback createGradientCallback(JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                                                   List<Integer> listNonZerosCtsDense, List<Integer> listNonZerosVarsDense,
                                                                   List<Integer> listNonZerosCtsSparse,
                                                                   List<Integer> listNonZerosVarsSparse);
    }
}


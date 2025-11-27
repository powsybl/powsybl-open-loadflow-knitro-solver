/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.artelys.knitro.api.*;
import com.artelys.knitro.api.callbacks.*;
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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;

import static com.google.common.primitives.Doubles.toArray;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 */
public class KnitroSolver extends AbstractAcSolver {

    private static final Logger LOGGER = LoggerFactory.getLogger(KnitroSolver.class);

    protected KnitroSolverParameters knitroParameters;

    public KnitroSolver(
            LfNetwork network,
            KnitroSolverParameters knitroParameters,
            EquationSystem<AcVariableType, AcEquationType> equationSystem,
            JacobianMatrix<AcVariableType, AcEquationType> j,
            TargetVector<AcVariableType, AcEquationType> targetVector,
            EquationVector<AcVariableType, AcEquationType> equationVector,
            boolean detailedReport) {

        super(network, equationSystem, j, targetVector, equationVector, detailedReport);
        this.knitroParameters = knitroParameters;
    }

    @Override
    public String getName() {
        return "Knitro Solver";
    }

    /**
     * Handles setting lower and upper variable bounds.
     */
    public class VariableBounds {
        private final List<Double> lowerBounds;
        private final List<Double> upperBounds;

        public VariableBounds(List<Variable<AcVariableType>> sortedVariables) {
            this.lowerBounds = new ArrayList<>();
            this.upperBounds = new ArrayList<>();
            setBounds(sortedVariables);
        }

        private void setBounds(List<Variable<AcVariableType>> sortedVariables) {
            double loBndV = knitroParameters.getLowerVoltageBound();
            double upBndV = knitroParameters.getUpperVoltageBound();

            for (Variable<AcVariableType> variable : sortedVariables) {
                Enum<AcVariableType> typeVar = variable.getType();
                if (typeVar == AcVariableType.BUS_V) {
                    lowerBounds.add(loBndV);
                    upperBounds.add(upBndV);
                } else {
                    lowerBounds.add(-KNConstants.KN_INFINITY);
                    upperBounds.add(KNConstants.KN_INFINITY);
                }
            }
        }

        public List<Double> getLowerBounds() {
            return lowerBounds;
        }

        public List<Double> getUpperBounds() {
            return upperBounds;
        }
    }

    /**
     * Handles initialization of the state vector.
     */
    public static class StateInitializer {
        private final List<Double> initialState;

        public StateInitializer(LfNetwork lfNetwork, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                VoltageInitializer voltageInitializer) {
            int numVar = equationSystem.getIndex().getSortedVariablesToFind().size();
            this.initialState = new ArrayList<>(numVar);
            initializeState(lfNetwork, equationSystem, voltageInitializer);
        }

        private void initializeState(LfNetwork lfNetwork, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                     VoltageInitializer voltageInitializer) {
            AcSolverUtil.initStateVector(lfNetwork, equationSystem, voltageInitializer);
            for (int i = 0; i < equationSystem.getIndex().getSortedVariablesToFind().size(); i++) {
                initialState.add(equationSystem.getStateVector().get(i));
            }
        }

        public List<Double> getInitialState() {
            return initialState;
        }
    }

    public void buildSparseJacobianMatrix(List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve, List<Integer> listNonLinearConsts, List<Integer> listNonZerosCtsSparse, List<Integer> listNonZerosVarsSparse) {
        for (Integer ct : listNonLinearConsts) { // for each non-linear constraint, we get the variables of which it depends on
            Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(ct);
            List<EquationTerm<AcVariableType, AcEquationType>> terms = equation.getTerms();
            List<Integer> listNonZerosVarsCurrentCt = new ArrayList<>(); //list of variables involved in current constraint

            for (EquationTerm<AcVariableType, AcEquationType> term : terms) {
                for (Variable<AcVariableType> variable : term.getVariables()) {
                    listNonZerosVarsCurrentCt.add(variable.getRow());
                }
            }
            List<Integer> uniqueListVarsCurrentCt = listNonZerosVarsCurrentCt.stream().distinct().sorted().toList(); // remove duplicate elements from the list, because the same variables may be present in several terms of the constraint
            listNonZerosVarsSparse.addAll(uniqueListVarsCurrentCt);
            // we add uniqueListVarsCurrentCt.size() times the constraint ct to the list of constraints to derive
            listNonZerosCtsSparse.addAll(new ArrayList<>(Collections.nCopies(uniqueListVarsCurrentCt.size(), ct)));
        }
    }

    private final class KnitroProblem extends KNProblem {

        /**
         * Callback function to evaluate non-linear parts of the objective and of constraints
         */

        private static final class CallbackEvalFC extends KNEvalFCCallback {

            private final List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve;
            private final List<Integer> listNonLinearConsts;

            private CallbackEvalFC(List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve, List<Integer> listNonLinearConsts) {
                this.sortedEquationsToSolve = sortedEquationsToSolve;
                this.listNonLinearConsts = listNonLinearConsts;
            }

            // Callback function for non-linear parts of objective and constraints
            @Override
            public void evaluateFC(final List<Double> x, final List<Double> obj, final List<Double> c) {
                LOGGER.trace("============ Knitro evaluating callback function ============");

                // =============== Non-linear constraints in P and Q ===============

                // Update current state
                StateVector currentState = new StateVector(toArray(x));
                LOGGER.trace("Current state vector {}", currentState.get());
                LOGGER.trace("Evaluating {} non-linear constraints", listNonLinearConsts.size());

                // Add non-linear constraints
                int indexNonLinearCst = 0; // callback index of current constraint added
                for (int equationId : listNonLinearConsts) {
                    Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(equationId);
                    AcEquationType typeEq = equation.getType();
                    double valueConst = 0;
                    if (NonLinearExternalSolverUtils.isLinear(typeEq, equation.getTerms())) { // check that the constraint is really non-linear
                        throw new IllegalArgumentException("Equation of type " + typeEq + " is linear, and should be considered in the main function of Knitro, not in the callback function");
                    } else {
                        // we evaluate the equation with respect to the current state
                        for (EquationTerm<AcVariableType, AcEquationType> term : equation.getTerms()) {
                            term.setStateVector(currentState);
                            if (term.isActive()) {
                                valueConst += term.eval();
                            }
                        }
                        try {
                            c.set(indexNonLinearCst, valueConst); // adding the constraint
                            LOGGER.trace("Adding non-linear constraint n° {}, of type {} and of value {}", equationId, typeEq, valueConst);
                        } catch (Exception e) {
                            throw new PowsyblException("Exception found while trying to add non-linear constraint n° " + equationId, e);
                        }
                    }
                    indexNonLinearCst += 1; // we move on to the next constraint
                }
            }
        }

        /**
         * Callback function to evaluate the gradient
         * When no objective, that is to say the matrix of the constraints, the Jacobian matrix
         */
        private static final class CallbackEvalG extends KNEvalGACallback {
            private final JacobianMatrix<AcVariableType, AcEquationType> oldMatrix;
            private final List<Integer> listNonZerosCtsDense;
            private final List<Integer> listNonZerosVarsDense;
            private final List<Integer> listNonZerosCtsSparse;
            private final List<Integer> listNonZerosVarsSparse;
            private final LfNetwork network;
            private final EquationSystem<AcVariableType, AcEquationType> equationSystem;
            private final KnitroSolverParameters knitroParameters;

            private CallbackEvalG(JacobianMatrix<AcVariableType, AcEquationType> oldMatrix, List<Integer> listNonZerosCts, List<Integer> listNonZerosVars, List<Integer> listNonZerosCts2, List<Integer> listNonZerosVars2, LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem, KnitroSolverParameters knitroParameters) {
                this.oldMatrix = oldMatrix;
                this.listNonZerosCtsDense = listNonZerosCts;
                this.listNonZerosVarsDense = listNonZerosVars;
                this.listNonZerosCtsSparse = listNonZerosCts2;
                this.listNonZerosVarsSparse = listNonZerosVars2;
                this.network = network;
                this.equationSystem = equationSystem;
                this.knitroParameters = knitroParameters;
            }

            @Override
            public void evaluateGA(final List<Double> x, final List<Double> objGrad, final List<Double> jac) {
                // Update current Jacobian
                equationSystem.getStateVector().set(toArray(x));
                AcSolverUtil.updateNetwork(network, equationSystem);
                oldMatrix.forceUpdate();

                // For sparse matrix, get values, row and column structure
                SparseMatrix sparseOldMatrix = oldMatrix.getMatrix().toSparse();
                int[] columnStart = sparseOldMatrix.getColumnStart();
                int[] rowIndices = sparseOldMatrix.getRowIndices();
                double[] values = sparseOldMatrix.getValues();

                // Number of constraints evaluated in callback
                int numCbCts = 0;
                if (knitroParameters.getGradientUserRoutine() == 1) {
                    numCbCts = listNonZerosCtsDense.size();
                } else if (knitroParameters.getGradientUserRoutine() == 2) {
                    numCbCts = listNonZerosCtsSparse.size();
                }

                // Pass coefficients of Jacobian matrix to Knitro
                for (int index = 0; index < numCbCts; index++) {
                    try {
                        int variable = 0;
                        int ct = 0;
                        if (knitroParameters.getGradientUserRoutine() == 1) {
                            variable = listNonZerosVarsDense.get(index);
                            ct = listNonZerosCtsDense.get(index);
                        } else if (knitroParameters.getGradientUserRoutine() == 2) {
                            variable = listNonZerosVarsSparse.get(index);
                            ct = listNonZerosCtsSparse.get(index);
                        }

                        // Start and end index in the values array for column ct
                        int colStart = columnStart[ct];
                        int colEnd = columnStart[ct + 1];
                        double valueSparse = 0.0;

                        // Iterate through the column range
                        for (int i = colStart; i < colEnd; i++) {
                            // Check if the row index matches var
                            if (rowIndices[i] == variable) {
                                // Get the corresponding value
                                valueSparse = values[i];
                                break;  // Exit loop since the value is found
                            }
                        }
                        jac.set(index, valueSparse);
                    } catch (Exception e) {
                        LOGGER.error(
                                "Exception found while trying to add Jacobian term {} in non-linear constraint n° {}",
                                listNonZerosVarsSparse.get(index),
                                listNonZerosCtsSparse.get(index)
                        );
                        LOGGER.error(e.getMessage());
                    }
                }
            }
        }

        /**
         * Callback function to evaluate the Hessian Matrix
         */
        private static final class CallbackEvalH extends KNEvalHCallback {
            @Override
            public void evaluateH(final List<Double> x, final double sigma, final List<Double> lambda, List<Double> hess) {
                // TODO : add Hessian matrix to improve performances
            }
        }

        /**
         * Knitro Problem definition with :
         * - initialization of variables (types, bounds, initial state)
         * - definition of linear constraints
         * - definition of non-linear constraints, evaluated in the callback function
         * - definition of the Jacobian matrix passed to Knitro to solve the problem
         */
        private KnitroProblem(LfNetwork lfNetwork, EquationSystem<AcVariableType, AcEquationType> equationSystem, TargetVector<AcVariableType, AcEquationType> targetVector, VoltageInitializer voltageInitializer, JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix) throws KNException {

            // =============== Variables ===============
            // Defining variables
            super(equationSystem.getIndex().getSortedVariablesToFind().size(), // only active variables
                    equationSystem.getIndex().getSortedEquationsToSolve().size()); // only active constraints
            int numVar = equationSystem.getIndex().getSortedVariablesToFind().size();
            List<Variable<AcVariableType>> sortedVariables = equationSystem.getIndex().getSortedVariablesToFind(); // ordering variables
            LOGGER.info("Defining {} variables", numVar);

            // Types, bounds and initial states of variables
            // Types
            List<Integer> listVarTypes = new ArrayList<>(Collections.nCopies(numVar, KNConstants.KN_VARTYPE_CONTINUOUS));
            setVarTypes(listVarTypes);

            // Bounds
            VariableBounds variableBounds = new VariableBounds(sortedVariables);
            setVarLoBnds(variableBounds.getLowerBounds());
            setVarUpBnds(variableBounds.getUpperBounds());

            // Initial state
            StateInitializer stateInitializer = new StateInitializer(lfNetwork, equationSystem, voltageInitializer);
            setXInitial(stateInitializer.getInitialState());
            LOGGER.info("Initialization of variables : type of initialization {}", voltageInitializer);

            // =============== Constraints ==============
            // ----- Active constraints -----
            List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve = equationSystem.getIndex().getSortedEquationsToSolve();
            int numConst = sortedEquationsToSolve.size();
            List<Integer> listNonLinearConsts = new ArrayList<>(); // list of indexes of non-linear constraints
            LOGGER.info("Defining {} active constraints", numConst);

            // ----- Linear constraints -----
            NonLinearExternalSolverUtils solverUtils = new NonLinearExternalSolverUtils();
            addLinearConstraints(sortedEquationsToSolve, solverUtils, listNonLinearConsts); // add linear constraints and fill the list of non-linear constraints

            // ----- Non-linear constraints -----
            // Callback
            // Pass to Knitro the indexes of non-linear constraints, that will be evaluated in the callback function
            setMainCallbackCstIndexes(listNonLinearConsts);

            // ----- RHS : targets -----
            setConEqBnds(Arrays.stream(targetVector.getArray()).boxed().toList());

            // =============== Objective ==============
            setObjConstPart(0.0);

            // =============== Callback ==============
            // ----- Constraints -----
            setObjEvalCallback(new CallbackEvalFC(sortedEquationsToSolve, listNonLinearConsts));

            // ----- Jacobian matrix -----
            // Non-zero pattern : for each constraint, we detail the variables of which the constraint is a function of.
            List<Integer> listNonZerosCtsDense = new ArrayList<>(); // for the dense method, list of constraints to pass to Knitro's non-zero pattern
            List<Integer> listNonZerosVarsDense = new ArrayList<>(); // for the dense method, list of variables to pass to Knitro's non-zero pattern
            List<Integer> listNonZerosCtsSparse = new ArrayList<>();
            List<Integer> listNonZerosVarsSparse = new ArrayList<>();

            setJacobianMatrix(lfNetwork, jacobianMatrix, sortedEquationsToSolve, listNonLinearConsts,
                    listNonZerosCtsDense, listNonZerosVarsDense,
                    listNonZerosCtsSparse, listNonZerosVarsSparse);
        }

        /**
         * Adds constraints to the Knitro problem, classifying them as linear or non-linear.
         *
         * @param sortedEquationsToSolve A list of equations to solve.
         * @param solverUtils Utils for solving external non-linear equations.
         * @param listNonLinearConsts A list to store ids of non-linear constraints.
         */
        private void addLinearConstraints(List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                                          NonLinearExternalSolverUtils solverUtils, List<Integer> listNonLinearConsts) {

            int numConst = sortedEquationsToSolve.size();

            for (int equationId = 0; equationId < numConst; equationId++) {
                addConstraint(equationId, sortedEquationsToSolve, solverUtils, listNonLinearConsts);
            }
        }

        /**
         * Adds a specific constraint to the Knitro problem, classifying it as linear or non-linear.
         *
         * @param equationId The id of the equation.
         * @param sortedEquationsToSolve A list of equations to solve.
         * @param solverUtils Utils for solving external non-linear equations.
         * @param listNonLinearConsts A list to store ids of non-linear constraints.
         */
        private void addConstraint(int equationId, List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve, NonLinearExternalSolverUtils solverUtils, List<Integer> listNonLinearConsts) {
            Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(equationId);
            AcEquationType typeEq = equation.getType();
            List<EquationTerm<AcVariableType, AcEquationType>> terms = equation.getTerms();

            if (NonLinearExternalSolverUtils.isLinear(typeEq, terms)) {
                try {
                    List<Integer> listVar = solverUtils.getLinearConstraint(typeEq, terms).listIdVar();

                    List<Double> listCoef = solverUtils.getLinearConstraint(typeEq, terms).listCoef();

                    for (int i = 0; i < listVar.size(); i++) {
                        this.addConstraintLinearPart(equationId, listVar.get(i), listCoef.get(i));
                    }
                    LOGGER.trace("Adding linear constraint n° {} of type {}", equationId, typeEq);
                } catch (UnsupportedOperationException e) {
                    throw new PowsyblException(e);
                }
            } else {
                // ----- Non-linear constraints -----
                listNonLinearConsts.add(equationId); // Add constraint number to list of non-linear constraints
            }
        }

        /**
         * Configures the Jacobian matrix for the Knitro problem, using either a dense or sparse representation.
         *
         * @param lfNetwork The PowSyBl network.
         * @param jacobianMatrix The PowSyBl Jacobian matrix.
         * @param sortedEquationsToSolve The list of equations to solve.
         * @param listNonLinearConsts The list of non-linear constraint ids.
         * @param listNonZerosCtsDense Dense non-zero constraints.
         * @param listNonZerosVarsDense Dense non-zero variables.
         * @param listNonZerosCtsSparse Sparse non-zero constraints.
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
                    KnitroSolverUtils.buildDenseJacobianMatrix(numVar, listNonLinearConsts, listNonZerosCtsDense, listNonZerosVarsDense);
                    this.setJacNnzPattern(listNonZerosCtsDense, listNonZerosVarsDense);
                } else if (knitroParameters.getGradientUserRoutine() == 2) {
                    // Sparse method: compute Jacobian only for variables the constraints depend on.
                    buildSparseJacobianMatrix(sortedEquationsToSolve, listNonLinearConsts, listNonZerosCtsSparse, listNonZerosVarsSparse);
                    this.setJacNnzPattern(listNonZerosCtsSparse, listNonZerosVarsSparse);
                }
                // Set the callback for gradient evaluations if the user directly passes the Jacobian to the solver.
                this.setGradEvalCallback(new KnitroProblem.CallbackEvalG(
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

    private void setSolverParameters(KNSolver solver) throws KNException {

        solver.setParam(KNConstants.KN_PARAM_GRADOPT, knitroParameters.getGradientComputationMode());
        solver.setParam(KNConstants.KN_PARAM_FEASTOL, knitroParameters.getRelConvEps());
        solver.setParam(KNConstants.KN_PARAM_MAXIT, knitroParameters.getMaxIterations());
    }

    @Override
    public AcSolverResult run(VoltageInitializer voltageInitializer, ReportNode reportNode) {
        int nbIter;
        AcSolverStatus acStatus;
        KnitroProblem instance;

        try {
            // Create a new Knitro problem instance
            instance = new KnitroProblem(network, equationSystem, targetVector, voltageInitializer, j);
        } catch (KNException e) {
            throw new PowsyblException("Exception while trying to build Knitro Problem", e);
        }

        try (KNSolver solver = new KNSolver(instance)) {
            // Initialize problem
            solver.initProblem();

            // Set solver parameters
            setSolverParameters(solver);

            // Solve
            solver.solve();
            KNSolution solution = solver.getSolution();
            List<Double> constraintValues = solver.getConstraintValues();
            acStatus = KnitroStatus.fromStatusCode(solution.getStatus()).toAcSolverStatus();
            KnitroSolverUtils.logKnitroStatus(KnitroStatus.fromStatusCode(solution.getStatus()));
            nbIter = solver.getNumberIters();

            // Log solution
            LOGGER.info("Optimal objective value  = {}", solution.getObjValue());
            LOGGER.info("Feasibility violation    = {}", solver.getAbsFeasError());
            LOGGER.info("Optimality violation     = {}", solver.getAbsOptError());

            LOGGER.debug("Optimal x");
            for (int i = 0; i < solution.getX().size(); i++) {
                LOGGER.debug(" x[{}] = {}", i, solution.getX().get(i));
            }
            LOGGER.debug("Optimal constraint values (with corresponding multiplier)");
            for (int i = 0; i < instance.getNumCons(); i++) {
                LOGGER.debug(" c[{}] = {} (lambda = {} )", i, constraintValues.get(i), solution.getLambda().get(i));
            }
            LOGGER.debug("Constraint violation");
            for (int i = 0; i < instance.getNumCons(); i++) {
                LOGGER.debug(" violation[{}] = {} ", i, solver.getConViol(i));
            }

            // Update the network if required
            if (acStatus == AcSolverStatus.CONVERGED || knitroParameters.isAlwaysUpdateNetwork()) {
                equationSystem.getStateVector().set(toArray(solution.getX())); // update equation system
                for (Equation<AcVariableType, AcEquationType> equation : equationSystem.getEquations()) { // update terms
                    for (EquationTerm<AcVariableType, AcEquationType> term : equation.getTerms()) {
                        term.setStateVector(equationSystem.getStateVector());
                    }
                }
                AcSolverUtil.updateNetwork(network, equationSystem); // update network
            }

        } catch (KNException e) {
            throw new PowsyblException("Exception while trying to solve with Knitro", e);
        }

        double slackBusActivePowerMismatch = network.getSlackBuses().stream().mapToDouble(LfBus::getMismatchP).sum();
        return new AcSolverResult(acStatus, nbIter, slackBusActivePowerMismatch);
    }

}

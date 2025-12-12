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
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.ac.solver.AcSolverUtil;
import com.powsybl.openloadflow.equations.*;
import com.powsybl.openloadflow.network.LfNetwork;
import com.powsybl.openloadflow.network.util.VoltageInitializer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.IntStream;

/**
 * Abstract base class for a Knitro optimization problem designed to solve the open load-flow equation system.
 * It provides common functionality, including:
 *  - Initialization and definition of variable bounds for the optimization problem.
 *  - Definition of constraints (including those evaluated via callbacks in {@link KnitroCallbacks}).
 *  - Representation of the constraint Jacobian for the problem.
 * This class can be extended to customize any of these features (e.g., in {@link RelaxedKnitroSolver.RelaxedKnitroProblem}).
 * For example, if you modify the optimization problem, you may also need to update the initialization of additional variables.
 *
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 */
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

    protected AbstractKnitroProblem(LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                    TargetVector<AcVariableType, AcEquationType> targetVector, JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                    KnitroSolverParameters knitroParameters, int numTotalVariables) {
        super(numTotalVariables, equationSystem.getIndex().getSortedEquationsToSolve().size());
        this.network = network;
        this.equationSystem = equationSystem;
        this.targetVector = targetVector;
        this.jacobianMatrix = jacobianMatrix;
        this.knitroParameters = knitroParameters;
        this.numberOfPowerFlowVariables = equationSystem.getIndex().getSortedVariablesToFind().size();
    }

    /**
     * Initializes variables (types, bounds, initial values).
     * Can be overridden by subclasses to add additional variables (e.g., slack variables).
     *
     * @param voltageInitializer The voltage initializer to use.
     * @param numTotalVariables  Total number of variables including any additional ones.
     */
    protected void initializeVariables(VoltageInitializer voltageInitializer, int numTotalVariables) throws KNException {
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
        initializeCustomizedVariables(lowerBounds, upperBounds, initialValues, numTotalVariables);

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
     * @param numTotalVariables Total number of variables.
     */
    protected void initializeCustomizedVariables(List<Double> lowerBounds, List<Double> upperBounds,
                                                 List<Double> initialValues, int numTotalVariables) {
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
        addLinearConstraints(activeConstraints, solverUtils, nonlinearConstraintIndexes);

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
     * @param nonLinearConstraintIds Output list of indices of non-linear constraints.
     */
    protected void addLinearConstraints(List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
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
     * @param equationId Index of the equation in the list.
     * @param sortedEquationsToSolve List of all equations to solve.
     * @param solverUtils Utilities to extract linear constraint components.
     * @param nonLinearConstraintIds Output list of non-linear constraint indices.
     */
    protected void addConstraint(int equationId, List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                                 NonLinearExternalSolverUtils solverUtils, List<Integer> nonLinearConstraintIds) {

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
            nonLinearConstraintIds.add(equationId);
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


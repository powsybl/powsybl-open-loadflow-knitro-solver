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

/**
 * Abstract base class for Knitro problem definitions, providing common functionality.
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
    protected final int numLFVariables;

    protected AbstractKnitroProblem(LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                    TargetVector<AcVariableType, AcEquationType> targetVector, JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                    KnitroSolverParameters knitroParameters, int numTotalVariables) {
        super(numTotalVariables, equationSystem.getIndex().getSortedEquationsToSolve().size());
        this.network = network;
        this.equationSystem = equationSystem;
        this.targetVector = targetVector;
        this.jacobianMatrix = jacobianMatrix;
        this.knitroParameters = knitroParameters;
        this.numLFVariables = equationSystem.getIndex().getSortedVariablesToFind().size();
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
        for (int i = 0; i < numLFVariables; i++) {
            initialValues.set(i, equationSystem.getStateVector().get(i));
        }

        // Set bounds for voltage variables based on Knitro parameters
        List<Variable<AcVariableType>> sortedVariables = equationSystem.getIndex().getSortedVariablesToFind();
        for (int i = 0; i < numLFVariables; i++) {
            if (sortedVariables.get(i).getType() == AcVariableType.BUS_V) {
                lowerBounds.set(i, knitroParameters.getLowerVoltageBound());
                upperBounds.set(i, knitroParameters.getUpperVoltageBound());
            }
        }

        // Allow subclasses to modify bounds and initial values (e.g., for slack variables)
        customizeVariableBounds(lowerBounds, upperBounds, initialValues, numTotalVariables);

        setVarLoBnds(lowerBounds);
        setVarUpBnds(upperBounds);
        setXInitial(initialValues);
        LOGGER.info("Variables initialization complete!");
    }

    /**
     * Allows subclasses to customize variable bounds and initial values.
     * Default implementation does nothing.
     *
     * @param lowerBounds Lower bounds list to modify.
     * @param upperBounds Upper bounds list to modify.
     * @param initialValues Initial values list to modify.
     * @param numTotalVariables Total number of variables.
     */
    protected void customizeVariableBounds(List<Double> lowerBounds, List<Double> upperBounds,
                                           List<Double> initialValues, int numTotalVariables) {
        // no customization by default
    }

    /**
     * Sets up constraints (linear and non-linear).
     */
    protected void setupConstraints() throws KNException {
        List<Equation<AcVariableType, AcEquationType>> activeConstraints = equationSystem.getIndex().getSortedEquationsToSolve();
        int numConstraints = activeConstraints.size();
        List<Integer> nonlinearConstraintIndexes = new ArrayList<>();

        LOGGER.info("Defining {} active constraints", numConstraints);

        NonLinearExternalSolverUtils solverUtils = new NonLinearExternalSolverUtils();
        addLinearConstraints(activeConstraints, solverUtils, nonlinearConstraintIndexes);
        setMainCallbackCstIndexes(nonlinearConstraintIndexes);
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
     * @param listNonZerosCtsDense Dense non-zero constraints.
     * @param listNonZerosVarsDense Dense non-zero variables.
     * @param listNonZerosCtsSparse Sparse non-zero constraints.
     * @param listNonZerosVarsSparse Sparse non-zero variables.
     */
    protected void setJacobianMatrix(List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve, List<Integer> listNonLinearConsts,
                                     List<Integer> listNonZerosCtsDense, List<Integer> listNonZerosVarsDense,
                                     List<Integer> listNonZerosCtsSparse, List<Integer> listNonZerosVarsSparse) {

        if (knitroParameters.getGradientComputationMode() == 1) { // User routine to compute the Jacobian
            try {
                if (knitroParameters.getGradientUserRoutine() == 1) {
                    // Dense method: all non-linear constraints are considered as a function of all variables.
                    NonLinearExternalSolverUtils.buildDenseJacobianMatrix(numLFVariables, listNonLinearConsts,
                            listNonZerosCtsDense, listNonZerosVarsDense);
                    this.setJacNnzPattern(listNonZerosCtsDense, listNonZerosVarsDense);
                } else if (knitroParameters.getGradientUserRoutine() == 2) {
                    // Sparse method: compute Jacobian only for variables the constraints depend on.
                    buildSparseJacobianMatrix(sortedEquationsToSolve, listNonLinearConsts,
                            listNonZerosCtsSparse, listNonZerosVarsSparse);
                    this.setJacNnzPattern(listNonZerosCtsSparse, listNonZerosVarsSparse);
                }
            } catch (KNException e) {
                throw new RuntimeException("Failed to set Jacobian pattern", e);
            }
            // Set the callback for gradient evaluations if the user directly passes the Jacobian to the solver.
            this.setGradEvalCallback(createGradientCallback(
                    jacobianMatrix,
                    listNonZerosCtsDense,
                    listNonZerosVarsDense,
                    listNonZerosCtsSparse,
                    listNonZerosVarsSparse));
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


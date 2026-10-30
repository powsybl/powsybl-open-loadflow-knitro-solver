/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.artelys.knitro.api.callbacks.KNEvalFCCallback;
import com.artelys.knitro.api.callbacks.KNEvalGACallback;
import com.powsybl.commons.PowsyblException;
import com.powsybl.math.matrix.SparseMatrix;
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.ac.solver.AcSolverUtil;
import com.powsybl.openloadflow.equations.*;
import com.powsybl.openloadflow.network.LfNetwork;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

import static com.google.common.primitives.Doubles.toArray;

/**
 * Common callback implementations for Knitro optimization problems, including:
 *  - {@link BaseCallbackEvalFC} to evaluate the constraints associated with the open load-flow equations.
 *  - {@link BaseCallbackEvalG} to evaluate the gradient (Jacobian matrix) of these constraints.
 * These callbacks are included as part of the definition of a Knitro optimization problem.
 * They can be extended to enrich the evaluation depending on the problem being solved.
 *
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public final class KnitroCallbacks {

    private static final Logger LOGGER = LoggerFactory.getLogger(KnitroCallbacks.class);

    private KnitroCallbacks() {
        // Utility class
    }

    /**
     * Base callback for evaluating non-linear constraints of a Knitro optimization problem.
     * This can be extended to handle specific modification of the constraints (e.g., adding slacks).
     */
    public static class BaseCallbackEvalFC extends KNEvalFCCallback {

        protected final List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve;
        protected final List<Integer> nonLinearConstraintIds;
        protected final EquationSystem<AcVariableType, AcEquationType> equationSystem;
        protected final EquationVector<AcVariableType, AcEquationType> equationVector;

        public BaseCallbackEvalFC(
                List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                List<Integer> nonLinearConstraintIds,
                EquationSystem<AcVariableType, AcEquationType> equationSystem,
                EquationVector<AcVariableType, AcEquationType> equationVector) {
            this.sortedEquationsToSolve = sortedEquationsToSolve;
            this.nonLinearConstraintIds = nonLinearConstraintIds;
            this.equationSystem = equationSystem;
            this.equationVector = equationVector;
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

            // move the equation system to the current point; f(x) (equation left-hand sides, by column) is recomputed
            // lazily from the state vector, transparently whether the equation system is vectorized or not
            equationSystem.getStateVector().set(toArray(x));
            double[] fx = equationVector.getArray();
            LOGGER.trace("Evaluating {} non-linear constraints", nonLinearConstraintIds.size());

            int callbackConstraintIndex = 0;

            for (int equationId : nonLinearConstraintIds) {
                Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(equationId);
                AcEquationType type = equation.getType();

                // the constraint index is the equation column: read the pre-evaluated left-hand side when available,
                // otherwise evaluate the equation (e.g. equations added on top of the open load flow system)
                double constraintValue = equationId < fx.length ? fx[equationId] : equation.eval();

                // Allow subclasses to add slack contributions
                constraintValue += addModificationOfNonLinearConstraints(equationId, type, x);

                try {
                    c.set(callbackConstraintIndex, constraintValue);
                    LOGGER.trace("Added non-linear constraint #{} (type: {}) = {}", equationId, type, constraintValue);
                } catch (Exception e) {
                    throw new PowsyblException("Error while adding non-linear constraint #" + equationId, e);
                }

                callbackConstraintIndex++;
            }
        }

        /**
         * Allows modification of the constraint value.
         * Default implementation returns no modification.
         * Should be overridden by subclasses that modify the callbacks of Jacobian matrix
         * (e.g., to add relaxation variables in {@link AbstractRelaxedKnitroSolver.AbstractRelaxedKnitroProblem}).
         *
         * @param equationId The equation ID.
         * @param equationType The equation type.
         * @param x The current point.
         * @return The constraint value with potential slack contributions.
         */
        protected double addModificationOfNonLinearConstraints(int equationId, AcEquationType equationType, List<Double> x) {
            return 0;
        }
    }

    /**
     * Base callback for evaluating the gradient (Jacobian matrix) non-linear constraints of a Knitro optimization problem.
     * Only constraints (no objective) are handled here.
     * This can be extended to handle specific modification of the constraints (e.g., adding slacks).
     */
    public static class BaseCallbackEvalG extends KNEvalGACallback {

        protected final JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix;
        protected final List<Integer> denseJacEntryConstraints;
        protected final List<Integer> denseJacEntryVariables;
        protected final List<Integer> sparseJacEntryConstraints;
        protected final List<Integer> sparseJacEntryVariables;

        protected final LfNetwork network;
        protected final EquationSystem<AcVariableType, AcEquationType> equationSystem;
        protected final KnitroSolverParameters knitroParameters;
        protected final int numLFVariables;

        public BaseCallbackEvalG(JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                 List<Integer> denseJacEntryConstraints, List<Integer> denseJacEntryVariables,
                                 List<Integer> sparseJacEntryConstraints, List<Integer> sparseJacEntryVariables,
                                 LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                 KnitroSolverParameters knitroParameters, int numLFVariables) {
            this.jacobianMatrix = jacobianMatrix;
            this.denseJacEntryConstraints = denseJacEntryConstraints;
            this.denseJacEntryVariables = denseJacEntryVariables;
            this.sparseJacEntryConstraints = sparseJacEntryConstraints;
            this.sparseJacEntryVariables = sparseJacEntryVariables;
            this.network = network;
            this.equationSystem = equationSystem;
            this.knitroParameters = knitroParameters;
            this.numLFVariables = numLFVariables;
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
            List<Integer> jacEntryConstraints;
            List<Integer> jacEntryVariables;

            int routineType = knitroParameters.getGradientUserRoutine();
            if (routineType == 1) {
                jacEntryConstraints = denseJacEntryConstraints;
                jacEntryVariables = denseJacEntryVariables;
            } else if (routineType == 2) {
                jacEntryConstraints = sparseJacEntryConstraints;
                jacEntryVariables = sparseJacEntryVariables;
            } else {
                throw new IllegalArgumentException("Unsupported gradientUserRoutine value: " + routineType);
            }

            // Fill Jacobian values
            fillJacobianValues(jacEntryConstraints, jacEntryVariables, columnStart, rowIndices, values, jac);
        }

        /**
         * Fills the Jacobian values, in the order of the non-zero pattern given to Knitro.
         * Can be overridden to handle slack variables.
         *
         * @param jacEntryConstraints Constraint index of each non-zero Jacobian entry.
         * @param jacEntryVariables Variable index of each non-zero Jacobian entry.
         * @param columnStart Column start array from sparse matrix.
         * @param rowIndices Row indices array from sparse matrix.
         * @param values Values array from sparse matrix.
         * @param jac Output Jacobian list.
         */
        protected void fillJacobianValues(List<Integer> jacEntryConstraints, List<Integer> jacEntryVariables,
                                          int[] columnStart, int[] rowIndices, double[] values, List<Double> jac) {
            for (int index = 0; index < jacEntryConstraints.size(); index++) {
                int constraintIndex = jacEntryConstraints.get(index);
                int variableIndex = jacEntryVariables.get(index);
                try {
                    double value;

                    // columnStart has one more element than the number of columns (open load flow equations)
                    if (constraintIndex < columnStart.length - 1) {
                        // Check if current variableIndex is a slack variable index
                        if (variableIndex >= numLFVariables) {
                            // In this case, add corresponding value
                            value = computeModifiedJacobianValue(variableIndex);
                        } else {
                            value = getJacobianTerm(columnStart, rowIndices, values, constraintIndex, variableIndex);
                        }
                    } else {
                        value = computeAddedConstraintJacobianValue(variableIndex, constraintIndex);
                    }

                    jac.set(index, value);
                } catch (Exception e) {
                    int varId = jacEntryVariables.get(index);
                    int ctId = jacEntryConstraints.get(index);
                    LOGGER.error("Error while filling Jacobian term at var {} and constraint {}", varId, ctId, e);
                }
            }
        }

        /**
         * Returns the derivative of a constraint (sparse matrix column) with respect to a variable (sparse matrix row),
         * or 0 if the variable is not in the column.
         * Row indices are not sorted within a column (e.g. with a vectorized equation system), so the whole column is
         * scanned, and duplicated entries are summed.
         */
        private static double getJacobianTerm(int[] columnStart, int[] rowIndices, double[] values, int column, int row) {
            double value = 0.0;
            for (int k = columnStart[column]; k < columnStart[column + 1]; k++) {
                if (rowIndices[k] == row) {
                    value += values[k];
                }
            }
            return value;
        }

        /**
         * Computes a modified Jacobian value. Default implementation returns 0.
         * Should be overridden by subclasses that modify the callbacks of Jacobian matrix
         * (e.g., to add relaxation variables in {@link AbstractRelaxedKnitroSolver.AbstractRelaxedKnitroProblem}).
         */
        protected double computeModifiedJacobianValue(int variableIndex) {
            return 0.0;
        }

        /**
         * Computes the value of an added constraint. Default implementation returns 0.
         * Should be overridden by subclasses that modify the callbacks of Jacobian matrix
         * (e.g., to add relaxation variables in {@link UseReactiveLimitsKnitroSolver.UseReactiveLimitsKnitroProblem}).
         */
        protected double computeAddedConstraintJacobianValue(int variableIndex, int constraintIndex) {
            return 0.0;
        }
    }
}


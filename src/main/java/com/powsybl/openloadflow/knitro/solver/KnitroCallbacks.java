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

import java.util.Arrays;
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

        protected final List<SingleEquation<AcVariableType, AcEquationType>> sortedSingleEquationsToSolve;
        protected final List<EquationArray<AcVariableType, AcEquationType>> activeConstraintsEqArray;
        protected final List<Integer> nonLinearConstraintIds;
        protected final List<Integer> nonLinearConstraintColumnId;
        protected final EquationSystem<AcVariableType, AcEquationType> equationSystem;
        protected final EquationVector<AcVariableType, AcEquationType> equationVector;

        public BaseCallbackEvalFC(
                List<SingleEquation<AcVariableType, AcEquationType>> sortedSingleEquationsToSolve,
                List<EquationArray<AcVariableType, AcEquationType>> activeConstraintsEqArray,
                List<Integer> nonLinearConstraintIds,
                List<Integer> nonlinearConstraintColumnId,
                EquationSystem<AcVariableType, AcEquationType> equationSystem,
                EquationVector<AcVariableType, AcEquationType> equationVector) {

            this.sortedSingleEquationsToSolve = sortedSingleEquationsToSolve;
            this.activeConstraintsEqArray = activeConstraintsEqArray;
            this.nonLinearConstraintIds = nonLinearConstraintIds;
            this.nonLinearConstraintColumnId = nonlinearConstraintColumnId;
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
            //  LOGGER.info("============ Knitro evaluating callback function ============");
            StateVector currentState = new StateVector(toArray(x));
            equationSystem.getStateVector().set(toArray(x));
//            StateVector currentState = equationSystem.getStateVector(); // optional local alias
            //  LOGGER.info("Current state vector: {}", currentState.get());
            //        LOGGER.info("Evaluating {} Equation Array ", activeConstraintsEqArray.size());
            int totalElements = nonLinearConstraintColumnId.size();
            //LOGGER.info("Evaluating {} non linear equation", totalElements);
            //LOGGER.info("size de c "+ c.size());
            //LOGGER.info(" # equ lineaire {} ", currentState.get().length - nonLinearConstraintColumnId.size() );
            int startIndex = currentState.get().length - totalElements;
            equationSystem.getStateVector().set(toArray(x));
            double[] lhs = equationVector.getArray();

            for (int k = 0; k < nonLinearConstraintColumnId.size(); k++) {
                int col = nonLinearConstraintColumnId.get(k);
                AcEquationType type = equationSystem.getIndex().getEquationAtColumn(col).getType();
                double constraintValue = addModificationOfNonLinearConstraints(col, type, x);
                //           addModificationOfNonLinearConstraints(col, equationSystem.getIndex().getEquationAtColumn(col).getType(), x );
                try {
                    c.set(k, lhs[col] + constraintValue);
//                    if (type == DISTR_Q) {
//                        LOGGER.info("c[{}] col={} type={} lhs={} modif={} total={}", k, col, type, lhs[col], constraintValue, lhs[col] + constraintValue);
//                    }
///                            c.set(k, lhs[col]);
//                    LOGGER.debug("c[{}] <= lhs[col={}] = {}", k, col, lhs[col]);
//                    LOGGER.debug("equation type " + equationSystem.getIndex().getEquationAtColumn(col));

                } catch (Exception e) {
                    throw new PowsyblException("Error while adding non-linear constraint #" + k, e);
                }
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
        protected final List<Integer> denseConstraintIndices;
        protected final List<Integer> denseVariableIndices;
        protected final List<Integer> sparseConstraintIndices;
        protected final List<Integer> sparseVariableIndices;

        protected final LfNetwork network;
        protected final EquationSystem<AcVariableType, AcEquationType> equationSystem;
        protected final KnitroSolverParameters knitroParameters;
        protected final int numLFVariables;

        public BaseCallbackEvalG(JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                 List<Integer> denseConstraintIndices, List<Integer> denseVariableIndices,
                                 List<Integer> sparseConstraintIndices, List<Integer> sparseVariableIndices,
                                 LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                 KnitroSolverParameters knitroParameters, int numLFVariables) {
            this.jacobianMatrix = jacobianMatrix;
            this.denseConstraintIndices = denseConstraintIndices;
            this.denseVariableIndices = denseVariableIndices;
            this.sparseConstraintIndices = sparseConstraintIndices;
            this.sparseVariableIndices = sparseVariableIndices;
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
            fillJacobianValues(constraintIndices, variableIndices, columnStart, rowIndices, values, jac);
        }

        /**
         * Fills the Jacobian values.
         * Can be overridden to handle slack variables.
         *
         * @param constraintIndices Constraint indices.
         * @param variableIndices Variable indices.
         * @param columnStart Column start array from sparse matrix.
         * @param rowIndices Row indices array from sparse matrix.
         * @param values Values array from sparse matrix.
         * @param jac Output Jacobian list.
         */
        protected void fillJacobianValues(List<Integer> constraintIndices, List<Integer> variableIndices,
                                          int[] columnStart, int[] rowIndices, double[] values, List<Double> jac) {
            int iRowIndices = 0;
            int currentConstraint = -1;
            for (int index = 0; index < constraintIndices.size(); index++) {
                int constraintIndex = constraintIndices.get(index);
                int variableIndex = variableIndices.get(index);
                try {
                    double value;

                    if (constraintIndex < Arrays.stream(columnStart).count()) { // Find matching (variableIndex, constraintIndex) entry in sparse column
                        int colStart = columnStart[constraintIndex];

                        // Check if we moved to the next constraint
                        if (currentConstraint != constraintIndex) {
                            // In this case, restart variable tracking slacks indices to 0 and change the currently handled constraint
                            iRowIndices = 0;
                            currentConstraint = constraintIndex;
                        }

                        // Check if current variableIndex is a slack variable index
                        if (variableIndex >= numLFVariables) {
                            // In this case, add corresponding value
                            value = computeModifiedJacobianValue(variableIndex);
                            // When in dense mode, the constructed index list and the index list from powsybl don't match
                        } else if (rowIndices[colStart + iRowIndices] != variableIndex) {
                            // In this case, set corresponding value to zero
                            value = 0.0;
                        } else {
                            // This is the case of a LF variable with a non-zero coefficient in the Jacobian
                            value = values[colStart + iRowIndices++];
                        }
                    } else {
                        value = computeAddedConstraintJacobianValue(variableIndex, constraintIndex);
                    }

                    jac.set(index, value);
                } catch (Exception e) {
                    int varId = variableIndices.get(index);
                    int ctId = constraintIndices.get(index);
                    LOGGER.error("Error while filling Jacobian term at var {} and constraint {}", varId, ctId, e);
                }
            }
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


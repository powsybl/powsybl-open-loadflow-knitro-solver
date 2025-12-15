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
import com.powsybl.openloadflow.equations.*;
import com.powsybl.openloadflow.network.LfNetwork;
import com.powsybl.openloadflow.network.util.VoltageInitializer;

import java.util.List;

/**
 * Standard Knitro solver, solving open load flow equation system as a feasibility problem.
 *
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 */
public class KnitroSolver extends AbstractKnitroSolver {

    public KnitroSolver(
            LfNetwork network,
            KnitroSolverParameters knitroParameters,
            EquationSystem<AcVariableType, AcEquationType> equationSystem,
            JacobianMatrix<AcVariableType, AcEquationType> j,
            TargetVector<AcVariableType, AcEquationType> targetVector,
            EquationVector<AcVariableType, AcEquationType> equationVector,
            boolean detailedReport) {

        super(network, knitroParameters, equationSystem, j, targetVector, equationVector, detailedReport);
    }

    @Override
    public String getName() {
        return "Knitro Solver";
    }

    @Override
    protected KNProblem createKnitroProblem(VoltageInitializer voltageInitializer) {
        try {
            return new KnitroProblem(network, equationSystem, targetVector, j, voltageInitializer, knitroParameters);
        } catch (KNException e) {
            throw new PowsyblException("Failed to create Knitro problem", e);
        }
    }

    /**
     * Optimization problem modeling the open load-flow equation system as a feasibility problem.
     */
    private static final class KnitroProblem extends AbstractKnitroProblem {

        /**
         * Knitro Problem definition with:
         * - initialization of variables (types, bounds, initial state)
         * - definition of linear constraints
         * - definition of non-linear constraints, evaluated in the callback function
         * - definition of the Jacobian matrix passed to Knitro to solve the problem
         */
        private KnitroProblem(LfNetwork lfNetwork, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                              TargetVector<AcVariableType, AcEquationType> targetVector,
                              JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                              VoltageInitializer voltageInitializer,
                              KnitroSolverParameters parameters) throws KNException {

            super(lfNetwork, equationSystem, targetVector, jacobianMatrix, parameters,
                    equationSystem.getIndex().getSortedVariablesToFind().size(),
                    equationSystem.getIndex().getSortedEquationsToSolve().size());

            LOGGER.info("Defining {} variables", numberOfPowerFlowVariables);

            // Initialize variables
            initializeVariables(voltageInitializer, numberOfPowerFlowVariables);
            LOGGER.info("Initialization of variables : type of initialization {}", voltageInitializer);

            // Set up the constraints of the optimization problem
            setupConstraints();

            // Constant objective (= 0), for feasibility problem
            setObjConstPart(0.0);

            // callbacks of the constraints
            setObjEvalCallback(new KnitroCallbacks.BaseCallbackEvalFC(activeConstraints, nonlinearConstraintIndexes));

            // set the representation of the jacobian matrix (dense or sparse)
            setJacobianMatrix(activeConstraints, nonlinearConstraintIndexes);
        }

        @Override
        protected KNEvalGACallback createGradientCallback(JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                                          List<Integer> listNonZerosCtsDense,
                                                          List<Integer> listNonZerosVarsDense,
                                                          List<Integer> listNonZerosCtsSparse,
                                                          List<Integer> listNonZerosVarsSparse) {

            return new KnitroCallbacks.BaseCallbackEvalG(jacobianMatrix, listNonZerosCtsDense, listNonZerosVarsDense,
                    listNonZerosCtsSparse, listNonZerosVarsSparse, network, equationSystem,
                    knitroParameters, numberOfPowerFlowVariables);
        }
    }

}

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
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.equations.*;
import com.powsybl.openloadflow.network.LfNetwork;
import com.powsybl.openloadflow.network.util.VoltageInitializer;

import java.util.ArrayList;
import java.util.List;

/**
 * Standard Knitro solver without slack variables.
 *
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 */
public class KnitroSolver extends AbstractKnitroSolver {

    public KnitroSolver(LfNetwork network, KnitroSolverParameters knitroParameters, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                        JacobianMatrix<AcVariableType, AcEquationType> j, TargetVector<AcVariableType, AcEquationType> targetVector,
                        EquationVector<AcVariableType, AcEquationType> equationVector, boolean detailedReport) {

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
            throw new RuntimeException("Failed to create Knitro problem", e);
        }
    }

    private final class KnitroProblem extends AbstractKnitroProblem {

        /**
         * Knitro Problem definition for standard solver (no slack variables).
         */
        private KnitroProblem(LfNetwork lfNetwork, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                              TargetVector<AcVariableType, AcEquationType> targetVector,
                              JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                              VoltageInitializer voltageInitializer,
                              KnitroSolverParameters parameters) throws KNException {

            super(lfNetwork, equationSystem, targetVector, jacobianMatrix, parameters,
                    equationSystem.getIndex().getSortedVariablesToFind().size());

            LOGGER.info("Defining {} variables", numLFVariables);

            // Initialize variables
            initializeVariables(voltageInitializer, numLFVariables);
            LOGGER.info("Initialization of variables : type of initialization {}", voltageInitializer);

            // Setup constraints
            setupConstraints();

            // Objective (constant = 0)
            setObjConstPart(0.0);

            // Callbacks
            List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve = equationSystem.getIndex().getSortedEquationsToSolve();
            List<Integer> listNonLinearConsts = new ArrayList<>();
            for (int i = 0; i < sortedEquationsToSolve.size(); i++) {
                Equation<AcVariableType, AcEquationType> eq = sortedEquationsToSolve.get(i);
                if (!NonLinearExternalSolverUtils.isLinear(eq.getType(), eq.getTerms())) {
                    listNonLinearConsts.add(i);
                }
            }
            setObjEvalCallback(new KnitroCallbacks.BaseCallbackEvalFC(sortedEquationsToSolve, listNonLinearConsts));

            // Jacobian matrix
            List<Integer> listNonZerosCtsDense = new ArrayList<>();
            List<Integer> listNonZerosVarsDense = new ArrayList<>();
            List<Integer> listNonZerosCtsSparse = new ArrayList<>();
            List<Integer> listNonZerosVarsSparse = new ArrayList<>();
            setJacobianMatrix(sortedEquationsToSolve, listNonLinearConsts, listNonZerosCtsDense, listNonZerosVarsDense,
                    listNonZerosCtsSparse, listNonZerosVarsSparse);
        }

        @Override
        protected KNEvalGACallback createGradientCallback(JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                                          List<Integer> listNonZerosCtsDense,
                                                          List<Integer> listNonZerosVarsDense,
                                                          List<Integer> listNonZerosCtsSparse,
                                                          List<Integer> listNonZerosVarsSparse) {

            return new KnitroCallbacks.BaseCallbackEvalG(jacobianMatrix, listNonZerosCtsDense, listNonZerosVarsDense,
                    listNonZerosCtsSparse, listNonZerosVarsSparse, network, equationSystem,
                    knitroParameters, numLFVariables);
        }
    }

}

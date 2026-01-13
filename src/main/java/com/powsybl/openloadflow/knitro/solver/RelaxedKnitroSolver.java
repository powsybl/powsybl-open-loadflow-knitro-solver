/**
 * Copyright (c) 2025, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.artelys.knitro.api.KNException;
import com.artelys.knitro.api.KNProblem;
import com.powsybl.commons.PowsyblException;
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.equations.EquationSystem;
import com.powsybl.openloadflow.equations.EquationVector;
import com.powsybl.openloadflow.equations.JacobianMatrix;
import com.powsybl.openloadflow.equations.TargetVector;
import com.powsybl.openloadflow.network.LfNetwork;
import com.powsybl.openloadflow.network.util.VoltageInitializer;

import java.util.AbstractMap;
import java.util.List;

/**
 * TODO
 *
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public class RelaxedKnitroSolver extends AbstractRelaxedKnitroSolver {

    public RelaxedKnitroSolver(
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
        return "Relaxed Knitro Solver";
    }

    @Override
    protected KNProblem createKnitroProblem(VoltageInitializer voltageInitializer) {
        try {
            return new RelaxedKnitroProblem(network, equationSystem, targetVector, j, knitroParameters, voltageInitializer);
        } catch (KNException e) {
            throw new PowsyblException("Failed to create relaxed Knitro problem", e);
        }
    }

    public final class RelaxedKnitroProblem extends AbstractRelaxedKnitroProblem {

        private RelaxedKnitroProblem(LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                     TargetVector<AcVariableType, AcEquationType> targetVector, JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                     KnitroSolverParameters knitroParameters, VoltageInitializer voltageInitializer) throws KNException {

            super(network, equationSystem, targetVector, jacobianMatrix, knitroParameters, numSlackVariables, 0);

            LOGGER.info("Defining {} variables", numTotalVariables);

            // Initialize variables (base class handles LF variables, we customize for slack)
            initializeVariables(voltageInitializer);
            LOGGER.info("Voltage initialization strategy: {}", voltageInitializer);

            // Set up the constraints of the relaxed optimization problem
            setupConstraints();

            // callbacks of the constraints
            setObjEvalCallback(new RelaxedCallbackEvalFC(this, activeConstraints, nonlinearConstraintIndexes));

            // set the representation of the Jacobian matrix (dense or sparse)
            setJacobianMatrix(activeConstraints, nonlinearConstraintIndexes);

            // set the representation of the Hessian matrix
            AbstractMap.SimpleEntry<List<Integer>, List<Integer>> hessNnz = getHessNnzRowsAndCols(nonlinearConstraintIndexes);
            setHessNnzPattern(hessNnz.getKey(), hessNnz.getValue());

            // set the objective function of the optimization problem
            addObjectiveFunction(numPEquations, slackPStartIndex, numQEquations, slackQStartIndex, numVEquations, slackVStartIndex);
        }
    }
}

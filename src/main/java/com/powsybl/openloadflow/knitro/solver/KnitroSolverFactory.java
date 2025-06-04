/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.google.auto.service.AutoService;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.ac.AcLoadFlowParameters;
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.ac.solver.AcSolver;
import com.powsybl.openloadflow.ac.solver.AcSolverFactory;
import com.powsybl.openloadflow.ac.solver.AcSolverParameters;
import com.powsybl.openloadflow.equations.EquationSystem;
import com.powsybl.openloadflow.equations.EquationVector;
import com.powsybl.openloadflow.equations.JacobianMatrix;
import com.powsybl.openloadflow.equations.TargetVector;
import com.powsybl.openloadflow.network.LfNetwork;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
@AutoService(AcSolverFactory.class)
public class KnitroSolverFactory implements AcSolverFactory {

    public static final String NAME = "KNITRO";

    @Override
    public String getName() {
        return NAME;
    }

    @Override
    public AcSolverParameters createParameters(LoadFlowParameters parameters) {
        OpenLoadFlowParameters parametersExt = OpenLoadFlowParameters.get(parameters);
        KnitroSolverParameters knitroSolverParameters = new KnitroSolverParameters()
                .setStateVectorScalingMode(parametersExt.getStateVectorScalingMode())
                .setLineSearchStateVectorScalingMaxIteration(parametersExt.getLineSearchStateVectorScalingMaxIteration())
                .setLineSearchStateVectorScalingStepFold(parametersExt.getLineSearchStateVectorScalingStepFold())
                .setMaxVoltageChangeStateVectorScalingMaxDv(parametersExt.getMaxVoltageChangeStateVectorScalingMaxDv())
                .setMaxVoltageChangeStateVectorScalingMaxDphi(parametersExt.getMaxVoltageChangeStateVectorScalingMaxDphi())
                .setAlwaysUpdateNetwork(parametersExt.isAlwaysUpdateNetwork());
        if (parameters.getExtension(KnitroLoadFlowParameters.class) != null) {
            knitroSolverParameters
                    .setGradientComputationMode(parameters.getExtension(KnitroLoadFlowParameters.class).getGradientComputationMode())
                    .setGradientUserRoutine(parameters.getExtension(KnitroLoadFlowParameters.class).getGradientUserRoutine())
                    .setHessianComputationMode(parameters.getExtension(KnitroLoadFlowParameters.class).getHessianComputationMode())
                    .setLowerVoltageBound(parameters.getExtension(KnitroLoadFlowParameters.class).getLowerVoltageBound())
                    .setUpperVoltageBound(parameters.getExtension(KnitroLoadFlowParameters.class).getUpperVoltageBound())
                    .setMaxIterations(parameters.getExtension(KnitroLoadFlowParameters.class).getMaxIterations())
                    .setConvEps(parameters.getExtension(KnitroLoadFlowParameters.class).getConvEps())
                    .setAlwaysUpdateNetwork(parameters.getExtension(KnitroLoadFlowParameters.class).isAlwaysUpdateNetwork())
                    .setCheckLoadFlowSolution(parameters.getExtension(KnitroLoadFlowParameters.class).isCheckLoadFlowSolution())
                    .setKnitroSolverType(parameters.getExtension(KnitroLoadFlowParameters.class).getKnitroSolverType());

        }
        return knitroSolverParameters;
    }

    @Override
    public AcSolver create(LfNetwork network, AcLoadFlowParameters parameters, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                           JacobianMatrix<AcVariableType, AcEquationType> j, TargetVector<AcVariableType, AcEquationType> targetVector,
                           EquationVector<AcVariableType, AcEquationType> equationVector) {
        KnitroSolverParameters knitroSolverParameters = (KnitroSolverParameters) parameters.getAcSolverParameters();
        KnitroSolverParameters.KnitroSolverType knitroSolverType = knitroSolverParameters.getKnitroSolverType();
        return switch (knitroSolverType) {
            case STANDARD ->
                    new KnitroSolver(network, knitroSolverParameters, equationSystem, j, targetVector, equationVector, parameters.isDetailedReport());
            case RESILIENT ->
                    new ResilientKnitroSolver(network, knitroSolverParameters, equationSystem, j, targetVector, equationVector, parameters.isDetailedReport());
            case REACTIVLIMITS ->
                    new KnitroSolverReacLim(network, knitroSolverParameters, equationSystem, j, targetVector, equationVector, parameters.isDetailedReport());
        };
    }

}

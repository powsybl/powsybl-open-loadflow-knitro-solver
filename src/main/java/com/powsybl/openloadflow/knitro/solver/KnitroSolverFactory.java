/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.google.auto.service.AutoService;
import com.powsybl.commons.PowsyblException;
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
            KnitroLoadFlowParameters knitroLoadFlowParameters = parameters.getExtension(KnitroLoadFlowParameters.class);
            knitroSolverParameters
                .setGradientComputationMode(knitroLoadFlowParameters.getGradientComputationMode())
                .setGradientUserRoutine(knitroLoadFlowParameters.getGradientUserRoutine())
                .setHessianComputationMode(knitroLoadFlowParameters.getHessianComputationMode())
                .setLowerVoltageBound(knitroLoadFlowParameters.getLowerVoltageBound())
                .setUpperVoltageBound(knitroLoadFlowParameters.getUpperVoltageBound())
                .setMaxIterations(knitroLoadFlowParameters.getMaxIterations())
                .setRelConvEps(knitroLoadFlowParameters.getRelConvEps())
                .setAbsConvEps(knitroLoadFlowParameters.getAbsConvEps())
                .setRelOptEps(knitroLoadFlowParameters.getRelOptEps())
                .setAbsOptEps(knitroLoadFlowParameters.getAbsOptEps())
                .setSlackThreshold(knitroLoadFlowParameters.getSlackThreshold())
                .setSolverType(knitroLoadFlowParameters.getKnitroSolverType())
                .setThreadNumber(knitroLoadFlowParameters.getThreadNumber());

        }
        return knitroSolverParameters;
    }

    @Override
    public AcSolver create(LfNetwork network, AcLoadFlowParameters parameters, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                           JacobianMatrix<AcVariableType, AcEquationType> j, TargetVector<AcVariableType, AcEquationType> targetVector,
                           EquationVector<AcVariableType, AcEquationType> equationVector) {
        KnitroSolverParameters knitroSolverParameters = (KnitroSolverParameters) parameters.getAcSolverParameters();
        KnitroSolverParameters.SolverType knitroSolverType = knitroSolverParameters.getSolverType();
        return switch (knitroSolverType) {
            case STANDARD -> new KnitroSolver(network, knitroSolverParameters, equationSystem, j, targetVector, equationVector, parameters.isDetailedReport());
            case RELAXED -> new RelaxedKnitroSolver(network, knitroSolverParameters, equationSystem, j, targetVector, equationVector, parameters.isDetailedReport());
            case USE_REACTIVE_LIMITS -> new UseReactiveLimitsKnitroSolver(network, knitroSolverParameters, equationSystem, j, targetVector, equationVector, parameters.isDetailedReport());
        };
    }

    @Override
    public void checkSolverAndParameterConsistency(LoadFlowParameters loadFlowParameters, OpenLoadFlowParameters openLoadFlowParameters) {
        KnitroLoadFlowParameters knitroLoadFlowParameters = loadFlowParameters.getExtension(KnitroLoadFlowParameters.class);
        if (knitroLoadFlowParameters != null && knitroLoadFlowParameters.getKnitroSolverType() == KnitroSolverParameters.SolverType.USE_REACTIVE_LIMITS) {
            // since reactive limits are taken into account in the Knitro solver, there is no need to activate the outer loop
            if (loadFlowParameters.isUseReactiveLimits()) {
                throw new PowsyblException("Knitro generator reactive limits and reactive limits outer loop cannot work simultaneously: useReactiveLimits LoadFlowParameter should be switched to false");
            }

            // exact dense Jacobian computation mode is not supported by Knitro generator reactive limits solver
            if (knitroLoadFlowParameters.getGradientComputationMode() == 1 && knitroLoadFlowParameters.getGradientUserRoutine() == 1) {
                throw new PowsyblException("Knitro generator reactive limits is incompatible with exact dense jacobian computation mode: gradientUserRoutine KnitroLoadFlowParameters should be switched to 1");
            }
        }
    }
}

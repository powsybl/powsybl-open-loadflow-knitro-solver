/**
 * Copyright (c) 2025, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.ieeecdf.converter.IeeeCdfNetworkFactory;
import com.powsybl.iidm.network.*;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.DenseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.network.SlackBusSelectionMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
class RelaxedKnitroSolverConvergenceTest {
    private Network network;

    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;
    private KnitroLoadFlowParameters knitroParams;

    @BeforeEach
    void setUp() {
        network = IeeeCdfNetworkFactory.create14();

        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new DenseMatrixFactory()));
        parameters = new LoadFlowParameters()
                .setUseReactiveLimits(false)
                .setDistributedSlack(false);

        OpenLoadFlowParameters.create(parameters)
                .setSlackBusSelectionMode(SlackBusSelectionMode.MOST_MESHED)
                .setAcSolverType(KnitroSolverFactory.NAME);

        knitroParams = new KnitroLoadFlowParameters();
        knitroParams.setKnitroSolverType(KnitroSolverParameters.SolverType.RELAXED);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);

        // Create a short circuit situation on IEEE14 network between buses B1 and B5

        //      Bus: B1                              Bus: B5                                 Bus: B2
        // (generator: B1-G) <--- line: L1-5-1 ---> ( empty ) <------ line ------> (generator: B2-G controls B5)

        // Line to be made low impedant
        Line l15 = network.getLine("L1-5-1");

        // Acquire regulating terminal
        Optional<? extends Terminal> t5Optional = l15.getTerminals()
                .stream()
                .filter(terminal -> terminal.getBusBreakerView()
                        .getBus()
                        .getId()
                        .equals("B5"))
                .findAny();
        assertFalse(t5Optional.isEmpty(), "Regulating bus' terminal not found");
        Terminal t5 = t5Optional.get();

        // Apply voltage mismatch and remote control to regulating generator
        network.getGenerator("B2-G").setTargetV(131.0429)
                .setVoltageRegulatorOn(true)
                .setRegulatingTerminal(t5);

        // Change line parameters to make it have low reactance
        l15.setX(0.001742)
                .setR(0.0)
                .setG1(0.0)
                .setB1(0.0)
                .setG2(0.0)
                .setB2(0.0);
    }

    @Test
    void testConvergenceOnI3E14() {
        network = IeeeCdfNetworkFactory.create14();

        // Convergence in sparse case with Jacobian callback
        LoadFlowResult resultRKN = loadFlowRunner.run(network, parameters);
        boolean isConvergedRKN = resultRKN.isFullyConverged();
        assertTrue(isConvergedRKN, "RKN should converge");

        // Convergence in dense case with Jacobian callback
        knitroParams.setGradientUserRoutine(1);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
        resultRKN = loadFlowRunner.run(network, parameters);
        isConvergedRKN = resultRKN.isFullyConverged();
        assertTrue(isConvergedRKN, "RKN should converge");

        // Convergence in dense user routine with finite differences
        knitroParams.setGradientUserRoutine(1);
        knitroParams.setGradientComputationMode(2);
        knitroParams.setThreadNumber(1);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
        resultRKN = loadFlowRunner.run(network, parameters);
        isConvergedRKN = resultRKN.isFullyConverged();
        assertTrue(isConvergedRKN, "RKN should converge");
    }

    @Test
    @Disabled("FIXME: windows crash on KN15 on GH")
    void testConvergenceAfterVoltagePerturbationOnI3E14() {
        LoadFlowResult resultRKN = loadFlowRunner.run(network, parameters);
        boolean isConvergedRKN = resultRKN.isFullyConverged();
        assertTrue(isConvergedRKN, "RKN should converge");
    }

    @Test
    @Disabled("FIXME: windows crash on KN15 on GH")
    void testConvergenceWithOuterLoopsOnI3E14() {
        parameters.setUseReactiveLimits(true)
                .setDistributedSlack(true);
        LoadFlowResult resultRKN = loadFlowRunner.run(network, parameters);
        boolean isConvergedRKN = resultRKN.isFullyConverged();
        assertTrue(isConvergedRKN, "RKN should converge");
    }
}

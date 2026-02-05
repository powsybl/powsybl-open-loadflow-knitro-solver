/**
 * Copyright (c) 2026, Artelys (http://www.artelys.com/)
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
import com.powsybl.openloadflow.ac.solver.NewtonRaphsonFactory;
import com.powsybl.openloadflow.network.SlackBusSelectionMode;
import org.junit.jupiter.api.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
class DemoTest {

    private static final Logger LOGGER = LoggerFactory.getLogger(DemoTest.class);

    @Test
    void testConvergenceOnI314WithVoltagePerturbation() {
        // Create IEEE-14 network
        Network network = IeeeCdfNetworkFactory.create14();

        // Induce the following voltage perturbation on the network :

        //      Bus: B1                              Bus: B5                                 Bus: B2
        // (generator: B1-G) <--- line: L1-5-1 ---> ( empty ) <------ line ------> (generator: B2-G controls B5)
        //
        //  target_V = 143.1      X = 0.0017 Ohm                                       target_V = 131.0429 kV

        // Acquire line L1-5-1 to be made low impedant
        Line l15 = network.getLine("L1-5-1");

        // Change line L1-5-1 parameters to make it have low reactance
        l15.setX(0.001742)   // Initial value : 40.64904 Ohm
                .setR(0.0)   // Initial value : 9.8469675 Ohm
                .setG1(0.0)  // Initial value : 0.0
                .setB1(0.0)  // Initial value : 0.0001349 S
                .setG2(0.0)  // Initial value : 0.0
                .setB2(0.0); // Initial value : 0.0001349 S

        // Acquire the terminal object that regulates bus B5
        Terminal t5 = getRegulatingTerminal(l15, "B5");

        // Apply voltage mismatch and remote control to regulating generator
        network.getGenerator("B2-G").setTargetV(131.0429) // Initial value : 141.075 kV
                .setVoltageRegulatorOn(true)
                .setRegulatingTerminal(t5);

        // Create loadFlow runner
        LoadFlow.Runner loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new DenseMatrixFactory()));

        // Deactivate outer loops
        LoadFlowParameters parameters = new LoadFlowParameters()
                .setUseReactiveLimits(false)
                .setDistributedSlack(false);

        // Configure Newton-Raphson solver
        OpenLoadFlowParameters.create(parameters)
                .setSlackBusSelectionMode(SlackBusSelectionMode.MOST_MESHED)
                .setAcSolverType(NewtonRaphsonFactory.NAME);

        // Newton-Raphson diverges (MAX_ITERATIONS_REACHED)
        LoadFlowResult resultNR = loadFlowRunner.run(network, parameters);
        boolean isConvergedNR = resultNR.isFullyConverged();
        assertFalse(isConvergedNR, "NR should not converge");

        // Configure Relaxed Knitro Solver
        OpenLoadFlowParameters.create(parameters)
                .setSlackBusSelectionMode(SlackBusSelectionMode.MOST_MESHED)
                .setAcSolverType(KnitroSolverFactory.NAME);

        // Add Knitro solver specific parameters
        KnitroLoadFlowParameters knitroParams = new KnitroLoadFlowParameters();
        knitroParams.setKnitroSolverType(KnitroSolverParameters.SolverType.RELAXED);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);

        // Knitro solver converges and locates problem
        LoadFlowResult resultRKN = loadFlowRunner.run(network, parameters);
        boolean isConvergedRKN = resultRKN.isFullyConverged();
        assertTrue(isConvergedRKN, "RKN should converge");
    }

    private Terminal getRegulatingTerminal(Line line, String regulatedBus) {
        Optional<? extends Terminal> optional = line.getTerminals()
                .stream()
                .filter(terminal -> terminal.getBusBreakerView()
                        .getBus()
                        .getId()
                        .equals(regulatedBus))
                .findAny();
        assertFalse(optional.isEmpty(), "Regulating bus' terminal not found");
        return optional.get();
    }
}

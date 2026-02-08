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
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
class DivergenceAnalyzerDemoTest {

    /**
     * Demo: resilient load flow using Relaxed Knitro as a divergence analyzer.
     * <p>
     * Main steps:
     * 1. Set up a stressed IEEE 14-bus case (voltage perturbation) where Newton-Raphson diverges.
     * 2. Run Relaxed Knitro on the same case; it converges and provides a feasible solution.
     * 3. Apply the voltage target corrections suggested by the divergence analysis.
     * 4. Re-run Newton-Raphson (warm-start then cold-start) to show it now converges thanks to those corrections.
     */
    @Test
    void testResilientKnitroSolverForNonConvergentPowerFlow() {
        // --- Step 1: Apply a perturbation on the IEEE 14-bus network ---
        Network network = IeeeCdfNetworkFactory.create14();

        // Add the following voltage perturbation:
        //      Bus B1                              Bus: B5                                 Bus: B2
        // (generator: B1-G) <--- line: L1-5-1 ---> ( empty ) <------ line ------> (generator: B2-G controls B5)
        //  target_V = 143.1      X = 0.0017 Ohm                                       target_V = 131.0429 kV
        Generator b1g = network.getGenerator("B1-G");
        Generator b2g = network.getGenerator("B2-G");
        Line l15 = network.getLine("L1-5-1");
        l15.setR(0.0).setX(0.001742).setG1(0.0).setB1(0.0).setG2(0.0).setB2(0.0);
        b2g.setVoltageRegulatorOn(true)
            .setRegulatingTerminal(l15.getTerminal2())
            .setTargetV(131.0429); // Initial value: 141.075 kV

        // --- Step 2: Verify that Newton-Raphson diverges on this case ---
        LoadFlowParameters parameters = new LoadFlowParameters().setUseReactiveLimits(false).setDistributedSlack(false);
        OpenLoadFlowParameters.create(parameters).setAcSolverType(NewtonRaphsonFactory.NAME);

        LoadFlow.Runner loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new DenseMatrixFactory()));
        LoadFlowResult resultNewtonRaphson = loadFlowRunner.run(network, parameters);
        assertFalse(resultNewtonRaphson.isFullyConverged());

        // --- Step 3: Divergence analysis using Relaxed Knitro solver ---
        OpenLoadFlowParameters.create(parameters).setAcSolverType(KnitroSolverFactory.NAME);
        KnitroLoadFlowParameters knitroParams = new KnitroLoadFlowParameters();
        knitroParams.setKnitroSolverType(KnitroSolverParameters.SolverType.RELAXED);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);

        LoadFlowResult resultDivergenceAnalyzer = loadFlowRunner.run(network, parameters);
        assertTrue(resultDivergenceAnalyzer.isFullyConverged());

        // Apply the voltage target corrections computed by the divergence analysis
        b1g.setTargetV(b1g.getTargetV() - 6.0082);
        b2g.setTargetV(b2g.getTargetV() + 6.0461);

        // --- Step 4: Validate the divergence analysis by re-running Newton-Raphson ---
        parameters.setVoltageInitMode(LoadFlowParameters.VoltageInitMode.PREVIOUS_VALUES);
        parameters.getExtension(OpenLoadFlowParameters.class).setAcSolverType(NewtonRaphsonFactory.NAME);
        resultNewtonRaphson = loadFlowRunner.run(network, parameters);

        // Check that the divergence analyzer solution is a valid solution for the OLF equation system
        assertTrue(resultNewtonRaphson.isFullyConverged());
        assertEquals(1, resultNewtonRaphson.getComponentResults().getFirst().getIterationCount());

        // Check that a cold start also converges thanks to the target corrections
        parameters.setVoltageInitMode(LoadFlowParameters.VoltageInitMode.UNIFORM_VALUES);
        resultNewtonRaphson = loadFlowRunner.run(network, parameters);
        assertTrue(resultNewtonRaphson.isFullyConverged());
    }
}

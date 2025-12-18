/**
 * Copyright (c) 2025, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */

package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.iidm.network.Network;
import com.powsybl.ieeecdf.converter.IeeeCdfNetworkFactory;
import com.powsybl.iidm.network.*;
import com.powsybl.iidm.network.test.EurostagTutorialExample1Factory;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.DenseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.network.EurostagFactory;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static com.powsybl.openloadflow.util.LoadFlowAssert.assertReactivePowerEquals;
import static org.junit.jupiter.api.Assertions.*;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Yoann Anezin {@literal <yoann.anezin at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public class ReactiveLimitsKnitroSolverFunctionalTest {
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;
    private KnitroLoadFlowParameters knitroParams;

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new DenseMatrixFactory()));

        parameters = new LoadFlowParameters()
                .setUseReactiveLimits(true)
                .setDistributedSlack(false);

        OpenLoadFlowParameters.create(parameters)
                .setAcSolverType(KnitroSolverFactory.NAME)
                .setVoltageInitModeOverride(OpenLoadFlowParameters.VoltageInitModeOverride.FULL_VOLTAGE);

        knitroParams = new KnitroLoadFlowParameters()
                .setKnitroSolverType(KnitroSolverParameters.SolverType.USE_REACTIVE_LIMITS)
                .setGradientComputationMode(2)
                .setThreadNumber(1);

        parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
    }

    private static Network modifiedEurostagFactory(double qLow, double qUp) {
        Network network = EurostagFactory.fix(EurostagTutorialExample1Factory.create());

        // access to already created equipments
        Generator gen = network.getGenerator("GEN");
        Substation p1 = network.getSubstation("P1");

        // reduce GEN reactive range
        gen.newMinMaxReactiveLimits()
                .setMinQ(0)
                .setMaxQ(280)
                .add();

        // create a new generator GEN2
        VoltageLevel vlgen2 = p1.newVoltageLevel()
                .setId("VLGEN2")
                .setNominalV(24.0)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();

        vlgen2.getBusBreakerView()
                .newBus()
                .setId("NGEN2")
                .add();

        vlgen2.newGenerator()
                .setId("GEN2")
                .setBus("NGEN2")
                .setConnectableBus("NGEN2")
                .setMinP(-9999.99)
                .setMaxP(9999.99)
                .setVoltageRegulatorOn(true)
                .setTargetV(24.5)
                .setTargetP(100)
                .add();

        // update added generator reactive limits
        network.getGenerator("GEN2")
                .newMinMaxReactiveLimits()
                .setMinQ(qLow)
                .setMaxQ(qUp)
                .add();

        p1.newTwoWindingsTransformer()
                .setId("NGEN2_NHV1")
                .setBus1("NGEN2")
                .setConnectableBus1("NGEN2")
                .setRatedU1(24.0)
                .setBus2("NHV1")
                .setConnectableBus2("NHV1")
                .setRatedU2(400.0)
                .setR(0.19253333333333333)
                .setX(8.019911489428772)
                .add();

        // fix active power balance
        network.getLoad("LOAD")
                .setP0(699.838);

        return network;
    }

    /**
     * Perturbation of the network to urge a PV-PQ switch and set the new PQ bus to the lower bound on reactive power
     */
    @Test
    void testReacLimEurostagQlow() {
        Network network = modifiedEurostagFactory(250, 300);

        // verify convergence using finite differences
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());

        Generator gen2 = network.getGenerator("GEN2");
        TwoWindingsTransformer ngen2Nhv1 = network.getTwoWindingsTransformer("NGEN2_NHV1");
        TwoWindingsTransformer nhv2Nload = network.getTwoWindingsTransformer("NHV2_NLOAD");

        assertReactivePowerEquals(-250, gen2.getTerminal()); // GEN is correctly limited to 250 MVar
        assertReactivePowerEquals(250, ngen2Nhv1.getTerminal1());
        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());

        // verify convergence using exact jacobian
        knitroParams.setGradientComputationMode(1);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
        result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());

        assertReactivePowerEquals(-250, gen2.getTerminal()); // GEN is correctly limited to 250 MVar
        assertReactivePowerEquals(250, ngen2Nhv1.getTerminal1());
        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());
    }

    /**
     * Perturbation of the network to urge a PV-PQ switch and set the new PQ bus to the upper bound on reactive power
     */
    @Test
    void testReacLimEurostagQup() {
        Network network = modifiedEurostagFactory(0, 100);

        // verify convergence using finite differences
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());

        Generator gen = network.getGenerator("GEN");
        Generator gen2 = network.getGenerator("GEN2");
        TwoWindingsTransformer ngen2Nhv1 = network.getTwoWindingsTransformer("NGEN2_NHV1");
        TwoWindingsTransformer nhv2Nload = network.getTwoWindingsTransformer("NHV2_NLOAD");

        assertReactivePowerEquals(-280, gen.getTerminal());
        assertReactivePowerEquals(-100, gen2.getTerminal()); // GEN is correctly limited to 100 MVar
        assertReactivePowerEquals(100, ngen2Nhv1.getTerminal1());
        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());

        // verify convergence using exact jacobian
        knitroParams.setGradientComputationMode(1);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
        result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());

        assertReactivePowerEquals(-280, gen.getTerminal());
        assertReactivePowerEquals(-100, gen2.getTerminal()); // GEN is correctly limited to 100 MVar
        assertReactivePowerEquals(100, ngen2Nhv1.getTerminal1());
        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());
    }

    /**
     * Case of a bus containing a load and a generator.
     */
    @Test
    void testReacLimEurostagQupWithLoad() {
        Network network = modifiedEurostagFactory(0, 100);

        // add reactive load to network
        network.getVoltageLevel("VLGEN2")
                .newLoad()
                .setId("LOAD2")
                .setBus("NGEN2")
                .setConnectableBus("NGEN2")
                .setP0(0.0)
                .setQ0(30.0)
                .add();

        // verify convergence using finite differences
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());

        Generator gen2 = network.getGenerator("GEN2");
        TwoWindingsTransformer ngen2Nhv1 = network.getTwoWindingsTransformer("NGEN2_NHV1");
        TwoWindingsTransformer nhv2Nload = network.getTwoWindingsTransformer("NHV2_NLOAD");

        assertReactivePowerEquals(-100, gen2.getTerminal()); // GEN is correctly limited to 100 MVar
        assertReactivePowerEquals(70, ngen2Nhv1.getTerminal1());
        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());

        // verify convergence using exact jacobian
        knitroParams.setGradientComputationMode(1);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
        result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());

        assertReactivePowerEquals(-100, gen2.getTerminal()); // GEN is correctly limited to 100 MVar
        assertReactivePowerEquals(70, ngen2Nhv1.getTerminal1());
        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());
    }

    /**
     * Case of a bus containing two generators.
     */
    @Test
    void testReacLimEurostagQupWithGen() {
        Network network = modifiedEurostagFactory(0, 100);

        // add second generator
        Generator gen2Bis = network.getVoltageLevel("VLGEN2")
                .newGenerator()
                .setId("GEN2BIS")
                .setBus("NGEN2")
                .setConnectableBus("NGEN2")
                .setMinP(-9999.99)
                .setMaxP(9999.99)
                .setVoltageRegulatorOn(true)
                .setTargetV(24.5)
                .setTargetP(50)
                .add();

        // add second generator reactive limits
        gen2Bis.newMinMaxReactiveLimits()
                .setMinQ(0)
                .setMaxQ(40)
                .add();

        // verify convergence using finite differences
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());

        Generator gen = network.getGenerator("GEN");
        Generator gen2 = network.getGenerator("GEN2");
        TwoWindingsTransformer ngen2Nhv1 = network.getTwoWindingsTransformer("NGEN2_NHV1");
        TwoWindingsTransformer nhv2Nload = network.getTwoWindingsTransformer("NHV2_NLOAD");

        assertReactivePowerEquals(-122.735, gen.getTerminal());
        assertReactivePowerEquals(-100, gen2.getTerminal()); // GEN is correctly limited to 100 MVar
        assertReactivePowerEquals(140.0, ngen2Nhv1.getTerminal1());
        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());

        // verify convergence using exact jacobian
        knitroParams.setGradientComputationMode(1);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
        result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());

        assertReactivePowerEquals(-122.735, gen.getTerminal());
        assertReactivePowerEquals(-100, gen2.getTerminal()); // GEN is correctly limited to 100 MVar
        assertReactivePowerEquals(140.0, ngen2Nhv1.getTerminal1());
        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());
    }

    @Test
    void testReacLimIeee14() {
        Network network = IeeeCdfNetworkFactory.create14();
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
    }

    @Test
    void testReacLimIeee30() {
        Network network = IeeeCdfNetworkFactory.create30();
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
    }
}

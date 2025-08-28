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
import com.powsybl.math.matrix.SparseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.network.EurostagFactory;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.*;

import static com.powsybl.openloadflow.util.LoadFlowAssert.assertReactivePowerEquals;
import static org.junit.jupiter.api.Assertions.*;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Yoann Anezin {@literal <yoann.anezin at artelys.com>}
 */
public class ReactiveNoJacobienneTest {
    private static final double DEFAULT_TOLERANCE = 1e-2;
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new DenseMatrixFactory()));
        parameters = new LoadFlowParameters().setUseReactiveLimits(true)
                .setDistributedSlack(false);
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters(); // set gradient computation mode
        knitroLoadFlowParameters.setGradientComputationMode(2);
        knitroLoadFlowParameters.setMaxIterations(300);
        knitroLoadFlowParameters.setKnitroSolverType(KnitroSolverParameters.KnitroSolverType.RESILIENT);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);
        //parameters.setVoltageInitMode(LoadFlowParameters.VoltageInitMode.DC_VALUES);
        //OpenLoadFlowParameters.create(parameters).setAcSolverType("NEWTON_RAPHSON");
        OpenLoadFlowParameters.create(parameters).setAcSolverType(KnitroSolverFactory.NAME);
//        OpenLoadFlowParameters.get(parameters).setVoltageInitModeOverride(OpenLoadFlowParameters.VoltageInitModeOverride.);
    }

    @Test
    void testReacLimEurostagQlow() {
        HashMap<String,Double> listMinQ = new HashMap<>();
        HashMap<String,Double> listMaxQ = new HashMap<>();
        Network network = EurostagFactory.fix(EurostagTutorialExample1Factory.create());

        // access to already created equipments
        Load load = network.getLoad("LOAD");
        VoltageLevel vlgen = network.getVoltageLevel("VLGEN");
        TwoWindingsTransformer nhv2Nload = network.getTwoWindingsTransformer("NHV2_NLOAD");
        Generator gen = network.getGenerator("GEN");
        Substation p1 = network.getSubstation("P1");

        // reduce GEN reactive range
        gen.newMinMaxReactiveLimits()
                .setMinQ(0)
                .setMaxQ(280)
                .add();
        listMinQ.put(gen.getId(), -280.0);
        listMaxQ.put(gen.getId(), 280.0);

        // create a new generator GEN2
        VoltageLevel vlgen2 = p1.newVoltageLevel()
                .setId("VLGEN2")
                .setNominalV(24.0)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vlgen2.getBusBreakerView().newBus()
                .setId("NGEN2")
                .add();
        Generator gen2 = vlgen2.newGenerator()
                .setId("GEN2")
                .setBus("NGEN2")
                .setConnectableBus("NGEN2")
                .setMinP(-9999.99)
                .setMaxP(9999.99)
                .setVoltageRegulatorOn(true)
                .setTargetV(24.5)
                .setTargetP(100)
                .add();
        gen2.newMinMaxReactiveLimits()
                .setMinQ(250)
                .setMaxQ(300)
                .add();
        listMinQ.put(gen2.getId(), 250.0);
        listMaxQ.put(gen2.getId(), 300.0);
        int zb380 = 380 * 380 / 100;
        TwoWindingsTransformer ngen2Nhv1 = p1.newTwoWindingsTransformer()
                .setId("NGEN2_NHV1")
                .setBus1("NGEN2")
                .setConnectableBus1("NGEN2")
                .setRatedU1(24.0)
                .setBus2("NHV1")
                .setConnectableBus2("NHV1")
                .setRatedU2(400.0)
                .setR(0.24 / 1800 * zb380)
                .setX(Math.sqrt(10 * 10 - 0.24 * 0.24) / 1800 * zb380)
                .add();

        // fix active power balance
        load.setP0(699.838);

        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());
//        assertReactivePowerEquals(-8.094, gen.getTerminal());
        assertReactivePowerEquals(-250, gen2.getTerminal()); // GEN is correctly limited to 250 MVar
        assertReactivePowerEquals(250, ngen2Nhv1.getTerminal1());
        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());
        ReacLimitsTestsUtils utilFunctions = new ReacLimitsTestsUtils();
        utilFunctions.checkSwitches(network, listMinQ, listMaxQ);
        utilFunctions.verifNewtonRaphson(network, parameters, loadFlowRunner, 0);
    }

    @Test
    void testReacLimEurostagQup() {
        HashMap<String,Double> listMinQ = new HashMap<>();
        HashMap<String,Double> listMaxQ = new HashMap<>();
        Network network = EurostagFactory.fix(EurostagTutorialExample1Factory.create());

        // access to already created equipments
        Load load = network.getLoad("LOAD");

        VoltageLevel vlgen = network.getVoltageLevel("VLGEN");
        TwoWindingsTransformer nhv2Nload = network.getTwoWindingsTransformer("NHV2_NLOAD");
        Generator gen = network.getGenerator("GEN");
        Substation p1 = network.getSubstation("P1");

        // reduce GEN reactive range
        gen.newMinMaxReactiveLimits()
                .setMinQ(0)
                .setMaxQ(280)
                .add();
        listMinQ.put(gen.getId(), -280.0);
        listMaxQ.put(gen.getId(), 280.0);

        // create a new generator GEN2
        VoltageLevel vlgen2 = p1.newVoltageLevel()
                .setId("VLGEN2")
                .setNominalV(24.0)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vlgen2.getBusBreakerView().newBus()
                .setId("NGEN2")
                .add();
        Generator gen2 = vlgen2.newGenerator()
                .setId("GEN2")
                .setBus("NGEN2")
                .setConnectableBus("NGEN2")
                .setMinP(-9999.99)
                .setMaxP(9999.99)
                .setVoltageRegulatorOn(true)
                .setTargetV(24.5)
                .setTargetP(100)
                .add();
        gen2.newMinMaxReactiveLimits()
                .setMinQ(0)
                .setMaxQ(100)
                .add();
        listMinQ.put(gen2.getId(), 0.0);
        listMaxQ.put(gen2.getId(), 100.0);
        int zb380 = 380 * 380 / 100;
        TwoWindingsTransformer ngen2Nhv1 = p1.newTwoWindingsTransformer()
                .setId("NGEN2_NHV1")
                .setBus1("NGEN2")
                .setConnectableBus1("NGEN2")
                .setRatedU1(24.0)
                .setBus2("NHV1")
                .setConnectableBus2("NHV1")
                .setRatedU2(400.0)
                .setR(0.24 / 1800 * zb380)
                .setX(Math.sqrt(10 * 10 - 0.24 * 0.24) / 1800 * zb380)
                .add();

        // fix active power balance
        load.setP0(699.838);

        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());
//        assertReactivePowerEquals(-164.315, gen.getTerminal());
//        assertReactivePowerEquals(-100, gen2.getTerminal()); // GEN is correctly limited to 100 MVar
//        assertReactivePowerEquals(100, ngen2Nhv1.getTerminal1());
//        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());
        ReacLimitsTestsUtils utilFunctions = new ReacLimitsTestsUtils();
        utilFunctions.checkSwitches(network, listMinQ, listMaxQ);
        utilFunctions.verifNewtonRaphson(network, parameters, loadFlowRunner, 0);
    }

    @Test
    void testReacLimEurostagQupWithLoad() {
        HashMap<String,Double> listMinQ = new HashMap<>();
        HashMap<String,Double> listMaxQ = new HashMap<>();
        Network network = EurostagFactory.fix(EurostagTutorialExample1Factory.create());

        // access to already created equipments
        Load load = network.getLoad("LOAD");

        VoltageLevel vlgen = network.getVoltageLevel("VLGEN");
        TwoWindingsTransformer nhv2Nload = network.getTwoWindingsTransformer("NHV2_NLOAD");
        Generator gen = network.getGenerator("GEN");
        Substation p1 = network.getSubstation("P1");

        // reduce GEN reactive range
        gen.newMinMaxReactiveLimits()
                .setMinQ(0)
                .setMaxQ(280)
                .add();
        listMinQ.put(gen.getId(), -280.0);
        listMaxQ.put(gen.getId(), 280.0);

        // create a new generator GEN2
        VoltageLevel vlgen2 = p1.newVoltageLevel()
                .setId("VLGEN2")
                .setNominalV(24.0)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vlgen2.getBusBreakerView().newBus()
                .setId("NGEN2")
                .add();
        Generator gen2 = vlgen2.newGenerator()
                .setId("GEN2")
                .setBus("NGEN2")
                .setConnectableBus("NGEN2")
                .setMinP(-9999.99)
                .setMaxP(9999.99)
                .setVoltageRegulatorOn(true)
                .setTargetV(24.5)
                .setTargetP(100)
                .add();
        gen2.newMinMaxReactiveLimits()
                .setMinQ(0)
                .setMaxQ(100)
                .add();
        listMinQ.put(gen2.getId(), 0.0);
        listMaxQ.put(gen2.getId(), 100.0);
        Load load2 = vlgen2.newLoad()
                .setId("LOAD2")
                .setBus("NGEN2")
                .setConnectableBus("NGEN2")
                .setP0(0.0)
                .setQ0(30.0)
                .add();
        int zb380 = 380 * 380 / 100;
        TwoWindingsTransformer ngen2Nhv1 = p1.newTwoWindingsTransformer()
                .setId("NGEN2_NHV1")
                .setBus1("NGEN2")
                .setConnectableBus1("NGEN2")
                .setRatedU1(24.0)
                .setBus2("NHV1")
                .setConnectableBus2("NHV1")
                .setRatedU2(400.0)
                .setR(0.24 / 1800 * zb380)
                .setX(Math.sqrt(10 * 10 - 0.24 * 0.24) / 1800 * zb380)
                .add();

        // fix active power balance
        load.setP0(699.838);

        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());
        //assertReactivePowerEquals(-196.263, gen.getTerminal());
        assertReactivePowerEquals(-100, gen2.getTerminal()); // GEN is correctly limited to 100 MVar
        assertReactivePowerEquals(70, ngen2Nhv1.getTerminal1());
        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());
        ReacLimitsTestsUtils utilFunctions = new ReacLimitsTestsUtils();
        utilFunctions.checkSwitches(network, listMinQ, listMaxQ);
        utilFunctions.verifNewtonRaphson(network, parameters, loadFlowRunner, 0);
    }

    @Test
    void testReacLimEurostagQupWithGen() {
        HashMap<String,Double> listMinQ = new HashMap<>();
        HashMap<String,Double> listMaxQ = new HashMap<>();
        Network network = EurostagFactory.fix(EurostagTutorialExample1Factory.create());

        // access to already created equipments
        Load load = network.getLoad("LOAD");

        VoltageLevel vlgen = network.getVoltageLevel("VLGEN");
        TwoWindingsTransformer nhv2Nload = network.getTwoWindingsTransformer("NHV2_NLOAD");
        Generator gen = network.getGenerator("GEN");
        Substation p1 = network.getSubstation("P1");

        // reduce GEN reactive range
        gen.newMinMaxReactiveLimits()
                .setMinQ(0)
                .setMaxQ(280)
                .add();
        listMinQ.put(gen.getId(), -280.0);
        listMaxQ.put(gen.getId(), 280.0);

        // create a new generator GEN2
        VoltageLevel vlgen2 = p1.newVoltageLevel()
                .setId("VLGEN2")
                .setNominalV(24.0)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vlgen2.getBusBreakerView().newBus()
                .setId("NGEN2")
                .add();
        Generator gen2 = vlgen2.newGenerator()
                .setId("GEN2")
                .setBus("NGEN2")
                .setConnectableBus("NGEN2")
                .setMinP(-9999.99)
                .setMaxP(9999.99)
                .setVoltageRegulatorOn(true)
                .setTargetV(24.5)
                .setTargetP(100)
                .add();
        gen2.newMinMaxReactiveLimits()
                .setMinQ(0)
                .setMaxQ(100)
                .add();
        listMinQ.put(gen2.getId(), 0.0);
        listMaxQ.put(gen2.getId(), 100.0);
        Generator gen2Bis = vlgen2.newGenerator()
                .setId("GEN2BIS")
                .setBus("NGEN2")
                .setConnectableBus("NGEN2")
                .setMinP(-9999.99)
                .setMaxP(9999.99)
                .setVoltageRegulatorOn(true)
                .setTargetV(24.5)
                .setTargetP(50)
                .add();
        gen2Bis.newMinMaxReactiveLimits()
                .setMinQ(0)
                .setMaxQ(40)
                .add();
        listMinQ.put(gen2Bis.getId(), 0.0);
        listMaxQ.put(gen2Bis.getId(), 40.0);
        int zb380 = 380 * 380 / 100;
        TwoWindingsTransformer ngen2Nhv1 = p1.newTwoWindingsTransformer()
                .setId("NGEN2_NHV1")
                .setBus1("NGEN2")
                .setConnectableBus1("NGEN2")
                .setRatedU1(24.0)
                .setBus2("NHV1")
                .setConnectableBus2("NHV1")
                .setRatedU2(400.0)
                .setR(0.24 / 1800 * zb380)
                .setX(Math.sqrt(10 * 10 - 0.24 * 0.24) / 1800 * zb380)
                .add();

        // fix active power balance
        load.setP0(699.838);
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());
        assertReactivePowerEquals(-122.735, gen.getTerminal());
        assertReactivePowerEquals(-100, gen2.getTerminal()); // GEN is correctly limited to 100 MVar
        assertReactivePowerEquals(140.0, ngen2Nhv1.getTerminal1());
        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());
        ReacLimitsTestsUtils utilFunctions = new ReacLimitsTestsUtils();
        utilFunctions.checkSwitches(network, listMinQ, listMaxQ);
        utilFunctions.verifNewtonRaphson(network, parameters, loadFlowRunner, 0);
    }

    @Test
    void testReacLimIeee14() {
        HashMap<String,Double> listMinQ = new HashMap<>();
        HashMap<String,Double> listMaxQ = new HashMap<>();
        parameters.setUseReactiveLimits(true);
        Network network = IeeeCdfNetworkFactory.create14();
        for (var g : network.getGenerators()) {
            if (g.getReactiveLimits().getMinQ(g.getTerminal().getBusView().getBus().getP()) > -1.7976931348623157E308) {
                listMinQ.put(g.getId(), g.getReactiveLimits().getMinQ(g.getTerminal().getBusView().getBus().getP()));
                listMaxQ.put(g.getId(), g.getReactiveLimits().getMaxQ(g.getTerminal().getBusView().getBus().getP()));
            } else {
                g.newMinMaxReactiveLimits()
                        .setMinQ(-2000)
                        .setMaxQ(2000)
                        .add();
                listMinQ.put(g.getId(), -2000.0);
                listMaxQ.put(g.getId(), 2000.0);
            }
        }
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        ReacLimitsTestsUtils utilFunctions = new ReacLimitsTestsUtils();
        utilFunctions.checkSwitches(network, listMinQ, listMaxQ);
        utilFunctions.verifNewtonRaphson(network, parameters, loadFlowRunner, 0);
    }

    @Test
    void testReacLimIeee30() {
        HashMap<String,Double> listMinQ = new HashMap<>();
        HashMap<String,Double> listMaxQ = new HashMap<>();
        parameters.setUseReactiveLimits(true);
        Network network = IeeeCdfNetworkFactory.create30();
        for (var g : network.getGenerators()) {
            if (g.getReactiveLimits().getMinQ(g.getTerminal().getBusView().getBus().getP()) > -1.7976931348623157E308) {
                listMinQ.put(g.getId(), g.getReactiveLimits().getMinQ(g.getTerminal().getBusView().getBus().getP()));
                listMaxQ.put(g.getId(), g.getReactiveLimits().getMaxQ(g.getTerminal().getBusView().getBus().getP()));
            }  else {
                g.newMinMaxReactiveLimits()
                        .setMinQ(-2000)
                        .setMaxQ(2000)
                        .add();
                listMinQ.put(g.getId(), -2000.0);
                listMaxQ.put(g.getId(), 2000.0);
            }
        }
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        ReacLimitsTestsUtils utilFunctions = new ReacLimitsTestsUtils();
        utilFunctions.checkSwitches(network, listMinQ, listMaxQ);
        utilFunctions.verifNewtonRaphson(network, parameters, loadFlowRunner, 0);
    }

    @Test
    void testReacLimIeee118() {
        HashMap<String,Double> listMinQ = new HashMap<>();
        HashMap<String,Double> listMaxQ = new HashMap<>();
        parameters.setUseReactiveLimits(true);
        Network network = IeeeCdfNetworkFactory.create118();
        for (var g : network.getGenerators()) {
            if (g.getReactiveLimits().getMinQ(g.getTerminal().getBusView().getBus().getP()) > -1.7976931348623157E308) {
                listMinQ.put(g.getId(), g.getReactiveLimits().getMinQ(g.getTerminal().getBusView().getBus().getP()));
                listMaxQ.put(g.getId(), g.getReactiveLimits().getMaxQ(g.getTerminal().getBusView().getBus().getP()));
            }
        }
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        ReacLimitsTestsUtils utilFunctions = new ReacLimitsTestsUtils();
        utilFunctions.checkSwitches(network, listMinQ, listMaxQ);
        utilFunctions.verifNewtonRaphson(network, parameters, loadFlowRunner, 0);
    }

    @Test
    void testReacLimIeee300() {
        HashMap<String,Double> listMinQ = new HashMap<>();
        HashMap<String,Double> listMaxQ = new HashMap<>();
        parameters.setUseReactiveLimits(true);
        Network network = IeeeCdfNetworkFactory.create300();
        for (var g : network.getGenerators()) {
            if (g.getReactiveLimits().getMinQ(g.getTerminal().getBusView().getBus().getP()) > -1.7976931348623157E308) {
                listMinQ.put(g.getId(), g.getReactiveLimits().getMinQ(g.getTerminal().getBusView().getBus().getP()));
                listMaxQ.put(g.getId(), g.getReactiveLimits().getMaxQ(g.getTerminal().getBusView().getBus().getP()));
            }
        }
        network.getGenerator("B7049-G").newMinMaxReactiveLimits().setMinQ(-500).setMaxQ(500).add();
        listMinQ.put("B7049-G", -500.0);
        listMaxQ.put("B7049-G", 500.0);
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        ReacLimitsTestsUtils utilFunctions = new ReacLimitsTestsUtils();
        utilFunctions.checkSwitches(network, listMinQ, listMaxQ);
        utilFunctions.verifNewtonRaphson(network, parameters, loadFlowRunner, 0);
    }
}
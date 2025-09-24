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
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.SparseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.network.SlackBusSelectionMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.time.LocalDateTime;

import java.util.*;

import static org.junit.jupiter.api.Assertions.*;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Yoann Anezin {@literal <yoann.anezin at artelys.com>}
 */
public class ReacLimPertubationTest {
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;
    private static String logFile = "D:\\Documents\\Logs_Tests\\Logs.txt";

    public ReacLimPertubationTest() throws IOException {
    }

    private int fixReacLim(Network network, HashMap<String, Double> listMinQ, HashMap<String, Double> listMaxQ) {
        int numbreLimReacAdded = 0;
        for (var g : network.getGenerators()) {
            if (g.getReactiveLimits().getMinQ(g.getTargetP()) > -1.7976931348623157E308) {
                listMinQ.put(g.getId(), g.getReactiveLimits().getMinQ(g.getTargetP()));
                listMaxQ.put(g.getId(), g.getReactiveLimits().getMaxQ(g.getTargetP()));
            } else {
                g.newMinMaxReactiveLimits()
                        .setMinQ(-2000)
                        .setMaxQ(2000)
                        .add();
                listMinQ.put(g.getId(), -2000.0);
                listMaxQ.put(g.getId(), 2000.0);
                numbreLimReacAdded++;
            }
        }
        return numbreLimReacAdded;
    }

    void logsWriting(KnitroLoadFlowParameters knitroLoadFlowParameters, KnitroWritter knitroWritter) {
        knitroWritter.write("Solver used : " + OpenLoadFlowParameters.get(parameters).getAcSolverType(), true);
        knitroWritter.write("Init Mode : " + parameters.getVoltageInitMode().toString(), true);
        if (OpenLoadFlowParameters.get(parameters).getAcSolverType().equals("KNITRO")) {
            knitroWritter.write("Version de Knitro : " + knitroLoadFlowParameters.getKnitroSolverType().name(), true);
        }
        knitroWritter.write("Override Init Mode : " + OpenLoadFlowParameters.get(parameters).getVoltageInitModeOverride().name(), true);
        knitroWritter.write("Max Iterations : " + knitroLoadFlowParameters.getMaxIterations(), true);
        knitroWritter.write("Gradient Computation Mode : " + knitroLoadFlowParameters.getGradientComputationMode(), true);
    }

    /**
     *Start all the test process and writes logs by the same time
     * @param logFile           file where logs are written
     * @param network           network of work
     * @param perturbProcess    Indicates the perturbation to apply. Current possible choices : ActivePowerGlobal,
     *                          ActivePowerLocal, ReactivePower, None
     * @param perturbValue      value applied in the pertubation chosen (wont be used in the "None" case)
     */
    void testprocess(String logFile, Network network, String perturbProcess, double perturbValue) {
        long start = System.nanoTime();

        KnitroWritter knitroWritter = new KnitroWritter(logFile);
        KnitroLoadFlowParameters knitroLoadFlowParameters = parameters.getExtension(KnitroLoadFlowParameters.class);
        knitroLoadFlowParameters.setKnitroWritter(knitroWritter);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);

        HashMap<String, Double> listMinQ = new HashMap<>();
        HashMap<String, Double> listMaxQ = new HashMap<>();
        parameters.setUseReactiveLimits(true);
        int numbreLimReacAdded = fixReacLim(network, listMinQ, listMaxQ);

        knitroWritter.write("[" + LocalDateTime.now() + "]", false);
        knitroWritter.write(numbreLimReacAdded + " bus pour lesquels les limites réactif ont été ajoutées", true);
        logsWriting(knitroLoadFlowParameters, knitroWritter);
        switch (perturbProcess) {
            case "ActivePowerGlobal":
                ReacLimitsTestsUtils.applyActivePowerPerturbation(network, perturbValue);
                knitroWritter.write("Perturbed by global loads' increasement (All multiplied by " +
                        perturbValue * 100 + "% of total load)", true);
                break;
            case "ActivePowerLocal":
                PerturbationFactory.applyActivePowerPerturbation(network,
                    PerturbationFactory.getActivePowerPerturbation(network), perturbValue);
                knitroWritter.write("Perturbed by uniq big load (" + perturbValue * 100 + "% of total load)", true);
                break;
            case "ReactivePower":
                PerturbationFactory.applyReactivePowerPerturbation(network,
                        PerturbationFactory.getReactivePowerPerturbation(network), perturbValue);
                knitroWritter.write("Perturbed by power injection by the shunt (Target Q = " + perturbValue + ")", true);
                break;
            case "None":
                knitroWritter.write("No Pertubations", true);
                break;
            default:
                knitroWritter.write("No Pertubations, you have miswritten the pertubation's instruction", true);
                break;
        }

        // solve and check
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        ReacLimitsTestsUtils.checkSwitches(network, listMinQ, listMaxQ);
        long end = System.nanoTime();
        knitroWritter.write("Durée du test : " + (end - start) * 1e-9 + " secondes", true);
        knitroWritter.write("Nombre d'itérations : " + result.getComponentResults().get(0).getIterationCount(), true);
        knitroWritter.write("Status à l'arrivée : " + result.getComponentResults().get(0).getStatus().name(), true);
    }

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new SparseMatrixFactory()));
        parameters = new LoadFlowParameters().setUseReactiveLimits(true)
                .setDistributedSlack(false);
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters(); // set gradient computation mode
        knitroLoadFlowParameters.setGradientComputationMode(1);
        knitroLoadFlowParameters.setMaxIterations(2000);
        knitroLoadFlowParameters.setKnitroSolverType(KnitroSolverParameters.KnitroSolverType.REACTIVLIMITS);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);
        //parameters.setVoltageInitMode(LoadFlowParameters.VoltageInitMode.DC_VALUES);
        //OpenLoadFlowParameters.create(parameters).setAcSolverType("NEWTON_RAPHSON");
//        OpenLoadFlowParameters.create(parameters).setAcSolverType(KnitroSolverFactory.NAME);
        OpenLoadFlowParameters.get(parameters).setVoltageInitModeOverride(OpenLoadFlowParameters.VoltageInitModeOverride.FULL_VOLTAGE);

    }

    /**
     *<pre>
     *     G1        LD2        G3
     *     |    L12   |   L23   |
     *     |  ------- | ------- |
     *     B1         B2        B3
     *</pre>
     */
    @Test
    public void createNetworkWithT2wtActivePower() {

        Network network = Network.create("yoann-n", "test");

        Substation substation1 = network.newSubstation()
                .setId("SUBSTATION1")
                .setCountry(Country.FR)
                .add();
        VoltageLevel vl1 = substation1.newVoltageLevel()
                .setId("VL_1")
                .setNominalV(132.0)
                .setLowVoltageLimit(118.8)
                .setHighVoltageLimit(145.2)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vl1.getBusBreakerView().newBus()
                .setId("BUS_1")
                .add();
        Generator g1 = vl1.newGenerator()
                .setId("GEN_1")
                .setBus("BUS_1")
                .setMinP(0.0)
                .setMaxP(140)
                .setTargetP(25)
                .setTargetV(135)
                .setVoltageRegulatorOn(true)
                .add();
        g1.newMinMaxReactiveLimits().setMinQ(-30000.0).setMaxQ(30000.0).add();

        Substation substation = network.newSubstation()
                .setId("SUBSTATION")
                .setCountry(Country.FR)
                .add();
        VoltageLevel vl2 = substation.newVoltageLevel()
                .setId("VL_2")
                .setNominalV(132.0)
                .setLowVoltageLimit(118.8)
                .setHighVoltageLimit(145.2)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vl2.getBusBreakerView().newBus()
                .setId("BUS_2")
                .add();
        vl2.newLoad()
                .setId("LOAD_2")
                .setBus("BUS_2")
                .setP0(35)
                .setQ0(20)
                .add();

        VoltageLevel vl3 = substation.newVoltageLevel()
                .setId("VL_3")
                .setNominalV(132.0)
                .setLowVoltageLimit(118.8)
                .setHighVoltageLimit(145.2)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vl3.getBusBreakerView().newBus()
                .setId("BUS_3")
                .add();
        Generator g3 = vl3.newGenerator()
                .setId("GEN_3")
                .setBus("BUS_3")
                .setMinP(0.0)
                .setMaxP(140)
                .setTargetP(15)
                .setTargetV(130)
                .setVoltageRegulatorOn(true)
                .add();
        g3.newMinMaxReactiveLimits().setMinQ(-3000.0).setMaxQ(3000.0).add();

        network.newLine()
                .setId("LINE_12")
                .setBus1("BUS_1")
                .setBus2("BUS_2")
                .setR(1.05)
                .setX(10.0)
                .setG1(0.0000005)
                .add();
        network.newLine()
                .setId("LINE_23")
                .setBus1("BUS_2")
                .setBus2("BUS_3")
                .setR(1.05)
                .setX(10.0)
                .setG1(0.0000005)
                .add();

        OpenLoadFlowParameters.get(parameters)
//                .setVoltageInitModeOverride(OpenLoadFlowParameters.VoltageInitModeOverride.FULL_VOLTAGE)
                .setSlackBusSelectionMode(SlackBusSelectionMode.NAME)
                .setSlackBusId("VL_1_0");

        logFile = "D:\\Documents\\Logs_Tests\\Logs_3bus_Active_Power_Perturbation.txt";
        testprocess(logFile, network, "ActivePowerLocal", 40.0);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    /**
     *<pre>
     *     G1        LD2        G3
     *     |    L12   |   L23   |
     *     |  ------- | ------- |
     *     B1         B2        B3
     *</pre>
     */
    @Test
    public void createNetworkWithT2wtReactivePower() {
        HashMap<String, Double> listMinQ = new HashMap<>();
        HashMap<String, Double> listMaxQ = new HashMap<>();

        Network network = Network.create("yoann-n", "test");

        Substation substation1 = network.newSubstation()
                .setId("SUBSTATION1")
                .setCountry(Country.FR)
                .add();
        VoltageLevel vl1 = substation1.newVoltageLevel()
                .setId("VL_1")
                .setNominalV(132.0)
                .setLowVoltageLimit(118.8)
                .setHighVoltageLimit(145.2)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vl1.getBusBreakerView().newBus()
                .setId("BUS_1")
                .add();
        Generator g1 = vl1.newGenerator()
                .setId("GEN_1")
                .setBus("BUS_1")
                .setMinP(0.0)
                .setMaxP(140)
                .setTargetP(25)
                .setTargetV(135)
                .setVoltageRegulatorOn(true)
                .add();
        g1.newMinMaxReactiveLimits().setMinQ(-3000.0).setMaxQ(3000.0).add();
        listMinQ.put(g1.getId(), -3000.0);
        listMaxQ.put(g1.getId(), 3000.0);

        Substation substation = network.newSubstation()
                .setId("SUBSTATION")
                .setCountry(Country.FR)
                .add();
        VoltageLevel vl2 = substation.newVoltageLevel()
                .setId("VL_2")
                .setNominalV(132.0)
                .setLowVoltageLimit(118.8)
                .setHighVoltageLimit(145.2)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vl2.getBusBreakerView().newBus()
                .setId("BUS_2")
                .add();
        vl2.newLoad()
                .setId("LOAD_2")
                .setBus("BUS_2")
                .setP0(35)
                .setQ0(20)
                .add();

        VoltageLevel vl3 = substation.newVoltageLevel()
                .setId("VL_3")
                .setNominalV(132.0)
                .setLowVoltageLimit(118.8)
                .setHighVoltageLimit(145.2)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vl3.getBusBreakerView().newBus()
                .setId("BUS_3")
                .add();
        Generator g3 = vl3.newGenerator()
                .setId("GEN_3")
                .setBus("BUS_3")
                .setMinP(0.0)
                .setMaxP(140)
                .setTargetP(15)
                .setTargetV(130)
                .setVoltageRegulatorOn(true)
                .add();
        g3.newMinMaxReactiveLimits().setMinQ(-3000.0).setMaxQ(3000.0).add();
        listMinQ.put(g3.getId(), -3000.0);
        listMaxQ.put(g3.getId(), 3000.0);

        network.newLine()
                .setId("LINE_12")
                .setBus1("BUS_1")
                .setBus2("BUS_2")
                .setR(1.05)
                .setX(10.0)
                .setG1(0.0000005)
                .add();
        network.newLine()
                .setId("LINE_23")
                .setBus1("BUS_2")
                .setBus2("BUS_3")
                .setR(1.05)
                .setX(10.0)
                .setG1(0.0000005)
                .add();
        network.getVoltageLevel("VL_1").newShuntCompensator()
                .setId("SC")
                .setBus("BUS_1")
                .setConnectableBus("BUS_1")
                .setSectionCount(1)
                .newLinearModel()
                .setBPerSection(3.25 * Math.pow(10, -3))
                .setMaximumSectionCount(1)
                .add()
                .add();

        OpenLoadFlowParameters.get(parameters)
//                .setVoltageInitModeOverride(OpenLoadFlowParameters.VoltageInitModeOverride.FULL_VOLTAGE)
                .setSlackBusSelectionMode(SlackBusSelectionMode.NAME)
                .setSlackBusId("VL_1_0");

        logFile = "D:\\Documents\\Logs_Tests\\Logs_3bus_Reactive_Power_Perturbation.txt";
        testprocess(logFile, network, "ReactivePower", 1E10);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    /**
     *<pre>
     *     G1        LD2        G3
     *     |    L12   |   L23   |
     *     |  ------- | ------- |
     *     B1         B2        B3
     *</pre>
     */
    @Test
    public void createNetworkWithT2wtVoltage() {
        HashMap<String, Double> listMinQ = new HashMap<>();
        HashMap<String, Double> listMaxQ = new HashMap<>();
        Network network = Network.create("yoann-n", "test");

        Substation substation1 = network.newSubstation()
                .setId("SUBSTATION1")
                .setCountry(Country.FR)
                .add();
        VoltageLevel vl1 = substation1.newVoltageLevel()
                .setId("VL_1")
                .setNominalV(132.0)
                .setLowVoltageLimit(118.8)
                .setHighVoltageLimit(145.2)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vl1.getBusBreakerView().newBus()
                .setId("BUS_1")
                .add();
        Generator g1 = vl1.newGenerator()
                .setId("GEN_1")
                .setBus("BUS_1")
                .setMinP(0.0)
                .setMaxP(140)
                .setTargetP(25)
                .setTargetV(135)
                .setVoltageRegulatorOn(true)
                .add();
        g1.newMinMaxReactiveLimits().setMinQ(-3000.0).setMaxQ(3000.0).add();

        Substation substation = network.newSubstation()
                .setId("SUBSTATION")
                .setCountry(Country.FR)
                .add();
        VoltageLevel vl2 = substation.newVoltageLevel()
                .setId("VL_2")
                .setNominalV(132.0)
                .setLowVoltageLimit(118.8)
                .setHighVoltageLimit(145.2)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vl2.getBusBreakerView().newBus()
                .setId("BUS_2")
                .add();
        vl2.newLoad()
                .setId("LOAD_2")
                .setBus("BUS_2")
                .setP0(35)
                .setQ0(20)
                .add();

        VoltageLevel vl3 = substation.newVoltageLevel()
                .setId("VL_3")
                .setNominalV(132.0)
                .setLowVoltageLimit(118.8)
                .setHighVoltageLimit(145.2)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vl3.getBusBreakerView().newBus()
                .setId("BUS_3")
                .add();
        Generator g3 = vl3.newGenerator()
                .setId("GEN_3")
                .setBus("BUS_3")
                .setMinP(0.0)
                .setMaxP(140)
                .setTargetP(15)
                .setTargetV(130)
                .setVoltageRegulatorOn(true)
                .add();
        g3.newMinMaxReactiveLimits().setMinQ(-3000.0).setMaxQ(3000.0).add();

        network.newLine()
                .setId("LINE_12")
                .setBus1("BUS_1")
                .setBus2("BUS_2")
                .setR(1.05)
                .setX(10.0)
                .setG1(0.0000005)
                .add();
        network.newLine()
                .setId("LINE_23")
                .setBus1("BUS_2")
                .setBus2("BUS_3")
                .setR(1.05)
                .setX(10.0)
                .setG1(0.0000005)
                .add();

        OpenLoadFlowParameters.get(parameters)
//                .setVoltageInitModeOverride(OpenLoadFlowParameters.VoltageInitModeOverride.FULL_VOLTAGE)
                .setSlackBusSelectionMode(SlackBusSelectionMode.NAME)
                .setVoltageRemoteControl(false)
                .setSlackBusId("VL_1_0");

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.75;
        PerturbationFactory.VoltagePerturbation perturbation = PerturbationFactory.getVoltagePerturbation(network);
        System.out.println(perturbation);
        PerturbationFactory.applyVoltagePerturbation(network, perturbation, rPU, xPU, alpha);

        fixReacLim(network, listMinQ, listMaxQ);
//        OpenLoadFlowParameters.get(parameters).setAcSolverType("NEWTON_RAPHSON");
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        ReacLimitsTestsUtils.checkSwitches(network, listMinQ, listMaxQ);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee14ActivePowerPerturbed() {
        String perturbation = "ActivePowerLocal";
        logFile = "D:\\Documents\\Logs_Tests\\Logs_ieee14_" + perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create14();
        testprocess(logFile, network, perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee30ActivePowerPerturbed() {
        String perturbation = "ActivePowerLocal";
        logFile = "D:\\Documents\\Logs_Tests\\Logs_ieee30_" + perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create30();
        testprocess(logFile, network, perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee30ReactivePowerPerturbed() {
        String perturbation = "ReactivePower";
        logFile = "D:\\Documents\\Logs_Tests\\Logs_ieee30_" + perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create30();
        testprocess(logFile, network, perturbation, 1E11);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee30VoltagePerturbed() {
        // set up network
        HashMap<String, Double> listMinQ = new HashMap<>();
        HashMap<String, Double> listMaxQ = new HashMap<>();
        parameters.setUseReactiveLimits(true);
        Network network = IeeeCdfNetworkFactory.create30();
        fixReacLim(network, listMinQ, listMaxQ);

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;
        PerturbationFactory.VoltagePerturbation perturbation = PerturbationFactory.getVoltagePerturbation(network);
        PerturbationFactory.applyVoltagePerturbation(network, perturbation, rPU, xPU, alpha);

        // solve and check
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        ReacLimitsTestsUtils.checkSwitches(network, listMinQ, listMaxQ);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee118ActivePowerPerturbed() {
        String perturbation = "ActivePowerLocal";
        logFile = "D:\\Documents\\Logs_Tests\\Logs_ieee118_" + perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create118();
        testprocess(logFile, network, perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee118ReactivePowerPerturbed() {
        String perturbation = "ReactivePower";
        logFile = "D:\\Documents\\Logs_Tests\\Logs_ieee118_" + perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create118();
        testprocess(logFile, network, perturbation, 1E10);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee118VoltagePerturbed() {
        // set up network
        HashMap<String, Double> listMinQ = new HashMap<>();
        HashMap<String, Double> listMaxQ = new HashMap<>();
        parameters.setUseReactiveLimits(true);
        Network network = IeeeCdfNetworkFactory.create118();
        fixReacLim(network, listMinQ, listMaxQ);

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;
        PerturbationFactory.VoltagePerturbation perturbation = PerturbationFactory.getVoltagePerturbation(network);
        PerturbationFactory.applyVoltagePerturbation(network, perturbation, rPU, xPU, alpha);

        // solve and check
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        ReacLimitsTestsUtils.checkSwitches(network, listMinQ, listMaxQ);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee300ActivePowerPerturbed() {
        String perturbation = "ActivePowerGlobal";
        logFile = "D:\\Documents\\Logs_Tests\\Logs_ieee300_" + perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create300();
        testprocess(logFile, network, perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee300ReactivePowerPerturbed() {
        String perturbation = "ReactivePower";
        logFile = "D:\\Documents\\Logs_Tests\\Logs_ieee300_" + perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create300();
        testprocess(logFile, network, perturbation, 1E11);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    void testxiidm1888ActivePowerOneLoadPerturbed() throws IOException {
        String perturbation = "ActivePowerLocal";
        logFile = "D:\\Documents\\Logs_Tests\\Logs_Rte1888_" + perturbation + ".txt";
        Network network = Network.read("D:\\Documents\\Réseaux\\rte1888.xiidm");
        testprocess(logFile, network, perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    void testxiidm6515() throws IOException {
        String perturbation = "None";
        logFile = "D:\\Documents\\Logs_Tests\\Logs_Rte6515_" + perturbation + ".txt";
        Network network = Network.read("D:\\Documents\\Réseaux\\rte6515.xiidm");
        testprocess(logFile, network, perturbation, 1.2);
    }
}

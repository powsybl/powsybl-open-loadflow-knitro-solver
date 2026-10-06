/**
 * Copyright (c) 2026, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.iidm.network.*;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.math.matrix.SparseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.network.SlackBusSelectionMode;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Checks the diagnostics of the P and Q slack variables, on networks where bus 2 cannot be balanced without
 * relaxing its power equations.
 *
 * @author Mael Verbois {@literal <mael.verbois at artelys.com>}
 */
class RelaxedKnitroSolverCsvTest {

    // Columns of the slack CSV export
    private static final int TYPE = 1;
    private static final int TRANSFO = 6;
    private static final int SHUNT = 7;
    private static final int LOAD_VIOLATION = 9;
    private static final int GEN_VIOLATION = 10;

    @TempDir
    Path tmp;

    /**
     * Slack bus B1 linked to an empty bus B2 by a 1 p.u. reactance (Zbase = 400² / 100 = 1600 Ω): even with both
     * voltages at the upper bound (1.5 p.u.), at most 1.5 * 1.5 / 1 = 2.25 p.u. (225 MW) can flow to B2.
     */
    private static Network createNetwork() {
        Network network = Network.create("test", "test");
        Substation s = network.newSubstation().setId("S").add();
        VoltageLevel vl1 = s.newVoltageLevel().setId("VL1").setNominalV(400).setTopologyKind(TopologyKind.BUS_BREAKER).add();
        vl1.getBusBreakerView().newBus().setId("B1").add();
        vl1.newGenerator().setId("G1").setBus("B1").setMinP(0).setMaxP(2000).setTargetP(0).setTargetV(400)
                .setVoltageRegulatorOn(true).add();
        VoltageLevel vl2 = s.newVoltageLevel().setId("VL2").setNominalV(400).setTopologyKind(TopologyKind.BUS_BREAKER).add();
        vl2.getBusBreakerView().newBus().setId("B2").add();
        network.newLine().setId("L12").setBus1("B1").setBus2("B2").setR(0).setX(1600).add();
        return network;
    }

    /**
     * Load flow parameters running the relaxed Knitro solver, with B1 as slack bus and the slacks exported to CSV.
     */
    private LoadFlowParameters createParameters() {
        LoadFlowParameters parameters = new LoadFlowParameters().setDistributedSlack(false);
        OpenLoadFlowParameters.create(parameters)
                .setSlackBusSelectionMode(SlackBusSelectionMode.NAME)
                .setSlackBusesIds(List.of("VL1"))
                .setAcSolverType(KnitroSolverFactory.NAME);
        parameters.addExtension(KnitroLoadFlowParameters.class, new KnitroLoadFlowParameters()
                .setKnitroSolverType(KnitroSolverParameters.SolverType.RELAXED)
                .setExportSolution(tmp.resolve("slacks").toString()));
        return parameters;
    }

    /**
     * Runs the load flow and returns the P (or Q) slack exported for B2.
     */
    private String[] runAndGetSlack(Network network, LoadFlowParameters parameters) throws IOException {
        assertTrue(new LoadFlow.Runner(new OpenLoadFlowProvider(new SparseMatrixFactory())).run(network, parameters).isFullyConverged());

        String exportPath = parameters.getExtension(KnitroLoadFlowParameters.class).getExportSolution();
        return Files.readAllLines(Path.of(exportPath + ".csv")).stream()
                .map(line -> line.split(";", -1))
                .filter(row -> row[0].equals("VL2_0") && row[TYPE].equals("P"))
                .findFirst()
                .orElseThrow(() -> new AssertionError("No " + "P" + " slack exported for VL2_0"));
    }

    @Test
    void testSlackOnBusWithLoadGeneratorAndShunt() throws IOException {
        // The 1000 MW load is cut by the slack (the load stays positive), which would bring the generator below its minP
        Network network = createNetwork();
        VoltageLevel vl2 = network.getVoltageLevel("VL2");
        vl2.newLoad().setId("L2").setBus("B2").setP0(1000).setQ0(100).add();
        vl2.newGenerator().setId("G2").setBus("B2").setMinP(0).setMaxP(20).setTargetP(10).setTargetQ(0)
                .setVoltageRegulatorOn(false).add();
        vl2.newShuntCompensator().setId("SH2").setBus("B2").setSectionCount(1)
                .newLinearModel().setBPerSection(1e-5).setMaximumSectionCount(1).add().add();

        String[] slackP = runAndGetSlack(network, createParameters());
        assertEquals("0", slackP[LOAD_VIOLATION]);
        assertEquals("1", slackP[GEN_VIOLATION]);
        assertFalse(slackP[SHUNT].isEmpty());
    }

    @Test
    void testSlackOnTransformerControlledBusViolatingLoad() throws IOException {
        // A transformer, whose ratio tap changer regulates B2, is added in parallel to the line. The 1000 MW are drawn
        // by a generator acting as a consumer (e.g. pumping), so the slack exceeds the 10 MW load
        Network network = createNetwork();
        VoltageLevel vl2 = network.getVoltageLevel("VL2");
        Load load = vl2.newLoad().setId("L2").setBus("B2").setP0(10).setQ0(0).add();
        vl2.newGenerator().setId("G2").setBus("B2").setMinP(-1000).setMaxP(0).setTargetP(-1000).setTargetQ(0)
                .setVoltageRegulatorOn(false).add();
        network.getSubstation("S").newTwoWindingsTransformer().setId("T12").setBus1("B1").setBus2("B2")
                .setRatedU1(400).setRatedU2(400).setR(0).setX(1600).add()
                .newRatioTapChanger().setLowTapPosition(0).setTapPosition(0).setLoadTapChangingCapabilities(true)
                .setRegulating(true).setRegulationMode(RatioTapChanger.RegulationMode.VOLTAGE).setRegulationValue(400)
                .setTargetDeadband(0).setRegulationTerminal(load.getTerminal())
                .beginStep().setRho(1.0).endStep()
                .add();

        // In incremental mode, the ratio stays fixed during the Knitro solve and is only moved between tap positions
        // by an outer loop. In WITH_GENERATOR_VOLTAGE_CONTROL mode, it would be a continuous, unbounded variable that
        // Knitro could raise to carry the whole consumption without any P slack
        LoadFlowParameters parameters = createParameters().setTransformerVoltageControlOn(true);
        OpenLoadFlowParameters.get(parameters)
                .setTransformerVoltageControlMode(OpenLoadFlowParameters.TransformerVoltageControlMode.INCREMENTAL_VOLTAGE_CONTROL);

        String[] slackP = runAndGetSlack(network, parameters);
        assertEquals("1", slackP[LOAD_VIOLATION]);
        assertFalse(slackP[TRANSFO].isEmpty());
    }

    @Test
    void testSlackOnBusWithoutAnyElement() throws IOException {
        // A 10 p.u. shunt conductance of the line on the B2 side draws at least 10 * 0.5² = 2.5 p.u. (lower voltage
        // bound), more than the 2.25 p.u. the line can carry, although no element is connected to B2
        Network network = createNetwork();
        network.getLine("L12").setG2(10.0 / 1600);

        String[] slackP = runAndGetSlack(network, createParameters());
        assertEquals("0", slackP[LOAD_VIOLATION]);
        assertEquals("0", slackP[GEN_VIOLATION]);
    }
}

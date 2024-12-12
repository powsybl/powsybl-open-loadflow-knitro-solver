/**
 * Copyright (c) 2023, RTE (http://www.rte-france.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.commons.PowsyblException;
import com.powsybl.ieeecdf.converter.IeeeCdfNetworkFactory;
import com.powsybl.iidm.network.Bus;
import com.powsybl.iidm.network.Generator;
import com.powsybl.iidm.network.Network;
import com.powsybl.iidm.network.ReactiveLimits;
import com.powsybl.iidm.network.extensions.PilotPoint;
import com.powsybl.iidm.network.extensions.SecondaryVoltageControl;
import com.powsybl.iidm.network.extensions.SecondaryVoltageControlAdder;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.DenseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.network.LfNetwork;
import com.powsybl.openloadflow.network.LfNetworkParameters;
import com.powsybl.openloadflow.network.LfSecondaryVoltageControl;
import com.powsybl.openloadflow.network.impl.LfNetworkLoaderImpl;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.concurrent.CompletionException;

import static com.powsybl.openloadflow.util.LoadFlowAssert.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * @author Geoffroy Jamgotchian {@literal <geoffroy.jamgotchian at rte-france.com>}
 */
class SecondaryVoltageControlTest {

    private Network network;

    private Bus b4;
    private Bus b6;
    private Bus b8;
    private Bus b10;
    private Generator g6;
    private Generator g8;

    private LoadFlow.Runner loadFlowRunner;

    private LoadFlowParameters parameters;

    private OpenLoadFlowParameters parametersExt;

    @BeforeEach
    void setUp() {
        network = IeeeCdfNetworkFactory.create14();
        b4 = network.getBusBreakerView().getBus("B4");
        b6 = network.getBusBreakerView().getBus("B6");
        b8 = network.getBusBreakerView().getBus("B8");
        b10 = network.getBusBreakerView().getBus("B10");
        g6 = network.getGenerator("B6-G");
        g8 = network.getGenerator("B8-G");

        // adjust reactive limit to avoid generators be to limit
        g8.newMinMaxReactiveLimits()
                .setMinQ(-6)
                .setMaxQ(200)
                .add();

        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new DenseMatrixFactory()));
        parameters = new LoadFlowParameters();
        parametersExt = OpenLoadFlowParameters.create(parameters)
                .setMaxPlausibleTargetVoltage(1.6)
                .setAcSolverType(KnitroSolverFactory.NAME);
    }

    private static double qToK(Generator g) {
        ReactiveLimits limits = g.getReactiveLimits();
        double q = -g.getTerminal().getQ();
        double p = -g.getTerminal().getP();
        return (2 * q - limits.getMaxQ(p) - limits.getMinQ(p))
                / (limits.getMaxQ(p) - limits.getMinQ(p));
    }

    @Test
    void testNoReactiveLimits() {
        parameters.setUseReactiveLimits(false);
        network.newExtension(SecondaryVoltageControlAdder.class)
                .newControlZone()
                .withName("z1")
                .newPilotPoint().withTargetV(13).withBusbarSectionsOrBusesIds(List.of("B10")).add()
                .newControlUnit().withId("B6-G").add()
                .newControlUnit().withId("B8-G").add()
                .add()
                .add();
        SecondaryVoltageControl control = network.getExtension(SecondaryVoltageControl.class);
        PilotPoint pilotPoint = control.getControlZone("z1").orElseThrow().getPilotPoint();

        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertEquals(LoadFlowResult.ComponentResult.Status.CONVERGED, result.getComponentResults().get(0).getStatus());
        assertEquals(3, result.getComponentResults().get(0).getIterationCount());
        assertVoltageEquals(12.611, b10);
        assertVoltageEquals(12.84, b6);
        assertVoltageEquals(21.8, b8);
        assertEquals(0.248, qToK(g6), DELTA_POWER);
        assertEquals(-0.77, qToK(g8), DELTA_POWER);

        parametersExt.setSecondaryVoltageControl(true);

        result = loadFlowRunner.run(network, parameters);
        assertEquals(LoadFlowResult.ComponentResult.Status.CONVERGED, result.getComponentResults().get(0).getStatus());
        assertEquals(8, result.getComponentResults().get(0).getIterationCount());
        assertVoltageEquals(13, b10);
        assertVoltageEquals(12.945, b6);
        assertVoltageEquals(23.839, b8);
        assertEquals(-0.412, qToK(g6), DELTA_POWER);
        assertEquals(-0.412, qToK(g8), DELTA_POWER);

        pilotPoint.setTargetV(13.5);
        result = loadFlowRunner.run(network, parameters);
        assertEquals(LoadFlowResult.ComponentResult.Status.CONVERGED, result.getComponentResults().get(0).getStatus());
        assertEquals(8, result.getComponentResults().get(0).getIterationCount());
        assertVoltageEquals(13.5, b10);
        assertVoltageEquals(13.358, b6);
        assertVoltageEquals(25.621, b8);
        assertEquals(-0.094, qToK(g6), DELTA_POWER);
        assertEquals(-0.094, qToK(g8), DELTA_POWER);

        pilotPoint.setTargetV(12);
        result = loadFlowRunner.run(network, parameters);
        assertEquals(LoadFlowResult.ComponentResult.Status.CONVERGED, result.getComponentResults().get(0).getStatus());
        assertEquals(8, result.getComponentResults().get(0).getIterationCount());
        assertVoltageEquals(12, b10);
        assertVoltageEquals(12.151, b6);
        assertVoltageEquals(20.269, b8);
        assertEquals(-0.932, qToK(g6), DELTA_POWER);
        assertEquals(-0.932, qToK(g8), DELTA_POWER);
    }

    @Test
    void testReactiveLimits() {
        network.newExtension(SecondaryVoltageControlAdder.class)
                .newControlZone()
                .withName("z1")
                .newPilotPoint().withTargetV(11.5).withBusbarSectionsOrBusesIds(List.of("B10")).add()
                .newControlUnit().withId("B6-G").add()
                .newControlUnit().withId("B8-G").add()
                .add()
                .add();

        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertEquals(LoadFlowResult.ComponentResult.Status.CONVERGED, result.getComponentResults().get(0).getStatus());
        assertEquals(3, result.getComponentResults().get(0).getIterationCount());
        assertVoltageEquals(12.606, b10);
        assertVoltageEquals(12.84, b6);
        assertVoltageEquals(21.8, b8);
        assertReactivePowerEquals(-12.730, g6.getTerminal()); // [-6, 24]
        assertReactivePowerEquals(-17.623, g8.getTerminal()); // [-6, 200]

        parametersExt.setSecondaryVoltageControl(true);

        result = loadFlowRunner.run(network, parameters);
        assertEquals(LoadFlowResult.ComponentResult.Status.CONVERGED, result.getComponentResults().get(0).getStatus());
        assertEquals(11, result.getComponentResults().get(0).getIterationCount());

        assertVoltageEquals(11.736, b10); // 11.5 kV was not feasible
        assertVoltageEquals(11.924, b6);
        assertVoltageEquals(19.537, b8);
        assertReactivePowerEquals(6, g6.getTerminal()); // [-6, 24] => qmin
        assertReactivePowerEquals(6, g8.getTerminal()); // [-6, 200] => qmin
    }

    @Test
    void testUnblockGeneratorFromLimit() {
        network.newExtension(SecondaryVoltageControlAdder.class)
                .newControlZone()
                .withName("z1")
                .newPilotPoint().withTargetV(15).withBusbarSectionsOrBusesIds(List.of("B10")).add()
                .newControlUnit().withId("B6-G").add()
                .newControlUnit().withId("B8-G").add()
                .add()
                .add();

        // to put g6 and g8 at q min
        g6.setTargetV(11.8);
        g8.setTargetV(19.5);

        parametersExt.setSecondaryVoltageControl(true);
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters(); // set gradient computation mode
        knitroLoadFlowParameters.setUpperVoltageBound(2.0);  // for this specific test, we allow the voltage to be greater than 1.5 (knitro's default maxRealisticVoltage) for the network to converge
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);

        // try to put g6 and g8 at qmax to see if they are correctly unblock from qmin
        var result = loadFlowRunner.run(network, parameters);
        assertEquals(LoadFlowResult.ComponentResult.Status.CONVERGED, result.getComponentResults().get(0).getStatus());
        assertEquals(21, result.getComponentResults().get(0).getIterationCount());

        assertVoltageEquals(15, b10);
        assertVoltageEquals(14.604, b6);
        assertVoltageEquals(30.744, b8);
        assertReactivePowerEquals(-24, g6.getTerminal()); // [-6, 24] => qmax
        assertReactivePowerEquals(-200, g8.getTerminal()); // [-6, 200] => qmax
    }

    @Test
    void multiNoReactiveLimitsZonesTest() {
        parameters.setUseReactiveLimits(false);
        network.newExtension(SecondaryVoltageControlAdder.class)
                .newControlZone()
                .withName("z1")
                .newPilotPoint().withTargetV(142).withBusbarSectionsOrBusesIds(List.of("B4")).add()
                .newControlUnit().withId("B1-G").add()
                .newControlUnit().withId("B2-G").add()
                .newControlUnit().withId("B3-G").add()
                .add()
                .newControlZone()
                .withName("z2")
                .newPilotPoint().withTargetV(14.5).withBusbarSectionsOrBusesIds(List.of("B10")).add()
                .newControlUnit().withId("B6-G").add()
                .newControlUnit().withId("B8-G").add()
                .add()
                .add();

        parametersExt.setSecondaryVoltageControl(true);
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters(); // set gradient computation mode
        knitroLoadFlowParameters.setUpperVoltageBound(2.0);  // for this specific test, we allow the voltage to be greater than 1.5 (knitro's default maxRealisticVoltage) for the network to converge
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);
        var result = loadFlowRunner.run(network, parameters);
        assertEquals(LoadFlowResult.ComponentResult.Status.CONVERGED, result.getComponentResults().get(0).getStatus());
        assertEquals(9, result.getComponentResults().get(0).getIterationCount());
        assertVoltageEquals(142, b4);
        assertVoltageEquals(14.5, b10);
    }

    @Test
    void testOpenBranchIssue() {
        // open branch L6-13-1 on side 2 so that a neighbor branch of B6-G is disconnected to the other side
        network.getLine("L6-13-1").getTerminal2().disconnect();

        parameters.setUseReactiveLimits(false);
        parametersExt.setSecondaryVoltageControl(true);

        network.newExtension(SecondaryVoltageControlAdder.class)
                .newControlZone()
                .withName("z1")
                .newPilotPoint().withTargetV(14.4).withBusbarSectionsOrBusesIds(List.of("B10")).add()
                .newControlUnit().withId("B6-G").add()
                .newControlUnit().withId("B8-G").add()
                .add()
                .add();

        var result = loadFlowRunner.run(network, parameters);
        assertEquals(LoadFlowResult.ComponentResult.Status.CONVERGED, result.getComponentResults().get(0).getStatus());
        assertEquals(9, result.getComponentResults().get(0).getIterationCount());
        assertVoltageEquals(14.4, b10);
        assertVoltageEquals(14.151, b6);
        assertVoltageEquals(28.913, b8);
    }

    @Test
    void pilotPointNotFoundTest() {
        network.newExtension(SecondaryVoltageControlAdder.class)
                .newControlZone()
                .withName("z1")
                .newPilotPoint().withTargetV(13).withBusbarSectionsOrBusesIds(List.of("XX", "YY")).add()
                .newControlUnit().withId("B6-G").add()
                .newControlUnit().withId("B8-G").add()
                .add();

        LfNetworkParameters networkParameters = new LfNetworkParameters().setSecondaryVoltageControl(true);
        List<LfNetwork> lfNetworks = LfNetwork.load(network, new LfNetworkLoaderImpl(), networkParameters);
        LfNetwork lfNetwork = lfNetworks.get(0);
        assertTrue(lfNetwork.getSecondaryVoltageControls().isEmpty());
    }

    @Test
    void controlUnitNotFoundTest() {
        network.newExtension(SecondaryVoltageControlAdder.class)
                .newControlZone()
                .withName("z1")
                .newPilotPoint().withTargetV(13).withBusbarSectionsOrBusesIds(List.of("B10")).add()
                .newControlUnit().withId("B99-G").add()
                .newControlUnit().withId("B8-G").add()
                .add()
                .add();

        LfNetworkParameters networkParameters = new LfNetworkParameters().setSecondaryVoltageControl(true);
        List<LfNetwork> lfNetworks = LfNetwork.load(network, new LfNetworkLoaderImpl(), networkParameters);
        LfNetwork lfNetwork = lfNetworks.get(0);
        List<LfSecondaryVoltageControl> secondaryVoltageControls = lfNetwork.getSecondaryVoltageControls();
        assertEquals(1, secondaryVoltageControls.size());
        assertEquals(1, secondaryVoltageControls.get(0).getControlledBuses().size()); // B99-G not found
    }

    @Test
    void testOptionalNoValueIssue() {
        network.newExtension(SecondaryVoltageControlAdder.class)
                .newControlZone()
                .withName("z1")
                .newPilotPoint().withTargetV(13).withBusbarSectionsOrBusesIds(List.of("B10")).add()
                .newControlUnit().withId("B6-G").add()
                .newControlUnit().withId("B9-SH").add() // this is a shunt which is not supported
                .add()
                .add();

        parametersExt.setSecondaryVoltageControl(true);

        CompletionException e = assertThrows(CompletionException.class, () -> loadFlowRunner.run(network, parameters));
        assertEquals("Control unit 'B9-SH' of zone 'z1' is expected to be either a generator or a VSC converter station", e.getCause().getMessage());
    }

    @Test
    void testAnotherOptionalNoValueIssue() {
        Generator g6 = network.getGenerator("B6-G");
        g6.newMinMaxReactiveLimits()
                .setMinQ(100)
                .setMaxQ(100.000001)
                .add();

        network.newExtension(SecondaryVoltageControlAdder.class)
                .newControlZone()
                .withName("z1")
                .newPilotPoint().withTargetV(13).withBusbarSectionsOrBusesIds(List.of("B10")).add()
                .newControlUnit().withId("B6-G").add()
                .newControlUnit().withId("B8-G").add()
                .add();

        parametersExt.setSecondaryVoltageControl(true);

        assertDoesNotThrow(() -> loadFlowRunner.run(network, parameters));
    }

    @Test
    void disjointControlZoneTest() {
        network.newExtension(SecondaryVoltageControlAdder.class)
                .newControlZone()
                .withName("z1")
                .newPilotPoint().withTargetV(13).withBusbarSectionsOrBusesIds(List.of("B10")).add()
                .newControlUnit().withId("B99-G").add()
                .newControlUnit().withId("B8-G").add()
                .add()
                .newControlZone()
                .withName("z2")
                .newPilotPoint().withTargetV(13).withBusbarSectionsOrBusesIds(List.of("B10")).add()
                .newControlUnit().withId("B1-G").add()
                .newControlUnit().withId("B8-G").add()
                .add()
                .add();

        LfNetworkParameters networkParameters = new LfNetworkParameters().setSecondaryVoltageControl(true);
        LfNetworkLoaderImpl networkLoader = new LfNetworkLoaderImpl();
        PowsyblException e = assertThrows(PowsyblException.class, () -> LfNetwork.load(network, networkLoader, networkParameters));
        assertEquals("Generator voltage control of controlled bus 'VL8_0' is present in more that one control zone", e.getMessage());
    }

    @Test
    void testNotPlausibleTargetV() {
        // g8 generator is very far from pilot point bus b12, there is no way for generators of the zone to control
        // the pilot point voltage, this is detected by secondary voltage control outer loop which fails
        network.newExtension(SecondaryVoltageControlAdder.class)
                .newControlZone()
                .withName("z1")
                .newPilotPoint().withTargetV(11.5).withBusbarSectionsOrBusesIds(List.of("B12")).add()
                .newControlUnit().withId("B8-G").add()
                .add()
                .add();

        parametersExt.setSecondaryVoltageControl(true);

        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertEquals(LoadFlowResult.ComponentResult.Status.FAILED, result.getComponentResults().get(0).getStatus());
    }
}

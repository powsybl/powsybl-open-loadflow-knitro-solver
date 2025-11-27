/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver.utils;

import com.powsybl.iidm.network.Bus;
import com.powsybl.iidm.network.Network;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.DenseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.knitro.solver.KnitroLoadFlowParameters;
import com.powsybl.openloadflow.knitro.solver.KnitroSolverFactory;
import com.powsybl.openloadflow.network.FourBusNetworkFactory;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static com.powsybl.openloadflow.util.LoadFlowAssert.assertAngleEquals;
import static com.powsybl.openloadflow.util.LoadFlowAssert.assertVoltageEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 */
class KnitroSolverStoppingCriteriaTest {

    private Network network;
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;
    private OpenLoadFlowParameters parametersExt;
    Bus b1;
    Bus b2;
    Bus b3;
    Bus b4;

    @BeforeEach
    void setUp() {
        // Sparse matrix solver only
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new DenseMatrixFactory()));

        // ============= Setting LoadFlow parameters =============
        parameters = new LoadFlowParameters();
        parametersExt = OpenLoadFlowParameters.create(parameters)
                .setAcSolverType(KnitroSolverFactory.NAME);
        // No OLs
        parameters.setBalanceType(LoadFlowParameters.BalanceType.PROPORTIONAL_TO_LOAD);
        parameters.setDistributedSlack(false)
                .setUseReactiveLimits(false);
        parameters.getExtension(OpenLoadFlowParameters.class)
                .setSvcVoltageMonitoring(false);
        network = FourBusNetworkFactory.createWithCondenser();
        b1 = network.getBusBreakerView().getBus("b1");
        b2 = network.getBusBreakerView().getBus("b2");
        b3 = network.getBusBreakerView().getBus("b3");
        b4 = network.getBusBreakerView().getBus("b4");
    }

    @Test
    void testEffectOfConvEpsPerEq() {
        /*
         * Checks the effect of changing Knitro's parameter convEpsPerEq on precision and values, when running Knitro solver
         */

        // ============= Model with default precision =============
        LoadFlowResult knitroResultDefault = loadFlowRunner.run(network, parameters);
        assertSame(LoadFlowResult.ComponentResult.Status.CONVERGED, knitroResultDefault.getComponentResults().get(0).getStatus());
        assertTrue(knitroResultDefault.isFullyConverged());
        assertVoltageEquals(1.0, b1);
        assertAngleEquals(0, b1);
        assertVoltageEquals(0.983834, b2);
        assertAngleEquals(-9.490705, b2);
        assertVoltageEquals(0.983124, b3);
        assertAngleEquals(-13.178514, b3);
        assertVoltageEquals(1.0, b4);
        assertAngleEquals(-6.531907, b4);

        // ============= Model with smaller precision =============
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters(); // set gradient computation mode
        knitroLoadFlowParameters.setRelConvEps(Math.pow(10, -2));
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);

        LoadFlowResult knitroResultLessPrecise = loadFlowRunner.run(network, parameters);

        assertSame(LoadFlowResult.ComponentResult.Status.CONVERGED, knitroResultLessPrecise.getComponentResults().get(0).getStatus());
        assertTrue(knitroResultLessPrecise.isFullyConverged());
        assertVoltageEquals(1.0, b1);
        assertAngleEquals(0, b1);
        assertVoltageEquals(0.983834, b2);
        assertAngleEquals(-9.485945, b2);
        assertVoltageEquals(0.983124, b3);
        assertAngleEquals(-13.170002, b3);
        assertVoltageEquals(1.0, b4);
        assertAngleEquals(-6.530383, b4);
    }
}

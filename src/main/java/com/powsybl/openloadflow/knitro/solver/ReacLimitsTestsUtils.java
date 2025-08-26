/**
 * Copyright (c) 2025, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */

package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.iidm.network.Network;
import com.powsybl.iidm.network.*;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.openloadflow.OpenLoadFlowParameters;

import java.util.*;


import static org.ejml.UtilEjml.assertTrue;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Yoann Anezin {@literal <yoann.anezin at artelys.com>}
 */
public class ReacLimitsTestsUtils {
    private static final double DEFAULT_TOLERANCE = 1e-3;

    public ArrayList<Integer> countAndSwitch(Network network, HashMap<String,Double> listMinQ, HashMap<String,Double> listMaxQ) throws Exception {
        int nmbSwitchQmin = 0;
        int nmbSwitchQmax = 0;
        int previousNmbBusPV = 0;
        ArrayList<Integer> switches = new ArrayList<>();
        for (Generator g : network.getGenerators()) {
            ArrayList<String> busVisited = new ArrayList<>();
            if (g.isVoltageRegulatorOn() && !busVisited.contains(g.getId())) {
                busVisited.add(g.getId());
                previousNmbBusPV += 1;
            }
            Terminal t = g.getTerminal();
            double v = t.getBusView().getBus().getV();
            if (g.isVoltageRegulatorOn()) {
                double Qming = g.getReactiveLimits().getMinQ(g.getTerminal().getBusView().getBus().getP());
                double Qmaxg = g.getReactiveLimits().getMaxQ(g.getTerminal().getBusView().getBus().getP());
                if (!(v + DEFAULT_TOLERANCE > g.getTargetV() && v - DEFAULT_TOLERANCE < g.getTargetV())) {
                    if (-t.getQ() + DEFAULT_TOLERANCE > Qming &&
                            -t.getQ() - DEFAULT_TOLERANCE < Qming) {
                        nmbSwitchQmin++;
                        assertTrue(v > g.getTargetV(), "V below its target on  Qmin switch of bus "
                                + t.getBusView().getBus().getId() + ". Current generator checked : " + g.getId());
                        g.setTargetQ(listMinQ.get(g.getId()));
                    } else if (-t.getQ() + DEFAULT_TOLERANCE > Qmaxg &&
                            -t.getQ() - DEFAULT_TOLERANCE < Qmaxg) {
                        nmbSwitchQmax++;
                        assertTrue(v < g.getTargetV(), "V above its target on a Qmax switch of bus "
                                + t.getBusView().getBus().getId() + ". Current generator checked : " + g.getId());
                        g.setTargetQ(listMaxQ.get(g.getId()));
                    } else {
                        throw new Exception("Value of Q not matching Qmin nor Qmax on the switch of bus "
                                + t.getBusView().getBus().getId() + ". Current generator checked : " + g.getId());
                    }
                    g.setVoltageRegulatorOn(false);

                }
            }
        }
        switches.add(nmbSwitchQmin);
        switches.add(nmbSwitchQmax);
        switches.add(previousNmbBusPV);
        return switches;
    }

    public void verifNewtonRaphson (Network network, LoadFlowParameters parameters, LoadFlow.Runner loadFlowRunner, int nbreIter) {
        OpenLoadFlowParameters.get(parameters).setVoltageInitModeOverride(OpenLoadFlowParameters.VoltageInitModeOverride.NONE);
        parameters.setVoltageInitMode(LoadFlowParameters.VoltageInitMode.PREVIOUS_VALUES);
        OpenLoadFlowParameters.get(parameters).setMaxNewtonRaphsonIterations(nbreIter)
                .setReportedFeatures(Collections.singleton(OpenLoadFlowParameters.ReportedFeatures.NEWTON_RAPHSON_LOAD_FLOW));
        OpenLoadFlowParameters.get(parameters).setAcSolverType("NEWTON_RAPHSON");
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());
    }

    public void checkSwitches(Network network, HashMap<String,Double> listMinQ, HashMap<String,Double> listMaxQ) {
        try {
            ArrayList<Integer> switches = countAndSwitch(network, listMinQ, listMaxQ);
            assertTrue(switches.get(2) > switches.get(1) + switches.get(0),
                    "No control on any voltage magnitude : all buses switched");
            System.out.println(switches.get(0) + " switches to PQ with Q = Qlow and " + switches.get(1) + " with Q = Qup");
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}

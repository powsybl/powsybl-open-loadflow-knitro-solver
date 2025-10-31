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
import com.powsybl.openloadflow.network.PlausibleValues;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import java.util.*;


import static org.ejml.UtilEjml.assertTrue;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Yoann Anezin {@literal <yoann.anezin at artelys.com>}
 */
public final class ReacLimitsTestsUtils {
    private static final double DEFAULT_TOLERANCE = 1e-3;
    private static final double DEFAULT_Q_TOLERANCE = 1e-2;
    private static final Logger LOGGER = LoggerFactory.getLogger(ReacLimitsTestsUtils.class);

    private ReacLimitsTestsUtils() {
        throw new UnsupportedOperationException();
    }

    /**
     * Récupère le réseau après optimisation et vérifie les switchs PV - PQ effectués. On parcourt les générateurs du
     * réseau et on vérifie si la target de l'un d'entre eux n'est pas respecté.
     * Si c'est le cas, c'est qu'un switch a dû avoir lieu, on vérifie alors la puissance réactive du bus controlé
     * Dans les calculs il faut surement prendre en compte les slacks (à vérifier lesquels)
     *
     */
    public static ArrayList<Integer> countAndSwitch(Network network, HashMap<String, Double> listMinQ, HashMap<String, Double> listMaxQ,
                                                    HashMap<String, Double> slacksP, HashMap<String, Double> slacksQ, HashMap<String, Double> slacksV) throws Exception {
        int nmbSwitchQmin = 0;
        int nmbSwitchQmax = 0;
        int previousNmbBusPV = 0;
        ArrayList<Integer> switches = new ArrayList<>();
        HashMap<String, String> visitedBuses = new HashMap<>();
        List<String> listBusSwitched = new ArrayList<>();
        for (Generator g : network.getGenerators()) {
            if (g.getRegulatingTerminal().getBusView().getBus() == null || !g.isVoltageRegulatorOn()) {
                continue;
            }
            String idbus = g.getRegulatingTerminal().getBusView().getBus().getId();
            Double slackP = slacksP.get(idbus);
            Double slackQ = slacksQ.get(idbus);
            Double slackV = slacksV.get(idbus);
            if (slackP == null) {
                slackP = 0.0;
            }
            if (slackQ == null) {
                slackQ = 0.0;
            }
            if (slackV == null) {
                slackV = 0.0;
            }
            Terminal t = g.getTerminal();
            Terminal regulatingTerm = g.getRegulatingTerminal();

            double v = regulatingTerm.getBusView().getBus().getV();
            double qMing = g.getReactiveLimits().getMinQ(g.getTargetP());
            double qMaxg = g.getReactiveLimits().getMaxQ(g.getTargetP());
            if (Math.abs(qMing - qMaxg) < PlausibleValues.MIN_REACTIVE_RANGE) {
                continue;
            }
            if (Math.abs(qMing) > PlausibleValues.MAX_REACTIVE_RANGE || Math.abs(qMaxg) > PlausibleValues.MAX_REACTIVE_RANGE) {
                continue;
            }

            if (visitedBuses.containsKey(idbus)) {
                switch (visitedBuses.get(idbus)) {
                    case "Switch Qmin":
                        assertTrue(v + slackV > g.getTargetV(), "Another generator did a Qmin switch," +
                                " expected the same thing to happened. Current generator : " + g.getId() + " on bus " + idbus);
                        assertTrue(-t.getQ() + DEFAULT_Q_TOLERANCE > qMing && -t.getQ() - DEFAULT_Q_TOLERANCE < qMing,
                                "Another generator did a Qmin switch, expected the same thing to happened. " +
                                        "Current generator : " + g.getId() + " on bus " + idbus);
                        break;
                    case "Switch Qmax":
                        assertTrue(v + slackV < g.getTargetV(), "Another generator did a Qmax switch," +
                                " expected the same thing to happened. Current generator : " + g.getId() + " on bus " + idbus);
                        assertTrue(-t.getQ() + DEFAULT_Q_TOLERANCE > qMaxg && -t.getQ() - DEFAULT_Q_TOLERANCE < qMaxg,
                                "Another generator did a Qmax switch, expected the same thing to happened. " +
                                        "Current generator : " + g.getId() + " on bus " + idbus);
                        break;
                    case "No Switch":
                        assertTrue(v + slackV + DEFAULT_TOLERANCE > g.getTargetV() && v + slackV - DEFAULT_TOLERANCE < g.getTargetV());
                }
                continue;
            }
            previousNmbBusPV++;
            if (!(v + slackV + DEFAULT_TOLERANCE > g.getTargetV() && v + slackV - DEFAULT_TOLERANCE < g.getTargetV())) {

                if ((-t.getQ() + 2*DEFAULT_Q_TOLERANCE > qMing &&
                        -t.getQ() - 2*DEFAULT_Q_TOLERANCE < qMing) && qMing != qMaxg) {
                    nmbSwitchQmin++;
                    listBusSwitched.add(idbus);
                    if (!(v + slackV > g.getTargetV())) {
                        LOGGER.warn("V ( " + v + slackV + " ) below its target ( " + g.getTargetV() + " ) on a Qmin switch of bus "
                                + t.getBusView().getBus().getId() + ". Current generator checked : " + g.getId());
                    }
                    visitedBuses.put(idbus, "Switch Qmin");
                } else if ((-t.getQ() + DEFAULT_Q_TOLERANCE > qMaxg &&
                        -t.getQ() - DEFAULT_Q_TOLERANCE < qMaxg) && qMing != qMaxg) {
                    nmbSwitchQmax++;
                    listBusSwitched.add(idbus);
                    if (!(v + slackV < g.getTargetV())) {
                        LOGGER.warn("V ( " + v + slackV + " ) above its target ( " + g.getTargetV() + " ) on a Qmax switch of bus "
                                + t.getBusView().getBus().getId() + ". Current generator checked : " + g.getId());
                    }
                    visitedBuses.put(idbus, "Switch Qmax");
                } else if ((-t.getQ() + DEFAULT_Q_TOLERANCE > qMaxg &&
                        -t.getQ() - DEFAULT_Q_TOLERANCE < qMaxg) && qMaxg == qMing) {
                    if (v + slackV > g.getTargetV()) {
                        nmbSwitchQmin++;
                        listBusSwitched.add(idbus);
                        visitedBuses.put(idbus, "Switch Qmin");
                    } else {
                        nmbSwitchQmax++;
                        listBusSwitched.add(idbus);
                        visitedBuses.put(idbus, "Switch Qmax");
                    }
                } else {
                    assertTrue((-t.getQ() + DEFAULT_Q_TOLERANCE > qMaxg && -t.getQ() - DEFAULT_Q_TOLERANCE < qMaxg) ||
                                    (-t.getQ() + DEFAULT_Q_TOLERANCE > qMing && -t.getQ() - DEFAULT_Q_TOLERANCE < qMing),
                            "Value of Q ( " + -t.getQ() + " ) not matching Qmin ( " + qMing + " )  nor Qmax ( "
                                    + qMaxg + " ) on the switch of bus " + t.getBusView().getBus().getId() +
                                    ". Current generator checked : " + g.getId());
                }
            } else {
                visitedBuses.put(idbus, "No Switch");
            }
        }
        switches.add(nmbSwitchQmin);
        switches.add(nmbSwitchQmax);
        switches.add(previousNmbBusPV);
        applicationStacks(network, slacksP, slacksQ, slacksV, listBusSwitched);
        return switches;
    }

    /**
     * Apply slacks used to the values of the network to make an accurate checker (doesn't work better)
     */
    private static void applicationStacks(Network network, HashMap<String, Double> slacksP, HashMap<String,
            Double> slacksQ, HashMap<String, Double> slacksV, List<String> listBusSwitched) {
        for (Generator g : network.getGenerators()) {
            if (g.getRegulatingTerminal().getBusView().getBus() == null || !g.isVoltageRegulatorOn()) {
                continue;
            }
            String idbus = g.getRegulatingTerminal().getBusView().getBus().getId();
            Double slackP = slacksP.get(idbus);
            Double slackQ = slacksQ.get(idbus);
            Double slackV = slacksV.get(idbus);

            if (slackP == null) {
                slackP = 0.0;
            }
            if (slackQ == null) {
                slackQ = 0.0;
            }
            if (slackV == null) {
                slackV = 0.0;
            }

            Terminal t = g.getTerminal();
            Terminal regulatingTerm = g.getRegulatingTerminal();
            double p = t.getP();
            double q = t.getQ();
            double v = regulatingTerm.getBusView().getBus().getV();


//            g.setTargetP(p);
//            g.setTargetQ(q);
            g.setTargetV(v);
        }
    }

    /**
     * Verification of the voltage values of the network post optimization with NR
     * @param network       network post optimisation
     * @param parameters    parameters
     * @param loadFlowRunner
     * @param nbreIter      max iteration of the NR methode, usually set at 0 (if so, need a personal olf's version)
     */
    public static void verifNewtonRaphson(Network network, LoadFlowParameters parameters, LoadFlow.Runner loadFlowRunner, int nbreIter) {
        OpenLoadFlowParameters.get(parameters).setVoltageInitModeOverride(OpenLoadFlowParameters.VoltageInitModeOverride.NONE);
        parameters.setVoltageInitMode(LoadFlowParameters.VoltageInitMode.PREVIOUS_VALUES);
        OpenLoadFlowParameters.get(parameters).setMaxNewtonRaphsonIterations(nbreIter)
                .setReportedFeatures(Collections.singleton(OpenLoadFlowParameters.ReportedFeatures.NEWTON_RAPHSON_LOAD_FLOW));
        OpenLoadFlowParameters.get(parameters).setAcSolverType("NEWTON_RAPHSON");
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());
    }

    /**
     * Reading of slacks variable used. Then check the switches made in the network
     * @param network   network post optimization
     * @param listMinQ  list of all the lower bound on Q on each generator
     * @param listMaxQ  liste of all the upper bound on Q on each generator
     */
    public static void checkSwitches(Network network, HashMap<String, Double> listMinQ, HashMap<String, Double> listMaxQ) {
        HashMap<String, Double> slacksP = new HashMap<String, Double>();
        HashMap<String, Double> slacksQ = new HashMap<String, Double>();
        HashMap<String, Double> slacksV = new HashMap<String, Double>();
        ArrayList<String> slacksfiles = new ArrayList<String>();
        slacksfiles.add("P");
        slacksfiles.add("Q");
        slacksfiles.add("V");
        for (String type : slacksfiles) {
            try (BufferedReader br = new BufferedReader(new FileReader("D:\\Documents\\Slacks\\Slacks" + type + ".txt"))) {
                String ligne;
                boolean isId = true; // True = on attend un identifiant, False = on attend une valeur
                String id = "";
                Double value;

                while ((ligne = br.readLine()) != null) {
                    if (ligne.trim().isEmpty()) {
                        continue; // ignorer les lignes vides éventuelles
                    }

                    if (isId) {
                        id = ligne.trim();
                    } else {
                        // Convertir la valeur en double
                        try {
                            value = Double.parseDouble(ligne.trim().replace(',', '.'));
                            switch (type) {
                                case "P":
                                    slacksP.put(id, value);
                                    break;
                                case "Q":
                                    slacksQ.put(id, value);
                                    break;
                                case "V":
                                    slacksV.put(id, value);
                                    break;
                            }
                        } catch (NumberFormatException e) {
                            System.err.println("Valeur non numérique : " + ligne);
                        }
                    }
                    isId = !isId; // alterner ID/valeur
                }

            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        try {
            ArrayList<Integer> switches = countAndSwitch(network, listMinQ, listMaxQ, slacksP, slacksQ, slacksV);
            assertTrue(switches.get(2) > switches.get(1) + switches.get(0),
                    "No control on any voltage magnitude : all buses switched");
            System.out.println(switches.get(0) + " switches to PQ with Q = Qlow and " + switches.get(1) + " with Q = Qup");
        } catch (Exception e) {
            for (String type : slacksfiles) {
                try (FileWriter fw = new FileWriter("D:\\Documents\\Slacks\\Slacks" + type + ".txt", false)) {
                    fw.write("");
                } catch (IOException e1) {
                    e1.printStackTrace();
                }
            }
            throw new RuntimeException(e);
        }

//        for (String type : slacksfiles) {
//            try (FileWriter fw = new FileWriter("D:\\Documents\\Slacks\\Slacks" + type + ".txt", false)) {
//                fw.write("");
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//        }
    }

    /**
     * Creates an active power perturbation of a given network.
     *
     * @param network      The network to perturb.
     * @param alpha        The active load mismatch to apply.
     */
    public static void applyActivePowerPerturbation(Network network, double alpha) {
        for (Load load : network.getLoads()) {
            load.setP0(alpha * load.getP0());
        }
    }
}

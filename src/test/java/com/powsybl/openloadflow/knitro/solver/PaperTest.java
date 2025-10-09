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
import com.powsybl.iidm.network.ReactiveLimits;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.SparseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
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
public class PaperTest {
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;
    private KnitroLoadFlowParameters knitroLoadFlowParameters;

    // Chemin du dossier où écrire les logs
    private static String path = "D:\\Documents\\Logs_Tests\\Logs_";
    // Choix de la Perturbation à effectuer : None / ActivePowerLocal / ActivePowerGlobal / ReactivePower
    private static String Perturbation = "None";

    public PaperTest() throws IOException {
    }

    private int fixReacLim(Network network, HashMap<String, Double> listMinQ, HashMap<String, Double> listMaxQ,
                           KnitroWritter knitroWritter) {
        int numbreLimReacAdded = 0;
        for (var g : network.getGenerators()) {
            if (g.getReactiveLimits().getMinQ(g.getTargetP()) > -1.7976931348623157E308) { //TODO Remplacer cette horrible valeure
                listMinQ.put(g.getId(), g.getReactiveLimits().getMinQ(g.getTargetP()));
                listMaxQ.put(g.getId(), g.getReactiveLimits().getMaxQ(g.getTargetP()));
            } else {
                knitroWritter.write("Bus " + g.getTerminal().getBusView().getBus().getId() +
                        " has no limits on reactive power", true);
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
//
        HashMap<String, Double> listMinQ = new HashMap<>();
        HashMap<String, Double> listMaxQ = new HashMap<>();
        parameters.setUseReactiveLimits(true);

        // Ecriture des paramètres initiaux
        knitroWritter.write("[" + LocalDateTime.now() + "]", false);
        int numbreLimReacAdded = fixReacLim(network, listMinQ, listMaxQ, knitroWritter);
        knitroWritter.write(numbreLimReacAdded + " bus pour lesquels les limites réactif ont été ajoutées", true);
        logsWriting(knitroLoadFlowParameters, knitroWritter);

        // Ecriture de la pertubation effectuée
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
                knitroWritter.write("Perturbed by power injection by a shunt (Target Q = " + perturbValue + ")", true);
                break;
            case "None":
                // TODO: to be uncomment if you want to add perturbations
//                network.getLoadStream().forEach(
//                        l -> {
//                            double kq = 1.3;
//                            double kp = 1.0;
//                            l.setP0(l.getP0() * kp);
//                            l.setQ0(l.getQ0() * kq);
//                        }
//                );
//                network.getLines().forEach(
//                        b -> {
//                            double kr = 1.3;
//                            double kx = 0.85;
//                            b.setR(b.getR() * kr);
//                            b.setX(b.getX() * kx);
//                        }
//                );
//                network.getTwoWindingsTransformers().forEach(
//                        b -> {
//                            double kr = 1.3;
//                            double kx = 0.85;
//                            b.setR(b.getR() * kr);
//                            b.setX(b.getX() * kx);
//                        }
//                );
//                network.getGeneratorStream().forEach(
//                        g -> {
//                            ReactiveLimits rl = g.getReactiveLimits();
//                            rl.getMinQ(g.getTargetP());
//                            rl.getMaxQ(g.getTargetP());
//                            g.newMinMaxReactiveLimits()
//                                    .setMinQ(rl.getMinQ(g.getTargetP()) * 0.95)
//                                    .setMaxQ(rl.getMaxQ(g.getTargetP()) * 0.95)
//                                    .add();
//                        }
//                );

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

        // Ecriture des dernières datas
        long end = System.nanoTime();
        knitroWritter.write("Durée du test : " + (end - start) * 1e-9 + " secondes", true);
        knitroWritter.write("Nombre d'itérations : " + result.getComponentResults().get(0).getIterationCount(), true);
        knitroWritter.write("Status à l'arrivée : " + result.getComponentResults().get(0).getStatus().name(), true);

        // add voltage quality index
        double vqi = 0;
        for (var b : network.getBusView().getBuses()) {
            vqi += Math.abs(b.getV() / b.getVoltageLevel().getNominalV() - 1.0);
        }
        int n = network.getBusView().getBusStream().toList().size();
        System.out.println("ici = " + vqi / n);
    }

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new SparseMatrixFactory()));
        parameters = new LoadFlowParameters().setUseReactiveLimits(true)
                .setDistributedSlack(false);
        knitroLoadFlowParameters = new KnitroLoadFlowParameters(); // set gradient computation mode
        knitroLoadFlowParameters.setGradientComputationMode(1);
        knitroLoadFlowParameters.setMaxIterations(2000);
        knitroLoadFlowParameters.setKnitroSolverType(KnitroSolverParameters.KnitroSolverType.REACTIVLIMITS);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);
        OpenLoadFlowParameters.create(parameters).setAcSolverType(KnitroSolverFactory.NAME);
//        OpenLoadFlowParameters.get(parameters).setVoltageInitModeOverride(OpenLoadFlowParameters.VoltageInitModeOverride.FULL_VOLTAGE);

    }

    @Test
    public void ieee14() {
        String logFile = path + "ieee14_" + Perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create14();
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee30() {
        String logFile = path + "ieee30_" + Perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create30();
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee118() {
        String logFile = path + "ieee118_" + Perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create118();
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee300() {
        String logFile = path + "ieee300_" + Perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create300();
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    void testxiidm1888() throws IOException {
        String logFile = path + "Rte1888_" + Perturbation + ".txt";
        Network network = Network.read("D:\\Documents\\Réseaux\\rte1888.xiidm");
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    void testxiidm6515() throws IOException {
        String logFile = path + "Rte6515_" + Perturbation + ".txt";
        Network network = Network.read("D:\\Documents\\Réseaux\\rte6515.xiidm");
        testprocess(logFile, network, Perturbation, 1.2);
    }

//    @Test
//    void paretoFront() throws IOException {
//        double wPower = 1;
////        double[] wV = {0.0, 0.05, 0.1, 0.25, 0.5, 1, 2, 5, 10, 15};
//        for (int i = 0; i < 1000; i++) {
//            String logFile = path + "ieee118_" + Perturbation + i + ".txt";
//            knitroLoadFlowParameters.setSlackPenalV(0.5 + i * 0.0075)
//                    .setSlackPenalP(wPower)
//                    .setSlackPenalQ(wPower);
//            Network network = IeeeCdfNetworkFactory.create118();
//            testprocess(logFile, network, Perturbation, 1.2);
//            parameters.setVoltageInitMode(LoadFlowParameters.VoltageInitMode.PREVIOUS_VALUES);
//        }
//    }

    @Test
    public void ieee118WithAblation() {
        String logFile = path + "ieee118_" + Perturbation + ".txt";

        // verify that will all mismatches this will work well
        Network network = IeeeCdfNetworkFactory.create118();
        testprocess(logFile, network, Perturbation, 1.2);
//        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);

        // verify results without penalty on V
        logFile = path + "ieee118_" + Perturbation + "_withoutV" + ".txt";
        network = IeeeCdfNetworkFactory.create118();
        knitroLoadFlowParameters.setWithPenalV(false);
        testprocess(logFile, network, Perturbation, 1.2);
//        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);

        // verify results without penalty on P/Q
        logFile = path + "ieee118_" + Perturbation + "_withoutPQ" + ".txt";
        network = IeeeCdfNetworkFactory.create118();
        knitroLoadFlowParameters.setWithPenalPQ(false);
        knitroLoadFlowParameters.setWithPenalV(true);
        testprocess(logFile, network, Perturbation, 1.2);
//        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }
}
 
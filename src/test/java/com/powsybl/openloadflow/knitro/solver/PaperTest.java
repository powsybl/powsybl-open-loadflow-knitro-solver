/**
 * Copyright (c) 2025, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.iidm.network.Bus;
import com.powsybl.iidm.network.Network;
import com.powsybl.ieeecdf.converter.IeeeCdfNetworkFactory;
import com.powsybl.iidm.network.ReactiveLimits;
import com.powsybl.iidm.network.ShuntCompensatorLinearModel;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.SparseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.network.SlackBusSelectionMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.io.IOException;
import java.time.LocalDateTime;

import java.util.*;
import java.util.function.Function;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assumptions.assumeFalse;

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

    void testprocess(String logFile, Network network, String perturbProcess, double perturbValue) {
        testprocess(logFile, network, perturbProcess, perturbValue, () -> {});
    }

    /**
     *Start all the test process and writes logs by the same time
     * @param logFile           file where logs are written
     * @param network           network of work
     * @param perturbProcess    Indicates the perturbation to apply. Current possible choices : ActivePowerGlobal,
     *                          ActivePowerLocal, ReactivePower, None
     * @param perturbValue      value applied in the pertubation chosen (wont be used in the "None" case)
     */
    void testprocess(String logFile, Network network, String perturbProcess, double perturbValue, Runnable perturb) {
        long start = System.nanoTime();

        KnitroWritter knitroWritter = new KnitroWritter(logFile);
        KnitroLoadFlowParameters knitroLoadFlowParameters = parameters.getExtension(KnitroLoadFlowParameters.class);
        knitroLoadFlowParameters.setKnitroWritter(knitroWritter);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);

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
                perturb.run();
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
    }

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new SparseMatrixFactory()));
        parameters = new LoadFlowParameters().setUseReactiveLimits(true)
                .setDistributedSlack(false);
        knitroLoadFlowParameters = new KnitroLoadFlowParameters(); // set gradient computation mode
        knitroLoadFlowParameters.setGradientComputationMode(1);
        knitroLoadFlowParameters.setKnitroSolverType(KnitroSolverParameters.KnitroSolverType.REACTIVLIMITS);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);
        OpenLoadFlowParameters.create(parameters).setAcSolverType(KnitroSolverFactory.NAME);

    }

    @Test
    public void ieeePertubation14() {
        String logFile = path + "ieee14_perturb.txt";
        Network network = IeeeCdfNetworkFactory.create14();

        Runnable perturb = () -> {
            network.getLoadStream().forEach(
                    l -> {
                        double kq = 2;
                        double kp = 2;
                        l.setP0(l.getP0() * kp);
                        l.setQ0(l.getQ0() * kq);
                    }
            );
            network.getLines().forEach(
                    b -> {
                        double kr = 1.5;
                        double kx = 0.8;
                        b.setR(b.getR() * kr);
                        b.setX(b.getX() * kx);
                    }
            );
            network.getTwoWindingsTransformers().forEach(
                    b -> {
                        double kr = 1.5;
                        double kx = 0.8;
                        b.setR(b.getR() * kr);
                        b.setX(b.getX() * kx);
                    }
            );
        };

        testprocess(logFile, network, Perturbation, 1.2, perturb);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieeePertubation30() {
        String logFile = path + "ieee30_perturb.txt";
        Network network = IeeeCdfNetworkFactory.create30();

        Runnable perturb = () -> {
            network.getLoadStream().forEach(
                    l -> {
                        double kq = 1.2;
                        double kp = 1.1;
                        l.setP0(l.getP0() * kp);
                        l.setQ0(l.getQ0() * kq);
                    }
            );
            network.getLines().forEach(
                    b -> {
                        double kr = 1.5;
                        double kx = 0.8;
                        b.setR(b.getR() * kr);
                        b.setX(b.getX() * kx);
                    }
            );
            network.getTwoWindingsTransformers().forEach(
                    b -> {
                        double kr = 1.5;
                        double kx = 0.8;
                        b.setR(b.getR() * kr);
                        b.setX(b.getX() * kx);
                    }
            );
        };

        testprocess(logFile, network, Perturbation, 1.2, perturb);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieeePertubation118() {
        String logFile = path + "ieee118_perturb.txt";
        Network network = IeeeCdfNetworkFactory.create118();
        Runnable perturb = () -> {
            network.getLoadStream().forEach(
                    l -> {
                        double kq = 1.2;
                        double kp = 1.1;
                        l.setP0(l.getP0() * kp);
                        l.setQ0(l.getQ0() * kq);
                    }
            );
            network.getLines().forEach(
                    b -> {
                        double kr = 1.5;
                        double kx = 0.6;
                        b.setR(b.getR() * kr);
                        b.setX(b.getX() * kx);
                    }
            );
            network.getTwoWindingsTransformers().forEach(
                    b -> {
                        double kr = 1.5;
                        double kx = 0.6;
                        b.setR(b.getR() * kr);
                        b.setX(b.getX() * kx);
                    }
            );
        };
        testprocess(logFile, network, Perturbation, 1.2, perturb);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieeePertubation300() {
        String logFile = path + "ieee300_perturb.txt";
        Network network = IeeeCdfNetworkFactory.create300();


        Runnable perturb = () -> {
            network.getLoadStream().forEach(
                    l -> {
                        double kq = 1.2;
                        double kp = 1.1;
                        l.setP0(l.getP0() * kp);
                        l.setQ0(l.getQ0() * kq);
                    }
            );
            network.getLines().forEach(
                    b -> {
                        double kr = 1.5;
                        double kx = 0.6;
                        b.setR(b.getR() * kr);
                        b.setX(b.getX() * kx);
                    }
            );
            network.getTwoWindingsTransformers().forEach(
                    b -> {
                        double kr = 1.5;
                        double kx = 0.6;
                        b.setR(b.getR() * kr);
                        b.setX(b.getX() * kx);
                    }
            );
        };

        testprocess(logFile, network, Perturbation, 1.2, perturb);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 0);
    }

    @Test
    void testxiidmPertubation1888() throws IOException {
        String logFile = path + "rte1888_perturb.txt";
        Network network = Network.read("C:\\Users\\parvy\\Downloads\\rte1888.xiidm");
        Runnable perturb = () -> {

            network.getLoadStream().forEach(
                    l -> {
                        double kq = 1.3;
                        double kp = 1.15;
                        l.setP0(l.getP0() * kp);
                        l.setQ0(l.getQ0() * kq);
                    }
            );
            network.getLines().forEach(
                    b -> {
                        double kr = 1.5;
                        double kx = 0.8;
                        b.setR(b.getR() * kr);
                        b.setX(b.getX() * kx);
                    }
            );
            network.getTwoWindingsTransformers().forEach(
                    b -> {
                        double kr = 1.5;
                        double kx = 0.8;
                        b.setR(b.getR() * kr);
                        b.setX(b.getX() * kx);
                    }
            );
        };

        testprocess(logFile, network, Perturbation, 1.2, perturb);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    void testxiidmPertubation6515() throws IOException {
        String logFile = path + "rte6515_perturb.txt";
        Network network = Network.read("C:\\Users\\parvy\\Downloads\\rte6515.xiidm");

        Runnable perturb = () -> {
            network.getLoadStream().forEach(
                    l -> {
                        double kq = 1.2;
                        double kp = 1.1;
                        l.setP0(l.getP0() * kp);
                        l.setQ0(l.getQ0() * kq);
                    }
            );
            network.getLines().forEach(
                    b -> {
                        double kr = 1.5;
                        double kx = 0.6;
                        b.setR(b.getR() * kr);
                        b.setX(b.getX() * kx);
                    }
            );
            network.getTwoWindingsTransformers().forEach(
                    b -> {
                        double kr = 1.5;
                        double kx = 0.6;
                        b.setR(b.getR() * kr);
                        b.setX(b.getX() * kx);
                    }
            );
        };

        testprocess(logFile, network, Perturbation, 1.2, perturb);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }




    /// Case with NR converges and same solution than knitro
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
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 0);
    }

    @Test
    void testxiidm1888() throws IOException {
        String logFile = path + "Rte1888_" + Perturbation + ".txt";
        Network network = Network.read("C:\\Users\\parvy\\Downloads\\rte1888.xiidm");
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    void testxiidm6515() throws IOException {
        String logFile = path + "Rte6515_" + Perturbation + ".txt";
        Network network = Network.read("C:\\Users\\parvy\\Downloads\\rte6515.xiidm");
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
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











    // Case NR divergence but Knitro converges without slack
    @Test
    public void ieee14NRDiverge() {
        String logFile = path + "ieee14_NR_diverge" + Perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create14();

        var g = network.getGenerator("B6-G");
        double newT = 6.74624289;
        System.out.println("Bus = " + g.getTerminal().getBusView().getBus().getId());
        System.out.println("Target original (kV) = " + g.getTargetV());
        System.out.println("Target original (p.u.) = " + g.getTargetV() / g.getTerminal().getVoltageLevel().getNominalV());
        System.out.println("Target new (kV) = " + newT);
        System.out.println("Target new (kV) = " + newT / g.getTerminal().getVoltageLevel().getNominalV());
        System.out.println("Delta = " + (newT - g.getTargetV()) / g.getTargetV());

        g.setTargetV(newT); // makes NR diverge
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee30NRDiverge() {
        String logFile = path + "ieee30_NR_diverge" + Perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create30();

        var g = network.getGenerator("B13-G");
        double newT = 6.575865;
        System.out.println("Bus = " + g.getTerminal().getBusView().getBus().getId());
        System.out.println("Target original (kV) = " + g.getTargetV());
        System.out.println("Target original (p.u.) = " + g.getTargetV() / g.getTerminal().getVoltageLevel().getNominalV());
        System.out.println("Target new (kV) = " + newT);
        System.out.println("Target new (kV) = " + newT / g.getTerminal().getVoltageLevel().getNominalV());
        System.out.println("Delta = " + (newT - g.getTargetV()) / g.getTargetV());

        g.setTargetV(newT); // makes NR diverge
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee118NRDiverge() {
        String logFile = path + "ieee118_NR_diverge" + Perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create118();

        var g = network.getGenerator("B110-G");
        double newT = 148.96793;
        System.out.println("Bus = " + g.getTerminal().getBusView().getBus().getId());
        System.out.println("Target original (kV) = " + g.getTargetV());
        System.out.println("Target original (p.u.) = " + g.getTargetV() / g.getTerminal().getVoltageLevel().getNominalV());
        System.out.println("Target new (kV) = " + newT);
        System.out.println("Target new (kV) = " + newT / g.getTerminal().getVoltageLevel().getNominalV());
        System.out.println("Delta = " + (newT - g.getTargetV()) / g.getTargetV());

        g.setTargetV(newT); // makes NR diverge
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    public void ieee300NRDiverge() {
        String logFile = path + "ieee300_NR_diverge" + Perturbation + ".txt";
        Network network = IeeeCdfNetworkFactory.create300();

        var g = network.getGenerator("B7062-G");
        double newT = 12.395289;
        System.out.println("Bus = " + g.getTerminal().getBusView().getBus().getId());
        System.out.println("Target original (kV) = " + g.getTargetV());
        System.out.println("Target original (p.u.) = " + g.getTargetV() / g.getTerminal().getVoltageLevel().getNominalV());
        System.out.println("Target new (kV) = " + newT);
        System.out.println("Target new (kV) = " + newT / g.getTerminal().getVoltageLevel().getNominalV());
        System.out.println("Delta = " + (newT - g.getTargetV()) / g.getTargetV());

        g.setTargetV(newT); // makes NR diverge
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    // TODO : find cases for RTE cases

    @Test
    void rte1888NRDiverge() throws IOException {
        String logFile = path + "Rte1888_NR_diverge" + Perturbation + ".txt";
        Network network = Network.read("C:\\Users\\parvy\\Downloads\\rte1888.xiidm");
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }

    @Test
    void rte6515NRDiverge() throws IOException {
        String logFile = path + "Rte6515_" + Perturbation + ".txt";
        Network network = Network.read("C:\\Users\\parvy\\Downloads\\rte6515.xiidm");
        testprocess(logFile, network, Perturbation, 1.2);
        ReacLimitsTestsUtils.verifNewtonRaphson(network, parameters, loadFlowRunner, 20);
    }



    /////// Case with switch as a degree of optimization freedom
    private static final String RKN = "KNITRO";
    private static final String NR = "NEWTON_RAPHSON";
    private static final String VOLTAGE_PERTURBATION = "voltage-perturbation";

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testVoltagePerturbationOnVariousI3ENetworks(NetworkProviders.NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.85;

        String logFile = nrNetwork.getId()+"VoltagePerturbation2.txt";
        voltagePerturbationTest(rknNetwork, nrNetwork, baseFilename, rPU, xPU, alpha, logFile);
    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on RTE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    void testVoltagePerturbationOnRteNetworks(NetworkProviders.NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.85;

        String logFile = nrNetwork.getId()+"VoltagePerturbation2.txt";
        voltagePerturbationTest(rknNetwork, nrNetwork, baseFilename, rPU, xPU, alpha, logFile);
    }

    private void voltagePerturbationTest(Network rknNetwork, Network nrNetwork, String baseFilename, double rPU, double xPU, double alpha, String logFile) {
        PerturbationFactory.VoltagePerturbation perturbation = PerturbationFactory.getVoltagePerturbation(nrNetwork);
        perturbation.print();
        PerturbationFactory.applyVoltagePerturbation(rknNetwork, perturbation, rPU, xPU, alpha);
        PerturbationFactory.applyVoltagePerturbation(nrNetwork, perturbation, rPU, xPU, alpha);
        compareResilience(rknNetwork, nrNetwork, baseFilename, VOLTAGE_PERTURBATION, logFile);
    }

    private void configureSolver(String solver) {
        OpenLoadFlowParameters.create(parameters)
                .setAcSolverType(solver);

        if (RKN.equals(solver)) {
            parameters.getExtension(KnitroLoadFlowParameters.class).setKnitroSolverType(KnitroSolverParameters.KnitroSolverType.RESILIENT);
        }
    }

    private void compareResilience(Network rknNetwork, Network nrNetwork, String baseFilename, String perturbationType, String logFile) {
        // Newton-Raphson
        configureSolver(NR);
        LoadFlowResult resultNR = loadFlowRunner.run(nrNetwork, parameters);
        boolean isConvergedNR = resultNR.isFullyConverged();
        boolean isFailedNR = resultNR.isFailed();

        long start = System.nanoTime();

        KnitroWritter knitroWritter = new KnitroWritter(logFile);
        KnitroLoadFlowParameters knitroLoadFlowParameters = parameters.getExtension(KnitroLoadFlowParameters.class);
        knitroLoadFlowParameters.setKnitroWritter(knitroWritter);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);
        parameters.setUseReactiveLimits(true);

        // Ecriture des paramètres initiaux
        logsWriting(knitroLoadFlowParameters, knitroWritter);

        assumeFalse(isConvergedNR && !isFailedNR, baseFilename + ": NR should not converge");

        HashMap<String, Double> listMinQ = new HashMap<>();
        HashMap<String, Double> listMaxQ = new HashMap<>();
        parameters.setUseReactiveLimits(true);

        // Ecriture des paramètres initiaux
        knitroWritter.write("[" + LocalDateTime.now() + "]", false);
        int numbreLimReacAdded = fixReacLim(rknNetwork, listMinQ, listMaxQ, knitroWritter);


        // Knitro Resilient
        configureSolver(RKN);
        LoadFlowResult resultRKN = loadFlowRunner.run(rknNetwork, parameters);
        boolean isConvergedRKN = resultRKN.isFullyConverged();
        assertTrue(isConvergedRKN, baseFilename + ": Knitro should converge");

        ReacLimitsTestsUtils.checkSwitches(rknNetwork, listMinQ, listMaxQ);

        // Ecriture des dernières datas
        long end = System.nanoTime();
        knitroWritter.write("Durée du test : " + (end - start) * 1e-9 + " secondes", true);
        knitroWritter.write("Nombre d'itérations : " + resultRKN.getComponentResults().get(0).getIterationCount(), true);
        knitroWritter.write("Status à l'arrivée : " + resultRKN.getComponentResults().get(0).getStatus().name(), true);

        for (Bus bus : rknNetwork.getBusView().getBuses()) {
            if (bus.getGenerators().iterator().hasNext()) {
                var gen = bus.getGenerators().iterator().next();
                if (gen != null) {
                    double t = gen.getTargetV();
                    double v = bus.getV();
                    double min = gen.getReactiveLimits().getMinQ(gen.getTargetP());
                    double max = gen.getReactiveLimits().getMaxQ(gen.getTargetP());
                    double q = - gen.getTerminal().getQ();

                    if (Math.abs(q - max) <= 1e-4 && v - t > 1e-3 || Math.abs(q - min) <= 1e-4 && 1e-3 < t - v) {
                        System.out.println("Anomalous");
                        System.out.println("Bus " + bus.getId());
                        System.out.println("Nom V = " + bus.getVoltageLevel().getNominalV());
                        System.out.println("target = " + t);
                        System.out.println("v = " + v);
                        System.out.println("min = " + min);
                        System.out.println("max = " + max);
                        System.out.println("q = " + q);
                        break;
                    }
                }
            }
        }
    }
}
 
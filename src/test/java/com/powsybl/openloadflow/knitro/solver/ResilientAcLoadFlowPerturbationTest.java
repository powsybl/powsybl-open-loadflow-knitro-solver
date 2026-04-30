package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.commons.report.ReportNode;
import com.powsybl.iidm.network.Bus;
import com.powsybl.iidm.network.Line;
import com.powsybl.iidm.network.Network;
import com.powsybl.iidm.network.Terminal;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.SparseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.dc.DcLoadFlowEngine;
import com.powsybl.openloadflow.dc.DcLoadFlowParameters;
import com.powsybl.openloadflow.dc.DcLoadFlowResult;
import com.powsybl.openloadflow.dc.equations.DcApproximationType;
import com.powsybl.openloadflow.graph.NaiveGraphConnectivityFactory;
import com.powsybl.openloadflow.network.*;
import com.powsybl.openloadflow.network.impl.LfNetworkLoaderImpl;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeFalse;
import com.powsybl.openloadflow.knitro.solver.NetworkProviders.NetworkPair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */

public class ResilientAcLoadFlowPerturbationTest {
    private static final Logger LOGGER = LoggerFactory.getLogger(ResilientAcLoadFlowPerturbationTest.class);
    private static final String RKN = "KNITRO";
    private static final String NR = "NEWTON_RAPHSON";
    private static final String VOLTAGE_PERTURBATION = "voltage-perturbation";
    private static final String ACTIVE_POWER_PERTURBATION = "active-perturbation";
    private static final String REACTIVE_POWER_PERTURBATION = "reactive-perturbation";
    private static final boolean EXPORT = false;
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;
    private double losses;

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new SparseMatrixFactory()));
        parameters = new LoadFlowParameters()
                .setUseReactiveLimits(false)
                .setDistributedSlack(false)
                .setVoltageInitMode(LoadFlowParameters.VoltageInitMode.DC_VALUES);

    }

    @AfterEach
    void tearDown() {
        loadFlowRunner = null;
        parameters = null;
    }

    private void configureSolver(String solver) {

        OpenLoadFlowParameters.create(parameters)
                    .setSlackBusSelectionMode(SlackBusSelectionMode.MOST_MESHED)
                    .setAcSolverType(solver);

        if (RKN.equals(solver)) {
            KnitroLoadFlowParameters knitroParams = new KnitroLoadFlowParameters();
            // Set the Knitro solver type to RELAXED
            knitroParams.setLosses(losses);
            knitroParams.setKnitroSolverType(KnitroSolverParameters.SolverType.RELAXED);
            parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
        }
    }

    private void configureDcSolver() {
        LoadFlowParameters dcParameters = new LoadFlowParameters()
                .setUseReactiveLimits(false)
                .setDistributedSlack(false)
                .setVoltageInitMode(LoadFlowParameters.VoltageInitMode.DC_VALUES);
        OpenLoadFlowParameters.create(dcParameters)
                .setSlackBusSelectionMode(SlackBusSelectionMode.MOST_MESHED)
                .setAcSolverType(NR)
                .setDcApproximationType(DcApproximationType.IGNORE_R);
    }

    private void compareResilience(Network rknNetwork, Network nrNetwork, Network dcNetwork, String baseFilename, String perturbationType) {
        // Newton-Raphson
        configureSolver(NR);
        LoadFlowResult resultNR = loadFlowRunner.run(nrNetwork, parameters);
        boolean isConvergedNR = resultNR.isFullyConverged();
        boolean isFailedNR = resultNR.isFailed();
        LOGGER.info("==== Test Information ====");
        LOGGER.info("Algorithm : NR");
        LOGGER.info("Type : {}", perturbationType);
        LOGGER.info("Network name : {}", baseFilename);
        assumeFalse(isConvergedNR && !isFailedNR, baseFilename + ": NR should not converge");

        // DC Load Flow
        //
        //configureDcSolver();

        LoadFlowParameters lf = new LoadFlowParameters()
                .setDistributedSlack(false)
                .setUseReactiveLimits(false)
                .setVoltageInitMode(LoadFlowParameters.VoltageInitMode.DC_VALUES);

        OpenLoadFlowParameters olf = OpenLoadFlowParameters.create(lf)
                .setDcApproximationType(DcApproximationType.IGNORE_G)
                .setSlackBusSelectionMode(SlackBusSelectionMode.MOST_MESHED);

        DcLoadFlowParameters dcParameters =
                OpenLoadFlowParameters.createDcParameters(
                        dcNetwork,
                        lf,
                        olf,
                        new SparseMatrixFactory(),
                        new NaiveGraphConnectivityFactory<>(LfElement::getNum),
                        false
                );
        List<DcLoadFlowResult> resultsDC =
                DcLoadFlowEngine.run(dcNetwork, new LfNetworkLoaderImpl(), dcParameters, ReportNode.NO_OP);

        boolean dcOk = resultsDC.stream().anyMatch(DcLoadFlowResult::isSuccess);

       // LoadFlowResult resultDC = loadFlowRunner.run(dcNetwork, dcParameters);
       // boolean isConvergedDC = resultDC.isFullyConverged();
        LOGGER.info("==== Test Information ====");
        LOGGER.info("Algorithm : NR with DC approximation");
        LOGGER.info("Type : {}", perturbationType);
        LOGGER.info("Network name : {}", baseFilename);
        assertTrue(dcOk, baseFilename + ": DC load flow should converge");


        this.losses = calculateDcLosses(dcNetwork); // Set the parameters losses so that the RKN RELAXED solver can acces it
        LOGGER.info("Calculated DC losses: {} MW", this.losses);

        // Knitro Resilient
        configureSolver(RKN);
        LoadFlowResult resultRKN = loadFlowRunner.run(rknNetwork, parameters);
        boolean isConvergedRKN = resultRKN.isFullyConverged();
        LOGGER.info("==== Test Information ====");
        LOGGER.info("Algorithm : RKN");
        LOGGER.info("Type : {}", perturbationType);
        LOGGER.info("Network name : {}", baseFilename);
        assertTrue(isConvergedRKN, baseFilename + ": Knitro should converge");

        if (EXPORT) {
            NetworkProviders.writeXML(rknNetwork, baseFilename + "-" + perturbationType + ".xml");
        }
    }

    private void voltagePerturbationTest(Network rknNetwork, Network nrNetwork, Network dcNetwork, String baseFilename, double rPU, double xPU, double alpha) {
        PerturbationFactory.VoltagePerturbation perturbation = PerturbationFactory.getVoltagePerturbation(nrNetwork);
        PerturbationFactory.applyVoltagePerturbation(rknNetwork, perturbation, rPU, xPU, alpha);
        PerturbationFactory.applyVoltagePerturbation(nrNetwork, perturbation, rPU, xPU, alpha);
        PerturbationFactory.applyVoltagePerturbation(dcNetwork, perturbation, rPU, xPU, alpha);
        System.out.println("PERTURBATION" + perturbation);
        compareResilience(rknNetwork, nrNetwork, dcNetwork, baseFilename, VOLTAGE_PERTURBATION);
    }

    private void activePowerPerturbationTest(Network rknNetwork, Network nrNetwork, Network dcNetwork, String baseFilename, double alpha) {
        String targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork);
        PerturbationFactory.applyActivePowerPerturbation(rknNetwork, targetLoadID, alpha);
        PerturbationFactory.applyActivePowerPerturbation(nrNetwork, targetLoadID, alpha);
        compareResilience(rknNetwork, nrNetwork, dcNetwork, baseFilename, ACTIVE_POWER_PERTURBATION);
    }

    private void reactivePowerPerturbationTest(Network rknNetwork, Network nrNetwork, Network dcNetwork, String baseFilename, double targetQ) {
        PerturbationFactory.ReactivePowerPerturbation perturbation = PerturbationFactory.getReactivePowerPerturbation(nrNetwork);
        PerturbationFactory.applyReactivePowerPerturbation(rknNetwork, perturbation, targetQ);
        PerturbationFactory.applyReactivePowerPerturbation(nrNetwork, perturbation, targetQ);
        compareResilience(rknNetwork, nrNetwork, dcNetwork, baseFilename, REACTIVE_POWER_PERTURBATION);
    }

    private double calculateDcLosses(Network dcNetwork) {
// older version to keep in case
//        double totalLosses = 0.0;
//        for (Line line : dcNetwork.getLines()) {
//            Terminal terminal1 = line.getTerminal1();
//            double p1 = terminal1.getP(); // MW injected at terminal
//            double r = line.getR(); // Ohms
//            if (r == 0) {
//                continue;
//            } else if (Double.isNaN(p1)) {
//                LOGGER.warn("Line {}: P1 is NaN, skipping loss calculation for this line", line.getId());
//                continue;
//            }
//            double vnom1 = terminal1.getVoltageLevel().getNominalV(); // kV
//            double loss = r * (Math.abs(p1) * Math.abs(p1)) / (vnom1 * vnom1);
//            totalLosses += loss;
//        }
//
//        LOGGER.info("Total losses: " + totalLosses);
//        return totalLosses;
        double total = 0.0;

        for (Line line : dcNetwork.getLines()) {
            Terminal b1 = line.getTerminal1(); //.getBusBreakerView().getBus();
            Bus b2 = line.getTerminal2().getBusBreakerView().getBus();
            LOGGER.debug("bus 1 "+b1);
            if (b1 == null || b2 == null) {
                LOGGER.warn("no busses ");
                continue;
            }
            Terminal t1 = line.getTerminal1();
            Bus b3 = t1.getBusBreakerView().getBus();
            System.out.println("bus=" + (b3 == null ? "null" : b3.getId())
                    + " angle=" + (b3 == null ? "null" : b3.getAngle()));
            double a1 = b1.getBusBreakerView().getBus().getAngle();
            double a2 = b2.getAngle();
            if (Double.isNaN(a1) || Double.isNaN(a2)) {
                LOGGER.warn("no angles"+a1 + a2);
                continue;
            }
            double sBaseMva = 100.0;

            double vBaseKv = b1.getVoltageLevel().getNominalV();
            double zBaseOhm = (vBaseKv * vBaseKv) / sBaseMva;

            double xPu = line.getX() / zBaseOhm;
            double rPu = line.getR() / zBaseOhm;

            double dThetaRad = Math.toRadians(a1 - b2.getAngle());

            double pPu = dThetaRad / xPu;
            double lossPu = rPu * pPu * pPu;
            double lossMw = lossPu * sBaseMva;


            if (!Double.isFinite(rPu) || !Double.isFinite(xPu) || xPu == 0.0) {
                LOGGER.warn("xPU = 0 ");
                continue;
            }

            if (!Double.isFinite(vBaseKv) || vBaseKv == 0.0) {
                LOGGER.warn("vnom = 0");
                continue;
            }

            total += lossMw;

            }

        return total;
    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testVoltagePerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        Network dcNetwork = pair.dcNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;

        voltagePerturbationTest(rknNetwork, nrNetwork, dcNetwork, baseFilename, rPU, xPU, alpha);
    }

//    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on RTE networks: {0}")
//    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
//    void testVoltagePerturbationOnRteNetworks(NetworkPair pair) {
//        String baseFilename = pair.baseFilename();
//
//        Network rknNetwork = pair.rknNetwork();
//        Network nrNetwork = pair.nrNetwork();
//        Network dcNetwork = pair.dcNetwork();
//
//        // Line Characteristics in per-unit
//        double rPU = 0.0;
//        double xPU = 1e-5;
//        // Voltage Mismatch
//        double alpha = 0.95;
//
//        voltagePerturbationTest(rknNetwork, nrNetwork, dcNetwork, baseFilename, rPU, xPU, alpha);
//    }

    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on various IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testActivePowerPerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        Network dcNetwork = pair.dcNetwork();

        // Final perturbed load's percentage
        double alpha = 0.10;

        activePowerPerturbationTest(rknNetwork, nrNetwork, dcNetwork, baseFilename, alpha);
    }

//    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on RTE networks: {0}")
//    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
//    void testActivePowerPerturbationOnRteNetworks(NetworkPair pair) {
//        String baseFilename = pair.baseFilename();
//
//        Network rknNetwork = pair.rknNetwork();
//        Network nrNetwork = pair.nrNetwork();
//        Network dcNetwork = pair.dcNetwork();
//
//        // Final perturbed load's percentage
//        double alpha = 0.10;
//
//        activePowerPerturbationTest(rknNetwork, nrNetwork, dcNetwork, baseFilename, alpha);
//    }

//    @ParameterizedTest(name = "Test resilience of RKN to reactive power perturbation on RTE networks: {0}")
//    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
//    void testReactivePowerPerturbationOnRteNetworks(NetworkPair pair) {
//        String baseFilename = pair.baseFilename();
//
//        Network rknNetwork = pair.rknNetwork();
//        Network nrNetwork = pair.nrNetwork();
//        Network dcNetwork = pair.dcNetwork();
//
//        // Target reactive power injection by the shunt section in VArs
//        double targetQ = 1e9;
//
//        reactivePowerPerturbationTest(rknNetwork, nrNetwork, dcNetwork, baseFilename, targetQ);
//    }
}

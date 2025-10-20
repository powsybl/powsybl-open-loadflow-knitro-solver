package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.ieeecdf.converter.IeeeCdfNetworkFactory;
import com.powsybl.iidm.network.Network;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.SparseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.network.SlackBusSelectionMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeFalse;

import com.powsybl.openloadflow.knitro.solver.NetworkProviders.NetworkPair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import static com.powsybl.openloadflow.knitro.solver.NetworkProviders.CONFIDENTIAL_DATA_DIR;
import static com.powsybl.openloadflow.knitro.solver.NetworkProviders.CONFIDENTIAL_DATA_DIR_BUS_BREAKER;
import static com.powsybl.openloadflow.knitro.solver.NetworkProviders.HU_INSTANCE;
import static com.powsybl.openloadflow.knitro.solver.NetworkProviders.ES_INSTANCE;

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

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new SparseMatrixFactory()));
        parameters = new LoadFlowParameters()
                .setUseReactiveLimits(true)
                .setDistributedSlack(false);
    }

    private void configureSolver(String solver) {
        OpenLoadFlowParameters.create(parameters)
                .setSlackBusSelectionMode(SlackBusSelectionMode.MOST_MESHED)
                .setAcSolverType(solver);
//                .setVoltageInitModeOverride(OpenLoadFlowParameters.VoltageInitModeOverride.FULL_VOLTAGE);

        if (RKN.equals(solver)) {
            KnitroLoadFlowParameters knitroParams = new KnitroLoadFlowParameters();
            // Set the Knitro solver type to RESILIENT
            knitroParams.setKnitroSolverType(KnitroSolverParameters.KnitroSolverType.RESILIENT);
            knitroParams.setWithPenalV(true);
            parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
        }
    }

    private void compareResilience(Network rknNetwork, Network nrNetwork, String baseFilename, String perturbationType) {
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

    private void voltagePerturbationTest(Network rknNetwork, Network nrNetwork, String baseFilename, double rPU, double xPU, double alpha) {
        PerturbationFactory.VoltagePerturbation perturbation = PerturbationFactory.getVoltagePerturbation(nrNetwork);
        PerturbationFactory.applyVoltagePerturbation(rknNetwork, perturbation, rPU, xPU, alpha);
        PerturbationFactory.applyVoltagePerturbation(nrNetwork, perturbation, rPU, xPU, alpha);
        compareResilience(rknNetwork, nrNetwork, baseFilename, VOLTAGE_PERTURBATION);
    }

    private void activePowerPerturbationTest(Network rknNetwork, Network nrNetwork, String baseFilename, double alpha) {
        String targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork);
        PerturbationFactory.applyActivePowerPerturbation(rknNetwork, targetLoadID, alpha);
        PerturbationFactory.applyActivePowerPerturbation(nrNetwork, targetLoadID, alpha);
        compareResilience(rknNetwork, nrNetwork, baseFilename, ACTIVE_POWER_PERTURBATION);
    }

    private void reactivePowerPerturbationTest(Network rknNetwork, Network nrNetwork, String baseFilename, double targetQ) {
        PerturbationFactory.ReactivePowerPerturbation perturbation = PerturbationFactory.getReactivePowerPerturbation(nrNetwork);
        PerturbationFactory.applyReactivePowerPerturbation(rknNetwork, perturbation, targetQ);
        PerturbationFactory.applyReactivePowerPerturbation(nrNetwork, perturbation, targetQ);
        compareResilience(rknNetwork, nrNetwork, baseFilename, REACTIVE_POWER_PERTURBATION);
    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testVoltagePerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

//        parameters.getExtension(KnitroLoadFlowParameters.class).setWithPenalV(false);

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 1.075;

        voltagePerturbationTest(rknNetwork, nrNetwork, baseFilename, rPU, xPU, alpha);
    }

    @Test
    void testVoltagePerturbationOnHUInstance() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, HU_INSTANCE);
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.9;

        voltagePerturbationTest(rknNetwork, nrNetwork, "HU", rPU, xPU, alpha);
    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on HU networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideNodeBreakerHUNetworks")
    @Disabled("Temporarily disabled")
    void testVoltagePerturbationOnHUData(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;

        voltagePerturbationTest(rknNetwork, nrNetwork, baseFilename, rPU, xPU, alpha);
    }

    @Test
    void testVoltagePerturbationOnESData() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, ES_INSTANCE);
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;

        voltagePerturbationTest(rknNetwork, nrNetwork, "ES", rPU, xPU, alpha);
    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on RTE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    void testVoltagePerturbationOnRteNetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.85;

        voltagePerturbationTest(rknNetwork, nrNetwork, baseFilename, rPU, xPU, alpha);
    }

    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on various IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testActivePowerPerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Final perturbed load's percentage
        double alpha = 0.2;
        activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, alpha);
    }

    @Test
    void testActivePowerPerturbationOnHUInstance() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, HU_INSTANCE);
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();

        // Final perturbed load's percentage
        double alpha = 0.10;

        activePowerPerturbationTest(rknNetwork, nrNetwork, "HU", alpha);
    }

    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on HU networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideNodeBreakerHUNetworks")
    @Disabled("Temporarily disabled")
    void testActivePowerPerturbationOnHUData(NetworkPair pair) {
        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Final perturbed load's percentage
        double alpha = 0.10;

        activePowerPerturbationTest(rknNetwork, nrNetwork, null, alpha);
    }

    @Test
    void testActivePowerPerturbationOnESData() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, ES_INSTANCE);
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();

        // Final perturbed load's percentage
        double alpha = 0.10;

        activePowerPerturbationTest(rknNetwork, nrNetwork, "ES", alpha);
    }

    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on RTE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    void testActivePowerPerturbationOnRteNetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Final perturbed load's percentage
        double alpha = 0.10;

        activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, alpha);
    }

    @Test
    void tesReactivePowerPerturbationOnHUInstance() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR_BUS_BREAKER, HU_INSTANCE);
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();

        // Target reactive power injection by the shunt section in VArs
        double targetQ = 1e9;

        reactivePowerPerturbationTest(rknNetwork, nrNetwork, "HU", targetQ);
    }

    @ParameterizedTest(name = "Test resilience of RKN to reactive power perturbation on HU networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideBusBreakerHUNetworks")
    @Disabled("Temporarily disabled")
    void testReactivePowerPerturbationOnHUData(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Target reactive power injection by the shunt section in VArs
        double targetQ = 1e9;

        reactivePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, targetQ);
    }

    @Test
    void testReactivePowerPerturbationOnESData() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, ES_INSTANCE);
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();

        // Target reactive power injection by the shunt section in VArs
        double targetQ = 5e9;

        reactivePowerPerturbationTest(rknNetwork, nrNetwork, "ES", targetQ);
    }

//    @ParameterizedTest(name = "Test resilience of RKN to reactive power perturbation on RTE networks: {0}")
//    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    @Test
    void testReactivePowerPerturbationOnRteNetworks() {
//        String baseFilename = pair.baseFilename();

//        Network rknNetwork = pair.rknNetwork();
//        Network nrNetwork = pair.nrNetwork();
        Network network = IeeeCdfNetworkFactory.create300();
        LOGGER.info(String.valueOf(network.getShuntCompensators().iterator().next()));

        // Target reactive power injection by the shunt section in VArs
        double targetQ = 1e9;

        reactivePowerPerturbationTest(network, network, "baseFilename", targetQ);
    }
}

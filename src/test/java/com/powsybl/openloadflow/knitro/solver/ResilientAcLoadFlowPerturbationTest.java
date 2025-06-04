package com.powsybl.openloadflow.knitro.solver;

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
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public class ResilientAcLoadFlowPerturbationTest {
    private static final String RKN = "KNITRO";
    private static final String NR = "NEWTON_RAPHSON";
    private static final String HU_INSTANCE = "HU/20220226T2330Z_1D_002/init.xiidm";
    private static final String ES_INSTANCE = "20250830T1330Z_1D_ES_006.xiidm";
    private static final String CONFIDENTIAL_DATA_DIR = "../../data_confidential/";
    private static final String CONFIDENTIAL_DATA_DIR_BUS_BREAKER = "../../data_confidential_bus_breaker/";
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;

    static Stream<ResilientAcLoadFlowUnitTest.NetworkPair> provideI3ENetworks() {
        return ResilientAcLoadFlowUnitTest.provideI3ENetworks();
    }

    static Stream<ResilientAcLoadFlowUnitTest.NetworkPair> provideNodeBreakerHUNetworks() {
        return ResilientAcLoadFlowUnitTest.provideNodeBreakerHUNetworks();
    }

    static Stream<ResilientAcLoadFlowUnitTest.NetworkPair> provideBusBreakerHUNetworks() {
        return ResilientAcLoadFlowUnitTest.provideBusBreakerHUNetworks();
    }

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new SparseMatrixFactory()));
        parameters = new LoadFlowParameters()
                .setUseReactiveLimits(false)
                .setDistributedSlack(false);
    }

    private void configureSolver(String solver) {
        OpenLoadFlowParameters.create(parameters)
                .setSlackBusSelectionMode(SlackBusSelectionMode.MOST_MESHED)
                .setAcSolverType(solver);

        if (RKN.equals(solver)) {
            KnitroLoadFlowParameters knitroParams = new KnitroLoadFlowParameters();
            // Set the Knitro solver type to RESILIENT
            knitroParams.setKnitroSolverType(KnitroSolverParameters.KnitroSolverType.RESILIENT);
            parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
        }
    }

    private void compareResilience(Network rknNetwork, Network nrNetwork, String baseFilename, String perturbationType) {
        // Newton-Raphson
        configureSolver(NR);
        LoadFlowResult resultNR = loadFlowRunner.run(nrNetwork, parameters);
        boolean isConvergedNR = resultNR.isFullyConverged();
        boolean isFailedNR = resultNR.isFailed();
        assertFalse(isConvergedNR && !isFailedNR, baseFilename + ": NR should not converge");

        // Knitro Resilient
        configureSolver(RKN);
        LoadFlowResult resultRKN = loadFlowRunner.run(rknNetwork, parameters);
        boolean isConvergedRKN = resultRKN.isFullyConverged();
        assertTrue(isConvergedRKN, baseFilename + ": Knitro should converge");

        if (perturbationType != null) {
            ResilientAcLoadFlowUnitTest.writeXML(rknNetwork, baseFilename + "-" + perturbationType + ".xml");
        }
    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on IEEE networks: {0}")
    @MethodSource("provideI3ENetworks")
    void testVoltagePerturbationOnVariousI3ENetworks(ResilientAcLoadFlowUnitTest.NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;

        PerturbationFactory.VoltagePerturbation perturbation = PerturbationFactory.getVoltagePerturbation(nrNetwork);
        PerturbationFactory.applyVoltagePerturbation(rknNetwork, perturbation, rPU, xPU, alpha);
        PerturbationFactory.applyVoltagePerturbation(nrNetwork, perturbation, rPU, xPU, alpha);

        compareResilience(rknNetwork, nrNetwork, baseFilename, null);
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
        double alpha = 0.95;

        PerturbationFactory.VoltagePerturbation perturbation = PerturbationFactory.getVoltagePerturbation(nrNetwork);
        PerturbationFactory.applyVoltagePerturbation(rknNetwork, perturbation, rPU, xPU, alpha);
        PerturbationFactory.applyVoltagePerturbation(nrNetwork, perturbation, rPU, xPU, alpha);

        compareResilience(rknNetwork, nrNetwork, "HU", "voltagePerturbation");
    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on HU networks: {0}")
    @MethodSource("provideNodeBreakerHUNetworks")
    @Disabled("Temporarily disabled")
    void testVoltagePerturbationOnHUData(ResilientAcLoadFlowUnitTest.NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;

        PerturbationFactory.VoltagePerturbation perturbation = PerturbationFactory.getVoltagePerturbation(nrNetwork);
        PerturbationFactory.applyVoltagePerturbation(rknNetwork, perturbation, rPU, xPU, alpha);
        PerturbationFactory.applyVoltagePerturbation(nrNetwork, perturbation, rPU, xPU, alpha);

        compareResilience(rknNetwork, nrNetwork, baseFilename, null);
    }

    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on various IEEE networks: {0}")
    @MethodSource("provideI3ENetworks")
    void testActivePowerPerturbationOnVariousI3ENetworks(ResilientAcLoadFlowUnitTest.NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Final perturbed load's percentage
        double alpha = 0.30;

        String targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork);

        PerturbationFactory.applyActivePowerPerturbation(rknNetwork, targetLoadID, alpha);
        PerturbationFactory.applyActivePowerPerturbation(nrNetwork, targetLoadID, alpha);

        compareResilience(rknNetwork, nrNetwork, baseFilename, null);
    }

    @Test
    void testActivePowerPerturbationOnHUInstance() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, HU_INSTANCE);
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();

        // Final perturbed load's percentage
        double alpha = 0.30;

        String targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork);

        PerturbationFactory.applyActivePowerPerturbation(rknNetwork, targetLoadID, alpha);
        PerturbationFactory.applyActivePowerPerturbation(nrNetwork, targetLoadID, alpha);

        compareResilience(rknNetwork, nrNetwork, "HU", null);
    }

    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on HU networks: {0}")
    @MethodSource("provideNodeBreakerHUNetworks")
    @Disabled("Temporarily disabled")
    void testActivePowerPerturbationOnHUData(ResilientAcLoadFlowUnitTest.NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Final perturbed load's percentage
        double alpha = 0.30;

        String targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork);

        PerturbationFactory.applyActivePowerPerturbation(rknNetwork, targetLoadID, alpha);
        PerturbationFactory.applyActivePowerPerturbation(nrNetwork, targetLoadID, alpha);

        compareResilience(rknNetwork, nrNetwork, baseFilename, null);
    }

    @Test
    void testActivePowerPerturbationOnESData() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, ES_INSTANCE);
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();

        // Final perturbed load's percentage
        double alpha = 0.30;

        String targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork);

        PerturbationFactory.applyActivePowerPerturbation(rknNetwork, targetLoadID, alpha);
        PerturbationFactory.applyActivePowerPerturbation(nrNetwork, targetLoadID, alpha);

        compareResilience(rknNetwork, nrNetwork, "ES", null);
    }

    @Test
    void tesReactivePowerPerturbationOnHUInstance() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR_BUS_BREAKER, HU_INSTANCE);
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();

        // Target reactive power injection by the shunt section in VArs
        double targetQ = 1e9;

        PerturbationFactory.ReactivePowerPerturbation perturbation = PerturbationFactory.getReactivePowerPerturbation(nrNetwork);

        PerturbationFactory.applyReactivePowerPerturbation(rknNetwork, perturbation, targetQ);
        PerturbationFactory.applyReactivePowerPerturbation(nrNetwork, perturbation, targetQ);

        compareResilience(rknNetwork, nrNetwork, "HU", null);
    }

    @ParameterizedTest(name = "Test resilience of RKN to reactive power perturbation on HU networks: {0}")
    @MethodSource("provideBusBreakerHUNetworks")
    @Disabled("Temporarily disabled")
    void testReactivePowerPerturbationOnHUData(ResilientAcLoadFlowUnitTest.NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Target reactive power injection by the shunt section in VArs
        double targetQ = 1e9;

        PerturbationFactory.ReactivePowerPerturbation perturbation = PerturbationFactory.getReactivePowerPerturbation(nrNetwork);

        PerturbationFactory.applyReactivePowerPerturbation(rknNetwork, perturbation, targetQ);
        PerturbationFactory.applyReactivePowerPerturbation(nrNetwork, perturbation, targetQ);

        compareResilience(rknNetwork, nrNetwork, baseFilename, null);
    }

    @Test
    void testReactivePowerPerturbationOnESData() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, ES_INSTANCE);
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();

        // Target reactive power injection by the shunt section in VArs
        double targetQ = 1e10;

        PerturbationFactory.ReactivePowerPerturbation perturbation = PerturbationFactory.getReactivePowerPerturbation(nrNetwork);

        PerturbationFactory.applyReactivePowerPerturbation(rknNetwork, perturbation, targetQ);
        PerturbationFactory.applyReactivePowerPerturbation(nrNetwork, perturbation, targetQ);

        compareResilience(rknNetwork, nrNetwork, "ES", null);
    }
}

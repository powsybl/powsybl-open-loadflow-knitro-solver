package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.iidm.network.*;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.SparseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.network.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;


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
    private static final String EXPORT_CSV = "Slack_info/";
    private static final boolean EXPORT = true;
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new SparseMatrixFactory()));
        parameters = new LoadFlowParameters()
                .setUseReactiveLimits(false)
                .setDistributedSlack(false);
    }

    private void configureSolver(String solver, Optional<String> filepath) {
        OpenLoadFlowParameters.create(parameters)
                .setSlackBusSelectionMode(SlackBusSelectionMode.MOST_MESHED)
                .setAcSolverType(solver);

        if (RKN.equals(solver)) {
            KnitroLoadFlowParameters knitroParams = new KnitroLoadFlowParameters();
            // Set the Knitro solver type to RESILIENT
            knitroParams.setKnitroSolverType(KnitroSolverParameters.SolverType.RELAXED);
            parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
            KnitroSolverParameters.DEFAULT_EXPORT_SOLUTION = filepath;
        }
    }

    private void compareResilience(Network rknNetwork, Network nrNetwork, String baseFilename, String perturbationType, String test) {
        // Path to export Slack info CSV
        Path path = Path.of(test, baseFilename);
        Path filePath = path.resolve(baseFilename + "_" + perturbationType);
        try {
            Files.createDirectories(filePath.getParent());
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        Optional<String> filepath = Optional.of(filePath.toString());

        // Newton-Raphson
        configureSolver(NR, filepath);
        LoadFlowResult resultNR = loadFlowRunner.run(nrNetwork, parameters);
        boolean isConvergedNR = resultNR.isFullyConverged();
        boolean isFailedNR = resultNR.isFailed();
        LOGGER.info("==== Test Information ====");
        LOGGER.info("Algorithm : NR");
        LOGGER.info("Type : {}", perturbationType);
        LOGGER.info("Network name : {}", baseFilename);
        assumeFalse(isConvergedNR && !isFailedNR, baseFilename + ": NR should not converge");

        // Knitro Resilient
        configureSolver(RKN, filepath);
        LoadFlowResult resultRKN = loadFlowRunner.run(rknNetwork, parameters);
        boolean isConvergedRKN = resultRKN.isFullyConverged();
        LOGGER.info("==== Test Information ====");
        LOGGER.info("Algorithm : RKN");
        LOGGER.info("Type : {}", perturbationType);
        LOGGER.info("Network name : {}", baseFilename);
        LOGGER.info("CSV name : {}", filepath.orElse("<empty>"));
        assertTrue(isConvergedRKN, baseFilename + ": Knitro should converge");

        if (EXPORT) {
            NetworkProviders.writeXML(rknNetwork, baseFilename + "-" + perturbationType + ".xml");
        }

    }

    private void voltagePerturbationTest(Network rknNetwork, Network nrNetwork, String baseFilename, double rPU, double xPU, double alpha, String test) {
        PerturbationFactory.VoltagePerturbation perturbation = PerturbationFactory.getVoltagePerturbation(nrNetwork);
        PerturbationFactory.applyVoltagePerturbation(rknNetwork, perturbation, rPU, xPU, alpha);
        PerturbationFactory.applyVoltagePerturbation(nrNetwork, perturbation, rPU, xPU, alpha);
        compareResilience(rknNetwork, nrNetwork, baseFilename, VOLTAGE_PERTURBATION, test);
    }

    private void activePowerPerturbationTest(Network rknNetwork, Network nrNetwork, String baseFilename, double alpha, String test) {
        String targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork);
        PerturbationFactory.applyActivePowerPerturbation(rknNetwork, targetLoadID, alpha);
        PerturbationFactory.applyActivePowerPerturbation(nrNetwork, targetLoadID, alpha);
        compareResilience(rknNetwork, nrNetwork, baseFilename, ACTIVE_POWER_PERTURBATION, test);
    }

    private void reactivePowerPerturbationTest(Network rknNetwork, Network nrNetwork, String baseFilename, double targetQ, String test) {
        PerturbationFactory.ReactivePowerPerturbation perturbation = PerturbationFactory.getReactivePowerPerturbation(nrNetwork);
        PerturbationFactory.applyReactivePowerPerturbation(rknNetwork, perturbation, targetQ);
        PerturbationFactory.applyReactivePowerPerturbation(nrNetwork, perturbation, targetQ);
        compareResilience(rknNetwork, nrNetwork, baseFilename, REACTIVE_POWER_PERTURBATION, test);
    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testVoltagePerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test = EXPORT_CSV + "test_V_IEEE";
        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;
        voltagePerturbationTest(rknNetwork, nrNetwork, baseFilename, rPU, xPU, alpha, test);
    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on RTE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    void testVoltagePerturbationOnRteNetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test = EXPORT_CSV + "test_V_RTE";
        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;
        voltagePerturbationTest(rknNetwork, nrNetwork, baseFilename, rPU, xPU, alpha, test);
    }

    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on various IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testActivePowerPerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test = EXPORT_CSV + "test_active_power_IEEE";
        // Final perturbed load's percentage
        double alpha = 0.1;
        activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, alpha, test);

    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on RTE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    void testandcorrectionVoltagePerturbationOnRteNetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test = EXPORT_CSV + "test_V_RTE";
        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;
        voltagePerturbationTest(rknNetwork, nrNetwork, baseFilename, rPU, xPU, alpha, test);
    }

    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on RTE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    void testActivePowerPerturbationOnRteNetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test = EXPORT_CSV + "test_active_power_RTE";
        // Final perturbed load's percentage
        double alpha = 0.30;

        activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, alpha, test);
    }

    @ParameterizedTest(name = "Test resilience of RKN to reactive power perturbation on RTE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    void testReactivePowerPerturbationOnRteNetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test = EXPORT_CSV + "test_reactive_power_RTE";
        LOGGER.info(String.valueOf(nrNetwork.getShuntCompensators().iterator().next()));

        // Target reactive power injection by the shunt section in VArs
        double targetQ = 1e9;

        reactivePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, targetQ, test);
    }
}

package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.iidm.network.Line;
import com.powsybl.iidm.network.Network;
import com.powsybl.iidm.network.Terminal;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.SparseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.network.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.io.TempDir;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeFalse;

import com.powsybl.openloadflow.knitro.solver.NetworkProviders.NetworkPair;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */

class ResilientAcLoadFlowPerturbationTest {
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
    private String exportSolution;

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new SparseMatrixFactory()));
        // With these parameters no outer loop are activated
        parameters = new LoadFlowParameters()
                .setUseReactiveLimits(false)
                .setDistributedSlack(false)
                .setVoltageInitMode(LoadFlowParameters.VoltageInitMode.UNIFORM_VALUES);

    }

    private void configureSolver(String solver) {
        OpenLoadFlowParameters.create(parameters)
                .setSlackBusSelectionMode(SlackBusSelectionMode.MOST_MESHED)
                .setAcSolverType(solver);

        if (RKN.equals(solver)) {
            KnitroLoadFlowParameters knitroParams = new KnitroLoadFlowParameters();
            // Set the Knitro solver type to RELAXED
            knitroParams.setKnitroSolverType(KnitroSolverParameters.SolverType.RELAXED);
            knitroParams.setLosses(losses);
            knitroParams.setExportSolution(exportSolution);
            parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
        }
    }

    private void compareResilience(Network rknNetwork, Network nrNetwork, Network dcNetwork, String baseFilename, String perturbationType, String test) {
        // Path to export Slack info CSV
        Path path = Path.of(test, baseFilename);
        Path filePath = path.resolve(baseFilename + "_" + perturbationType);
        try {
            Files.createDirectories(filePath.getParent());
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        this.exportSolution = filePath.toString();

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
        LoadFlowParameters dcParameters = new LoadFlowParameters()
                .setDc(true);
        LoadFlowResult resultsDC = LoadFlow.run(dcNetwork, dcParameters);
        boolean isConvergedDC = resultsDC.isFullyConverged();
        LOGGER.info("==== Test Information ====");
        LOGGER.info("Algorithm : DC Load Flow");
        LOGGER.info("Type : {}", perturbationType);
        LOGGER.info("Network name : {}", baseFilename);
        assertTrue(isConvergedDC, baseFilename + ": DC load flow should converge");

        this.losses = calculateDcLosses(dcNetwork);

        // Knitro Resilient
        configureSolver(RKN);
        LoadFlowResult resultRKN = loadFlowRunner.run(rknNetwork, parameters);
        boolean isConvergedRKN = resultRKN.isFullyConverged();
        LOGGER.info("==== Test Information ====");
        LOGGER.info("Algorithm : RKN");
        LOGGER.info("Type : {}", perturbationType);
        LOGGER.info("Network name : {}", baseFilename);
        LOGGER.info("CSV name : {}", exportSolution);
        assertTrue(isConvergedRKN, baseFilename + ": Knitro should converge");

        if (EXPORT) {
            NetworkProviders.writeXML(rknNetwork, baseFilename + "-" + perturbationType + ".xml");
        }
    }

    private void voltagePerturbationTest(Network rknNetwork, Network nrNetwork, Network dcNetwork, String baseFilename, double rPU, double xPU, double alpha, String test) {
        PerturbationFactory.VoltagePerturbation perturbation = PerturbationFactory.getVoltagePerturbation(nrNetwork);
        PerturbationFactory.applyVoltagePerturbation(rknNetwork, perturbation, rPU, xPU, alpha);
        PerturbationFactory.applyVoltagePerturbation(nrNetwork, perturbation, rPU, xPU, alpha);
        PerturbationFactory.applyVoltagePerturbation(dcNetwork, perturbation, rPU, xPU, alpha);
        compareResilience(rknNetwork, nrNetwork, dcNetwork, baseFilename, VOLTAGE_PERTURBATION, test);
    }

    private void activePowerPerturbationTest(Network rknNetwork, Network nrNetwork, Network dcNetwork, String baseFilename, double alpha, String test) {
        String targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork);
        PerturbationFactory.applyActivePowerPerturbation(rknNetwork, targetLoadID, alpha);
        PerturbationFactory.applyActivePowerPerturbation(nrNetwork, targetLoadID, alpha);
        PerturbationFactory.applyActivePowerPerturbation(dcNetwork, targetLoadID, alpha);
        compareResilience(rknNetwork, nrNetwork, dcNetwork, baseFilename, ACTIVE_POWER_PERTURBATION, test);
    }

    private void reactivePowerPerturbationTest(Network rknNetwork, Network nrNetwork, Network dcNetwork, String baseFilename, double targetQ, String test) {
        PerturbationFactory.ReactivePowerPerturbation perturbation = PerturbationFactory.getReactivePowerPerturbation(nrNetwork);
        PerturbationFactory.applyReactivePowerPerturbation(rknNetwork, perturbation, targetQ);
        PerturbationFactory.applyReactivePowerPerturbation(nrNetwork, perturbation, targetQ);
        PerturbationFactory.applyReactivePowerPerturbation(dcNetwork, perturbation, targetQ);
        compareResilience(rknNetwork, nrNetwork, dcNetwork, baseFilename, REACTIVE_POWER_PERTURBATION, test);
    }

    private double calculateDcLosses(Network dcNetwork) {
        double totalLosses = 0.0;

        for (Line line : dcNetwork.getLines()) {
            Terminal terminal1 = line.getTerminal1();
            double p1 = terminal1.getP(); // MW
            double r = line.getR(); // Ohms
            if (r == 0) {
                continue;
            } else if (Double.isNaN(p1)) {
                LOGGER.warn("Line {}: P1 is NaN, skipping loss calculation for this line", line.getId());
                continue;
            }
            double vnom1 = terminal1.getVoltageLevel().getNominalV(); // kV
            double loss = r * (Math.abs(p1) * Math.abs(p1)) / (vnom1 * vnom1);
            totalLosses += loss;
        }
        LOGGER.info("Total DC losses: {}", totalLosses);
        return totalLosses;
    }

    @TempDir
    Path tmp;

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testVoltagePerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();
        String exportPath = tmp.resolve("Slack_info").toString();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        Network dcNetwork = pair.dcNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;
        voltagePerturbationTest(rknNetwork, nrNetwork, dcNetwork, baseFilename, rPU, xPU, alpha, exportPath);
    }

    // Same test but with the activation of outerloop calculation
    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation with active outerloop on IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testVoltagePerturbationOnVariousI3ENetworksOuterLoopActivated(NetworkPair pair) {
        parameters = new LoadFlowParameters()
                .setUseReactiveLimits(true)
                .setDistributedSlack(true);

        String baseFilename = pair.baseFilename();
        String exportPath = tmp.resolve("Slack_info").toString();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        Network dcNetwork = pair.dcNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;
        voltagePerturbationTest(rknNetwork, nrNetwork, dcNetwork, baseFilename, rPU, xPU, alpha, exportPath);
    }

    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation (extreme perturbation) on IEEE30 network: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3E30Networks")
    void testActivePowerPerturbationOnI3E30Network(NetworkProviders.NetworkPair pair) {
        String baseFilename = pair.baseFilename();
        String exportPath = tmp.resolve("Slack_info").toString();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        Network dcNetwork = pair.dcNetwork();

        // Final percentage applied to perturb the load
        // The perturbation is intentionally extreme to trigger the logging message for P
        double alpha = 5;

        activePowerPerturbationTest(rknNetwork, nrNetwork, dcNetwork, baseFilename, alpha, exportPath);
    }

    @ParameterizedTest(name = "Test resilience of RKN to a active power perturbation on IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testActivePowerPerturbationOnI3ENetwork(NetworkProviders.NetworkPair pair) {
        String baseFilename = pair.baseFilename();
        String exportPath = tmp.resolve("Slack_info").toString();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        Network dcNetwork = pair.dcNetwork();

        // Final perturbed load's percentage
        double alpha = 0.10;

        activePowerPerturbationTest(rknNetwork, nrNetwork, dcNetwork, baseFilename, alpha, exportPath);
    }
}



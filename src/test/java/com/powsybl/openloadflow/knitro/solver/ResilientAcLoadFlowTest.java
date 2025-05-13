package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.ieeecdf.converter.IeeeCdfNetworkFactory;
import com.powsybl.iidm.modification.SetGeneratorToLocalRegulation;
import com.powsybl.iidm.modification.topology.RemoveFeederBay;
import com.powsybl.iidm.network.*;
import com.powsybl.iidm.serde.XMLExporter;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.DenseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.network.SlackBusSelectionMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public class ResilientAcLoadFlowTest {
    private static final String DEFAULT_OUTPUT_DIR = "./outputs/";
    private static final double DEFAULT_TOLERANCE = 1e-3;
    private static final double BASE_100MVA = 100.0;
    private static final String RKN = "KNITRO";
    private static final String NR = "NEWTON_RAPHSON";
    private static final String CONFIDENTIAL_DATA_DIR = "../../data_confidential/";
    private static final String CONFIDENTIAL_DATA_DIR_BUS_BREAKER = "../../data_confidential_bus_breaker/";
    private static final String HU_FILENAME_TEST_INSTANCE = "HU/20220226T2330Z_1D_002/init.xiidm";
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;

    static Stream<NetworkPair> provideNetworks() {
        return Stream.of(
                new NetworkPair(IeeeCdfNetworkFactory.create14(), IeeeCdfNetworkFactory.create14(), "ieee14"),
                new NetworkPair(IeeeCdfNetworkFactory.create30(), IeeeCdfNetworkFactory.create30(), "ieee30"),
                new NetworkPair(IeeeCdfNetworkFactory.create118(), IeeeCdfNetworkFactory.create118(), "ieee118"),
                new NetworkPair(IeeeCdfNetworkFactory.create300(), IeeeCdfNetworkFactory.create300(), "ieee300")
        );
    }

    static Stream<NetworkPair> provideHUNetworks(String dir) {
        Path baseDir = Path.of(dir, "HU");
        String initFileName = "init.xiidm";

        try (Stream<Path> cases = Files.list(baseDir)) {
            List<NetworkPair> networkPairs = cases.filter(Files::isDirectory)
                    .map(subDir -> subDir.resolve(initFileName))
                    .filter(Files::exists)
                    .map(initPath -> {
                        Network nrNetwork = Network.read(initPath).getNetwork();
                        Network rknNetwork = Network.read(initPath).getNetwork();
                        String name = initPath.getParent().getFileName().toString();
                        return new NetworkPair(rknNetwork, nrNetwork, name);
                    })
                    .toList();
            return networkPairs.stream();

        } catch (Exception e) {
            throw new RuntimeException("Failed to load HU real network cases", e);
        }
    }

    static Stream<NetworkPair> provideNodeBreakerHUNetworks() {
        return provideHUNetworks(CONFIDENTIAL_DATA_DIR);
    }

    static Stream<NetworkPair> provideBusBreakerHUNetworks() {
        return provideHUNetworks(CONFIDENTIAL_DATA_DIR_BUS_BREAKER);
    }

    static Stream<VoltagePerturbation> getAllVoltagePerturbationsOnHUNetworks() {
        List<VoltagePerturbation> perturbations = new ArrayList<>();

        String noGeneratorBusID;
        String regulatingBusID;
        String generatorID;
        String lowImpedanceLineID;

        List<Generator> generators;
        List<Line> lowImpedanceLines;
        List<Line> normalLines;

        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, HU_FILENAME_TEST_INSTANCE);
        Network network = Network.read(fileName).getNetwork();
        generators = network.getGeneratorStream()
                .toList();

        boolean hasRegulatingGenerator;
        boolean hasGenerators;
        boolean isLowImpedanceLine;

        Terminal terminal11;
        Terminal terminal12;
        Terminal terminal21;
        Terminal terminal22;

        Bus noGeneratorBus;

        String terminal11ID;
        String terminal12ID;

        String terminal21ID;
        String terminal22ID;

        for (Generator generator : generators) {
            terminal11 = generator.getTerminal();
            terminal11ID = terminal11.getBusBreakerView()
                    .getBus()
                    .getId();

            lowImpedanceLines = terminal11.getBusBreakerView()
                    .getBus()
                    .getLineStream()
                    .toList();

            for (Line lowImpedanceLine : lowImpedanceLines) {
                // Obtain the terminal at the other end of the line
                terminal12 = lowImpedanceLine.getTerminal1();
                terminal12ID = terminal12.getBusBreakerView()
                        .getBus()
                        .getId();

                if (terminal12ID.equals(terminal11ID)) {
                    terminal12 = lowImpedanceLine.getTerminal2();
                }

                // Check if this terminal is connected to a bus with no generators
                hasGenerators = terminal12.getBusBreakerView()
                        .getBus()
                        .getGenerators()
                        .iterator()
                        .hasNext();
                if (hasGenerators) {
                    continue;
                }

                noGeneratorBus = terminal12.getBusBreakerView()
                        .getBus();
                noGeneratorBusID = noGeneratorBus.getId();

                // Get the related buses and their generators
                normalLines = noGeneratorBus.getLineStream()
                        .toList();

                for (Line normalLine : normalLines) {
                    // Get the two terminals of each line that is connected to the noGeneratorBus
                    terminal21 = normalLine.getTerminal1();
                    terminal21ID = terminal21.getBusBreakerView()
                            .getBus()
                            .getId();
                    terminal22 = normalLine.getTerminal2();
                    terminal22ID = terminal22.getBusBreakerView()
                            .getBus()
                            .getId();

                    // terminal21 is connected to the noGeneratorBus
                    if (terminal22ID.equals(noGeneratorBusID)) {
                        terminal22 = terminal21;
                        terminal22ID = terminal21ID;
                    }

                    // Check that this line is not the same as lowImpedanceLine
                    isLowImpedanceLine = generator.getTerminal()
                            .getBusBreakerView()
                            .getBus()
                            .getId()
                            .equals(terminal22ID);

                    if (isLowImpedanceLine) {
                        continue;
                    }

                    hasRegulatingGenerator = terminal22.getBusBreakerView()
                            .getBus()
                            .getGenerators()
                            .iterator()
                            .hasNext();

                    if (hasRegulatingGenerator) {
                        generatorID = generator.getId();
                        lowImpedanceLineID = lowImpedanceLine.getId();
                        regulatingBusID = terminal22.getBusBreakerView()
                                .getBus()
                                .getId();
                        perturbations.add(new VoltagePerturbation(noGeneratorBusID, regulatingBusID, generatorID, lowImpedanceLineID));
                    }
                }
            }
        }
        assertFalse(perturbations.isEmpty(), "No possible perturbations were found!");
        return perturbations.stream();
    }

    static Stream<ReactivePowerPerturbation> getAllReactivePowerPerturbationsOnHUNetworks() {
        List<ReactivePowerPerturbation> perturbations = new ArrayList<>();

        String targetBusID;
        String targetGeneratorID;
        String targetShuntID;

        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR_BUS_BREAKER, HU_FILENAME_TEST_INSTANCE);
        Network network = Network.read(fileName).getNetwork();

        List<Bus> networkBuses = network.getBusBreakerView()
                .getBusStream()
                .toList();

        boolean hasGenerators;
        boolean hasShunts;

        for (Bus bus : networkBuses) {
            hasGenerators = bus.getGenerators()
                    .iterator()
                    .hasNext();
            hasShunts = bus.getShuntCompensatorStream()
                    .iterator()
                    .hasNext();
            if (!hasGenerators || !hasShunts) {
                continue;
            }
            for (Generator generator : bus.getGenerators()) {
                for (ShuntCompensator shuntCompensator : bus.getShuntCompensators()) {
                    targetBusID = bus.getId();
                    targetGeneratorID = generator.getId();
                    targetShuntID = shuntCompensator.getId();
                    perturbations.add(new ReactivePowerPerturbation(targetBusID, targetGeneratorID, targetShuntID));
                }
            }
        }

        assertFalse(perturbations.isEmpty(), "No match was found");
        return perturbations.stream();
    }

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new DenseMatrixFactory()));
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

    private void writeXML(Network network, String name) {
        Properties properties = new Properties();
        properties.put(XMLExporter.VERSION, "1.12");
        Path path = Path.of(DEFAULT_OUTPUT_DIR, name);
        network.write("XIIDM", properties, path);
    }

    private void exportAsBusBreakerTopology(Path initPath, Path endPath) {
        Network network = Network.read(initPath).getNetwork();
        for (VoltageLevel vl : network.getVoltageLevels()) {
            vl.convertToTopology(TopologyKind.BUS_BREAKER);
        }
        Properties exportParameters = new Properties();
        exportParameters.put(XMLExporter.TOPOLOGY_LEVEL, "BUS_BREAKER");
        network.write("XIIDM", exportParameters, endPath);
    }

    private void checkElectricalQuantities(Network n1, Network n2, double tolerance) {
        // Check generators active and reactive per unit powers
        for (Generator g1 : n1.getGenerators()) {
            Generator g2 = n2.getGenerator(g1.getId());
            assertNotNull(g2, "Generator " + g1.getId() + " not found in second network");
            Terminal t1 = g1.getTerminal();
            Terminal t2 = g2.getTerminal();

            double p1 = t1.getP() / BASE_100MVA;
            double p2 = t2.getP() / BASE_100MVA;
            double q1 = t1.getQ() / BASE_100MVA;
            double q2 = t2.getQ() / BASE_100MVA;

            assertEquals(p1, p2, tolerance, "Mismatch on P for generator " + g1.getId());
            assertEquals(q1, q2, tolerance, "Mismatch on Q for generator " + g1.getId());
        }

        // Check loads active and reactive powers
        for (Load l1 : n1.getLoads()) {
            Load l2 = n2.getLoad(l1.getId());
            assertNotNull(l2, "Load " + l1.getId() + " not found in second network");
            Terminal t1 = l1.getTerminal();
            Terminal t2 = l2.getTerminal();

            double p1 = t1.getP() / BASE_100MVA;
            double p2 = t2.getP() / BASE_100MVA;
            double q1 = t1.getQ() / BASE_100MVA;
            double q2 = t2.getQ() / BASE_100MVA;

            assertEquals(p1, p2, tolerance, "Mismatch on P for load " + l1.getId());
            assertEquals(q1, q2, tolerance, "Mismatch on Q for load " + l1.getId());
        }

        // Check bus per unit voltages and angles (in degrees)
        for (Bus bus1 : n1.getBusView().getBuses()) {
            Bus bus2 = n2.getBusView().getBus(bus1.getId());
            assertNotNull(bus2, "Bus " + bus1.getId() + " not found in second network");

            double v1 = bus1.getV() / bus1.getVoltageLevel().getNominalV();
            double v2 = bus2.getV() / bus2.getVoltageLevel().getNominalV();
            double phi1 = bus1.getAngle();
            double phi2 = bus2.getAngle();

            assertEquals(v1, v2, tolerance, "Mismatch on V for bus " + bus1.getId());
            assertEquals(phi1, phi2, tolerance, "Mismatch on Phi for bus " + bus1.getId());
        }

        // Check lines current magnitudes per unit
        for (Line line1 : n1.getLines()) {
            Line line2 = n2.getLine(line1.getId());
            assertNotNull(line2, "Line " + line1.getId() + " not found in second network");

            double vNom1T1 = line1.getTerminal1().getVoltageLevel().getNominalV();
            double vNom1T2 = line1.getTerminal2().getVoltageLevel().getNominalV();
            double vNom2T1 = line2.getTerminal1().getVoltageLevel().getNominalV();
            double vNom2T2 = line2.getTerminal2().getVoltageLevel().getNominalV();

            assertEquals(vNom1T1, vNom2T1, "Mismatch on Vnom for line " + line1.getId() + " terminal 1");
            assertEquals(vNom1T2, vNom2T2, "Mismatch on Vnom for line " + line1.getId() + " terminal 2");

            double i1T1 = computePerUnitCurrent(vNom1T1, line1.getTerminal1().getI());
            double i2T1 = computePerUnitCurrent(vNom2T1, line2.getTerminal1().getI());
            double i1T2 = computePerUnitCurrent(vNom1T2, line1.getTerminal2().getI());
            double i2T2 = computePerUnitCurrent(vNom2T2, line2.getTerminal2().getI());

            assertEquals(i1T1, i2T1, tolerance, "Mismatch on I for line " + line1.getId() + " terminal 1");
            assertEquals(i1T2, i2T2, tolerance, "Mismatch on I for line " + line1.getId() + " terminal 2");
        }
    }

    double computePerUnitCurrent(double vNom, double i) {
        return ((Math.sqrt(3) * vNom) / (BASE_100MVA * 1e3)) * i;
    }

    private void compareSolvers(Network rknNetwork, Network nrNetwork, String baseFilename) {
        configureSolver(RKN);
        LoadFlowResult result1 = loadFlowRunner.run(rknNetwork, parameters);
        assertTrue(result1.isFullyConverged());

        configureSolver(NR);
        LoadFlowResult result2 = loadFlowRunner.run(nrNetwork, parameters);
        assertTrue(result2.isFullyConverged());

        checkElectricalQuantities(rknNetwork, nrNetwork, DEFAULT_TOLERANCE);

        if (baseFilename != null) {
            writeXML(nrNetwork, baseFilename + "-NR.xml");
            writeXML(rknNetwork, baseFilename + "-RKN.xml");
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
            writeXML(rknNetwork, baseFilename + "-" + perturbationType + ".xml");
        }
    }

    @ParameterizedTest
    @MethodSource("provideNetworks")
    void testLoadFlowComparisonOnVariousI3ENetworks(NetworkPair pair) {
        compareSolvers(pair.rknNetwork(), pair.nrNetwork(), null);
    }

    @ParameterizedTest(name = "Test resilience of RKN to all possible reactive power perturbations on HU networks: {0}")
    @MethodSource("getAllReactivePowerPerturbationsOnHUNetworks")
    void testAllReactivePowerPerturbationsOnHUData(ReactivePowerPerturbation perturbation) {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR_BUS_BREAKER, HU_FILENAME_TEST_INSTANCE);

        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();

        // Target reactive power injection by the shunt section in VArs
        double targetQ = 4e8;

        applyReactivePowerPerturbation(nrNetwork, perturbation, targetQ);
        applyReactivePowerPerturbation(rknNetwork, perturbation, targetQ);

        compareResilience(rknNetwork, nrNetwork, "HU", null);
    }

    @ParameterizedTest(name = "Test resilience of RKN to reactive power perturbation on HU networks: {0}")
    @MethodSource("provideBusBreakerHUNetworks")
    void testReactivePowerPerturbationOnHUData(NetworkPair pair) {
        String baseFilename = pair.baseFilename;

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Target reactive power injection by the shunt section in VArs
        double targetQ = 1e9;

        ReactivePowerPerturbation perturbation = getReactivePowerPerturbation(nrNetwork);

        applyReactivePowerPerturbation(rknNetwork, perturbation, targetQ);
        applyReactivePowerPerturbation(nrNetwork, perturbation, targetQ);

        compareResilience(rknNetwork, nrNetwork, baseFilename, null);
    }

    private ReactivePowerPerturbation getReactivePowerPerturbation(Network network) {
        String targetBusID = "";
        String targetGeneratorID = "";
        String targetShuntID = "";

        List<Bus> networkBuses = network.getBusBreakerView()
                .getBusStream()
                .toList();

        boolean hasGenerators = false;
        boolean hasShunts = false;

        for (Bus bus : networkBuses) {
            hasGenerators = bus.getGenerators()
                    .iterator()
                    .hasNext();
            hasShunts = bus.getShuntCompensatorStream()
                    .iterator()
                    .hasNext();
            if (hasGenerators && hasShunts) {
                targetBusID = bus.getId();
                targetGeneratorID = bus.getGenerators()
                        .iterator()
                        .next()
                        .getId();
                targetShuntID = bus.getShuntCompensators()
                        .iterator()
                        .next()
                        .getId();
                break;
            }
        }

        assertTrue(hasGenerators && hasShunts, "No match was found");
        return new ReactivePowerPerturbation(targetBusID, targetGeneratorID, targetShuntID);
    }

    private void applyReactivePowerPerturbation(Network network, ReactivePowerPerturbation perturbation, double targetQ) {
        String targetBusID = perturbation.targetBusID;
        String targetGeneratorID = perturbation.targetGeneratorID;
        String targetShuntID = perturbation.targetShuntID;

        Bus targetBus = network.getBusBreakerView()
                .getBus(targetBusID);
        Generator targetGenerator = network.getGenerator(targetGeneratorID);

        // Make the generator control the voltage locally
        SetGeneratorToLocalRegulation generatorModification = new SetGeneratorToLocalRegulation(targetGeneratorID);
        generatorModification.apply(network);

        // Get the susceptance value to inject targetQ of reactive power
        double generatorVoltage = targetGenerator.getTargetV();
        double targetB = targetQ / Math.pow(generatorVoltage * 1e3, 2);

        VoltageLevel targetVL = targetBus.getVoltageLevel();

        // Removing the old shunt compensator
        RemoveFeederBay removedShuntCompensator = new RemoveFeederBay(targetShuntID);
        removedShuntCompensator.apply(network);

        // Creating the new shunt with the modified parameters
        ShuntCompensatorAdder modifiedShuntAdder = targetVL.newShuntCompensator()
                .setId(targetShuntID + "-modified")
                .setName(targetShuntID + "-modified")
                .setBus(targetBusID)
                .setSectionCount(1);

        modifiedShuntAdder.newLinearModel()
                .setMaximumSectionCount(1)
                .setBPerSection(targetB)
                .add();

        modifiedShuntAdder.add();
    }

    @ParameterizedTest(name = "Test resilience of RKN to an active power perturbation on HU networks: {0}")
    @MethodSource("provideNodeBreakerHUNetworks")
    void testActivePowerPerturbationOnHUData(NetworkPair pair) {
        String baseFilename = pair.baseFilename;

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Final perturbed load's percentage
        double alpha = 0.06;

        String targetLoadID = getActivePowerPerturbation(nrNetwork);

        applyActivePowerPerturbation(rknNetwork, targetLoadID, alpha);
        applyActivePowerPerturbation(nrNetwork, targetLoadID, alpha);

        compareResilience(rknNetwork, nrNetwork, baseFilename, null);
    }

    private String getActivePowerPerturbation(Network network) {
        Optional<? extends Load> targetLoadOp = network.getLoadStream()
                .filter(load -> load.getP0() > 0.0)
                .min(Comparator.comparingDouble(Load::getP0));

        assertFalse(targetLoadOp.isEmpty(), "Network contains no loads.");

        return targetLoadOp.get()
                .getId();
    }

    private void applyActivePowerPerturbation(Network network, String targetLoadID, double alpha) {
        Load targetLoad = network.getLoad(targetLoadID);

        double totalActiveLoad = 0.0;
        double activeLoad;

        for (Load load : network.getLoads()) {
            activeLoad = load.getP0();
            if (activeLoad > 0.0) {
                totalActiveLoad += activeLoad;
            }
        }

        targetLoad.setP0(alpha * totalActiveLoad);
    }

    @ParameterizedTest(name = "Test resilience of RKN to all possible voltage perturbation on HU networks: {0}")
    @MethodSource("getAllVoltagePerturbationsOnHUNetworks")
    void testAllVoltagePerturbationsOnHUData(VoltagePerturbation perturbation) {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, "HU/20220226T2330Z_1D_002/init.xiidm");

        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 5e-8;
        // Voltage Mismatch
        double alpha = 0.95;

        applyVoltagePerturbation(nrNetwork, perturbation, rPU, xPU, alpha);
        applyVoltagePerturbation(rknNetwork, perturbation, rPU, xPU, alpha);

        compareResilience(rknNetwork, nrNetwork, "HU", null);
    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on HU networks: {0}")
    @MethodSource("provideNodeBreakerHUNetworks")
    void testVoltagePerturbationOnHUData(NetworkPair pair) {
        String baseFilename = pair.baseFilename;

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 5e-8;
        // Voltage Mismatch
        double alpha = 0.95;

        VoltagePerturbation perturbation = getVoltagePerturbation(nrNetwork);
        applyVoltagePerturbation(rknNetwork, perturbation, rPU, xPU, alpha);
        applyVoltagePerturbation(nrNetwork, perturbation, rPU, xPU, alpha);

        compareResilience(rknNetwork, nrNetwork, baseFilename, "voltagePerturbation");
    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on IEEE networks: {0}")
    @MethodSource("provideNetworks")
    void testVoltagePerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename;

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();

        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.85;

        VoltagePerturbation perturbation = getVoltagePerturbation(nrNetwork);
        applyVoltagePerturbation(rknNetwork, perturbation, rPU, xPU, alpha);
        applyVoltagePerturbation(nrNetwork, perturbation, rPU, xPU, alpha);

        compareResilience(rknNetwork, nrNetwork, baseFilename, "voltagePerturbation");
    }

    private VoltagePerturbation getVoltagePerturbation(Network network) {
        String noGeneratorBusID = "";
        String regulatingBusID = "";
        String generatorID = "";
        String lowImpedanceLineID = "";

        List<Generator> generators;
        List<Line> lowImpedanceLines;
        List<Line> normalLines;

        generators = network.getGeneratorStream()
                .toList();

        //boolean hasRegulatingGenerator = true;
        boolean hasRegulatingGenerator = false;
        boolean hasGenerators;
        boolean isLowImpedanceLine;

        Terminal terminal11;
        Terminal terminal12;
        Terminal terminal21;
        Terminal terminal22;

        Bus noGeneratorBus;

        String terminal11ID;
        String terminal12ID;

        String terminal21ID;
        String terminal22ID;

        outerLoop:
        for (Generator generator : generators) {
            terminal11 = generator.getTerminal();
            terminal11ID = terminal11.getBusBreakerView()
                    .getBus()
                    .getId();

            lowImpedanceLines = terminal11.getBusBreakerView()
                    .getBus()
                    .getLineStream()
                    .toList();

            for (Line lowImpedanceLine : lowImpedanceLines) {
                // Obtain the terminal at the other end of the line
                terminal12 = lowImpedanceLine.getTerminal1();
                terminal12ID = terminal12.getBusBreakerView()
                        .getBus()
                        .getId();

                if (terminal12ID.equals(terminal11ID)) {
                    terminal12 = lowImpedanceLine.getTerminal2();
                }

                // Check if this terminal is connected to a bus with no generators
                hasGenerators = terminal12.getBusBreakerView()
                        .getBus()
                        .getGenerators()
                        .iterator()
                        .hasNext();
                if (hasGenerators) {
                    continue;
                }

                noGeneratorBus = terminal12.getBusBreakerView()
                        .getBus();
                noGeneratorBusID = noGeneratorBus.getId();

                // Get the related buses and their generators
                normalLines = noGeneratorBus.getLineStream()
                        .toList();

                for (Line normalLine : normalLines) {
                    // Get the two terminals of each line that is connected to the noGeneratorBus
                    terminal21 = normalLine.getTerminal1();
                    terminal21ID = terminal21.getBusBreakerView()
                            .getBus()
                            .getId();
                    terminal22 = normalLine.getTerminal2();
                    terminal22ID = terminal22.getBusBreakerView()
                            .getBus()
                            .getId();

                    // terminal21 is connected to the noGeneratorBus
                    if (terminal22ID.equals(noGeneratorBusID)) {
                        terminal22 = terminal21;
                        terminal22ID = terminal21ID;
                    }

                    // Check that this line is not the same as lowImpedanceLine
                    isLowImpedanceLine = generator.getTerminal()
                            .getBusBreakerView()
                            .getBus()
                            .getId()
                            .equals(terminal22ID);

                    if (isLowImpedanceLine) {
                        continue;
                    }

                    hasRegulatingGenerator = terminal22.getBusBreakerView()
                            .getBus()
                            .getGenerators()
                            .iterator()
                            .hasNext();

                    if (hasRegulatingGenerator) {
                        generatorID = generator.getId();
                        lowImpedanceLineID = lowImpedanceLine.getId();
                        regulatingBusID = terminal22.getBusBreakerView()
                                .getBus()
                                .getId();
                        break outerLoop;
                    }
                }
            }
        }
        assertTrue(hasRegulatingGenerator, "No match was found!");
        return new VoltagePerturbation(noGeneratorBusID, regulatingBusID, generatorID, lowImpedanceLineID);
    }

    private void applyVoltagePerturbation(Network network, VoltagePerturbation perturbation, double rPU, double xPU, double alpha) {
        String noGeneratorBusID = perturbation.noGeneratorBusID();
        String regulatingBusID = perturbation.regulatingBusID();
        String generatorID = perturbation.generatorID();
        String lowImpedanceLineID = perturbation.lowImpedanceLineID();

        // Acquire components to change
        Bus noGeneratorBus = network.getBusBreakerView()
                .getBus(noGeneratorBusID);
        Bus regulatingBus = network.getBusBreakerView()
                .getBus(regulatingBusID);
        Generator generator = network.getGenerator(generatorID);
        Line lowImpedanceLine = network.getLine(lowImpedanceLineID);

        // Acquire the components voltage levels
        VoltageLevel generatorBusVL = generator.getTerminal()
                .getVoltageLevel();
        VoltageLevel regulatingBusVL = regulatingBus
                .getVoltageLevel();
        VoltageLevel noGeneratorBusVL = noGeneratorBus.getVoltageLevel();

        // Apply voltage mismatch
        double vNomGenerator = generatorBusVL.getNominalV();
        double vNomRegulator = regulatingBusVL.getNominalV();
        double vNomNoGenerator = noGeneratorBusVL.getNominalV();
        double targetV = generator.getTargetV() / vNomRegulator * vNomGenerator * alpha;

        // Get regulating terminal
        Terminal regulatingTerminal;
        Optional<? extends Terminal> regulatingTerminalOp = lowImpedanceLine.getTerminals()
                .stream()
                .filter(terminal -> terminal.getBusBreakerView()
                        .getBus()
                        .getId()
                        .equals(noGeneratorBusID))
                .findAny();

        assertFalse(regulatingTerminalOp.isEmpty(), "Regulating bus' terminal not found");
        regulatingTerminal = regulatingTerminalOp.get();

        List<Generator> regulatingGenerators = regulatingBus.getGeneratorStream()
                .toList();
        for (Generator regulatingGenerator : regulatingGenerators) {
            regulatingGenerator.setTargetV(targetV)
                    .setVoltageRegulatorOn(true)
                    .setRegulatingTerminal(regulatingTerminal);
        }
        generator.setVoltageRegulatorOn(true);

        // Changing line parameters by setting their reactance and shunt admittance to 0
        double r = rPU * vNomGenerator * vNomNoGenerator / BASE_100MVA;
        double x = xPU * vNomGenerator * vNomNoGenerator / BASE_100MVA;

        lowImpedanceLine.setR(r)
                .setX(x)
                .setG1(0.0)
                .setB1(0.0)
                .setG2(0.0)
                .setB2(0.0);
    }

    @ParameterizedTest(name = "Test HU networks convergence: {0}")
    @MethodSource("provideNodeBreakerHUNetworks")
    void testConvergenceOnHUData(NetworkPair pair) {
        compareSolvers(pair.rknNetwork(), pair.nrNetwork(), null);
    }

    @Test
    void testConvergenceOnTyndpData() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, "CGM_TYNDP22.xiidm");
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();
        compareSolvers(rknNetwork, nrNetwork, null);
    }

    @Test
    void convertHUDataToBusBreakerTopology() {
        String dataSource = "HU";
        Path initRoot = Path.of(CONFIDENTIAL_DATA_DIR, dataSource);
        Path endRoot = Path.of(CONFIDENTIAL_DATA_DIR_BUS_BREAKER, dataSource);

        String initFileName = "init.xiidm";

        try (Stream<Path> pathStream = Files.list(initRoot)) {
            pathStream.filter(Files::isDirectory)
                    .map(subDir -> subDir.resolve(initFileName))
                    .filter(Files::exists)
                    .forEach(initFile -> {
                        try {
                            Path relativePath = initRoot.relativize(initFile);
                            Path outputFile = endRoot.resolve(relativePath);
                            Files.createDirectories(outputFile.getParent());
                            exportAsBusBreakerTopology(initFile, outputFile);
                        } catch (Exception e) {
                            throw new RuntimeException("Failed to convert the files", e);
                        }
                    });
        } catch (Exception e) {
            throw new RuntimeException("Failed to load HU real network cases", e);
        }
    }

    record NetworkPair(Network rknNetwork, Network nrNetwork, String baseFilename) {
    }

    record VoltagePerturbation(String noGeneratorBusID,
                               String regulatingBusID,
                               String generatorID,
                               String lowImpedanceLineID) {
    }

    record ReactivePowerPerturbation(String targetBusID,
                                     String targetGeneratorID,
                                     String targetShuntID) {

    }
}

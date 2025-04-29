package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.ieeecdf.converter.IeeeCdfNetworkFactory;
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
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.function.Supplier;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 */
public class ResilientAcLoadFlowTest {
    private static final String DEFAULT_OUTPUT_DIR = "./outputs/";
    private static final double DEFAULT_TOLERANCE = 1e-3;
    private static final double BASE_100MVA = 100.0;
    private static final String RKN = "KNITRO";
    private static final String NR = "NEWTON_RAPHSON";
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

    static Stream<NetworkPair> provideRealNetworkData() {
        Path baseDir = Path.of("../../data_confidential/HU");
        String initFileName = "init.xiidm";

        try {
            List<NetworkPair> cases = Files.list(baseDir)
                    .filter(Files::isDirectory)
                    .map(subDir -> subDir.resolve(initFileName))
                    .filter(Files::exists)
                    .map(initPath -> {
                        Network nrNetwork = Network.read(initPath).getNetwork();
                        Network rknNetwork = Network.read(initPath).getNetwork();
                        String name = initPath.getParent().getFileName().toString();
                        return new NetworkPair(rknNetwork, nrNetwork, name);
                    })
                    .toList();

            return cases.stream();

        } catch (Exception e) {
            throw new RuntimeException("Failed to load HU real network cases", e);
        }
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

    @ParameterizedTest
    @MethodSource("provideNetworks")
    void testLoadFlowComparisonOnVariousI3ENetworks(NetworkPair pair) {
        compareSolvers(pair.rknNetwork(), pair.nrNetwork(), null);
    }

    @Test
    void testResilienceWithHighSusceptanceOnVariousI3ENetworks() {
        Stream.<Map.Entry<String, Supplier<Network>>>of(
                Map.entry("ieee14", IeeeCdfNetworkFactory::create14),
                Map.entry("ieee30", IeeeCdfNetworkFactory::create30),
                Map.entry("ieee118", IeeeCdfNetworkFactory::create118),
                Map.entry("ieee300", IeeeCdfNetworkFactory::create300)
        ).forEach(entry -> {
            String name = entry.getKey();
            Supplier<Network> factory = entry.getValue();

            Network rknNetwork = factory.get();
            Network nrNetwork = factory.get();

            // Clone variant for baseline comparison
            final String variantId = "PERTURBED";
            rknNetwork.getVariantManager().cloneVariant(VariantManagerConstants.INITIAL_VARIANT_ID, variantId);
            rknNetwork.getVariantManager().setWorkingVariant(variantId);

            applyHighSusceptancePerturbation(rknNetwork);
            applyHighSusceptancePerturbation(nrNetwork);

            // Newton-Raphson
            configureSolver(NR);
            LoadFlowResult resultNR = loadFlowRunner.run(nrNetwork, parameters);
            boolean isConverged = resultNR.isFullyConverged();
            assertFalse(isConverged, name + ": NR should not converge");

            // Knitro Resilient
            configureSolver(RKN);
            LoadFlowResult resultRKN = loadFlowRunner.run(rknNetwork, parameters);
            isConverged = resultRKN.isFullyConverged();
            assertTrue(isConverged, name + ": Knitro should converge");

        });
    }

    private void applyHighSusceptancePerturbation(Network network) {
        //TODO: change the perturbation logic to match the new requirements (more targeted --> between two generators)
        List<Line> lines = new ArrayList<>();
        List<Load> loads = new ArrayList<>();
        network.getLines().forEach(lines::add);
        network.getLoads().forEach(loads::add);

        // Perturb the lines by setting their reactance and shunt admittance to 0
        int limit = Math.min(lines.size(), (int) (0.2 * lines.size()));
        for (int i = 0; i < limit; i++) {
            Line line = lines.get(i);
            line.setX(0.0);
            line.setG1(0.0);
            line.setB1(0.0);
            line.setG2(0.0);
            line.setB2(0.0);
        }

        // Perturb the loads by increasing their reactive charges
        limit = Math.min(loads.size(), (int) (0.1 * loads.size()));
        for (int i = 0; i < limit; i++) {
            Load load = loads.get(i);
            load.setQ0(load.getQ0() + 500.0);
        }
    }

    @ParameterizedTest(name = "Test HU networks convergence: {0}")
    @MethodSource("provideRealNetworkData")
    void testConvergenceOnHUData(NetworkPair pair) {
        compareSolvers(pair.rknNetwork(), pair.nrNetwork(), null);
    }

    record NetworkPair(Network rknNetwork, Network nrNetwork, String baseFilename) {
    }
}

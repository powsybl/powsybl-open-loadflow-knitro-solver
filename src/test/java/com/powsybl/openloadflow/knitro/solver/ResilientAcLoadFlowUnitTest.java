package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.iidm.network.*;
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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.file.Path;

import static com.powsybl.openloadflow.knitro.solver.NetworkProviders.*;
import static org.junit.jupiter.api.Assertions.*;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public class ResilientAcLoadFlowUnitTest {
    private static final Logger LOGGER = LoggerFactory.getLogger(ResilientAcLoadFlowUnitTest.class);
    private static final double DEFAULT_TOLERANCE = 1e-3;
    private static final double BASE_100MVA = 100.0;
    private static final boolean EXPORT = false;
    private static final String RKN = "KNITRO";
    private static final String NR = "NEWTON_RAPHSON";
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new SparseMatrixFactory()));
        parameters = new LoadFlowParameters()
                .setUseReactiveLimits(false)
                .setVoltageInitMode(LoadFlowParameters.VoltageInitMode.DC_VALUES)
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
        // Newton-Raphson
        configureSolver(NR);
        LoadFlowResult result2 = loadFlowRunner.run(nrNetwork, parameters);
        LOGGER.info("==== Test Information ====");
        LOGGER.info("Algorithm : NR");
        LOGGER.info("Type : unit");
        LOGGER.info("Network name : {}", baseFilename);
        assertTrue(result2.isFullyConverged());

        // Knitro Resilient
        configureSolver(RKN);
        LoadFlowResult result1 = loadFlowRunner.run(rknNetwork, parameters);
        LOGGER.info("==== Test Information ====");
        LOGGER.info("Algorithm : RKN");
        LOGGER.info("Type : unit");
        LOGGER.info("Network name : {}", baseFilename);
        assertTrue(result1.isFullyConverged());

        checkElectricalQuantities(rknNetwork, nrNetwork, DEFAULT_TOLERANCE);

        if (EXPORT) {
            NetworkProviders.writeXML(nrNetwork, baseFilename + "-NR.xml");
            NetworkProviders.writeXML(rknNetwork, baseFilename + "-RKN.xml");
        }
    }

    @ParameterizedTest
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testLoadFlowComparisonOnVariousI3ENetworks(NetworkPair pair) {
        compareSolvers(pair.rknNetwork(), pair.nrNetwork(), pair.baseFilename());
    }

    @Test
    void testConvergenceOnHUInstance() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, HU_INSTANCE);
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();
        compareSolvers(rknNetwork, nrNetwork, "HU_INSTANCE");
    }

    @ParameterizedTest(name = "Test HU networks convergence: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideNodeBreakerHUNetworks")
    @Disabled("Temporarily disabled")
    void testConvergenceOnHUData(NetworkPair pair) {
        compareSolvers(pair.rknNetwork(), pair.nrNetwork(), pair.baseFilename());
    }

    @Test
    void testConvergenceOnESData() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, ES_INSTANCE);
        Network nrNetwork = Network.read(fileName).getNetwork();
        Network rknNetwork = Network.read(fileName).getNetwork();
        compareSolvers(rknNetwork, nrNetwork, "ES");
    }

    @ParameterizedTest
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    void testLoadFlowComparisonOnRteNetworks(NetworkPair pair) {
        compareSolvers(pair.rknNetwork(), pair.nrNetwork(), pair.baseFilename());
    }

    @Test
    @Disabled("Temporarily disabled")
    void testConvergenceOnTyndpData() {
        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, TYNDP_INSTANCE);
        Network network = Network.read(fileName).getNetwork();
        configureSolver(RKN);
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());
    }
}

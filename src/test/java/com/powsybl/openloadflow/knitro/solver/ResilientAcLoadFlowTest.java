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

import java.nio.file.Path;
import java.util.*;
import java.util.function.Supplier;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 */
public class ResilientAcLoadFlowTest {

    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;

    private static final String DEFAULT_OUTPUT_DIR = "./outputs/";

    private static final double DEFAULT_P_TOLERANCE = 1e-2;
    private static final double DEFAULT_Q_TOLERANCE = 1e-2;
    private static final double DEFAULT_V_TOLERANCE = 1e-2;
    private static final double DEFAULT_I_TOLERANCE = 1e-2;
    private static final double DEFAULT_PHI_TOLERANCE = 1e-2;

    private static final String RKN = "KNITRO";
    private static final String NR = "NEWTON_RAPHSON";

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
            knitroParams.setGradientComputationMode(2);
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

    private void checkElectricalQuantities(Network n1, Network n2, double pTolerance, double qTolerance, double vTolerance, double iTolerance, double phiTolerance) {
        // Check generator active and reactive powers
        for (Generator g1 : n1.getGenerators()) {
            Generator g2 = n2.getGenerator(g1.getId());
            assertNotNull(g2, "Generator " + g1.getId() + " not found in second network");
            Terminal t1 = g1.getTerminal();
            Terminal t2 = g2.getTerminal();

            assertEquals(t1.getP(), t2.getP(), pTolerance, "Mismatch on P for generator " + g1.getId());
            assertEquals(t1.getQ(), t2.getQ(), qTolerance, "Mismatch on Q for generator " + g1.getId());
        }

        // Check load active and reactive powers
        for (Load l1 : n1.getLoads()) {
            Load l2 = n2.getLoad(l1.getId());
            assertNotNull(l2, "Load " + l1.getId() + " not found in second network");
            Terminal t1 = l1.getTerminal();
            Terminal t2 = l2.getTerminal();

            assertEquals(t1.getP(), t2.getP(), pTolerance, "Mismatch on P for load " + l1.getId());
            assertEquals(t1.getQ(), t2.getQ(), qTolerance, "Mismatch on Q for load " + l1.getId());
        }

        // Check bus voltages and angles
        for (Bus bus1 : n1.getBusView().getBuses()) {
            Bus bus2 = n2.getBusView().getBus(bus1.getId());
            assertNotNull(bus2, "Bus " + bus1.getId() + " not found in second network");
            assertEquals(bus1.getV(), bus2.getV(), vTolerance, "Mismatch on V for bus " + bus1.getId());
            assertEquals(bus1.getAngle(), bus2.getAngle(), phiTolerance, "Mismatch on Phi for bus " + bus1.getId());
        }

        // Check line current magnitudes
        for (Line line1 : n1.getLines()) {
            Line line2 = n2.getLine(line1.getId());
            assertNotNull(line2, "Line " + line1.getId() + " not found in second network");

            double i1T1 = line1.getTerminal1().getI();
            double i2T1 = line2.getTerminal1().getI();
            assertEquals(i1T1, i2T1, iTolerance, "Mismatch on I (terminal1) for line " + line1.getId());

            double i1T2 = line1.getTerminal2().getI();
            double i2T2 = line2.getTerminal2().getI();
            assertEquals(i1T2, i2T2, iTolerance, "Mismatch on I (terminal2) for line " + line1.getId());
        }
    }

    private void compareSolvers(Network rknNetwork, Network nrNetwork, String baseFilename) {
        configureSolver(RKN);
        LoadFlowResult result1 = loadFlowRunner.run(rknNetwork, parameters);
        assertTrue(result1.isFullyConverged());

        configureSolver(NR);
        LoadFlowResult result2 = loadFlowRunner.run(nrNetwork, parameters);
        assertTrue(result2.isFullyConverged());

        checkElectricalQuantities(rknNetwork, nrNetwork, DEFAULT_P_TOLERANCE, DEFAULT_Q_TOLERANCE, DEFAULT_V_TOLERANCE, DEFAULT_I_TOLERANCE, DEFAULT_PHI_TOLERANCE);

        if (baseFilename != null) {
            writeXML(nrNetwork, baseFilename + "-NR.xml");
            writeXML(rknNetwork, baseFilename + "-RKN.xml");
        }
    }

    static Stream<NetworkPair> provideNetworks() {
        return Stream.of(
                new NetworkPair(IeeeCdfNetworkFactory.create14(), IeeeCdfNetworkFactory.create14(), "ieee14"),
                new NetworkPair(IeeeCdfNetworkFactory.create30(), IeeeCdfNetworkFactory.create30(), "ieee30"),
                new NetworkPair(IeeeCdfNetworkFactory.create118(), IeeeCdfNetworkFactory.create118(), "ieee118"),
                new NetworkPair(IeeeCdfNetworkFactory.create300(), IeeeCdfNetworkFactory.create300(), "ieee300")
        );
    }

    @ParameterizedTest
    @MethodSource("provideNetworks")
    void testLoadFlowComparisonOnVariousI3ENetworks(NetworkPair pair) {
        compareSolvers(pair.rknNetwork(), pair.nrNetwork(), null);
    }

    record NetworkPair(Network rknNetwork, Network nrNetwork, String baseFilename) {
    }

    @Test
    void testResilienceWithLowImpedanceLineOnVariousI3ENetworks() {
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

            double r = 1e-3;
            double x = 1e-3;

            //General case
            List<String> targets = getLowImpedanceLineToChange(nrNetwork);
            applyLowImpedanceLineChanges(rknNetwork, targets, r, x);
            applyLowImpedanceLineChanges(nrNetwork, targets, r, x);

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

    private List<String> getLowImpedanceLineToChange(Network network) {
        String targetBusID;
        String connectedBusID;
        String targetGeneratorID;
        String targetLineID;

        //Selecting a random generator from network
        Generator gen = network.getGenerators()
                .iterator()
                .next();
        targetGeneratorID = gen.getId();

        //The bus associated with the target generator
        targetBusID = gen.getTerminal()
                .getBusBreakerView()
                .getBus()
                .getId();

        //Selecting a random line that is connected to the bus of the generator
        Line line = gen.getTerminal()
                .getBusBreakerView()
                .getBus()
                .getLines()
                .iterator()
                .next();
        targetLineID = line.getId();

        //Getting the bus associated with the other end of selected line
        connectedBusID = line.getTerminal1()
                .getBusBreakerView()
                .getBus()
                .getId();
        if (connectedBusID.equals(targetBusID)) {
            connectedBusID = line.getTerminal2()
                    .getBusBreakerView()
                    .getBus()
                    .getId();
        }

        //Choosing a random load for total balance if needed
        String loadID = network.getLoads().iterator().next().getId();

        return Arrays.asList(targetGeneratorID, targetBusID, connectedBusID, targetLineID, loadID);
    }

    private void applyLowImpedanceLineChanges(Network network, List<String> targets, double r, double x) {
        String targetGeneratorID = targets.get(0);
        String targetBusID = targets.get(1);
        String connectedBusID = targets.get(2);
        String targetLineID = targets.get(3);

        Bus connectedBus = network.getBusBreakerView().getBus(connectedBusID);
        Generator targetGenerator = network.getGenerator(targetGeneratorID);

        VoltageLevel connectedVL = connectedBus.getVoltageLevel();
        VoltageLevel targetVL = network.getBusBreakerView()
                .getBus(targetBusID)
                .getVoltageLevel();

        //voltage Mismatch
        double alpha = 0.1;

        //Offsetting connected voltage level
        connectedVL.setNominalV(targetVL.getNominalV() * alpha);

        //adding the connected generator or modifying preexisting one
        boolean hasGenerator = connectedBus
                .getGenerators()
                .iterator()
                .hasNext();
        if (hasGenerator) {
            Generator gConnected = connectedBus
                    .getGenerators()
                    .iterator()
                    .next();
            gConnected.setMinP(targetGenerator.getMinP())
                    .setMaxP(targetGenerator.getMaxP())
                    .setTargetP(0.0)
                    .setTargetQ(0.0)
                    .setTargetV(targetVL.getNominalV() * alpha)
                    .setVoltageRegulatorOn(true);
        } else {
            connectedVL.newGenerator()
                    .setId(connectedBusID + "-" + "G-added")
                    .setBus(connectedBusID)
                    .setMinP(targetGenerator.getMinP())
                    .setMaxP(targetGenerator.getMaxP())
                    .setTargetP(0.0)
                    .setTargetQ(0.0)
                    .setTargetV(targetVL.getNominalV() * alpha)
                    .setVoltageRegulatorOn(true)
                    .add();
        }

        //Changing line parameters by setting their reactance and shunt admittance to 0
        Line lineTarget = network.getLine(targetLineID);
        lineTarget.setR(r)
                .setX(x)
                .setG1(0.0)
                .setB1(0.0)
                .setG2(0.0)
                .setB2(0.0);
    }

    @Test
    void testConvergenceOnRealNetworks() {
        Path huData = Path.of("../../data_confidential/HU/20220226T2330Z_1D_002/init.xiidm");
        Network n1 = Network.read(huData).getNetwork();
        Network n2 = Network.read(huData).getNetwork();

        configureSolver(NR);
        LoadFlowResult resultNR = loadFlowRunner.run(n1, parameters);
        assertTrue(resultNR.isFullyConverged(), "Newton-Raphson should converge");

        configureSolver(RKN);
        LoadFlowResult resultRKN = loadFlowRunner.run(n2, parameters);
        assertTrue(resultRKN.isFullyConverged(), "Knitro should converge");

        checkElectricalQuantities(n1, n2, DEFAULT_P_TOLERANCE, 10e-1, DEFAULT_V_TOLERANCE, 10e-1, DEFAULT_PHI_TOLERANCE);
    }
}

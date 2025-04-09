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
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.nio.file.Path;
import java.util.Properties;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 */
public class AcResilientLoadFlowI3ETest {

    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;
    private static final String OUTPUT_DIR = "./outputs/";
    private static final double PQV_TOLERANCE = 1e-3;
    private static final double FLOW_TOLERANCE = 1e-2;

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

        if ("KNITRO".equals(solver)) {
            KnitroLoadFlowParameters knitroParams = new KnitroLoadFlowParameters();
            knitroParams.setGradientComputationMode(2);
            knitroParams.setKnitroSolverType(KnitroSolverParameters.KnitroSolverType.RESILIENT);
            parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
        }
    }

    private void writeXML(Network network, String name) {
        Properties properties = new Properties();
        properties.put(XMLExporter.VERSION, "1.12");
        Path path = Path.of(OUTPUT_DIR, name);
        network.write("XIIDM", properties, path);
    }

    private void assertTerminalEquals(Terminal t1, Terminal t2) {
        assertEquals(t1.getP(), t2.getP(), PQV_TOLERANCE);
        assertEquals(t1.getQ(), t2.getQ(), PQV_TOLERANCE);
        assertEquals(t1.getVoltageLevel().getNominalV(), t2.getVoltageLevel().getNominalV(), PQV_TOLERANCE);
    }

    private void checkPQV(Network n1, Network n2) {
        for (Generator g1 : n1.getGenerators()) {
            Generator g2 = n2.getGenerator(g1.getId());
            assertTerminalEquals(g1.getTerminal(), g2.getTerminal());
        }
        for (Load l1 : n1.getLoads()) {
            Load l2 = n2.getLoad(l1.getId());
            assertTerminalEquals(l1.getTerminal(), l2.getTerminal());
        }
    }

    private void checkFlows(Network n1, Network n2) {
        n1.getLines().forEach(line1 -> {
            Line line2 = n2.getLine(line1.getId());
            assertEquals(line1.getTerminal1().getI(), line2.getTerminal1().getI(), FLOW_TOLERANCE);
            assertEquals(line1.getTerminal2().getI(), line2.getTerminal2().getI(), FLOW_TOLERANCE);
        });
    }

    private void compareSolvers(Network knitroNetwork, Network newtonNetwork, String baseFilename) {
        configureSolver("KNITRO");
        LoadFlowResult result1 = loadFlowRunner.run(knitroNetwork, parameters);
        assertTrue(result1.isFullyConverged());

        configureSolver("NEWTON_RAPHSON");
        LoadFlowResult result2 = loadFlowRunner.run(newtonNetwork, parameters);
        assertTrue(result2.isFullyConverged());

        checkPQV(knitroNetwork, newtonNetwork);
        checkFlows(knitroNetwork, newtonNetwork);

        writeXML(knitroNetwork, baseFilename + "-KNITRO.xml");
        writeXML(newtonNetwork, baseFilename + "-NR.xml");
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
    void testLoadFlowComparison(NetworkPair pair) {
        compareSolvers(pair.knitroNetwork(), pair.newtonNetwork(), pair.baseFilename());
    }

    record NetworkPair(Network knitroNetwork, Network newtonNetwork, String baseFilename) {
    }
}

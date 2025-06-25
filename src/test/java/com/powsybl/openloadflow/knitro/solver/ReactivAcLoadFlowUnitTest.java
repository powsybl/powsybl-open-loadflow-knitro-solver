package com.powsybl.openloadflow.knitro.solver;

import com.artelys.knitro.api.KNConstants;
import com.powsybl.ieeecdf.converter.IeeeCdfNetworkFactory;
import com.powsybl.iidm.network.*;
import com.powsybl.iidm.network.test.EurostagTutorialExample1Factory;
import com.powsybl.iidm.serde.XMLExporter;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.DenseMatrixFactory;
import com.powsybl.math.matrix.SparseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.network.EurostagFactory;
import com.powsybl.openloadflow.network.LfNetwork;
import com.powsybl.openloadflow.network.SlackBusSelectionMode;
import com.powsybl.openloadflow.network.impl.LfNetworkLoaderImpl;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Properties;
import java.util.stream.Stream;

import static com.powsybl.openloadflow.util.LoadFlowAssert.assertReactivePowerEquals;
import static org.junit.jupiter.api.Assertions.*;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Yoann Anezin {@literal <yoann.anezin at artelys.com>}
 */
public class ReactivAcLoadFlowUnitTest {
    private static final double DEFAULT_TOLERANCE = 1e-3;
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;

    static Stream<ResilientAcLoadFlowUnitTest.NetworkPair> provideI3ENetworks() {
        return Stream.of(
                new ResilientAcLoadFlowUnitTest.NetworkPair(IeeeCdfNetworkFactory.create14(), IeeeCdfNetworkFactory.create14(), "ieee14"),
                new ResilientAcLoadFlowUnitTest.NetworkPair(IeeeCdfNetworkFactory.create30(), IeeeCdfNetworkFactory.create30(), "ieee30"),
                new ResilientAcLoadFlowUnitTest.NetworkPair(IeeeCdfNetworkFactory.create118(), IeeeCdfNetworkFactory.create118(), "ieee118"),
                new ResilientAcLoadFlowUnitTest.NetworkPair(IeeeCdfNetworkFactory.create300(), IeeeCdfNetworkFactory.create300(), "ieee300")
        );
    }

    private void checkNotAllPQ (Network network) {
        boolean allPQ = false;
        for (Generator g : network.getGenerators()) {
            Terminal t = g.getTerminal();

            double v = t.getBusView().getBus().getV();
            allPQ = v + DEFAULT_TOLERANCE > g.getTargetV() && v - DEFAULT_TOLERANCE < g.getTargetV() && g.isVoltageRegulatorOn();
            if (allPQ) {
                break;
            }
        }
        assertTrue(allPQ, "No control on any voltage magnitude : all buses switched");
    }

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new DenseMatrixFactory()));
        parameters = new LoadFlowParameters().setUseReactiveLimits(true)
                .setDistributedSlack(false);
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters(); // set gradient computation mode
        knitroLoadFlowParameters.setGradientComputationMode(2);
        knitroLoadFlowParameters.setKnitroSolverType(KnitroSolverParameters.KnitroSolverType.REACTIVLIMITS);
        parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);
        OpenLoadFlowParameters.create(parameters).setAcSolverType(KnitroSolverFactory.NAME);
    }

    @Test
    void testReacLimEurostag() { /** passe avec et sans outerloop */
        Network network = EurostagFactory.fix(EurostagTutorialExample1Factory.create());

        // access to already created equipments
        Load load = network.getLoad("LOAD");
        VoltageLevel vlgen = network.getVoltageLevel("VLGEN");
        TwoWindingsTransformer nhv2Nload = network.getTwoWindingsTransformer("NHV2_NLOAD");
        Generator gen = network.getGenerator("GEN");
        Substation p1 = network.getSubstation("P1");

        // reduce GEN reactive range
        gen.newMinMaxReactiveLimits()
                .setMinQ(0)
                .setMaxQ(280)
                .add();

        // create a new generator GEN2
        VoltageLevel vlgen2 = p1.newVoltageLevel()
                .setId("VLGEN2")
                .setNominalV(24.0)
                .setTopologyKind(TopologyKind.BUS_BREAKER)
                .add();
        vlgen2.getBusBreakerView().newBus()
                .setId("NGEN2")
                .add();
        Generator gen2 = vlgen2.newGenerator()
                .setId("GEN2")
                .setBus("NGEN2")
                .setConnectableBus("NGEN2")
                .setMinP(-9999.99)
                .setMaxP(9999.99)
                .setVoltageRegulatorOn(true)
                .setTargetV(24.5)
                .setTargetP(100)
                .add();
        gen2.newMinMaxReactiveLimits()
                .setMinQ(0)
                .setMaxQ(100)
                .add();
        int zb380 = 380 * 380 / 100;
        TwoWindingsTransformer ngen2Nhv1 = p1.newTwoWindingsTransformer()
                .setId("NGEN2_NHV1")
                .setBus1("NGEN2")
                .setConnectableBus1("NGEN2")
                .setRatedU1(24.0)
                .setBus2("NHV1")
                .setConnectableBus2("NHV1")
                .setRatedU2(400.0)
                .setR(0.24 / 1800 * zb380)
                .setX(Math.sqrt(10 * 10 - 0.24 * 0.24) / 1800 * zb380)
                .add();

        // fix active power balance
        load.setP0(699.838);

        parameters.setUseReactiveLimits(true);
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged());
        assertReactivePowerEquals(-164.315, gen.getTerminal());
        assertReactivePowerEquals(-100, gen2.getTerminal()); // GEN is correctly limited to 100 MVar
        assertReactivePowerEquals(100, ngen2Nhv1.getTerminal1());
        assertReactivePowerEquals(-200, nhv2Nload.getTerminal2());
        checkNotAllPQ(network);
    }

    @Test
    void testReacLimIeee14OuterloopOn() { /** PQ only */
        parameters.setUseReactiveLimits(true);
        Network network = IeeeCdfNetworkFactory.create14();
        for (var g : network.getGenerators()) {
            g.newMinMaxReactiveLimits()
                    .setMinQ(0)
                    .setMaxQ(100)
                           .add();
         }
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        checkNotAllPQ(network);
    }

    @Test
    void testReacLimIeee14OuterloopOff() { /** PQ only */
        parameters.setUseReactiveLimits(false);
        Network network = IeeeCdfNetworkFactory.create14();
        for (var g : network.getGenerators()) {
            g.newMinMaxReactiveLimits()
                    .setMinQ(0)
                    .setMaxQ(100)
                    .add();
        }
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        checkNotAllPQ(network);
    }

    @Test
    void testReacLimIeee30OuterloopOn() { /** Unfeasible Point */
        parameters.setUseReactiveLimits(true);
        Network network = IeeeCdfNetworkFactory.create30();
        for (var g : network.getGenerators()) {
            g.newMinMaxReactiveLimits()
                    .setMinQ(0)
                    .setMaxQ(100)
                    .add();
        }
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        checkNotAllPQ(network);
    }

    @Test
    void testReacLimIeee30OuterloopOff() { /** Only PQ */
        parameters.setUseReactiveLimits(false);
        Network network = IeeeCdfNetworkFactory.create30();
        for (var g : network.getGenerators()) {
            g.newMinMaxReactiveLimits()
                    .setMinQ(0)
                    .setMaxQ(100)
                    .add();
        }
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        checkNotAllPQ(network);
    }

    @Test
    void testReacLimIeee118OuterloopOn() { /** Unfeasible Point */
        parameters.setUseReactiveLimits(true);
        Network network = IeeeCdfNetworkFactory.create118();
        for (var g : network.getGenerators()) {
            g.newMinMaxReactiveLimits()
                    .setMinQ(0)
                    .setMaxQ(100)
                    .add();
        }
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        checkNotAllPQ(network);
    }

    @Test
    void testReacLimIeee118OuterloopOff() { /** Succeed */
        parameters.setUseReactiveLimits(false);
        Network network = IeeeCdfNetworkFactory.create118();
        for (var g : network.getGenerators()) {
            g.newMinMaxReactiveLimits()
                    .setMinQ(0)
                    .setMaxQ(100)
                    .add();
        }
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        checkNotAllPQ(network);
    }

    @Test
    void testReacLimIeee300OuterloopOn() { /** Unfeasible Point */
        parameters.setUseReactiveLimits(true);
        Network network = IeeeCdfNetworkFactory.create300();
        for (var g : network.getGenerators()) {
            g.newMinMaxReactiveLimits()
                    .setMinQ(0)
                    .setMaxQ(100)
                    .add();
        }
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        checkNotAllPQ(network);
    }

    @Test
    void testReacLimIeee300OuterloopOff() { /** Succeed */
        parameters.setUseReactiveLimits(false);
        Network network = IeeeCdfNetworkFactory.create300();
        for (var g : network.getGenerators()) {
            g.newMinMaxReactiveLimits()
                    .setMinQ(0)
                    .setMaxQ(100)
                    .add();
        }
        LoadFlowResult result = loadFlowRunner.run(network, parameters);
        assertTrue(result.isFullyConverged(), "Not Fully Converged");
        checkNotAllPQ(network);
    }
}

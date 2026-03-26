package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.ieeecdf.converter.IeeeCdfNetworkFactory;
import com.powsybl.iidm.network.*;
import com.powsybl.iidm.serde.XMLExporter;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.SparseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.ac.AcLoadFlowContext;
import com.powsybl.openloadflow.ac.AcLoadFlowParameters;
import com.powsybl.openloadflow.ac.AcLoadFlowResult;
import com.powsybl.openloadflow.ac.AcloadFlowEngine;
import com.powsybl.openloadflow.graph.EvenShiloachGraphDecrementalConnectivityFactory;
import com.powsybl.openloadflow.network.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Properties;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import static com.powsybl.openloadflow.knitro.solver.NetworkProviders.*;
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
    private static final boolean EXPORT = true;
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;
    public  static final String CORRECTION_FOLDER = "CORRECTION/";

//    private static Optional<String> DEFAULT_EXPORT_SOLUTION = Optional.empty();

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

           // String fullpath = CORRECTION_FOLDER +test+"/"+filepath.orElse("");
            KnitroSolverParameters.DEFAULT_EXPORT_SOLUTION = filepath;
        }
    }

    private void compareResilience(Network rknNetwork, Network nrNetwork, String baseFilename, String perturbationType,String test) {
        // Newton-Raphson
        Path path = Path.of(test, baseFilename);
        Path filePath = path.resolve(baseFilename + "_" + perturbationType);
        try {
            Files.createDirectories(filePath.getParent());
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
        Optional<String> filepath = Optional.of(filePath.toString());
//        Path path = Path.of(test, baseFilename);
//       // String filename = baseFilename + "_" + perturbationType + ".csv";
//        Path filePath = path.resolve(baseFilename + "_" + perturbationType);
//        Optional<String> filepath = Optional.of(filePath.toString());
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
        //assertTrue(isConvergedRKN, baseFilename + ": Knitro should converge");

        if (EXPORT) {
            NetworkProviders.writeXML(rknNetwork, baseFilename + "-" + perturbationType + ".xml");
        }

    }

    private void voltagePerturbationTest(Network rknNetwork, Network nrNetwork, String baseFilename, double rPU, double xPU, double alpha,String test) {
        PerturbationFactory.VoltagePerturbation perturbation = PerturbationFactory.getVoltagePerturbation(nrNetwork);
        PerturbationFactory.applyVoltagePerturbation(rknNetwork, perturbation, rPU, xPU, alpha);
        PerturbationFactory.applyVoltagePerturbation(nrNetwork, perturbation, rPU, xPU, alpha);
        compareResilience(rknNetwork, nrNetwork, baseFilename, VOLTAGE_PERTURBATION,test);
        System.out.println("Voltage perturbation applied on buses " + perturbation + " with alpha = " + alpha);
    }

    private void activePowerPerturbationTest(Network rknNetwork, Network nrNetwork, String baseFilename, double alpha, String test, int loadIndex) {
        String targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork,loadIndex);
        PerturbationFactory.applyActivePowerPerturbation(rknNetwork, targetLoadID, alpha);
        PerturbationFactory.applyActivePowerPerturbation(nrNetwork, targetLoadID, alpha);
        compareResilience(rknNetwork, nrNetwork, baseFilename, ACTIVE_POWER_PERTURBATION,test);
        System.out.println("Active power perturbation applied on load " + targetLoadID + " with alpha = " + alpha);
    }

    private void reactivePowerPerturbationTest(Network rknNetwork, Network nrNetwork, String baseFilename, double targetQ, String test) {
        PerturbationFactory.ReactivePowerPerturbation perturbation = PerturbationFactory.getReactivePowerPerturbation(nrNetwork);
        PerturbationFactory.applyReactivePowerPerturbation(rknNetwork, perturbation, targetQ);
        PerturbationFactory.applyReactivePowerPerturbation(nrNetwork, perturbation, targetQ);
        compareResilience(rknNetwork, nrNetwork, baseFilename, REACTIVE_POWER_PERTURBATION,test);
    }

    protected void updateNetworkCorrection (Network network,Network network2, String baseFilename, String busId, String controltype, String controleID, double correction,String filenameSuffix, String test) {
        Bus bus = network.getBusView().getBus(busId);
        if (bus == null) {
            LOGGER.warn("Bus {} not found in the network.", busId);
            return;
        }
        switch (controltype) {
            case "generator" -> {
                Generator gen = network.getGenerator(controleID);
//                Optional<LfGenerator> maybeGenerator = bus.getGenerators().stream().filter(g -> controleID.equals(g.getId()))
//                        .findFirst();
//                maybeGenerator.ifPresent(g -> g.setTargetP(g.getTargetP() + contribution));
                LOGGER.info("Generator before firste modification: {}", gen.getTargetP());
                gen.setTargetP(gen.getTargetP()+correction);
                if (gen == null) {
                    LOGGER.warn("Generator {} not found on bus {}", controleID, busId);
                }
                else {
                    LOGGER.info("Generator {} found on bus {}, applied correction of {}", controleID, busId, correction);
                    LOGGER.info("New targetP for generator {}: {}", controleID, gen.getTargetP());
                }
            }
            case "load_P" -> {
                Load load = network.getLoad(controleID);
                LOGGER.info("Load before firste modification: {}, correction to do {}", load.getP0(), correction);

                load.setP0(load.getP0() + correction);
                if (load == null) {
                    LOGGER.warn("Load {} not found on bus {}", controleID, busId);
                }
                LOGGER.info("Load after firste modification: {}", load.getP0());

            }
            case "load_Q" -> {
                Load load = network.getLoad(controleID);
                load.setQ0((load.getQ0()+ correction) );
                if (load == null) {
                    LOGGER.warn("Load {} not found on bus {}", controleID, busId);
                }
            }
            case "shunt" -> {
                ShuntCompensator shunt = network.getShuntCompensator(controleID);
                shunt.setSectionCount((int) (shunt.getSectionCount() + correction));
//                ShuntCompensator shunt = network.getShuntCompensator(controleID);
//                shunt.setQ(shunt.getTargetQ() + correction);
                if (shunt == null) {
                    LOGGER.warn("Shunt compensator {} not found on bus {}", controleID, busId);
                }
            }
            case "gen_V" -> {
                Generator genV = network.getGenerator(controleID);
                LOGGER.info("Generator before firste modification: {}", genV.getTargetV());

                genV.setTargetV(genV.getTargetV() + correction);
                LOGGER.info("Generator after modification: {}", genV.getTargetV());

                if (genV == null) {
                    LOGGER.warn("Generator {} not found on bus {}", controleID, busId);
                }
            }
             default -> LOGGER.warn("Unknown control type: {}", controltype);
        }
     //   System.out.println("Applied correction of " + correction + " on " + controltype + " " + controleID + " at bus " + busId "new value: " + genera);
    compareResilience(network, network2, baseFilename , filenameSuffix,test);
    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testVoltagePerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test ="test_V_IEEE";
        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;
        voltagePerturbationTest(rknNetwork, nrNetwork, baseFilename, rPU, xPU, alpha,test);
    }
//
//    @Test
//    void testVoltagePerturbationOnHUInstance() {
//        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, HU_INSTANCE);
//        Network nrNetwork = Network.read(fileName).getNetwork();
//        Network rknNetwork = Network.read(fileName).getNetwork();
//        String test ="test_V_HU";
//        // Line Characteristics in per-unit
//        double rPU = 0.0;
//        double xPU = 1e-5;
//        // Voltage Mismatch
//        double alpha = 0.95;
//
//        voltagePerturbationTest(rknNetwork, nrNetwork, "HU", rPU, xPU, alpha,test);
//    }

//    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on HU networks: {0}")
//    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideNodeBreakerHUNetworks")
//    @Disabled("Temporarily disabled")
//    void testVoltagePerturbationOnHUData(NetworkPair pair) {
//        String baseFilename = pair.baseFilename();
//
//        Network rknNetwork = pair.rknNetwork();
//        Network nrNetwork = pair.nrNetwork();
//        String test ="test_V_HU";
//        // Line Characteristics in per-unit
//        double rPU = 0.0;
//        double xPU = 1e-5;
//        // Voltage Mismatch
//        double alpha = 0.95;
//
//        voltagePerturbationTest(rknNetwork, nrNetwork, baseFilename, rPU, xPU, alpha,test);
//    }

//    @Test
//    void testVoltagePerturbationOnESData() {
//        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, ES_INSTANCE);
//        Network nrNetwork = Network.read(fileName).getNetwork();
//        Network rknNetwork = Network.read(fileName).getNetwork();
//
//        // Line Characteristics in per-unit
//        double rPU = 0.0;
//        double xPU = 1e-5;
//        // Voltage Mismatch
//        double alpha = 0.95;
//
//        voltagePerturbationTest(rknNetwork, nrNetwork, "ES", rPU, xPU, alpha);
//    }

    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on RTE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    void testVoltagePerturbationOnRteNetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test ="test_V_RTE";
        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;
        voltagePerturbationTest(rknNetwork, nrNetwork, baseFilename, rPU, xPU, alpha,test);
    }

    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on various IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testActivePowerPerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test ="test_active_power_IEEE";
        // Final perturbed load's percentage
        double alpha = 0.1;
        int loadindex = 0;
        activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, alpha,test,loadindex);

    }
    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on various IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void testandcorrectionActivePowerPerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test ="test_active_power_and_correction";
        double alpha = 0.1;
        int loadindex = 0;

        activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, alpha,test, loadindex);
        System.out.print("\n -------------------First correction in P---------------------- \n");
        updateNetworkCorrection(rknNetwork,nrNetwork,baseFilename, "VL3_0", "load_P", "B3-L", -391.9002,"1",test);
        System.out.print("\n -------------------2 correction in P---------------------- \n");
        updateNetworkCorrection(rknNetwork,nrNetwork,baseFilename, "VL110_0", "load_P", "B110-L", -29.8138,"2",test);
        System.out.print("\n -------------------3 correction in P---------------------- \n");
        updateNetworkCorrection(rknNetwork,nrNetwork,baseFilename, "VL528_0", "load_P", "B528-L", -14.1953,"3",test);
        System.out.print("\n -------------------4 correction in P---------------------- \n");

        updateNetworkCorrection(rknNetwork,nrNetwork,baseFilename, "VL125_0", "load_P", "B125-L", -9.3403,"4",test);
        System.out.print("\n -------------------5 correction in P---------------------- \n");
        updateNetworkCorrection(rknNetwork,nrNetwork,baseFilename, "VL528_0", "load_P", "B528-L", -7.5012,"5",test);
        System.out.print("\n -------------------6 correction in P---------------------- \n");
        updateNetworkCorrection(rknNetwork,nrNetwork,baseFilename, "VL114_0", "load_P", "B114-L", -5.2244,"6" ,test);
        System.out.print("\n -------------------7 correction in P---------------------- \n");
        updateNetworkCorrection(rknNetwork,nrNetwork,baseFilename, "VL123_0", "load_P", "B123-L", -4.8755,"7",test);
    }


    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on various IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void correctionmoreActivePowerPerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test = "test_active_power_and_correction_more_ieee";
        // Final perturbed load's percentage
        double alpha = 0.1;
        // Forcer l'export CSV dans un dossier précis
        int loadindex =0;
        activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, alpha,test, loadindex);
        System.out.print("\n -------------------First correction in P 10% more ---------------------- \n");
        updateNetworkCorrection(rknNetwork, nrNetwork, baseFilename, "VL3_0", "load_P", "B3-L", -431.0903, "1",test);
        System.out.print("\n -------------------2 correction in P---------------------- \n");
        updateNetworkCorrection(rknNetwork, nrNetwork, baseFilename, "VL114_0", "load_P", "B114-L", -18.0071, "2",test);
        System.out.print("\n -------------------3 correction in P---------------------- \n");
        updateNetworkCorrection(rknNetwork, nrNetwork, baseFilename, "VL528_0", "load_P", "B528-L", -12.881, "3",test);
        System.out.print("\n -------------------4 correction in P---------------------- \n");
        updateNetworkCorrection(rknNetwork, nrNetwork, baseFilename, "VL528_0", "load_P", "B528-L", -6.5441, "4",test);
        System.out.print("\n -------------------5 correction in P---------------------- \n");
        updateNetworkCorrection(rknNetwork, nrNetwork, baseFilename, "VL528_0", "load_P", "B528-L", -3.2459, "5",test);
    }
    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on various IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void correction20ActivePowerPerturbationOnVariousI3ENetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test = "test_active_power_and_correction_20_ieee";
        // Final perturbed load's percentage
        double alpha = 0.1;
        // Forcer l'export CSV dans un dossier précis
        int loadindex =0;
        activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, alpha,test, loadindex);
        System.out.print("\n -------------------First correction in P 20% more ---------------------- \n");
        updateNetworkCorrection(rknNetwork, nrNetwork, baseFilename, "VL3_0", "load_P", "B3-L", -470.2803, "1",test);
        System.out.print("\n -------------------2 correction in P---------------------- \n");
        updateNetworkCorrection(rknNetwork, nrNetwork, baseFilename, "VL528_0", "load_P", "B528-L", -8.41008, "2",test);

//        updateNetworkCorrection(rknNetwork, nrNetwork, baseFilename, "VL114_0", "load_P", "B114-L", -18.0071, "2",test);
        System.out.print("\n -------------------3 correction in P---------------------- \n");
        updateNetworkCorrection(rknNetwork, nrNetwork, baseFilename, "VL528_0", "load_P", "B528-L", -2.71824, "3",test);
//        System.out.print("\n -------------------4 correction in P---------------------- \n");
//        updateNetworkCorrection(rknNetwork, nrNetwork, baseFilename, "VL528_0", "load_P", "B528-L", -6.5441, "4",test);
//        System.out.print("\n -------------------5 correction in P---------------------- \n");
//        updateNetworkCorrection(rknNetwork, nrNetwork, baseFilename, "VL528_0", "load_P", "B528-L", -3.2459, "5",test);
    }
//    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on various IEEE networks: {0}")
//    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
//    void correction_load2_activepower_IEEE(NetworkPair pair) {
//        String baseFilename = pair.baseFilename();
//        String test = "test_active_power_and_correction_load2_ieee";
//        double alpha = 0.1;
//        List<String> loadIDs = new ArrayList<>();
//
//        for (int loadindex = 0; loadindex < 3; loadindex++) {
//            Network rknNetwork = new pair.rknNetwork();
//            Network nrNetwork = pair.nrNetwork();
//
//            String targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork, loadindex);
//            loadIDs.add(targetLoadID);
//
//            activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename + "_load" + loadindex, alpha, test, loadindex);
//        }
//
//        try (var writer = new java.io.FileWriter("loadIDs.csv")) {
//            for (String id : loadIDs) {
//                writer.write(id + "\n");
//            }
//        } catch (IOException e) {
//            LOGGER.error("Error writing loadIDs to file", e);
//        }
//    }

//    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on various IEEE networks: {0}")
//    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
//    void correction_load2_activepower_IEEE(NetworkPair pair) {
//        String baseFilename = pair.baseFilename();
////        Network rknNetwork = pair.rknNetwork();
//        Network nrNetwork_load = pair.nrNetwork();
//        String test = "test_active_power_and_correction_load2_ieee";
//        // Final perturbed load's percentage
//        double alpha = 0.1;
//        int loadindex = 0;
//        // Forcer l'export CSV dans un dossier précis
//        String targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork_load, loadindex);
//        List<String> loadIDs = new ArrayList<>();
//
//        for ( loadindex = 0; loadindex < 3; loadindex++) {
//            Network rknNetwork = pair.rknNetwork();
//            Network nrNetwork = pair.nrNetwork();
//            activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename+"_load"+loadindex, alpha, test, loadindex);
//            loadindex = loadindex + 1;
//            loadIDs.add(targetLoadID);
//
//            targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork, loadindex);
//        }
////        try (var writer = new java.io.FileWriter("loadIDs.csv")) {
////            for (String id : loadIDs) {
////                writer.write(id + "\n");
////            }
////        } catch (IOException e) {
////            LOGGER.error("Error writing loadIDs to file", e);
////        }
//    }

    @ParameterizedTest(name = "Test resilience of RKN: {0} - Load index {1}")
    @MethodSource("provideNetworksAndLoadIndices")
    void correction_all_load_activepower_IEEE(NetworkPair pair, int loadindex) {
        String baseFilename = pair.baseFilename();
        Network rknNetwork = IeeeCdfNetworkFactory.create300();
        Network nrNetwork = IeeeCdfNetworkFactory.create300();
        String test = "test_active_power_and_correction_all_load_ieee";
        double alpha = 0.1;
        activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename + "_load" + loadindex, alpha, test, loadindex);
        }
    static Stream<Arguments> provideNetworksAndLoadIndices() {
        return provideI3ENetworks()
                .flatMap(pair -> IntStream.range(0, 188)
                        .mapToObj(i -> Arguments.of(pair, i)));
    }
    @ParameterizedTest(name = "Test resilience of RKN: {0} - Load index {1}")
    @MethodSource("provideNetworksAndLoadIndices")
    void correction_all_load_activepower_5_IEEE(NetworkPair pair, int loadindex) {
        String baseFilename = pair.baseFilename();
        Network rknNetwork = IeeeCdfNetworkFactory.create300();
        Network nrNetwork = IeeeCdfNetworkFactory.create300();
        String test = "test_active_power_and_correction_all_load_5_ieee";
        double alpha = 0.5;

        activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename + "_load" + loadindex, alpha, test, loadindex);
    }

    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on various IEEE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideI3ENetworks")
    void correction_nonconvKN_IEEE(NetworkPair pair){
        String baseFilename = pair.baseFilename();
        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test = "test_active_power_BL_ieee";
        // Final perturbed load's percentage
        double alpha = 0.1;
        int loadindex = 1;
        // Forcer l'export CSV dans un dossier précis
      //  String targetLoadID = PerturbationFactory.getActivePowerPerturbation(nrNetwork, loadindex);
        activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename+"_load"+loadindex, alpha, test, loadindex);
    }
    @ParameterizedTest(name = "Test resilience of RKN to a voltage perturbation on RTE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    void testandcorrectionVoltagePerturbationOnRteNetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test ="test_V_RTE";
        // Line Characteristics in per-unit
        double rPU = 0.0;
        double xPU = 1e-5;
        // Voltage Mismatch
        double alpha = 0.95;
        voltagePerturbationTest(rknNetwork, nrNetwork, baseFilename, rPU, xPU, alpha,test);
//        System.out.print("\n -------------------First correction in P 10% more ---------------------- \n");
//        updateNetworkCorrection(rknNetwork, nrNetwork, baseFilename, "VL3_0", "load_P", "B3-L", -431.0903, "1",test);
    }

//        @Test
//    void testActivePowerPerturbationOnHUInstance() {
//        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, HU_INSTANCE);
//        Network nrNetwork = Network.read(fileName).getNetwork();
//        Network rknNetwork = Network.read(fileName).getNetwork();
//
//        // Final perturbed load's percentage
//        double alpha = 0.30;
//
//        activePowerPerturbationTest(rknNetwork, nrNetwork, "HU", alpha);
//    }

//    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on HU networks: {0}")
//    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideNodeBreakerHUNetworks")
//    @Disabled("Temporarily disabled")
//    void testActivePowerPerturbationOnHUData(NetworkPair pair) {
//        Network rknNetwork = pair.rknNetwork();
//        Network nrNetwork = pair.nrNetwork();
//
//        // Final perturbed load's percentage
//        double alpha = 0.30;
//
//        activePowerPerturbationTest(rknNetwork, nrNetwork, null, alpha);
//    }

//    @Test
//    void testActivePowerPerturbationOnESData() {
//        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, ES_INSTANCE);
//        Network nrNetwork = Network.read(fileName).getNetwork();
//        Network rknNetwork = Network.read(fileName).getNetwork();
//
//        // Final perturbed load's percentage
//        double alpha = 0.10;
//
//        activePowerPerturbationTest(rknNetwork, nrNetwork, "ES", alpha);
//    }

    @ParameterizedTest(name = "Test resilience of RKN to active power perturbation on RTE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    void testActivePowerPerturbationOnRteNetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test = "test_active_power_RTE";
        // Final perturbed load's percentage
        double alpha = 0.30;

        activePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, alpha,test,0);
    }

//    @Test
//    void tesReactivePowerPerturbationOnHUInstance() {
//        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR_BUS_BREAKER, HU_INSTANCE);
//        Network nrNetwork = Network.read(fileName).getNetwork();
//        Network rknNetwork = Network.read(fileName).getNetwork();
//
//        // Target reactive power injection by the shunt section in VArs
//        double targetQ = 1e9;
//
//        reactivePowerPerturbationTest(rknNetwork, nrNetwork, "HU", targetQ);
//    }

//    @ParameterizedTest(name = "Test resilience of RKN to reactive power perturbation on HU networks: {0}")
//    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideBusBreakerHUNetworks")
//    @Disabled("Temporarily disabled")
//    void testReactivePowerPerturbationOnHUData(NetworkPair pair) {
//        String baseFilename = pair.baseFilename();
//
//        Network rknNetwork = pair.rknNetwork();
//        Network nrNetwork = pair.nrNetwork();
//
//        // Target reactive power injection by the shunt section in VArs
//        double targetQ = 1e9;
//
//        reactivePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, targetQ);
//    }
//
//    @Test
//    void testReactivePowerPerturbationOnESData() {
//        Path fileName = Path.of(CONFIDENTIAL_DATA_DIR, ES_INSTANCE);
//        Network nrNetwork = Network.read(fileName).getNetwork();
//        Network rknNetwork = Network.read(fileName).getNetwork();
//
//        // Target reactive power injection by the shunt section in VArs
//        double targetQ = 5e9;
//
//        reactivePowerPerturbationTest(rknNetwork, nrNetwork, "ES", targetQ);
//    }

    @ParameterizedTest(name = "Test resilience of RKN to reactive power perturbation on RTE networks: {0}")
    @MethodSource("com.powsybl.openloadflow.knitro.solver.NetworkProviders#provideRteNetworks")
    void testReactivePowerPerturbationOnRteNetworks(NetworkPair pair) {
        String baseFilename = pair.baseFilename();

        Network rknNetwork = pair.rknNetwork();
        Network nrNetwork = pair.nrNetwork();
        String test = "test_reactive_power_RTE";
        LOGGER.info(String.valueOf(nrNetwork.getShuntCompensators().iterator().next()));

        // Target reactive power injection by the shunt section in VArs
        double targetQ = 1e9;

        reactivePowerPerturbationTest(rknNetwork, nrNetwork, baseFilename, targetQ,test);
    }
}
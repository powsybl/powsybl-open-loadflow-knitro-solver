package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.iidm.modification.SetGeneratorToLocalRegulation;
import com.powsybl.iidm.modification.topology.RemoveFeederBay;
import com.powsybl.iidm.network.*;
import com.powsybl.iidm.serde.XMLExporter;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assumptions.assumeFalse;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public final class PerturbationFactory {
    private static final Logger LOGGER = LoggerFactory.getLogger(PerturbationFactory.class);
    private static final double BASE_100MVA = 100.0;

    private PerturbationFactory() {

    }

    /**
     * Finds all possible voltage perturbations of a network until a maximum number is reached.
     *
     * @param network          The network to perturb.
     * @param maxPerturbations The maximum possible number of perturbations to extract.
     * @return ALL possible voltage perturbations.
     */
    public static Stream<VoltagePerturbation> enumerateVoltagePerturbations(Network network, int maxPerturbations) {
        List<VoltagePerturbation> perturbations = new ArrayList<>();

        Terminal noGeneratorTerminal;
        Terminal regulatingTerminal;
        Terminal generatorTerminal;

        List<Generator> generators = network.getGeneratorStream()
                .toList();

        for (Generator generator : generators) {
            // Break, if maximum number of wanted perturbations is reached.
            if (perturbations.size() >= maxPerturbations) {
                break;
            }

            generatorTerminal = generator.getTerminal();
            List<Line> lowImpedanceLines = generatorTerminal
                    .getBusBreakerView()
                    .getBus()
                    .getLineStream()
                    .toList();

            for (Line lowImpedanceLine : lowImpedanceLines) {
                // Obtain the terminal at the other end of the line
                noGeneratorTerminal = getOtherEndOfLine(generatorTerminal, lowImpedanceLine);

                // Check if this bus has generators
                boolean hasGenerators = terminalHasGenerators(noGeneratorTerminal);

                if (hasGenerators) {
                    continue;
                }

                // Get the lines connected to the noGeneratorBus
                List<Line> normalLines = noGeneratorTerminal.getBusBreakerView()
                        .getBus()
                        .getLineStream()
                        .toList();

                for (Line normalLine : normalLines) {
                    regulatingTerminal = getOtherEndOfLine(noGeneratorTerminal, normalLine);

                    // Check that this line is not the same as lowImpedanceLine
                    boolean isLowImpedanceLine = generator.getTerminal()
                            .getBusBreakerView()
                            .getBus()
                            .getId()
                            .equals(regulatingTerminal.getBusBreakerView()
                                    .getBus()
                                    .getId());

                    if (isLowImpedanceLine) {
                        continue;
                    }

                    boolean hasRegulatingGenerators = terminalHasGenerators(regulatingTerminal);

                    if (hasRegulatingGenerators) {
                        perturbations.add(new VoltagePerturbation(
                                noGeneratorTerminal.getBusBreakerView().getBus().getId(),
                                regulatingTerminal.getBusBreakerView().getBus().getId(),
                                generator.getId(),
                                lowImpedanceLine.getId()
                        ));
                    }
                }
            }
        }
        assertFalse(perturbations.isEmpty(), "No possible voltage perturbations were found!");
        return perturbations.stream();
    }

    public static VoltagePerturbation getVoltagePerturbation(Network network) {
        return enumerateVoltagePerturbations(network, 1).toList().get(0);
    }

    /**
     * Creates a voltage perturbation of a given network.
     *
     * @param network      The network to perturb.
     * @param perturbation The perturbation to apply.
     * @param rPU          The per-unit resistance of the modified line.
     * @param xPU          The per-unit reactance of the modified line.
     * @param alpha        The voltage mismatch to apply on the regulating generator.
     */
    public static void applyVoltagePerturbation(Network network, VoltagePerturbation perturbation, double rPU, double xPU, double alpha) {
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

        // Acquire voltage mismatch
        double vNomGenerator = generatorBusVL.getNominalV();
        double vNomRegulator = regulatingBusVL.getNominalV();
        double vNomNoGenerator = noGeneratorBusVL.getNominalV();
        double targetV = generator.getTargetV() / vNomRegulator * vNomGenerator * alpha;

        // Acquire regulating terminal
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

        // Apply voltage mismatch and remote regulation to regulatingGenerators
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

    /**
     * Finds a possible active power perturbation.
     *
     * @param network The network to perturb.
     * @return A possible active power perturbation.
     */
    public static String getActivePowerPerturbation(Network network) {
        Optional<Bus> targetBusOp = network.getBusBreakerView()
                .getBusStream()
                .filter(bus -> bus.getLoadStream().findAny().isPresent())
                .filter(bus -> bus.getLoadStream().anyMatch(load -> load.getP0() > 0.0))
                .findAny();

        assumeFalse(targetBusOp.isEmpty(), "Network has no loads");

        Bus targetBus = targetBusOp.get();
        Optional<Load> targetLoadOp = targetBus.getLoadStream().filter(load -> load.getP0() > 0.0).findAny();

        assumeFalse(targetLoadOp.isEmpty());
        return targetLoadOp.get().getId();
    }

    /**
     * Creates an active power perturbation of a given network.
     *
     * @param network      The network to perturb.
     * @param targetLoadID The ID of the load to perturb.
     * @param alpha        The active load mismatch to apply on the target load.
     */
    public static void applyActivePowerPerturbation(Network network, String targetLoadID, double alpha) {
        double totalActiveLoad = network.getLoadStream()
                .mapToDouble(Load::getP0)
                .filter(d -> d > 0.0)
                .sum();

        Load targetLoad = network.getLoad(targetLoadID);
        targetLoad.setP0(alpha * totalActiveLoad);
    }

    /**
     * Finds all possible reactive power perturbations of a network until a maximum number is reached.
     *
     * @param network          The network to perturb.
     * @param maxPerturbations The maximum possible number of perturbations to extract.
     * @return ALL possible reactive power perturbations.
     */
    public static Stream<ReactivePowerPerturbation> enumerateReactivePowerPerturbations(Network network, int maxPerturbations) {
        List<ReactivePowerPerturbation> perturbations = new ArrayList<>();

        List<Bus> networkBuses = network.getBusBreakerView()
                .getBusStream()
                .toList();

        for (Bus bus : networkBuses) {
            // Break, if maximum number of wanted perturbations is reached.
            if (perturbations.size() >= maxPerturbations) {
                break;
            }

            boolean hasGenerators = bus.getGenerators()
                    .iterator()
                    .hasNext();
            boolean hasShunts = bus.getShuntCompensatorStream()
                    .iterator()
                    .hasNext();
            if (!hasGenerators || !hasShunts) {
                continue;
            }
            for (Generator generator : bus.getGenerators()) {
                for (ShuntCompensator shuntCompensator : bus.getShuntCompensators()) {
                    perturbations.add(new ReactivePowerPerturbation(
                            bus.getId(),
                            generator.getId(),
                            shuntCompensator.getId()
                    ));
                }
            }
        }

        assumeFalse(perturbations.isEmpty(), "No possible reactive power perturbation was found");
        return perturbations.stream();
    }

    public static ReactivePowerPerturbation getReactivePowerPerturbation(Network network) {
        return enumerateReactivePowerPerturbations(network, 1).toList().get(0);
    }

    /**
     * Creates a reactive power perturbation of a given network.
     *
     * @param network      The network to perturb.
     * @param perturbation The perturbation to apply.
     * @param targetQ      The target reactive power to inject.
     */
    public static void applyReactivePowerPerturbation(Network network, ReactivePowerPerturbation perturbation, double targetQ) {
        String targetBusID = perturbation.targetBusID;
        String targetGeneratorID = perturbation.targetGeneratorID;
        String targetShuntID = perturbation.targetShuntID;

        Bus targetBus = network.getBusBreakerView()
                .getBus(targetBusID);
        Generator targetGenerator = network.getGenerator(targetGeneratorID);

        // Make the generator control the voltage locally
        SetGeneratorToLocalRegulation generatorModification = new SetGeneratorToLocalRegulation(targetGeneratorID);
        generatorModification.apply(network);

        // Get the susceptance value corresponding to targetQ of reactive power
        double generatorVoltage = targetGenerator.getTargetV();
        double targetB = targetQ / Math.pow(generatorVoltage * 1e3, 2);

        VoltageLevel targetVL = targetBus.getVoltageLevel();

        // Removing the old shunt compensator
        RemoveFeederBay removedShuntCompensator = new RemoveFeederBay(targetShuntID);
        removedShuntCompensator.apply(network);

        // Creating the new shunt with the modified parameters
        ShuntCompensatorAdder modifiedShuntAdder = targetVL.newShuntCompensator()
                .setId(targetShuntID + "-MODIFIED")
                .setName(targetShuntID + "-MODIFIED")
                .setBus(targetBusID)
                .setSectionCount(1);

        modifiedShuntAdder.newLinearModel()
                .setMaximumSectionCount(1)
                .setBPerSection(targetB)
                .add();

        modifiedShuntAdder.add();
    }

    private static Terminal getOtherEndOfLine(Terminal terminal1, Line line) {
        Terminal terminal2 = line.getTerminal1();
        boolean isSameSide = terminal2.getBusBreakerView()
                .getBus()
                .getId()
                .equals(terminal1.getBusBreakerView()
                        .getBus()
                        .getId());
        if (isSameSide) {
            terminal2 = line.getTerminal2();
        }
        return terminal2;
    }

    private static boolean terminalHasGenerators(Terminal terminal) {
        return terminal.getBusBreakerView()
                .getBus()
                .getGenerators()
                .iterator()
                .hasNext();
    }

    public static void exportNetworkAsBusBreakerTopology(Path initPath, Path endPath) {
        Network network = Network.read(initPath).getNetwork();
        for (VoltageLevel vl : network.getVoltageLevels()) {
            vl.convertToTopology(TopologyKind.BUS_BREAKER);
        }
        Properties exportParameters = new Properties();
        exportParameters.put(XMLExporter.TOPOLOGY_LEVEL, "BUS_BREAKER");
        network.write("XIIDM", exportParameters, endPath);
    }

    public static void convertNodeBreakerDataToBusBreaker(String nodeBreakerDir, String busBreakerDir, String initFileName) {
        Path initRoot = Path.of(nodeBreakerDir);
        Path endRoot = Path.of(busBreakerDir);

        try (Stream<Path> pathStream = Files.list(initRoot)) {
            pathStream.filter(Files::isDirectory)
                    .map(subDir -> subDir.resolve(initFileName))
                    .filter(Files::exists)
                    .forEach(initFile -> {
                        try {
                            Path relativePath = initRoot.relativize(initFile);
                            Path outputFile = endRoot.resolve(relativePath);
                            Files.createDirectories(outputFile.getParent());
                            exportNetworkAsBusBreakerTopology(initFile, outputFile);
                        } catch (Exception e) {
                            throw new RuntimeException("Failed to convert the files", e);
                        }
                    });
        } catch (Exception e) {
            throw new RuntimeException("Failed to load HU real network cases", e);
        }
    }

    public record VoltagePerturbation(String noGeneratorBusID,
                                      String regulatingBusID,
                                      String generatorID,
                                      String lowImpedanceLineID) {
    }

    public record ReactivePowerPerturbation(String targetBusID,
                                            String targetGeneratorID,
                                            String targetShuntID) {
    }

}

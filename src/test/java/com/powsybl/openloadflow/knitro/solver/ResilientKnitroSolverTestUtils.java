package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.iidm.network.*;

import java.util.List;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertFalse;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public final class ResilientKnitroSolverTestUtils {

    private ResilientKnitroSolverTestUtils() {

    }

    /**
     * Creates a voltage perturbation of a given network.
     *
     * @param network      The network to perturb.
     * @param perturbation The perturbation to apply.
     * @param x            The reactance of the modified line.
     * @param targetV      The new voltage target value to apply on the regulating generator.
     */
    public static void applyVoltagePerturbation(Network network, VoltagePerturbation perturbation, double x, double targetV) {
        String noGeneratorBusID = perturbation.noGeneratorBusID();
        String regulatingBusID = perturbation.regulatingBusID();
        String generatorID = perturbation.generatorID();
        String lowImpedanceLineID = perturbation.lowImpedanceLineID();

        // Acquire components to change
        Bus regulatingBus = network.getBusBreakerView()
                .getBus(regulatingBusID);
        Generator generator = network.getGenerator(generatorID);
        Line lowImpedanceLine = network.getLine(lowImpedanceLineID);

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

        // Changing line parameters
        lowImpedanceLine.setX(x)
                .setR(0.0)
                .setG1(0.0)
                .setB1(0.0)
                .setG2(0.0)
                .setB2(0.0);
    }

    public record VoltagePerturbation(String noGeneratorBusID,
                                      String regulatingBusID,
                                      String generatorID,
                                      String lowImpedanceLineID) {
    }
}

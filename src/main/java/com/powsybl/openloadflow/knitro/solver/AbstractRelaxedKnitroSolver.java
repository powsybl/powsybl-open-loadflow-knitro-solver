/**
 * Copyright (c) 2025, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.artelys.knitro.api.*;
import com.artelys.knitro.api.callbacks.KNEvalGACallback;
import com.powsybl.commons.PowsyblException;
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.equations.*;
import com.powsybl.openloadflow.network.*;
import com.powsybl.openloadflow.util.PerUnit;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.Map;
import java.util.stream.Collectors;

import static com.powsybl.openloadflow.knitro.solver.RelaxedKnitroSolver.RelaxedKnitroProblem.incrementSolveCount;
import static com.powsybl.openloadflow.knitro.solver.RelaxedKnitroSolver.RelaxedKnitroProblem.getSolveCount;

/**
 * Abstract class for relaxed Knitro solvers, solving the open load flow equation system by minimizing constraint violations through relaxation.
 * It provides common functionality, including:
 *      - Post-processing of the computed solutions, including the reporting of relaxation variables.
 *      - An extended optimization problem formulation dedicated to solving the open load flow equation system, including relaxations.
 * This class can be extended to add custom behavior to any of these features (e.g., in {@link com.powsybl.openloadflow.knitro.solver.UseReactiveLimitsKnitroSolver}).
 * For example, if you modify the optimization problem, you may also need to update the solution-processing logic.
 *
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 */
public abstract class AbstractRelaxedKnitroSolver extends AbstractKnitroSolver {
    private static final String FEASIBLE = "feasible";
    private static final String VIOLATED = "violated";

    private static final Logger LOGGER = LoggerFactory.getLogger(AbstractRelaxedKnitroSolver.class);
    private static final String SLACK_LOG = "Slack {}[ {} ] → {}";
    private static final String CSV_EXTENSION = ".csv";
    private static final String CSV_EXTENSION_OPTI = "_optim_info.csv";

    // Variable weight
    protected double weightP1;
    protected static final double WEIGHT_Q_1 = 1.0;
    protected static final double GAMMA_FACTOR = 0.1;

    // Total number of variables (including power flow and slack variables)
    protected int numSlackVariables;

    // Number of equations for active power (P), reactive power (Q), and voltage magnitude (V)
    protected final int numPEquations;
    protected final int numQEquations;
    protected final int numVEquations;

    // Starting indices for slack variables in the variable vector
    protected final int slackPStartIndex;
    protected final int slackQStartIndex;
    protected final int slackVStartIndex;

    // Mappings from global equation indices to local indices by equation type
    protected final Map<Integer, Integer> pEquationLocalIds;
    protected final Map<Integer, Integer> qEquationLocalIds;
    protected final Map<Integer, Integer> vEquationLocalIds;

    // Mapping of the slack variable and info
    private final ArrayList<SlackVariableInfo> slackContributions = new ArrayList<>();
    // Mapping of gamma : each Voltage Level is assign to a gamma depending on its nominal voltage
    protected HashMap<Double, Double> voltageLevelGammaMap;
    protected HashMap<Integer, Double> weightVMap;

    protected AbstractRelaxedKnitroSolver(LfNetwork network, KnitroSolverParameters knitroParameters, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                          JacobianMatrix<AcVariableType, AcEquationType> j, TargetVector<AcVariableType, AcEquationType> targetVector,
                                          EquationVector<AcVariableType, AcEquationType> equationVector, boolean detailedReport) {
        super(network, knitroParameters, equationSystem, j, targetVector, equationVector, detailedReport);

        List<SingleEquation<AcVariableType, AcEquationType>> sortedEquations = equationSystem.getIndex().getSortedSingleEquationsToSolve();
        // Count number of equations by type

        this.numPEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_P).count();
        this.numQEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_Q).count();
        this.numVEquations = (int) sortedEquations.stream().filter(e -> e.getType() == AcEquationType.BUS_TARGET_V).count();

        this.numSlackVariables = 2 * (numPEquations + numQEquations + numVEquations);
        // the slack variables start after power flow variables
        this.slackPStartIndex = equationSystem.getIndex().getSortedVariablesToFind().size();
        this.slackQStartIndex = slackPStartIndex + 2 * numPEquations;
        this.slackVStartIndex = slackQStartIndex + 2 * numQEquations;

        // Map equations to local indices
        this.pEquationLocalIds = new HashMap<>();
        this.qEquationLocalIds = new HashMap<>();
        this.vEquationLocalIds = new HashMap<>();

        int pCounter = 0;
        int qCounter = 0;
        int vCounter = 0;

        // Map GAMMA values : for the different voltage level (kV) map the corresponding gamma value
        voltageLevelGammaMap = new HashMap<>();
        voltageLevelGammaMap.put(72.5, 39.55);
        voltageLevelGammaMap.put(145.0, 100.45);
        voltageLevelGammaMap.put(245.0, 212.18);
        voltageLevelGammaMap.put(420.0, 458.30);

        weightVMap = new HashMap<>();

        for (int i = 0; i < sortedEquations.size(); i++) {
            AcEquationType type = sortedEquations.get(i).getType();
            double gammaValue = 0.0;

            switch (type) {
                case BUS_TARGET_P -> pEquationLocalIds.put(i, pCounter++);
                case BUS_TARGET_Q -> qEquationLocalIds.put(i, qCounter++);
                case BUS_TARGET_V -> {
                    // Set WEIGHT_V_1 based on the nominal voltage of the bus and the corresponding gamma
                    LfBus vlInfo = network.getBus(sortedEquations.get(i).getElementNum());
                    gammaValue = getGammaValues(vlInfo);
                    if (gammaValue == 0.0 || Double.isNaN(gammaValue) || Double.isInfinite(gammaValue)) {
                        throw new PowsyblException("Gamma value is not define for bus " + vlInfo.getId() + " with nominal voltage " + vlInfo.getNominalV() + " kV. Please check the voltage level gamma mapping.");
                    }
                    weightVMap.put(vCounter, gammaValue); // for each index of V  I have the corresponding gamma
                    vEquationLocalIds.put(i, vCounter++);
                }
                default -> {
                    // Other equation types don't require slack variables
                }
            }
        }

        // Weight P
        double activeGeneration = computeActiveGeneration(network);
        double deltaP = computeDeltaP(network, activeGeneration, this.knitroParameters.getLosses());
        if (deltaP == 0 || Double.isNaN(deltaP)) {
            throw new PowsyblException("DIVIDED BY ZERO: DeltaP is equal to 0, cannot compute weightP1. Please check that the network has non-zero active power generation and load, and/or adjust the losses parameter.");
        }

        weightP1 = getWeightP1(activeGeneration, deltaP);
    }

    protected double getGammaValues(LfBus vlInfo) {
        double gamma = 0.0;
        if (vlInfo.getNominalV() <= 85.0) {
            gamma = voltageLevelGammaMap.get(72.5);
        } else if (vlInfo.getNominalV() > 85.0 && vlInfo.getNominalV() <= 200.0) {
            gamma = voltageLevelGammaMap.get(145.0);
        } else if (vlInfo.getNominalV() > 200.0 && vlInfo.getNominalV() <= 350.0) {
            gamma = voltageLevelGammaMap.get(245.0);
        } else if (vlInfo.getNominalV() > 350.0) {
            gamma = voltageLevelGammaMap.get(420.0);
        }
        return gamma;
    }

    private double getWeightP1(double activeGeneration, double deltaP) {
        weightP1 = Math.min(activeGeneration / (10 * deltaP), 1000);
        return weightP1;
    }

    @Override
    protected void processSolution(KNSolver solver, KNSolution solution, KNProblem problemInstance) {
        super.processSolution(solver, solution, problemInstance);

        List<Double> x = solution.getX();
        int outerloopIteration = incrementSolveCount();

        // ========== Slack Logging ==========
        logSlackValues("P", slackPStartIndex, numPEquations, x, outerloopIteration);
        logSlackValues("Q", slackQStartIndex, numQEquations, x, outerloopIteration);
        logSlackValues("V", slackVStartIndex, numVEquations, x, outerloopIteration);

        // ========== Penalty Computation ==========
        double penaltyP = computeSlackPenalty(x, slackPStartIndex, numPEquations, weightP1);
        double penaltyQ = computeSlackPenalty(x, slackQStartIndex, numQEquations, WEIGHT_Q_1);
        double penaltyV = computeSlackPenaltyTypeV(x, slackVStartIndex, numVEquations, weightVMap);
        double totalPenalty = penaltyP + penaltyQ + penaltyV;

        LOGGER.info("==== Slack penalty details ====");
        LOGGER.info("Penalty P = {}", penaltyP);
        LOGGER.info("Penalty Q = {}", penaltyQ);
        LOGGER.info("Penalty V = {}", penaltyV);
        LOGGER.info("Total penalty = {}", totalPenalty);

        // Weight use in the objective function
        LOGGER.info("==== Objective function weight details ====");
        LOGGER.info("Total LOSSES DC =  {} MW", this.knitroParameters.getLosses());
        LOGGER.info("Weight P1 = {}", weightP1);
        LOGGER.info("Weight Q1 = {}", WEIGHT_Q_1);
        if (LOGGER.isInfoEnabled()) {
            String gammaStr = weightVMap.entrySet().stream()
                    .collect(Collectors.groupingBy(Map.Entry::getValue, Collectors.counting()))
                    .entrySet().stream()
                    .map(e -> String.format("%d x %s", e.getValue(), e.getKey()))
                    .collect(Collectors.joining(", "));

            LOGGER.info(String.format("Gamma values : %s", gammaStr));
        }

        SlackVariableInfo[] slackArray = slackContributions.toArray(new SlackVariableInfo[0]); // Object with all the present slack information
        String csvPath = this.knitroParameters.getExportSolution();

        logSlackSummary(slackArray); // Generic summary of the network, number of slack of each type, number of load or generator violations
        if (!csvPath.equals(this.knitroParameters.DEFAULT_EXPORT_SOLUTION) && !csvPath.isEmpty()) {
            List<String> csvLines = slackInfoCsv(slackArray);
            List<String> optimInfo = optimInfoCsv(totalPenalty, penaltyP, penaltyQ, penaltyV, solution, solver);

            writeSlackInfoCsv(csvPath + CSV_EXTENSION, csvLines);
            writeOptimInfoCsv(csvPath + CSV_EXTENSION_OPTI, optimInfo);
        }

    }

    /**
     * Logs information like bus name and slack value for most significant slack variables.
     *
     * @param type The slack variable type.
     * @param startIndex The start index of slack variables associated to the given type.
     * @param count The maximum number of slack variables associated to the given type.
     * @param x The variable values as returned by solver.
     */
    protected void logSlackValues(String type, int startIndex, int count, List<Double> x, int outerloopIteration) {
        LOGGER.debug("==== Slack diagnostics for {} (p.u. and physical units) ====", type);
        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            double epsilon = sp - sm;

            boolean shouldSkip = Math.abs(epsilon) <= knitroParameters.getSlackThreshold();
            String name = null;
            StringBuilder interpretation = new StringBuilder();

            if (!shouldSkip) {
                name = getSlackVariableBusName(i, type);
                var bus = network.getBusById(name);
                int hasLoadViolation = 0;
                int hasGenViolation = 0;
                switch (type) {
                    case "P" -> {
                        SlackVariableInfo slackVar = logSlackTypeP(bus, epsilon, type, outerloopIteration);
                        LOGGER.debug(SLACK_LOG, type, name, slackVar.interpretation);
                        slackContributions.add(new SlackVariableInfo(name, epsilon, type, bus, hasLoadViolation, hasGenViolation, interpretation.toString(), outerloopIteration));
                    }

                    case "Q" -> {
                        SlackVariableInfo slackVar = logSlackTypeQ(bus, epsilon, type, outerloopIteration);
                        LOGGER.debug(SLACK_LOG, type, name, slackVar.interpretation);
                        slackContributions.add(new SlackVariableInfo(name, epsilon, type, bus, hasLoadViolation, hasGenViolation, interpretation.toString(), outerloopIteration));
                    }

                    case "V" -> {
                        SlackVariableInfo slackVar = logSlackTypeV(bus, epsilon, type, outerloopIteration);
                        LOGGER.debug(SLACK_LOG, type, name, slackVar.interpretation);
                        slackContributions.add(new SlackVariableInfo(name, epsilon, type, bus, hasLoadViolation, hasGenViolation, interpretation.toString(), outerloopIteration));
                    }
                    default -> interpretation.append("Unknown slack type");
                }
            }
        }
    }
    /**
     * Finds the bus associated to a slack variable.
     *
     * @param index The index of the slack variable
     * @param type The slack variable type.
     * @return The id of the bus associated to the slack variable.
     */

    private String getSlackVariableBusName(Integer index, String type) {
        Set<Map.Entry<Integer, Integer>> equationSet = switch (type) {
            case "P" -> pEquationLocalIds.entrySet();
            case "Q" -> qEquationLocalIds.entrySet();
            case "V" -> vEquationLocalIds.entrySet();
            default -> throw new IllegalStateException("Unexpected variable type: " + type);
        };

        Optional<Integer> varIndexOptional = equationSet.stream()
                .filter(entry -> index.equals(entry.getValue()))
                .map(Map.Entry::getKey)
                .findAny();

        int varIndex;
        if (varIndexOptional.isPresent()) {
            varIndex = varIndexOptional.get();
        } else {
            throw new PowsyblException("Variable index associated with slack variable " + type + " was not found");
        }

        LfBus bus = network.getBus(equationSystem.getIndex().getSortedSingleEquationsToSolve().get(varIndex).getElementNum());

        return bus.getId();
    }

    private class SlackVariableInfo {
        String busId;
        double slackValuepu;
        String type;
        int hasLoadViolation;
        int hasGenViolation;
        Collection<LfGenerator> generators;
        List<VoltageControl<?>> voltageControls;
        Collection<LfLoad> loads;
        Optional<LfShunt> shunts;
        Optional<TransformerVoltageControl> transformers;
        String interpretation;
        int outerloopIteration;

        public SlackVariableInfo(String busId, double slackValuepu, String type, LfBus lfBus, int hasLoadViolation, int hasGenViolation, String interpretation, int outerloopIteration) {
            this.busId = busId;
            this.slackValuepu = slackValuepu;
            this.type = type;
            this.voltageControls = lfBus.getVoltageControls();
            this.generators = lfBus.getGenerators();
            this.loads = lfBus.getLoads();
            this.hasLoadViolation = hasLoadViolation;
            this.hasGenViolation = hasGenViolation;
            this.shunts = lfBus.getShunt();
            this.transformers = lfBus.getTransformerVoltageControl();
            this.interpretation = interpretation;
            this.outerloopIteration = outerloopIteration;
        }
    }

    private SlackVariableInfo logSlackTypeV(LfBus bus, double epsilon, String type, int outerloopIteration) {
        StringBuilder interpretation = new StringBuilder();
        int hasLoadViolation = 0;
        int hasGenViolation = 0;
        Optional<VoltageControl<?>> maybeControl = bus.getVoltageControls().stream().findAny();

        if (Math.abs(epsilon) < 0.001) {
            interpretation.append(String.format("ΔV = %f p.u. (%f kV) ", epsilon, epsilon * bus.getNominalV()));
        } else {
            interpretation.append(String.format("ΔV = %.4f p.u. (%.4f kV) ", epsilon, epsilon * bus.getNominalV()));
        }
        if (maybeControl.isPresent()) {
            for (VoltageControl<?> vc : bus.getVoltageControls()) {
                interpretation.append(String.format("%n\t\tVoltage Control status is:%s of type %s located at %s," +
                        "Voltage target before slack change %.2f [p.u]", vc.getMergeStatus(), vc.getType(), vc.getControllerElements(), vc.getTargetValue()));
                interpretation.append(String.format("%n\t\tWith slack applied, voltage constraints at bus is %s ", isFeasibleV(epsilon, vc.getTargetValue()) ? FEASIBLE : VIOLATED));
            }
        }
        return new SlackVariableInfo(bus.getId(), epsilon, type, bus, hasLoadViolation, hasGenViolation, interpretation.toString(), outerloopIteration);
    }

    private SlackVariableInfo logSlackTypeQ(LfBus bus, double epsilon, String type, int outerloopIteration) {
        StringBuilder interpretation = new StringBuilder();
        Map<Double, Object> info = new HashMap<>();
        int hasLoadViolation = 0;
        int hasGenViolation = 0;

        Optional<LfGenerator> maybeGenerator = bus.getGenerators().stream().findAny();
        Optional<LfLoad> maybeLoad = bus.getLoads().stream().findAny();
        Optional<LfShunt> maybeShunt = bus.getShunt().stream().findAny();
        Optional<TransformerVoltageControl> maybeTransfo = bus.getTransformerVoltageControl().stream().findAny();

        interpretation.append(String.format("ΔQ = %.4f p.u. (%.1f MW)", epsilon, epsilon * PerUnit.SB));
        String isfeasibleQ = "";

        if (maybeGenerator.isPresent()) {
            GenInterpretation generator = buildGenInterpretation(bus.getGenerators(), interpretation, info);
            isfeasibleQ = isGenFeasible(bus.getTargetQ() + epsilon * PerUnit.SB, generator.minSum(), generator.maxSum()) ? FEASIBLE : VIOLATED;
            interpretation.append(String.format(" With slack applied, generator limits are %s  ", isfeasibleQ));
            if (isfeasibleQ.equals(VIOLATED)) {
                interpretation.append(String.format("%n\t\tChangement out of generation bus range [%.2f,%.2f]", generator.minSum(), generator.maxSum()));
                hasGenViolation = 1;
            }
        }
        if (maybeLoad.isPresent()) {
            info.put(bus.getLoadTargetQ(), "load_Q");
            interpretation.append(String.format("%n\t\tLoad : %s MW ", bus.getLoads()));
            isfeasibleQ = isLoadFeasible(epsilon * PerUnit.SB, bus.getLoadTargetQ()) ? FEASIBLE : VIOLATED;
            interpretation.append(String.format("With slack applied, Load constraints is %s ", isfeasibleQ));
            if (isfeasibleQ.equals(VIOLATED)) {
                interpretation.append(String.format("%n\t\tLoad Target Q :  %.4f MVar after slack : %f", bus.getLoadTargetQ(), bus.getLoadTargetQ() + epsilon * PerUnit.SB));
                hasLoadViolation = 1;
            }
        }
        if (maybeShunt.isPresent()) {
            interpretation.append(String.format("%n\t\tShunt susceptance: %.4f p.u. ", maybeShunt.get().getB()));
        }
        if (maybeTransfo.isPresent()) {
            interpretation.append(String.format("%n\t\tControl voltage is made by a transformer "));
        }
        if (maybeLoad.isEmpty() && maybeGenerator.isEmpty() && maybeShunt.isEmpty() && maybeTransfo.isEmpty()) {
            interpretation.append(String.format("%n\t\tNo direct connected Load, Generator, Transformer Control voltage or Shunt "));
        }
        return new SlackVariableInfo(bus.getId(), epsilon, type, bus, hasLoadViolation, hasGenViolation, interpretation.toString(), outerloopIteration);
    }

    private SlackVariableInfo logSlackTypeP(LfBus bus, double epsilon, String type, int outerloopIteration) {
        StringBuilder interpretation = new StringBuilder();
        Map<Double, Object> info = new HashMap<>();
        int hasLoadViolation = 0;
        int hasGenViolation = 0;

        Optional<LfGenerator> maybeGenerator = bus.getGenerators().stream().findAny();
        Optional<LfLoad> maybeLoad = bus.getLoads().stream().findAny();
        Optional<LfShunt> maybeShunt = bus.getShunt().stream().findAny();
        Optional<TransformerVoltageControl> maybeTransfo = bus.getTransformerVoltageControl().stream().findAny();

        interpretation.append(String.format("ΔP = %.4f p.u. (%.1f MW) ", epsilon, epsilon * PerUnit.SB));
        String isload = "";
        String feasibleP = "";

        if (maybeGenerator.isPresent()) {
            GenInterpretation generator = buildGenInterpretation(bus.getGenerators(), interpretation, info);
            feasibleP = isGenFeasible(bus.getTargetP() + epsilon * PerUnit.SB, generator.minSum(), generator.maxSum()) ? FEASIBLE : VIOLATED;
            interpretation.append(String.format("With slack applied, the resulting generator limits are %s  ", feasibleP));
            if (feasibleP.equals(VIOLATED)) {
                interpretation.append(String.format("%n\t\tChangement out of generation bus range : [%.2f; %.2f] MW ", generator.minSum(), generator.maxSum()));
                hasGenViolation = 1;
            }
        }
        if (maybeLoad.isPresent()) {
            interpretation.append(String.format("%n\t\tLoad : %s MW ", bus.getLoads()));
            isload = isLoadFeasible(bus.getLoadTargetP(), epsilon * PerUnit.SB) ? FEASIBLE : VIOLATED;
            info.put(bus.getLoadTargetP(), "load_P");
            interpretation.append(String.format("Load Target P : %.4f MW. With slack applied, Load constraints are %s ", bus.getLoadTargetP(), isload));
            if (isload.equals(VIOLATED)) {
                interpretation.append(String.format("%n\t\tLoad status after changement : %.4f MW ", bus.getLoadTargetP() + epsilon * PerUnit.SB));
                hasLoadViolation = 1;
            }
        }
        if (maybeShunt.isPresent()) {
            interpretation.append(String.format("%n\t\tShunt susceptance : %.4f p.u. ", maybeShunt.get().getB()));
        }
        if (maybeTransfo.isPresent()) {
            interpretation.append(String.format("%n\t\tControl voltage is made by a transformer "));
        }
        if (maybeLoad.isEmpty() && maybeGenerator.isEmpty() && maybeShunt.isEmpty() && maybeTransfo.isEmpty()) {
            interpretation.append(String.format("%n\t\tNo direct connected Load, Generator, Transformer Control voltage or Shunt "));
        }
        return new SlackVariableInfo(bus.getId(), epsilon, type, bus, hasLoadViolation, hasGenViolation, interpretation.toString(), outerloopIteration);
    }

    private record GenInterpretation(double minSum, double maxSum, String interpretation) {

    }

    private static GenInterpretation buildGenInterpretation(List<LfGenerator> generators, StringBuilder interpretation, Map<Double, Object> info) {
        double minSum = 0.0;
        double maxSum = 0.0;

        for (LfGenerator gen : generators) {
            minSum += gen.getMinQ();
            maxSum += gen.getMaxQ();

            info.put(gen.getMinQ(), "gen_minQ");
            info.put(gen.getMaxQ(), "gen_maxQ");
            info.put(gen.getTargetQ(), "gen_Q");
            info.put(PerUnit.SB, "pu_base");

            interpretation.append(String.format("%n\t\tGenerator %s of range: [%.2f,%.2f] ",
                    gen.getId(), gen.getMinQ(), gen.getMaxQ()
            ));
        }
        return new GenInterpretation(minSum, maxSum, interpretation.toString());
    }

    private static boolean isGenFeasible(double newdata, double min, double max) {
        return newdata >= min && newdata <= max;
    }

    private static boolean isLoadFeasible(double slack, double loadTarget) {
        return slack + loadTarget >= 0;
    }

    private static boolean isFeasibleV(double slack, double vRef) {
        double vNewref = slack + vRef;
        return vNewref >= 0.8 && vNewref <= 1.2; //0.8 et 1.2 V
    }

    private void logSlackSummary(SlackVariableInfo[] slackArray) {
        Map<Integer, List<SlackVariableInfo>> groupedByIteration = Arrays.stream(slackArray)
                .collect(Collectors.groupingBy(si -> si.outerloopIteration));
        List<SlackVariableInfo> currentIterationSlacks = groupedByIteration.getOrDefault(getSolveCount(), List.of());

        if (!currentIterationSlacks.isEmpty()) {
            LOGGER.info("==== Perturbation general impact  ====");
            LOGGER.info("Total number of Slack = {}", currentIterationSlacks.size());
            long affectedBus = currentIterationSlacks.stream()
                    .map(si -> si.busId)
                    .distinct().count();
            int loadViolationCount = (int) currentIterationSlacks.stream()
                    .filter(si -> si.hasLoadViolation == 1)
                    .count();
            int genViolationCount = (int) currentIterationSlacks.stream()
                    .filter(si -> si.hasGenViolation == 1)
                    .count();

            LOGGER.info("Total number of bus affected = {}", affectedBus);
            LOGGER.info("Total number of load violation = {}", loadViolationCount);
            LOGGER.info("Total number of generator violation = {}", genViolationCount);

            int busCount = network.getBuses().size();
            if (busCount > 0) {
                double percentAffected = 100.0 * affectedBus / busCount;
                LOGGER.info("Percentage of affected bus = {} %", Math.round(percentAffected * 100.0) / 100.0);
            } else {
                LOGGER.info("No buses in the network were found.");
            }
        }
    }

    private List<String> slackInfoCsv(SlackVariableInfo[] slackArray) {
        List<String> csvLines = new ArrayList<>();
        csvLines.add("busId;type;slackValue_pu;gen;controlevoltage;transfo;shunt;load;load_violation;gen_violation;outerLoopIteration");
        for (SlackVariableInfo si : slackArray) {
            String gens = (si.generators != null && !si.generators.isEmpty()) ? si.generators.stream().map(Object::toString).collect(Collectors.joining(";")) : "";
            String controlers = (si.voltageControls != null && !si.voltageControls.isEmpty()) ? si.voltageControls.stream().map(Object::toString).collect(Collectors.joining(";")) : "";
            int hasLoadViolation = si.hasLoadViolation == 1 ? 1 : 0;
            int hasGenViolation = si.hasGenViolation == 1 ? 1 : 0;
            String transfo = si.transformers.stream().map(Object::toString).collect(Collectors.joining(";"));
            String shunt = si.shunts.stream().map(Object::toString).collect(Collectors.joining(";"));
            String loads = si.loads.stream().map(Object::toString).collect(Collectors.joining(";"));
            int outerloopIteration = si.outerloopIteration;
            csvLines.add(String.format("%s;%s;%.6f;%s;%s;%s;%s;%s;%s;%s;%d",
                    si.busId, si.type, si.slackValuepu,
                    gens, controlers, transfo, shunt, loads, hasLoadViolation, hasGenViolation, outerloopIteration));
        }
        return csvLines;
    }

    private List<String> optimInfoCsv(double totalPenalty, double penaltyP, double penaltyQ, double penaltyV, KNSolution solution, KNSolver solver) {
        List<String> optimInfo = new ArrayList<>();
        optimInfo.add("total_penalty;penaltyP;penaltyQ;penaltyV;status;iterations");
        try {
            optimInfo.add(String.format("%s;%s;%s;%s;%s;%s", totalPenalty, penaltyP, penaltyQ, penaltyV, solution.getStatus(), solver.getNumberIters()));
        } catch (KNException e) {
            throw new PowsyblException("Failed to gather optimization info.", e);
        }
        return optimInfo;
    }

    private void writeSlackInfoCsv(String filename, List<String> lines) {
        try {
            java.nio.file.Files.write(
                    java.nio.file.Paths.get(filename),
                    lines,
                    java.nio.charset.StandardCharsets.UTF_8,
                    java.nio.file.StandardOpenOption.CREATE,
                    java.nio.file.StandardOpenOption.TRUNCATE_EXISTING
            );
            LOGGER.info("Slack informations and contributions exported to {}", filename);
        } catch (java.io.IOException e) {
            LOGGER.warn("Failed to write slack CSV to {}", filename);
        }
    }

    private void writeOptimInfoCsv(String filename, List<String> lines) {
        try {
            java.nio.file.Files.write(
                    java.nio.file.Paths.get(filename),
                    lines,
                    java.nio.charset.StandardCharsets.UTF_8,
                    java.nio.file.StandardOpenOption.CREATE,
                    java.nio.file.StandardOpenOption.TRUNCATE_EXISTING
            );
            LOGGER.info("Optimization information exported to {}", filename);
        } catch (java.io.IOException e) {
            LOGGER.warn("Failed to write optimization info CSV: {}", e.getMessage());
        }
    }

    /**
     * Calculates the total loss associated to a slack variable type
     *
     * @param x          The variable values as returned by solver.
     * @param startIndex The start index of slack variables associated to the given type.
     * @param count      The maximum number of slack variables associated to the given type.
     * @param weight1    The weight in front of the given slack variables terms (L1)
     * @return The total penalty associated to the slack variables type.
     */
    double computeSlackPenalty(List<Double> x, int startIndex, int count, double weight1) {
        double penalty = 0.0;
        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            penalty += weight1 * (sp + sm); // Linear terms
        }
        return penalty;
    }

    /**
     * Calculates the total loss associated to a slack variable of type V
     *
     * @param x          The variable values as returned by solver.
     * @param startIndex The start index of slack variables associated to the given type.
     * @param count      The maximum number of slack variables associated to the given type.
     * @param weight    The weight in front of the given slack variables terms : omegaV depending on the voltage level of the bus
     * @return The total penalty associated to the slack variables type.
     */
    double computeSlackPenaltyTypeV(List<Double> x, int startIndex, int count, HashMap<Integer, Double> weight) {
        double penalty = 0.0;
        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            penalty += GAMMA_FACTOR * weight.get(i) * (sp + sm); // Linear terms
        }
        return penalty;
    }

    /**
     * Calculates Delta P = |Pgen - Pload - Losses|
     *
     * @param network           LfNetwork
     * @param activeGeneration  The total active power generation in the network
     * @param losses            The approximated losses computed by a DC LoadF
     * @return Delta P
     */
    private double computeDeltaP(LfNetwork network, double activeGeneration, double losses) {
        double activeLoad = 0.0;

        for (LfBus b : network.getBuses()) {
            activeLoad += b.getLoadTargetP() * 100.0;
        }
        return Math.abs(activeGeneration - activeLoad - losses); //minus total Losses
    }

    /**
     * Calculates Active Generation
     *
     * @param network           LfNetwork
     * @return The total active power generation in the network
     */
    private double computeActiveGeneration(LfNetwork network) {
        double activeGeneration = 0.0;

        for (LfBus b : network.getBuses()) {
            activeGeneration += b.getGenerationTargetP() * 100.0;
        }
        return activeGeneration;
    }

    /**
     * Optimization problem-solving the open load flow equation system by minimizing constraint violations through relaxation.
     */
    public abstract class AbstractRelaxedKnitroProblem extends AbstractKnitroProblem {

        /**
         * Relaxed Knitro problem definition including:
         * - initialization of variables (types, bounds, initial state)
         * - definition of linear constraints
         * - definition of non-linear constraints, evaluated in extended the callback function
         * - definition of the extended Jacobian matrix passed to Knitro to solve the problem
         * - definition of the objective function to be minimized (equation system violations)
         */
        protected AbstractRelaxedKnitroProblem(LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                               TargetVector<AcVariableType, AcEquationType> targetVector, JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                               KnitroSolverParameters knitroParameters, int numAdditionalVariables, int numAdditionalConstraints) {
            super(network, equationSystem, targetVector, jacobianMatrix, knitroParameters, numAdditionalVariables, numAdditionalConstraints);
        }

        void addObjectiveFunction(int numPEquations, int slackPStartIndex, int numQEquations, int slackQStartIndex,
                                  int numVEquations, int slackVStartIndex) throws KNException {
            // initialise lists to track linear objective function terms of the form: a * x
            List<Integer> linIndexes = new ArrayList<>(); // list of indexes of the variable x
            List<Double> linCoefs = new ArrayList<>(); // list of indexes of the coefficient a

            // add slack penalty terms, for each slack type, linear terms from the absolute value
            addSlackObjectiveTerms(numPEquations, slackPStartIndex, weightP1, linIndexes, linCoefs);
            addSlackObjectiveTerms(numQEquations, slackQStartIndex, AbstractRelaxedKnitroSolver.WEIGHT_Q_1, linIndexes, linCoefs);
            addSlackObjectiveTermTypeV(numVEquations, slackVStartIndex, weightVMap, linIndexes, linCoefs);

            setObjectiveLinearPart(linIndexes, linCoefs);
        }

        /**
         * Adds quadratic and linear terms related to slack variables of type P and Q  to the objective function.
         */
        void addSlackObjectiveTerms(int numEquations, int slackStartIdx, double weight1,
                                    List<Integer> linIndexes, List<Double> linCoefs) {
            for (int i = 0; i < numEquations; i++) {
                int idxSm = slackStartIdx + 2 * i; // negative slack variable index
                int idxSp = slackStartIdx + 2 * i + 1; // positive slack variable index

                // Add linear terms: weight1 * (sp + sm)

                // add first linear term : weight1 * sp
                linIndexes.add(idxSp);
                linCoefs.add(weight1);

                // add second linear term : weight1 * sm
                linIndexes.add(idxSm);
                linCoefs.add(weight1);
            }
        }

        /**
         * Adds quadratic and linear terms related to slack variables of type V to the objective function.
         */
        void addSlackObjectiveTermTypeV(int numEquations, int slackStartIdx, HashMap<Integer, Double> weight,
                                        List<Integer> linIndexes, List<Double> linCoefs) {

            for (int i = 0; i < numEquations; i++) {
                int idxSm = slackStartIdx + 2 * i; // negative slack variable index
                int idxSp = slackStartIdx + 2 * i + 1; // positive slack variable index

                // Add linear terms: weight * (sp + sm)

                // add first linear term : weight * sp
                linIndexes.add(idxSp);
                linCoefs.add(weight.get(i) * GAMMA_FACTOR);

                // add second linear term : weight  * sm
                linIndexes.add(idxSm);
                linCoefs.add(weight.get(i) * GAMMA_FACTOR);
            }
        }

        @Override
        protected void initializeCustomizedVariables(List<Double> lowerBounds, List<Double> upperBounds,
                                                     List<Double> initialValues) {
            // set a lower bound to slack variables (>= 0)
            // initial values have already been set to 0
            for (int i = numberOfPowerFlowVariables; i < numTotalVariables; i++) {
                lowerBounds.set(i, 0.0);
            }
        }

        @Override
        protected void addAdditionalConstraintVariables(int equationId, AcEquationType equationType,
                                                        List<Integer> varIndices, List<Double> coefficients) {
            // Add slack variables if applicable
            int slackBase = getSlackIndexBase(equationType, equationId);
            if (slackBase >= 0) {
                varIndices.add(slackBase);       // Sm
                varIndices.add(slackBase + 1);   // Sp
                coefficients.add(1.0);
                coefficients.add(-1.0);
            }
        }

        @Override
        protected void addAdditionalJacobianVariables(int constraintIndex,
                                                      SingleEquation<AcVariableType, AcEquationType> equation,
                                                      List<Integer> variableIndices) {
            AcEquationType equationType = equation.getType();
            // get slack variable local index (within its equation type)
            int slackStart = switch (equationType) {
                case BUS_TARGET_P -> pEquationLocalIds.getOrDefault(constraintIndex, -1);
                case BUS_TARGET_Q -> qEquationLocalIds.getOrDefault(constraintIndex, -1);
                case BUS_TARGET_V -> vEquationLocalIds.getOrDefault(constraintIndex, -1);
                default -> -1;
            };

            if (slackStart >= 0) {
                // get slack variable type starting index (within total variables' indexes)
                int slackBaseIndex = switch (equationType) {
                    case BUS_TARGET_P -> slackPStartIndex;
                    case BUS_TARGET_Q -> slackQStartIndex;
                    case BUS_TARGET_V -> slackVStartIndex;
                    default -> throw new IllegalStateException("Unexpected constraint type: " + equationType);
                };
                // get slack variables Sm and Sp indexes
                variableIndices.add(slackBaseIndex + 2 * slackStart);     // Sm
                variableIndices.add(slackBaseIndex + 2 * slackStart + 1); // Sp
            }
        }

        /**
         * Returns the base index of the slack variable associated with a given equation type and ID.
         *
         * @param equationType Type of the equation (P, Q, or V).
         * @param equationId   Index of the equation.
         * @return Base index of the corresponding slack variable, or -1 if not applicable.
         */
        protected int getSlackIndexBase(AcEquationType equationType, int equationId) {
            return switch (equationType) {
                case BUS_TARGET_P -> pEquationLocalIds.getOrDefault(equationId, -1) >= 0
                        ? slackPStartIndex + 2 * pEquationLocalIds.get(equationId) : -1;
                case BUS_TARGET_Q -> qEquationLocalIds.getOrDefault(equationId, -1) >= 0
                        ? slackQStartIndex + 2 * qEquationLocalIds.get(equationId) : -1;
                case BUS_TARGET_V -> vEquationLocalIds.getOrDefault(equationId, -1) >= 0
                        ? slackVStartIndex + 2 * vEquationLocalIds.get(equationId) : -1;
                default -> -1;
            };
        }

        @Override
        protected KNEvalGACallback createGradientCallback(JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                                          List<Integer> listNonZerosCtsDense, List<Integer> listNonZerosVarsDense,
                                                          List<Integer> listNonZerosCtsSparse, List<Integer> listNonZerosVarsSparse) {

            return new AbstractRelaxedKnitroProblem.RelaxedCallbackEvalG(jacobianMatrix, listNonZerosCtsDense, listNonZerosVarsDense,
                    listNonZerosCtsSparse, listNonZerosVarsSparse, network,
                    equationSystem, knitroParameters, numberOfPowerFlowVariables);
        }

        /**
         * Callback used by Knitro to evaluate the non-linear parts of the objective and constraint functions.
         */
        public static class RelaxedCallbackEvalFC extends KnitroCallbacks.BaseCallbackEvalFC {

            private final AbstractRelaxedKnitroProblem problemInstance;

            RelaxedCallbackEvalFC(AbstractRelaxedKnitroProblem problemInstance,
                                  List<SingleEquation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                                  List<Integer> nonLinearConstraintIds) {
                super(sortedEquationsToSolve, nonLinearConstraintIds);
                this.problemInstance = problemInstance;
            }

            @Override
            protected double addModificationOfNonLinearConstraints(int equationId, AcEquationType equationType,
                                                                   List<Double> x) {
                int slackIndexBase = problemInstance.getSlackIndexBase(equationType, equationId);
                if (slackIndexBase >= 0) {
                    double sm = x.get(slackIndexBase);        // negative slack
                    double sp = x.get(slackIndexBase + 1);    // positive slack
                    return sp - sm;              // add slack contribution
                }
                return 0;
            }
        }

        /**
         * Callback used by Knitro to evaluate the gradient (Jacobian matrix) of the constraints.
         * Only constraints (no objective) are handled here.
         */
        public static class RelaxedCallbackEvalG extends KnitroCallbacks.BaseCallbackEvalG {

            RelaxedCallbackEvalG(JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                 List<Integer> denseConstraintIndices, List<Integer> denseVariableIndices,
                                 List<Integer> sparseConstraintIndices, List<Integer> sparseVariableIndices,
                                 LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                 KnitroSolverParameters knitroParameters, int numLFVariables) {

                super(jacobianMatrix, denseConstraintIndices, denseVariableIndices, sparseConstraintIndices, sparseVariableIndices,
                        network, equationSystem, knitroParameters, numLFVariables);
            }

            @Override
            protected double computeModifiedJacobianValue(int variableIndex) {
                if (((variableIndex - numLFVariables) & 1) == 0) {
                    // set Jacobian entry to -1.0 if slack variable is Sm
                    return -1.0;
                } else {
                    // set Jacobian entry to +1.0 if slack variable is Sp
                    return 1.0;
                }
            }
        }
    }
}

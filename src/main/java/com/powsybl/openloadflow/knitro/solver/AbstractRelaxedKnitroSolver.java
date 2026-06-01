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

        this.numSlackVariables = 2 * (numPEquations + numQEquations + numVEquations); //declaration of all the slacks variabel each type and *2 for positive and negative slack variables

        // the slack variables start after power flow variables
        this.slackPStartIndex = equationSystem.getIndex().getSortedVariablesToFind().size(); // P
        this.slackQStartIndex = slackPStartIndex + 2 * numPEquations; // Q
        this.slackVStartIndex = slackQStartIndex + 2 * numQEquations; // V

        // Map equations to local indices
        this.pEquationLocalIds = new HashMap<>();
        this.qEquationLocalIds = new HashMap<>();
        this.vEquationLocalIds = new HashMap<>();

        int pCounter = 0;
        int qCounter = 0;
        int vCounter = 0;

        // MAP GAMMA : VL in kV and the corresponding gamma
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

        // Weight P
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

        // ========== Slack Logging ==========
        logSlackValues("P", slackPStartIndex, numPEquations, x);
        logSlackValues("Q", slackQStartIndex, numQEquations, x);
        logSlackValues("V", slackVStartIndex, numVEquations, x);

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

        SlackVariableInfo[] slackArray = slackContributions.toArray(new SlackVariableInfo[0]);
        logSlackSummary(slackArray);

        String csv = this.knitroParameters.getExportSolution().orElse(null);

        if (csv != null && !csv.isEmpty()) {
            List<String> csvLines = slackInfoCsv(slackArray);
            List<String> optimInfo = optimInfoCsv(totalPenalty, penaltyP, penaltyQ, penaltyV, solution, solver);

            writeSlackInfoCsv(csv + ".csv", csvLines);
            writeOptimInfoCsv(csv + "_optim_info.csv", optimInfo);
        }

        // Weight use in the objective function
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
    }

    /**
     * Logs information like bus name and slack value for most significant slack variables.
     *
     * @param type The slack variable type.
     * @param startIndex The start index of slack variables associated to the given type.
     * @param count The maximum number of slack variables associated to the given type.
     * @param x The variable values as returned by solver.
     */
    protected void logSlackValues(String type, int startIndex, int count, List<Double> x) {
        LOGGER.debug("==== Slack diagnostics for {} (p.u. and physical units) ====", type);
        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            double epsilon = sp - sm;

            boolean shouldSkip = Math.abs(epsilon) <= knitroParameters.getSlackThreshold();
            String name = null;
            StringBuilder interpretation = new StringBuilder();
            Map<Double, Object> info = new HashMap<>();
            double slackvalue = 0.0;
            if (!shouldSkip) {
                name = getSlackVariableBusName(i, type);
                var bus = network.getBusById(name);
                int loadViolation = 0;
                int genViolation = 0;
                Optional<LfGenerator> maybeGenerator = bus.getGenerators().stream().findAny();
                Optional<LfLoad> maybeLoad = bus.getLoads().stream().findAny();
                Optional<LfShunt> maybeShunt = bus.getShunt().stream().findAny();
                Optional<TransformerVoltageControl> maybeTransfo = bus.getTransformerVoltageControl().stream().findAny();
                switch (type) {
                    case "P" -> {
                        interpretation.append(String.format("ΔP = %.4f p.u. (%.1f MW)", epsilon, epsilon * PerUnit.SB));
                        String isload = "";
                        String feasibleP = "";
                        slackvalue = epsilon * PerUnit.SB;

                        if (maybeGenerator.isPresent()) {
                            List<LfGenerator> generator = bus.getGenerators();
                            double[] genMaxP = new double[bus.getGenerators().size()];
                            double[] genMinP = new double[bus.getGenerators().size()];
                            for (LfGenerator gen : generator) {
                                genMaxP = new double[]{gen.getMaxP()};
                                genMinP = new double[]{gen.getMinP()};
                                info.put(gen.getMinP(), "gen_minP");
                                info.put(gen.getMaxP(), "gen_maxP");
                                info.put(gen.getTargetP(), "gen_P");
                                info.put(PerUnit.SB, "pu_base");
                                interpretation.append(String.format("%n                                                           Generator %s of range: [%.2f,%.2f] ", gen.getId(), gen.getMinP(), gen.getMaxP()));
                            }
                            feasibleP = isGenFeasible(bus.getTargetP() + epsilon * PerUnit.SB, Arrays.stream(genMinP).sum(), Arrays.stream(genMaxP).sum()) ? FEASIBLE : VIOLATED;
                            interpretation.append(String.format("If slack applied, generator limits are %s  ", feasibleP));
                            if (feasibleP.equals(VIOLATED)) {
                                interpretation.append(String.format("%n                                                           Changement out of generation bus range : [%.2f; %.2f] MW", Arrays.stream(genMinP).sum(), Arrays.stream(genMaxP).sum()));
                                genViolation = 1;
                            }
                        }
                        if (maybeLoad.isPresent()) {
                            interpretation.append(String.format("%n                                                           Load : %s MW", bus.getLoads()));
                            isload = isLoadFeasible(bus.getLoadTargetP(), epsilon * PerUnit.SB) ? FEASIBLE : VIOLATED;
                            info.put(bus.getLoadTargetP(), "load_P");
                            interpretation.append(String.format(" Load Target P : %.4f MW. If slack applied, Load constraints is %s", bus.getLoadTargetP(), isload));
                            if (isload.equals(VIOLATED)) {
                                interpretation.append(String.format("%n                                                           Load status after changement : %.4f MW", bus.getLoadTargetP() + epsilon * PerUnit.SB));
                                loadViolation = 1;
                            }
                        }
                        if (maybeShunt.isPresent()) {
                            interpretation.append(String.format("%n                                                           Shunt susceptance : %.4f p.u.", maybeShunt.get().getB()));
                        }
                        if (maybeTransfo.isPresent()) {
                            interpretation.append(String.format("%n                                                           Control voltage is made by a transformer "));
                        }
                        if (maybeLoad.isEmpty() && maybeGenerator.isEmpty() && maybeShunt.isEmpty() && maybeTransfo.isEmpty()) {
                            interpretation.append(String.format("%n                                                           No direct connected Load, Generator, Transformer Control voltage or Shunt"));
                        }
                    }
                    case "Q" -> {
                        double loadQ = 0.0;
                        String isfeasibleQ = "";
                        String isfeasibleloadQ = "";
                        interpretation.append(String.format("ΔQ = %.4f p.u. (%.1f MVAr)", epsilon, epsilon * PerUnit.SB));
                        slackvalue = epsilon * PerUnit.SB;
                        if (maybeGenerator.isPresent()) {
                            List<LfGenerator> generator = bus.getGenerators();
                            double[] genMaxQ = new double[bus.getGenerators().size()];
                            double[] genMinQ = new double[bus.getGenerators().size()];
                            for (LfGenerator gen : generator) {
                                genMaxQ = new double[]{gen.getMaxQ()};
                                genMinQ = new double[]{gen.getMinQ()};
                                info.put(gen.getMinQ(), "gen_minQ");
                                info.put(gen.getMaxQ(), "gen_maxQ");
                                info.put(gen.getTargetQ(), "gen_Q");
                                info.put(PerUnit.SB, "pu_base");
                                interpretation.append(String.format("%n                                                           Generator %s of range: [%.2f,%.2f] ", gen.getId(), gen.getMinQ(), gen.getMaxQ()));
                            }
                            isfeasibleQ = isGenFeasible(bus.getTargetQ() + epsilon * PerUnit.SB, Arrays.stream(genMinQ).sum(), Arrays.stream(genMaxQ).sum()) ? FEASIBLE : VIOLATED;
                            interpretation.append(String.format(" If slack applied, generator limits are %s  ", isfeasibleQ));
                            if (isfeasibleQ.equals(VIOLATED)) {
                                interpretation.append(String.format("%n                                                           Changement out of generation bus range [%.2f,%.2f]", Arrays.stream(genMinQ).sum(), Arrays.stream(genMaxQ).sum()));
                                genViolation = 1;
                            }
                        }
                        if (maybeLoad.isPresent()) {
                            loadQ = bus.getLoadTargetQ(); //[MVar]
                            info.put(loadQ, "load_Q");
                            interpretation.append(String.format("%n                                                           Load : %s MW", bus.getLoads()));
                            isfeasibleloadQ = isLoadFeasible(epsilon * PerUnit.SB, bus.getLoadTargetQ()) ? FEASIBLE : VIOLATED;
                            interpretation.append(String.format("If slack applied, Load constraints is %s ", isfeasibleloadQ));
                            if (isfeasibleloadQ.equals(VIOLATED)) {
                                interpretation.append(String.format("%n                                                           Load Target Q :  %.4f MVar after slack : %f", bus.getLoadTargetQ(), bus.getLoadTargetQ() + epsilon * PerUnit.SB));
                                loadViolation = 1;
                            }

                        }
                        if (maybeShunt.isPresent()) {
                            interpretation.append(String.format("%n                                                           Shunt susceptance: %.4f p.u.", maybeShunt.get().getB()));
                        }
                        if (maybeTransfo.isPresent()) {
                            interpretation.append(String.format("%n                                                           Control voltage is made by a transformer "));
                        }
                        if (maybeLoad.isEmpty() && maybeGenerator.isEmpty() && maybeShunt.isEmpty() && maybeTransfo.isEmpty()) {
                            interpretation.append(String.format("%n                                                           No direct connected Load, Generator, Transformer Control voltage or Shunt"));
                        }
                    }
                    case "V" -> {
                        Optional<VoltageControl<?>> maybeControl = bus.getVoltageControls().stream().findAny();
                        if (bus == null) {
                            LOGGER.warn("Bus {} not found while logging V slack.", name);
                            shouldSkip = true;
                        } else {
                            info.put(bus.getV(), "bus_V");
                            info.put(bus.getNominalV(), "bus_nominalV");
                            slackvalue = epsilon * bus.getNominalV();
                            if (Math.abs(epsilon) < 0.001) {
                                interpretation.append(String.format("ΔV = %f p.u. (%f kV) ", epsilon, epsilon * bus.getNominalV()));
                            } else {
                                interpretation.append(String.format("ΔV = %.4f p.u. (%.4f kV) ", epsilon, epsilon * bus.getNominalV()));
                            }
                            if (maybeControl.isPresent()) {
                                List<VoltageControl<?>> controls = bus.getVoltageControls();
                                for (VoltageControl vc : controls) {
                                    interpretation.append(String.format("%n                                                           Voltage Control status is:%s of type %s located at %s," +
                                            "%n                                                           Voltage target before slack change %.2f [p.u]", vc.getMergeStatus(), vc.getType(), vc.getControllerElements(), vc.getTargetValue()));
                                    interpretation.append(String.format("%n                                                           If slack applied, voltage constraints at bus is %s ", isFeasibleV(epsilon, bus.getNominalV(), vc.getTargetValue()) ? FEASIBLE: VIOLATED));
                                }
                            }

                        }
                    }
                    default -> interpretation.append("Unknown slack type");
                }
                slackContributions.add(new SlackVariableInfo(name, epsilon, slackvalue, type, bus, loadViolation, genViolation, info));
            }

            if (shouldSkip) {
                continue;
            }
            String msg = String.format("Slack %s[ %s ] → %s", type, name, interpretation);
            LOGGER.debug(msg);
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
        double slackValue;
        double slackValuepu;
        String type;
        int loadViolation;
        int genViolation;
        Map<Double, Object> info = new HashMap<>();
        Collection<LfBranch> lines;
        Collection<LfGenerator> generators;
        List<VoltageControl<?>> voltageControls;
        Collection<LfLoad> loads;
        Optional<LfShunt> shunts;
        Optional<TransformerVoltageControl> transformers;

        public SlackVariableInfo(String busId, double slackValuepu, double slackValue, String type, LfBus lfBus, int loadViolation, int genViolation, Map<Double, Object>info) {
            this.busId = busId;
            this.slackValue = slackValue;
            this.slackValuepu = slackValuepu;
            this.type = type;
            this.voltageControls = lfBus.getVoltageControls();
            this.generators = lfBus.getGenerators();
            this.lines = lfBus.getBranches();
            this.loads = lfBus.getLoads();
            this.loadViolation = loadViolation;
            this.genViolation = genViolation;
            this.shunts = lfBus.getShunt();
            this.transformers = lfBus.getTransformerVoltageControl();
            if (info != null) {
                this.info.putAll(info);
            }
        }
    }

    /**
     * Checks if a new value is feasible according to given bounds.
     * @param newdata
     * @param min
     * @param max
     * @return
     */

    private static boolean isGenFeasible(double newdata, double min, double max) {
        return newdata >= min && newdata <= max;
    }

    private static boolean isLoadFeasible(double slack, double loadTarget) {
        return slack + loadTarget >= 0;
    }

    private static boolean isFeasibleV(double slack, double vNominal, double vRef) {
        double vNewref = slack * vNominal + vRef;
        return vNewref / vNominal >= 0.8 && vNewref / vNominal <= 1.2; //0.8 et 1.2 V
    }

    private void logSlackSummary(SlackVariableInfo[] slackArray) {
        LOGGER.info("==== Perturbation general impact  ====");
        LOGGER.info("Total number of Slack  = {}", slackArray.length);
        long affectedBus = Arrays.stream(slackArray)
                .map(si -> si.busId)
                .distinct().count();
        int loadViolations = (int) Arrays.stream(slackArray)
                .filter(si -> si.loadViolation == 1)
                .count();
        int genViolations = (int) Arrays.stream(slackArray)
                .filter(si -> si.genViolation == 1)
                .count();
        LOGGER.info("Total number of bus affected = {}", affectedBus);
        LOGGER.info("Total number of load violation {}", loadViolations);
        LOGGER.info("Total number of generator violation {}", genViolations);
        LOGGER.info("Percentage of affected bus = {} %", String.format("%.2f", 100.0 * affectedBus / network.getBuses().size()));
    }

    private List<String> slackInfoCsv(SlackVariableInfo[] slackArray) {
        List<String> csvLines = new ArrayList<>();
        csvLines.add("busId;type;slackValue_pu;slackValue;gen;controlevoltage;transfo;shunt;load;load_violation;gen_violation");
        for (SlackVariableInfo si : slackArray) {
            String gens = si.generators == null ? "" : si.generators.stream().map(Object::toString).collect(Collectors.joining(";"));
            String controlers = si.voltageControls == null ? "" : si.voltageControls.stream().map(Object::toString).collect(Collectors.joining(";"));
            int loadViolation = si.loadViolation == 1 ? 1 : 0;
            int genViolation = si.genViolation == 1 ? 1 : 0;
            String transfo = si.transformers == null ? "" : si.transformers.stream().map(Object::toString).collect(Collectors.joining(";"));
            String shunt = si.shunts == null ? "" : si.shunts.stream().map(Object::toString).collect(Collectors.joining(";"));
            String loads = si.loads == null ? "" : si.loads.stream().map(Object::toString).collect(Collectors.joining(";"));
            csvLines.add(String.format("%s;%s;%.6f;%.6f;%s;%s;%s;%s;%s;%s;%s",
                    si.busId, si.type, si.slackValuepu, si.slackValue,
                    gens, controlers, transfo, shunt, loads, loadViolation, genViolation));
        }
        return csvLines;
    }


    private List<String> optimInfoCsv(double totalPenalty, double penaltyP, double penaltyQ, double penaltyV, KNSolution solution, KNSolver solver) {
        List<String> optimInfo = new ArrayList<>();
        optimInfo.add("total_penalty;penaltyP;penaltyQ;penaltyV;status;iterations");
        try {
            optimInfo.add(String.format("%s;%s;%s;%s;%s;%s", totalPenalty, penaltyP, penaltyQ, penaltyV, solution.getStatus(), solver.getNumberIters()));
        } catch (KNException e) {
//            throw new KNException("Failed to gather optimization info.", e);
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
            LOGGER.info("Optimization info exported to {}", filename);
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

            // add slack penalty terms, for each slack type, of the form: (Sp - Sm)^2 = Sp^2 + Sm^2 - 2*Sp*Sm + linear terms from the absolute value
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

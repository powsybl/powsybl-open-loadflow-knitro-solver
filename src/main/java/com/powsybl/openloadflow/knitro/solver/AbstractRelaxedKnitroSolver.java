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
import com.powsybl.iidm.network.Battery;
import com.powsybl.iidm.network.IdentifiableType;
import com.powsybl.iidm.network.Terminal;
import com.powsybl.iidm.network.components.ConnectedComponent;
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.equations.*;
import com.powsybl.openloadflow.network.*;
import com.powsybl.openloadflow.network.action.LfTerminalsConnectionAction;
import com.powsybl.openloadflow.network.impl.LfBatteryImpl;
import com.powsybl.openloadflow.util.PerUnit;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import javax.xml.transform.Transformer;
import java.util.*;
import java.util.stream.Stream;

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

    private static final Logger LOGGER = LoggerFactory.getLogger(AbstractRelaxedKnitroSolver.class);

    // Penalty weights in the objective function
    protected static final double WEIGHT_P_PENAL = 1.0;
    protected static final double WEIGHT_Q_PENAL = 1.0;
    protected static final double WEIGHT_V_PENAL = 1.0;

    // Weights of the linear in the objective function
    protected static final double WEIGHT_ABSOLUTE_PENAL = 3.0;

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

    // Mapping of the slack variabel and info
    private final ArrayList<SlackVariableInfo> slackContributions = new ArrayList<>();


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
        this.slackPStartIndex = equationSystem.getIndex().getSortedVariablesToFind().size(); //d'abord P
        this.slackQStartIndex = slackPStartIndex + 2 * numPEquations; // ensuite Q
        this.slackVStartIndex = slackQStartIndex + 2 * numQEquations;// ensuite V

        // Map equations to local indices
        this.pEquationLocalIds = new HashMap<>();
        this.qEquationLocalIds = new HashMap<>();
        this.vEquationLocalIds = new HashMap<>();

        int pCounter = 0;
        int qCounter = 0;
        int vCounter = 0;

        for (int i = 0; i < sortedEquations.size(); i++) {
            AcEquationType type = sortedEquations.get(i).getType();

            switch (type) {
                case BUS_TARGET_P -> pEquationLocalIds.put(i, pCounter++);
                case BUS_TARGET_Q -> qEquationLocalIds.put(i, qCounter++);
                case BUS_TARGET_V -> vEquationLocalIds.put(i, vCounter++);
                default -> {
                    // Other equation types don't require slack variables
                }
            }
        }
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
        double penaltyP = computeSlackPenalty(x, slackPStartIndex, numPEquations, WEIGHT_P_PENAL);
        double penaltyQ = computeSlackPenalty(x, slackQStartIndex, numQEquations, WEIGHT_Q_PENAL);
        double penaltyV = computeSlackPenalty(x, slackVStartIndex, numVEquations, WEIGHT_V_PENAL);
        double totalPenalty = penaltyP + penaltyQ + penaltyV;

        LOGGER.info("==== Slack penalty details ====");
        LOGGER.info("Penalty P = {}", penaltyP);
        LOGGER.info("Penalty Q = {}", penaltyQ);
        LOGGER.info("Penalty V = {}", penaltyV);
        LOGGER.info("Total penalty = {}", totalPenalty);


//        try (CSVWriter writer = new CSVWriter(new FileWriter("slack.csv"))) {
//            writer.writeNext(new String[]{"busId","type","slackValue","gens","controllers","lines","info"});
//
//            for (SlackVariableInfo si : slackArray) {
//                SlackVariableInfo svi = (SlackVariableInfo) si;
//
//                writer.writeNext(new String[]{
//                        si.getBusId(),
//                        si.getType(),
//                        String.valueOf(si.getSlackValue()),
//                        svi.getGenerators().toString(),
//                        svi.getVoltageControls().toString(),
//                        svi.getLines().toString(),
//                        si.getInfo().toString()
//                });
//            }
//        }
        Optional<String> filepath = this.knitroParameters.getExportSolution();
        String csv = filepath.orElse(null);
       // System.out.println("filepath = " + csv);;
        if (!filepath.isEmpty()) {

          //  System.out.println("rentre dans if : filepath = " + csv);
            SlackVariableInfo[] slackArray = slackContributions.toArray(new SlackVariableInfo[0]);

            String[] slackLines = slackContributions.stream()
                    .map(si -> String.format("%s: %s, %.4f,%s,%s,%s,%s,%s,%s,%s, %s", si.busId, si.type, si.slackValue, si.generators, si.voltageControls,si.transformers, si.shunts,si.loads, si.load_to_gen,si.gen_violation, si.info))
                    .toArray(String[]::new);

            LOGGER.info("==== Perturbation general impact  ====");
            LOGGER.info("Total number of Slack  = {}", slackArray.length);
            long affectedBus = Arrays.stream(slackArray)
                   // .filter(si -> Math.abs(si.slackValue) > knitroParameters.getSlackThreshold())
                    .map(si -> si.busId)
                    .distinct().count();
            int loadViolations = (int) Arrays.stream(slackArray)
                    .filter(si -> si.load_to_gen == 1)
                    .count();
            int genViolations = (int) Arrays.stream(slackArray)
                    .filter(si -> si.gen_violation == 1)
                    .count();
            LOGGER.info("Total number of bus affected = {}", affectedBus);
            LOGGER.info("Total number of load violation {}",loadViolations);
            LOGGER.info("Total number of generator violation {}",genViolations);
            LOGGER.info("Percentage of affected bus = {} %", 100.0 * affectedBus / network.getBuses().size());

//        for (String line : slackLines) {
//            LOGGER.info(line);
//        }

            List<String> csvLines = new ArrayList<>();
            csvLines.add("busId/type/slackValue_pu/slackValue/gen/controlevoltage/lines/transfo/shunt/load/load_violation/gen_violation/info");

            for (SlackVariableInfo si : slackArray) {
                StringBuilder sb = new StringBuilder();
                for (Map.Entry<Double, Object> e : si.info.entrySet()) {
                    if (sb.length() > 0) {
                        sb.append(",");
                    }
                    sb.append(e.getKey()).append(":").append(e.getValue());
                }
                String infoStr = sb.toString().replaceAll(",", ";"); // éviter les virgules dans les champs
                String gens = si.generators == null ? "" : si.generators.stream().map(Object::toString).collect(java.util.stream.Collectors.joining(";"));
                String controlers = si.voltageControls == null ? "" : si.voltageControls.stream().map(Object::toString).collect(java.util.stream.Collectors.joining(";"));
                String lines = si.lines == null ? "" : si.lines.stream().map(Object::toString).collect(java.util.stream.Collectors.joining(";"));
                int load_to_gen = si.load_to_gen == 1 ? 1 : 0;
                int gen_violation = si.gen_violation == 1 ?1: 0;
                String transfo = si.transformers == null ? "" : si.transformers.stream().map(Object::toString).collect(java.util.stream.Collectors.joining(";"));
                String shunt = si.shunts == null ? "" : si.shunts.stream().map(Object::toString).collect(java.util.stream.Collectors.joining(";"));
                String loads = si.loads == null ? "" : si.loads.stream().map(Object::toString).collect(java.util.stream.Collectors.joining(";"));
                csvLines.add(String.format("[%s]/[%s]/[%.6f]/[%.6f]/[%s]/[%s]/[%s]/[%s]/[%s]/[%s]/[%s]/[%s]/[%s]", si.busId, si.type, si.slackValue_pu, si.slackValue, gens, controlers, lines,transfo, shunt,loads, load_to_gen,gen_violation, infoStr));
            }


            try {
                java.nio.file.Files.write(
                        java.nio.file.Paths.get(csv+".csv"),
                        csvLines,
                        java.nio.charset.StandardCharsets.UTF_8,
                        java.nio.file.StandardOpenOption.CREATE,
                        java.nio.file.StandardOpenOption.TRUNCATE_EXISTING
                );
                LOGGER.info("Slack contributions exported to {}", csv+".csv");
            } catch (java.io.IOException e) {
                LOGGER.warn("Failed to write slack CSV: {}", e.getMessage());
            }
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

            // Get significant slack values above threshold
            boolean shouldSkip = Math.abs(epsilon) <= knitroParameters.getSlackThreshold();
            String name = null;
            String interpretation = null;
      //     String busId = null;
            Map<Double, Object> info = new HashMap<>();
            double slackvalue = 0.0;
            if (!shouldSkip) {
                name = getSlackVariableBusName(i, type);
                var bus = network.getBusById(name);
               // var geometry = bus.GeographicalTag();
                int load_to_gen = 0;
                int gen_violation =0;
                Optional<LfGenerator> maybeGenerator = bus.getGenerators().stream().findAny();
                Optional<LfLoad> maybeLoadP = bus.getLoads().stream().findAny();
                Optional<LfShunt> maybeShunt = bus.getShunt().stream().findAny();
                Optional<TransformerVoltageControl>  maybeTransfo = bus.getTransformerVoltageControl().stream().findAny();
                switch (type) {
                    case "P" ->{
                        interpretation = String.format("ΔP = %.4f p.u. (%.1f MW)", epsilon, epsilon * PerUnit.SB);
//                        double gen_P = 0.0;
//                       list gen_MaxP = null;
//                        double gen_MinP =0.0;
                        double loadP =0.0;
//                        double isgen =0.0;
                        String isload="";
                        String feasibleP = "";
                        IdentifiableType genType = null;
                        slackvalue = epsilon*PerUnit.SB;


                        // Optional<Transformer> maybeTransfo2 = bus.getTransformers().stream().findAny();
                    //    Stream<Battery> maybeBatteryP = bus.getArea();
                        //Stream<LfBattery> maybeBattery = bus.getBatteries().stream();
                        if (maybeGenerator.isPresent()) {
                            List<LfGenerator> generator =  bus.getGenerators();
                            double[] gen_MaxP = new double[bus.getGenerators().size()];
                            double[] gen_MinP = new double[bus.getGenerators().size()];
                            for(LfGenerator gen : generator) {
                              //  if (gen.getGeneratorControlType() == LfGenerator.GeneratorControlType.VOLTAGE) {
                                    gen_MaxP = new double[]{gen.getMaxP()}; // verifier si c'est la valeur max du generateur
                                    gen_MinP = new double[]{gen.getMinP()};
                                  //  info.put(gen.getId(), "gen_id");
                                    info.put(gen.getMinP(), "gen_minP");
                                    info.put(gen.getMaxP(), "gen_maxP");
                                    info.put(gen.getTargetP(), "gen_P");
                                    info.put(PerUnit.SB, "pu_base");
                                    interpretation += String.format(" generator %s, range: [%.2f,%.2f] ", gen.getId(), gen.getMinP(), gen.getMaxP());
                               // }
                            }

                            feasibleP = IsGenfeasible(bus.getTargetP() + epsilon * PerUnit.SB, Arrays.stream(gen_MinP).min().orElse(0.0), Arrays.stream(gen_MaxP).sum()) ? "feasible" : "violated";
                            interpretation += String.format(" after slack, generator limits: %s  ", feasibleP);
                            if (feasibleP == "violated") {
                                interpretation += String.format("Changement out of generation bus range : [%.2f; %2.f] MW", Arrays.stream(gen_MinP).min().orElse(0.0), Arrays.stream(gen_MaxP).sum());
                                gen_violation = 1;
                            }
//                            if (generator.stream().filter()getGeneratorControlType() == LfGenerator.GeneratorControlType.VOLTAGE) {
////                                gen_P = generator.getTargetP();
////                                gen_MaxP = generator.getMaxP(); // verifier si c'est la valeur max du generateur
////                                isgen =1;
////                                gen_MinP = generator.getMinP();
//                                feasibleP = IsGenfeasible(generator.getTargetP()+epsilon*PerUnit.SB, generator.getMinP(), generator.getMaxP()) ? "feasible" : "violated";
//                                info.put(generator.getMinP(), "gen_minP");
//                                 info.put(generator.getMaxP(), "gen_maxP");
//                                 info.put(generator.getTargetP(), "gen_P");
//                                info.put(PerUnit.SB,"pu_base");
//                                interpretation += String.format(" after slack, generator limits: %s  ", feasibleP);
//                            }
                        }
                        if (maybeLoadP.isPresent()){
                                loadP = bus.getLoadTargetP();
                                isload =IsLoadfeasible(bus.getLoadTargetP(),epsilon*PerUnit.SB)?"feasible":"violated";
                                info.put(bus.getLoadTargetP(), "load_P");
                                interpretation += String.format(" Load Target P : %.4f MW,  after slack, Load constraints: %s ",bus.getLoadTargetP(), isload);
                                if (isload=="violated"){
                                    interpretation += String.format("  after slack changement : %f", bus.getLoadTargetP()+epsilon*PerUnit.SB);
                                    load_to_gen=1;
                                }
                        }
                        if (maybeShunt.isPresent()){
                            interpretation+= String.format(" shunt susceptance: %.4f p.u.", maybeShunt.get().getB());
                        }
                        if (maybeTransfo.isPresent()){
                            interpretation+= String.format(" control voltage by transformer ");
                        }
                        if(maybeLoadP.isEmpty() && maybeGenerator.isEmpty() && maybeShunt.isEmpty() && maybeTransfo.isEmpty()){
                            interpretation += String.format(" No direct connected Load, Generator, Transformer Control voltage or Shunt" );
                        }}
                    case "Q" -> {
                        double genQ = 0.0;
                    //    double gen_MaxQ = 0.0;
                      //  double gen_MinQ = 0.0;
                        double loadQ = 0.0;
                        double Q_limits = 0.0;
                        double isgen = 0.0;
                        double isload = 0.0;
                        String isfeasibleQ = "";
                        String isfeasibleloadQ="";
                    //    Optional<LfGenerator> maybeGenerator = bus.getGenerators().stream().findAny();
                        Optional<LfLoad> maybeLoadQ = bus.getLoads().stream().findAny();
                        interpretation = String.format("ΔQ = %.4f p.u. (%.1f MVAr)", epsilon, epsilon * PerUnit.SB);
                        slackvalue = epsilon*PerUnit.SB;
                        if (maybeGenerator.isPresent()) {
                            List<LfGenerator> generator =  bus.getGenerators();
                            double[] gen_MaxQ = new double[bus.getGenerators().size()];
                            double[] gen_MinQ = new double[bus.getGenerators().size()];
                            for(LfGenerator gen : generator) {
                                //  if (gen.getGeneratorControlType() == LfGenerator.GeneratorControlType.VOLTAGE) {
                                gen_MaxQ = new double[]{gen.getMaxQ()}; // verifier si c'est la valeur max du generateur
                                gen_MinQ = new double[]{gen.getMinQ()};
                                //  info.put(gen.getId(), "gen_id");
                                info.put(gen.getMinQ(), "gen_minQ");
                                info.put(gen.getMaxQ(), "gen_maxQ");
                                info.put(gen.getTargetQ(), "gen_Q");
                                info.put(PerUnit.SB, "pu_base");
                                interpretation += String.format(" generator %s, range: [%.2f,%.2f] ", gen.getId(), gen.getMinQ(), gen.getMaxQ());
                                // }
                            }
                            isfeasibleQ = IsGenfeasible(bus.getTargetQ() + epsilon * PerUnit.SB, Arrays.stream(gen_MinQ).min().orElse(0.0), Arrays.stream(gen_MaxQ).sum()) ? "feasible" : "violated";
                            interpretation += String.format(" after slack, generator limits: %s  ", isfeasibleQ);
                            if (isfeasibleQ=="violated"){
                                    interpretation += String.format("Changement out of generator range : [%.2f; %2.f] Mvar", Arrays.stream(gen_MinQ).min().orElse(0.0), Arrays.stream(gen_MaxQ).sum());
                                    gen_violation =1;
                                }
                        }
                        if (maybeLoadP.isPresent()){
                            {
                                loadQ = bus.getLoadTargetQ();//[MVar]
                                info.put(loadQ, "load_Q");
                                isload = 1;;
                                isfeasibleloadQ = IsLoadfeasible(epsilon*PerUnit.SB, bus.getLoadTargetQ())? "feasible" :"violated"; ;
                                //      loadmax = bus.getLoads();
                                interpretation+= String.format(" after slack, Load constraints: %s ", isfeasibleloadQ);
                                if (isfeasibleloadQ=="violated"){
                                    interpretation += String.format("Load Target Q :  %.4f MVar after slack : %f", bus.getLoadTargetQ(), bus.getLoadTargetQ()+epsilon*PerUnit.SB);
                                    load_to_gen =1;
                                }
                            }

                        }
                        if (maybeShunt.isPresent()){
                            interpretation+= String.format(" shunt susceptance: %.4f p.u.", maybeShunt.get().getB());
                        }
                        if (maybeTransfo.isPresent()){
                            interpretation+= String.format(" control voltage by transformer ");
                        }
                        if(maybeLoadP.isEmpty() && maybeGenerator.isEmpty() && maybeShunt.isEmpty() && maybeTransfo.isEmpty()){
                            interpretation += String.format(" No direct connected Load, Generator, Transformer Control voltage or Shunt" );
                        }
                    }
                    case "V" -> {


                        //  List<VoltageControl> controls = bus.getVoltageControls().stream();
                        Optional<VoltageControl<?>> maybeControl = bus.getVoltageControls().stream().findAny();//filter(vc -> vc.getControlledBus().getId().equals(bus.getId()));;
//                        Component connectedComponent = bus.getTerminals().stream().map(Terminal::getConnectedComponent).filter(Object::nonNull).findAny().orElse(null);
                        if (bus == null) {
                            LOGGER.warn("Bus {} not found while logging V slack.", name);
                            shouldSkip = true;
                        } else {
                           // interpretation = String.format("ΔV = %f p.u. (%f kV)", epsilon, epsilon * bus.getNominalV());
                            info.put(bus.getV(), "bus_V");
                            info.put(bus.getNominalV(),"bus_nominalV");
                            slackvalue = epsilon*bus.getNominalV();
                            if (Math.abs(epsilon) < 0.001){
                                interpretation = String.format("ΔV = %f p.u. (%f kV) ", epsilon, epsilon * bus.getNominalV());
                            }
                                else {
                                interpretation = String.format("ΔV = %.4f p.u. (%.4f kV) ", epsilon, epsilon * bus.getNominalV());
                            }
                            if(maybeControl.isPresent()){
                                List<VoltageControl<?>> controls = bus.getVoltageControls();
                                for (VoltageControl vc : controls) {
//                                    if(vc.getMergeStatus()==MAIN){
                                        interpretation += String.format(" %n            Voltage control status is:%s of type %s located at %s, voltage target before slack change %.2f [p.u]", vc.getMergeStatus(), vc.getType(), vc.getControllerElements(), vc.getTargetValue());
                                        interpretation+= String.format(" ,after slack, voltage constraints at bus: %s ", IsfeasibleV(epsilon,bus.getNominalV(),vc.getTargetValue()) ? "feasible" : "violated");

//                                    }
//                                     else {
//                                        interpretation += String.format(" %n Voltage control status is : %s of type: %s , located at %s, with target before slacke change %.4f [kV] ", vc.getMergeStatus(), vc.getType(),vc.getControllerElements(), vc.getTargetValue());
//                                    }

//                                    System.out.println("Type: " + vc.getType());
//                                    System.out.println("Controller elements: " + vc.getControllerElements());
//                                    System.out.println("Status: " + vc.getMergeStatus());
                                }


                                //  Object voltageControlTargetV = bus.getVoltageControls().stream().filter(vc -> vc.getControlledBus().getId().equals(bus.getId()));;
                             //   interpretation += String.format(", voltage controlled by %s  at bus %s target %.4f", bus.getGeneratorVoltageControl().get().getType(), bus.getVoltageControls().getClass(),maybeControl.get().getTargetValue());
                                //info.put(controls.getFirst(), "controller");

                                //interpretation += String.format(", voltage controlled by a %s located at ", bus.getVoltageControls().stream());

                            }

                        }
                    }
                    default -> interpretation = "Unknown slack type";
                }
                slackContributions.add(new SlackVariableInfo(name, epsilon, slackvalue , type, bus,load_to_gen,info));
            }

            if (shouldSkip) {
                continue;
            }

            String msg = String.format("Slack %s[ %s ] → %s ", type, name, interpretation);
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
        double slackValue_pu;
       // String interpretation;
       // double epsilon;
        String type;
        int load_to_gen;
        int gen_violation;
        //Optional<VoltageControl<?>> VoltageControl;
        Map<Double, Object> info = new HashMap<>();
        Stream<Terminal> connectedTerminals;
        ConnectedComponent component;
        Collection<LfBranch> lines;
        //Collections<LfBranch> branches;
        Collection<LfGenerator> generators;
      //  List<VoltageController> controllers = new ArrayList<>();
        List<VoltageControl<?>> voltageControls;
        Collection<LfLoad> loads;
        Optional<LfShunt> shunts;
        Optional<TransformerVoltageControl> transformers;
        public SlackVariableInfo(String busId, double slackValue_pu, double slackValue, String type, LfBus lfBus,int load_to_gen, Map<Double, Object>info){// Map<Double, Object>info) {
            this.busId = busId;
            this.slackValue = slackValue;
            this.slackValue_pu = slackValue_pu;
            this.type = type;
            this.voltageControls = lfBus.getVoltageControls();
            this.generators = lfBus.getGenerators();
            this.lines = lfBus.getBranches();
            this.loads = lfBus.getLoads();
            this.load_to_gen = load_to_gen;
            this.gen_violation =gen_violation;
            this.shunts = lfBus.getShunt();
            this.transformers = lfBus.getTransformerVoltageControl();
          //  findVoltageControllers(lfBus);
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
    private static boolean IsGenfeasible(double newdata, double min, double max){
        return newdata>= min && newdata <= max;
        }

    private static boolean IsLoadfeasible(double slack, double loadTarget){
        return slack+loadTarget>=0;
    }
    private static boolean IsfeasibleV(double slack, double Vnominal, double Vref){
        double Vnewref = slack*Vnominal + Vref;
        return Vnewref/Vnominal>= 0.8 && Vnewref/Vnominal <= 1.2; //0.8 et 1.2
    }

    /**
     * Calculates the total loss associated to a slack variable type
     *
     * @param x          The variable values as returned by solver.
     * @param startIndex The start index of slack variables associated to the given type.
     * @param count      The maximum number of slack variables associated to the given type.
     * @param weight     The weight inf front of the given slack variables terms
     * @return The total penalty associated to the slack variables type.
     */
    double computeSlackPenalty(List<Double> x, int startIndex, int count, double weight) {
        double penalty = 0.0;
        for (int i = 0; i < count; i++) {
            double sm = x.get(startIndex + 2 * i);
            double sp = x.get(startIndex + 2 * i + 1);
            double diff = sp - sm;
            penalty += weight * (diff * diff); // Quadratic terms
            penalty += weight * WEIGHT_ABSOLUTE_PENAL * (sp + sm); // Linear terms
        }
        return penalty;
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
            // initialise lists to track quadratic objective function terms of the form: a * x1 * x2
            List<Integer> quadRows = new ArrayList<>(); // list of indexes of the first variable x1
            List<Integer> quadCols = new ArrayList<>(); // list of indexes of the second variable x2
            List<Double> quadCoefs = new ArrayList<>(); // list of indexes of the coefficient a

            // initialise lists to track linear objective function terms of the form: a * x
            List<Integer> linIndexes = new ArrayList<>(); // list of indexes of the variable x
            List<Double> linCoefs = new ArrayList<>(); // list of indexes of the coefficient a

            // add slack penalty terms, for each slack type, of the form: (Sp - Sm)^2 = Sp^2 + Sm^2 - 2*Sp*Sm + linear terms from the absolute value
            addSlackObjectiveTerms(numPEquations, slackPStartIndex, AbstractRelaxedKnitroSolver.WEIGHT_P_PENAL, AbstractRelaxedKnitroSolver.WEIGHT_ABSOLUTE_PENAL, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);
            addSlackObjectiveTerms(numQEquations, slackQStartIndex, AbstractRelaxedKnitroSolver.WEIGHT_Q_PENAL, AbstractRelaxedKnitroSolver.WEIGHT_ABSOLUTE_PENAL, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);
            addSlackObjectiveTerms(numVEquations, slackVStartIndex, AbstractRelaxedKnitroSolver.WEIGHT_V_PENAL, AbstractRelaxedKnitroSolver.WEIGHT_ABSOLUTE_PENAL, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);

            setObjectiveQuadraticPart(quadRows, quadCols, quadCoefs);
            setObjectiveLinearPart(linIndexes, linCoefs);
        }

        /**
         * Adds quadratic and linear terms related to slack variables to the objective function.
         */
        void addSlackObjectiveTerms(int numEquations, int slackStartIdx, double weight, double lambda,
                                    List<Integer> quadRows, List<Integer> quadCols, List<Double> quadCoefs,
                                    List<Integer> linIndexes, List<Double> linCoefs) {
            for (int i = 0; i < numEquations; i++) {
                int idxSm = slackStartIdx + 2 * i; // negative slack variable index
                int idxSp = slackStartIdx + 2 * i + 1; // positive slack variable index

                // Add quadratic terms: weight * (sp^2 + sm^2 - 2 * sp * sm)

                // add first quadratic term : weight * sp^2
                quadRows.add(idxSp);
                quadCols.add(idxSp);
                quadCoefs.add(weight);

                // add second quadratic term : weight * sm^2
                quadRows.add(idxSm);
                quadCols.add(idxSm);
                quadCoefs.add(weight);

                // add third quadratic term : weight * (- 2 * sp * sm)
                quadRows.add(idxSp);
                quadCols.add(idxSm);
                quadCoefs.add(-2 * weight);

                // Add linear terms: weight * lambda * (sp + sm)

                // add first linear term : weight * lambda * sp
                linIndexes.add(idxSp);
                linCoefs.add(lambda * weight);

                // add second linear term : weight * lambda * sm
                linIndexes.add(idxSm);
                linCoefs.add(lambda * weight);
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

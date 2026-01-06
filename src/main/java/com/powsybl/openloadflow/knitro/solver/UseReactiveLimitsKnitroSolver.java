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
import com.powsybl.math.matrix.SparseMatrix;
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.ac.solver.AcSolverUtil;
import com.powsybl.openloadflow.equations.*;
import com.powsybl.openloadflow.network.*;
import com.powsybl.openloadflow.network.util.VoltageInitializer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static com.google.common.primitives.Doubles.toArray;
import static com.powsybl.openloadflow.ac.equations.AcEquationType.*;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Yoann Anezin {@literal <yoann.anezin at artelys.com>}
 */
public class UseReactiveLimitsKnitroSolver extends AbstractRelaxedKnitroSolver {

    private static final Logger LOGGER = LoggerFactory.getLogger(UseReactiveLimitsKnitroSolver.class);

    // Number of variables including slack variables
    private final int numLFandSlackVariables;

    // Number of equations for active power (P), reactive power (Q), and voltage magnitude (V)
    private final int complConstVariables;

    // Starting indices for slack variables in the variable vector
    private final int compVarStartIndex;

    // Mappings from global equation indices to local indices by equation type
    private final Map<Integer, Integer> elemNumControlledControllerBus;
    private static final Map<Integer, Equation<AcVariableType, AcEquationType>> INDEQUNACTIVEQ = new LinkedHashMap<>();

    // Unactivated Equations on reactiv power to deal with
    private final List<Equation<AcVariableType, AcEquationType>> equationsQBusV;
    private final List<Integer> listElementNumWithQEqUnactivated;
    private final Map<Integer, Integer> vSuppEquationLocalIds;
    protected KnitroSolverParameters knitroParameters;

    public UseReactiveLimitsKnitroSolver(
            LfNetwork network,
            KnitroSolverParameters knitroParameters,
            EquationSystem<AcVariableType, AcEquationType> equationSystem,
            JacobianMatrix<AcVariableType, AcEquationType> jacobian,
            TargetVector<AcVariableType, AcEquationType> targetVector,
            EquationVector<AcVariableType, AcEquationType> equationVector,
            boolean detailedReport) {
        super(network, knitroParameters, equationSystem, jacobian, targetVector, equationVector, detailedReport);
        this.knitroParameters = knitroParameters;

        this.numLFVariables = equationSystem.getIndex().getSortedVariablesToFind().size();

        List<Equation<AcVariableType, AcEquationType>> sortedEquations = equationSystem.getIndex().getSortedEquationsToSolve();

        // Count number of classic LF equations by type
        this.numLFandSlackVariables = numberOfVariables;
        this.compVarStartIndex = slackVStartIndex + 2 * numVEquations;

        this.elemNumControlledControllerBus = new LinkedHashMap<>();

        // The new equation system implemented here duplicates and modifies V equations
        // It also adds two equations on Q on each PV bus
        // First we need to collect those Q equations

        // At first, we isolate buses with V equation
        List<Equation<AcVariableType, AcEquationType>> activeEquationsV = sortedEquations.stream()
                .filter(e -> e.getType() == AcEquationType.BUS_TARGET_V).toList();
        List<Integer> listBusesWithVEq = activeEquationsV.stream()
                .map(e -> e.getTerms().get(0).getElementNum()).toList();

        List<Equation<AcVariableType, AcEquationType>> equationsQToAdd = new ArrayList<>();

        // Collect the Q equation associated
        List<Integer> listBusesWithQEqToAdd = new ArrayList<>();

        // For each bus with a V equation
        for (int elementNum : listBusesWithVEq) {
            LfBus controlledBus = network.getBuses().get(elementNum);   // Take the controller bus

            // Look at the bus controlling voltage and take its Q equation
            LfBus controllerBus = controlledBus.getGeneratorVoltageControl().get().getControllerElements().get(0);
            List<Equation<AcVariableType, AcEquationType>> listEqControllerBus = equationSystem.getEquations(ElementType.BUS, controllerBus.getNum());
            Equation<AcVariableType, AcEquationType> equationQToAdd = listEqControllerBus.stream()
                    .filter(e -> e.getType() == BUS_TARGET_Q).toList().get(0);

            // We are taking into account only buses with limits on reactive power
            if (!(controllerBus.getMaxQ() >= 1.7976931348623156E30 || controllerBus.getMinQ() <= -1.7976931348623156E30)) {
                equationsQToAdd.add(equationQToAdd);
                elemNumControlledControllerBus.put(controllerBus.getNum(), controlledBus.getNum());  // link between controller and controlled bus
                listBusesWithQEqToAdd.add(controllerBus.getNum());
            }

        }

        // 3 new variables are used on both V equations modified, and 2 on the Q equations listed above
        this.complConstVariables = equationsQToAdd.size() * 5;

        this.numberOfVariables = numLFandSlackVariables + complConstVariables;
        this.equationsQBusV = Stream.concat(equationsQToAdd.stream(),
                equationsQToAdd.stream()).toList(); //Duplication to get b_low and b_up eq
        this.listElementNumWithQEqUnactivated = listBusesWithQEqToAdd;

        // Map equations to local indices
        this.vSuppEquationLocalIds = new HashMap<>();

        int vSuppCounter = 0;

        for (int i = 0; i < sortedEquations.size(); i++) {
            Equation<AcVariableType, AcEquationType> equation = sortedEquations.get(i);
            AcEquationType type = equation.getType();
            switch (type) {
                case BUS_TARGET_P:
                case BUS_TARGET_Q:
                    break;
                case BUS_TARGET_V:
                    // In case there is a Vsup equation
                    LfBus controlledBus = network.getBuses().get(equation.getElement(network).get().getNum());
                    LfBus controllerBus = controlledBus.getGeneratorVoltageControl().get().getControllerElements().get(0);
                    if (listElementNumWithQEqUnactivated.contains(controllerBus.getNum())) {
                        vSuppEquationLocalIds.put(i, vSuppCounter++);
                    }
                    break;
            }
        }
    }

    /**
     * Returns the name of the solver.
     */
    @Override
    public String getName() {
        return "Knitro Reactive Limits Solver";
    }

    @Override
    protected KNProblem createKnitroProblem(VoltageInitializer voltageInitializer) {
        try {
            return new UseReactiveLimitsKnitroProblem(network, equationSystem, targetVector, j, voltageInitializer, knitroParameters);
        } catch (KNException e) {
            throw new PowsyblException("Failed to create relaxed Knitro problem", e);
        }
    }

    @Override
    protected void processSolution(KNSolver solver, KNSolution solution, KNProblem problemInstance) {
        super.processSolution(solver, solution, problemInstance);

        List<Double> x = solution.getX();

        LOGGER.info("=== Switches Done===");
        logSwitches(x, compVarStartIndex, complConstVariables / 5);
    }

    /**
     * Inform all switches PV -> PQ done in the solution found
     * @param x             current network's estate
     * @param startIndex    first index of complementarity constraints variables in x
     * @param count         number of b_low / b_up different variables
     */
    private void logSwitches(List<Double> x, int startIndex, int count) {
        for (int i = 0; i < count; i++) {
            double vInf = x.get(startIndex + 5 * i);
            double vSup = x.get(startIndex + 5 * i + 1);
            double bLow = x.get(startIndex + 5 * i + 2);
            double bUp = x.get(startIndex + 5 * i + 3);
            String bus = equationSystem.getIndex()
                    .getSortedEquationsToSolve().stream().filter(e ->
                            e.getType() == BUS_TARGET_V).toList().get(i).getElement(network).get().getId();
            if (Math.abs(bLow) < 1E-3 && !(vInf < 1E-4 && vSup < 1E-4)) {
                LOGGER.info("Switch PV -> PQ on bus {}, Q set at Qmin", bus);

            } else if (Math.abs(bUp) < 1E-3 && !(vInf < 1E-4 && vSup < 1E-4)) {
                LOGGER.info("Switch PV -> PQ on bus {}, Q set at Qmax", bus);

            }
        }
    }

    private final class UseReactiveLimitsKnitroProblem extends AbstractRelaxedKnitroProblem {

        private List<Equation<AcVariableType, AcEquationType>> completeEquationsToSolve;

        /**
         * Knitro problem definition including:
         * - Initialization of variables (types, bounds, initial state)
         * - Definition of linear and non-linear constraints
         * - Objective function setup
         * - Jacobian matrix setup for Knitro
         */
        private UseReactiveLimitsKnitroProblem(
                LfNetwork network,
                EquationSystem<AcVariableType, AcEquationType> equationSystem,
                TargetVector<AcVariableType, AcEquationType> targetVector,
                JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                VoltageInitializer voltageInitializer,
                KnitroSolverParameters parameters) throws KNException {
            // =============== Variable Initialization ===============
            super(network, equationSystem,
                    targetVector, jacobianMatrix, parameters, numberOfVariables, equationSystem.getIndex().getSortedEquationsToSolve().size() +
                            3 * complConstVariables / 5, numLFandSlackVariables, voltageInitializer);

            LOGGER.info("Defining {} variables", numberOfVariables);

            // Set up the constraints of the optimization problem
            setupConstraints();

            // Initialize variables
            initializeVariables(voltageInitializer, numberOfVariables);
            LOGGER.info("Initialization of variables : type of initialization {}", voltageInitializer);

            // =============== Callbacks and Jacobian ===============
            setObjEvalCallback(new UseReactiveLimitsCallbackEvalFC(this, completeEquationsToSolve, nonlinearConstraintIndexes));

            setJacobianMatrix(completeEquationsToSolve, nonlinearConstraintIndexes);

            // TODO : uncomment me
//            AbstractMap.SimpleEntry<List<Integer>, List<Integer>> hessNnz = getHessNnzRowsAndCols(nonlinearConstraintIndexes);
//            setHessNnzPattern(hessNnz.getKey(), hessNnz.getValue());

            // set the objective function of the optimization problem
            addObjectiveFunction(numPEquations, slackPStartIndex, numQEquations, slackQStartIndex, numVEquations, slackVStartIndex);
        }

        @Override
        protected void setupConstraints() throws KNException {
            List<Equation<AcVariableType, AcEquationType>> activeConstraints = equationSystem.getIndex().getSortedEquationsToSolve();

            // Linear and nonlinear constraints (the latter are deferred to callback)
            NonLinearExternalSolverUtils solverUtils = new NonLinearExternalSolverUtils();

            List<Integer> nonlinearConstraintIndexes = new ArrayList<>();                                                   // contains the indexes of all non-linear constraints
            completeEquationsToSolve = new ArrayList<>(activeConstraints);   // Contains all equations of the final system to be solved
            List<Double> wholeTargetVector = new ArrayList<>(Arrays.stream(targetVector.getArray()).boxed().toList());      // Contains all the target of the system to be solved
            for (int equationId = 0; equationId < activeConstraints.size(); equationId++) {
                addActivatedConstraints(network, equationId, activeConstraints, solverUtils, nonlinearConstraintIndexes,
                        completeEquationsToSolve, targetVector, wholeTargetVector, listElementNumWithQEqUnactivated);       // Add Linear constraints, index nonLinear ones and get target values
            }
            int totalActiveConstraints = completeEquationsToSolve.size();
            completeEquationsToSolve.addAll(equationsQBusV);                                                                // Add all unactivated equation on Q

            // Set Target Q on the unactive equations added
            for (int equationId = totalActiveConstraints; equationId < completeEquationsToSolve.size(); equationId++) {
                Equation<AcVariableType, AcEquationType> equation = completeEquationsToSolve.get(equationId);
                LfBus controllerBus = network.getBus(equation.getElementNum()); //controlledBus.getGeneratorVoltageControl().get().getControllerElements().get(0);
                if (equationId - totalActiveConstraints < equationsQBusV.size() / 2) {
                    wholeTargetVector.add(controllerBus.getMinQ() - controllerBus.getLoadTargetQ());    //blow target
                } else {
                    wholeTargetVector.add(controllerBus.getMaxQ() - controllerBus.getLoadTargetQ());    //bup target
                }
                nonlinearConstraintIndexes.add(equationId);
                INDEQUNACTIVEQ.put(equationId, equation);
            }

            int numConstraints = completeEquationsToSolve.size();
            LOGGER.info("Defined {} constraints", numConstraints);
            setMainCallbackCstIndexes(nonlinearConstraintIndexes);
            setConEqBnds(wholeTargetVector);

            // =============== Declaration of Complementarity Constraints ===============
            List<Integer> listTypeVar = new ArrayList<>(Collections.nCopies(2 * complConstVariables / 5, KNConstants.KN_CCTYPE_VARVAR));
            List<Integer> bVarList = new ArrayList<>(); // b_up, b_low
            List<Integer> vInfSuppList = new ArrayList<>(); // V_inf, V_sup

            for (int i = 0; i < complConstVariables / 5; i++) {
                vInfSuppList.add(compVarStartIndex + 5 * i); //Vinf
                vInfSuppList.add(compVarStartIndex + 5 * i + 1); //Vsup
                bVarList.add(compVarStartIndex + 5 * i + 3); // bup
                bVarList.add(compVarStartIndex + 5 * i + 2); // blow
            }

            setCompConstraintsTypes(listTypeVar);
            setCompConstraintsParts(bVarList, vInfSuppList); // b_low compl with V_sup  and  b_up compl with V_inf
        }

        @Override
        protected void setUpScalingFactors(int numTotalVariables) throws KNException {
            List<Double> scalingFactors = new ArrayList<>(Collections.nCopies(numTotalVariables, 1.0));
            List<Double> scalingCenters = new ArrayList<>(Collections.nCopies(numTotalVariables, 0.0));
            for (int i = numLFVariables; i < slackVStartIndex; i++) {
                scalingFactors.set(i, 1e-2);
            }

            //todo : verify that this range(1,n) coincides with order given to knitro
            int n = numTotalVariables;
            ArrayList<Integer> list = new ArrayList<>(n);
            for (int i = 0; i < n; i++) {
                list.add(i);
            }

            setVarScaleFactors(new KNSparseVector<>(list, scalingFactors));
            setVarScaleCenters(new KNSparseVector<>(list, scalingCenters));
        }

        @Override
        protected void initializeCustomizedVariables(List<Double> lowerBounds, List<Double> upperBounds, List<Double> initialValues, int numTotalVariables) {
            super.initializeCustomizedVariables(lowerBounds, upperBounds, initialValues, numTotalVariables);

            // Set bounds for complementarity variables (≥ 0)
            for (int i = 0; i < complConstVariables / 5; i++) {
                lowerBounds.set(compVarStartIndex + 5 * i, 0.0);
                lowerBounds.set(compVarStartIndex + 5 * i + 1, 0.0);

                lowerBounds.set(compVarStartIndex + 5 * i + 2, 0.0);
                lowerBounds.set(compVarStartIndex + 5 * i + 3, 0.0);
                lowerBounds.set(compVarStartIndex + 5 * i + 4, 0.0);

                initialValues.set(compVarStartIndex + 5 * i + 2, 1.0);
                initialValues.set(compVarStartIndex + 5 * i + 3, 1.0);
            }
        }

        /**
         * Adds a single constraint to the Knitro problem.
         * Linear constraints are directly encoded; non-linear ones are delegated to the callback.
         *
         * @param equationId             Index of the equation in the list.
         * @param equationsToSolve       Base list of all equations to solve.
         * @param solverUtils            Utilities to extract linear constraint components.
         * @param nonLinearConstraintIds Output list of non-linear constraint indices.
         */
        private void addActivatedConstraints(
                LfNetwork network,
                int equationId,
                List<Equation<AcVariableType, AcEquationType>> equationsToSolve,
                NonLinearExternalSolverUtils solverUtils,
                List<Integer> nonLinearConstraintIds,
                List<Equation<AcVariableType, AcEquationType>> completeEquationsToSolve,
                TargetVector<AcVariableType, AcEquationType> targetVector,
                List<Double> wholeTargetVector,
                List<Integer> listBusesWithQEqToAdd) {

            Equation<AcVariableType, AcEquationType> equation = equationsToSolve.get(equationId);
            AcEquationType equationType = equation.getType();
            List<EquationTerm<AcVariableType, AcEquationType>> terms = equation.getTerms();

            if (equationType == BUS_TARGET_V) {
                LfBus controlledBus = network.getBuses().get(equation.getElement(network).get().getNum());
                LfBus controllerBus = controlledBus.getGeneratorVoltageControl().get().getControllerElements().get(0);
                boolean addComplConstraintsVariable = listBusesWithQEqToAdd.contains(controllerBus.getNum());  //help to decide wether V eq have to be dupplicated or not
                if (NonLinearExternalSolverUtils.isLinear(equationType, terms)) {
                    try {

                        // Extract linear constraint components
                        var linearConstraint = solverUtils.getLinearConstraint(equationType, terms);
                        List<Integer> varVInfIndices = new ArrayList<>(linearConstraint.listIdVar());
                        List<Double> coefficientsVInf = new ArrayList<>(linearConstraint.listCoef());

                        // To add complementarity conditions, Knitro requires that they be written as two variables
                        // that complement each other. That is why we are introducing new variables that will play this role.
                        // We call them V_inf, V_sup, b_low and b_up. The two lasts appear in non-linear constraints
                        // Equations on V are duplicated, we add V_inf to one and V_sup to the other.
                        // We are also adding a variable V_aux that allows us to perform the PV -> PQ switch.

                        // ---- V_inf Equation ----
                        if (addComplConstraintsVariable) {
                            int compVarBaseIndex = compVarStartIndex + 5 * vSuppEquationLocalIds.get(equationId);
                            varVInfIndices.add(compVarBaseIndex); // V_inf
                            varVInfIndices.add(compVarBaseIndex + 4); // V_aux
                            coefficientsVInf.add(1.0);
                            coefficientsVInf.add(-1.0);
                        }
                        // Add slack variables if applicable
                        int slackBase = getSlackIndexBase(equationType, equationId);
                        if (slackBase >= 0) {
                            varVInfIndices.add(slackBase);       // Sm
                            varVInfIndices.add(slackBase + 1);   // Sp
                            coefficientsVInf.add(-1.0);
                            coefficientsVInf.add(1.0);
                        }

                        for (int i = 0; i < varVInfIndices.size(); i++) {
                            this.addConstraintLinearPart(equationId, varVInfIndices.get(i), coefficientsVInf.get(i));
                        }

                        LOGGER.trace("Added linear constraint #{} of type {}", equationId, equationType);

                        // ---- V_sup Equation ----
                        if (addComplConstraintsVariable) {
                            int compVarBaseIndex = compVarStartIndex + 5 * vSuppEquationLocalIds.get(equationId);
                            List<Integer> varVSupIndices = new ArrayList<>(linearConstraint.listIdVar());
                            List<Double> coefficientsVSup = new ArrayList<>(linearConstraint.listCoef());

                            // Add complementarity constraints' variables
                            varVSupIndices.add(compVarBaseIndex + 1); // V_sup
                            varVSupIndices.add(compVarBaseIndex + 4); // V_aux
                            coefficientsVSup.add(-1.0);
                            coefficientsVSup.add(1.0);

                            // Add slack variables if applicable
                            if (slackBase >= 0) {
                                varVSupIndices.add(slackBase);       // Sm
                                varVSupIndices.add(slackBase + 1);   // Sp
                                coefficientsVSup.add(-1.0);
                                coefficientsVSup.add(1.0);
                            }

                            for (int i = 0; i < varVSupIndices.size(); i++) {
                                this.addConstraintLinearPart(equationsToSolve.size() + vSuppEquationLocalIds
                                        .get(equationId), varVSupIndices.get(i), coefficientsVSup.get(i));
                            }
                        }
                    } catch (UnsupportedOperationException e) {
                        throw new PowsyblException("Failed to process linear constraint for equation #" + equationId, e);
                    }

                } else {
                    nonLinearConstraintIds.add(equationId);
                    nonLinearConstraintIds.add(equationsToSolve.size() + vSuppEquationLocalIds.get(equationId));
                }

                // Add the duplicated equation (ie V_sup eq) to the list of eq to solve and its target
                if (addComplConstraintsVariable) {
                    Equation<AcVariableType, AcEquationType> vEq = equationsToSolve.get(equationId);
                    completeEquationsToSolve.add(vEq);
                    wholeTargetVector.add(Arrays.stream(targetVector.getArray()).boxed().toList().get(equationId));
                }

            } else {
                if (NonLinearExternalSolverUtils.isLinear(equationType, terms)) {
                    try {
                        // Extract linear constraint components
                        var linearConstraint = solverUtils.getLinearConstraint(equationType, terms);
                        List<Integer> varIndices = new ArrayList<>(linearConstraint.listIdVar());
                        List<Double> coefficients = new ArrayList<>(linearConstraint.listCoef());

                        // Add slack variables if applicable
                        int slackBase = getSlackIndexBase(equationType, equationId);
                        if (slackBase >= 0) {
                            varIndices.add(slackBase);       // Sm
                            varIndices.add(slackBase + 1);   // Sp
                            coefficients.add(-1.0);
                            coefficients.add(+1.0);
                        }

                        for (int i = 0; i < varIndices.size(); i++) {
                            this.addConstraintLinearPart(equationId, varIndices.get(i), coefficients.get(i));
                        }

                        LOGGER.trace("Added linear constraint #{} of type {}", equationId, equationType);
                    } catch (UnsupportedOperationException e) {
                        throw new PowsyblException("Failed to process linear constraint for equation #" + equationId, e);
                    }
                } else {
                    nonLinearConstraintIds.add(equationId);
                }
            }
        }

        private int getcompVarBaseIndex(int equationId) {
            return compVarStartIndex + 5 * vSuppEquationLocalIds.get(equationId);
        }

        private int getElemNumControlledBus(int elemNum) {
            return elemNumControlledControllerBus.get(elemNum);
        }

        @Override
        protected KNEvalGACallback createGradientCallback(JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                                          List<Integer> listNonZerosCtsDense, List<Integer> listNonZerosVarsDense,
                                                          List<Integer> listNonZerosCtsSparse, List<Integer> listNonZerosVarsSparse) {

            return new UseReactiveLimitsCallbackEvalG(jacobianMatrix, listNonZerosCtsDense, listNonZerosVarsDense,
                    listNonZerosCtsSparse, listNonZerosVarsSparse, network, equationSystem,
                    knitroParameters, numberOfPowerFlowVariables);
        }

        @Override
        protected void addAdditionalJacobianVariables(int constraintIndex,
                                                      Equation<AcVariableType, AcEquationType> equation,
                                                      List<Integer> variableIndices) {
            super.addAdditionalJacobianVariables(constraintIndex, equation, variableIndices);
            int numberLFEq = equationSystem.getIndex().getSortedEquationsToSolve().size() - 3 * vSuppEquationLocalIds.size();
            AcEquationType equationType = equation.getType();
            // Add complementarity constraints' variables if the constraint type has them
            int compVarStart;
            // Case of inactive Q equations, we take the V equation associated to it to get the right index used to order the equation system
            if (equationType == BUS_TARGET_Q && !equation.isActive()) {
                int elemNumControlledBus = elemNumControlledControllerBus.get(equation.getElementNum());        // Controller bus
                List<Equation<AcVariableType, AcEquationType>> listEqControlledBus = equationSystem             // Equations of the Controller bus
                        .getEquations(ElementType.BUS, elemNumControlledBus);
                Equation<AcVariableType, AcEquationType> eqVControlledBus = listEqControlledBus.stream()        // Take the one on V
                        .filter(e -> e.getType() == BUS_TARGET_V).toList().get(0);
                int indexEqVAssociated = equationSystem.getIndex().getSortedEquationsToSolve().indexOf(eqVControlledBus);   // Find the index of the V equation associated

                compVarStart = vSuppEquationLocalIds.get(indexEqVAssociated);
                if (constraintIndex < numberLFEq + 2 * vSuppEquationLocalIds.size()) {
                    variableIndices.add(compVarStartIndex + 5 * compVarStart + 2); // b_low
                } else {
                    variableIndices.add(compVarStartIndex + 5 * compVarStart + 3); // b_up
                }
            }
        }

        /**
         * Callback used by Knitro to evaluate the non-linear parts of the objective and constraint functions.
         */
        private static final class UseReactiveLimitsCallbackEvalFC extends RelaxedCallbackEvalFC {

            private final UseReactiveLimitsKnitroProblem problemInstance;

            private UseReactiveLimitsCallbackEvalFC(UseReactiveLimitsKnitroProblem problemInstance,
                                          List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                                          List<Integer> nonLinearConstraintIds) {
                super(problemInstance, sortedEquationsToSolve,nonLinearConstraintIds);
                this.problemInstance = problemInstance;
            }

            @Override
            protected double addModificationOfNonLinearConstraints(int equationId, AcEquationType equationType,
                                                                   List<Double> x) {
                double constraintValue = 0;
                Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(equationId);

                if (equation.isActive()) {
                    int slackIndexBase = problemInstance.getSlackIndexBase(equationType, equationId);
                    if (slackIndexBase >= 0) {
                        double sm = x.get(slackIndexBase);        // negative slack
                        double sp = x.get(slackIndexBase + 1);    // positive slack
                        constraintValue += sp - sm;              // add slack contribution
                    }
                } else {
                    int nmbreEqUnactivated = sortedEquationsToSolve.stream().filter(
                            e -> !e.isActive()).toList().size();
                    int elemNum = equation.getElementNum();
                    int elemNumControlledBus = problemInstance.getElemNumControlledBus(elemNum);
                    List<Equation<AcVariableType, AcEquationType>> controlledBusEquations = sortedEquationsToSolve.stream()
                            .filter(e -> e.getElementNum() == elemNumControlledBus).toList();
                    // the V equation
                    Equation<AcVariableType, AcEquationType> equationV = controlledBusEquations.stream().filter(e -> e.getType() == BUS_TARGET_V).toList().get(0);
                    int equationVId = sortedEquationsToSolve.indexOf(equationV);                                        //Index of V equation
                    int compVarBaseIndex = problemInstance.getcompVarBaseIndex(equationVId);
                    if (equationId - sortedEquationsToSolve.size() + nmbreEqUnactivated / 2 < 0) { // Q_low Constraint
                        double bLow = x.get(compVarBaseIndex + 2);
                        constraintValue -= bLow;
                    } else {    // Q_up Constraint
                        double bUp = x.get(compVarBaseIndex + 3);
                        constraintValue += bUp;
                    }
                }

                return constraintValue;
            }
        }

        /**
         * Callback used by Knitro to evaluate the gradient (Jacobian matrix) of the constraints.
         * Only constraints (no objective) are handled here.
         */
        private static final class UseReactiveLimitsCallbackEvalG extends RelaxedCallbackEvalG {

            private final int numLFVar;
            private final int numVEq;
            private final int numPQEq;

            private final LinkedHashMap indRowVariable;

            private UseReactiveLimitsCallbackEvalG(
                    JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                    List<Integer> denseConstraintIndices,
                    List<Integer> denseVariableIndices,
                    List<Integer> sparseConstraintIndices,
                    List<Integer> sparseVariableIndices,
                    LfNetwork network,
                    EquationSystem<AcVariableType, AcEquationType> equationSystem,
                    KnitroSolverParameters knitroParameters,
                    int numLFVariables) {

                super(jacobianMatrix, denseConstraintIndices, denseVariableIndices, sparseConstraintIndices, sparseVariableIndices,
                        network, equationSystem, knitroParameters, numLFVariables);

                this.numLFVar = equationSystem.getIndex().getSortedVariablesToFind().size();

                this.numVEq = equationSystem.getIndex().getSortedEquationsToSolve().stream().filter(
                        e -> e.getType() == BUS_TARGET_V).toList().size();

                this.numPQEq = equationSystem.getIndex().getSortedEquationsToSolve().stream().filter(
                        e -> e.getType() == BUS_TARGET_Q ||
                                e.getType() == BUS_TARGET_P).toList().size();

                this.indRowVariable = new LinkedHashMap();
                List<Variable<AcVariableType>> listVar = equationSystem.getVariableSet().getVariables().stream().toList();
                for (Variable<AcVariableType> variable : listVar) {
                    indRowVariable.put(variable.getRow(), variable);
                }
            }

            @Override
            public void evaluateGA(final List<Double> x, final List<Double> objGrad, final List<Double> jac) {
                // Update internal state and Jacobian
                equationSystem.getStateVector().set(toArray(x));
                AcSolverUtil.updateNetwork(network, equationSystem);
                jacobianMatrix.forceUpdate();

                // Get sparse matrix representation
                SparseMatrix sparseMatrix = jacobianMatrix.getMatrix().toSparse();
                int[] columnStart = sparseMatrix.getColumnStart();
                double[] values = sparseMatrix.getValues();

                // Determine which list to use based on Knitro settings
                List<Integer> constraintIndices;
                List<Integer> variableIndices;

                int routineType = knitroParameters.getGradientUserRoutine();
                if (routineType == 1) {
                    constraintIndices = denseConstraintIndices;
                    variableIndices = denseVariableIndices;
                } else if (routineType == 2) {
                    constraintIndices = sparseConstraintIndices;
                    variableIndices = sparseVariableIndices;
                } else {
                    throw new IllegalArgumentException("Unsupported gradientUserRoutine value: " + routineType);
                }

                // Fill Jacobian values
                boolean firstIteration = true;
                int iRowIndices = 0;
                int currentConstraint = -1;
                int numVariables = equationSystem.getIndex().getSortedVariablesToFind().size();
                for (int index = 0; index < constraintIndices.size(); index++) {
                    try {
                        if (firstIteration) {
                            currentConstraint = constraintIndices.get(index);
                        }

                        int ct = constraintIndices.get(index);
                        int var = variableIndices.get(index);
                        double value;

                        if (ct < Arrays.stream(columnStart).count()) {

                            // Find matching (var, ct) entry in sparse column
                            int colStart = columnStart[ct];

                            if (!firstIteration) {
                                if (currentConstraint != ct) {
                                    iRowIndices = 0;
                                    currentConstraint = ct;
                                }
                            }

                            if (var >= numVariables) {
                                if (((var - numVariables) & 1) == 0) {
                                    // set Jacobian entry to -1.0 if slack variable is Sm
                                    value = -1.0;
                                } else {
                                    // set Jacobian entry to +1.0 if slack variable is Sp
                                    value = 1.0;
                                }
                            } else {
                                value = values[colStart + iRowIndices++];
                            }
                            jac.set(index, value);
                            if (firstIteration) {
                                firstIteration = false;
                            }
                        } else { // Case of Unactivated Q equations
                            // If var is a LF variable : derivate non-activated equations
                            value = 0.0;
                            Equation<AcVariableType, AcEquationType> equation = INDEQUNACTIVEQ.get(ct);
                            if (var < numLFVar) {
                                // add non-linear terms
                                for (Map.Entry<Variable<AcVariableType>, List<EquationTerm<AcVariableType, AcEquationType>>> e : equation.getTermsByVariable().entrySet()) {
                                    for (EquationTerm<AcVariableType, AcEquationType> term : e.getValue()) {
                                        Variable<AcVariableType> v = e.getKey();
                                        if (indRowVariable.get(var) == v) {
                                            value += term.isActive() ? term.der(v) : 0;
                                        }

                                    }
                                }
                            }
                            // todo last resort is check how it was here !
                            // Check if var is a b_low or b_up var
                            if (var >= numLFVar + 2 * (numPQEq + numVEq)) {
                                int rest = (var - numLFVar - 2 * (numPQEq + numVEq)) % 5;
                                if (rest == 2) {
                                    // set Jacobian entry to -1.0 if variable is b_low
                                    value = -1.0;
                                } else if (rest == 3) {
                                    // set Jacobian entry to 1.0 if variable is b_up
                                    value = 1.0;
                                }
                            }
                            jac.set(index, value);
                        }
                    } catch (Exception e) {
                        int varId = routineType == 1 ? denseVariableIndices.get(index) : sparseVariableIndices.get(index);
                        int ctId = routineType == 1 ? denseConstraintIndices.get(index) : sparseConstraintIndices.get(index);
                        LOGGER.error("Error while filling Jacobian term at var {} and constraint {}", varId, ctId, e);
                    }
                }
            }
        }
    }
}

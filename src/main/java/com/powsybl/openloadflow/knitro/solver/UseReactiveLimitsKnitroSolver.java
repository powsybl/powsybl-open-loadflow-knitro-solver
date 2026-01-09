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
 * TODO
 *
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Yoann Anezin {@literal <yoann.anezin at artelys.com>}
 */
public class UseReactiveLimitsKnitroSolver extends AbstractRelaxedKnitroSolver {

    private static final Logger LOGGER = LoggerFactory.getLogger(UseReactiveLimitsKnitroSolver.class);

    // Number of variables including slack variables
    private final int numLFandSlackVariables;

    // Number of complementary equations added to open load flow equations system
    private final int numComplementaryEquationsAddedToEquationSystem;

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
        // TODO : remove this and only homogeneize after
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

        this.numComplementaryEquationsAddedToEquationSystem = equationsQToAdd.size();

        // for each complementary equation to add, 3 new variables are used on V equations and 2 on Q equations
        // le nombre de variable total comprend ces variables, ainsi que celle issue d'OLF et de la relaxation (slack)
        this.numberOfVariables = numLFandSlackVariables + numComplementaryEquationsAddedToEquationSystem * 5;
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
        // log solution, slacks and penalties
        super.processSolution(solver, solution, problemInstance);

        // log the switches computed by the optimization
        logSwitches(solution, compVarStartIndex, numComplementaryEquationsAddedToEquationSystem);
    }

    /**
     * Inform all switches PV -> PQ done in the solution found
     *
     * @param solution      Solution returned by the optimization
     * @param startIndex    first index of complementarity constraints variables in x
     * @param count         number of b_low / b_up different variables
     */
    private void logSwitches(KNSolution solution, int startIndex, int count) {
        LOGGER.info("=== Switches Done===");
        List<Double> x = solution.getX(); // solution returned by the optimization
        double eps = knitroParameters.getSlackThreshold();

        for (int i = 0; i < count; i++) {
            double vInf = x.get(startIndex + 5 * i);
            double vSup = x.get(startIndex + 5 * i + 1);
            double bLow = x.get(startIndex + 5 * i + 2);
            double bUp = x.get(startIndex + 5 * i + 3);
            // FIXME : this is not determinist
            String busId = equationSystem.getIndex()
                    .getSortedEquationsToSolve().stream().filter(e ->
                            e.getType() == BUS_TARGET_V).toList().get(i).getElement(network).get().getId();

            // switch to lower bound case: bLow is null and the auxiliary variable is not
            if (Math.abs(bLow) < eps && (vInf > eps || vSup > eps)) {
                LOGGER.info("Switch PV -> PQ on bus {}, Q set at Qmin", busId);

            // switch to upper bound case: bUp is null and the auxiliary variable is not
            } else if (Math.abs(bUp) < eps && (vInf > eps || vSup > eps)) {
                LOGGER.info("Switch PV -> PQ on bus {}, Q set at Qmax", busId);
            }
        }
    }

    private final class UseReactiveLimitsKnitroProblem extends AbstractRelaxedKnitroProblem {

        private List<Equation<AcVariableType, AcEquationType>> completeEquationsToSolve;
        private List<Double> completeTargetVector;

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
                            3 * numComplementaryEquationsAddedToEquationSystem, numLFandSlackVariables, voltageInitializer);

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

        // ok
        @Override
        protected void initializeCustomizedVariables(List<Double> lowerBounds, List<Double> upperBounds, List<Double> initialValues, int numTotalVariables) {
            // initialize slack variables
            super.initializeCustomizedVariables(lowerBounds, upperBounds, initialValues, numTotalVariables);

            // initialize auxiliary variables in complementary constraints
            for (int i = 0; i < numComplementaryEquationsAddedToEquationSystem; i++) {
                lowerBounds.set(compVarStartIndex + 5 * i, 0.0);
                lowerBounds.set(compVarStartIndex + 5 * i + 1, 0.0);

                lowerBounds.set(compVarStartIndex + 5 * i + 2, 0.0);
                lowerBounds.set(compVarStartIndex + 5 * i + 3, 0.0);
                lowerBounds.set(compVarStartIndex + 5 * i + 4, 0.0);

                initialValues.set(compVarStartIndex + 5 * i + 2, 1.0);
                initialValues.set(compVarStartIndex + 5 * i + 3, 1.0);
            }
        }

        // ok
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
        protected void setupConstraints() throws KNException {
            activeConstraints = equationSystem.getIndex().getSortedEquationsToSolve();

            // ce probleme modélise des contraintes de complémentarité pour modéliser le passage PV/PQ des bus
            // pour se faire, des contraintes sont ajoutées au système par rapport au système d'OLF, et donc des membres droits également
            // il n'est donc pas possible d'utiliser directement le système et le membre droit, c'est pourquoi on initialise les objets suivants
            // ceux ci comprennent tous les élements du système / rhs d'OLF, et tous ceux qui sont ajoutés (pour les contraintes de complémentarité)
            completeEquationsToSolve = new ArrayList<>(activeConstraints);  // contains all equations of the final system to be solved
            completeTargetVector = new ArrayList<>(Arrays.stream(targetVector.getArray()).boxed().toList()); // contains all the target of the system to be solved

            // create solver utils here to only create one
            NonLinearExternalSolverUtils solverUtils = new NonLinearExternalSolverUtils();

            // add linear constraints and fill the list of non-linear constraints
            addLinearConstraints(activeConstraints, solverUtils);

            // nombre de variables issues du système d'OLF et dont les équations de contrôle en tension ont été dupliquées
            // pour les cas où il y a des bornes en réactif bien définies
            int intermediateNumberOfActiveEquations = completeEquationsToSolve.size();

            // ajoute les constraintes qui permettent de considérer les bornes en réactif
            completeEquationsToSolve.addAll(equationsQBusV);
            for (int equationId = intermediateNumberOfActiveEquations; equationId < completeEquationsToSolve.size(); equationId++) {
                Equation<AcVariableType, AcEquationType> equation = completeEquationsToSolve.get(equationId);
                LfBus controllerBus = network.getBus(equation.getElementNum());
                if (equationId - intermediateNumberOfActiveEquations < equationsQBusV.size() / 2) {
                    completeTargetVector.add(controllerBus.getMinQ() - controllerBus.getLoadTargetQ());    // blow target
                } else {
                    completeTargetVector.add(controllerBus.getMaxQ() - controllerBus.getLoadTargetQ());    // bup target
                }
                // ces équations sont non-linéaires sachant que les flux en réactif apparaissent dans le bilan
                nonlinearConstraintIndexes.add(equationId);
                INDEQUNACTIVEQ.put(equationId, equation);
            }

            // cela comprend le nombre d'équations provenant du système d'OLF,
            // mais aussi de toutes celles ajoutées pour modéliser la complémentarité
            int totalNumConstraints = completeEquationsToSolve.size();
            LOGGER.info("Defining {} active constraints", totalNumConstraints);
            // TODO : ajouter complementary constraints

            // pass to Knitro the indexes of non-linear constraints, that will be evaluated in the callback function
            // NOTE: dans les non-linear constraints, il y a également ici des éléments spécifiques aux contraintes que l'on a ajoutées
            setMainCallbackCstIndexes(nonlinearConstraintIndexes);

            // right hand side (targets), specific au problème complet que l'on cherche à résoudre
            setConEqBnds(completeTargetVector);

            // declaration of complementarity constraints in Knitro
            addComplementaryConstraints();
        }

        /**
         * Adds a single constraint to the Knitro problem.
         * Linear constraints are directly encoded; non-linear ones are delegated to the callback.
         *
         * @param equationId             Index of the equation in the list.
         */
        @Override
        protected void addConstraint(int equationId, List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                                     NonLinearExternalSolverUtils solverUtils) {

            Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(equationId);
            AcEquationType equationType = equation.getType();
            List<EquationTerm<AcVariableType, AcEquationType>> terms = equation.getTerms();

            if (equationType == BUS_TARGET_V) {
                LfBus controlledBus = network.getBuses().get(equation.getElement(network).get().getNum());
                LfBus controllerBus = controlledBus.getGeneratorVoltageControl().get().getControllerElements().get(0);

                // boolean to indicate if the V equation will be duplicated
                boolean addComplementarityConstraintsVariable = listElementNumWithQEqUnactivated.contains(controllerBus.getNum());

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

                        // vInf equation, provenant de l'équation du système d'OLF
                        if (addComplementarityConstraintsVariable) {
                            int compVarBaseIndex = getComplementarityVarBaseIndex(equationId);
                            varVInfIndices.add(compVarBaseIndex); // vInf
                            varVInfIndices.add(compVarBaseIndex + 4); // vAux
                            coefficientsVInf.add(1.0);
                            coefficientsVInf.add(-1.0);
                        }

                        // add slack variables if applicable
                        // NOTE: des slacks peuvent être ajoutées pour relaxer l'équation, même s'il n'y a pas de complémentarité pour cette équation
                        addAdditionalConstraintVariables(equationId, equationType, varVInfIndices, coefficientsVInf);

                        for (int i = 0; i < varVInfIndices.size(); i++) {
                            this.addConstraintLinearPart(equationId, varVInfIndices.get(i), coefficientsVInf.get(i));
                        }

                        LOGGER.trace("Added linear constraint #{} of type {}", equationId, equationType);

                        // vSup equation, ajoutée uniquement s'il s'agit d'une contrainte de complémentarité
                        // cette équation sert à relacher la contrainte en tension dans le cas où la borne min en réactif est atteinte
                        if (addComplementarityConstraintsVariable) {
                            List<Integer> varVSupIndices = new ArrayList<>(linearConstraint.listIdVar());
                            List<Double> coefficientsVSup = new ArrayList<>(linearConstraint.listCoef());

                            // add complementarity constraints' variables
                            int compVarBaseIndex = getComplementarityVarBaseIndex(equationId);
                            varVSupIndices.add(compVarBaseIndex + 1); // V_sup
                            varVSupIndices.add(compVarBaseIndex + 4); // V_aux
                            coefficientsVSup.add(-1.0);
                            coefficientsVSup.add(1.0);

                            // add slack variables if applicable
                            addAdditionalConstraintVariables(equationId, equationType, varVSupIndices, coefficientsVSup);

                            for (int i = 0; i < varVSupIndices.size(); i++) {
                                int equationIndex = sortedEquationsToSolve.size() + vSuppEquationLocalIds.get(equationId);
                                this.addConstraintLinearPart(equationIndex, varVSupIndices.get(i), coefficientsVSup.get(i));
                            }
                        }
                    } catch (UnsupportedOperationException e) {
                        throw new PowsyblException("Failed to process linear constraint for equation #" + equationId, e);
                    }

                // si l'équation en V n'est pas linéaire, on ajoute l'indice au non linear equations (ainsi que le duplicat vSup)
                } else {
                    nonlinearConstraintIndexes.add(equationId);
                    nonlinearConstraintIndexes.add(sortedEquationsToSolve.size() + vSuppEquationLocalIds.get(equationId));
                }

                // les equations dupliquees en tension ont été spécifiées au problème Knitro dans les lignes précédentes
                // afin de garder une cohérence avec le système d'équation modélisé, ainsi que le rhs, on doit également mettre à jour les objects correspondant
                // notamment, il faut ajouter l'équation pour vSup à la liste des équations modélisées et au rhs
                if (addComplementarityConstraintsVariable) {
                    completeEquationsToSolve.add(equation);
                    completeTargetVector.add(Arrays.stream(targetVector.getArray()).boxed().toList().get(equationId)); // on applique la même tension
                }

            // for other type of equations, the constraint can be added as usual
            } else {
                super.addConstraint(equationId, sortedEquationsToSolve, solverUtils);
            }
        }

        /**
         * Ajoute les complementarity constraints dans la définition du problème Knitro.
         */
        private void addComplementaryConstraints() throws KNException {
            List<Integer> listTypeVar = new ArrayList<>(Collections.nCopies(2 * numComplementaryEquationsAddedToEquationSystem, KNConstants.KN_CCTYPE_VARVAR));
            List<Integer> bVarList = new ArrayList<>(); // bUp and bLow
            List<Integer> vInfSuppList = new ArrayList<>(); // vInf and vSup

            for (int i = 0; i < numComplementaryEquationsAddedToEquationSystem; i++) {
                vInfSuppList.add(compVarStartIndex + 5 * i); // vInf
                vInfSuppList.add(compVarStartIndex + 5 * i + 1); // vSup
                bVarList.add(compVarStartIndex + 5 * i + 3); // bUp
                bVarList.add(compVarStartIndex + 5 * i + 2); // bLow
            }

            // add the complementary conditions
            // 0 <= bLow perp vSup >= 0 and 0 <= bUp perp vInf >= 0
            setCompConstraintsTypes(listTypeVar);
            setCompConstraintsParts(bVarList, vInfSuppList);
        }

        private int getComplementarityVarBaseIndex(int equationId) {
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

        /**
         * Builds the sparse Jacobian matrix by identifying non-zero entries
         * for each non-linear constraint, including contributions from slack variables.
         *
         * @param sortedEquationsToSolve Ordered list of equations to solve.
         * @param nonLinearConstraintIds Indices of non-linear constraints within the sorted equation list.
         * @param jacobianRowIndices     Output: row indices (constraints) of non-zero Jacobian entries.
         * @param jacobianColumnIndices  Output: column indices (variables) of non-zero Jacobian entries.
         */
        @Override
        public void buildSparseJacobianMatrix(
                List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve,
                List<Integer> nonLinearConstraintIds,
                List<Integer> jacobianRowIndices,
                List<Integer> jacobianColumnIndices) {
            int numberLFEq = sortedEquationsToSolve.size() - 3 * vSuppEquationLocalIds.size();

            for (Integer constraintIndex : nonLinearConstraintIds) {
                Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(constraintIndex);
                AcEquationType equationType = equation.getType();

                // Collect all variable indices involved in the current equation
                Set<Integer> involvedVariables = equation.getTerms().stream()
                        .flatMap(term -> term.getVariables().stream())
                        .map(Variable::getRow)
                        .collect(Collectors.toCollection(TreeSet::new));

                // Add slack variables if the constraint type has them
                int slackStart = switch (equationType) {
                    case BUS_TARGET_P -> pEquationLocalIds.getOrDefault(constraintIndex, -1);
                    case BUS_TARGET_Q -> qEquationLocalIds.getOrDefault(constraintIndex, -1);
                    case BUS_TARGET_V -> vEquationLocalIds.getOrDefault(constraintIndex, -1);
                    default -> -1;
                };

                if (slackStart >= 0) {
                    int slackBaseIndex = switch (equationType) {
                        case BUS_TARGET_P -> slackPStartIndex;
                        case BUS_TARGET_Q -> slackQStartIndex;
                        case BUS_TARGET_V -> slackVStartIndex;
                        default -> throw new IllegalStateException("Unexpected constraint type: " + equationType);
                    };
                    involvedVariables.add(slackBaseIndex + 2 * slackStart);     // Sm
                    involvedVariables.add(slackBaseIndex + 2 * slackStart + 1); // Sp
                }

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
                        involvedVariables.add(compVarStartIndex + 5 * compVarStart + 2); // b_low
                    } else {
                        involvedVariables.add(compVarStartIndex + 5 * compVarStart + 3); // b_up
                    }
                }

                // Add one entry for each non-zero (constraintIndex, variableIndex)
                jacobianColumnIndices.addAll(involvedVariables);
                jacobianRowIndices.addAll(Collections.nCopies(involvedVariables.size(), constraintIndex));

                long end = System.nanoTime();
            }
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
                super(problemInstance, sortedEquationsToSolve, nonLinearConstraintIds);
                this.problemInstance = problemInstance;
            }

            @Override
            protected double addModificationOfNonLinearConstraints(int equationId, AcEquationType equationType,
                                                                   List<Double> x) {
                double constraintValue = 0;
                Equation<AcVariableType, AcEquationType> equation = sortedEquationsToSolve.get(equationId);
                // if the equation is active, then it is treated as an open load flow constraint
                if (equation.isActive()) {
                    // add relaxation if necessary
                    constraintValue += super.addModificationOfNonLinearConstraints(equationId, equationType, x);

                // otherwise, these are bLow and bUp constraints and the term must be added
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
                    int compVarBaseIndex = problemInstance.getComplementarityVarBaseIndex(equationVId);
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

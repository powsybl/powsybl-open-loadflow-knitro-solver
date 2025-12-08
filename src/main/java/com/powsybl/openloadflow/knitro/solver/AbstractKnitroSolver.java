/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.artelys.knitro.api.*;
import com.powsybl.commons.PowsyblException;
import com.powsybl.commons.report.ReportNode;
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.ac.solver.AbstractAcSolver;
import com.powsybl.openloadflow.ac.solver.AcSolverResult;
import com.powsybl.openloadflow.ac.solver.AcSolverStatus;
import com.powsybl.openloadflow.ac.solver.AcSolverUtil;
import com.powsybl.openloadflow.equations.*;
import com.powsybl.openloadflow.network.LfBus;
import com.powsybl.openloadflow.network.LfNetwork;
import com.powsybl.openloadflow.network.util.VoltageInitializer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

import static com.google.common.primitives.Doubles.toArray;

/**
 * Abstract base class for Knitro solvers dedicated to solving the open load-flow equation system.
 * It provides common functionality, including:
 *      - Configuration of the external Knitro solver.
 *      - Creation of the optimization problem (extending {@link AbstractKnitroProblem}).
 *      - Processing of the obtained solution.
 * This class can be extended to add custom behavior to any of these features (e.g., in {@link RelaxedKnitroSolver}.
 * For example, if you modify the optimization problem, you may also need to update the solution-processing logic.
 *
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public abstract class AbstractKnitroSolver extends AbstractAcSolver {

    protected static final Logger LOGGER = LoggerFactory.getLogger(AbstractKnitroSolver.class);

    protected KnitroSolverParameters knitroParameters;

    protected AbstractKnitroSolver(LfNetwork network, KnitroSolverParameters knitroParameters,
                                EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                JacobianMatrix<AcVariableType, AcEquationType> j, TargetVector<AcVariableType, AcEquationType> targetVector,
                                EquationVector<AcVariableType, AcEquationType> equationVector,
                                boolean detailedReport) {

        super(network, equationSystem, j, targetVector, equationVector, detailedReport);
        this.knitroParameters = knitroParameters;
    }

    /**
     * Sets Knitro solver parameters based on the provided KnitroSolverParameters object.
     * Uses the improved parameter set from RelaxedKnitroSolver.
     *
     * @param solver The Knitro solver instance to configure.
     * @throws KNException if Knitro fails to accept a parameter.
     */
    protected void setSolverParameters(KNSolver solver) throws KNException {
        LOGGER.info("Configuring Knitro solver parameters...");

        solver.setParam(KNConstants.KN_PARAM_GRADOPT, knitroParameters.getGradientComputationMode());
        solver.setParam(KNConstants.KN_PARAM_FEASTOL, knitroParameters.getRelConvEps());
        solver.setParam(KNConstants.KN_PARAM_FEASTOLABS, knitroParameters.getAbsConvEps());
        solver.setParam(KNConstants.KN_PARAM_OPTTOL, knitroParameters.getRelOptEps());
        solver.setParam(KNConstants.KN_PARAM_OPTTOLABS, knitroParameters.getAbsOptEps());
        solver.setParam(KNConstants.KN_PARAM_MAXIT, knitroParameters.getMaxIterations());
        solver.setParam(KNConstants.KN_PARAM_HESSOPT, knitroParameters.getHessianComputationMode());
        solver.setParam(KNConstants.KN_PARAM_SOLTYPE, KNConstants.KN_SOLTYPE_BESTFEAS);
        solver.setParam(KNConstants.KN_PARAM_OUTLEV, 3);

        LOGGER.info("Knitro parameters set: GRADOPT={}, HESSOPT={}, FEASTOL={}, OPTTOL={}, MAXIT={}",
                knitroParameters.getGradientComputationMode(),
                knitroParameters.getHessianComputationMode(),
                knitroParameters.getRelConvEps(),
                knitroParameters.getRelOptEps(),
                knitroParameters.getMaxIterations());
    }

    /**
     * Creates the Knitro problem instance.
     * This must be implemented by subclasses.
     *
     * @param voltageInitializer The voltage initializer to use. This initializes the optimization.
     * @return The created Knitro problem instance.
     */
    protected abstract KNProblem createKnitroProblem(VoltageInitializer voltageInitializer);

    /**
     * Processes the solution after solving.
     * Can be overridden by subclasses for additional processing.
     *
     * @param solver The Knitro solver instance.
     * @param solution The solution obtained from Knitro.
     * @param problemInstance The problem instance.
     */
    protected void processSolution(KNSolver solver, KNSolution solution, KNProblem problemInstance) {
        // log optimization solution details
        LOGGER.info("==== Solution Summary ====");
        LOGGER.info("Optimal objective value  = {}", solution.getObjValue());
        try {
            LOGGER.info("Feasibility violation    = {}", solver.getAbsFeasError());
            LOGGER.info("Optimality violation     = {}", solver.getAbsOptError());

            LOGGER.debug("Optimal x");
            for (int i = 0; i < solution.getX().size(); i++) {
                LOGGER.debug(" x[{}] = {}", i, solution.getX().get(i));
            }
            LOGGER.debug("Optimal constraint values (with corresponding multiplier)");
            List<Double> constraintValues = solver.getConstraintValues();
            for (int i = 0; i < problemInstance.getNumCons(); i++) {
                LOGGER.debug(" c[{}] = {} (lambda = {} )", i, constraintValues.get(i), solution.getLambda().get(i));
            }
            LOGGER.debug("Constraint violation");
            for (int i = 0; i < problemInstance.getNumCons(); i++) {
                LOGGER.debug(" violation[{}] = {} ", i, solver.getConViol(i));
            }
        } catch (KNException e) {
            LOGGER.warn("Failed to get some solution details", e);
        }
    }

    public static void logKnitroStatus(KnitroStatus status) {
        LOGGER.info("Knitro Status: {}", status);
    }

    /**
     * Gets the solution from the solver.
     * Can be overridden by subclasses.
     *
     * @param solver The Knitro solver instance.
     * @return The solution.
     */
    protected KNSolution getSolution(KNSolver solver) {
        return solver.getSolution();
    }

    /**
     * Updates the network with the solution if required.
     *
     * @param solution The solution to apply.
     * @param solverStatus The solver status.
     */
    protected void updateNetworkIfRequired(KNSolution solution, AcSolverStatus solverStatus) {
        if (solverStatus == AcSolverStatus.CONVERGED || knitroParameters.isAlwaysUpdateNetwork()) {
            equationSystem.getStateVector().set(toArray(solution.getX()));
            for (Equation<AcVariableType, AcEquationType> equation : equationSystem.getEquations()) {
                for (EquationTerm<AcVariableType, AcEquationType> term : equation.getTerms()) {
                    term.setStateVector(equationSystem.getStateVector());
                }
            }
            AcSolverUtil.updateNetwork(network, equationSystem);
        }
    }

    @Override
    public AcSolverResult run(VoltageInitializer voltageInitializer, ReportNode reportNode) {
        int nbIter;
        AcSolverStatus acStatus;
        KNProblem instance;

        try {
            instance = createKnitroProblem(voltageInitializer);
        } catch (Exception e) {
            throw new PowsyblException("Exception while trying to build Knitro Problem", e);
        }

        try (KNSolver solver = new KNSolver(instance)) {
            solver.initProblem();
            setSolverParameters(solver);
            solver.solve();

            KNSolution solution = getSolution(solver);
            acStatus = KnitroStatus.fromStatusCode(solution.getStatus()).toAcSolverStatus();
            logKnitroStatus(KnitroStatus.fromStatusCode(solution.getStatus()));
            nbIter = solver.getNumberIters();

            processSolution(solver, solution, instance);
            updateNetworkIfRequired(solution, acStatus);

        } catch (KNException e) {
            throw new PowsyblException("Exception while trying to solve with Knitro", e);
        }

        double slackBusActivePowerMismatch = network.getSlackBuses().stream()
                .mapToDouble(LfBus::getMismatchP)
                .sum();
        return new AcSolverResult(acStatus, nbIter, slackBusActivePowerMismatch);
    }
}


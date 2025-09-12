/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.commons.extensions.AbstractExtension;
import com.powsybl.loadflow.LoadFlowParameters;

/**
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public class KnitroLoadFlowParameters extends AbstractExtension<LoadFlowParameters> {

    private int gradientComputationMode = KnitroSolverParameters.DEFAULT_GRADIENT_COMPUTATION_MODE;
    private int gradientUserRoutine = KnitroSolverParameters.DEFAULT_GRADIENT_USER_ROUTINE;
    private int hessianComputationMode = KnitroSolverParameters.DEFAULT_HESSIAN_COMPUTATION_MODE;
    private double lowerVoltageBound = KnitroSolverParameters.DEFAULT_LOWER_VOLTAGE_BOUND;
    private double upperVoltageBound = KnitroSolverParameters.DEFAULT_UPPER_VOLTAGE_BOUND;
    private int maxIterations = KnitroSolverParameters.DEFAULT_MAX_ITERATIONS;
    private double convEps = KnitroSolverParameters.DEFAULT_RELATIVE_FEASIBILITY_STOPPING_CRITERIA;
    private double absConvEps = KnitroSolverParameters.DEFAULT_ABSOLUTE_FEASIBILITY_STOPPING_CRITERIA;
    private double optEps = KnitroSolverParameters.DEFAULT_RELATIVE_OPTIMALITY_STOPPING_CRITERIA;
    private double absOptEps = KnitroSolverParameters.DEFAULT_ABSOLUTE_OPTIMALITY_STOPPING_CRITERIA;
    private double slackThreshold = KnitroSolverParameters.DEFAULT_SLACK_THRESHOLD;
    private int numThreads = KnitroSolverParameters.DEFAULT_THREAD_NUMBER;
    private boolean alwaysUpdateNetwork = KnitroSolverParameters.ALWAYS_UPDATE_NETWORK_DEFAULT_VALUE;
    private boolean checkLoadFlowSolution = KnitroSolverParameters.CHECK_LOAD_FLOW_SOLUTION_DEFAULT_VALUE;
    private KnitroSolverParameters.KnitroSolverType knitroSolverType = KnitroSolverParameters.DEFAULT_KNITRO_SOLVER_TYPE;

    public int getGradientComputationMode() {
        return gradientComputationMode;
    }

    public KnitroLoadFlowParameters setGradientComputationMode(int gradientComputationMode) {
        if (gradientComputationMode < 1 || gradientComputationMode > 3) {
            throw new IllegalArgumentException("Gradient mode must be between 1 and 3");
        }
        this.gradientComputationMode = gradientComputationMode;
        return this;
    }

    public boolean isAlwaysUpdateNetwork() {
        return alwaysUpdateNetwork;
    }

    public KnitroLoadFlowParameters setAlwaysUpdateNetwork(boolean alwaysUpdateNetwork) {
        this.alwaysUpdateNetwork = alwaysUpdateNetwork;
        return this;
    }

    public boolean isCheckLoadFlowSolution() {
        return checkLoadFlowSolution;
    }

    public KnitroLoadFlowParameters setCheckLoadFlowSolution(boolean checkLoadFlowSolution) {
        this.checkLoadFlowSolution = checkLoadFlowSolution;
        return this;
    }

    public int getGradientUserRoutine() {
        return gradientUserRoutine;
    }

    public KnitroLoadFlowParameters setGradientUserRoutine(int gradientUserRoutine) {
        if (gradientUserRoutine < 1 || gradientUserRoutine > 2) {
            throw new IllegalArgumentException("User routine must be between 1 and 2");
        }

        if (gradientComputationMode == 1) {
            this.numThreads = KnitroSolverParameters.DEFAULT_THREAD_NUMBER;
        } else {
            // Evaluation callback is not thread safe when gradient computation mode is set to finite differences
            this.numThreads = KnitroSolverParameters.DEFAULT_THREAD_NUMBER_FINITE_DIFFERENCES;
        }

        this.gradientUserRoutine = gradientUserRoutine;
        return this;
    }

    public int getHessianComputationMode() {
        return hessianComputationMode;
    }

    public KnitroLoadFlowParameters setHessianComputationMode(int hessianComputationMode) {
        if (hessianComputationMode < 1 || hessianComputationMode > 7) {
            throw new IllegalArgumentException("Hessian computation mode must be between 1 and 7");
        }
        this.hessianComputationMode = hessianComputationMode;
        return this;
    }

    public double getLowerVoltageBound() {
        return lowerVoltageBound;
    }

    public KnitroLoadFlowParameters setLowerVoltageBound(double lowerVoltageBound) {
        if (lowerVoltageBound < 0) {
            throw new IllegalArgumentException("Realistic voltage bounds must strictly greater than 0");
        }
        this.lowerVoltageBound = lowerVoltageBound;
        return this;
    }

    public double getUpperVoltageBound() {
        return upperVoltageBound;
    }

    public KnitroLoadFlowParameters setUpperVoltageBound(double upperVoltageBound) {
        if (upperVoltageBound < 0) {
            throw new IllegalArgumentException("Realistic voltage bounds must strictly greater than 0");
        }
        if (upperVoltageBound <= lowerVoltageBound) {
            throw new IllegalArgumentException("Realistic voltage upper bounds must greater than lower bounds");
        }
        this.upperVoltageBound = upperVoltageBound;
        return this;
    }

    public int getMaxIterations() {
        return maxIterations;
    }

    public KnitroLoadFlowParameters setMaxIterations(int maxIterations) {
        if (maxIterations < 0) {
            throw new IllegalArgumentException("Max iterations parameter must be greater than 0");
        }
        this.maxIterations = maxIterations;
        return this;
    }

    public double getConvEps() {
        return convEps;
    }

    public KnitroLoadFlowParameters setConvEps(double convEps) {
        if (convEps <= 0) {
            throw new IllegalArgumentException("Feasibility stopping criteria must be greater than 0");
        }
        this.convEps = convEps;
        return this;
    }

    public double getAbsConvEps() {
        return absConvEps;
    }

    public KnitroLoadFlowParameters setAbsConvEps(double absConvEps) {
        if (absConvEps <= 0) {
            throw new IllegalArgumentException("Absolute feasibility stopping criteria must be greater than 0");
        }
        this.absConvEps = absConvEps;
        return this;
    }

    public double getOptEps() {
        return optEps;
    }

    public KnitroLoadFlowParameters setOptEps(double optEps) {
        if (optEps <= 0) {
            throw new IllegalArgumentException("Relative optimality stopping criteria must be greater than 0");
        }
        this.optEps = optEps;
        return this;
    }

    public double getAbsOptEps() {
        return absOptEps;
    }

    public KnitroLoadFlowParameters setAbsOptEps(double absOptEps) {
        if (absOptEps <= 0) {
            throw new IllegalArgumentException("Absolute optimality stopping criteria must be greater than 0");
        }
        this.absOptEps = absOptEps;
        return this;
    }

    public double getSlackThreshold() {
        return slackThreshold;
    }

    public KnitroLoadFlowParameters setSlackThreshold(double slackThreshold) {
        if (slackThreshold <= 0) {
            throw new IllegalArgumentException("Slack value threshold must be strictly greater than 0");
        }
        this.slackThreshold = slackThreshold;
        return this;
    }

    public int getNumThreads() {
        return numThreads;
    }

    public KnitroLoadFlowParameters setNumThreads(int numThreads) {
        if (this.gradientComputationMode != 1 && numThreads != KnitroSolverParameters.DEFAULT_THREAD_NUMBER_FINITE_DIFFERENCES) {
            throw new IllegalArgumentException("Solver uses finite differences to evaluate the Jacobian. Cannot use multithreading");
        }
        this.numThreads = numThreads;
        return this;
    }

    public KnitroSolverParameters.KnitroSolverType getKnitroSolverType() {
        return knitroSolverType;
    }

    public KnitroLoadFlowParameters setKnitroSolverType(KnitroSolverParameters.KnitroSolverType knitroSolverType) {
        this.knitroSolverType = knitroSolverType;
        return this;
    }

    @Override
    public String getName() {
        return "knitro-load-flow-parameters";
    }

}

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
    private double convEps = KnitroSolverParameters.DEFAULT_STOPPING_CRITERIA;
    private boolean alwaysUpdateNetwork = KnitroSolverParameters.ALWAYS_UPDATE_NETWORK_DEFAULT_VALUE;
    private boolean checkLoadFlowSolution = KnitroSolverParameters.CHECK_LOAD_FLOW_SOLUTION_DEFAULT_VALUE;
    private KnitroSolverParameters.KnitroSolverType knitroSolverType = KnitroSolverParameters.DEFAULT_KNITRO_SOLVER_TYPE;
    private KnitroWritter knitroWritter;
    private double slackPenalV = KnitroSolverParameters.DEFAULT_WEIGHT_SLACK_V;
    private double slackPenalP = KnitroSolverParameters.DEFAULT_WEIGHT_SLACK_P;
    private double slackPenalQ = KnitroSolverParameters.DEFAULT_WEIGHT_SLACK_Q;

    public KnitroLoadFlowParameters(KnitroWritter knitroWritter) {
        this.knitroWritter = knitroWritter;
    }

    public KnitroLoadFlowParameters() {
        this.knitroWritter = new KnitroWritter("Logs.txt");
    }

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
            throw new IllegalArgumentException("Convergence stopping criteria must be greater than 0");
        }
        this.convEps = convEps;
        return this;
    }

    public KnitroSolverParameters.KnitroSolverType getKnitroSolverType() {
        return knitroSolverType;
    }

    public KnitroLoadFlowParameters setKnitroSolverType(KnitroSolverParameters.KnitroSolverType knitroSolverType) {
        this.knitroSolverType = knitroSolverType;
        return this;
    }

    public KnitroWritter getKnitroWritter() {
        return knitroWritter;
    }

    public KnitroLoadFlowParameters setKnitroWritter(KnitroWritter knitroWritter) {
        this.knitroWritter = knitroWritter;
        return this;
    }

    public double getSlackPenalV() {
        return slackPenalV;
    }

    public KnitroLoadFlowParameters setSlackPenalV(double slackPenalV) {
        this.slackPenalV = slackPenalV;
        return this;
    }

    public double getSlackPenalP() {
        return slackPenalP;
    }

    public KnitroLoadFlowParameters setSlackPenalP(double slackPenalP) {
        this.slackPenalP = slackPenalP;
        return this;
    }

    public double getSlackPenalQ() {
        return slackPenalQ;
    }

    public KnitroLoadFlowParameters setSlackPenalQ(double slackPenalQ) {
        this.slackPenalQ = slackPenalQ;
        return this;
    }

    @Override
    public String getName() {
        return "knitro-load-flow-parameters";
    }
}

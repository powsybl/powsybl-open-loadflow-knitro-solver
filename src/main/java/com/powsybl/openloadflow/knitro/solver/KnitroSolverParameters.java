/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.openloadflow.ac.solver.AcSolverParameters;
import com.powsybl.openloadflow.ac.solver.LineSearchStateVectorScaling;
import com.powsybl.openloadflow.ac.solver.MaxVoltageChangeStateVectorScaling;
import com.powsybl.openloadflow.ac.solver.StateVectorScalingMode;

import java.util.Objects;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public class KnitroSolverParameters implements AcSolverParameters {

    public static final String DEFAULT_LOG_FILE_NAME = "Logs.txt";
    public static final int DEFAULT_GRADIENT_COMPUTATION_MODE = 1; // Specifies how the Jacobian matrix is computed
    public static final int DEFAULT_GRADIENT_USER_ROUTINE = 2; // If the user chooses to pass the exact Jacobian to knitro, specifies the sparsity pattern for the Jacobian matrix.
    public static final int DEFAULT_HESSIAN_COMPUTATION_MODE = 6; // Specifies how the Hessian matrix is computed. 6 means that the Hessian is approximated using the L-BFGS method, which is a quasi-Newton method.
    public static final double DEFAULT_LOWER_VOLTAGE_BOUND = 0.5; // Lower bound for voltage magnitude
    public static final double DEFAULT_UPPER_VOLTAGE_BOUND = 1.5; // Upper bound for voltage magnitude
    public static final int DEFAULT_MAX_ITERATIONS = 400;
    public static final double DEFAULT_STOPPING_CRITERIA = Math.pow(10, -6);
    public static final StateVectorScalingMode DEFAULT_STATE_VECTOR_SCALING_MODE = StateVectorScalingMode.NONE;
    public static final boolean ALWAYS_UPDATE_NETWORK_DEFAULT_VALUE = true;
    public static final boolean CHECK_LOAD_FLOW_SOLUTION_DEFAULT_VALUE = false; // If the solver converges, launches a load flow taking into account slack variable results to check if the solution is a real load flow solution
    public static final KnitroSolverType DEFAULT_KNITRO_SOLVER_TYPE = KnitroSolverType.STANDARD;
    public static final double DEFAULT_WEIGHT_SLACK_V = 10.0;
    public static final double DEFAULT_WEIGHT_SLACK_P = 1.0;
    public static final double DEFAULT_WEIGHT_SLACK_Q = 1.0;
    public KnitroSolverType knitroSolverType = DEFAULT_KNITRO_SOLVER_TYPE;
    private StateVectorScalingMode stateVectorScalingMode = DEFAULT_STATE_VECTOR_SCALING_MODE;
    private int lineSearchStateVectorScalingMaxIteration = LineSearchStateVectorScaling.DEFAULT_MAX_ITERATION;
    private double lineSearchStateVectorScalingStepFold = LineSearchStateVectorScaling.DEFAULT_STEP_FOLD;
    private double maxVoltageChangeStateVectorScalingMaxDv = MaxVoltageChangeStateVectorScaling.DEFAULT_MAX_DV;
    private double maxVoltageChangeStateVectorScalingMaxDphi = MaxVoltageChangeStateVectorScaling.DEFAULT_MAX_DPHI;
    private int gradientComputationMode = DEFAULT_GRADIENT_COMPUTATION_MODE;
    private int gradientUserRoutine = DEFAULT_GRADIENT_USER_ROUTINE;
    private double lowerVoltageBound = DEFAULT_LOWER_VOLTAGE_BOUND;
    private double upperVoltageBound = DEFAULT_UPPER_VOLTAGE_BOUND;
    private boolean alwaysUpdateNetwork = ALWAYS_UPDATE_NETWORK_DEFAULT_VALUE;
    private boolean checkLoadFlowSolution = CHECK_LOAD_FLOW_SOLUTION_DEFAULT_VALUE;
    private int maxIterations = DEFAULT_MAX_ITERATIONS;
    private double convEps = DEFAULT_STOPPING_CRITERIA;
    private int hessianComputationMode = DEFAULT_HESSIAN_COMPUTATION_MODE;
    private String logFileName = DEFAULT_LOG_FILE_NAME;
    private KnitroWritter knitroWritter;
    private double weightSlackV = DEFAULT_WEIGHT_SLACK_V;
    private double weightSlackP = DEFAULT_WEIGHT_SLACK_P;
    private double weightSlackQ = DEFAULT_WEIGHT_SLACK_Q;

    public KnitroSolverParameters(KnitroWritter knitroWritter) {
        this.knitroWritter = knitroWritter;
    }

    public KnitroSolverParameters() {
        this.knitroWritter = new KnitroWritter("Logs.txt");
    }

    public int getGradientComputationMode() {
        return gradientComputationMode;
    }

    public KnitroSolverParameters setGradientComputationMode(int gradientComputationMode) {
        if (gradientComputationMode < 1 || gradientComputationMode > 3) {
            throw new IllegalArgumentException("Knitro gradient computation mode must be between 1 and 3");
        }
        this.gradientComputationMode = gradientComputationMode;
        return this;
    }

    public int getGradientUserRoutine() {
        return gradientUserRoutine;
    }

    public KnitroSolverParameters setGradientUserRoutine(int gradientUserRoutine) {
        if (gradientUserRoutine < 1 || gradientUserRoutine > 2) {
            throw new IllegalArgumentException("User routine must be between 1 and 2");
        }
        this.gradientUserRoutine = gradientUserRoutine;
        return this;
    }

    public int getHessianComputationMode() {
        return hessianComputationMode;
    }

    public KnitroSolverParameters setHessianComputationMode(int hessianComputationMode) {
        if (hessianComputationMode < 1 || hessianComputationMode > 7) {
            throw new IllegalArgumentException("Knitro hessian computation mode must be between 1 and 7. Please refer to the Knitro documentation for more details.");
        }
        this.hessianComputationMode = hessianComputationMode;
        return this;
    }

    public double getLowerVoltageBound() {
        return lowerVoltageBound;
    }

    public KnitroSolverParameters setLowerVoltageBound(double lowerVoltageBound) {
        if (lowerVoltageBound < 0) {
            throw new IllegalArgumentException("Realistic voltage bounds must strictly greater than 0");
        }
        this.lowerVoltageBound = lowerVoltageBound;
        return this;
    }

    public double getUpperVoltageBound() {
        return upperVoltageBound;
    }

    public KnitroSolverParameters setUpperVoltageBound(double upperVoltageBound) {
        if (upperVoltageBound < 0) {
            throw new IllegalArgumentException("Realistic voltage bounds must strictly greater than 0");
        }
        if (upperVoltageBound <= lowerVoltageBound) {
            throw new IllegalArgumentException("Realistic voltage upper bounds must greater than lower bounds");
        }
        this.upperVoltageBound = upperVoltageBound;
        return this;
    }

    public StateVectorScalingMode getStateVectorScalingMode() {
        return stateVectorScalingMode;
    }

    public KnitroSolverParameters setStateVectorScalingMode(StateVectorScalingMode stateVectorScalingMode) {
        this.stateVectorScalingMode = Objects.requireNonNull(stateVectorScalingMode);
        return this;
    }

    public boolean isAlwaysUpdateNetwork() {
        return alwaysUpdateNetwork;
    }

    public KnitroSolverParameters setAlwaysUpdateNetwork(boolean alwaysUpdateNetwork) {
        this.alwaysUpdateNetwork = alwaysUpdateNetwork;
        return this;
    }

    public boolean isCheckLoadFlowSolution() {
        return checkLoadFlowSolution;
    }

    public KnitroSolverParameters setCheckLoadFlowSolution(boolean checkLoadFlowSolution) {
        this.checkLoadFlowSolution = checkLoadFlowSolution;
        return this;
    }

    public int getLineSearchStateVectorScalingMaxIteration() {
        return lineSearchStateVectorScalingMaxIteration;
    }

    public KnitroSolverParameters setLineSearchStateVectorScalingMaxIteration(int lineSearchStateVectorScalingMaxIteration) {
        this.lineSearchStateVectorScalingMaxIteration = lineSearchStateVectorScalingMaxIteration;
        return this;

    }

    public double getLineSearchStateVectorScalingStepFold() {
        return lineSearchStateVectorScalingStepFold;
    }

    public KnitroSolverParameters setLineSearchStateVectorScalingStepFold(double lineSearchStateVectorScalingStepFold) {
        this.lineSearchStateVectorScalingStepFold = lineSearchStateVectorScalingStepFold;
        return this;
    }

    public double getMaxVoltageChangeStateVectorScalingMaxDv() {
        return maxVoltageChangeStateVectorScalingMaxDv;
    }

    public KnitroSolverParameters setMaxVoltageChangeStateVectorScalingMaxDv(double maxVoltageChangeStateVectorScalingMaxDv) {
        this.maxVoltageChangeStateVectorScalingMaxDv = maxVoltageChangeStateVectorScalingMaxDv;
        return this;
    }

    public double getMaxVoltageChangeStateVectorScalingMaxDphi() {
        return maxVoltageChangeStateVectorScalingMaxDphi;
    }

    public KnitroSolverParameters setMaxVoltageChangeStateVectorScalingMaxDphi(double maxVoltageChangeStateVectorScalingMaxDphi) {
        this.maxVoltageChangeStateVectorScalingMaxDphi = maxVoltageChangeStateVectorScalingMaxDphi;
        return this;
    }

    public int getMaxIterations() {
        return maxIterations;
    }

    public KnitroSolverParameters setMaxIterations(int maxIterations) {
        this.maxIterations = maxIterations;
        return this;
    }

    public double getConvEps() {
        return convEps;
    }

    public KnitroSolverParameters setConvEps(double convEps) {
        this.convEps = convEps;
        return this;
    }

    public KnitroSolverType getKnitroSolverType() {
        return knitroSolverType;
    }

    public KnitroSolverParameters setKnitroSolverType(KnitroSolverType knitroSolverType) {
        this.knitroSolverType = Objects.requireNonNull(knitroSolverType);
        return this;
    }

    public KnitroWritter getKnitroWritter() {
        return knitroWritter;
    }

    public KnitroSolverParameters setKnitroWritter(KnitroWritter knitroWritter) {
        this.knitroWritter = knitroWritter;
        return this;
    }

    public double getWeightSlackV() {
        return weightSlackV;
    }

    public KnitroSolverParameters setWeightSlackV(double weightSlackV) {
        this.weightSlackV = weightSlackV;
        return this;
    }

    public double getWeightSlackP() {
        return weightSlackP;
    }

    public KnitroSolverParameters setWeightSlackP(double weightSlackP) {
        this.weightSlackP = weightSlackP;
        return this;
    }

    public double getWeightSlackQ() {
        return weightSlackQ;
    }

    public KnitroSolverParameters setWeightSlackQ(double weightSlackQ) {
        this.weightSlackQ = weightSlackQ;
        return this;
    }

    @Override
    public String toString() {
        return "KnitroSolverParameters(" +
                "gradientComputationMode=" + gradientComputationMode +
                ", stoppingCriteria=" + convEps +
                ", minRealisticVoltage=" + lowerVoltageBound +
                ", maxRealisticVoltage=" + upperVoltageBound +
                ", alwaysUpdateNetwork=" + alwaysUpdateNetwork +
                ", checkLoadFlowSolution=" + checkLoadFlowSolution +
                ", maxIterations=" + maxIterations +
                ", knitroSolverType=" + knitroSolverType +
                ')';
    }

    public enum KnitroSolverType {
        STANDARD,
        RESILIENT,
        REACTIVLIMITS,
    }
}

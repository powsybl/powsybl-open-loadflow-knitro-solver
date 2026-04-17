/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.commons.config.ModuleConfig;
import com.powsybl.commons.config.PlatformConfig;
import com.powsybl.commons.extensions.AbstractExtension;
import com.powsybl.loadflow.LoadFlowParameters;

import java.util.Map;
import java.util.Optional;

/**
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public class KnitroLoadFlowParameters extends AbstractExtension<LoadFlowParameters> {

    public static final String MODULE_SPECIFIC_PARAMETERS = "open-load-flow-knitro-solver-default-parameters";

    private int gradientComputationMode = KnitroSolverParameters.DEFAULT_GRADIENT_COMPUTATION_MODE;
    private int gradientUserRoutine = KnitroSolverParameters.DEFAULT_GRADIENT_USER_ROUTINE;
    private int hessianComputationMode = KnitroSolverParameters.DEFAULT_HESSIAN_COMPUTATION_MODE;
    private double lowerVoltageBound = KnitroSolverParameters.DEFAULT_LOWER_VOLTAGE_BOUND;
    private double upperVoltageBound = KnitroSolverParameters.DEFAULT_UPPER_VOLTAGE_BOUND;
    private int maxIterations = KnitroSolverParameters.DEFAULT_MAX_ITERATIONS;
    private double relConvEps = KnitroSolverParameters.DEFAULT_RELATIVE_FEASIBILITY_STOPPING_CRITERIA;
    private double absConvEps = KnitroSolverParameters.DEFAULT_ABSOLUTE_FEASIBILITY_STOPPING_CRITERIA;
    private double relOptEps = KnitroSolverParameters.DEFAULT_RELATIVE_OPTIMALITY_STOPPING_CRITERIA;
    private double absOptEps = KnitroSolverParameters.DEFAULT_ABSOLUTE_OPTIMALITY_STOPPING_CRITERIA;
    private double slackThreshold = KnitroSolverParameters.DEFAULT_SLACK_THRESHOLD;
    private KnitroSolverParameters.SolverType knitroSolverType = KnitroSolverParameters.DEFAULT_SOLVER_TYPE;
    private int threadNumber = KnitroSolverParameters.DEFAULT_THREAD_NUMBER;

    public static final String GRADIENT_COMPUTATION_MODE_PARAM_NAME = "gradientComputationMode";
    public static final String GRADIENT_USER_ROUTINE_PARAM_NAME = "gradientUserRoutine";
    public static final String HESSIAN_COMPUTATION_MODE_PARAM_NAME = "hessianComputationMode";
    public static final String LOWER_VOLTAGE_BOUND_PARAM_NAME = "lowerVoltageBound";
    public static final String UPPER_VOLTAGE_BOUND_PARAM_NAME = "upperVoltageBound";
    public static final String MAX_ITERATIONS_PARAM_NAME = "maxIterations";
    public static final String RELATIVE_FEASIBILITY_STOPPING_CRITERIA_PARAM_NAME = "relativeFeasibilityStoppingCriteria";
    public static final String ABSOLUTE_FEASIBILITY_STOPPING_CRITERIA_PARAM_NAME = "absoluteFeasibilityStoppingCriteria";
    public static final String RELATIVE_OPTIMALITY_STOPPING_CRITERIA_PARAM_NAME = "relativeOptimalityStoppingCriteria";
    public static final String ABSOLUTE_OPTIMALITY_STOPPING_CRITERIA_PARAM_NAME = "absoluteOptimalityStoppingCriteria";
    public static final String SLACK_THRESHOLD_PARAM_NAME = "slackThreshold";
    public static final String SOLVER_TYPE_PARAM_NAME = "solverType";
    public static final String THREAD_NUMBER_PARAM_NAME = "threadNumber";

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

    public double getRelConvEps() {
        return relConvEps;
    }

    public KnitroLoadFlowParameters setRelConvEps(double relConvEps) {
        if (relConvEps <= 0) {
            throw new IllegalArgumentException("Relative feasibility stopping criteria must be strictly greater than 0");
        }
        this.relConvEps = relConvEps;
        return this;
    }

    public double getAbsConvEps() {
        return absConvEps;
    }

    public KnitroLoadFlowParameters setAbsConvEps(double absConvEps) {
        if (absConvEps <= 0) {
            throw new IllegalArgumentException("Absolute feasibility stopping criteria must be strictly greater than 0");
        }
        this.absConvEps = absConvEps;
        return this;
    }

    public double getRelOptEps() {
        return relOptEps;
    }

    public KnitroLoadFlowParameters setRelOptEps(double relOptEps) {
        if (relOptEps <= 0) {
            throw new IllegalArgumentException("Relative optimality stopping criteria must be strictly greater than 0");
        }
        this.relOptEps = relOptEps;
        return this;
    }

    public double getAbsOptEps() {
        return absOptEps;
    }

    public KnitroLoadFlowParameters setAbsOptEps(double absOptEps) {
        if (absOptEps <= 0) {
            throw new IllegalArgumentException("Absolute optimality stopping criteria must be strictly greater than 0");
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

    public KnitroSolverParameters.SolverType getKnitroSolverType() {
        return knitroSolverType;
    }

    public KnitroLoadFlowParameters setKnitroSolverType(KnitroSolverParameters.SolverType knitroSolverType) {
        this.knitroSolverType = knitroSolverType;
        return this;
    }

    public int getThreadNumber() {
        return threadNumber;
    }

    public KnitroLoadFlowParameters setThreadNumber(int threadNumber) {
        if (threadNumber < -1) {
            throw new IllegalArgumentException("Thread number must be greater than or equal to -1");
        }
        this.threadNumber = threadNumber;
        return this;
    }

    @Override
    public String getName() {
        return "knitro-load-flow-parameters";
    }

    public static KnitroLoadFlowParameters load() {
        return load(PlatformConfig.defaultConfig());
    }

    public static KnitroLoadFlowParameters load(PlatformConfig platformConfig) {
        KnitroLoadFlowParameters parameters = new KnitroLoadFlowParameters();
        return parameters.update(platformConfig);
    }

    public KnitroLoadFlowParameters update(PlatformConfig platformConfig) {
        platformConfig.getOptionalModuleConfig(MODULE_SPECIFIC_PARAMETERS)
            .ifPresent((ModuleConfig config) -> {
                config.getOptionalIntProperty(GRADIENT_COMPUTATION_MODE_PARAM_NAME)
                    .ifPresent(this::setGradientComputationMode);
                config.getOptionalIntProperty(GRADIENT_USER_ROUTINE_PARAM_NAME)
                    .ifPresent(this::setGradientUserRoutine);
                config.getOptionalIntProperty(HESSIAN_COMPUTATION_MODE_PARAM_NAME)
                    .ifPresent(this::setHessianComputationMode);
                config.getOptionalDoubleProperty(LOWER_VOLTAGE_BOUND_PARAM_NAME)
                    .ifPresent(this::setLowerVoltageBound);
                config.getOptionalDoubleProperty(UPPER_VOLTAGE_BOUND_PARAM_NAME)
                    .ifPresent(this::setUpperVoltageBound);
                config.getOptionalIntProperty(MAX_ITERATIONS_PARAM_NAME)
                    .ifPresent(this::setMaxIterations);
                config.getOptionalDoubleProperty(RELATIVE_FEASIBILITY_STOPPING_CRITERIA_PARAM_NAME)
                    .ifPresent(this::setRelConvEps);
                config.getOptionalDoubleProperty(ABSOLUTE_FEASIBILITY_STOPPING_CRITERIA_PARAM_NAME)
                    .ifPresent(this::setAbsConvEps);
                config.getOptionalDoubleProperty(RELATIVE_OPTIMALITY_STOPPING_CRITERIA_PARAM_NAME)
                    .ifPresent(this::setRelOptEps);
                config.getOptionalDoubleProperty(ABSOLUTE_OPTIMALITY_STOPPING_CRITERIA_PARAM_NAME)
                    .ifPresent(this::setAbsOptEps);
                config.getOptionalDoubleProperty(SLACK_THRESHOLD_PARAM_NAME)
                    .ifPresent(this::setSlackThreshold);
                config.getOptionalEnumProperty(SOLVER_TYPE_PARAM_NAME, KnitroSolverParameters.SolverType.class)
                    .ifPresent(this::setKnitroSolverType);
                config.getOptionalIntProperty(THREAD_NUMBER_PARAM_NAME)
                    .ifPresent(this::setThreadNumber);
            });
        return this;
    }

    public KnitroLoadFlowParameters update(Map<String, String> properties) {
        Optional.ofNullable(properties.get(GRADIENT_COMPUTATION_MODE_PARAM_NAME))
            .ifPresent(prop -> this.setGradientComputationMode(Integer.parseInt(prop)));
        Optional.ofNullable(properties.get(GRADIENT_USER_ROUTINE_PARAM_NAME))
            .ifPresent(prop -> this.setGradientUserRoutine(Integer.parseInt(prop)));
        Optional.ofNullable(properties.get(HESSIAN_COMPUTATION_MODE_PARAM_NAME))
            .ifPresent(prop -> this.setHessianComputationMode(Integer.parseInt(prop)));
        Optional.ofNullable(properties.get(LOWER_VOLTAGE_BOUND_PARAM_NAME))
            .ifPresent(prop -> this.setLowerVoltageBound(Double.parseDouble(prop)));
        Optional.ofNullable(properties.get(UPPER_VOLTAGE_BOUND_PARAM_NAME))
            .ifPresent(prop -> this.setUpperVoltageBound(Double.parseDouble(prop)));
        Optional.ofNullable(properties.get(MAX_ITERATIONS_PARAM_NAME))
            .ifPresent(prop -> this.setMaxIterations(Integer.parseInt(prop)));
        Optional.ofNullable(properties.get(RELATIVE_FEASIBILITY_STOPPING_CRITERIA_PARAM_NAME))
            .ifPresent(prop -> this.setRelConvEps(Double.parseDouble(prop)));
        Optional.ofNullable(properties.get(ABSOLUTE_FEASIBILITY_STOPPING_CRITERIA_PARAM_NAME))
            .ifPresent(prop -> this.setAbsConvEps(Double.parseDouble(prop)));
        Optional.ofNullable(properties.get(RELATIVE_OPTIMALITY_STOPPING_CRITERIA_PARAM_NAME))
            .ifPresent(prop -> this.setRelOptEps(Double.parseDouble(prop)));
        Optional.ofNullable(properties.get(ABSOLUTE_OPTIMALITY_STOPPING_CRITERIA_PARAM_NAME))
            .ifPresent(prop -> this.setAbsConvEps(Double.parseDouble(prop)));
        Optional.ofNullable(properties.get(SLACK_THRESHOLD_PARAM_NAME))
            .ifPresent(prop -> this.setSlackThreshold(Double.parseDouble(prop)));
        Optional.ofNullable(properties.get(SOLVER_TYPE_PARAM_NAME))
            .ifPresent(prop -> this.setKnitroSolverType(KnitroSolverParameters.SolverType.valueOf(prop)));
        Optional.ofNullable(properties.get(THREAD_NUMBER_PARAM_NAME))
            .ifPresent(prop -> this.setThreadNumber(Integer.parseInt(prop)));
        return this;
    }

}

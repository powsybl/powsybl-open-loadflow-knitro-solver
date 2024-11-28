package knitroextension;

import com.powsybl.commons.extensions.AbstractExtension;
import com.powsybl.loadflow.LoadFlowParameters;
import java.util.Objects;

public class ExternalSolverExtensionParameters extends AbstractExtension<LoadFlowParameters> {

    private int gradientComputationMode = KnitroSolverParameters.DEFAULT_GRADIENT_COMPUTATION_MODE;
    private int gradientUserRoutine = KnitroSolverParameters.DEFAULT_GRADIENT_USER_ROUTINE;
    private double lowerVoltageBound = KnitroSolverParameters.DEFAULT_LOWER_VOLTAGE_BOUND;
    private double upperVoltageBound = KnitroSolverParameters.DEFAULT_UPPER_VOLTAGE_BOUND;
    private int maxIterations = KnitroSolverParameters.DEFAULT_MAX_ITERATIONS;
    private KnitroSolverStoppingCriteria stoppingCriteria = new KnitroSolverStoppingCriteria() {
    };

    public int getGradientComputationMode() {
        return gradientComputationMode;
    }

    public ExternalSolverExtensionParameters setGradientComputationMode(int gradientComputationMode) {
        if (gradientComputationMode < 1 || gradientComputationMode > 3) {
            throw new IllegalArgumentException("User routine must be between 1 and 3");
        }
        this.gradientComputationMode = gradientComputationMode;
        return this;
    }

    public int getGradientUserRoutine() {
        return gradientUserRoutine;
    }

    public ExternalSolverExtensionParameters setGradientUserRoutine(int gradientUserRoutine) {
        if (gradientUserRoutine < 1 || gradientUserRoutine > 2) {
            throw new IllegalArgumentException("User routine must be between 1 and 2");
        }
        this.gradientUserRoutine = gradientUserRoutine;
        return this;
    }

    public double getLowerVoltageBound() {
        return lowerVoltageBound;
    }

    public ExternalSolverExtensionParameters setLowerVoltageBound(double lowerVoltageBound) {
        if (lowerVoltageBound < 0) {
            throw new IllegalArgumentException("Realistic voltage bounds must strictly greater than 0");
        }
        this.lowerVoltageBound = lowerVoltageBound;
        return this;
    }

    public double getUpperVoltageBound() {
        return upperVoltageBound;
    }

    public ExternalSolverExtensionParameters setUpperVoltageBound(double upperVoltageBound) {
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

    public ExternalSolverExtensionParameters setMaxIterations(int maxIterations) {
        if (maxIterations < 0 ) {
            throw new IllegalArgumentException("Max iterations parameter must be greater than 0");
        }
        this.maxIterations = maxIterations;
        return this;
    }


    public KnitroSolverStoppingCriteria getStoppingCriteria() {
        return stoppingCriteria;
    }

    public ExternalSolverExtensionParameters setStoppingCriteria(KnitroSolverStoppingCriteria stoppingCriteria) {
        this.stoppingCriteria = Objects.requireNonNull(stoppingCriteria);
        return this;
    }

    @Override
    public String getName() {
        return "external-solver-extension";
    }

}

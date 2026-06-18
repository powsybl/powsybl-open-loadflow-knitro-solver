
package com.powsybl.openloadflow.knitro.solver;

public final class SlackFeasibility {

    private SlackFeasibility() { }

    private static final double V_MIN_PU = 0.8;
    private static final double V_MAX_PU = 1.2;

    //
    public static boolean isGenFeasible(double newValue, double min, double max) {
        return newValue >= min && newValue <= max;
    }

    //A load is feasible if the post-slack values stays >= 0
    public static boolean isLoadFeasible(double slack, double loadTarget) {
        return slack + loadTarget >= 0;
    }

    // The voltage value is acceptable if the post-slack values stays between V_MIN_PU and V_MAX_PU
    public static boolean isFeasibleV(double slack, double vRef) {
        double v = slack + vRef;
        return v >= V_MIN_PU && v <= V_MAX_PU;
    }
}

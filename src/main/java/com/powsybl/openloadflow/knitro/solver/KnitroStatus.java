/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.openloadflow.ac.solver.AcSolverStatus;
import org.apache.commons.lang3.Range;

import java.util.Arrays;

/**
 * Enum representing all possible status returned by Knitro optimization solver.
 * These are grouped either individually or by ranges, and mapped to corresponding {@link AcSolverStatus} values.
 *
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 */
public enum KnitroStatus {

    /**
     * Successful convergence to a local optimum.
     */
    CONVERGED_TO_LOCAL_OPTIMUM(0, 0, AcSolverStatus.CONVERGED),

    /**
     * Converged to a feasible but not necessarily optimal solution.
     */
    CONVERGED_TO_FEASIBLE_APPROXIMATE_SOLUTION(-199, -100, AcSolverStatus.CONVERGED),

    /**
     * Solver terminated at an infeasible point.
     */
    TERMINATED_AT_INFEASIBLE_POINT(-299, -200, AcSolverStatus.SOLVER_FAILED),

    /**
     * The problem was detected as unbounded.
     */
    PROBLEM_UNBOUNDED(-399, -300, AcSolverStatus.SOLVER_FAILED),

    /**
     * Optimization stopped due to reaching iteration or time limits.
     */
    TERMINATED_DUE_TO_PRE_DEFINED_LIMIT(-499, -400, AcSolverStatus.MAX_ITERATION_REACHED),

    /**
     * Failure in a user-defined callback function.
     */
    CALLBACK_ERROR(-500, -500, AcSolverStatus.SOLVER_FAILED),

    /**
     * Internal LP solver failure in active-set method.
     */
    LP_SOLVER_ERROR(-501, -501, AcSolverStatus.SOLVER_FAILED),

    /**
     * Evaluation failure (e.g., division by zero or invalid sqrt).
     */
    EVALUATION_ERROR(-502, -502, AcSolverStatus.SOLVER_FAILED),

    /**
     * Insufficient memory to solve the problem.
     */
    OUT_OF_MEMORY(-503, -503, AcSolverStatus.SOLVER_FAILED),

    /**
     * Solver was stopped manually by the user.
     */
    USER_TERMINATION(-504, -504, AcSolverStatus.SOLVER_FAILED),

    /**
     * File open error when trying to read input.
     */
    INPUT_FILE_ERROR(-505, -505, AcSolverStatus.SOLVER_FAILED),

    /**
     * Modeling error: invalid variable/constraint setup.
     */
    MODEL_DEFINITION_ERROR(-530, -506, AcSolverStatus.SOLVER_FAILED),

    /**
     * Internal Knitro error – contact support.
     */
    INTERNAL_ERROR(-600, -600, AcSolverStatus.SOLVER_FAILED),

    /**
     * Fallback for unknown status codes.
     */
    UNKNOWN_STATUS(Integer.MIN_VALUE, Integer.MAX_VALUE, AcSolverStatus.SOLVER_FAILED);

    private final Range<Integer> codeRange;
    private final AcSolverStatus mappedStatus;

    /**
     * Constructs a KnitroStatus with a range of status codes and the associated AcSolverStatus.
     *
     * @param min          The minimum status code value (inclusive).
     * @param max          The maximum status code value (inclusive).
     * @param mappedStatus The corresponding AcSolverStatus.
     */
    KnitroStatus(int min, int max, AcSolverStatus mappedStatus) {
        this.codeRange = Range.of(min, max);
        this.mappedStatus = mappedStatus;
    }

    /**
     * Returns the KnitroStatus corresponding to the given status code.
     *
     * @param statusCode the status code returned by Knitro
     * @return the matching KnitroStatus enum constant, or UNKNOWN_STATUS if unknown
     */
    public static KnitroStatus fromStatusCode(int statusCode) {
        return Arrays.stream(KnitroStatus.values())
                .filter(status -> status.codeRange.contains(statusCode))
                .findFirst()
                .orElse(UNKNOWN_STATUS);
    }

    /**
     * Returns the {@link AcSolverStatus} associated with this KnitroStatus.
     *
     * @return the corresponding AcSolverStatus
     */
    public AcSolverStatus toAcSolverStatus() {
        return mappedStatus;
    }
}

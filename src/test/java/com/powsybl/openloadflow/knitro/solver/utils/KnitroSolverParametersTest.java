/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver.utils;

import com.powsybl.openloadflow.knitro.solver.KnitroLoadFlowParameters;
import com.powsybl.openloadflow.knitro.solver.KnitroSolverParameters;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

/**
 * @author Pierre Arvy {@literal <pierre.arvy at artelys.com>}
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
class KnitroSolverParametersTest {

    @Test
    void testGradientComputationMode() {
        KnitroSolverParameters parametersKnitro = new KnitroSolverParameters();
        // default value
        assertEquals(1, parametersKnitro.getGradientComputationMode());

        // set other value
        parametersKnitro.setGradientComputationMode(3);
        assertEquals(3, parametersKnitro.getGradientComputationMode());

        // wrong values
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> parametersKnitro.setGradientComputationMode(0));
        assertEquals("Knitro gradient computation mode must be between 1 and 3", e.getMessage());
        IllegalArgumentException e2 = assertThrows(IllegalArgumentException.class, () -> parametersKnitro.setGradientComputationMode(4));
        assertEquals("Knitro gradient computation mode must be between 1 and 3", e2.getMessage());
    }

    @Test
    void testGradientUserRoutineIntegrity() {
        KnitroSolverParameters parametersKnitro = new KnitroSolverParameters();
        // default value
        assertEquals(2, parametersKnitro.getGradientUserRoutine());

        // set other value
        parametersKnitro.setGradientUserRoutine(1);
        assertEquals(1, parametersKnitro.getGradientUserRoutine());

        // wrong values
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> parametersKnitro.setGradientUserRoutine(0));
        assertEquals("User routine must be between 1 and 2", e.getMessage());
        IllegalArgumentException e2 = assertThrows(IllegalArgumentException.class, () -> parametersKnitro.setGradientUserRoutine(3));
        assertEquals("User routine must be between 1 and 2", e2.getMessage());
    }

    @Test
    void testVoltageBoundsIntegrity() {
        KnitroSolverParameters parametersKnitro = new KnitroSolverParameters();
        // default values
        assertEquals(0.5, parametersKnitro.getLowerVoltageBound());
        assertEquals(1.5, parametersKnitro.getUpperVoltageBound());

        // check other values
        parametersKnitro.setLowerVoltageBound(0.95);
        parametersKnitro.setUpperVoltageBound(1.05);
        assertEquals(0.95, parametersKnitro.getLowerVoltageBound());
        assertEquals(1.05, parametersKnitro.getUpperVoltageBound());

        // wrong values
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> parametersKnitro.setLowerVoltageBound(-1.0));
        assertEquals("Realistic voltage bounds must strictly greater than 0", e.getMessage());
        IllegalArgumentException e2 = assertThrows(IllegalArgumentException.class, () -> parametersKnitro.setUpperVoltageBound(-1.0));
        assertEquals("Realistic voltage bounds must strictly greater than 0", e2.getMessage());
        IllegalArgumentException e3 = assertThrows(IllegalArgumentException.class, () -> parametersKnitro.setUpperVoltageBound(0.90));
        assertEquals("Realistic voltage upper bounds must greater than lower bounds", e3.getMessage());
    }

    @Test
    void testMaxIterationsIntegrity() {
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters();
        // check default max iterations value
        assertEquals(200, knitroLoadFlowParameters.getMaxIterations());

        // set other value
        knitroLoadFlowParameters.setMaxIterations(400);
        assertEquals(400, knitroLoadFlowParameters.getMaxIterations());

        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setMaxIterations(-1));
        assertEquals("Max iterations parameter must be greater than 0", e.getMessage());
    }

    @Test
    void testRelConvEpsIntegrity() {
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters(); // set gradient computation mode
        // check default conv value
        assertEquals(Math.pow(10, -6), knitroLoadFlowParameters.getRelConvEps());

        // set other value
        knitroLoadFlowParameters.setRelConvEps(Math.pow(10, -2));
        assertEquals(Math.pow(10, -2), knitroLoadFlowParameters.getRelConvEps());

        // wrong values
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setRelConvEps(-1.0));
        assertEquals("Relative feasibility stopping criteria must be strictly greater than 0", e.getMessage());
        IllegalArgumentException e2 = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setRelConvEps(0));
        assertEquals("Relative feasibility stopping criteria must be strictly greater than 0", e2.getMessage());
    }

    @Test
    void testAbsConvEpsIntegrity() {
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters(); // set gradient computation mode
        // check default absConvEps value
        assertEquals(Math.pow(10, -3), knitroLoadFlowParameters.getAbsConvEps());

        // check setter and getter
        knitroLoadFlowParameters.setAbsConvEps(Math.pow(10, -2));
        assertEquals(Math.pow(10, -2), knitroLoadFlowParameters.getAbsConvEps());

        // check out-of-bounds values
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setAbsConvEps(-1.0));
        assertEquals("Absolute feasibility stopping criteria must be strictly greater than 0", e.getMessage());
        IllegalArgumentException e2 = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setAbsConvEps(0));
        assertEquals("Absolute feasibility stopping criteria must be strictly greater than 0", e2.getMessage());
    }

    @Test
    void testRelOptEpsIntegrity() {
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters(); // set gradient computation mode
        // check default conv value
        assertEquals(Math.pow(10, -6), knitroLoadFlowParameters.getRelOptEps());

        // check other value
        knitroLoadFlowParameters.setRelOptEps(Math.pow(10, -2));
        assertEquals(Math.pow(10, -2), knitroLoadFlowParameters.getRelOptEps());

        // check out-of-bounds values
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setRelOptEps(-1.0));
        assertEquals("Relative optimality stopping criteria must be strictly greater than 0", e.getMessage());
        IllegalArgumentException e2 = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setRelOptEps(0));
        assertEquals("Relative optimality stopping criteria must be strictly greater than 0", e2.getMessage());
    }

    @Test
    void testAbsOptEpsIntegrity() {
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters(); // set gradient computation mode
        // check default value
        assertEquals(Math.pow(10, -3), knitroLoadFlowParameters.getAbsOptEps());

        // check other value
        knitroLoadFlowParameters.setAbsOptEps(Math.pow(10, -2));
        assertEquals(Math.pow(10, -2), knitroLoadFlowParameters.getAbsOptEps());

        // check out-of-bounds values
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setAbsOptEps(-1.0));
        assertEquals("Absolute optimality stopping criteria must be strictly greater than 0", e.getMessage());
        IllegalArgumentException e2 = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setAbsOptEps(0));
        assertEquals("Absolute optimality stopping criteria must be strictly greater than 0", e2.getMessage());
    }

    @Test
    void testSlackThresholdIntegrity() {
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters();

        // check default conv value
        assertEquals(Math.pow(10, -6), knitroLoadFlowParameters.getSlackThreshold());

        // check other value
        knitroLoadFlowParameters.setSlackThreshold(Math.pow(10, -2));
        assertEquals(Math.pow(10, -2), knitroLoadFlowParameters.getSlackThreshold());

        // check out-of-bounds values
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setSlackThreshold(-1.0));
        assertEquals("Slack value threshold must be strictly greater than 0", e.getMessage());
        IllegalArgumentException e2 = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setSlackThreshold(0));
        assertEquals("Slack value threshold must be strictly greater than 0", e2.getMessage());
    }

    @Test
    void testHessianComputationModeIntegrity() {
        KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters();

        // check default value
        assertEquals(6, knitroLoadFlowParameters.getHessianComputationMode());

        // check other value
        knitroLoadFlowParameters.setHessianComputationMode(2);
        assertEquals(2, knitroLoadFlowParameters.getHessianComputationMode());

        // wrong values
        IllegalArgumentException e = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setHessianComputationMode(0));
        assertEquals("Hessian computation mode must be between 1 and 7", e.getMessage());
        IllegalArgumentException e2 = assertThrows(IllegalArgumentException.class, () -> knitroLoadFlowParameters.setHessianComputationMode(8));
        assertEquals("Hessian computation mode must be between 1 and 7", e2.getMessage());
    }

    @Test
    void testToString() {
        KnitroSolverParameters parameters = new KnitroSolverParameters();
        assertEquals("KnitroSolverParameters(solverType=STANDARD, gradientComputationMode=1, gradientUserRoutine=2, hessianComputationMode=6, relativeFeasibilityStoppingCriteria=1.0E-6, absoluteFeasibilityStoppingCriteria=0.001, relativeOptimalityStoppingCriteria=1.0E-6, absoluteOptimalityStoppingCriteria=0.001, optimalityStoppingCriteria=1.0E-6, slackThreshold=1.0E-6, minRealisticVoltage=0.5, maxRealisticVoltage=1.5, alwaysUpdateNetwork=false, maxIterations=200)",
                parameters.toString());
    }

}

/**
 * Copyright (c) 2025, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import org.junit.jupiter.api.Test;

import static com.powsybl.openloadflow.knitro.solver.SlackFeasibility.isFeasibleV;
import static com.powsybl.openloadflow.knitro.solver.SlackFeasibility.isGenFeasible;
import static com.powsybl.openloadflow.knitro.solver.SlackFeasibility.isLoadFeasible;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Salomé Lavine {@literal <salome.lavine at artelys.com>}
 */
class SlackFeasibilityTest {

    private static final double V_MIN_PU = 0.8;
    private static final double V_MAX_PU = 1.2;

    @Test
    void testIsGenFeasible() {
        assertTrue(isGenFeasible(5.0, 0.0, 10.0));   // inside range 5.0 in [0.0, 10.0]
        assertFalse(isGenFeasible(-1.0, 0.0, 10.0)); // below min -1.0 not in [0.0, 10.0]
        assertFalse(isGenFeasible(11.0, 0.0, 10.0)); // above max 11.0 not in [0.0, 10.0]
        assertTrue(isGenFeasible(5.0, 5.0, 10.0));   // lower boundary 5.0 in [5.0, 10.0]
        assertTrue(isGenFeasible(10.0, 5.0, 10.0));  // upper boundary 10.0 in [5.0, 10.0]
    }

    @Test
    void testIsLoadFeasible() {
        assertTrue(isLoadFeasible(0.0, 5.0));   // slack + load = 5.0 >= 0
        assertFalse(isLoadFeasible(-6.0, 5.0)); // slack + load = -1.0 < 0
        assertTrue(isLoadFeasible(0.0, 0.0));   // slack + load = 0.0 >= 0 (lower boundary)
    }

    @Test
    void testIsFeasibleV() {
        assertTrue(isFeasibleV(0.0, 1.0, V_MIN_PU, V_MAX_PU));    // 1.0 in [0.8, 1.2]
        assertFalse(isFeasibleV(0.5, 1.0, V_MIN_PU, V_MAX_PU));   // upper 1.5 > 1.2
        assertTrue(isFeasibleV(0.2, 1.0, V_MIN_PU, V_MAX_PU));    // upper boundary 1.2 <= 1.2
        assertFalse(isFeasibleV(-0.5, 1.0, V_MIN_PU, V_MAX_PU));  // lower 0.5 < 0.8
        assertTrue(isFeasibleV(-0.2, 1.0, V_MIN_PU, V_MAX_PU));   // lower boundary 0.8 >= 0.8
    }

    @Test
    void testIsFeasibleVUsesGivenBounds() {
        // The same post-slack voltage is feasible or not depending on the configured bounds: with the solver
        // defaults (0.5 / 1.5) a 1.3 p.u. voltage is accepted, with a tighter setting it is not.
        assertTrue(isFeasibleV(0.3, 1.0,
                KnitroSolverParameters.DEFAULT_LOWER_VOLTAGE_BOUND, KnitroSolverParameters.DEFAULT_UPPER_VOLTAGE_BOUND));
        assertFalse(isFeasibleV(0.3, 1.0, 0.9, 1.1));
    }
}

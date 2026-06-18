
package com.powsybl.openloadflow.knitro.solver;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static com.powsybl.openloadflow.knitro.solver.SlackFeasibility.*;

class SlackFeasibilityTest {

    @Test
    void testIsGenFeasible() {
        assertTrue(isGenFeasible(5.0, 0.0, 10.0));   // inside range
        assertFalse(isGenFeasible(-1.0, 0.0, 10.0)); // below min
        assertFalse(isGenFeasible(11.0, 0.0, 10.0)); // above max
    }

    @Test
    void testIsLoadFeasible() {
        assertTrue(isLoadFeasible(0.0, 5.0));   // slack + load = 5 >= 0
        assertFalse(isLoadFeasible(-6.0, 5.0)); // slack + load = -1 < 0
    }

    @Test
    void testIsFeasibleV() {
        // assuming V_MIN_PU = 0.8 and V_MAX_PU = 1.2
        assertTrue(isFeasibleV(0.0, 1.0));    // 1.0 in [0.5, 1.5]
        assertFalse(isFeasibleV(0.5, 1.0));   // upper 1.3 > 1.2
        assertFalse(isFeasibleV(-0.5, 1.0));  // lower 0.5 < 0.8
    }
}

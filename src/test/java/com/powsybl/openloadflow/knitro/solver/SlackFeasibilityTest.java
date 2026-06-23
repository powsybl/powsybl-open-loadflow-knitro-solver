
package com.powsybl.openloadflow.knitro.solver;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
import static com.powsybl.openloadflow.knitro.solver.SlackFeasibility.*;
/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Salomé Lavine {@literal <salome.lavine at artelys.com>}
 */

class SlackFeasibilityTest {

    @Test
    void testIsGenFeasible() {
        assertTrue(isGenFeasible(5.0, 0.0, 10.0));   // inside range 5.0 in [0.0, 10.0]
        assertFalse(isGenFeasible(-1.0, 0.0, 10.0)); // below min -1.0 not in [0.0, 10.0]
        assertFalse(isGenFeasible(11.0, 0.0, 10.0)); // above max 11.0 not in [0.0, 10.0]
        assertTrue(isGenFeasible(5.0, 5.0, 10.0));  // lower boundary 5.0 in [0.0, 10.0]
        assertTrue(isGenFeasible(10.0, 5.0, 10.0));  // upper boundary 10.0 in [0.0, 10.0]
    }

    @Test
    void testIsLoadFeasible() {
        assertTrue(isLoadFeasible(0.0, 5.0));   // slack + load = 5.0 >= 0
        assertFalse(isLoadFeasible(-6.0, 5.0)); // slack + load = -1.0 < 0
        assertTrue(isLoadFeasible(0.0, 0.0));   // slack + load = 0.0 >= 0 (lower boundary)
    }

    @Test
    void testIsFeasibleV() {
        // assuming V_MIN_PU = 0.8 and V_MAX_PU = 1.2
        assertTrue(isFeasibleV(0.0, 1.0));    // 1.0 in [0.8, 1.2]
        assertFalse(isFeasibleV(0.5, 1.0));   // upper 1.5 >= 1.2
        assertTrue(isFeasibleV(0.2, 1.0));    // upper boundary 1.2 >= 1.2
        assertFalse(isFeasibleV(-0.5, 1.0));  // lower 0.5 <= 0.8
        assertTrue(isFeasibleV(-0.2, 1.0));   // lower boundary 0.8 <= 0.8
    }
}

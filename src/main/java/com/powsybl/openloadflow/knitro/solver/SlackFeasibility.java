/**
 * Copyright (c) 2025, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Salomé Lavine {@literal <salome.lavine at artelys.com>}
 */
public final class SlackFeasibility {

    private SlackFeasibility() { }

    // A generator constraint is feasible if the post-slack value is contained in its range
    public static boolean isGenFeasible(double newValue, double min, double max) {
        return newValue >= min && newValue <= max;
    }

    // A load is feasible if the post-slack value stays >= 0
    public static boolean isLoadFeasible(double slack, double loadTarget) {
        return slack + loadTarget >= 0;
    }

    /**
     * The voltage value is acceptable if the post-slack value stays within the voltage bounds actually
     * applied to the V variables of the problem, i.e. the ones configured on {@link KnitroSolverParameters}.
     *
     * @param slack  The slack value, in p.u.
     * @param vRef   The voltage target the slack applies to, in p.u.
     * @param vMinPu The lower voltage bound of the problem, in p.u.
     * @param vMaxPu The upper voltage bound of the problem, in p.u.
     * @return true if the post-slack voltage stays within [vMinPu, vMaxPu].
     */
    public static boolean isFeasibleV(double slack, double vRef, double vMinPu, double vMaxPu) {
        double v = slack + vRef;
        return v >= vMinPu && v <= vMaxPu;
    }
}

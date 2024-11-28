/**
 * Copyright (c) 2024, Artelys (https://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package knitroextension;

/**
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 */
public class DefaultKnitroSolverStoppingCriteria implements KnitroSolverStoppingCriteria {

    public final double convEpsPerEq;

    public DefaultKnitroSolverStoppingCriteria() {
        this(KnitroSolverStoppingCriteria.DEFAULT_CONV_EPS_PER_EQ);
    }

    public DefaultKnitroSolverStoppingCriteria(double convEpsPerEq) {
        this.convEpsPerEq = convEpsPerEq;
    }

    public double getConvEpsPerEq() {
        return convEpsPerEq;
    }

}

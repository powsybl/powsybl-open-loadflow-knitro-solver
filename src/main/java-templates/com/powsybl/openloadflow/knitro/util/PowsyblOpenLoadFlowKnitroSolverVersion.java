/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.util;

import com.google.auto.service.AutoService;
import com.powsybl.tools.*;

/**
 * @author Damien Jeandemange {@literal <damien.jeandemange at artelys.com>}
 */
@AutoService(Version.class)
public class PowsyblOpenLoadFlowKnitroSolverVersion extends AbstractVersion {

    public PowsyblOpenLoadFlowKnitroSolverVersion() {
        super("powsybl-open-loadflow-knitro-solver", "${project.version}", "${buildNumber}", "${scmBranch}", Long.parseLong("${timestamp}"));
    }
}

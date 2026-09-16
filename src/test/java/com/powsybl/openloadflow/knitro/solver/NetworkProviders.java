/**
 * Copyright (c) 2025, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.ieeecdf.converter.IeeeCdfNetworkFactory;
import com.powsybl.iidm.network.Network;
import com.powsybl.iidm.serde.XMLExporter;

import java.nio.file.Path;
import java.util.Properties;
import java.util.stream.Stream;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public final class NetworkProviders {
    public static final String DEFAULT_OUTPUT_DIR = "./Outputs/";

    private NetworkProviders() {
        throw new UnsupportedOperationException("Utility Class");
    }

    public static Stream<NetworkPair> provideI3ENetworks() {
        return Stream.of(
                new NetworkPair(IeeeCdfNetworkFactory.create14(), IeeeCdfNetworkFactory.create14(), IeeeCdfNetworkFactory.create14(), "ieee14"),
                new NetworkPair(IeeeCdfNetworkFactory.create30(), IeeeCdfNetworkFactory.create30(), IeeeCdfNetworkFactory.create30(), "ieee30")
        );
    }

    public static Stream<NetworkPair> provideI3E30Networks() {
        return Stream.of(
                new NetworkPair(IeeeCdfNetworkFactory.create30(), IeeeCdfNetworkFactory.create30(), IeeeCdfNetworkFactory.create30(), "ieee30")
        );
    }

    public static void writeXML(Network network, String name) {
        Properties properties = new Properties();
        properties.put(XMLExporter.VERSION, "1.12");
        Path path = Path.of(DEFAULT_OUTPUT_DIR, name);
        network.write("XIIDM", properties, path);
    }

    public record NetworkPair(Network rknNetwork, Network nrNetwork, Network dcNetwork, String baseFilename) {

    }
}

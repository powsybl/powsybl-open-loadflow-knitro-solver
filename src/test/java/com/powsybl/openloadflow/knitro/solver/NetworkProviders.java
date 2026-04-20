
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
    public static final String DATA_DIR = "data";
    public static final String DEFAULT_OUTPUT_DIR = "./Outputs/";
    public static final String RTE6515_INSTANCE = "rte6515.xiidm";
    public static final String RTE1888_INSTANCE = "rte1888.xiidm";

    private NetworkProviders() {
        throw new UnsupportedOperationException("Classe utilitaire");
    }

    public static Stream<NetworkPair> provideRteNetworks() {
        Path fileNameRte6515 = Path.of(DATA_DIR, RTE6515_INSTANCE);
        Path fileNameRte1888 = Path.of(DATA_DIR, RTE1888_INSTANCE);
        return Stream.of(
                new NetworkPair(Network.read(fileNameRte1888).getNetwork(), Network.read(fileNameRte1888).getNetwork(), Network.read(fileNameRte1888).getNetwork(), "rte1888"),
                new NetworkPair(Network.read(fileNameRte6515).getNetwork(), Network.read(fileNameRte6515).getNetwork(), Network.read(fileNameRte6515).getNetwork(), "rte6515")
        );
    }

    public static Stream<NetworkPair> provideI3ENetworks() {
        return Stream.of(
                new NetworkPair(IeeeCdfNetworkFactory.create14(), IeeeCdfNetworkFactory.create14(), IeeeCdfNetworkFactory.create14(), "ieee14"),
                new NetworkPair(IeeeCdfNetworkFactory.create30(), IeeeCdfNetworkFactory.create30(), IeeeCdfNetworkFactory.create30(), "ieee30"),
                new NetworkPair(IeeeCdfNetworkFactory.create118(), IeeeCdfNetworkFactory.create118(), IeeeCdfNetworkFactory.create118(), "ieee118"),
                new NetworkPair(IeeeCdfNetworkFactory.create300(), IeeeCdfNetworkFactory.create300(), IeeeCdfNetworkFactory.create300(), "ieee300")
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

package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.ieeecdf.converter.IeeeCdfNetworkFactory;
import com.powsybl.iidm.network.Network;
import com.powsybl.iidm.network.TopologyKind;
import com.powsybl.iidm.network.VoltageLevel;
import com.powsybl.iidm.serde.XMLExporter;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Properties;
import java.util.stream.Stream;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public final class NetworkProviders {
    public static final String CONFIDENTIAL_DATA_DIR = "C:/Users/parvy/Downloads/";
    public static final String CONFIDENTIAL_DATA_DIR_BUS_BREAKER = "../../data_confidential_bus_breaker/";
    public static final String DEFAULT_OUTPUT_DIR = "./outputs/";
    public static final String HU_INSTANCE = "HU/20220226T2330Z_1D_002/init.xiidm";
    public static final String ES_INSTANCE = "20250830T1330Z_1D_ES_006.xiidm";
    public static final String TYNDP_INSTANCE = "CGM_TYNDP22.xiidm";
    public static final String RTE6515_INSTANCE = "rte6515.xiidm";
    public static final String RTE1888_INSTANCE = "rte1888.xiidm";

    private NetworkProviders() {

    }

    public static Stream<NetworkPair> provideRteNetworks() {
        Path fileNameRte6515 = Path.of(CONFIDENTIAL_DATA_DIR, RTE6515_INSTANCE);
        Path fileNameRte1888 = Path.of(CONFIDENTIAL_DATA_DIR, RTE1888_INSTANCE);
        return Stream.of(
                new NetworkPair(Network.read(fileNameRte1888).getNetwork(), Network.read(fileNameRte1888).getNetwork(), "rte1888"),
                new NetworkPair(Network.read(fileNameRte6515).getNetwork(), Network.read(fileNameRte6515).getNetwork(), "rte6515")
        );
    }

    public static Stream<NetworkPair> provideI3ENetworks() {
        return Stream.of(
                new NetworkPair(IeeeCdfNetworkFactory.create14(), IeeeCdfNetworkFactory.create14(), "ieee14"),
                new NetworkPair(IeeeCdfNetworkFactory.create30(), IeeeCdfNetworkFactory.create30(), "ieee30"),
                new NetworkPair(IeeeCdfNetworkFactory.create118(), IeeeCdfNetworkFactory.create118(), "ieee118"),
                new NetworkPair(IeeeCdfNetworkFactory.create300(), IeeeCdfNetworkFactory.create300(), "ieee300")
        );
    }

    public static Stream<NetworkPair> provideHUNetworks(String dir) {
        Path baseDir = Path.of(dir, "HU");
        String initFileName = "init.xiidm";

        try (Stream<Path> cases = Files.list(baseDir)) {
            List<NetworkPair> networkPairs = cases.filter(Files::isDirectory)
                    .map(subDir -> subDir.resolve(initFileName))
                    .filter(Files::exists)
                    .map(initPath -> {
                        Network nrNetwork = Network.read(initPath).getNetwork();
                        Network rknNetwork = Network.read(initPath).getNetwork();
                        String name = initPath.getParent().getFileName().toString();
                        return new NetworkPair(rknNetwork, nrNetwork, name);
                    })
                    .toList();
            return networkPairs.stream();

        } catch (Exception e) {
            throw new RuntimeException("Failed to load HU real network cases", e);
        }
    }

    public static Stream<NetworkPair> provideNodeBreakerHUNetworks() {
        return provideHUNetworks(CONFIDENTIAL_DATA_DIR);
    }

    public static Stream<NetworkPair> provideBusBreakerHUNetworks() {
        return provideHUNetworks(CONFIDENTIAL_DATA_DIR_BUS_BREAKER);
    }

    public static void writeXML(Network network, String name) {
        Properties properties = new Properties();
        properties.put(XMLExporter.VERSION, "1.12");
        Path path = Path.of(DEFAULT_OUTPUT_DIR, name);
        network.write("XIIDM", properties, path);
    }

    public static void exportNetworkAsBusBreakerTopology(Path initPath, Path endPath) {
        Network network = Network.read(initPath).getNetwork();
        for (VoltageLevel vl : network.getVoltageLevels()) {
            vl.convertToTopology(TopologyKind.BUS_BREAKER);
        }
        Properties exportParameters = new Properties();
        exportParameters.put(XMLExporter.TOPOLOGY_LEVEL, "BUS_BREAKER");
        network.write("XIIDM", exportParameters, endPath);
    }

    public static void convertNodeBreakerDataToBusBreaker(String nodeBreakerDir, String busBreakerDir, String initFileName) {
        Path initRoot = Path.of(nodeBreakerDir);
        Path endRoot = Path.of(busBreakerDir);

        try (Stream<Path> pathStream = Files.list(initRoot)) {
            pathStream.filter(Files::isDirectory)
                    .map(subDir -> subDir.resolve(initFileName))
                    .filter(Files::exists)
                    .forEach(initFile -> {
                        try {
                            Path relativePath = initRoot.relativize(initFile);
                            Path outputFile = endRoot.resolve(relativePath);
                            Files.createDirectories(outputFile.getParent());
                            exportNetworkAsBusBreakerTopology(initFile, outputFile);
                        } catch (Exception e) {
                            throw new RuntimeException("Failed to convert the files", e);
                        }
                    });
        } catch (Exception e) {
            throw new RuntimeException("Failed to load HU real network cases", e);
        }
    }

    public record NetworkPair(Network rknNetwork, Network nrNetwork, String baseFilename) {
    }
}

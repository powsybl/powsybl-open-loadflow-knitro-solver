package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.ieeecdf.converter.IeeeCdfNetworkFactory;
import com.powsybl.iidm.network.Network;
import com.powsybl.loadflow.LoadFlow;
import com.powsybl.loadflow.LoadFlowParameters;
import com.powsybl.loadflow.LoadFlowResult;
import com.powsybl.math.matrix.SparseMatrixFactory;
import com.powsybl.openloadflow.OpenLoadFlowParameters;
import com.powsybl.openloadflow.OpenLoadFlowProvider;
import com.powsybl.openloadflow.ac.solver.NewtonRaphsonStoppingCriteriaType;
import com.powsybl.openloadflow.knitro.solver.ResilientKnitroSolverTestUtils.VoltagePerturbation;
import com.powsybl.openloadflow.network.SlackBusSelectionMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static com.powsybl.openloadflow.knitro.solver.ResilientKnitroSolverTestUtils.applyVoltagePerturbation;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * @author Martin Debouté {@literal <martin.deboute at artelys.com>}
 * @author Amine Makhen {@literal <amine.makhen at artelys.com>}
 */
public class ResilientKnitroSolverConvergenceTest {
    private static final String RKN = "KNITRO";
    private static final String NR = "NEWTON_RAPHSON";
    private LoadFlow.Runner loadFlowRunner;
    private LoadFlowParameters parameters;

    @BeforeEach
    void setUp() {
        loadFlowRunner = new LoadFlow.Runner(new OpenLoadFlowProvider(new SparseMatrixFactory()));
        parameters = new LoadFlowParameters()
                .setUseReactiveLimits(false)
                .setDistributedSlack(false)
                .setVoltageInitMode(LoadFlowParameters.VoltageInitMode.DC_VALUES);
    }

    private void configureSolver(String solver) {
        OpenLoadFlowParameters.create(parameters)
                .setSlackBusSelectionMode(SlackBusSelectionMode.MOST_MESHED)
                .setNewtonRaphsonStoppingCriteriaType(NewtonRaphsonStoppingCriteriaType.PER_EQUATION_TYPE_CRITERIA)
                .setAcSolverType(solver);

        if (RKN.equals(solver)) {
            KnitroLoadFlowParameters knitroParams = new KnitroLoadFlowParameters();
            // Set the Knitro solver type to RESILIENT
            knitroParams.setKnitroSolverType(KnitroSolverParameters.KnitroSolverType.RESILIENT);
            parameters.addExtension(KnitroLoadFlowParameters.class, knitroParams);
        }
    }

    private void verifyConvergence(Network rknNetwork, Network nrNetwork) {
        // Verify Newton-Raphson solver diverges
        configureSolver(NR);
        LoadFlowResult resultNR = loadFlowRunner.run(nrNetwork, parameters);
        boolean isConvergedNR = resultNR.isFullyConverged();
        assertFalse(isConvergedNR, "NR should not converge");

        // Verify resilient Knitro solver converges
        configureSolver(RKN);
        LoadFlowResult resultRKN = loadFlowRunner.run(rknNetwork, parameters);
        boolean isConvergedRKN = resultRKN.isFullyConverged();
        assertTrue(isConvergedRKN, "RKN should converge");
    }

    @Test
    void testConvergenceAfterVoltagePerturbationOnI3E14() {
        // Create network objects
        Network rknNetwork = IeeeCdfNetworkFactory.create14();
        Network nrNetwork = IeeeCdfNetworkFactory.create14();

        // Define perturbation
        String bus1Id = "B5";
        String bus2Id = "B2";
        String generatorId = "B1-G";
        String lineId = "L1-5-1";

        VoltagePerturbation voltagePerturbation = new VoltagePerturbation(
                bus1Id,
                bus2Id,
                generatorId,
                lineId
        );

        double x = 0.001742;
        double targetV = 131.0429;

        // Apply perturbation to both networks
        applyVoltagePerturbation(nrNetwork, voltagePerturbation, x, targetV);
        applyVoltagePerturbation(rknNetwork, voltagePerturbation, x, targetV);

        verifyConvergence(rknNetwork, nrNetwork);
    }
}

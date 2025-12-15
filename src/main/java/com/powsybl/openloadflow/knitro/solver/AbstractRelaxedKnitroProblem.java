package com.powsybl.openloadflow.knitro.solver;

import com.artelys.knitro.api.KNException;
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.equations.*;
import com.powsybl.openloadflow.network.LfNetwork;

import java.util.*;
import java.util.stream.Collectors;

public abstract class AbstractRelaxedKnitroProblem extends AbstractKnitroProblem {

    protected final int numLfAndSlackVariables;

    protected AbstractRelaxedKnitroProblem(LfNetwork network, EquationSystem<AcVariableType, AcEquationType> equationSystem,
                                           TargetVector<AcVariableType, AcEquationType> targetVector, JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                                           KnitroSolverParameters knitroParameters,
                                           int numTotalVariables, int numTotalConstraints, int numLfAndSlackVariables) {
        super(network, equationSystem, targetVector, jacobianMatrix, knitroParameters, numTotalVariables, numTotalConstraints);
        this.numLfAndSlackVariables = numLfAndSlackVariables;
    }

    /**
     * Returns the sparsity pattern of the hessian matrix associated with the problem.
     *
     * @param nonlinearConstraintIndexes A list of the indexes of non-linear equations.
     * @return row and column coordinates of non-zero entries in the hessian matrix.
     */
    AbstractMap.SimpleEntry<List<Integer>, List<Integer>> getHessNnzRowsAndCols(List<Integer> nonlinearConstraintIndexes) {
        record NnzCoordinates(int iRow, int iCol) {
        }

        Set<NnzCoordinates> hessianEntries = new LinkedHashSet<>();

        // Non-linear constraints contributions in the hessian matrix
        for (int index : nonlinearConstraintIndexes) {
            Equation<AcVariableType, AcEquationType> equation = equationSystem.getIndex().getSortedEquationsToSolve().get(index);
            for (EquationTerm<AcVariableType, AcEquationType> term : equation.getTerms()) {
                for (Variable<AcVariableType> var1 : term.getVariables()) {
                    int i = var1.getRow();
                    for (Variable<AcVariableType> var2 : term.getVariables()) {
                        int j = var2.getRow();
                        if (j >= i) {
                            hessianEntries.add(new NnzCoordinates(i, j));
                        }
                    }
                }
            }
        }

        // Slacks variables contributions in the objective function
        for (int iSlack = numberOfPowerFlowVariables; iSlack < numLfAndSlackVariables; iSlack++) {
            hessianEntries.add(new NnzCoordinates(iSlack, iSlack));
            if (((iSlack - numberOfPowerFlowVariables) & 1) == 0) {
                hessianEntries.add(new NnzCoordinates(iSlack, iSlack + 1));
            }
        }

        // Sort the entries by row and column indices
        hessianEntries = hessianEntries.stream()
                .sorted(Comparator.comparingInt(NnzCoordinates::iRow).thenComparingInt(NnzCoordinates::iCol))
                .collect(Collectors.toCollection(LinkedHashSet::new));

        List<Integer> hessRows = new ArrayList<>();
        List<Integer> hessCols = new ArrayList<>();
        for (NnzCoordinates entry : hessianEntries) {
            hessRows.add(entry.iRow());
            hessCols.add(entry.iCol());
        }

        return new AbstractMap.SimpleEntry<>(hessRows, hessCols);
    }

    // TODO : to be refactored with an AbstractRelaxedKnitroSolverClass
    void addObjectiveFunction(int numPEquations, int slackPStartIndex, int numQEquations, int slackQStartIndex,
                              int numVEquations, int slackVStartIndex) throws KNException {
        // initialise lists to track quadratic objective function terms of the form: a * x1 * x2
        List<Integer> quadRows = new ArrayList<>(); // list of indexes of the first variable x1
        List<Integer> quadCols = new ArrayList<>(); // list of indexes of the second variable x2
        List<Double> quadCoefs = new ArrayList<>(); // list of indexes of the coefficient a

        // initialise lists to track linear objective function terms of the form: a * x
        List<Integer> linIndexes = new ArrayList<>(); // list of indexes of the variable x
        List<Double> linCoefs = new ArrayList<>(); // list of indexes of the coefficient a

        // add slack penalty terms, for each slack type, of the form: (Sp - Sm)^2 = Sp^2 + Sm^2 - 2*Sp*Sm + linear terms from the absolute value
        addSlackObjectiveTerms(numPEquations, slackPStartIndex, RelaxedKnitroSolver.WEIGHT_P_PENAL, RelaxedKnitroSolver.WEIGHT_ABSOLUTE_PENAL, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);
        addSlackObjectiveTerms(numQEquations, slackQStartIndex, RelaxedKnitroSolver.WEIGHT_Q_PENAL, RelaxedKnitroSolver.WEIGHT_ABSOLUTE_PENAL, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);
        addSlackObjectiveTerms(numVEquations, slackVStartIndex, RelaxedKnitroSolver.WEIGHT_V_PENAL, RelaxedKnitroSolver.WEIGHT_ABSOLUTE_PENAL, quadRows, quadCols, quadCoefs, linIndexes, linCoefs);

        setObjectiveQuadraticPart(quadRows, quadCols, quadCoefs);
        setObjectiveLinearPart(linIndexes, linCoefs);
    }

    /**
     * Adds quadratic and linear terms related to slack variables to the objective function.
     */
    void addSlackObjectiveTerms(int numEquations, int slackStartIdx, double weight, double lambda,
                                List<Integer> quadRows, List<Integer> quadCols, List<Double> quadCoefs,
                                List<Integer> linIndexes, List<Double> linCoefs) {
        for (int i = 0; i < numEquations; i++) {
            int idxSm = slackStartIdx + 2 * i; // negative slack variable index
            int idxSp = slackStartIdx + 2 * i + 1; // positive slack variable index

            // Add quadratic terms: weight * (sp^2 + sm^2 - 2 * sp * sm)

            // add first quadratic term : weight * sp^2
            quadRows.add(idxSp);
            quadCols.add(idxSp);
            quadCoefs.add(weight);

            // add second quadratic term : weight * sm^2
            quadRows.add(idxSm);
            quadCols.add(idxSm);
            quadCoefs.add(weight);

            // add third quadratic term : weight * (- 2 * sp * sm)
            quadRows.add(idxSp);
            quadCols.add(idxSm);
            quadCoefs.add(-2 * weight);

            // Add linear terms: weight * lambda * (sp + sm)

            // add first linear term : weight * lambda * sp
            linIndexes.add(idxSp);
            linCoefs.add(lambda * weight);

            // add second linear term : weight * lambda * sm
            linIndexes.add(idxSm);
            linCoefs.add(lambda * weight);
        }
    }

}

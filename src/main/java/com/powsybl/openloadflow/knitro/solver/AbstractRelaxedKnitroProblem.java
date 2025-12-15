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
    private AbstractMap.SimpleEntry<List<Integer>, List<Integer>> getHessNnzRowsAndCols(List<Integer> nonlinearConstraintIndexes) {
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
        for (int iSlack = numberOfPowerFlowVariables; iSlack < this.numLfAndSlackVariables; iSlack++) {
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

    void setJacobianMatrix(LfNetwork lfNetwork, JacobianMatrix<AcVariableType, AcEquationType> jacobianMatrix,
                           List<Equation<AcVariableType, AcEquationType>> sortedEquationsToSolve, List<Integer> listNonLinearConsts,
                           List<Integer> listNonZerosCtsDense, List<Integer> listNonZerosVarsDense,
                           List<Integer> listNonZerosCtsSparse, List<Integer> listNonZerosVarsSparse) throws KNException {
        int numVar = equationSystem.getIndex().getSortedVariablesToFind().size();
        if (knitroParameters.getGradientComputationMode() == 1) { // User routine to compute the Jacobian
            if (knitroParameters.getGradientUserRoutine() == 1) {
                // Dense method: all non-linear constraints are considered as a function of all variables.
                buildDenseJacobianMatrix(numVar, listNonLinearConsts, listNonZerosCtsDense, listNonZerosVarsDense);
                this.setJacNnzPattern(listNonZerosCtsDense, listNonZerosVarsDense);
            } else if (knitroParameters.getGradientUserRoutine() == 2) {
                // Sparse method: compute Jacobian only for variables the constraints depend on.
                buildSparseJacobianMatrix(sortedEquationsToSolve, listNonLinearConsts, listNonZerosCtsSparse, listNonZerosVarsSparse);
                this.setJacNnzPattern(listNonZerosCtsSparse, listNonZerosVarsSparse);
            }
            // Set the callback for gradient evaluations if the user directly passes the Jacobian to the solver.
            this.setGradEvalCallback(createGradientCallback(jacobianMatrix, listNonZerosCtsDense, listNonZerosVarsDense,
                    listNonZerosCtsSparse, listNonZerosVarsSparse));
        }
    }

}

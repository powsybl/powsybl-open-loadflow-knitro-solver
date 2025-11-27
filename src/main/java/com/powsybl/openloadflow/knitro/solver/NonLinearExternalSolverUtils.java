/**
 * Copyright (c) 2024, Artelys (http://www.artelys.com/)
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 * SPDX-License-Identifier: MPL-2.0
 */
package com.powsybl.openloadflow.knitro.solver;

import com.powsybl.openloadflow.ac.equations.AcVariableType;
import com.powsybl.openloadflow.ac.equations.AcEquationType;
import com.powsybl.openloadflow.equations.EquationTerm;
import com.powsybl.openloadflow.equations.VariableEquationTerm;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.*;
import java.util.stream.IntStream;

/**
 * @author Jeanne Archambault {@literal <jeanne.archambault at artelys.com>}
 */
public final class NonLinearExternalSolverUtils {

    private static final Logger LOGGER = LoggerFactory.getLogger(NonLinearExternalSolverUtils.class);

    // List of always linear constraints
    private static final List<AcEquationType> LINEAR_CONSTRAINTS_TYPES = new ArrayList<>(Arrays.asList(
            AcEquationType.BUS_TARGET_PHI,
            AcEquationType.DUMMY_TARGET_P,
            AcEquationType.DUMMY_TARGET_Q,
            AcEquationType.ZERO_V,
            AcEquationType.ZERO_PHI,
            AcEquationType.DISTR_SHUNT_B,
            AcEquationType.DISTR_RHO,
            AcEquationType.SHUNT_TARGET_B,
            AcEquationType.BRANCH_TARGET_ALPHA1,
            AcEquationType.BRANCH_TARGET_RHO1
    ));

    // Classifies a constraint as linear or non-linear based on its type and terms
    public static boolean isLinear(AcEquationType typeEq, List<EquationTerm<AcVariableType, AcEquationType>> terms) {
        // Check if the constraint type is BUS_TARGET_V
        if (typeEq == AcEquationType.BUS_TARGET_V) {
            return terms.size() == 1; // If there's only one term, it is linear
        }
        return LINEAR_CONSTRAINTS_TYPES.contains(typeEq);
    }

    // Return lists of variables and coefficients to pass to Knitro for a linear constraint
    public VarAndCoefList getLinearConstraint(AcEquationType typeEq, List<EquationTerm<AcVariableType, AcEquationType>> terms) throws UnsupportedOperationException {
        VarAndCoefList varAndCoefList = null;

        // Check if the constraint is linear
        if (isLinear(typeEq, terms)) {
            switch (typeEq) {
                case BUS_TARGET_V, BUS_TARGET_PHI, DUMMY_TARGET_P, DUMMY_TARGET_Q, SHUNT_TARGET_B, BRANCH_TARGET_ALPHA1, BRANCH_TARGET_RHO1:
                    // BUS_TARGET_V should be treated as linear
                    varAndCoefList = addConstraintConstantTarget(terms);
                    break;
                case DISTR_SHUNT_B, DISTR_RHO:
                    varAndCoefList = addConstraintDistrQ(terms);
                    break;
                case ZERO_V, ZERO_PHI:
                    varAndCoefList = addConstraintZero(terms);
                    break;
                default:
                    throw new UnsupportedOperationException("Non-linear equation : " + typeEq);
            }
        }
        return varAndCoefList;
    }

    public record VarAndCoefList(List<Integer> listIdVar, List<Double> listCoef) {
    }

    public VarAndCoefList addConstraintConstantTarget(List<EquationTerm<AcVariableType, AcEquationType>> terms) {
        // get the variable V/Theta/DummyP/DummyQ/... corresponding to the constraint
        int idVar = terms.get(0).getVariables().get(0).getRow();
        return new VarAndCoefList(List.of(idVar), List.of(1.0));
    }

    public VarAndCoefList addConstraintZero(List<EquationTerm<AcVariableType, AcEquationType>> terms) {
        // get the variables Vi and Vj / Thetai and Thetaj corresponding to the constraint
        int idVari = terms.get(0).getVariables().get(0).getRow();
        int idVarj = terms.get(1).getVariables().get(0).getRow();
        return new VarAndCoefList(Arrays.asList(idVari, idVarj), Arrays.asList(1.0, -1.0));
    }

    public VarAndCoefList addConstraintDistrQ(List<EquationTerm<AcVariableType, AcEquationType>> terms) {
        // get the variables corresponding to the constraint
        List<Integer> listVar = new ArrayList<>();
        List<Double> listCoef = new ArrayList<>();
        for (EquationTerm<AcVariableType, AcEquationType> equationTerm : terms) {
            double scalar = 0.0;
            if (((EquationTerm.MultiplyByScalarEquationTerm) equationTerm).getChildren().get(0) instanceof VariableEquationTerm<?, ?>) {
                scalar = ((EquationTerm.MultiplyByScalarEquationTerm) equationTerm).getScalar();
            } else if (((EquationTerm.MultiplyByScalarEquationTerm) equationTerm).getChildren().get(0) instanceof EquationTerm.MultiplyByScalarEquationTerm<?, ?>) {
                scalar = ((EquationTerm.MultiplyByScalarEquationTerm) equationTerm).getScalar();
                scalar *= ((EquationTerm.MultiplyByScalarEquationTerm) ((EquationTerm.MultiplyByScalarEquationTerm) equationTerm).getChildren().get(0)).getScalar();
            }
            listVar.add(((EquationTerm.MultiplyByScalarEquationTerm<AcVariableType, AcEquationType>) equationTerm).getChildren().get(0) .getVariables().get(0).getRow());
            listCoef.add(scalar);
        }
        return new VarAndCoefList(listVar, listCoef);
    }

    /**
     * Builds the row and column index lists corresponding to the dense Jacobian structure,
     * assuming each non-linear constraint is derived with respect to every variable.
     *
     * @param numVars               Total number of variables.
     * @param listNonLinearConsts   List of non-linear constraint indices.
     * @param listNonZerosCtsDense  Output list to receive constraint indices for non-zero Jacobian entries.
     * @param listNonZerosVarsDense Output list to receive variable indices for non-zero Jacobian entries.
     */
    public static void buildDenseJacobianMatrix(
            int numVars,
            List<Integer> listNonLinearConsts,
            List<Integer> listNonZerosCtsDense,
            List<Integer> listNonZerosVarsDense) {

        // Each non-linear constraint will have a partial derivative with respect to every variable
        for (Integer constraintId : listNonLinearConsts) {
            for (int varIndex = 0; varIndex < numVars; varIndex++) {
                listNonZerosCtsDense.add(constraintId);
            }
        }

        // We repeat the full list of variables for each non-linear constraint
        List<Integer> variableIndices = IntStream.range(0, numVars).boxed().toList();
        for (int i = 0; i < listNonLinearConsts.size(); i++) {
            listNonZerosVarsDense.addAll(variableIndices);
        }
    }

    public static void logKnitroStatus(KnitroStatus status) {
        LOGGER.info("Knitro Status: {}", status);
    }
}

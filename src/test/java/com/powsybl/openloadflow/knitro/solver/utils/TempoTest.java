package com.powsybl.openloadflow.knitro.solver.utils;

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  This example demonstrates how to use Knitro to solve the following
 *  simple mathematical program with equilibrium/complementarity
 *  constraints (MPEC/MPCC).
 *
 *   min   (x0 - 5)^2 + (2 x1 + 1)^2
 *   s.t.  -1.5 x0 + 2 x1 + x2 - 0.5 x3 + x4 = 2
 *         x2 complements (3 x0 - x1 - 3)
 *         x3 complements (-x0 + 0.5 x1 + 4)
 *         x4 complements (-x0 - x1 + 7)
 *         x0, x1, x2, x3, x4 >= 0
 *
 *  The complementarity constraints must be converted so that one
 *  nonnegative variable complements another nonnegative variable.
 *
 *   min   (x0 - 5)^2 + (2 x1 + 1)^2
 *   s.t.  -1.5 x0 + 2 x1 + x2 - 0.5 x3 + x4 = 2   (c0)
 *         3 x0 - x1 - 3 - x5 = 0                  (c1)
 *         -x0 + 0.5 x1 + 4 - x6 = 0               (c2)
 *         -x0 - x1 + 7 - x7 = 0                   (c3)
 *         x2 complements x5
 *         x3 complements x6
 *         x4 complements x7
 *         x0, x1, x2, x3, x4, x5, x6, x7 >= 0
 *
 *  The solution is (1, 0, 3.5, 0, 0, 0, 3, 6), with objective value 17.
 *++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

import com.artelys.knitro.api.*;

import java.util.Arrays;

public final class TempoTest {

    private TempoTest() {
        throw new UnsupportedOperationException();
    }

    private static class ProblemMPEC1 extends KNProblem {
        public ProblemMPEC1() throws KNException {
            super(8, 4);

            // Variables
            setVarLoBnds(Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
            setXInitial(Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

            // Constraints
            setConEqBnds(Arrays.asList(2.0, 3.0, -4.0, -7.0));
            setCompConstraintsTypes(Arrays.asList(KNConstants.KN_CCTYPE_VARVAR, KNConstants.KN_CCTYPE_VARVAR, KNConstants.KN_CCTYPE_VARVAR));
            // x2 x3 and x4 complements respectively x5, x6 and x7
            setCompConstraintsParts(Arrays.asList(2, 3, 4), Arrays.asList(5, 6, 7));

            // Objective structure
            /*  Note that the objective (x0 - 5)^2 + (2 x1 + 1)^2 when
             *  expanded becomes:
             *     x0^2 + 4 x1^2 - 10 x0 + 4 x1 + 26  */
            /*  Add quadratic coefficients for the objective */
            setObjectiveQuadraticPart(Arrays.asList(0, 1), Arrays.asList(0, 1), Arrays.asList(1.0, 4.0));
            /*  Add linear coefficients for the objective */
            setObjectiveLinearPart(Arrays.asList(0, 1), Arrays.asList(-10.0, 4.0));
            /*  Add constant to the objective */
            setObjConstPart(26.0);

            // Constraints Strucutres
            /* c0 */
            addConstraintLinearPart(0, Arrays.asList(0, 1, 2, 3, 4), Arrays.asList(-1.5, 2.0, 1.0, -0.5, 1.0));
            /* c1 */
            addConstraintLinearPart(1, Arrays.asList(0, 1, 5), Arrays.asList(3.0, -1.0, -1.0));
            /* c2 */
            addConstraintLinearPart(2, Arrays.asList(0, 1, 6), Arrays.asList(-1.0, 0.5, -1.0));
            /* c3 */
            addConstraintLinearPart(3, Arrays.asList(0, 1, 7), Arrays.asList(-1.0, -1.0, -1.0));
        }
    }

    public static void main(String[] args) throws KNException {
        // Create a problem instance.
        ProblemMPEC1 instance = new ProblemMPEC1();
        // Create a solver
        try (KNSolver solver = new KNSolver(instance)) {

            solver.initProblem();
            solver.solve();

            KNSolution solution = solver.getSolution();

            System.out.print("\n\n");
            System.out.format("Knitro converged with final status = %d%n", solution.getStatus());
            System.out.format("  optimal objective value  = %f%n", solution.getObjValue());
            System.out.format("  optimal primal values  x0=%f%n", solution.getX().get(0));
            System.out.format("                         x1=%f%n", solution.getX().get(1));
            System.out.format("                         x2=%f complements x5=%f%n", solution.getX().get(2), solution.getX().get(5));
            System.out.format("                         x3=%f complements x6=%f%n", solution.getX().get(3), solution.getX().get(6));
            System.out.format("                         x4=%f complements x7=%f%n", solution.getX().get(4), solution.getX().get(7));
            System.out.format("  feasibility violation    = %f%n", solver.getAbsFeasError());
            System.out.format("  KKT optimality violation = %f%n", solver.getAbsOptError());
        }
    }
}

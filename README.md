# Non-linear External Optimization Solver

This document provides an overview of the **Non-linear External Optimization Solver** extension and its integration into a Java project.

---

## Overview

By default, load flow calculations in the solver are performed using the **Newton-Raphson** method. However, this extension allows these calculations to be executed using the **non-linear solver Knitro**, modeling the problem as a **constraint satisfaction problem** (without an objective function). 

To use the Knitro solver, a trial license is available on the [Artelys website](https://www.artelys.com). Users must invoke `.setAcSolverType(KnitroSolverFactory.NAME)` in the `OpenLoadFlowParameters` extension.

### Functionality

The Knitro solver is used as a substitute for the **inner loop calculations** in the load flow process. The **outer loop** operates identically to the Newton-Raphson method. After the outer loop is complete, the inner loop calculations are executed with Knitro.

## Configuration

### Parameter Settings

Most parameters for Knitro are configured similarly to the Newton-Raphson method. However, specific parameters tailored to Knitro are provided through the `KnitroLoadFlowParameters` extension of `LoadFlowParameters`. 

#### Example: Setting Gradient Computation Mode

```java
LoadFlowParameters parameters = new LoadFlowParameters();
KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters();
knitroLoadFlowParameters.setGradientComputationMode(2);
parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);
```

### Default Knitro Parameters

1. **Voltage Bounds**:
    - Default range: **0.5 p.u. to 1.5 p.u.** : they define lower and upper bounds of voltages.
    - Modify using:
        - `setLowerVoltageBound`
        - `setUpperVoltageBound`

2. **Jacobian Matrix Usage**:
    - The solver utilizes the **Jacobian matrix** for solving successive linear approximations of the problem.
    - **Gradient Computation Modes**:
        - `1 (exact)`: Gradients computed in PowSyBl are provided to Knitro.
        - `2 (forward)`: Knitro computes gradients via forward finite differences.
        - `3 (central)`: Knitro computes gradients via central finite differences.
    - Use `setGradientComputationMode` in the `KnitroLoadFlowParameters` extension.

3. **Jacobian Sparsity**:
    - Default: **Sparse form** (highly recommended, improves calculation as load flow problems are highly sparse problems).
    - To specify dense form:
        - Use `setGradientUserRoutineKnitro` in the `KnitroLoadFlowParameters` extension.
    - **Options**:
        - `1 (dense)`: All constraints are considered as dependent of all variables.
        - `2 (sparse)`: Derivatives are computed only for variables involved in the constraints (recommended).

4. **Maximum Iterations**:
    - Default: **200**
    - Modify using `setMaxIterations`.

## Constraint Handling

Constraints are categorized into two types:

1. **Linear and Quadratic Constraints**:
    - Explicitly passed to the solver.

2. **Non-linear (Non-quadratic) Constraints**:
    - Evaluated during each iteration by a **callback class**, based on the current state.

---

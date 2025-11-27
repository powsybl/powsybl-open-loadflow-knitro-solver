# PowSyBl Open Load Flow - Knitro Solver

[![Actions Status](https://github.com/powsybl/powsybl-open-loadflow-knitro-solver/actions/workflows/maven.yml/badge.svg?branch=main)](https://github.com/powsybl/powsybl-open-loadflow-knitro-solver/actions/workflows/maven.yml)
[![Snapshot Status](https://github.com/powsybl/powsybl-open-loadflow-knitro-solver/actions/workflows/snapshot-ci.yml/badge.svg?branch=main)](https://github.com/powsybl/powsybl-open-loadflow-knitro-solver/actions/workflows/snapshot-ci.yml)
[![Coverage Status](https://sonarcloud.io/api/project_badges/measure?project=com.powsybl%3Apowsybl-open-loadflow-knitro-solver&metric=coverage)](https://sonarcloud.io/component_measures?id=com.powsybl%3Apowsybl-open-loadflow-knitro-solver&metric=coverage)
[![Quality Gate](https://sonarcloud.io/api/project_badges/measure?project=com.powsybl%3Apowsybl-open-loadflow-knitro-solver&metric=alert_status)](https://sonarcloud.io/dashboard?id=com.powsybl%3Apowsybl-open-loadflow-knitro-solver)
[![MPL-2.0 License](https://img.shields.io/badge/license-MPL_2.0-blue.svg)](https://www.mozilla.org/en-US/MPL/2.0/)
[![Slack](https://img.shields.io/badge/slack-powsybl-blueviolet.svg?logo=slack)](https://join.slack.com/t/powsybl/shared_invite/zt-36jvd725u-cnquPgZb6kpjH8SKh~FWHQ)


PowSyBl (**Pow**er **Sy**stem **Bl**ocks) is an open source library written in Java, that makes it easy to write complex
software for power systems’ simulations and analysis. Its modular approach allows developers to extend or customize its
features.

PowSyBl is part of the LF Energy Foundation, a project of The Linux Foundation that supports open source innovation projects
within the energy and electricity sectors.

<p align="center">
<img src="https://raw.githubusercontent.com/powsybl/powsybl-gse/main/gse-spi/src/main/resources/images/logo_lfe_powsybl.svg?sanitize=true" alt="PowSyBl Logo" width="50%"/>
</p>

Read more at https://www.powsybl.org !

This project and everyone participating in it is under the [Linux Foundation Energy governance principles](https://www.powsybl.org/pages/project/governance.html) and must respect the [PowSyBl Code of Conduct](https://github.com/powsybl/.github/blob/main/CODE_OF_CONDUCT.md).
By participating, you are expected to uphold this code. Please report unacceptable behavior to [powsybl-tsc@lists.lfenergy.org](mailto:powsybl-tsc@lists.lfenergy.org).

## PowSyBl vs PowSyBl Open Load Flow Knitro Solver

PowSyBl Open Load Flow Knitro Solver is an extension to [PowSyBl Open Load Flow](https://github.com/powsybl/powsybl-open-loadflow) allowing to solve
the load flow equations with the **non-linear solver Knitro** instead of the default **Newton-Raphson** method.

The Knitro solver extension offers two different ways to model the load flow problem: either as a **constraint satisfaction problem** (without an objective function) or as an **optimisation problem** with relaxed constraints (and an objective function minimizing the violations). 

## Getting Started

To use the PowSyBl Open Load Flow Knitro Solver extension, a valid Knitro installation is required.

### Platform compatibility

Knitro supports Linux, Windows, and macOS; however, its Java bindings are currently available only on Linux and Windows.

### Installing Knitro

1. Obtain the installation kit and trial license from the [Artelys website](https://www.artelys.com/solvers/knitro/programs/#trial).
2. Configure the following environment variables:
  - `KNITRODIR`: Path to the Knitro installation directory.
  - `ARTELYS_LICENSE`: Path to the Knitro license file or its content.

You may then validate your installation by running one of the Java examples like this (here on Linux):

```bash
cd $KNITRODIR/examples/java/examples
# compile example
javac -cp ".;../lib/*" com/artelys/knitro/examples/ExampleNLP1.java
# run example
java -cp ".;../lib/*" com.artelys.knitro.examples.ExampleNLP1
```

<details>

<summary>Here an example output (click to expand)</summary>

```
$ java -cp ".;../lib/*" com.artelys.knitro.examples.ExampleNLP1

--- snip ---

Optimal objective value  = 306.5000025616414
Optimal x (with corresponding multiplier)
x1 = 0,500000 (lambda = -700,000000)
x2 = 2,000000 (lambda = -0,000000)
Optimal constraint values (with corresponding multiplier)
 c[0] = 1,000000 (lambda = -700,000000)
 c[1] = 4,500000 (lambda = -0,000000)
Feasibility violation    = 0,000000
Optimality violation     = 0,000003
```

</details>


### Installing Knitro Java Bindings
The Knitro Java bindings require a private JAR file that must be installed locally, as it is not available on Maven Central.

On **Linux**, use the following command:
```bash
./mvnw install:install-file -Dfile="$KNITRODIR/examples/Java/lib/Knitro-Interfaces-2.5-KN_14.2.0.jar" -DgroupId=com.artelys -DartifactId=knitro-interfaces -Dversion=14.2.0 -Dpackaging=jar -DgeneratePom=true
```

On **Windows**, use the following command:
```bash
mvn install:install-file -Dfile="$env:KNITRODIR/examples/Java/lib/Knitro-Interfaces-2.5-KN_14.2.0.jar" -DgroupId="com.artelys" -DartifactId=knitro-interfaces -Dversion="14.2.0" -Dpackaging=jar -DgeneratePom=true
```

### Running a Load Flow with Knitro Solver

To run a load flow with PowSyBl Open Load Flow Knitro Solver. We first add a few Maven
dependencies to respectively have access to network model, IEEE test networks and simple logging capabilities:

```xml
<dependency>
    <groupId>com.powsybl</groupId>
    <artifactId>powsybl-iidm-impl</artifactId>
    <version>7.0.0</version>
</dependency>
<dependency>
    <groupId>com.powsybl</groupId>
    <artifactId>powsybl-ieee-cdf-converter</artifactId>
    <version>7.0.0</version>
</dependency>
<dependency>
    <groupId>org.slf4j</groupId>
    <artifactId>slf4j-simple</artifactId>
    <version>2.0.13</version>
</dependency>
```

We are now able to load the IEEE 14 bus test network:
 ```java
Network network = IeeeCdfNetworkFactory.create14();
 ```

After adding dependency on both Open Load Flow implementation and Knitro Solver extension:
```xml
<dependency>
    <groupId>com.powsybl</groupId>
    <artifactId>powsybl-open-loadflow</artifactId>
    <version>2.0.0</version>
</dependency>
<dependency>
    <groupId>com.powsybl</groupId>
    <artifactId>powsybl-open-loadflow-knitro-solver</artifactId>
    <version>0.1.0</version>
</dependency>
```

To run the load flow with the Knitro solver, configure the
[Open Load Flow parameter `acSolverType`](https://powsybl.readthedocs.io/projects/powsybl-open-loadflow/en/latest/loadflow/parameters.html)
as follows:

```java
LoadFlowParameters parameters = new LoadFlowParameters();
OpenLoadFlowParameters.create(parameters)
   .setAcSolverType("KNITRO"); // Change default Open Load Flow parameter acSolverType from NEWTON_RAPHSON to KNITRO
LoadFlow.run(network, parameters);
```

## Features

The Knitro solver is used as a substitute for the **inner loop calculations** in the load flow process.
The **outer loops** such as distributed slack, reactive limits, etc... operate identically as when using the Newton-Raphson method.

### Configuration

Most parameters for Knitro are configured similarly to the Newton-Raphson method.
However, specific parameters tailored to Knitro are provided through the `KnitroLoadFlowParameters` extension of `LoadFlowParameters`. 

Here an example on how to provide Knitro solver specific parameters in Java:

```java
LoadFlowParameters parameters = new LoadFlowParameters();
OpenLoadFlowParameters.create(parameters)
   .setAcSolverType("KNITRO");
KnitroLoadFlowParameters knitroLoadFlowParameters = new KnitroLoadFlowParameters();
knitroLoadFlowParameters.setGradientComputationMode(2);
parameters.addExtension(KnitroLoadFlowParameters.class, knitroLoadFlowParameters);
```

### Knitro Parameters

1. **Knitro Solver Type**:
    - Specifies the way to model the load flow problem : 
    - **Knitro Solver Types**:
      - `STANDARD (default)` : the constraint satisfaction problem formulation and a direct substitute to the Newton-Raphson solver.
      - `RELAXED` : the optimisation problem formulation relaxing satisfaction problem.
    - Use `setKnitroSolverType` in the `KnitroLoadFlowParameters` extension.

2. **Voltage Bounds**:
    - Default range: **0.5 p.u. to 1.5 p.u.** : they define lower and upper bounds of voltages.
    - Modify using:
        - `setLowerVoltageBound`
        - `setUpperVoltageBound`

3. **Jacobian Matrix Usage**:
    - The solver utilizes the **Jacobian matrix** for solving successive linear approximations of the problem.
    - **Gradient Computation Modes**:
        - `1 (exact)`: Gradients computed in PowSyBl are provided to Knitro.
        - `2 (forward)`: Knitro computes gradients via forward finite differences.
        - `3 (central)`: Knitro computes gradients via central finite differences.
    - Use `setGradientComputationMode` in the `KnitroLoadFlowParameters` extension.

4. **Jacobian Sparsity**:
    - Default: **Sparse form** (highly recommended, improves calculation as load flow problems are highly sparse problems).
    - To specify dense form:
        - Use `setGradientUserRoutineKnitro` in the `KnitroLoadFlowParameters` extension.
    - **Options**:
        - `1 (dense)`: All constraints are considered as dependent of all variables.
        - `2 (sparse)`: Derivatives are computed only for variables involved in the constraints (recommended).

5. **Maximum Iterations**:
    - Default: **200**
    - Modify using `setMaxIterations`.

### Constraint Handling

Constraints are categorized into two types:

1. **Linear and Quadratic Constraints**:
    - Explicitly passed to the solver.

2. **Non-linear (Non-quadratic) Constraints**:
    - Evaluated during each iteration by a **callback class**, based on the current state.

---

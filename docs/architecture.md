# Architecture

How the code is organized now. The decisions behind it are in
[decisions.md](decisions.md); where it is heading is in
[vision.md](vision.md).

## Pictures first

The sections up to "Modules" are diagrams to look things up in; the rest
of the file is the detailed text.

| Question | Diagram |
| --- | --- |
| What goes into and comes out of each step? | [Data flow](#data-flow) |
| Which library may use which? | [Libraries](#libraries) |
| What is inside `Problem`, `MeshedProblem`, `AssembledProblem`, a matrix? | [What each object contains](#what-each-object-contains) |
| Where do facts and parameter values come in? | [A coefficient, from text to matrix](#a-coefficient-from-text-to-matrix) |
| How do mesh nodes map to matrix rows, and where do Dirichlet nodes go? | [Nodes and unknowns](#nodes-and-unknowns) |
| What happens inside `assemble`? | [Inside assemble](#inside-assemble) |
| What are the stages of meshing? | [Meshing pipeline](#meshing-pipeline) |
| In what order does a time step run? | [One time step](#one-time-step) |
| Which interfaces exist and what implements them? | [Interfaces and their components](#interfaces-and-their-components) |
| How does adaptive refinement go round (estimate, mark, refine)? | [Adaptive refinement](#adaptive-refinement-estimation-marking-and-the-loop-in-pde2d_adapt-refinement-in-pde2d_mesh) |

## Data flow

Rectangles are steps (an object with one main function, or a function);
rounded nodes are data.

```mermaid
flowchart LR
    SVG[SVG import / shapes]
    IMP[Gmsh / .pde2d import]
    MSH[Mesher]
    RM[Remesher]
    CMB[MeshedProblem constructor]
    ASM[Assembler]
    LS[LinearSolver]
    TM[TimeMethod step]
    SB[build_solution]

    G([CurveGeometry])
    SPEC([MeshSpec])
    M([Mesh])
    P([Problem])
    MP([MeshedProblem])
    AP([AssembledProblem])
    LR([LinearSolveResult])
    S([Solution])

    SVG --> G
    G --> MSH
    SPEC --> MSH
    MSH --> M
    IMP --> M
    M --> RM
    SPEC --> RM
    RM --> M
    M --> CMB
    P --> CMB
    CMB --> MP
    MP --> ASM
    ASM --> AP
    AP -. previous .-> ASM
    AP --> TM
    TM --> LS
    AP --> LS
    LS --> LR
    LR --> SB
    MP --> SB
    SB --> S
    S -. initial .-> TM
```

The `StaticSolver` and the `Integrator` are the drivers of
the right-hand part: each takes a `MeshedProblem`, calls the `Assembler`
itself, and returns a `Solution`. The static solver goes once through
Assembler → LinearSolver → build_solution; the integrator repeats
Assembler → TimeMethod step → LinearSolver for every time step and builds a
`Solution` for snapshots and at the end.

### Steps

| Step | Call | Takes | Gives | Library |
| --- | --- | --- | --- | --- |
| SVG import, shapes | `io::import_svg`, `geometry` shape functions, user code | a file or numbers | `CurveGeometry` | `pde2d_io`, `pde2d_geometry` |
| Mesh import | `io::import_gmsh`, `io::import_pde2d` | a file | `Mesh` (no curve links) | `pde2d_io` |
| Mesher | `Mesher::run(geometry, spec)` | `CurveGeometry`, `MeshSpec` | `MeshResult`: the `Mesh`, reports, `properties`, `target_met` | `pde2d_mesh` |
| Remesher | `Remesher::run(mesh, spec)` | `Mesh`, `MeshSpec` | `MeshResult` | `pde2d_mesh` |
| Combining | `MeshedProblem(mesh, problem)` | `Mesh`, `Problem` | `MeshedProblem` | `pde2d_fem` |
| Assembler | `Assembler::assemble(meshed, t, previous, max_time_order)`; `Assembler::time_order(meshed)` | `MeshedProblem`, a time, optionally the previous result, the highest time order to build (default 2) | `AssembledProblem`; the time order alone, from the formula facts | `pde2d_fem` |
| Linear solver | `requirements()` checked by the caller, then `prepare(matrix)`, then `solve(rhs)` | a `CsrMatrix` (for the check: its `always_symmetric` or `symmetric_now` fact, or `is_symmetric` of its values), a right-hand side | `LinearSolveResult` | `pde2d_solve` |
| Time method | `advance(assembled, dt, prepared, state)` | one `AssembledProblem` (at the method's evaluation point), the prepared solver, the state, dt | the next state (through a linear solve) | `pde2d_solve` |
| Building the solution | `build_solution(meshed, free_values, t)` | the short vector, `MeshedProblem` | `Solution` | `pde2d_solve` |
| Error estimation | `ErrorEstimator::estimate(meshed, solution, derivatives)` | `MeshedProblem`, a `Solution` on its mesh, u_t and u_tt when the estimator needs them | `ErrorEstimate`: one indicator per triangle, the total, a report | `pde2d_adapt` |
| Marking | `MarkingStrategy::mark(mesh, indicators)` | `Mesh`, one error indicator per triangle | the marked triangles | `pde2d_adapt` |
| Refinement | `Remesher::refine(mesh, marked)` | `Mesh`, marked triangles | `RefinementResult`: the finer `Mesh`, reports, `properties`, optional `Parents` | `pde2d_mesh` |
| Adaptation | `Adaptation::estimate(state)`, `adapt(state, estimate)` (`MeshRefinement`: estimate, mark, `Remesher::refine`) | a `SolvedState` (problem, solution, derivatives) | `AdaptationResult`: a new `Mesh` or none, properties, parents, the estimate, reports | `pde2d_adapt` |
| Adaptive loop (static) | `AdaptiveStaticSolver::solve(meshed, target, t)` | `MeshedProblem` (its mesh is the start), `AdaptiveTarget` | `AdaptiveStaticResult`: every cycle's solution, estimate and reports, the last problem, why it stopped | `pde2d_adapt` |
| Solution transfer | `transfer_solution(old, meshed, parents)` | a `Solution` (values, velocities), the new `MeshedProblem`, optional `Parents` | `SolutionTransfer`: the `Solution` on the new mesh, a report | `pde2d_adapt` |
| Adaptive loop (in time) | `AdaptiveIntegrator::integrate(meshed, t0, t1, target, output, initial)` | `MeshedProblem` (its mesh is the start), times, `AdaptiveTimeTarget`, `IntegrationOutput` | `AdaptiveIntegrationResult`: the final solution, snapshots, step records, every piece (problem, integrator report, the adaptation at its end) | `pde2d_adapt` |
| Mesh properties | `measure(mesh, warning_angle_degrees)` | `Mesh` | `MeshProperties`: counts, smallest/largest angle, shortest/longest edge, triangles below the warning angle | `pde2d_mesh` |

### Data

| Data | What it holds | Changes after it is built? |
| --- | --- | --- |
| `CurveGeometry` | validated loops of exact curves with region names; computed depth and parent | only region edits |
| `MeshSpec` | the target: length bounds, absolute or relative to the domain size | plain settings |
| `Mesh` | nodes, counterclockwise triangles, boundary edges with region and optional curve link, region names, chains | a plain struct |
| `Problem` | the equation's coefficients, boundary conditions by region name and initial conditions as `Expression`s (each with its facts), and the parameter values | a plain struct |
| `MeshedProblem` | the mesh and the problem; every expression compiled to a function of (x, y, t) with its facts; the Dirichlet nodes and the node ↔ unknown tables; ∇·A and the time derivatives of the Dirichlet data | never |
| `AssembledProblem` | at one time t: the operators mass, damping, stiffness (each a `free` and a `dirichlet` `CsrMatrix`), the load, the Dirichlet values, the right-hand side, its own facts, a report | never |
| `LinearSolveResult` | one value per free unknown, iterations, relative residual, backward error, pivot ratio, threads, warnings (`messages`) | never |
| `Solution` | one value per mesh node, the mesh, the time (and, for time order 2, one velocity per node) | a plain struct |

Every stage that does work also returns or holds a `StageReport` (time,
metrics, "worked, but…" messages).

## Libraries

An arrow means "may include and link". Dependencies point one way only.
The name on the right of each box is the outside library it uses, only
inside its `.cpp` files.

```mermaid
flowchart BT
    GEO[pde2d_geometry]
    MESH[pde2d_mesh]
    IO["pde2d_io · libxml2"]
    FEM["pde2d_fem · SymEngine, OpenMP"]
    SOLVE["pde2d_solve · Eigen, SuiteSparse, OpenBLAS, OpenMP"]
    ADAPT[pde2d_adapt]

    MESH --> GEO
    IO --> MESH
    FEM --> MESH
    SOLVE --> FEM
    ADAPT --> SOLVE
```

`pde2d_io` and `pde2d_fem` do not use each other. `pde2d_adapt` uses
`pde2d_solve` (and through it fem, mesh and geometry) for the estimators
and the adaptive loop; nothing uses it, and it does not use io. The interface target `pde2d::pde2d` links all of them
for programs that use the whole library, such as the app (`app/`,
[app.md](app.md)), which sits on top and is never used by the library.

OpenMP gives the threads: in `pde2d_fem` the assembly of one colour, in
`pde2d_solve` Eigen's sparse matrix-vector product. `pde2d_fem` links it
privately; `pde2d_solve` makes it a public requirement, so a program that
links it and includes Eigen itself compiles Eigen's inline functions with
the same OpenMP setting (otherwise the linker may keep a serial copy of
`Eigen::nbThreads()` and the solvers run on one thread;
`tests/solve/eigen_consumer.cpp`). `<omp.h>` is included only from `.cpp`
files and private headers
(`src/fem/threads.hpp`, `src/solve/eigen_threads.hpp`). The thread count
is an option of each component that uses threads
(`AssemblerOptions::threads`, `ConjugateGradientOptions::threads`,
`BiCgStabOptions::threads`; default 1, 0 = all hardware threads, that is
`omp_get_num_procs()`, the processors available to the process; at most
1024); there is no global setting, and every count gives the same bits,
except in the parallel direct solvers (below).

SuiteSparse (CHOLMOD and ParU) gives `pde2d_solve` its parallel direct
solvers, and OpenBLAS (the system BLAS, pthreads build) their dense
kernels; both are linked privately and appear only in
`src/solve/suitesparse.hpp`, `supernodal_cholesky.cpp`,
`multifrontal_lu.cpp` and `factorization_threads.cpp`. SuiteSparse's
headers declare C functions and non-inline C++ functions, so unlike
Eigen they cannot give a program a differently compiled inline copy;
nothing about them is a public requirement. CHOLMOD's CMake config asks
for OpenMP's C component, so the project enables the C language; OpenBLAS
is found with `find_library` (Ubuntu's OpenBLAS config file points at a
wrong directory).

## What each object contains

A line with a diamond means "holds"; the number says how many.

```mermaid
classDiagram
    direction LR
    class Problem {
        Equation equation
        Parameters parameters
        map~string, BoundaryCondition~ boundary_conditions
        InitialConditions initial
    }
    class Equation {
        Expression a_xx, a_xy, a_yy
        Expression b_x, b_y
        Expression c
        Expression m1, m2
        Expression f
    }
    class BoundaryCondition {
        Expression alpha
        Expression beta_n, beta_tau, beta_x, beta_y
        Expression gamma
        Expression g
        int priority
    }
    class InitialConditions {
        optional~Expression~ u
        optional~Expression~ u_t
    }
    class Expression {
        parsed text
        facts() zero, depends_on_x, depends_on_y, depends_on_t
    }
    Problem *-- "1" Equation
    Problem *-- "1 per region name" BoundaryCondition
    Problem *-- "1" InitialConditions
    Equation *-- "9" Expression
    BoundaryCondition *-- "7" Expression
```

```mermaid
classDiagram
    direction LR
    class Mesh {
        vector~Point~ nodes
        vector~Triangle~ triangles
        vector~BoundaryEdge~ boundary_edges
        vector~string~ region_names
        vector~CurveChain~ chains
    }
    class MeshedProblem {
        mesh() shared_ptr to const Mesh
        problem() the Problem as given
        equation() CompiledEquation
        conditions() one CompiledCondition per mesh region
        divergence_of_a() CompiledDivergence
        dirichlet_nodes()
        dirichlet_value(k, t), dirichlet_rate(k, t), dirichlet_acceleration(k, t)
        free_nodes()
        free_index(node)
        initial_values(t0), initial_velocities(t0)
        initial_free_state(t0)
        report()
    }
    class CompiledEquation {
        CompiledExpression a_xx ... f
    }
    class CompiledCondition {
        ConditionKind kind
        bool oblique
        int priority
        CompiledExpression alpha ... g
    }
    class CompiledExpression {
        operator()(x, y, t) double
        facts()
    }
    MeshedProblem o-- "1" Mesh
    MeshedProblem *-- "1" CompiledEquation
    MeshedProblem *-- "1 per region" CompiledCondition
    CompiledEquation *-- "9" CompiledExpression
    CompiledCondition *-- "7" CompiledExpression
```

```mermaid
classDiagram
    direction LR
    class AssembledProblem {
        meshed_problem() shared_ptr to const MeshedProblem
        time()
        mass() Operator, M2
        damping() Operator, B = M1 - D
        stiffness() Operator, A = -K + C + R - S + T
        load() F - G on the free rows
        dirichlet_values() U_D at t
        dirichlet_rates() U_D' at t
        dirichlet_accelerations() U_D'' at t
        right_hand_side() load - A_FD U_D - B_FD U_D' - M2_FD U_D''
        load_depends_on_t()
        right_hand_side_depends_on_t()
        time_order()
        report()
    }
    class Operator {
        CsrMatrix free
        CsrMatrix dirichlet
        bool depends_on_t
        bool always_zero
        bool always_symmetric
        bool zero_now
        bool symmetric_now
    }
    class CsrMatrix {
        shared_ptr to const SparsityPattern pattern
        vector~double~ values
        entry(row, column)
        multiply(vector)
    }
    class SparsityPattern {
        rows, columns
        row_offsets
        column_indices
        find(row, column)
    }
    AssembledProblem o-- "1" MeshedProblem
    AssembledProblem *-- "3" Operator
    Operator *-- "2" CsrMatrix
    CsrMatrix o-- "1 shared" SparsityPattern
```

`free` is free rows × free columns (what a solver gets); `dirichlet` is
free rows × Dirichlet columns (it multiplies the known values). All `free`
matrices share one pattern object and all `dirichlet` matrices another.

## A coefficient, from text to matrix

The same coefficient at each stage, for `a_xx = "-k*(1+t)"` with `k = 1000`.

```mermaid
flowchart LR
    T(["text: -k*(1+t)"])
    PAR(["parameters: k = 1000"])
    E(["Expression in Problem<br/>parsed form + facts"])
    C(["CompiledExpression in MeshedProblem<br/>function (x, y, t) + the same facts"])
    V(["a number at a quadrature point"])
    B(["element block 3×3"])
    M(["entries of a CsrMatrix"])

    T -->|"Expression(text): parse, check the allow-list, compute facts"| E
    E -->|"MeshedProblem constructor: put the parameter values in, compile"| C
    PAR --> C
    C -->|"assembly: call it at (x, y, t)"| V
    V -->|"times gradients or shape functions, times the weight"| B
    B -->|"scatter through free_index"| M
```

- Facts (`zero`, `depends_on_x`, `depends_on_y`, `depends_on_t`) are
  computed once, when the text is parsed, and travel with the expression.
  Assembly reads them to skip zero terms and to decide what to rebuild at
  a new t.
- Parameter names become numbers in the `MeshedProblem` constructor.
- Coefficient values are never stored: only the matrices are kept.
- Boundary conditions go the same way (`BoundaryCondition` →
  `CompiledCondition`, one per mesh region).

## Nodes and unknowns

The mesh numbers every node; the matrices have one row per node whose
value is unknown. `MeshedProblem` holds the tables. Example: 6 nodes,
nodes 1 and 4 on a Dirichlet region.

| mesh node | 0 | 1 | 2 | 3 | 4 | 5 |
| --- | --- | --- | --- | --- | --- | --- |
| kind | free | Dirichlet | free | free | Dirichlet | free |
| `free_index(node)` (node → row) | 0 | none | 1 | 2 | none | 3 |
| position in `dirichlet_nodes()` | | 0 | | | 1 | |

`free_nodes()` is the other direction (row → node): `[0, 2, 3, 5]`.

```mermaid
flowchart LR
    BLK(["one entry of an element block:<br/>row node p, column node q"])
    Q1{"p Dirichlet?"}
    Q2{"q Dirichlet?"}
    SKIP["skipped: no equation for a known value"]
    FF["goes to the free matrix<br/>at (free_index(p), free_index(q))"]
    FD["goes to the dirichlet matrix<br/>at (free_index(p), k of q)"]

    BLK --> Q1
    Q1 -- yes --> SKIP
    Q1 -- no --> Q2
    Q2 -- no --> FF
    Q2 -- yes --> FD
```

Who uses the tables:

| Use | Table | Where |
| --- | --- | --- |
| placing block entries | `free_index` | assembly (above) |
| moving known values to the right-hand side | `dirichlet_value(k, t)` and its two time derivatives | assembly: `right_hand_side` |
| starting values per node | `dirichlet_value(k, t0)` at Dirichlet nodes | `initial_values(t0)` |
| the solver's short vector back to all nodes | `free_nodes`, `dirichlet_value` | `build_solution` |

## Inside assemble

`Assembler::assemble(meshed_problem, t, previous, max_time_order)`.

```mermaid
flowchart TD
    START(["meshed_problem, t, previous or nothing, max_time_order"])
    PLAN["classify every term from the facts:<br/>zero, constant, or depends on t;<br/>a term above max_time_order is zero"]
    HAVE{"previous given?"}
    STRUCT["build once: the two sparsity patterns,<br/>triangle colours, edge colours"]
    REUSE["take from previous: patterns, colours,<br/>the constant part of every operator it built"]
    TRI["triangles, colour by colour:<br/>blocks K, C, R, M1, M2, F (3-point rule)"]
    EDGE["boundary edges of natural conditions, colour by colour:<br/>blocks S, T, D, G (2-point rule)"]
    SCAT["scatter each block (see Nodes and unknowns)<br/>into the constant part or the part in t"]
    TOT["total of each operator = constant part + part in t"]
    RHS["U_D at t, U_D' and U_D'' only up to the time order;<br/>right-hand side = load − lifting"]
    FACTS["facts from the numbers of each free part:<br/>zero_now, symmetric_now"]
    OUT(["AssembledProblem: operators, load, right-hand side, facts from the formulas and from the numbers, report"])

    START --> PLAN --> HAVE
    HAVE -- no --> STRUCT --> TRI
    HAVE -- yes --> REUSE --> TRI
    TRI --> EDGE --> SCAT --> TOT --> FACTS --> RHS --> OUT
```

- Without `previous`, constant terms and terms in t are all computed.
  With it, only the terms in t are; the result is bit-for-bit the same.
- A zero term is never computed. An operator with no term in t is copied
  whole from `previous`.
- `max_time_order` 2 builds everything, 1 leaves out M2 (and U_D''), 0
  also M1 and D (and U_D'): A and the load only, the lifting
  load − A_FD·U_D. A left-out operator is a zero operator. An operator
  that `previous` did not build (it had a lower `max_time_order`) is built
  here in full, so the result stays bit for bit that of a fresh assembly
  at the same `max_time_order`.
- Two kinds of facts: the formula facts (`depends_on_t`, `always_zero`,
  `always_symmetric`) come from the plan and hold for every t; they alone
  decide the time order, reuse and what is built. The number facts
  (`zero_now`, `symmetric_now`) are read from each free part's values
  after the totals (`has_only_zero_entries`, `is_symmetric`), hold at
  this t only, and are implied by the formula facts. `Assembler::
  time_order(meshed)` makes the same plan at `max_time_order` 2 without
  assembling, so it equals `assemble(...).time_order()` at every t.
- The code has three pieces: computing a block (`element_blocks.cpp`, no
  shared state), scattering it, and the driver that loops over colours
  (`assembler.cpp`). Only the driver knows about threads: it shares the
  elements of one colour among `AssemblerOptions::threads` threads.

## Meshing pipeline

`Mesher::run(geometry, spec)`; the part in the box is the `Remesher`,
which can also start from any `Mesh`.

```mermaid
flowchart LR
    G([CurveGeometry])
    SPEC([MeshSpec])
    DISC[BoundaryDiscretizer]
    BP([BoundaryPolygon])
    TRI[InitialTriangulator]
    WM([WorkingMesh])
    M0([Mesh from anywhere])
    M([Mesh in MeshResult or RefinementResult])

    G --> DISC
    SPEC --> DISC
    DISC --> BP --> TRI --> WM
    MK([marked triangles])
    M0 --> WM
    subgraph Remesher
        direction LR
        REF["Refiner (optional): run or refine<br/>calls the local improvers"]
        FIN["final improvers, in order"]
        MEAS["measure the final mesh"]
        REF --> FIN --> MEAS
    end
    WM --> REF
    SPEC -- run --> REF
    MK -- refine --> REF
    MEAS --> M
```

`run` gives a `MeshResult` (with `target_met`), `refine` a
`RefinementResult` (with the optional parents); both carry the
`MeshProperties` of the final mesh.

## One time step

`Integrator::integrate(meshed, t0, t1, output, initial)`; the problem
never changes, the state (`TimeState`: U, and for time order 2 also U' and
U'', one value per free unknown) lives inside the call.

```mermaid
sequenceDiagram
    participant I as Integrator
    participant C as StepController
    participant A as Assembler
    participant M as TimeMethod
    participant L as LinearSolver
    I->>A: time_order(meshed)
    A-->>I: the time order, from the formula facts (checked against capabilities().supports)
    Note over I: state = initial, or initial_free_state(t0), none for time order 0
    I->>M: start_needs(time order)
    M-->>I: nothing, static_part or whole_problem
    I->>A: assemble(meshed, t0, max_time_order 0 or 2), only if not nothing
    I->>M: start(assembled at t0, linear solver, state), only if not nothing
    Note over M: Newmark: throws if M2 is zero now, else checks requirements, prepare(M2), solve for U'' at t0
    Note over M: ImplicitEuler at order 0: state = solution of A(t0) U = b(t0)
    loop until t reaches t1
        I->>C: next_step(t) (an error-controlling controller: only for the first step, else the last decision's dt)
        C-->>I: dt
        Note over I: shorten the step to land on a snapshot time or t1
        I->>A: assemble(meshed, t + fraction * dt, previous if any)
        A-->>I: AssembledProblem
        I->>M: step_matrix_weights(dt)
        Note over I: when dt changed or an operator in it depends on t: build the step matrix, check the solver's requirements (always symmetric by the facts, else is_symmetric of its values)
        I->>L: prepare(weighted M2 + B + A), only in that case
        I->>M: advance(assembled, dt, prepared, state)
        M->>L: solve(right-hand side)
        L-->>M: LinearSolveResult
        M-->>I: new state, iterations and residual
        Note over I,C: an error-controlling controller only: M.local_error(before, after, dt, earlier steps), then C.judge: rejected, the same step again from its start with a smaller dt; accepted, the next dt
        Note over I: step record into the summary (kept if keep_step_history)
        Note over I: snapshot if asked: build_solution(meshed, U, t)
    end
    Note over I: return build_solution(meshed, U, t1), snapshots, history, report
```

The `StaticSolver` is the same without the loop: assemble once with
`max_time_order` 0, check the solver's requirements against A's
facts (`always_symmetric`, else `symmetric_now`), one linear solve of stiffness·U = `right_hand_side()`
(= load − A_FD·U_D), then `build_solution`.

## Interfaces and their components

A component is passed in when the object that uses it is built
(`Mesher{discretizer, triangulator, refiner, improvers}`,
`Integrator(method, step_controller, linear_solver)`).

```mermaid
flowchart LR
    subgraph mesh [pde2d_mesh]
        BD([BoundaryDiscretizer]) --- CLD[ChordLengthDiscretizer]
        IT([InitialTriangulator]) --- EAR[EarClipping]
        IT --- CDT[Cdt]
        RF([Refiner]) --- RUP[Ruppert]
        RF --- LEB[LongestEdgeBisection]
        RF --- NVB[NewestVertexBisection]
        RF --- RGB[RedGreenBlueRefinement]
        IMP([Improver]) --- DF[DelaunayFlips]
        IMP --- LS[LaplacianSmoothing]
    end
    subgraph solve [pde2d_solve]
        TMI([TimeMethod]) --- IE[ImplicitEuler]
        TMI --- IM[ImplicitMidpoint]
        TMI --- NM[Newmark]
        LSI([LinearSolver]) --- SLU[SparseLu]
        LSI --- BCG[BiCgStab]
        LSI --- CG[ConjugateGradient]
        LSI --- LDLT[SimplicialLdlt]
        SCI([StepController]) --- FS[FixedStep]
        SCI --- AS[AdaptiveStep]
    end
    subgraph adapt [pde2d_adapt]
        MKS([MarkingStrategy]) --- MXM[MaximumMarking]
        MKS --- DRM[DorflerMarking]
        MKS --- FFM[FixedFractionMarking]
        MKS --- EQM[EquidistributionMarking]
        EST([ErrorEstimator]) --- RES[ResidualEstimator]
        EST --- KEL[KellyEstimator]
        EST --- REC[RecoveryEstimator]
        ADP([Adaptation]) --- MR[MeshRefinement]
    end
```

Rounded boxes are interfaces, rectangles implement them. An `Improver`
offers one or both roles, `LocalImprover` (called by a refiner) and
`FinalImprover` (a pass at the end). The curve kinds are not here: they
are a closed set held in a `std::variant` (see Geometry).

## Modules

```
include/pde2d/geometry/   src/geometry/   ->  library pde2d_geometry
include/pde2d/mesh/       src/mesh/       ->  library pde2d_mesh
include/pde2d/io/         src/io/         ->  library pde2d_io
include/pde2d/fem/        src/fem/        ->  library pde2d_fem
include/pde2d/solve/      src/solve/      ->  library pde2d_solve
include/pde2d/adapt/      src/adapt/      ->  library pde2d_adapt
```

`pde2d_mesh` depends on `pde2d_geometry`, and `pde2d_io` and `pde2d_fem`
each on `pde2d_mesh` (so on both), `pde2d_solve` on `pde2d_fem`, and
`pde2d_adapt` on `pde2d_solve` (and below), never the reverse. Public headers are in
`include/pde2d/<module>/`; everything in `src/` is private, including the
private headers there (`src/mesh/*.hpp`, `src/io/*.hpp`, `src/fem/*.hpp`,
`src/solve/*.hpp`).
`pde2d_geometry`, `pde2d_mesh` and `pde2d_adapt` have no external
dependencies;
`pde2d_io` links libxml2 privately (only `src/io/svg_import.cpp` includes
it), and `pde2d_fem` links SymEngine 0.14 privately (only through the
private header `src/fem/symengine_expression.hpp`), and `pde2d_solve` uses
Eigen 3 (header-only) privately (only in the private header
`src/solve/eigen_view.hpp`, `sparse_lu.cpp` and `bicgstab.cpp`). The tests use
Boost.Multiprecision (header-only) in one place:
`geometry_predicates_oracle` checks the adaptive predicates against an
arbitrary-precision evaluation. The viewer tests need Node.js and are
skipped without it.

Namespaces: `pde2d::geometry`, `pde2d::mesh`, `pde2d::io`, `pde2d::fem`,
`pde2d::solve` and `pde2d::adapt`.

The graphical interface in `app/` (namespace `pde2d::app`) is not a
module: it is a program over the public headers, described in
[app.md](app.md).

### Files

| Path | Contents |
| --- | --- |
| `include/pde2d/geometry/` | `point.hpp`, `predicates.hpp` (exact `orient`, `incircle`), `bounding_box.hpp` (`BoundingBox`), `curve.hpp` (the curve kinds with their core methods, the `CurveKind` concept, `Curve`, functions built on the core), `polygon.hpp`, `curve_geometry.hpp` (`Loop`, `CurveGeometry`), `shapes.hpp` |
| `src/geometry/` | their implementation; `predicates.cpp` is Shewchuk's adaptive arithmetic, compiled with `-fno-fast-math -ffp-contract=off` |
| `include/pde2d/mesh/` | `mesh.hpp` (output `Mesh`, with optional chains and links; the refinement edge convention and `set_longest_edges_as_refinement_edges`), `mesh_spec.hpp` (`LengthBound`, `MeshSpec`), `domain_size.hpp` (what relative lengths multiply), `mesh_properties.hpp` (`MeshProperties`, `measure`, `default_warning_angle_degrees`), `curve_chain.hpp` (`CurveChain`, `CurveLink`, `default_corner_angle_degrees`), `boundary_polygon.hpp`, `working_mesh.hpp`, `report.hpp` (`StageReport`, `Parents`, `MeshResult`, `RefinementResult`), `components.hpp` (interfaces, improver roles, capabilities, `ImproverUse`, `PipelinePlan`, `plan_pipeline`), `delaunay.hpp` (shared flip routines), one header per component (the refiners `ruppert.hpp`, `longest_edge_bisection.hpp`, `newest_vertex_bisection.hpp`, `red_green_blue_refinement.hpp`), `remesher.hpp` (`RemesherOptions`, `Remesher` with `run` and `refine`), `mesher.hpp` (`MesherOptions`, `Mesher`) |
| `src/mesh/` | implementation; private headers `delaunay_internal.hpp`, `polygon_nodes.hpp`, `stage.hpp`, `local_improvers.hpp`, `edge_recovery.hpp`, `loop_chains.hpp` (joining a loop's curves into chains; shared by the discretizer and the Remesher), `boundary_split_policy.hpp` (the fixed, internal policy for splitting curved boundary edges), `boundary_split_steps.hpp` (its steps, a friend of `WorkingMesh`) and `mesh_validity.hpp` (the geometric validity check run by the `WorkingMesh` constructor) and `domain_size_policy.hpp` (the fixed definition of the domain size) and `edge_bisection.hpp` (the bisection core shared by the three bisection refiners: marked edges, closure, division patterns, parents, the `run` loop; `detail::check_marked`) and `refinement_edges.hpp` (the longest-edge tie order `detail::longest_local_edge`, shared by the core and `set_longest_edges_as_refinement_edges`; `detail::set_longest_edges_as_refinement_edges` of a working mesh, for all triangles or those whose order an edit chose; `RefinementEdgeRecord`, a friend of `WorkingMesh`); `bisection_refiners.cpp` holds the three bisection refiners (each a rule for the core) |
| `include/pde2d/io/` | `svg.hpp` (SVG import/export of geometries, SVG pictures of meshes), `pde2d_mesh.hpp` (the old `.pde2d` format), `gmsh.hpp` (Gmsh import) |
| `src/io/` | implementation; libxml2 only in `svg_import.cpp`; private headers `svg_path.hpp` (path data parser), `text.hpp` (files and exact numbers) |
| `include/pde2d/fem/` | `expression.hpp` (`Expression`, `ExpressionFacts`, `Parameters`), `compiled_expression.hpp` (`CompiledExpression`), `problem.hpp` (`Equation`, `BoundaryCondition`, `InitialConditions`, `Problem`, `ConditionKind`, `classify`, the summaries), `equation_text.hpp` (`parse_equation`, `parse_condition`, `equation_text`, `condition_text`), `meshed_problem.hpp` (`MeshedProblem` and its compiled parts), `csr_matrix.hpp` (`SparsityPattern`, `CsrMatrix`, `has_only_zero_entries`, `is_symmetric`), `assembled_problem.hpp` (`Operator`, `AssembledProblem`), `assembler.hpp` (`AssemblerOptions`, `Assembler`), `quadrature.hpp` (`TriangleRule`, `EdgeRule`, `triangle_rule`, `edge_rule`, `point_at`) |
| `src/fem/` | implementation; private headers `symengine_expression.hpp` (the only file that includes SymEngine; `Expression::Impl`), `derivative.hpp` (`detail::derivative`, the only place derivatives are taken), `fact_summaries.hpp` (the summaries, written once for `Problem` and `MeshedProblem`), `element_blocks.hpp` (assembly's pure element block functions) and `threads.hpp` (checking and resolving `AssemblerOptions::threads`; includes `<omp.h>`); `assembler.cpp` holds the driver (with the OpenMP loop over a colour), patterns, colouring and `detail::AssemblyInternals` |
| `include/pde2d/solve/` | `linear_solver.hpp` (`LinearSolver`, `LinearSolverRequirements`, `PreparedLinearSolver`, `LinearSolveResult`), `sparse_lu.hpp`, `bicgstab.hpp`, `conjugate_gradient.hpp`, `simplicial_ldlt.hpp`, `solution.hpp` (`Solution`, `build_solution`), `static_solver.hpp` (`StaticSolver`, `StaticResult`), `time_method.hpp` (`TimeMethod`, `TimeMethodCapabilities`, `StartNeeds`, `StepMatrixWeights`, `TimeState`, `StepSolveInfo`, `ErrorEstimateDeclaration`, `PreviousStep`), `implicit_euler.hpp`, `implicit_midpoint.hpp`, `newmark.hpp`, `step_controller.hpp` (`StepController`, `TakenStep`, `StepDecision`), `fixed_step.hpp`, `adaptive_step.hpp` (`AdaptiveStepOptions`, `AdaptiveStep`), `integrator.hpp` (`Integrator`, `IntegrationOutput`, `FreeState`, `StepAnchor`, `StepRecord`, `IntegrationResult`), `supernodal_cholesky.hpp` (`SupernodalCholeskyOptions`, `SupernodalCholesky`), `multifrontal_lu.hpp` (`MultifrontalLuOptions`, `MultifrontalLu`) |
| `src/solve/` | implementation; private headers `eigen_view.hpp` (`Eigen::Map` views of `CsrMatrix` and vectors), `iterative_solve.hpp` (the prepared solver shared by `BiCgStab` and `ConjugateGradient`), `requirements.hpp` (checking a solver's requirements against assembly's facts), `threads.hpp` (checking and resolving a `threads` option; includes `<omp.h>`), `eigen_threads.hpp` (`detail::EigenThreads`, the scope every Eigen computation runs in), `factorization_threads.hpp` (`detail::FactorizationThreads`, the scope every SuiteSparse call runs in: a process-wide lock, OpenBLAS's count and the calling thread's OpenMP count; the OpenBLAS calls are in `factorization_threads.cpp`), `suitesparse.hpp` (what the two SuiteSparse solvers share: their options check, CHOLMOD's view of a `CsrMatrix` as the compressed columns of its transpose, the checks of a result; includes `cholmod.h`), `pivot_ratio.hpp` (`detail::small_pivot_ratio` = 1e-10 and the warning below it, shared by `SimplicialLdlt`, `SupernodalCholesky` and `MultifrontalLu`; no heavy dependency) and `vectors.hpp` (norms, the residual measures: relative residual and backward error, operator products); Eigen is included only in `eigen_view.hpp`, `eigen_threads.hpp`, `iterative_solve.hpp` and the Eigen solver `.cpp` files (`sparse_lu.cpp`, `bicgstab.cpp`, `conjugate_gradient.cpp`, `simplicial_ldlt.cpp`), SuiteSparse only in `suitesparse.hpp`, `supernodal_cholesky.cpp` and `multifrontal_lu.cpp`; `time_methods.cpp` holds the three time methods with their local error estimates, `fixed_step.cpp` the `StepController` defaults and `FixedStep`, `adaptive_step.cpp` the error-controlled step |
| `app/` | the graphical interface (`pde2d_app`): `src/` (case format with its migrations and generated schema, the geometry sources with the drawing and the boundary locator, the mesh and solve catalogs, the stage cache, runs in chunks per client under two locks, comparisons and probes, exporters with a stored zip writer, problem presets, `--set` and `--sweep`, local HTTP server), `web/` (the page, plain JS modules, with the geometry editor on the picture), `examples/` (cases), `Start pde2d app.cmd` and `start-app.sh` (double-click launcher), `tests/` (`browser/`: the page in headless Chrome; `example_checks.json`: the examples' expected figures), `third_party/cpp-httplib/`; its own guide is `app/AGENTS.md`, its design `docs/app.md` and its decisions `docs/app-decisions.md` |
| `include/pde2d/adapt/` | `error_estimator.hpp` (`ErrorEstimator`, `ErrorEstimate`, `TimeDerivatives`), `residual_estimator.hpp` (`ResidualEstimatorOptions`), `kelly_estimator.hpp` (`KellyEstimatorOptions`), `recovery_estimator.hpp` (`RecoveryEstimatorOptions`, `RecoveryNorm`), `marking.hpp` (`MarkingStrategy`), one header per marking strategy (`maximum_marking.hpp`, `dorfler_marking.hpp`, `fixed_fraction_marking.hpp`, `equidistribution_marking.hpp`), `adaptation.hpp` (`Adaptation`, `AdaptationCapabilities`, `AdaptationResult`, `SolvedState`), `mesh_refinement.hpp` (`MeshRefinement`), `adaptive_static_solver.hpp` (`AdaptiveStaticSolver`, `AdaptiveTarget`, `ErrorTolerance`, `StopReason`, `AdaptiveCycle`, `AdaptiveStaticResult`), `solution_transfer.hpp` (`transfer_solution`, `SolutionTransfer`), `adaptive_integrator.hpp` (`AdaptiveIntegrator`, `AdaptiveIntegratorOptions`, `AdaptationSchedule`, `AdaptiveTimeTarget`, `AdaptationOutcome`, `AdaptationPoint`, `IntegrationPiece`, `AdaptiveIntegrationResult`); refinement of marked triangles is `mesh::Remesher::refine` |
| `src/adapt/` | `error_estimator.cpp` (the shared checks of `estimate`), `residual_estimators.cpp` (the residual terms, shared by the residual and Kelly estimators), `recovery_estimator.cpp`, `marking.cpp` (the shared checks and the four strategies), `mesh_refinement.cpp`, `adaptive_static_solver.cpp`, `solution_transfer.cpp`, `adaptive_integrator.cpp`; private headers `point_location.hpp` (`detail::PointLocator`: a bucket grid over a mesh's triangles and boundary edges, exact containment, nearest boundary point; for the transfer without parents) and `estimator_geometry.hpp` (per-triangle area, diameter and gradient of u_h, interior edges with their two triangles, the triangle of each boundary edge, checked coefficient values, the check of a quadrature degree option) |
| `tests/<module>/` | one test program per topic, registered in that directory's `CMakeLists.txt`; `tests/io/data/` holds hand-written `.msh` files; `tests/mesh/refinement_checks.hpp` holds the oracles of a refinement result (also used by `tests/adapt/cycles.cpp`); `tests/adapt/estimator_support.hpp` computes the true energy error of a P1 solution (exact gradients, subdivision at a singular point) |
| `tests/support/` | `check.hpp` (test helper), `mesh_checks.hpp` (independent mesh oracles, including a brute-force overlap check in `check_valid_mesh`), `shapes.hpp` (test geometries), `l_shape_mesh.hpp` (an L-shape with many colours, for the threaded tests `tests/fem/assembly_threads.cpp` and `tests/solve/threads.cpp`) |
| `tools/` | `test.sh` (configure, build, ctest), `format.sh`, `mesh_viewer/` (the old browser viewer, unchanged except its README) |
| `examples/` | `mesh_viewer/produce_mesh.cpp` (meshes a plate with a hole, writes `.pde2d`), the viewer's startup example files (from the old code) |
| `docs/` | `decisions.md`, `architecture.md`, `vision.md`, `formats/pde2d-mesh.md` (the `.pde2d` format) |

## Geometry

| Type | Role |
| --- | --- |
| `Point` | a point or vector, with `+ - * dot cross norm distance` |
| `Line`, `QuadraticBezier`, `CubicBezier`, `EllipticalArc` | the kinds of exact curve, parameter `t` in [0, 1]; each owns its mathematics as methods (the core, listed in the `CurveKind` concept) |
| `Curve` | one curve of any kind, held by value (a class around a `std::variant` of the kinds); the same core methods, forwarded; `kind()` gives the variant |
| `BoundaryCurve` | a `Curve` and its region name |
| `Loop` | a closed list of `BoundaryCurve`s; built by the user |
| `CurveGeometry` | a validated set of loops with computed depth and parent |

Curve operations are of three sorts:

- **The core**, methods of every kind and of `Curve`: `validate`,
  `point_at`, `derivative_at`, `second_derivative_at`,
  `derivative_bounds`, `split`, `subcurve`, `reversed`, `transformed`,
  `bounding_box`, `exact_length`, `exact_length_parameter`. A method is in
  the core only if it needs the kind's own data. The `CurveKind` concept
  lists them and a `static_assert` checks every kind, so a kind that
  misses one does not compile.
- **Functions built on the core**, written once for every kind:
  `start_point`, `end_point`, `is_line`, `curve_length`,
  `parameter_at_length_fraction`, `bounding_box` of several curves.
  `curve_length` uses the kind's `exact_length` when it returns a value
  (lines, circular arcs) and integrates numerically otherwise.
- **Code that differs by kind but belongs to another module** (SVG path
  commands, mirroring for SVG): `std::visit` over `curve.kind()` with
  `overloaded{...}`, one handler per kind and no catch-all, so a new kind
  does not compile until it is handled there.

The formulas are ported from the previous code. `orient` and `incircle`
are exact predicates: Shewchuk's adaptive-precision `orient2d` and
`incircle` (floating-point filter, then exact expansion arithmetic in
stages), with a slow exact-integer evaluation for the rare undecided case
whose coordinates lie outside [2^-200, 2^200], where the expansion
arithmetic could overflow or underflow. Outside that range the filter's
answer is used only if none of its products underflowed (an underflowed
product, magnified by a later one, can exceed the error bound); the
range check comes before the filter, so no stage runs unprotected. `predicates.cpp` is compiled with
`-fno-fast-math -ffp-contract=off` and refuses to compile under
`__FAST_MATH__` or with `FLT_EVAL_METHOD != 0` (x87 extended precision),
because the error-free transformations need every operation rounded on
its own. `polygon.hpp` has `signed_area` (relative to the first point, so a
small polygon far from the origin does not cancel), `polygon_orientation`
(exact: the turn at the lowest point, decided by `orient`; `CurveGeometry`
and `BoundaryPolygon` decide loop directions with it, because the sign of
an area underflows or overflows at extreme scales),
`locate_in_polygon` and `find_crossing` for polygons.
`bounding_box.hpp`: `BoundingBox{min, max}` with `merged`, `padded`,
`contains`, `width`, `height`, `diagonal`, and `bounding_box` of points.
The box of a curve is its core method `bounding_box()` (end points plus
the exact interior extremes: roots of the Bezier derivatives, axis
extremes of arcs); `CurveGeometry::bounding_box()` is the box of all its
curves. Every axis-aligned box in the library uses it (the
geometry's validation scale, SVG view boxes, the CDT's enclosing
triangle, the validity grid, the polygon sweep, curve linking).

`CurveGeometry` construction:

1. validate each curve and region name, and that every loop is closed
   (within 1e-9 of the geometry's size);
2. flatten every loop to a polyline close to the curves (deviation at most
   1e-6 of the size) and reject any crossing or touching with exact
   predicates;
3. depth of a loop = number of loops containing it; parent = the
   containing loop one level up. Depth and parent live in private vectors
   next to the loops, not in `Loop`;
4. normalize directions: even depth counterclockwise, odd depth clockwise,
   so the domain is always on the left.

Region editing (`rename_region`, `merge_region_into`, `split_region`) is
done by methods, because it must keep the names unique. Shapes (`circle`,
`ellipse`, `rectangle`, `annulus`, `rectangle_with_semicircle`, `polygon`,
`regular_polygon`) are functions returning a `CurveGeometry`.

## Meshing: data flow

```
CurveGeometry ──► BoundaryDiscretizer ──► BoundaryPolygon
                      (MeshSpec)               │ validate()
                                               ▼
                                     InitialTriangulator
                                               │
                                               ▼
                          WorkingMesh ──► Refiner (optional) ──► Improver ... ──► Mesh
                          └──────────────── Remesher ─────────────────────────┘
```

`Mesher{discretizer, triangulator, refiner, improvers}` runs the stages in
this fixed order. The last two run inside a `Remesher{refiner, improvers}`,
which can also start from a `Mesh` that came from anywhere, with or
without the geometry behind it. Improvers are given only to the `Mesher`
(or `Remesher`); the refiner learns which of them to call locally from
the assembled pipeline, never from its own options.

| Type | Role |
| --- | --- |
| `LengthBound{absolute, relative}` | a length limit: optional absolute length, optional multiple of the domain size; the smaller applies, none set means no limit; `resolve(bound, domain_size)`, `validate(bound, name)`; in `mesh_spec.hpp` |
| `MeshSpec{max_edge_length}` | the mesher's only external input besides the geometry; `max_edge_length` is a `LengthBound` with at least one value |
| `CurveChain` | consecutive exact curves treated as one path, measured by arc length `s`; cumulative lengths, `position(s)`, `point_at(s)` |
| `CurveLink{chain, s0, s1}` | which chain (index into the holder's chain list) and arc-length interval an edge approximates, `0 <= s0 < s1 <= length`; in `curve_chain.hpp`, with `default_corner_angle_degrees` (30) |
| `BoundaryPolygon` | loops of straight edges; per edge a region id and an optional `CurveLink`; the chains; region names |
| `WorkingMesh` | the mutable mesh the stages edit |
| `Mesh` | the output: `nodes`, `Triangle{nodes}`, `BoundaryEdge{nodes, region, link}`, `region_names`, `chains` (links and chains empty for an imported mesh); `validate(const Mesh &)` checks it like `WorkingMesh::from_mesh` |
| `ImproverUse` | `local_and_final`, `local_only`, `final_only`: in each improver's options |
| `PipelinePlan` | resolved at construction: indices of the local improvers and of the final passes, and the warnings |
| `StageReport` | `stage`, `time`, `messages`, `metrics` (name → number) |
| `MeshResult` | `mesh`, `reports` (the first is always "pipeline"), `properties` (the final mesh, measured once), `target_met` (`properties.longest_edge <= max_edge_length`), `max_edge_length` and `domain_size` (what the spec resolved to) |
| `RefinementResult` | what `Remesher::refine` returns: `mesh`, `reports`, `properties`, `parents` (`std::optional<Parents>`) |
| `Parents` | `triangles` (input triangle of each output triangle) and `nodes` (`{v, v}` or the input edge `{a, b}` of each output node); see "Parents" below |
| `MeshProperties` | `nodes`, `triangles`, `smallest_angle_degrees`, `largest_angle_degrees`, `shortest_edge`, `longest_edge`, `warning_angle_degrees`, `triangles_below_warning_angle`; from `measure(mesh, warning_angle_degrees)` in `mesh_properties.hpp` |
| `RemesherOptions`, `MesherOptions` | `warning_angle_degrees` (default `default_warning_angle_degrees` = 10, in [0, 60]) |

### Lengths and the domain size

`domain_size(...)` (`domain_size.hpp`) is the length that relative values
multiply. Its definition is a private policy (`domain_size_policy.hpp`,
fixed by the library like the boundary split policy): currently the
diagonal of the axis-aligned bounding box (`geometry::bounding_box`) of
the boundary. It is measured on the geometry's curves, on a mesh's chains
when it has them (the same curves, so exactly the same number), else on
its boundary nodes (an imported mesh: splitting a boundary edge at its
midpoint keeps the box). Settings never know the domain: the discretizer
resolves `max_edge_length` and `max_curve_deviation` against
`domain_size(geometry)`, each refiner resolves `max_edge_length` against
`domain_size(mesh)`, and the `Remesher` does the same for `target_met`.
Each reports `domain_size` and the resolved `max_edge_length` (and
`max_curve_deviation`) as metrics. Nothing is cached (GitHub issue #26).

### Chains

The discretizer joins consecutive curves of a loop into a `CurveChain`
unless the region changes or the boundary turns by more than
`corner_angle_degrees` (default 30°, `default_corner_angle_degrees`) at
the junction. The
turning angle is the angle between the end tangent of one curve and the
start tangent of the next; where a derivative vanishes at that end, the
second derivative's direction is used, then the curve's chord; a junction
without a usable direction is a break. A loop without any break is one
chain from the start of its first curve, so that point (the seam) is
always a node and links never wrap around.

Each chain is sampled as one path: `ceil(length / h)` pieces of equal arc
length, raised while a chord is longer than `h` or a piece may deviate from
its chord by more than `max_curve_deviation` (when set). For a piece across
junctions the bound is the maximum over its curve pieces of
`(1 - u) d0 + u d1 + M u (1 - u) / 2` (`d0, d1` the distances of the curve
piece's ends from the chord, `M` its second-derivative bound), which is
`M / 8` on one curve and does not grow at a smooth junction. Junctions
inside a chain are not forced to be nodes.

A linked boundary edge is split at `s = (s0 + s1) / 2`: binary search over
the chain's cumulative lengths for the curve, then length inversion
(`parameter_at_length_fraction`) inside it. An edge lying on one straight
curve is split at its chord midpoint (the same point without the curve's
rounding); an unlinked edge too. Chains live only on the meshing side.
The chain building is shared private code (`src/mesh/loop_chains.*`), used
by the discretizer and by the `Remesher` when it links a mesh to a
geometry, so both build the same chains.

### WorkingMesh

- Triangles are counterclockwise. Edge `i` of a triangle is opposite its
  vertex `i` (from vertex `i+1` to `i+2`), so the triangle is on its left.
- `neighbour(t, i)` is the triangle across edge `i`, or `no_index`.
- An edge without a neighbour is exactly one boundary edge, oriented with
  the domain on its left, with a region id and an optional curve link.
  Boundary loops never touch, so each boundary node has one outgoing
  boundary edge.
- Edits (`flip`, `insert_in_triangle`, `split_interior_edge`,
  `split_boundary_edge`, `try_move_node`, `set_refinement_edge`) keep all
  of this and return the triangles they touched (`split_interior_edge`:
  the two old indices, then the other half of each).
  `set_refinement_edge(t, edge)` rotates a triangle's vertices (and
  neighbours) so that `edge` becomes edge 0, its refinement edge. An edit that would
  invert a triangle throws and leaves the mesh unchanged.
- **Set vertex orders.** The mesh remembers, privately, each triangle's
  vertices as the constructor received them or `set_refinement_edge`
  last left them (kept through `compact()`). A triangle whose vertices
  differ now had its order chosen by an edit (a flip, an insertion, a
  re-triangulating boundary split); moving a node changes no order. The
  private `src/mesh/refinement_edges.*` (a friend, `RefinementEdgeRecord`)
  reads this for `Remesher::refine` (see "Refinement edges").
- **Deletions.** Splitting a curved boundary edge may delete triangles and
  nodes. They stay as tombstones (`is_triangle_deleted`,
  `is_node_deleted`), so indices held by a refiner stay valid and new
  entries are always appended. `compact()` removes them and renumbers in
  index order; the `Remesher` calls it right after the refiner, so final
  passes never see tombstones, and `to_mesh()` never contains them.
  Boundary edges are never deleted.
- A split boundary edge keeps its region, and its link is split into two
  halves of the arc-length interval.
- The constructor validates everything (orientation, conformity, boundary
  edges matching the triangulation, no unused nodes, indices in range,
  links inside their chains), then the geometry (see "Geometric validity"
  below). So does `from_mesh()`, which the importers and the `Remesher`
  use, and `validate(const Mesh &)` runs the same checks on their own.
- `to_mesh()` lists boundary edges loop by loop in walking order, and
  copies the chains and each edge's link into the `Mesh`. `from_mesh()`
  takes them back, so a mesh refined once can be refined again on the
  true curves without the geometry.

### Geometric validity

A mesh can be connected correctly and still overlap itself, for example a
small triangle lying inside a big one with all six sides as boundary
edges. After the connectivity checks the constructor runs a geometric
check (private, `src/mesh/mesh_validity.*`). It rests on one identity:
when every triangle is counterclockwise and every interior edge is used
once in each direction, the number of triangles covering a point equals
the winding number of the boundary edges around it. So the triangles
overlap nowhere exactly when:

1. **no fold**: at every node, the triangles' angular sectors contain one
   reference direction exactly once (an interior node's triangles turn
   once around it, a boundary node's less than once; the reference is
   the outgoing boundary edge's direction there), decided with `orient`;
2. **no crossing boundary edges**: no two boundary edges cross or touch,
   except consecutive edges at their common node (this also rejects two
   nodes at the same place on the boundary and loops that touch);
3. **no piece inside another**: no node lies in or on a triangle that
   does not use it (exact `orient`), which also rejects a piece in the
   material of another (an island is fine only inside a hole).

Every decision uses exact predicates. Checks 2 and 3 use a uniform grid
over the nodes' bounding box with about one cell per triangle; each
triangle and boundary edge is listed in the cells its bounding box
touches. The cost is O(n) for meshes whose triangles are of similar size;
very long thin triangles or many boundary edges in one cell make it
slower (worst case quadratic) but never wrong. Errors name the node,
triangle or boundary edges ("Node 3 lies in triangle 0 but is not one of
its nodes ...", "Boundary edges 0 (0 -> 1) and 3 (3 -> 4) cross or touch
...", "The triangles around node 0 overlap: they turn around it 2 times
..."). Edits (flips, splits, moves) keep validity by construction and are
not re-checked.

### Splitting a boundary edge

`WorkingMesh::split_boundary_edge(k)` goes through the library's boundary
split policy, a private virtual interface (`src/mesh/boundary_split_policy.*`)
with one fixed implementation; it is not a user option. Its steps are
private too: static functions of `detail::BoundarySplitSteps`
(`src/mesh/boundary_split_steps.*`), which `WorkingMesh` names as a friend
so they can edit its internals. Only `split_boundary_edge`,
`is_unsplittable` and `BoundarySplitResult` are public. The steps, with
`a -> b` the edge, `c` the opposite vertex and `p` the split point:

1. **Direct** (`split_directly`): if `p` lies inside the angle at `c`, join
   it to `c` (two triangles replace one).
2. **Local re-triangulation.**
   - `p` outside the mesh (the curve bulges outwards): add the triangle
     `(a, p, b)` on the edge (`add_cap`), unless a neighbouring boundary
     edge crosses it.
   - `p` inside the mesh (the curve bulges inwards): first check the
     straight triangle `(a, b, p)`: no other boundary edge may cross it,
     no boundary node may lie in it, and no node on its new sides. Then
     insert `p` into the triangle containing it, force `a-p` and `p-b` by
     flips (`detail::recover_edge`, shared with the CDT), and cut off the
     region between them and the old edge with every node in it, which
     lies outside the true domain (`cut_off`). The region is
     bounded by forced edges, so deleting its triangles leaves no hole.
3. **Give up**: change nothing, mark the edge unsplittable
   (`is_unsplittable`) and return the reason. There is no chord fallback
   for linked edges.

Refiners never retry an unsplittable edge; they add a message to their
report and count `unsplittable_edges`, `retriangulated_splits` and
`deleted_nodes`. The target may then be missed, which `target_met` shows.

### Components and capabilities

Every stage is a virtual interface with `name()`. Each concrete component
takes its own options struct (designated initializers) in its constructor.

```cpp
class BoundaryDiscretizer { BoundaryPolygon run(geometry, spec, report); };
class InitialTriangulator { TriangulatorCapabilities capabilities();
                            WorkingMesh run(polygon, report); };
class Refiner             { RefinerCapabilities capabilities();
                            void run(mesh, spec,
                                     std::span<const LocalImprover *const> local,
                                     report);
                            std::optional<Parents> refine(mesh, marked,
                                     std::span<const LocalImprover *const> local,
                                     report); };
class Improver            { std::string name(); ImproverEffects effects();
                            ImproverUse use();
                            const LocalImprover *as_local();   // or null
                            const FinalImprover *as_final(); }; // or null
class LocalImprover       { std::string name();
                            size_t improve_around(mesh, touched); };
class FinalImprover       { void run(mesh, spec, report); }; // spec null in refine
```

Declarations:

- `ImproverEffects{flips_edges, moves_nodes, makes_delaunay,
  keeps_parents}`;
- `RefinerCapabilities{tolerates_flips, tolerates_node_moves,
  maintains_delaunay, keeps_parents}` (what the refiner accepts from local
  improvers, what its output keeps, and whether its own edits keep the
  parents);
- `TriangulatorCapabilities{supports_holes, supports_several_pieces}`;
- `ImproverUse` (from the improver's options) and the roles: `as_local()`
  and `as_final()` return the object to call in that role, or null for a
  role the improver does not have. `DelaunayFlips` derives from all three
  interfaces and returns itself from both; `LaplacianSmoothing` has only
  the final role. `LocalImprover` keeps `name()` because the refiner names
  each local improver in its report.

Checks:

1. **At construction** (`plan_pipeline`, called by `Remesher`, so also by
   `Mesher`): an improver runs locally if its mode allows it, it has a
   local version, and the refiner tolerates its effects (a Delaunay maker
   is refused by a refiner that maintains Delaunay). It runs as a final
   pass if its mode allows it and it has a final role. Both keep the list
   order. A mode that cannot be honoured is a **warning**, one per role,
   for example "delaunay_flips: ruppert already maintains Delaunay, so it
   runs only as a final pass" or "<name>: it has no final version, so it
   runs only locally"; so is a final pass that would make an already
   Delaunay mesh Delaunay (it still runs). Only a null component throws.
2. **When a run starts** (`Mesher::run`): the spec is valid, and the
   triangulator supports the geometry's holes and pieces.

The `Remesher` stores the plan (`plan()`, also on the `Mesher`) and a
"pipeline" `StageReport` (`pipeline_report()`) that every run copies to
the front of its reports. Its first message is the plan in words ("plan:
refiner ruppert; local: none; final: delaunay_flips,
laplacian_smoothing", or "plan: no refiner; ..."), followed by the
warnings; its metrics are `local_improvers` and `final_passes` (counts),
and a run adds `refinement_edges_set` (the triangles it rotated to their
longest edge, see "Refinement edges"); its `time` is the time the
planning took. Like every report it keeps
its structured fields, so times can be summed. The refiner calls
the local improvers after each change and reports the sum of their
returned counts as `local_improver_changes` (and one
`local_improver_changes.<name>` each).

Between stages the `Mesher` validates the `BoundaryPolygon` (no crossing
chords, orientation by depth, nesting), so a discretizer's mistake is
reported as such. After the last stage the `Remesher` measures the final
mesh once (`measure`, giving `MeshProperties`), and everything about the
final mesh is read from that one measurement: the target (`target_met`:
`properties.longest_edge` at most the resolved
`MeshResult::max_edge_length`) and the warning in the pipeline report
when triangles have an angle below `warning_angle_degrees` ("N of M
triangles have an angle below the warning angle of 10 degrees (the
smallest angle is ... degrees)"). Nothing is refused.
Failures are exceptions with specific messages; report messages only mean
"worked, but…".

### v1 components

| Component | File | Notes |
| --- | --- | --- |
| `ChordLengthDiscretizer` | `chord_length_discretizer.*` | chains sampled by equal arc length, chord ≤ h; options `max_curve_deviation` (a `LengthBound`, off by default), `corner_angle_degrees` (30) |
| `EarClipping` | `ear_clipping.*` | per piece: bridge holes, cut best-shaped ears first |
| `Cdt` | `cdt.*` | incremental Delaunay in an enclosing triangle, edge recovery by flips, outside removed, final flips |
| `Ruppert` | `ruppert.*` | makes its input Delaunay, splits encroached/long boundary edges, inserts circumcenters; `refine` inserts the marked triangles' circumcenters; options `min_angle_degrees`, `max_points`; accepts only local improvers without effects; does not keep parents |
| `LongestEdgeBisection` | `longest_edge_bisection.hpp`, `bisection_refiners.cpp` | the bisection core with longest edges (Rivara's longest-edge propagation path); option `max_points`; tolerates local flips and moves; keeps parents |
| `NewestVertexBisection` | `newest_vertex_bisection.hpp`, `bisection_refiners.cpp` | the bisection core with refinement edges; option `max_points`; tolerates local flips and moves; keeps parents |
| `RedGreenBlueRefinement` | `red_green_blue_refinement.hpp`, `bisection_refiners.cpp` | the bisection core marking all three edges, red division; option `max_points`; tolerates local flips and moves; keeps parents |
| `DelaunayFlips` | `delaunay_flips.*` | Lawson flips; both roles (`LocalImprover` and `FinalImprover`); option `use` (default `local_and_final`) |
| `LaplacianSmoothing` | `laplacian_smoothing.*` | interior nodes to neighbour average, inverting moves skipped; final role only; options `iterations`, `use` (default `final_only`) |

Shared routines that components call are plain functions in
`delaunay.hpp`: `is_locally_delaunay`, `flip_to_delaunay`,
`flip_to_delaunay_around`. Private shared code in `src/mesh/`:
`delaunay_internal.hpp` (Lawson with a touched list), `edge_recovery.*`
(forcing an edge by flips), `local_improvers.hpp` (calling and counting a
refiner's local improvers), `boundary_split_policy.*` and
`boundary_split_steps.*` (the policy and its steps), `loop_chains.*`
(joining a loop's curves into chains), `mesh_validity.*` (the geometric
validity check), `edge_bisection.*` (the bisection core shared by the
three bisection refiners: marked edges, closure, division patterns,
parents, the `run` loop; `detail::check_marked`, shared by every
refiner's `refine`).

### Remesher and curve links

`Remesher::run(mesh, spec)` uses the mesh's own chains and links: a mesh
made by the `Mesher` or `Remesher` carries them, so refining it again puts
new boundary points on the true curves. An imported mesh has none, and its
boundary edges are split at chord midpoints.

`Remesher::run(mesh, geometry, spec, corner_angle_degrees = 30)` is for a
mesh without links (any links it has are replaced). It first joins the
geometry's loops into chains with the same code and rule as the
discretizer, then links each boundary edge to the chain of its region that
contains both end points (within 1e-9 of the mesh size), taking the short
way along the chain. An edge whose ends lie on two different chains
(across a corner, a region change or the start of a closed loop) stays
unlinked and is split at its chord midpoint; the "curve_links" report
(right after "pipeline") counts `chains`, `linked_edges` and
`unlinked_edges`. A mesh region missing from the geometry, a boundary node
off its region's curves, or an angle outside [0, 180] is an error.

## Input and output (`pde2d_io`)

```
SVG file ──import_svg──► CurveGeometry ──export_svg──► SVG file
                                         (Mesher)
.msh file ──import_gmsh──► Mesh ──export_pde2d──► .pde2d ──► tools/mesh_viewer
.pde2d file ──import_pde2d──►  └──export_mesh_svg──► SVG picture
```

Every function has a `std::istream&` / `std::ostream&` form and a
`std::filesystem::path` form. Problems in the content throw
`std::invalid_argument` with a specific message; a file that cannot be
opened, read or written throws `std::runtime_error`.

| Function | Header | Notes |
| --- | --- | --- |
| `import_svg` | `svg.hpp` | `<path>` elements; all path commands onto `Line`, `QuadraticBezier`, `CubicBezier`, `EllipticalArc` (arcs converted on the unit circle, so any radius works; a nearly flat arc, sweep below 0.01 rad, becomes a `CubicBezier` through exactly its end points); region = `data-region`, else `id`; subpaths joined into loops by end points (a run may be reversed); holes by depth; `<g>` entered, metadata ignored; transforms, shapes, nested `<svg>`, DTDs rejected; user units; y flipped, (x, y) → (x, −y), after the runs are joined (so error messages give file coordinates); `CurveGeometry` then normalizes the reversed loop directions |
| `export_svg` | `svg.hpp` | points written as (x, −y), so `import_svg(export_svg(g))` gives the same numbers back; one `<path>` per run of one region in a loop; arcs in pieces of at most 90°; id = region name if unique and a valid XML id, else `pde2d_run_<k>` with `data-region`; shortest exact numbers |
| `export_mesh_svg` | `svg.hpp` | nodes drawn at (x, −y), the right way up like `export_svg`; triangles (light fill, grey edges) and one coloured `<path>` per region with `data-region` and a `<title>`; no nodes, no links |
| `export_pde2d` | `pde2d_mesh.hpp` | the old `PDE2D_MESH` format (`docs/formats/pde2d-mesh.md`): version 2 for one loop, 5 with `RESULT 0` for several; outer loop (through the lexicographically smallest boundary node) first; links dropped |
| `import_pde2d` | `pde2d_mesh.hpp` | versions 2–5; v1 and untagged edges rejected; result trailers checked and ignored; validated like a `WorkingMesh` (with the geometric check); records read as they come, never allocated for a claimed count first; triangles keep the file's vertex order (their refinement edges); no links |
| `import_gmsh` | `gmsh.hpp` | MSH 4.1 and 2.2 ASCII; 3-node triangles only; boundary edges must lie on line elements of one physical group of dimension 1 (its name, or `physical_<tag>`); triangles made counterclockwise, each rotated to its longest edge as refinement edge (`set_longest_edges_as_refinement_edges`); unused nodes dropped; no links |

Imported meshes have no chains or links, so the `Remesher` splits their
boundary edges at chord midpoints unless it is given the geometry.
Private code: `svg_path.*` (the path data parser, the SVG arc
conversion and the y mirror `mirrored_y`), `text.hpp` (files, exact number parsing and printing).

The mesh viewer (`tools/mesh_viewer/`) is the previous code's viewer,
unchanged. `examples/mesh_viewer/produce_mesh` writes a mesh for it.

## Problems (`pde2d_fem`)

```
text ──► Expression ─┐
                     ├─► Problem ──┐
         Parameters ─┘             ├─► MeshedProblem ──► Assembler ──► AssembledProblem
                   Mesh (shared) ──┘                      (at t, optional previous)
```

| Type | Header | Role |
| --- | --- | --- |
| `Expression` | `expression.hpp` | a scalar expression in x, y, t and named parameters, built from text (`Expression("1 + k*x*y")`); immutable, copies share the parsed form; `text()`, `facts()`; `Expression()` is zero |
| `ExpressionFacts` | `expression.hpp` | `zero`, `depends_on_x/y/t`, as written |
| `Parameters` | `expression.hpp` | `std::map<std::string, double>`: name → value |
| `CompiledExpression` | `compiled_expression.hpp` | an `Expression` with its parameters substituted, compiled to a function `(x, y, t) -> double`; keeps the expression's facts; `CompiledExpression()` is zero |
| `Equation` | `problem.hpp` | the coefficients `a_xx a_xy a_yy b_x b_y c m1 m2 f` of `a_xx u_xx + a_xy u_xy + a_yy u_yy + b_x u_x + b_y u_y + c u + m1 u_t + m2 u_tt = f`; unset ones are zero |
| `BoundaryCondition` | `problem.hpp` | `alpha beta_n beta_tau beta_x beta_y gamma g` of `alpha u + beta_n u_n + beta_tau u_tau + beta_x u_x + beta_y u_y + gamma u_t = g`, and `priority` (default 0) |
| `InitialConditions` | `problem.hpp` | `std::optional<Expression>` `u` and `u_t` |
| `Problem` | `problem.hpp` | `equation`, `parameters`, `boundary_conditions` (region name → condition), `initial` |
| `ConditionKind` | `problem.hpp` | `dirichlet`, `natural`, `dynamic`; `classify(condition)` and `is_oblique(condition)` |
| (functions) | `equation_text.hpp` | `parse_equation(text)` → `Equation`, `parse_condition(text)` → `BoundaryCondition` (one line each, with grad, div, lap, dot or ∇, ∇·, ∇², Δ, vectors and matrices; see "Equations as one line"), and back: `equation_text(equation)`, `condition_text(condition)` |
| `CompiledEquation`, `CompiledCondition` | `meshed_problem.hpp` | the same members as `CompiledExpression`s; a condition also has its `kind`, `oblique` and `priority` |
| `DirichletNode` | `meshed_problem.hpp` | a node and the regions whose `g / alpha` is averaged there (one, or two at an equal-priority corner) |
| `MeshedProblem` | `meshed_problem.hpp` | `Problem` + `std::shared_ptr<const Mesh>`, checked and compiled, up to assembly |
| `CompiledDivergence` | `meshed_problem.hpp` | `x`, `y`: the compiled divergence of A = [[a_xx, a_xy/2], [a_xy/2, a_yy]] (`MeshedProblem::divergence_of_a()`) |
| `SparsityPattern`, `CsrMatrix` | `csr_matrix.hpp` | compressed sparse rows: `rows`, `columns`, `row_offsets`, `column_indices` (signed `Index`, sorted and unique per row), `find`; a matrix is a shared pattern plus `values`, with `entry(i, j)` and `multiply(x)`; the plain functions `has_only_zero_entries(matrix)` and `is_symmetric(matrix, relative_tolerance = 1e-12)` (\|a_ij − a_ji\| ≤ tol · max \|a\|, a missing entry counts as 0) read facts from the values |
| `Operator` | `assembled_problem.hpp` | one operator of the system: `free` (X_FF), `dirichlet` (X_FD); formula facts for every t: `depends_on_t`, `always_zero`, `always_symmetric` (of `free`, from the terms built); number facts at the assembled t: `zero_now`, `symmetric_now` (of `free`, from its values) |
| `AssembledProblem` | `assembled_problem.hpp` | the system M2 U'' + B U' + A U = b in the free unknowns at one t |
| `Assembler` | `assembler.hpp` | `assemble(meshed_problem, t, previous = nullptr, max_time_order = 2)`; `time_order(meshed_problem)`, the assembled time order from the formula facts, evaluating nothing |

```cpp
using namespace pde2d::fem;
Problem problem{
    .equation = {.a_xx = Expression("-k"), .a_yy = Expression("-k"),
                 .f = Expression("1")},
    .parameters = {{"k", 1000.0}},
    .boundary_conditions = {
        {"wall", {.alpha = Expression("1"), .g = Expression("0")}},
        {"inlet", {.beta_n = Expression("1"), .g = Expression("x*y")}}}};
auto meshed = std::make_shared<const MeshedProblem>(mesh, problem);
double a = meshed->equation().a_xx(0.5, 0.5, 0.0); // -1000
```

**Equations as one line** (`equation_text.hpp`, `src/fem/equation_text.cpp`).
`parse_equation("u_t - ∇·(k∇u) = 1")` reads a line into an `Equation`,
`parse_condition("k*u_n + h*u = h*u_out")` into a `BoundaryCondition`;
nothing after them knows the line. A small recursive-descent parser of
its own reads the structure (`=`, `+ - * / ^`, `·`, vectors `(a, b)`,
matrices `[[a, b], [c, d]]`, `grad div lap dot`, `∇ ∇· ∇² Δ`, a factor in
front of ∇ or Δ); every part without u, the normal or an operator of the
line (a number, a name, a function call with its arguments, and `+ - * /
^` and parentheses of such) is handed to `Expression` as it is written,
so coefficients keep Expression's syntax, allow-list and messages
(`Piecewise` and its conditions included). The line is evaluated to a
form linear in u: a SymEngine coefficient per derivative of u (u, u_x,
u_y, u_xx, u_xy, u_yy, u_t, u_tt) and, in a condition, per component
n_x, n_y of the normal; a product of two parts with u, a function of u
or a third derivative is refused where it is written. grad and div
differentiate by the product rule, the coefficients through
`detail::derivative`, so `-∇·(k∇u)` becomes a = −k, b = −∇k. The left
side minus the right side gives the coefficients and −(the part without
u) the right-hand side; a condition's normal components must form
β_n u_n + β_τ u_τ (the u_x n_x and u_y n_y coefficients equal, the u_x
n_y one the negative of the u_y n_x one), after `expand`. Precedence, loosest first: `+ -`, `·`, `* /`, a sign, a factor in
front of ∇ or Δ, `^`, then ∇ and Δ, whose operand is a power or such a
product: `n·k∇u` is `n·(k∇u)`, `∇·k∇u` is `∇·(k∇u)`, `Δx^2` is
`Δ(x^2)`, and `1/2 Δu` is refused with a hint. A no-break space is a
space. Coefficients are built with `detail::expression_from_form`
(checked, printed by `detail::form_text`: SymEngine's printer with every
floating number in full, so the text reads back to the same doubles).
`equation_text` and `condition_text` print the coefficient form back
from the parsed forms, which `parse_*` reads to the same coefficients;
they throw `std::invalid_argument` when a coefficient uses a parameter
the line would read otherwise (u, u_x, ...; n and tau in a condition).

**Expressions.** SymEngine parses the text when an `Expression` is built
(`parse`, with `^` as power). Floating literals are restored in a bottom-up
pass that treats exactly integral power exponents as integers, including
after parameter substitution. Constant descendants are checked even when
a finite parent or substitution would hide them. The parsed form is then checked against the
allow-list (`detail::unsupported`, `src/fem/expression.cpp`), walking it
the way SymEngine's evaluator does: numbers (each must be finite as a
double, so `1e1000` and `10^400` are rejected), x, y, t, parameter names,
`pi` and `E`; `+ - * /` and powers; `sin cos tan asin acos atan sinh
cosh tanh exp log sqrt abs`; `Piecewise` whose last branch is `(value,
True)`, with conditions built from `< <= > >= == !=` (`Eq`, `Ne`) and
`And`, `Or`, `Not`, allowed only as `Piecewise` conditions. Anything else
throws `std::invalid_argument` naming it: a parse error, an unknown
function (`foo(x)`), a construct not on the list (`loggamma`, `Xor`,
`max`, `EulerGamma`; `atan2` is refused by name before parsing, because
SymEngine's parser turns `atan2(x, x)` into `pi/4` whatever the sign of
x), a complex value (`I`, `sqrt(-1)`), an infinite or undefined constant
(`oo`, `1/0`, `nan`), a constant part whose value is not a finite real
double (`exp(1000)`, `asin(2)`; evaluated once, also after parameters are
substituted), a condition used as a number
(`2*(x < 0)`), a `Piecewise` without a final unconditional branch (SymEngine's evaluator
would read past its branches). Constructs are added to the list one at a
time, each with a test against the standard library
(`tests/fem/expression.cpp`) and a tested derivative
(`tests/fem/derivative.cpp`, and in t through `MeshedProblem`). The facts are read off the parsed form:
`zero` when it is the number zero (SymEngine's parser already combines
like terms, so `x - x` is zero, but nothing more is simplified),
`depends_on_*` when the symbol is free in it. Free symbols other than x,
y, t are parameter names, unknown constants for the facts.
`CompiledExpression` checks the parameters (a plain symbol name other than
x, y, t, which the parser reads back as that symbol, so not `pi` or `E`; a
finite value), substitutes the values (an unknown name throws), checks the
substituted form again (`log(k)` with k = -1 is complex, `1/k` with k = 0
is infinite) and prepares SymEngine's `LambdaRealDoubleVisitor` (a tree of
`std::function`s; no LLVM in this build). A call does not check its
result. Calls may run in several threads at once: they only read the
prepared functions, and every allowed construct evaluates through standard
math functions without shared state (`tests/fem/expression_threads.cpp`,
also run under ThreadSanitizer and Helgrind). SymEngine itself is built
without thread-safe reference counting, so all symbolic work (building,
substituting, differentiating, compiling, dropping the last copy of an
expression) happens in one thread.

**Derivatives** are taken in one place, the private
`detail::derivative(expression, Variable::x | y | t)`
(`src/fem/derivative.hpp`). It runs SymEngine's differentiation with two
rules of its own: `abs`, which SymEngine leaves as an unevaluated
`Derivative`, becomes `d|u| = sign(u) du` with the sign written as
`Piecewise((1, 0 < u), (-1, u < 0), (0, True))` (0 at the kink), so
`d/dt abs(t - 1)` is `-Piecewise((1, 0 < 1 - t), (-1, 1 - t < 0), (0,
True))`; and a `Piecewise` is differentiated branch by branch with its
conditions kept, collapsing to the common value when all branches give
the same derivative. The expression itself keeps its `abs`; only its
derivative contains the sign. The result is checked against the
allow-list like text (`expression_from_form`), so a derivative that leaves
it (`d/dx 1e308*x^2` has an infinite number) throws. Every allowed
construct has a derivative inside the allow-list. It is used for
`d/dt (g / alpha)` (`time_derivative_of_quotient`), its second time
derivative, and the divergence of A (`divergence_of_a`), all in the
`MeshedProblem` constructor.

**Classification** (`classify`), by which coefficients are zero as
written: no spatial derivative and no `gamma` is Dirichlet (`alpha` must
be set: the value is `g / alpha`); a derivative with a normal part
(`beta_n`, `beta_x` or `beta_y`) is natural, or dynamic with `gamma`.
Rejected: no term at all, a boundary ODE (`gamma` without a spatial
derivative) and tangential only (`beta_tau` alone). `is_oblique`: `beta_tau`,
`beta_x` or `beta_y` is set.

**Summaries** are functions, computed from the facts every time, for a
`Problem` and for a `MeshedProblem` (one private template,
`src/fem/fact_summaries.hpp`): `time_order` (2 with `m2`, else 1 with `m1`
or any `gamma`, else 0), `coefficients_depend_on_t`, `data_depends_on_t` (`f` or
any `g`).

**`MeshedProblem` construction.** `mesh::validate` runs first, then:

1. region names are matched both ways (a condition for an unknown region
   or a mesh region without a condition is an error);
2. every condition is classified; `conditions()` is indexed by region id;
3. Dirichlet nodes: each boundary node has one incoming and one outgoing
   boundary edge; it is Dirichlet if either edge's region is. With one
   Dirichlet region it takes that one (Dirichlet beats natural and
   dynamic); with two, the higher `priority` wins and equal priorities are
   averaged. The other nodes are the free nodes, in index order: unknown
   `i` of the shrunk system is node `free_nodes()[i]`, and `free_index`
   goes back;
4. every expression is compiled with the parameters; an error names the
   coefficient ("Region 'wall', coefficient g: ..."). The
   time-independent winning boundary data is checked for finiteness at
   every Dirichlet node, independently of initial conditions. The
   competing values at Dirichlet corners are then compared: equal as written, never
   reported; otherwise, if one depends on t, reported as "may differ";
   otherwise evaluated at the node and reported when they differ;
5. the initial conditions that `time_order` requires are compiled (a
   missing one is an error; one that is not needed is ignored and one that
   depends on t is evaluated at the start time, each with a report
   message); for time order 1 or 2 so is `d/dt (g / alpha)` of every
   Dirichlet condition that depends on t, and for time order 2 its second
   derivative (with `detail::derivative`; a derivative outside the
   allow-list is an error). Each compiled initial
   condition is then compared with the boundary data that replaces it at
   the Dirichlet nodes, u with `g / alpha` and u_t with `d/dt (g /
   alpha)`, like the corners: when neither depends on t, both are
   evaluated at every Dirichlet node and compared exactly (the boundary
   data must be finite there, or it is an error), and a difference is one
   message with the count and the first three nodes; otherwise one message
   says they may differ;
6. the divergence of A is differentiated (`detail::divergence_of_a`,
   through `detail::derivative`) and compiled: `divergence_of_a()`;
7. the facts stay with each compiled function, and the summaries read
   them.

`dirichlet_value(k, t)` is the average of `g / alpha` over the regions of
`dirichlet_nodes()[k]` (with `std::midpoint`, so two large finite values
do not overflow; an error if not finite). `dirichlet_rate(k, t)` and
`dirichlet_acceleration(k, t)` average the first and second time
derivatives the same way; they exist for time order 1 or 2 and for time
order 2 respectively and throw `std::invalid_argument` below that. `initial_values(t0)` and
`initial_velocities(t0)` return `std::optional<std::vector<double>>`: u
and u_t at every mesh node at the start time t0 the time integrator
passes, computed on every call, or an empty optional when the time order
has no such values (no values for time order 0, no velocities below 2). At
a Dirichlet node they come from the boundary data (`dirichlet_value(k,
t0)`, and the average of `d/dt (g / alpha)` for the velocity), elsewhere
from the initial condition at `(x, y, t0)`. A t0 that is not finite (for
any time order) or a value that is not finite throws
`std::invalid_argument`. `initial_free_state(t0)` returns the same
initial conditions at the free nodes only (a `FreeInitialState` with
`values` and, for time order 2, `velocities`, indexed like
`free_nodes()`), evaluating no boundary data: the integrator starts from
it, so Dirichlet data whose time derivative is not finite at t0 (u = √t
from t0 = 0) stops only a method whose start reads the problem at t0:

```cpp
if (const auto u0 = meshed->initial_values(0.0)) // empty for time order 0
  for (std::size_t node = 0; node < u0->size(); ++node)
    use((*u0)[node]);
```

`report()` is a `mesh::StageReport` named "meshed_problem" with the time, the metrics
`regions`, `dirichlet_nodes`, `free_nodes`, `corners`,
`dirichlet_over_natural_corners`, `priority_corners`, `averaged_corners`,
`differing_corners`, `differing_initial_values` and
`differing_initial_velocities` (Dirichlet nodes where the initial
condition differs from the boundary data, or all of them when it may
differ), one message per differing corner, for example
"Corner at node 0 (0, 0): regions 'bottom' and 'left' (both priority 0)
give 0 and 1; their average 0.5 is used", one per initial condition
that is not needed or depends on t, and one per initial condition that
differs from the boundary data, for example "The initial condition u
differs from g / alpha at 4 of 5 Dirichlet nodes, where g / alpha is used:
node 0 (0, 0) has u = 1, g / alpha = 0; node 1 (0.5, 0) has u = 1, g /
alpha = 0.5; node 3 (0, 0.5) has u = 1, g / alpha = 0; and 1 more", or
may differ: "The initial condition u and g / alpha may differ at the 8
Dirichlet nodes, where g / alpha is used; one of them depends on t, so
whether they differ depends on the start time t0".

## Assembly (`pde2d_fem`)

```cpp
auto meshed = std::make_shared<const MeshedProblem>(mesh, problem);
const Assembler assembler;
AssembledProblem a = assembler.assemble(meshed, 0.0);
// static: a.stiffness().free * U = a.right_hand_side()
AssembledProblem b = assembler.assemble(meshed, 0.1, &a); // reuses a
// Only A and the load, and only U_D (no U_D', U_D''): time order 0.
AssembledProblem c = assembler.assemble(meshed, 0.0, nullptr, 0);
// The elements of each colour on 4 threads: the same bits as a.
AssembledProblem d = Assembler({.threads = 4}).assemble(meshed, 0.0);
```

The system in the free unknowns (`MeshedProblem::free_nodes()`, the
Dirichlet nodes eliminated) is

```
M2 U'' + B U' + A U = b,   B = M1 - D,   A = -K + C + R - S + T,
b = (F - G) - A_FD U_D - B_FD U_D' - M2_FD U_D''
```

`mass()`, `damping()` and `stiffness()` are `Operator`s, each with a
`free` matrix (free rows and columns) and a `dirichlet` matrix (free rows,
column k is `dirichlet_nodes()[k]`). All `free` matrices share one
`SparsityPattern` object and all `dirichlet` matrices another; a solver can
view the arrays with `Eigen::Map` (the index type is signed). `load()` is
F - G, `dirichlet_values()`, `dirichlet_rates()` and
`dirichlet_accelerations()` are U_D, U_D' and U_D'' (rates only for
`time_order() >= 1`, accelerations only for 2, otherwise empty), and
`right_hand_side()` is b (a lifting product is left out when its operator
is zero).

**max_time_order** (0, 1 or 2, default 2; anything else throws
`std::invalid_argument`). 1 leaves out M2 and U_D''; 0 also M1 and D and
U_D': only A (K, C, R, S, T) and the load are built and only U_D is
evaluated, so b = load − A_FD U_D. A left-out operator is a zero operator,
so `time_order()` is at most `max_time_order`. The `StaticSolver` uses 0,
so it never evaluates a derivative of the boundary data it drops (d/dt
√t at t = 0, GitHub issue #35).

**Terms.** Interior, per triangle: K (a_xx, a_xy, a_yy), C (b and
`divergence_of_a()`), R (c), M1 (m1), M2 (m2), F (f). Boundary, per edge
of a natural or dynamic region: S (alpha), T (the tangential part), D
(gamma), G (g). On an edge with outward normal n and tangent tau (the
domain on the left), with κ_n = nᵀAn, κ_τ = nᵀAτ,
β̂_n = beta_n + (beta_x, beta_y)·n and β̂_τ = beta_tau + (beta_x, beta_y)·τ,
the condition is solved for u_n and substituted into (A∇u)·n = κ_n u_n +
κ_τ u_τ:

```
S_ij = ∫ κ_n alpha / β̂_n φ_i φ_j      T_ij = ∫ (κ_τ - κ_n β̂_τ / β̂_n) φ_i ∂τ φ_j
D_ij = ∫ κ_n gamma / β̂_n φ_i φ_j      G_i  = ∫ κ_n g / β̂_n φ_i
```

β̂_n = 0 at a quadrature point throws. Dirichlet regions have no boundary
term. Quadrature comes from `quadrature.hpp` (the only place with points
and weights): `triangle_rule(2)`, barycentric (2/3, 1/6, 1/6) and
rotations, and `edge_rule(3)`, 2-point Gauss–Legendre. Each computation
asks for the exact degree it needs; there is no global quadrature
setting. Available: triangles up to degree 5 (1, 3, 6 and 7 points),
edges up to degree 9 (Gauss–Legendre with 1 to 5 points), all with
positive weights and points inside.

**Facts.** Per term, from the facts of the coefficients feeding it
(zero as written, depends on t): K from A; C from b and div A; a boundary
term is zero when A is (κ = 0) or its own coefficient (alpha, gamma, g)
is. T is zero when A is, or when A is isotropic
(`MeshedProblem::a_is_isotropic()`: a_xy zero and a_xx − a_yy expands to
zero, decided symbolically in the `MeshedProblem` constructor; then
κ_τ = 0) and beta_tau, beta_x, beta_y are zero (then β̂_τ = 0). A term
depends on t when one of its non-zero factors (including β̂_n, and β̂_τ
for T) does. An operator is `always_zero` when every
term of it is zero and `depends_on_t` when one of them depends on t;
`time_order()` is 2 with a mass that is not always zero, else 1 with such
a damping, else 0 (lower than the problem's when a dynamic condition has
A = 0); `Assembler::time_order(meshed)` gives the same without
assembling. `always_symmetric`: M2 and B always (mass-like integrals), A
when C and every T are zero (K, R and S are symmetric); a zero operator
is symmetric. It is a fact from the terms, not a test of the values, and
holds up to rounding (the blocks are computed in full). The facts from
the numbers hold at the assembled t only: `zero_now` when every value of
`free` is zero (m2 = sin(t) at t = 0), `symmetric_now` when `free` is
symmetric to the default tolerance of `is_symmetric` (an anisotropic
diagonal A whose boundary edges are all axis-aligned, where T
vanishes); each is true when its formula fact is. Wherever A is not zero the
condition of a natural or dynamic edge is solved for u_n, so β̂_n ≠ 0 is
checked there even when every term of the edge is zero.
`load_depends_on_t()` reads F and G; `right_hand_side_depends_on_t()`
adds the lifting products: a non-zero operator with Dirichlet nodes whose
values (g / alpha of a used region) or, for the stiffness, whose own terms
depend on t.

**Structure** (private, in `assembler.cpp`). Each result has a small
`detail::AssemblyInternals`: the `max_time_order` it was built with, the
constant parts of the operators and load that have terms depending on t
(shared pointers, null otherwise), and a shared
`detail::AssemblyStructure`, built once and carried through `previous`:
the free index and the Dirichlet column of
every node; the two patterns, from one loop over the triangles (every node
pair (p, q) with p free adds column q to row p in the free or the
Dirichlet pattern; rows sorted, duplicates removed; boundary edges add
nothing); a greedy colouring of the triangles in index order (the smallest
colour not used by an earlier triangle sharing a node) and, in a separate
pass, of the boundary edges.

**Building.** The driver loops over the colours. With one thread it runs
the elements of a colour in index order; with more
(`Assembler(AssemblerOptions{.threads = n})`), an OpenMP loop shares them
among n threads (`for_each_in_colour`). The elements of a colour share no
node, so each slot of a matrix and each load entry receives at most one
addition per colour, and the additions to it come in colour order for
every thread count: the result is the same bit for bit. An exception (a
coefficient that is not finite) cannot leave an OpenMP thread; each is
caught, and after the colour the one with the lowest element index is
thrown again, which is the one the serial loop throws. For each element it calls a pure block function
(`detail::triangle_blocks`, `detail::edge_blocks` in
`src/fem/element_blocks.cpp`), which computes the full 3 x 3 (2 x 2)
blocks of the requested terms, summed per operator and split into a
constant set and a time set; then it scatters them: a Dirichlet row is
skipped, a Dirichlet column goes to the `dirichlet` matrix, the rest to
`free`, each slot found by binary search in its row. A constant term goes
to the constant arrays, a term depending on t to the time arrays. Each
total is the constant part, or constant + time elementwise when the
operator has terms in t (the constant part is then kept for reuse). With
`previous` (same `MeshedProblem` object, else `std::invalid_argument`),
the structure and the constant parts are taken over and only the terms in
t are computed; an operator without such terms is copied from `previous`.
An operator above `previous`'s `max_time_order` was not built there and is
built here in full. The result is the same bit for bit as a fresh
assembly at that t and `max_time_order`.

**Report** ("assembly"): time; metrics `free_nodes`, `dirichlet_nodes`,
`free_nonzeros`, `dirichlet_nonzeros`, `triangle_colours`, `edge_colours`,
`used_previous`, `max_time_order`, `threads` (the count used, 0
resolved), and `mass_rebuilt`, `damping_rebuilt`,
`stiffness_rebuilt`, `load_rebuilt` (1 when something of it was computed,
0 when reused whole or left out by `max_time_order`).

## Solvers and time integration (`pde2d_solve`)

```cpp
auto meshed = std::make_shared<const MeshedProblem>(mesh, problem);

// Static: the time terms dropped, A U = load - A_FD U_D at t.
StaticResult s = StaticSolver(std::make_shared<SparseLu>()).solve(meshed);

// In time.
const Integrator integrator(std::make_shared<ImplicitMidpoint>(),
                            std::make_shared<FixedStep>(FixedStepOptions{.dt = 0.01}),
                            std::make_shared<BiCgStab>());
IntegrationResult r = integrator.integrate(meshed, 0.0, 1.0,
                                           {.snapshot_times = {0.5}});
// Continuing the run.
IntegrationResult r2 = integrator.integrate(meshed, 1.0, 2.0, {}, &r.final);
```

**Linear solvers** (`linear_solver.hpp`). `LinearSolver::prepare(
std::shared_ptr<const CsrMatrix>)` returns a
`std::unique_ptr<const PreparedLinearSolver>`, which holds the matrix and
solves `solve(rhs) -> LinearSolveResult` as often as needed: values,
iterations, `residual` (the relative residual |b − Ax|₂ / |b|₂ computed
after the solve), `threads` (the resolved option of the iterative and the
parallel direct solvers, 1 for the Eigen direct solvers), `backward_error` (|b − Ax|∞ / (|A|∞ |x|∞ + |b|∞), |A|∞
the largest absolute row sum, from the same residual; every solver fills
it; a residual entry that is not finite makes it NaN, which no
tolerance accepts), `pivot_ratio` (the smallest pivot over the largest,
from `SimplicialLdlt`, `SupernodalCholesky` and `MultifrontalLu`; 0 when
not measured) and `messages` (warnings, "worked, but…", each naming the
solver; empty except for `SupernodalCholesky` on a matrix it factored as
−A, and for the three solvers that measure `pivot_ratio` when it is below
`detail::small_pivot_ratio` = 1e-10 (`src/solve/pivot_ratio.hpp`, fixed,
not an option): "<solver>: the matrix is nearly singular (pivot ratio
below 1e-10; its value is the result's pivot_ratio, in reports
pivot_ratio or min_pivot_ratio), so the answer may have few correct
digits; …", made once in `prepare` by `detail::pivot_ratio_warning` and
added to every solve's result, after the −A warning; the text names the
threshold, not the ratio, so it is the same for every solve of a solver
(a step matrix prepared again at every step gives it once in the
integrator's report); the answer is
returned and the backward error checked as before). The members are in this order: the first four (values, iterations,
residual, threads) are those the struct had before the other three were
added, so a positional initialization written then keeps its meaning;
new members go at the end. The `StaticSolver`
copies the messages into its "static_solver" report; the `Integrator`
passes its time method a private wrapper of the linear solver
(`LoggingSolver` in `integrator.cpp`) whose prepared solvers
(`LoggingPreparedSolver`) log every solve's result, the steps' and
those of `start()`: its "integrator" report gets each distinct
message once, in the order they were added, and the largest
`backward_error` (`max_backward_error`) and smallest `pivot_ratio` above
0 (`min_pivot_ratio`, 0 when no solve measured one) over those solves,
all three read from the log in one snapshot (`SolveLog::snapshot`, under
one lock), so that a solve of a prepared solver kept beyond
`integrate()` cannot make them disagree.
The log (`SolveLog`) is
per-run state shared (`std::shared_ptr`) by the wrapper and every
prepared solver it made, and guarded by a mutex held only to add or copy,
never during a solve: a time method may solve from several threads at
once with one prepared solver (solves that overlap add their messages in
the order they take the lock), and may keep a prepared solver beyond
`integrate()` (a later solve works and adds to a log that no report reads
any more).
A solver declares `requirements()` (`LinearSolverRequirements{
symmetric_matrix}`); `prepare` sees only values, so the callers (the
`Integrator`, the `StaticSolver`, `Newmark::start`, `ImplicitEuler::start`)
check the requirements first (`detail::check_requirements`,
`src/solve/requirements.hpp`): a symmetry requirement is met when the
matrix is always symmetric by assembly's facts or symmetric now by its
values (`Operator::symmetric_now`, or `is_symmetric` of a step matrix,
`detail::check_step_matrix_requirements`). They throw
`std::invalid_argument` naming the caller, the solver and the matrix.
Definiteness is not a declared requirement: a solver that needs it
detects its failure and throws `std::runtime_error`.

| Solver | Requires | What it is |
| --- | --- | --- |
| `SparseLu` | nothing | Eigen's SparseLU with COLAMD; the matrix is copied once into Eigen's column-major format; a zero pivot throws `std::runtime_error` in `prepare`, a result that is not finite throws in `solve` |
| `BiCgStab` | nothing | `BiCgStabOptions{.tolerance = 1e-10, .max_iterations = 1000, .threads = 1}`; Jacobi preconditioner; views our arrays through `Eigen::Map` without copying; starts from zero; non-convergence throws `std::runtime_error` |
| `ConjugateGradient` | symmetric | Eigen's ConjugateGradient with Lower \| Upper and Jacobi; `ConjugateGradientOptions` as for `BiCgStab`; converges only for a positive definite matrix, otherwise it does not reach its tolerance and throws |
| `SimplicialLdlt` | symmetric | Eigen's SimplicialLDLT with AMD (lower triangle, copied to column-major); no pivoting, so a zero pivot (singular, or indefinite needing pivots) throws `std::runtime_error` in `prepare`; a tiny pivot passes the factorization, so `solve` checks the backward error against `SimplicialLdltOptions{.residual_tolerance = 1e-8}` and throws `std::runtime_error` above it; pivot ratio min \|D\| / max \|D\| |
| `SupernodalCholesky` | symmetric | CHOLMOD's supernodal L Lᵀ (always supernodal; CHOLMOD's default ordering, AMD or METIS); reads the lower triangle through a view of our arrays (no copy); `SupernodalCholeskyOptions{.residual_tolerance = 1e-8, .threads = 1}`; when every diagonal entry is negative (`CsrMatrix::entry(i, i) < 0`, a missing one counts as 0) `prepare` factors −A (a negated copy of the values on the same pattern, freed after the factorization) and `solve` solves −A x = −b (a negated copy of b, which CHOLMOD gets as a copy anyway), checks with A and b as given, and adds the warning "…the matrix is negative definite…, so -A was factored…; the answer is unchanged" to every result; `prepare` throws `std::runtime_error` for a matrix (or −A) that is not positive definite (a zero or negative pivot, naming the column; exactly singular, indefinite, a zero diagonal entry or one of each sign included); `solve` checks the backward error like `SimplicialLdlt`; pivot ratio `cholmod_l_rcond` ((min/max diag L)²) |
| `MultifrontalLu` | nothing | ParU's multifrontal LU with threshold partial pivoting (UMFPACK's analysis, METIS, rows scaled by their largest entry); the matrix is copied once into compressed columns; `MultifrontalLuOptions{.residual_tolerance = 1e-8, .threads = 1}`; `prepare` throws `std::runtime_error` for an exact zero pivot (a front's or a singleton's) and, before ParU, for a row with no entry that is not zero; `solve` checks the backward error; pivot ratio over the diagonal of U of the scaled matrix, singletons included |

The two iterative solvers share one prepared solver
(`detail::PreparedIterative<EigenSolver>`, `src/solve/iterative_solve.hpp`):
the right-hand side is scaled so that a tiny one still converges, a zero
one gives x = 0 with no iterations, and the measured residual decides
convergence (restarts while they halve it). An empty (0 × 0) system has an
empty solution. Eigen appears only in `src/solve/eigen_view.hpp`,
`src/solve/eigen_threads.hpp`, `src/solve/iterative_solve.hpp` and the
solver `.cpp` files.

**Threads in the solvers.** `pde2d_solve` is compiled with OpenMP, so
Eigen runs its row-major sparse matrix times vector product in parallel
(only above 20000 nonzeros): the rows are shared among the threads and
each row is summed by one of them, so the product is the same for every
count; the dot products, norms, vector updates and the Jacobi
preconditioner are not parallelized by Eigen. So the iterations and the
answer of `BiCgStab` and `ConjugateGradient` are the same bit for bit for
every `threads`. Eigen reads the count from `Eigen::nbThreads()`, which
is `omp_get_max_threads()` of the calling thread unless the program calls
`Eigen::setNbThreads`. Every Eigen computation in `pde2d_solve` runs inside
a `detail::EigenThreads` scope (`eigen_threads.hpp`), which sets the
calling thread's own OpenMP count (`omp_set_num_threads`) and restores it
afterwards: the iterative solvers set their option, the Eigen direct
solvers 1 (they stay serial). Nothing global is written, so solves running at once in
different threads keep their own counts; a program that calls
`Eigen::setNbThreads` overrides them all (the answers do not change).
The assembly report has `threads` (asked for) and `threads_used` (the most
threads that actually ran in one colour; OpenMP may give fewer, for
example inside a caller's parallel region); no colour asks for more
threads than it has elements. The bit-for-bit guarantee assumes the
caller and OpenMP's worker threads use the same floating-point rounding
mode (the default, round to nearest).

**Threads in the parallel direct solvers.** `SupernodalCholesky` and
`MultifrontalLu` take their threads from two places: OpenMP inside
SuiteSparse (CHOLMOD's loops over a supernode's updates,
`cholmod_common::nthreads_max`; ParU's tasks over independent fronts and
over the blocks of a large dense update, `PARU_CONTROL_MAX_THREADS`, which
ParU caps at the calling thread's `omp_get_max_threads()`), and the BLAS
(OpenBLAS, which does the dense work on supernodes and fronts). Both get
the resolved `threads`. OpenMP's count is set for the calling thread only,
but OpenBLAS has one count for the process (`openblas_set_num_threads`),
so every factorization and every solve runs inside a
`detail::FactorizationThreads` scope: it takes a process-wide lock, sets
both counts and restores them at the end. Two direct solves of this kind
that would run at once in different threads run one after another, each
with its own count for its whole run; code outside the library that calls
OpenBLAS meanwhile sees the scope's count. Ubuntu's SuiteSparse is built
for a generic BLAS, so ParU does not set the BLAS threads itself (its
built-in OpenBLAS support is compiled out); under its OpenMP tasks the
shared OpenBLAS pool may keep up to about twice `threads` threads busy,
which measured faster than a one-thread BLAS. The answers may differ in
the last digits between thread counts, and for ParU, which schedules its
tasks while it runs, between runs; with 1 thread they are reproducible bit
for bit (`tests/solve/parallel_direct.cpp`, also with two solves at once).

**What the direct solvers check.** Each solve of `SimplicialLdlt`,
`SupernodalCholesky` and `MultifrontalLu` checks the backward error, not
the relative residual, against `residual_tolerance` (`detail::
measure_residual`, `vectors.hpp`: one pass over the matrix gives b − Ax
and the row sums). The residual of an accurate answer is rounding of size
ε |A| |x| (each entry of Ax is a sum of up to about ten products), so the
relative residual grows with |A| |x| / |b| and a coefficient contrast of
1e6 makes it about 1e-7, while the backward error stays a few ε. The
infinity norm is the one this rounding bound is written in (entrywise
|fl(Ax) − Ax| ≤ γ (|A| |x|)), |A|∞ is exact and cheap from CSR rows, and
taking maxima squares nothing, so nothing underflows or overflows. The
iterative solvers and `SparseLu` report it too (the iterative ones from
the last residual of the scaled system; the backward error does not change
with the scale) but do not check it.

The factorizations refuse only an exact zero pivot (and the Cholesky any
pivot that is not positive). A singular matrix rarely meets an exact zero
pivot in floating point: a pure Neumann problem factorizes with a last
pivot of rounding (a pivot ratio of about 5e-14 on 65 × 65 nodes), and
its compatible load then gives one of its solutions (residual about
2e-13); a very ill-conditioned but well-posed problem is solved too
(a Robin coefficient of 2e-9 on one end of a 1 × 1/128 strip: pivot ratio
4e-12, the answer as accurate as its condition number of about 1e15
allows, which is about 10% here). The ratio of the smallest pivot to the
largest is reported in `pivot_ratio`, not judged (below 1e-10 the
result carries a warning, see above): CHOLMOD's
`cholmod_l_rcond`; for ParU, its smallest and largest |U_kk| over the
fronts (`PARU_GET_MIN_UDIAG`, `PARU_GET_MAX_UDIAG`) combined with the
singletons, which ParU eliminates before the fronts and leaves out: a
singleton's pivot is entry (P[k], Q[k]) of the scaled matrix, read from
our matrix and ParU's row scale factors (ParU checks the pattern, not the
values, for singletons, so an explicitly stored zero can be one; it
throws as an exact zero pivot, checked before dividing by the row scale).
Before ParU sees the matrix, `MultifrontalLu` refuses one with a row
that has no entry that is not zero (no slot, or only stored zeros), as
singular with the row named: ParU 7.12 checks the row scale factors
only of the rows that reach a front, so a zero row that is a singleton
would get through (its pivot 0 / 0). `MultifrontalLu` sets ParU's row scaling
(`PARU_PRESCALE_MAX`, the default) explicitly, since the singleton pivots
depend on it.

**Known limits of the SuiteSparse solvers.** OpenBLAS's thread count is
one setting for the whole process: a program that sets it
(`openblas_set_num_threads`) during one of these solves can have its
setting overwritten when the scope restores the count it found. Ubuntu's
ParU (SuiteSparse 7.12) defines the global `operator new(std::size_t)` and
`operator delete(void *)`, so every program that links `pde2d_solve`
allocates through them; on failure they throw `std::bad_alloc` at once,
skipping a `std::new_handler` the program installed. Both are said in
the two solvers' header comments.

Times measured once (Ryzen 9 6900HX, WSL; best of three; `prepare` is the
symbolic analysis and the numeric factorization) on the unit square cut
into 550 × 550 × 2 triangles, 301401 unknowns, 2.1 million nonzeros:

| Solver | Threads | `prepare` (s) | `solve` (s) |
| --- | --- | --- | --- |
| `SimplicialLdlt` | 1 | 1.87 | 0.047 |
| `SupernodalCholesky` | 1 / 2 / 4 | 0.53 / 0.51 / 0.49 | 0.049 / 0.049 / 0.048 |
| `SparseLu` (Poisson / convection) | 1 | 5.14 / 4.71 | 0.081 / 0.081 |
| `MultifrontalLu` (Poisson) | 1 / 2 / 4 | 2.46 / 2.44 / 2.10 | 0.090 / 0.092 / 0.090 |
| `MultifrontalLu` (convection) | 1 / 2 / 4 | 2.60 / 2.46 / 2.21 | 0.093 / 0.094 / 0.096 |

In 2D the fronts are small (the largest separator has about 2√n
unknowns), so most of `prepare` is the serial analysis (ordering and
symbolic factorization: about 0.2 s of CHOLMOD's 0.5 s, 1.3 s of ParU's
2.1 s) and the threads gain little. On a 3D Laplacian (45³ = 91125
unknowns, large dense fronts) they do: `SupernodalCholesky` 1.56 / 1.21 /
1.10 s, `MultifrontalLu` 4.13 / 3.00 / 2.58 s for 1 / 2 / 4 threads.

**Solution** (`solution.hpp`). `Solution{mesh, time, values, velocities,
accelerations}` (`accelerations` added last: positional initializations
keep their meaning);
`build_solution(meshed, free_values, t)` places the free values at
`free_nodes()` and `dirichlet_value(k, t)` at the Dirichlet nodes.
`velocities` is filled only by the `Integrator` for time order 2 (free
part from the state, Dirichlet nodes from `dirichlet_rate`), so that a run
can be continued; `accelerations` only for a method that keeps the
acceleration (`TimeMethod::keeps_acceleration`, Newmark: free part from
the state, Dirichlet nodes from `dirichlet_acceleration`), so that a
continued run does not compute it again, and by the adaptive loop after
damping steps.

**StaticSolver** (`static_solver.hpp`). Assembles at t with
`max_time_order` 0, checks the solver's requirements against
`stiffness().always_symmetric`, else `stiffness().symmetric_now`, solves `stiffness().free · U =
right_hand_side()` (= load − A_FD U_D at that order), and returns
`StaticResult{solution, reports}` with the reports "assembly" and
"static_solver" (free_unknowns, linear_iterations, residual,
backward_error, pivot_ratio, linear_threads; its messages are the linear
solver's, `LinearSolveResult::messages`). `StaticSolver(linear_solver, AssemblerOptions = {})`:
it makes its assembler, with the given thread count, in its constructor.

**Time methods** (`time_method.hpp`). A `TimeMethod` declares
`capabilities()` (`TimeMethodCapabilities{time_order_0, time_order_1,
time_order_2}` with `supports(order)`), `evaluation_fraction()` (where in the step the
problem is assembled) and `step_matrix_weights(dt)` (`StepMatrixWeights{
mass, damping, stiffness}`: the step matrix is mass·M₂ + damping·B +
stiffness·A, so the integrator can build it and tell when it changes). It
declares `start_needs(time_order)` (`StartNeeds::nothing`, `static_part`
or `whole_problem`: what its start reads at t0), has `start(assembled at
t0, linear_solver, state)` (default: nothing; called only when the start
needs something) and `advance(assembled, dt, prepared, state) ->
StepSolveInfo`. It declares `keeps_acceleration(time_order)` (default
false; Newmark true): its state carries U″ and its start computes nothing
else, so the `Integrator` returns the accelerations in its Solutions and,
given an initial Solution with accelerations, takes them and calls no
start. It declares `velocity_quotient_is_acceleration(time_order)`
(default false; `ImplicitEuler` true at order 2): the quotient (V₁ −
V₀)/dt of a step satisfies M₂ U″ + B U′ + A U = b at the step's end, so a
method that keeps the acceleration may continue from it (the adaptive
loop after damping steps; the midpoint method's quotient is the
acceleration at the middle, Newmark's the average of the two ends). The state is a
`TimeState{values, velocities, accelerations}` in the free unknowns. The
right-hand side the methods use is `right_hand_side()`, which already
holds the whole lifting.

| Method | Orders | Start needs | Assembled at | Step matrix | Right-hand side |
| --- | --- | --- | --- | --- | --- |
| `ImplicitEuler` | 0, 1, 2 | `static_part` at order 0 (start solves A(t0) U = b(t0)), else `nothing` | t + dt | M₂/dt² + B/dt + A | b + M₂(U₀/dt² + V₀/dt) + B U₀/dt; then V₁ = (U₁ − U₀)/dt |
| `ImplicitMidpoint` | 1, 2 | `nothing` | t + dt/2 | 2M₂/dt² + B/dt + A/2 | b + M₂(2U₀/dt² + 2V₀/dt) + (B/dt − A/2) U₀; then V₁ = 2(U₁ − U₀)/dt − V₀ |
| `Newmark` (β = 1/4, γ = 1/2) | 2 | `whole_problem` | t + dt | M₂/(β dt²) + γ B/(β dt) + A | b + M₂(U₀/(β dt²) + V₀/(β dt) + (1/(2β) − 1) W₀) + B(γ U₀/(β dt) + (γ/β − 1) V₀ + dt (γ/(2β) − 1) W₀) |

**Local error estimates** (for an error-controlling step controller,
decided 2026-11-18). A method declares `error_estimate(time_order)`
(`std::optional<ErrorEstimateDeclaration{order, previous_steps}>`: its
order p and how many accepted steps before the current one it reads) and
computes `local_error(before, after, dt, previous)` (one entry per free
unknown) from states it already has, with no solve:

| Method | Orders | p | Earlier steps | Estimate of the step's local error in U |
| --- | --- | --- | --- | --- |
| `ImplicitEuler` | 1 | 1 | 1 | dt²/2 u″, u″ = (d₁ − d₀)/dt, dᵢ the quotients of this step and the one before (Euler's quotients are the rates at the steps' ends, dt apart whatever dt₋₁ is): dt (d₁ − d₀)/2 |
| `ImplicitEuler` | 2 | 1 | 0 | dt/2 (V₁ − V₀) |
| `ImplicitMidpoint` | 1 | 2 | 2 | dt³/12 u‴ (the trapezoidal rule's term), u‴ = 6 U[t₋₂, t₋₁, t₀, t₁], the third divided difference of the values at the ends of the last three steps: dt³/2 U[…] |
| `ImplicitMidpoint` | 2 | 2 | 1 | dt³/12 u‴ (U is the trapezoidal rule of V), u‴ = V″ from the endpoint velocities: a₁ = (V₁ − V₀)/dt, a₀ = (V₀ − V₋₁)/dt₋₁ (u″ at the two steps' middles), V₋₁ = 2(U₀ − U₋₁)/dt₋₁ − V₀ (that step's update): dt³(a₁ − a₀)/(6(dt + dt₋₁)) |
| `Newmark` | 2 | 2 | 0 | Zienkiewicz–Xie (1991): dt²(β − 1/6)(W₁ − W₀) = dt²/12 (W₁ − W₀) |

`ImplicitEuler` declares none at time order 0 (no time error). Without
free unknowns (every node a Dirichlet node) every estimate is empty
(norm 0). A step with
fewer earlier steps than declared (the first steps of a run, of a piece
after a transfer) has no estimate. An estimate is made from values the
method computed: a step long enough to jump over a fast change of the
data is not seen as wrong when the method did not sample it (the midpoint
rule samples the load only at the middle of the step), so a run with
fronts in its data needs `max_dt` (`tests/solve/adaptive_step.cpp`).

A zero operator adds nothing, so one set of weights serves every order of
a method: at time order 1 the M₂ terms are absent, at 0 (Euler only) also
the B terms, leaving A U₁ = b(t₁). At time order 2 Euler and midpoint step
the pair U′ = V, M₂ V′ + B V + A U = b (Euler: (U₁ − U₀)/dt = V₁; midpoint:
(U₁ − U₀)/dt = (V₁ + V₀)/2 with everything at the middle), V₁ eliminated;
they need no acceleration, so M₂ may be singular or zero at some time
(m2 = sin(t)²), and the midpoint step with M₂ = 0 is the first-order step.
Midpoint refuses time order 0 (U₁ = 2A⁻¹b − U₀ oscillates). None inverts B
or M₂ during the steps, so B may be singular (m1 = 0 with a dynamic
condition). `Newmark::start` solves M₂(t0) W₀ = b − B V₀ − A U₀ for the
starting acceleration, so M₂ must not be singular; when it is zero at t0
(`zero_now`, checked first) or the solve fails (M₂ singular at t0), it
throws `std::runtime_error` naming the cause and
`ImplicitMidpoint` and `ImplicitEuler`, which need no acceleration (no
fallback).

**Step control** (`step_controller.hpp`, `fixed_step.hpp`,
`adaptive_step.hpp`). `StepController::next_step(t)`,
`controls_error()` (default false) and `judge(TakenStep) -> StepDecision`
(the interface extended for the error-controlled step, decided
2026-11-17). A controller is const and stateless; what a run remembers
is in the `Integrator`'s loop and, across runs, in `StepAnchor`. Two
kinds, told apart only by `controls_error()`:

- `FixedStep({.dt = ...})` (and any controller that does not control the
  error) is asked `next_step(t)` before every step; `judge` is never
  called, every step is accepted, and the loop does exactly what it did
  before the extension (no copy of the state, no history).
- `AdaptiveStep(AdaptiveStepOptions{.initial_dt, .atol = 1e-6, .rtol =
  1e-3, .safety = 0.9, .min_factor = 0.2, .max_factor = 5, .min_keep_factor
  = 0.9, .max_keep_factor = 1.5, .min_dt = 0, .max_dt = ∞})` gives
  `initial_dt` as the first step of a run that carries no next dt, and
  judges every step: the error norm is the root mean square of eᵢ /
  (atol + rtol·max(|uᵢ before|, |uᵢ after|)) (`error_norm`); above 1 the
  step is rejected and taken again with length·max(min_factor,
  safety·err^(−1/(p+1))), at least `min_dt` (a rejected step not longer
  than `min_dt` leaves no shorter try: `std::runtime_error`), else
  accepted with the proposal length·safety·err^(−1/(p+1)), limited to
  [min_factor·dt, max_factor·dt] (dt the controller's step, not the
  shorter length of a landing step, so a landing does not shrink the next
  step) and to [min_dt, max_dt]; a proposal within [0.9 dt, 1.5 dt] keeps
  dt, so the step matrix is not prepared again. A step without an
  estimate is accepted with dt kept. Options are checked in the
  constructor (`std::invalid_argument`). The error per step is
  controlled, so the global error behaves like tol^(p/(p+1)).

Measured (`tests/solve/adaptive_step.cpp`, u = (1 + x + 2y) g(t) on a
4 × 4 square, so only the time error remains; the error is the largest
nodal error at t1/4, t1/2, 3t1/4 and t1; `max_dt` = t1/20; against the
largest fixed step t1/(4m) at least as accurate):

| Problem, method | Adaptive: error, steps (rejected), prepares | Fixed: dt, error, steps, prepares |
| --- | --- | --- |
| heat, front g = tanh(40(t − ½)), midpoint, rtol 1e-4 | 0.0025, 93 (19), 40 | 1/264, 0.0023, 264, 1 |
| heat, the same front, implicit Euler, rtol 1e-3 | 0.049, 135 (20), 40 | 1/860, 0.047, 860, 1 |
| wave, g = cos 2πt + ½ tanh(20(t − 1)), Newmark, rtol 1e-4 | 0.021, 187 (45), 71 | 1/84, 0.019, 168, 1 |
| wave, the same with tanh(400(t − 1)), Newmark, rtol 1e-4 | 0.013, 219 (60), 96 | 1/518, 0.0066, 1036, 1 |

The adaptive step pays where the solution has a fast phase; on a smooth
oscillation it takes about as many steps as a fixed one (a wave's phase
error grows over the run, which a per-step tolerance does not see), and
it prepares the linear solver far more often (each change of dt, and
twice for each landing that shortens a step).

**Integrator** (`integrator.hpp`). `Integrator(method, step_controller,
linear_solver, AssemblerOptions = {})` (it makes its assembler with these
options); `integrate(meshed, t0, t1, IntegrationOutput, const
Solution *initial = nullptr, std::optional<std::size_t> max_steps = {},
const StepAnchor *anchor = nullptr) -> IntegrationResult{final,
snapshots, history, report, before_last_step, anchor}`
(`before_last_step` and `anchor` were added last, so a positional
initialization written before them keeps its meaning; new members go at
the end). `method()` returns the time method, so that a driver can read
its declarations.

- The time order is `Assembler::time_order(meshed)`, from the formula
  facts without evaluating anything, checked against
  `capabilities().supports(order)`; when it is lower than
  `time_order(meshed)` (the order as written), the refusal says that time
  terms vanished in assembly.
- The state comes from `initial` (same mesh object; velocities required
  for time order 2; for a method that keeps the acceleration its
  accelerations when it has them, checked for size and finiteness) or
  from `initial_free_state(t0)` (the free nodes
  only; no boundary data is evaluated). At time order 0 there is no initial state (`initial` is
  only checked for its mesh and size): the method's start computes it.
- The problem is assembled at t0 only as far as `start_needs(order)`
  says (`static_part`: `max_time_order` 0, `whole_problem`: 2), and
  `start` is called only then; with `nothing` there is no assembly at t0,
  so boundary data whose rate is not finite at t0 (u = √t from 0) works
  with Euler and midpoint. When the accelerations of a method that keeps
  them came from `initial`, the start is skipped as for `nothing` (no
  assembly at t0). After the start the state must have a value per free
  unknown, and an acceleration per free unknown for a method that keeps
  it, or the integrator throws `std::runtime_error`.
- Times: on a stretch of equal steps from an anchor, step k ends at
  anchor + k·dt; a step ending within 1e-9·dt of the next target (a
  snapshot time or t1) lands on it, and a step that would pass it is
  shortened. The anchor moves to every target landed on, so a run that
  lands on t1 and a run that starts there take the same steps.
- Each step assembles at `(1 − fraction)·t + fraction·t_new` with the
  previous result as `previous`, and prepares the linear solver only when
  there is none yet, dt changed, or an operator with a non-zero weight in
  the step matrix (and not `always_zero`) `depends_on_t`; formula facts
  only. Before each prepare it checks the solver's requirements: the
  step matrix is symmetric when every operator with a non-zero weight is
  always symmetric or always zero, else when `is_symmetric` holds for
  the step matrix about to be prepared.
- With an error-controlling controller the method must declare an error
  estimate at the time order (`std::invalid_argument` naming the method,
  before anything is assembled). Each step's start is kept; after
  `advance` the method's `local_error` (when the run has the earlier
  steps it reads) goes to `judge` in a `TakenStep{t, dt, length, order,
  values_before, values_after, local_error}`. A rejected step is recorded
  (`accepted = false`), the state and the anchor go back to the step's
  start, and the step is taken again with the decision's smaller dt (a
  rejection that does not shrink throws `std::runtime_error`); an accepted
  one gives the next dt and joins the last accepted steps the estimate
  reads. `max_steps`, `snapshot_every` and `keep_before_last_step` count
  accepted steps. The anchor carries `next_dt` and `previous_steps` (the
  last accepted steps, values in the free unknowns and lengths), so a run
  continued with its anchor equals one long run, rejections included, bit
  for bit; they are empty for a controller that does not control the
  error, and checked (sizes, finite) only when used.
- Each step's `StepRecord{t, dt, accepted, linear_iterations, residual,
  time, error_norm}` updates a running summary and is kept only with
  `keep_step_history`; snapshots are taken at `snapshot_times` (strictly
  increasing in [t0, t1]; t0 keeps the start) and every `snapshot_every`
  steps, once per step. The summary becomes the report "integrator"
  (t0, t1, time_order, steps (accepted), rejected_steps, min_dt and
  max_dt (accepted steps), linear_iterations (every step taken),
  max_linear_iterations, max_residual, assemblies,
  prepares, snapshots, assembly_threads, linear_threads,
  max_backward_error, min_pivot_ratio: the worst of every solve of the
  call, `start()`'s included, from the logging wrapper, not from
  `StepSolveInfo`; min_pivot_ratio is 0 when nothing measured it), whose
  messages
  are the linear solver's warnings of every solve of the call (also of
  `start()`), each distinct one once, in the order they were added
  (solves from several threads at once: the order in which they took the
  log's lock).
- Continuing a run from `previous.final` gives the same values bit for
  bit when the long run lands on the split time, for `Newmark` too: its
  acceleration comes with `final` and its start is not called again, so
  it also continues from a time where M₂ is zero (m2 = (t − 0.5)² with
  damping, continued at 0.5; `tests/solve/time_integration.cpp`). Without
  the accelerations, Newmark's start computes them again (to rounding,
  and only where M₂ is invertible).
- `with_method(method)` returns an `Integrator` with the same step
  controller, linear solver and assembler options and another method
  (the adaptive loop's damping steps).
- `max_steps` (at least 1) ends the run after that many steps if t1 is
  not reached first; `final.time` is where it ended (metric `t_end`; `t1`
  stays the asked end), the last step is not shortened, and snapshot
  times after it are not taken. A run continued from there starts a new
  stretch of steps at that time (bit for bit like a long run only when
  the step times are exact in binary, as anchor + k·dt and (anchor +
  n·dt) + k·dt then agree), unless it is given the earlier run's
  `IntegrationResult::anchor` (`StepAnchor{time, dt, steps}`: the
  stretch the run ended on, its steps ending at time + k·dt; after a
  landing time is the landing time and steps 0): then its first step
  continues that stretch when the controller gives the same dt, and the
  step times are those of one long run (also when the controller's
  answer depends on t). An anchor that does not stand at t0 (time +
  steps·dt ≠ t0) throws. `IntegrationOutput::keep_before_last_step` keeps the state at
  the start of the last step (`before_last_step`, a `FreeState{time,
  values, velocities, dt}` in the free unknowns, with velocities for time
  order 2, and the step's length as the method used it: a step landing
  within 1e-9·dt of a target keeps its nominal dt, so the difference of
  the times can differ from it): no boundary data is evaluated for it
  (the rate of u = √t does not exist at t0 = 0), and a caller can form
  the difference quotient of the final state at the free nodes, dividing
  by `dt`; the adaptive loop uses both.

## Adaptive refinement (estimation, marking and the loop in `pde2d_adapt`, refinement in `pde2d_mesh`)

Adaptive refinement goes round three steps: estimate, mark, refine.
Estimators and marking strategies are components of `pde2d_adapt`;
refinement is `Remesher::refine`, with the same refiners as meshing.
`MeshRefinement` puts the three together as one `Adaptation`, the
`AdaptiveStaticSolver` runs the loop for static problems and the
`AdaptiveIntegrator` the loop in time, which moves the state to each new
mesh with `transfer_solution`.

```mermaid
flowchart LR
    SOLVE["StaticSolver::solve"]
    EST["ErrorEstimator::estimate"]
    STOP{"target met or<br/>a budget reached?"}
    MK["MarkingStrategy::mark"]
    RF["Remesher::refine"]
    NEW["MeshedProblem(new mesh, same Problem)"]
    MP([MeshedProblem])
    SOL([Solution])
    ETA(["ErrorEstimate: one indicator per triangle, total"])
    OUT(["AdaptiveStaticResult"])

    MP --> SOLVE --> SOL --> EST --> ETA --> STOP
    STOP -- yes --> OUT
    STOP -- no --> MK --> RF --> NEW --> MP
    subgraph MeshRefinement
        EST
        MK
        RF
    end
```

```cpp
Mesh mesh = Mesher(...).run(geometry, spec).mesh; // longest edges first
auto meshed = std::make_shared<const MeshedProblem>(
    std::make_shared<const Mesh>(std::move(mesh)), problem);
auto refinement = std::make_shared<MeshRefinement>(
    std::make_shared<ResidualEstimator>(),
    std::make_shared<DorflerMarking>(DorflerMarkingOptions{.theta = 0.5}),
    std::make_shared<Remesher>(std::make_shared<NewestVertexBisection>()));
const AdaptiveStaticSolver adaptive(StaticSolver(std::make_shared<SparseLu>()),
                                    {refinement});
AdaptiveStaticResult r = adaptive.solve(
    meshed, {.tolerance = {.relative = 0.01}, .max_nodes = 100000});
// r.cycles.back().solution, r.stop_reason, r.report
```

### Marking

`MarkingStrategy::mark(mesh, indicators)` takes one finite, non-negative
indicator per triangle (`std::span<const double>`, length
`mesh.triangles.size()`, else `std::invalid_argument`) and returns the
marked triangle indices in increasing order. It is not virtual: it checks
the input and applies the rules every strategy shares (a zero indicator
is never marked, so all zeros mark nothing; sorted, no repetitions), then
calls the strategy's private virtual `select`. Strategies, each with its
options struct validated in the constructor:

| Strategy | Options | Marks |
| --- | --- | --- |
| `MaximumMarking` | `theta` in [0, 1], default 0.5 | η_T ≥ θ · max η |
| `DorflerMarking` | `theta` in [0, 1], default 0.5 | the smallest set, largest first (ties by index), with Σ_M η² ≥ θ Σ η² (squares scaled by the largest indicator; both sums in the same order) |
| `FixedFractionMarking` | `fraction` in [0, 1], default 0.3 | the round(fraction · N) largest (nearest integer, halves up; ties by index) |
| `EquidistributionMarking` | `tolerance`, required, finite and > 0 | η_T ≥ tol / √N |

### Refinement edges

Every triangle's **refinement edge is the edge opposite its vertex 0**
(a convention of `Mesh` and `WorkingMesh`, documented in `mesh.hpp`).
The bisection refiners read it from the vertex order and write it into
the triangles they make (the newest vertex first, with
`WorkingMesh::set_refinement_edge`), so it persists across cycles without
a field in `Mesh`. The vertex order stays counterclockwise.
`set_longest_edges_as_refinement_edges(mesh)` (`mesh.hpp`) rotates every
triangle so that its longest edge is the refinement edge (ties: the
edge order of the bisection core, squared length and then node indices,
which both triangles of an edge agree on, so waiting triangles cannot
cycle; one private function, `detail::longest_local_edge` in
`src/mesh/refinement_edges.hpp`, serves both).

**The library delivers the order; the user never calls the function**
(except on a mesh built or edited by hand):

| Where | Which triangles get their longest edge first |
| --- | --- |
| `Mesher::run`, `Remesher::run` (all three forms) | every triangle, after the final passes, before `to_mesh` and the one measurement |
| `Remesher::refine` | only the triangles whose vertex order an edit chose, during the refiner or a final pass: made or changed by flips (local or final Delaunay flips; the red division's flip is followed by `set_refinement_edge`, so red children keep the red order), Ruppert's insertions and flips, a re-triangulating boundary split. Triangles a bisection made (newest vertex first, `set_refinement_edge`) and input triangles nothing touched keep their order: newest vertex bisection's memory between cycles. Moving a node (Laplacian smoothing) keeps the order |
| `import_gmsh` | every triangle |
| `import_pde2d` | none: a file keeps its order (it may be a mesh saved in the middle of adaptation) |

How `refine` knows which triangles: the `WorkingMesh` records each
triangle's vertices as its constructor received them (`from_mesh`: the
input order) or as `set_refinement_edge` last left them; a triangle whose
vertices now differ was made or changed by an edit that chose vertex 0
freely. The Remesher calls the private
`detail::set_longest_edges_as_refinement_edges(working, scope)` (scope
`all` for `run`, `edited_order` for `refine`) after the final passes; it
rotates with `set_refinement_edge`. The pipeline report's metric
`refinement_edges_set` counts the triangles whose vertex 0 changed (0
for a bisection refiner without improvers).

`run(spec)` of `NewestVertexBisection` and `RedGreenBlueRefinement` also
sets the longest edges before refining (metric `refinement_edges_set` in
their report), since its input may be any mesh; `refine(marked)` never
does, since between cycles the stored order is the method's memory.
Code that edits triangles (flips, Ruppert, triangulators, Gmsh import)
chooses vertex 0 freely inside the library; the order is delivered at
the end of each of the calls above.

### Refiners refine marked triangles

Every `Refiner` has two calls: `run(mesh, spec, ...)` towards a
`MeshSpec`, and `refine(mesh, marked, ...)`, one refinement of marked
triangles. `Remesher::refine(mesh, marked)` calls the second with the same
plan, improvers and reports as `run`.

The three bisection refiners share one private core
(`src/mesh/edge_bisection.*`) and differ in one rule each
(`detail::BisectionRule`):

| Refiner | A marked triangle marks | A triangle not yet bisected in this step is bisected at |
| --- | --- | --- |
| `LongestEdgeBisection` | its longest edge | its longest edge (measured then) |
| `NewestVertexBisection` | its refinement edge (edge 0) | its refinement edge |
| `RedGreenBlueRefinement` | all three edges | its refinement edge; red where all three edges are split |

One step (`EdgeBisection::refine`):

1. mark the edges of the marked triangles by the rule;
2. split every marked edge, in the order they were marked. An edge is
   split only when it is the bisection edge of the triangles on both its
   sides; a side that must first be bisected at another edge pushes that
   edge on a stack, which is split first (the closure, so no node ever
   hangs: Rivara's longest-edge propagation path for longest edges, the
   recursive closure of newest vertex bisection otherwise). Interior
   edges are split at their midpoint (`split_interior_edge`), boundary
   edges by `split_boundary_edge` (the boundary split policy: the curve
   point of a linked edge, the chord midpoint otherwise). The new node
   becomes vertex 0 of every triangle the split makes, and a triangle a
   bisection made is bisected next at its edge 0, an edge of its parent;
3. `RedGreenBlueRefinement`: a triangle whose three edges were all split
   has the bisection pattern (bisected, both children bisected); one flip
   of the edge v0-m0 makes it red, and the four children are given the
   vertex order of the images of v0, v1, v2;
4. the local improvers run after every split (and every red division) on
   the triangles it made, as in every refiner; once an improver has
   changed the mesh, every triangle is bisected by the method's first rule
   (longest edges for `LongestEdgeBisection`, as in Rivara's propagation
   path), and an edge a flip removed before its split is not split.

Bisecting (v0, v1, v2) at the midpoint m of v1-v2 gives (m, v0, v1) and
(m, v2, v0). Red gives (v0, m2, m1), (m2, v1, m0), (m1, m0, v2),
(m0, m1, m2), m_i on the edge opposite v_i: each lists the images of
v0, v1, v2 under the similarity that maps the parent onto it, so its
refinement edge corresponds to the parent's. Green and blue are NVB
bisections and a red child is refined later like a copy of its parent,
so every triangle is similar to one that NVB alone makes from an initial
triangle: at most four similarity classes per initial triangle, and green
triangles need not be removed between cycles. Near curved boundaries this
does not hold (the curve points change the shapes); there red refinement
flattens triangles with two boundary edges on a smooth curve ("ears") by
half each cycle, NVB does not (see `red_green_blue_refinement.hpp`); the
mesh properties' warning shows it.

Special cases, each reported: triangles whose refinement edges wait for
each other in a cycle (possible only with refinement edges set neither by
bisection nor to the longest edges, as on a mesh built by hand) have one edge split
out of order, so the mesh stays conforming but those triangles leave the
bisection pattern (`incompatible_splits`); an unsplittable boundary edge
leaves the triangles waiting for it unrefined (`unsplittable_edges`); a
red division that a curve point would invert keeps the bisection pattern
(`red_not_possible`).

`run` (until every edge is at most h) is a loop around the step: mark the
triangles with a longer edge, refine them, repeat until none is left or a
step splits nothing. `Ruppert` keeps its own quality loop for `run`; its
`refine` inserts the circumcenters of the marked triangles with its
encroachment rules (an encroaching circumcenter is replaced by splits of
the boundary edges it encroaches).

### Parents

`Parents{triangles, nodes}` (`report.hpp`): for each output triangle the
input triangle it lies in (a child of a curved boundary triangle may
stick out of it), for each output node `{v, v}` (input node v, which
keeps its index) or `{a, b}`, a < b (a node on input edge a-b: the
midpoint, or the curve point of a linked boundary edge). The bisection
core tracks them through its splits (a split reuses one index and appends
the other halves, which inherit the parent). They survive only if no
component made them untrue:

- the refiner declares `RefinerCapabilities::keeps_parents` (the three
  bisection refiners yes, `Ruppert` no) and its `refine` returned them
  (a bisection refiner returns none after a re-triangulating boundary
  split, with the reason in its report);
- every local improver and final pass of the plan declares
  `ImproverEffects::keeps_parents` (`DelaunayFlips` and
  `LaplacianSmoothing` do not).

Otherwise `RefinementResult::parents` is empty and the pipeline report
names each component that dropped them ("parents dropped: delaunay_flips
(local): it flips edges; ...").

### Remesher::refine

`Remesher::refine(mesh, marked)`: validates the mesh
(`std::invalid_argument`, "Remesher::refine: the input mesh is not
valid: ...") and the marked indices (out of range: `std::invalid_argument`);
needs a refiner; runs the refiner's `refine` with the planned local
improvers, compacts, runs the final passes (with a null `MeshSpec`,
since refinement has no length target), gives the triangles whose order
an edit chose their longest edge as refinement edge (see "Refinement
edges"), validates the result (a curve point bulging into another part of the mesh
is a `std::runtime_error`), measures it, and decides the parents.
Reports: pipeline, refiner, final passes. With nothing marked the refiner
changes nothing and the final passes do not run (the pipeline report says
so), so the mesh comes back unchanged. `refined_by_closure` counts every
unmarked input triangle that changed, including those a re-triangulating
boundary split replaced. Dörfler marking with θ = 1 marks every positive
indicator.

### Error estimators

`ErrorEstimator::estimate(meshed, solution, derivatives)` is not
virtual: it checks the input (the solution on the problem's mesh object,
one finite value per node; u_t and u_tt in `TimeDerivatives` each empty
or one finite value per node; else `std::invalid_argument`), times the
work, asks the estimator's private `indicators()` for one η_T per
triangle, and returns `ErrorEstimate{indicators, total, report}` with
total = √(Σ η_T²) and the report metrics `triangles`, `total`,
`max_indicator` (plus the estimator's own). The time is
`solution.time`. All integrals use the rules of `fem/quadrature.hpp`, of
the exact degrees in the estimator's options (validated in its
constructor: a degree the table has no rule for, below 0 or above
`fem::max_triangle_degree` / `max_edge_degree`, is a
`std::invalid_argument` naming the option). The defaults are exact for
coefficients and data of degree at most 1 (with β̂_n constant).

| Estimator | η_T² | Needs | Quadrature: options (default exact degree) |
| --- | --- | --- | --- |
| `ResidualEstimator` | h_T² ‖r‖²_T + Σ_interior ½ h_E ‖J‖²_E + Σ_natural/dynamic h_E ‖R‖²_E | u_t when m1 ≠ 0 or a region is dynamic (A ≠ 0), u_tt when m2 ≠ 0 (zero as written); missing: `std::invalid_argument` | `ResidualEstimatorOptions{.triangle_degree = 4, .edge_degree = 6}`: triangles 4, jumps and boundary 6 |
| `KellyEstimator` | Σ_interior ½ h_E ‖J‖²_E + Σ_natural/dynamic h_E ‖R‖²_E, R without γ u_t | nothing | `KellyEstimatorOptions{.edge_degree = 6}`: jumps and boundary 6 |
| `RecoveryEstimator` | ∫_T (G − ∇u_h)·W(G − ∇u_h), G the area-weighted nodal average of ∇u_h, linear between nodes; W = \|A\| (`RecoveryNorm::energy`, default) or I (`gradient`) | nothing | `RecoveryEstimatorOptions{.norm, .triangle_degree = 3}`: triangles 3 |

With A = [[a_xx, a_xy/2], [a_xy/2, a_yy]], h_T the longest edge and h_E
the edge length: r = f − b·∇u_h − c u_h − m1 u_t − m2 u_tt (the
second-order part cancels exactly for P1: div(A∇u_h) = (∇·A)·∇u_h,
which the −(∇·A)·∇u term of the weak form removes); J = n·A(∇u_h|T −
∇u_h|T′) on an interior edge; on a natural or dynamic edge R =
κ_n/β̂_n · (g − α u_h − β_n ∂_n u_h − β_τ ∂_τ u_h − β_x ∂_x u_h − β_y ∂_y
u_h − γ u_t), the flux the condition prescribes minus the flux of u_h
(the same κ_n and β̂_n as assembly; β̂_n = 0 throws). Dirichlet edges
add nothing; with A zero as written there are no edge terms. Report
metrics: `element_part`, `jump_part`, `boundary_part` (√ of the sums of
each kind; the Kelly estimator has the last two). The energy norm takes
|e·A e| at each point, which is e·|A|e when A is definite or
semidefinite there; an indefinite A (a_xx a_yy − a_xy²/4 < 0 beyond
rounding) or A zero as written throws, naming `RecoveryNorm::gradient`.

The private `estimator_geometry.hpp` computes, once per call, the area,
diameter and gradient of u_h of each triangle and the edge topology
(interior edges from sorting the triangle edges by their node pair, the
triangle of each boundary edge), in index order.

### Adaptations and the adaptive loop

An `Adaptation` (`adaptation.hpp`) improves the discretization after a
solve, in two steps a driver can test its target between:
`estimate(state)` (its own `ErrorEstimate`, or nothing) and `adapt(state,
estimate)` (`estimate` is what `estimate()` gave for the same state, or
null to estimate itself). The state is a `SolvedState{problem, solution,
derivatives}`. It declares `AdaptationCapabilities{estimates_error,
changes_mesh}`; drivers read these, never the type. `AdaptationResult`
holds the new mesh (null when unchanged), its `MeshProperties` and
`Parents` (when kept), the estimate and the reports.

`MeshRefinement(estimator, marking, remesher)` (both capabilities):
estimate (or the given one, which must have one indicator per triangle),
`marking.mark`, `remesher.refine` of the solution's mesh. Nothing marked
leaves the mesh unchanged without calling the Remesher. Reports: the
estimator's, `mesh_refinement` (metrics `triangles`, `marked`; messages
naming the marking strategy and "nothing marked"), then the Remesher's.

`AdaptiveStaticSolver(static_solver, adaptations)` throws without
adaptations, for a null one, and when more than one declares
`changes_mesh` (all act on the same solved state until solutions can be
transferred). `solve(meshed, target, t = 0)`:

1. solve with the `StaticSolver` at t; the derivatives passed on are
   zeros (a steady state);
2. the cycle's estimate is that of the first adaptation declaring
   `estimates_error` (none: no estimate, and a tolerance throws);
3. stop with `tolerance_met` when its total ≤ the tolerance (the smaller
   of `absolute` and `relative` × the first cycle's total), else with
   `max_cycles` on the `max_cycles`-th solve;
4. every adaptation adapts in order (the estimating one gets its
   estimate); no new mesh: `nothing_changed`; a new mesh with more nodes
   than `max_nodes`: `node_budget`, the mesh is discarded unsolved;
5. otherwise a new `MeshedProblem` from the same `Problem` on the new
   mesh, and the next cycle.

`AdaptiveTarget{tolerance, max_cycles, max_nodes}` needs at least one;
values must be finite and positive (counts at least 1), and a starting
mesh above `max_nodes` throws. `AdaptiveStaticResult` holds every
`AdaptiveCycle{solution, estimate, reports}` (reports: `meshed_problem`
for problems the loop built, the static solver's, then the
adaptations'), the last cycle's problem, the `StopReason` (`to_string`)
and the report `adaptive_static_solver` (metrics `cycles`, `nodes`,
`triangles`, `estimate`, `tolerance`; the message "stopped: <reason>:
...").

Measured (tests): on the unit square with u = sin πx sin πy the residual
estimator's effectivity is 5.5–5.6 and the recovery estimator's tends to
1 (1.018 → 1.003 from n = 8 to 64); every total converges like h. On
the L-shape with u = r^(2/3) sin(2θ/3), Dörfler θ = 0.5 and newest
vertex bisection give energy error ∝ N^(−0.53) (residual, Kelly and
recovery estimators alike), uniform refinement N^(−0.33).

### Solution transfer

`transfer_solution(old, problem, parents = nullptr)`
(`solution_transfer.hpp`) moves a P1 `Solution` to `problem`'s mesh by
interpolation and returns `SolutionTransfer{solution, report}`: the
solution on that mesh object at `old.time`, velocities and accelerations
when the old one had them; the Dirichlet nodes of `problem` get
`dirichlet_value(k, time)` (`dirichlet_rate` for the velocities,
`dirichlet_acceleration` for the accelerations). Each new node is first
written as weights of at most three old nodes (a private plan), applied
to the values, the velocities and the accelerations alike:

| Case | A new node gets |
| --- | --- |
| parents, `{v, v}` | old node v's value (copied exactly) |
| parents, `{a, b}` | (u[a] + u[b]) / 2: the old u_h at the edge's midpoint, also for a curve point on a curved boundary edge, which lies off the old chord (outside the old mesh on a convex curve, inside on a concave one): it gets the value at the chord point it was split from, its nearest old boundary point to second order (exactly on a circle) |
| no parents, a boundary node with a curve link | when both meshes have its chain (same index, length and number of curves, as a Remesher passes them on): the old u_h at its arc length on the old edge of that chain that covers it (from its outgoing edge's s0, else its incoming edge's s1), weighted by arc length along the edge (a curve midpoint gets the average of the edge's ends, as with parents) |
| no parents, another boundary node | the old u_h at its nearest point of the old boundary loop matched to its own loop (nearest edge of that loop, lowest index on ties, clamped to its ends), never another wall's: in a thin annulus a new inner curve point can be nearer the outer polygon than its own chord |
| no parents, an interior node | the old u_h in the old triangle that contains it (exact `orient`; lowest index on an edge or node; at an old node's position its value exactly), or, if none does (a node moved outside the old mesh near a curved boundary), at its nearest old boundary point |

Boundary loops are the connected pieces of the boundary edges
(`detail::boundary_loops`, union-find over their nodes; valid loops never
touch), numbered in the order of their first edge, on the old and the new
mesh alike. A new loop is matched to an old one by votes of its boundary
nodes: each votes for the old loop of its old boundary point (through its
link, else its nearest old boundary point overall); if any of its nodes
lie exactly on the old boundary (placed by a link, or a kept old node at
distance 0) only those vote. The most votes win, ties the lower old loop.
Regions are not used: two walls may share one.

Point location (`detail::PointLocator`, `src/adapt/point_location.hpp`)
is a uniform bucket grid over the old mesh's bounding box (about one
triangle per cell): each cell lists the triangles and boundary edges
whose boxes meet it, in index order; containment is tested in the
point's cell, the nearest boundary point by rings of cells around it
until no farther cell can be nearer (optionally among the edges of one
loop). There is no walk, so the cycling of
`WorkingMesh::locate` (GitHub issue #22) and its stop at the boundary of
a non-convex domain cannot occur. Every weight is in [0, 1] and they sum
to 1, so no new extremes appear; on a refinement (with or without
parents) the new u_h equals the old one inside the old mesh. Report
"solution_transfer": metrics `nodes`, `by_parents`, `kept_nodes`,
`edge_nodes` (with parents), `located`, `by_link`, `nearest_boundary`,
`other_loop_nearer` (without: boundary nodes whose nearest old boundary
point overall was on another loop), `dirichlet_nodes`. Bad input (no mesh, sizes, values that are not
finite, parents that do not fit: sizes, {a, b} with a > b or out of
range) is `std::invalid_argument`.

### The adaptive loop in time

`AdaptiveIntegrator(integrator, adaptations, AdaptiveIntegratorOptions =
{})` (`adaptive_integrator.hpp`; the same constructor checks as
`AdaptiveStaticSolver`: at least one adaptation, none null, at most one
that `changes_mesh`; and a damping method when there are damping steps).
`integrate(meshed, t0, t1, AdaptiveTimeTarget, IntegrationOutput = {},
initial = nullptr)` integrates in pieces:

```mermaid
flowchart LR
    MP([MeshedProblem, state])
    INT["Integrator::integrate(meshed, t_k, end, ..., &state, max_steps)"]
    EST["estimate at t_k+1 (u_t, u_tt from the last step)"]
    TOL{"above the tolerance?"}
    AD["every adaptation adapts once"]
    BUD{"new mesh within max_nodes?"}
    TR["MeshedProblem(new mesh, same Problem); transfer_solution"]
    OUT(["AdaptiveIntegrationResult"])
    MP --> INT -->|t1 reached| OUT
    INT --> EST --> TOL -- no --> MP
    TOL -- yes --> AD --> BUD -- "no mesh" --> MP
    BUD -- yes --> TR --> MP
    BUD -- "no: adapting stops, one piece to t1" --> MP
```

- **Schedule**: `AdaptationSchedule{interval, steps}` (at least one; the
  interval finite and positive, steps at least 1). A piece ends at t_k +
  interval (passed as the integrator's t1, so it lands there; within
  1e-9·interval of t1 it is t1, and of a snapshot time on either side
  that time, so a snapshot at an adaptation time adds no step of rounding
  size) or after `steps` steps (`max_steps`), whichever comes first; the
  run ends at t1 without adapting there. Snapshot landings inside a piece
  count as steps. When t + interval rounds to t, `std::runtime_error`
  (the interval is too small at the current time).
- **Derivatives at an adaptation time**, at every node, from the piece's
  `before_last_step` (the free unknowns; requested only for a piece that
  can end in an adaptation: before t1, or with a step schedule), dt its
  `FreeState::dt` (the length the method used, not the difference of the
  times): u_t =
  (u(t) − u(t − dt)) / dt for time order 1 (and 0), at the Dirichlet
  nodes with `dirichlet_value` at the step's start; for time order 2 u_t = the
  velocities, u_tt = their difference quotient at the free nodes and
  `dirichlet_acceleration(k, t)` at the Dirichlet nodes, so no rate is
  evaluated at t − dt (u = √t has none at t0 = 0, where plain implicit
  Euler integrates it).
- **Tolerance**: as in the static loop (absolute and/or relative to the
  estimate at the first adaptation time, the smaller applies; the
  estimate is the first estimating adaptation's, which is then given it).
  At most the tolerance: nothing adapts (`tolerance_met`); no tolerance:
  it adapts at every adaptation time. Each time refines at most once and
  no piece is integrated again.
- **Node budget**: a new mesh above `max_nodes` is discarded
  (`node_budget`), adapting stops (the mesh only grows), and the rest is
  one piece to t1; a starting mesh above it throws. A run always reaches
  t1 unless something throws.
- **Refinement limit**: `max_refinements` (at least 1) counts
  refinements, not adaptation times (the schedule already fixes how often
  the run checks; each refinement builds a `MeshedProblem` and leaves a
  transfer error). After the last allowed one adapting stops and the rest
  is one piece to t1, with a message in the report like the node
  budget's.
- **Adaptive time steps**: with an error-controlling step controller
  (`AdaptiveStep`) the pieces on one problem continue its dt and the
  earlier steps its estimate reads through the anchor (below), so a run
  that never refines equals one `integrate()` call, rejections included;
  "every n steps" counts accepted steps. After a transfer the next piece
  gets an anchor at t with only `next_dt` (the dt the controller proposed
  on the old mesh; a new stretch, no earlier steps, which are on the old
  mesh), so its first steps (one for implicit Euler at time order 1, two
  for the midpoint method) have no estimate and keep that dt. The damping
  method must declare an error estimate too (`std::invalid_argument`
  before the first step). The report adds `rejected_steps`.
- **State**: the piece's `final` and its `IntegrationResult::anchor`,
  on the same problem when nothing changed (so pieces continue the run
  exactly: the step times are those of one `integrate()` that lands on
  the interval's adaptation times and the snapshot times, the "every n
  steps" times included without landing there, and the values bit for
  bit, Newmark too, since its acceleration is carried), else `transfer_solution` onto a new
  `MeshedProblem` (values, velocities and accelerations; reports
  "meshed_problem", "solution_transfer" in the `AdaptationPoint`).
- **Damping steps** (`AdaptiveIntegratorOptions{damping_steps = 0,
  damping_method}`, driver options: the same for every job): after each
  transfer the next `damping_steps` steps are a piece of their own
  (`IntegrationPiece::damping`), integrated by
  `integrator.with_method(damping_method)` (the same step controller,
  linear solver and assembler options); then the integrator's method
  continues on the same problem to the same adaptation time. The
  stretch to an adaptation time is fixed when it starts (its end and step
  limit), and the damping steps count in it; where only the method
  changes (damping steps that end before the adaptation time), the next
  piece gets the damping piece's `IntegrationResult::anchor`, so the
  step times are those of one uninterrupted stretch with the same step
  controller (with implicit Euler as both methods the run is the same
  bit for bit); a damping piece that reaches
  the adaptation time carries the adaptation itself, and damping steps
  still owed carry over an adaptation time that changed nothing (a new
  transfer starts them again). `damping_method` must support the
  problem's time order (`Assembler::time_order` against its
  capabilities, checked before the first step, `std::invalid_argument`).
  The acceleration at the end of the damping steps is decided from the
  damping method's declarations: one that keeps the acceleration
  (`keeps_acceleration`, Newmark) hands over its own (already in its
  Solutions); one that declares `velocity_quotient_is_acceleration`
  (implicit Euler) gets the u_tt of the derivatives ((u_t(t) − u_t(t −
  dt))/dt at the free nodes, `dirichlet_acceleration` at the Dirichlet
  nodes), which satisfies M₂ u_tt + B u_t + A u = b at t, the equation
  Newmark's start solves, without needing M₂ invertible; it goes into
  the piece's final state and into a snapshot at that time. Either way
  Newmark continues without its start. Any other damping method ahead of
  a method that keeps the acceleration (the midpoint method ahead of
  Newmark) throws `std::invalid_argument` naming both, before the first
  step. Never used on a failure: only when configured.
- **Output**: `snapshot_times` are split among the pieces (each on its
  piece's mesh), `keep_step_history` concatenates the records;
  `snapshot_every` throws (it counts the steps of one call).

`AdaptiveIntegrationResult{final, snapshots, history, pieces, report}`:
each `IntegrationPiece{problem, t_start, t_end, report (the
integrator's, of the damping method's run for a damping piece),
adaptation, damping}` and each `AdaptationPoint{time, steps,
estimate, outcome, reports}` (`AdaptationOutcome`: `refined`,
`tolerance_met`, `nothing_changed`, `node_budget`; `to_string`). Report
"adaptive_integrator": metrics `t0`, `t1`, `pieces`, `adaptation_times`,
`refinements`, `steps`, `damping_steps` (included in `steps`), `rejected_steps`, `nodes`, `triangles`, `max_nodes_used`,
`tolerance`; a message when the budget or `max_refinements` stopped
adapting. `AdaptiveTimeTarget{every, tolerance, max_nodes,
max_refinements}`.

Measured (`tests/adapt/adaptive_integrator.cpp`): a Gaussian bump (s =
0.08) moving through u_t − Δu = f on the unit square, dt = 1/64, adapting
every 2 steps (residual estimator, Dörfler 0.5, NVB) from 270 nodes:
implicit Euler ends with 2518 nodes and energy error 0.088; uniform NVB
refinement gives 0.385 with 1372 nodes and 0.250 with 2927. A transfer
leaves the old mesh's interpolation error in the new mesh's fine modes;
implicit Euler damps it within a few steps (after one uniform refinement
the run agrees with a run on the refined mesh from t0 to 2e-5 two steps
later), the midpoint method and Newmark do not: the same bump run with
the midpoint method ends with 3699 nodes and 0.51 (uniform 2927 nodes:
0.27), and on a standing wave Newmark's nodal u_t error grows from 0.23
on the starting mesh to 1.04 (|u_t| ≤ 4.4) while u improves (1.01 when
Newmark recomputes the acceleration with its start after each transfer
instead of taking the interpolated one).

Damping steps (same file): adapting every 4 steps of 1/128 (so the
midpoint method still takes about half the steps), the midpoint method
ends with 3532 nodes and 0.325; with 2 damping steps of implicit Euler
after each transfer with 2514 nodes and 0.088 (30 of 64 steps damping;
implicit Euler alone: 2514 nodes, 0.088), against uniform 2927 nodes,
0.262. After one uniform refinement at t = 1/32, the midpoint run with 2
damping steps agrees with implicit Euler's within 5e-5 relative and is
better than a midpoint run on the refined mesh from t0 (0.568 against
0.576 at t = 0.5), which keeps the interpolation error of its own initial
data. On the standing wave, Newmark's u_t error at t = 0.5 is 1.04
without damping steps, 0.84 with 1 and 0.68 with 2 (u 0.030, 0.024,
0.021); on the linear wave, which has no transfer error, 2 damping steps
make the errors of u and u_t 0.014 and 0.065 instead of 0.0006 and
0.0016 (implicit Euler's first-order time error), so the option stays
off by default.

## Determinism

Results never depend on addresses or hash order (SymEngine's sets of
symbols are sorted by name before use): loops, curves, nodes,
triangles and boundary edges are processed in index order, queues are FIFO
or ordered by explicit keys with index tie-breaks, maps are ordered, and
`compact()` keeps the index order of what remains.
The same input gives the same mesh, bit for bit. Marking orders equal
indicators by triangle index; refinement marks edges in the order of the
marked triangles (sorted, repetitions removed) and splits them in that
order, each after the edges it waits for.

Threads never change a result: assembly shares only the elements of one
colour among threads (no slot gets two additions in one colour), and the
iterative solvers' matrix-vector product sums each row in one thread;
`tests/fem/assembly_threads.cpp` and `tests/solve/threads.cpp` compare
1, 2 and 4 threads bit for bit.

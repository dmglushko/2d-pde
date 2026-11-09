# Equations, boundary conditions, and solutions

The optional C++23 `fem` module connects the existing triangle meshes to symbolic
coefficient functions, P1 assembly, sparse linear solvers, and time evolution.
The geometry API remains usable without the numerical dependencies.

Open [the computed solution gallery](showcase.html) for two stationary problems,
heat diffusion, a vibrating membrane, a damped membrane, and a wave problem with
an explicit `u_t` term on its boundary. Select **Wave: vibrating membrane** and
press **Play**. The gallery includes exact-solution comparisons and a separate
mesh-refinement experiment. Frames are calculated in C++, not simulated in the
viewer.

The [100-second wave examples](long-waves.md) use a larger 10-by-10 square:
a localized source with fixed Dirichlet edges, and a source-free Neumann wave.
They include energy/work checks and a recorded execution-time breakdown.

## Build and reproduce

Prerequisites: C++23 compiler, CMake 3.20+, Boost headers, Eigen3 development
headers, and Node.js for packaging the standalone gallery. SymEngine is a
compiled dependency. Enable FEM explicitly; geometry-only builds need neither
Eigen nor SymEngine. Downloading hash-verified SymEngine v0.14.0 is opt-in.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPDE2D_BUILD_FEM=ON \
  -DPDE2D_FETCH_SYMENGINE=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
./build/fem_showcase build/fem-showcase.json
node tools/render_solutions.mjs build/fem-showcase.json docs/fem/showcase.html
```

For an existing SymEngine installation, pass `-DSymEngine_DIR=/path/to/lib/cmake/symengine`.
Set `-DPDE2D_FETCH_SYMENGINE=OFF` to forbid dependency downloads. Link a CMake
consumer against the interface target `pde2d_fem`. The existing geometry and SVG
tests remain available through `./tools/test.sh`.

## A complete stationary problem

```cpp
#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "fem/solver.hpp"

using namespace pde2d::fem;
pde2d::discretization::RectangleGeometry geometry(1,1);
geometry.setCenter(.5,.5);
pde2d::discretization::MeshOptions meshing;
meshing.boundary.max_edge_length=.1;
meshing.boundary.preserve_corners=true;
meshing.refinement.max_edge_length=.15;
auto generated=pde2d::discretization::generate_mesh(geometry,meshing);
if(!generated.report.size_satisfied)
    throw std::runtime_error(generated.report.message);

Problem problem;
problem.equation.source=1; // Default operator is -u_xx-u_yy.
auto grounded=dirichlet(0);
problem.assign(grounded,{"left","right"});
problem.assign(neumann(0),{"bottom"});
problem.assign(robin(2,1,0),{"top"}); // 2*u + du/dn = 0.
AssemblyOptions integration;
integration.quadrature_degree=6;
auto solution=solve_stationary(generated.mesh,problem,integration);
// solution.values[i] is the value at generated.mesh.nodes[i].
// solution.report contains the measured algebraic residual.
```

One condition can be assigned to multiple named regions. Conditions and
assignments live on the problem; geometry tags remain reusable independently.
Every mesh boundary edge must receive exactly one assignment. Missing regions,
unknown names, multiple assignments to an edge, and incompatible Dirichlet
values at shared corners are errors. There is no implicit zero-Neumann fallback.

## Equation convention and symbolic data

The scalar input equation is

\[
m_2u_{tt}+m_1u_t+a_{xx}u_{xx}+a_{xy}u_{xy}+a_{yy}u_{yy}
+b_xu_x+b_yu_y+cu=f.
\]

Set fields `m2`, `m1`, `a_xx`, `a_xy`, `a_yy`, `b_x`, `b_y`, `c`, and `source`
on `problem.equation`. The mixed coefficient multiplies `u_xy` once: there is
**no implicit factor of two**. Coefficients may depend on `x,y,t` and remain
independent of the unknown. This is a non-divergence-form input; e.g.
`a_xx=-a` represents `-a*u_xx`, not `-(a*u_x)_x`. Time coefficients likewise
multiply derivatives directly: `m1*u_t` does not mean `(m1*u)_t`.

`Expr` is an actual SymEngine expression. Use `x,y,t` and mathematical functions
to construct formulas in C++; use `diff(expr,x)` for a symbolic derivative.
Boundary expressions may additionally use `nx,ny`, the outward normal of the
straight mesh edge. Normals exist only on boundary edges: interior
coefficients, `interpolate`, initial conditions and exact solutions for error
norms reject `nx,ny` with an `std::invalid_argument` (for example "Initial
value cannot depend on boundary normals (nx)"). `require_symbols(expr,
field_symbols, what)` performs the same check for your own contexts. `Evaluator` prepares an expression for repeated numerical
evaluation while retaining its symbolic tree. Unknown free symbols and
nonfinite evaluations are rejected. No user-provided opaque callback is needed.

Coefficients must be sufficiently smooth for the coefficient derivatives used
by the weak-form conversion. This module does not define interface laws for
discontinuous strong-form coefficients. Mixed space-time derivatives, nonlinear
terms, and coupled systems are outside the current implementation.

## Boundary representation

The full local input is

\[
\alpha u+\gamma u_t+\beta_q q_n+\beta_n\partial_nu
+\beta_xu_x+\beta_yu_y=g.
\]

These map to the fields `alpha`, `gamma`, `beta_flux`, `beta_normal`, `beta_x`,
`beta_y`, and `value` of `BoundaryCondition`. Keeping `beta_normal` explicit makes
normal derivative conditions convenient even though `du/dn=nx*u_x+ny*u_y`.

For this module's sign convention,

\[
A=\begin{pmatrix}a_{xx}&a_{xy}/2\\a_{xy}/2&a_{yy}\end{pmatrix},
\qquad q_n=-n^T A\nabla u.
\]

Thus `q_n=du/dn` for `-Delta u`; for `-k Delta u`, it is `k du/dn`.
This defines a signed mathematical conormal derivative; it is not automatically
a physical outward heat flux. In anisotropic equations it need not be parallel
to the normal derivative. The sign is explicit because the earlier design
discussion's integration-by-parts expression `F_n=n^T A grad(u)` is `-q_n`.

Convenient builders:

| Builder | Condition |
| --- | --- |
| `dirichlet(g)` | `u=g` |
| `neumann(g)` | `du/dn=g` |
| `robin(alpha,beta,g)` | `alpha*u+beta*du/dn=g` |
| `dynamic_robin(gamma,alpha,beta,g)` | `gamma*u_t+alpha*u+beta*du/dn=g` |
| `flux_neumann(g)` | `q_n=g` |
| `flux_robin(alpha,beta,g)` | `alpha*u+beta*q_n=g` |
| `dynamic_flux_robin(gamma,alpha,beta,g)` | `gamma*u_t+alpha*u+beta*q_n=g` |

Every edge's condition is reduced to one of three forms. Write the derivative
terms as one vector, `D·∇u` with `D = β_q c + β_n n + (β_x, β_y)` and the
conormal `c = -A n` (so `q_n = c·∇u`), and let `s = (-n_y, n_x)` be the unit
tangent of the straight edge, oriented counterclockwise (the domain on its
left), whatever the order of the edge's nodes:

| Form | When | Weak treatment |
| --- | --- | --- |
| Dirichlet | `D` reduces to zero on the edge | `u = g/α` is imposed at the nodes |
| Natural | `D` is exactly parallel to `c` (e.g. `du/dn` for isotropic `A`) | `q_n = (g - αu - γu_t)/β`: symmetric edge terms |
| Oblique | `D` is not parallel to `c` | `D·∇u = β q_n + τ du/ds`; the extra term `(τ/β) du/ds` is assembled |

The oblique form covers general oblique-derivative conditions, such as
`du/dn + r du/ds = g` (write `beta_x = -r*ny`, `beta_y = r*nx`), and plain
`du/dn` data under anisotropic or cross diffusion, where the normal derivative
is not the natural flux. On each edge `β = n·D / n·c` and
`τ = det(c, D) / n·c`. Two quantities must keep one strict sign on the edge:
the normal component of the conormal, `n·c = -nᵀAn` (always positive for an
elliptic `A`), and `β`, which vanishes exactly when `D` is tangential. A
purely tangential condition (`D·n = 0`, an ODE along the boundary) is rejected
("the derivative is tangential to the edge"). On a straight P1 edge `du/ds` is
constant, so the tangential term is a nonsymmetric 2×2 block that couples the
two edge nodes; the assembled stiffness is then nonsymmetric, so use SparseLU
or GMRES rather than conjugate gradients. The weak form adds no terms at
corners: each edge uses its own normal and tangent. Oblique-derivative
problems are not well posed in general, and the library does not check it:
`analyze` reports such regions as `BoundaryForm::Oblique`, says so in the
region's `form.reason`, and marks the problem `symmetric=false` with Unknown
evidence. With a smooth solution and an obliqueness that varies smoothly along
the boundary, P1 converges at second order in L2 (see
`tests/fem/oblique_conditions.cpp`). Where the oblique direction jumps at a
corner, the dual problem can have a corner singularity: on the unit square
we measured L2 rates down to about 1.5 for some 45° jumps (not for others;
it depends on the corner and the sign of the jump), while the H1 rate stayed
first order. The derivation is in
`cpp/fem/detail/edge_forms/oblique.hpp`.

General input does not imply that every boundary problem is well posed.
Equivalence checks cancel symbolic identities before substituting rounded
normals, then restrict expressions to each straight edge before the exact-zero
test. They do not accept approximate proportionality: data that are only
nearly parallel to the flux are treated as oblique, never projected. Divided
multipliers (`α` for Dirichlet, `β` for natural and oblique, `n·c` for
oblique) must stay nonzero on that edge; sampled zeros and sign changes are
rejected. These checks cannot prove nonvanishing for arbitrary symbolic
functions, so that remains a requirement on the supplied data.

Boundary `gamma*u_t` adds a boundary integral to the first-derivative matrix.
It is present during both heat and wave evolution. It uses the trace of the
same nodal field as the interior; it does not create a separate boundary field.

## Assembly and quadrature

With a symmetric split of the mixed derivative, assembly produces

\[
M_2\ddot U+M_1\dot U+KU=F.
\]

The interior stiffness uses `-A` in its gradient terms, and the effective
first-derivative coefficients are

\[
b_x-\partial_x a_{xx}-\tfrac12\partial_y a_{xy},\qquad
b_y-\partial_y a_{yy}-\tfrac12\partial_x a_{xy}.
\]

SymEngine differentiates these coefficient expressions. A condition
`alpha*u+gamma*u_t+beta*q_n=g` adds `alpha/beta` to the boundary stiffness,
`gamma/beta` to boundary `M1`, and `g/beta` to the boundary load. An oblique
condition `alpha*u+gamma*u_t+beta*q_n+tau*du/ds=g` adds the same terms plus
`(tau/beta) du/ds` tested against each basis function to the stiffness.
`assemble(mesh,problem,time,options)` returns unconstrained matrices and a list
of Dirichlet values. `constrain` eliminates both constrained rows and columns
and adjusts the load, preserving symmetry when the original matrix is symmetric.

`quadrature_degree=2` is the default: the triangle rule has three positive
weights and integrates total-degree-two polynomials exactly. Higher degrees
through 64 use positive Gauss rules with a triangle Duffy transform. Edge
integration uses positive Gauss–Legendre rules on the straight edges. The option
is polynomial exactness, not interpolation order; elements remain P1. Higher
quadrature cannot compensate for an under-resolved mesh or nonsmooth data.

## Linear solvers and time integration

`SolverOptions::method` selects `SparseLU` (default), `ConjugateGradient`, or
`GMRES`. `SolverOptions::preconditioner` selects the preconditioner of the
iterative methods: `Default` (the diagonal/Jacobi preconditioner, unchanged
behavior), `Identity`, `Diagonal`, `IncompleteCholesky` (CG only) or
`IncompleteLUT` (GMRES only), the latter two with Eigen's default parameters.
SparseLU accepts only `Default`; unsupported combinations throw. A user backend
can replace the built-in methods without editing the library:

```cpp
SolverOptions solver;
solver.backend=make_linear_solver_backend("MyBackend",
    [](const SolverOptions&){ return std::make_unique<MyBackend>(); });
```

`MyBackend` implements `LinearSolverBackend` (`prepare` once per matrix,
`solve` per right-hand side; the interface is in
`fem/linear_solver_types.hpp`, included by `fem/linear_solver.hpp`; see
[cpp/fem/EXTENDING.md](../../cpp/fem/EXTENDING.md)). Validation, the rank
check, the true-residual check, timings and factorization reuse stay shared,
and the same options work for `solve_linear`, `solve_stationary` and
`TimeOptions::solver`. Solver reports use the actual residual of the solved algebraic system.
An algebraic residual measures how accurately the discrete equations were
solved; it does not measure PDE approximation error. The gallery reports both
residuals and continuous exact-solution errors.

`SolverOptions::validation` defaults to `SolverValidation::Checked`: every
preparation checks numerical rank with SparseQR, and CG additionally checks
symmetry and positive definiteness with LDLT. These direct decompositions can
dominate iterative solve time and memory on large meshes, even with preparation
reuse. The numerical rank check is retained for CG because positive LDLT pivots
alone do not preserve the existing numerical rank rejection criterion.

For a matrix whose properties the caller has established, explicitly opt in:

```cpp
SolverOptions solver;
solver.method=LinearSolver::ConjugateGradient;
solver.validation=SolverValidation::AssumeValid;
auto solution=solve_linear(a,b,solver);
```

`AssumeValid` requires the caller to guarantee a nonsingular matrix, and for CG
a symmetric positive definite matrix. It skips SparseQR and the CG LDLT check.
CG and GMRES then prepare only their diagonal preconditioner, with no direct
decomposition; SparseLU still performs its requested LU factorization. Shape,
finite input, option, CG symmetry, and true residual checks remain active. A
small residual (including a zero RHS) cannot certify nonsingularity or positive
definiteness: invalid caller assumptions may produce a converged report for a
nonunique solution or an indefinite system. Use `Checked` when those guarantees
are needed. This policy applies to `PreparedLinearSolver`, `solve_linear`,
`solve_stationary`, and `TimeOptions::solver`; changing it invalidates prepared
solver reuse. Existing `factorizations` timing counters include preconditioner
setup, so a count of one in trusted iterative mode does not imply a direct
factorization; `rank_checks` and `rank_check_seconds` are zero in that mode.

For time evolution, include `fem/evolution.hpp`:

```cpp
problem.equation.m2=1; // u_tt-Delta u=0: a vibrating membrane.
problem.equation.source=0;
problem.boundaries.clear();
problem.assign(dirichlet(0),{"bottom","right","top","left"});
TimeOptions time;
time.method=TimeMethod::Newmark;
time.step=.01;
time.end=2;
const double pi=std::numbers::pi;
InitialConditions initial{.value=sin(pi*x)*sin(pi*y), .velocity=Expr(0)};
auto motion=evolve(generated.mesh,problem,initial,time);
// motion.frames includes the initial frame and the exact final time.
```

`Problem` is only the spatial problem (equation and boundary conditions).
Initial conditions are passed to `evolve` separately as `InitialConditions`:
`value` is u at `time.start`, and `velocity` is u_t at `time.start`. A
second-order scheme (Newmark) requires `velocity` and throws if it is
omitted. First-order schemes need only `value`; a given `velocity` only fills
the initial frame's `Frame::velocity` (0 when omitted).

Each of `value` and `velocity` is a `Field`: an expression in `x`, `y`, `t`
(interpolated at the nodes) or a `Vector` with one finite value per mesh node.
Nodal values restart an evolution, for example on a refined mesh:

```cpp
auto earlier=evolve(coarse,problem,initial,time);
auto moved=transfer_field(coarse,earlier.frames.back().values,fine).values;
time.start=time.end; time.end=4;
auto later=evolve(fine,problem,InitialConditions{moved},time);
```

A wrong size or a nonfinite entry throws ("Initial value must have one finite
entry per mesh node"); Dirichlet compatibility is checked as for expressions.
Newmark restarted from nodal `value` and `velocity` recomputes its
acceleration from the equation.

Each scheme is a `TimeIntegrator` (`fem/time_integration.hpp`). Step stamps
come from `TimeGrid` (`fem/time_grid.hpp`) unless `time.step_controller` sets a
`StepController` (`fem/step_control.hpp`). Every step is first a trial: a
controller's `judge()` may reject it, the trial is discarded (no frame, no
observer call, counted in `timings.rejected_steps`) and `next()` proposes the
retry. More than `time.max_consecutive_rejections` (default 100) rejected
trials in a row throw `std::runtime_error`, so a controller that never accepts
cannot loop forever. A step's `h` must equal its stamp difference up to
rounding (`step_size_consistent`). A user scheme runs through
`evolve` without library changes: set `time.integrator=[]{ return
std::make_unique<MyIntegrator>(); };` (called once per evolution; `method` is
then ignored, and reports use `TimeIntegrator::name()`). See
[cpp/fem/EXTENDING.md](../../cpp/fem/EXTENDING.md).

Backward Euler and Crank–Nicolson handle first-order evolution. Newmark uses
average acceleration (`beta=1/4`, `gamma=1/2`) and supports both `m2*u_tt` and
`m1*u_t` simultaneously, including dynamic boundary contributions. Time-dependent
data are reevaluated at each required time; constant contributions are prepared once.
A shorter final step reaches the requested end time.

Initial displacement must agree with strong Dirichlet data. Wave evolution also
requires compatible initial velocity. Moving Dirichlet boundaries require
consistent lifting; boundary values are enforced at each step. Expressions
used for moving prescribed values must be differentiable in time.

Pure Neumann stationary diffusion has a constant nullspace. This implementation
reports failure instead of choosing a normalization silently. Add an appropriate
Dirichlet condition or a coercive reaction/Robin term; automatic mean-zero gauges
are not implemented. Ordinary Galerkin P1 has no transport stabilization, and the
API does not certify ellipticity, well-posedness, or stability for arbitrary
coefficient choices.

`PreparedAssembly(mesh,problem,options)` owns an immutable snapshot with geometry,
evaluators and constant local contributions prepared once. Its `assemble(time)`
reevaluates time-dependent data. `PreparedLinearSolver` reuses rank checking and
factorization while the actual constrained matrix and solver options are equal;
changing either forces preparation. `evolve` uses both automatically.

## Repeated stationary solves, initial guesses and plans

`PreparedStationary(mesh,problem,assembly,solver)` (`fem/solver.hpp`) keeps the
prepared assembly and the last prepared linear solver, so repeated solves (a
time-dependent source sampled at several times, a parameter loop, an adaptive
loop's solves on one mesh) prepare once. `statistics()` counts solves,
assemblies, preparations and reuses. `solve_stationary` is the one-shot
wrapper and returns the same values.

```cpp
PreparedStationary stationary(fine,problem,{},cg);
auto solution=stationary.solve(0,moved);    // initial guess, e.g. transferred
```

An initial guess (`solve(time,guess)`, `solve_stationary(...,guess)`,
`solve_linear(a,b,options,guess)`, `PreparedLinearSolver::solve(b,guess)`)
starts the iterative backends (CG, GMRES); direct backends ignore it.
`SolverReport::initial_guess` says `None`, `Used` or `Ignored`.

A `SolvePlan` (`fem/plan.hpp`) bundles assembly options, solver options, time
settings (`TimePlan`) and the decisions explaining them. `plan_from_user(assembly,solver)`
and `plan_from_user(time_options)` record today's user choices; the overloads
`solve_stationary(mesh,problem,plan)`, `PreparedStationary(mesh,problem,plan)`
and `evolve(mesh,problem,initial,plan,output)` give the same results as the
option-based calls. Nothing chooses settings automatically yet; a planner
(`SolvePlanner`, `fem/planner.hpp`) returns a `SolvePlan`, and `UserPlanner`
is the identity.

Each `Decision` has a typed subject (`DecisionSubject::LinearSolver`,
`TimeSteps`, `Quadrature`, ...; `decision_subject_name` prints it). Override a
setting with a setter, which changes the option and its decision together:

```cpp
auto plan=plan_from_user(assembly,solver);
plan.set_linear_solver(LinearSolver::ConjugateGradient)
    .set_preconditioner(Preconditioner::IncompleteCholesky);
plan.set_quadrature_degree(4,{"source has degree 3",Evidence::Proven,
                              DecisionSource::Automatic});  // a chooser's basis
```

Every solve validates the plan at entry (`validate_decisions`): a decision that
no longer matches its option, for example after `plan.solver.method=GMRES`
without the setter, throws `"Stale decision: linear solver records "SparseLU"
but the options give "GMRES""`; duplicate subjects and mesh subjects in a
solve plan throw too (`"Decision subject mesh size does not belong to a
solve plan (owner Mesh; use SolvePlan::mesh)"`). Every subject has an owner
(`decision_subject_owner`: Solve, Time, Mesh or Adapt) and each plan has one
binding table (`solve_plan_bindings`, `mesh_plan_bindings`) from which its
user decisions, setters and validation all come; `plan.update(subject,
change, why)` changes any bound option with its decision. `TimeOptions` is a
`TimePlan` plus `EvolutionOutput` plus assembly and solver options, so every
time setting is declared once.

A `MeshPlan` (`fem/mesh_plan.hpp`, `mesh_plan_from_user(mesh_options)`) is
the meshing counterpart: `MeshOptions` with decisions for the mesh algorithm,
boundary treatment, mesh size and size field (and a refinement criterion
when `RefinementOptions::criterion` is set), with setters
(`set_interior_algorithm`, `set_boundary_treatment`, `set_max_edge_length`,
`set_size_field(field, descriptor)`, `set_refinement_criterion`) and
`update`. Put it in `plan.mesh`; a solve given the `MeshResult`
(`solve_stationary(result,problem,plan)`, `evolve(result,...)`) checks it
against `result.options`.

`plan.analysis` holds the analysis a planner based its decisions on, bound to
its inputs: `make_analysis_record(problem[, mesh][, options])`
(`fem/analysis.hpp`) records the properties with the problem fingerprint, the
mesh fingerprint (when it sampled a mesh) and the sampling options, and is
the only way to make one. Solves, evolutions and `adapt` throw `"Stale
analysis: ..."` when the plan's analysis was made for another problem or
mesh; `refresh_analysis(plan, problem, mesh)` recomputes it with its own
options.

A solve given a `MeshResult` also checks its report: meshing can return a
mesh that misses its size target (`report.size_satisfied == false`), and
`PreparedStationary`, `solve_stationary`, `evolve` and `adapt` then throw
`MeshResult misses its size target (<report.message>); check
report.size_satisfied or pass accept_unsatisfied(result)`. Pass
`accept_unsatisfied(result)` (`fem/mesh_acceptance.hpp`) to solve on it
anyway; the record keeps the unsatisfied report. `TriangleMesh` overloads do
not check.

## Adaptive loop

The adaptive loop of a stationary problem is a composition of public
components, each usable on its own: `PreparedStationary` (solve, warm
started from a guess), `estimate_error` (an `ErrorEstimator`, default the
gradient-recovery estimator), a `MarkingStrategy` (`fem/marking.hpp`;
`BulkMarking` is Dörfler marking with `mark_bulk`),
`discretization::refine_mesh(result, marked, options)` and
`transfer_field`. `adapt` (`fem/adaptivity.hpp`) runs them in that order:

```cpp
AdaptOptions options;
options.marker = std::make_shared<BulkMarking>(0.6); // unset: BulkMarking{0.5}
options.max_iterations = 8;          // refine-and-solve iterations after the first
options.tolerance = 1e-3;            // stop once estimate.global <= tolerance
options.transfer.max_outside_distance = 0.02;   // curved boundary
auto history = adapt(mesh_result, problem, plan, options);
auto &u = history.solution.values;   // on history.mesh.mesh
```

| `AdaptOptions` field | Default | Meaning |
| --- | --- | --- |
| `estimator` | gradient recovery (plan's execution) | `std::shared_ptr<const ErrorEstimator>` |
| `marker` | `BulkMarking{0.5}` | `std::shared_ptr<const MarkingStrategy>` |
| `max_iterations` | 10 | refine-and-solve iterations after the first solve |
| `tolerance` | 0 (off) | stop once the global estimate is at most this |
| `refinement` | each result's own options | `MeshOptions` of `refine_mesh(result, marked, options)` |
| `warm_start`, `transfer` | true, `{}` | seed each solve with `transfer_field` of the previous solution |
| `time` | 0 | time at which the stationary problem is solved and estimated |
| `accept_unsatisfied_refinement` | false | keep going on a refined mesh that missed its size target |
| `solve_planner` | unset (plan as given) | `std::shared_ptr<const SolvePlanner>`, run before every solve |
| `mesh_planner` | unset (`refinement`) | `std::shared_ptr<const MeshPlanner>`, run after marking; its options refine the mesh |

Across iterations the plan follows the mesh: a mesh-bound `plan.analysis` is
recomputed on every refined mesh, `plan.mesh` (when set) is replaced by the
mesh plan of the refinement options when they differ from it (or by the mesh
planner's plan), and then the solve planner, if any, plans the solve with the
iteration's mesh and the previous `AdaptiveIteration` (`PlanningContext`).
The planner's plan is validated before solving ("Solve planner <name>
returned an invalid plan: ...") and recorded: `plan.planner`, the run
record's strategies and a `DecisionSubject::SolvePlanner` decision of the
iteration.

The loop stops, checked after every solve and estimate in this order, with
`AdaptiveStop::ToleranceReached`, `MaxIterations`, `NothingMarked` (the
strategy marked no element) or `UnsatisfiedRefinement` (the refined mesh
missed its size target; that mesh is not solved on). The initial
`MeshResult` must meet its size target or be passed as
`accept_unsatisfied(result)`. The driver adds no numerics: the same calls
composed by hand give the same meshes and values bit for bit
(`tests/fem/adaptive_loop.cpp` checks this).

`AdaptiveHistory` holds the `settings` it ran with (`AdaptiveSettings`:
estimator and marker names, descriptors and whether they were the defaults,
`max_iterations`, `tolerance`, the requested `refinement` options,
`warm_start`, `transfer` options, `time`, `accept_unsatisfied_refinement`
and the planners' names), the final `mesh`, `solution` and `estimate`, the
`stop` reason and message, and one `AdaptiveIteration` per solve: its
`RunRecord` (plan, mesh fingerprint and provenance with the refinement
report that produced the mesh), solver iterations and initial-guess use,
the `TransferRecord` that seeded it (source and target nodes, extrapolated
nodes, largest outside distance), an `EstimateSummary` (estimator name and
descriptor, global estimate, elements, largest and mean indicator) and,
unless it was the last, a `MarkingRecord` (strategy name, descriptor,
marked and total elements), the `MeshOptions` refine_mesh was called with
(`refinement_options`, kept even when the refined mesh missed its size
target and was never solved on), the `MeshReport` of the refined mesh (with
`MarkedRefinementReport` counters), the refined result's mesh run record
(`refinement_run`) and its `Decision`s (with planners, a
`SolvePlanner` decision first and a `MeshPlanner` decision before the
refinement):
`DecisionSubject::Marking` (choice `Bulk(fraction=0.6)`, or
`<name> (not replayable)` without a descriptor) and
`DecisionSubject::Refinement` (choice `marked longest-edge bisection
(<algorithm>, <boundary treatment>)`).

`write_json(out, history)` (`fem/adaptivity_json.hpp`) writes
`"schema": "pde2d.adaptive_history"`, `"schema_version": 1`, then `stop`
(`reason`, `message`), `settings` (`estimator` and `marker` with `name`,
`descriptor`, `default`; `max_iterations`, `tolerance`, `refinement`,
`warm_start`, `transfer` (`max_outside_distance`, `execution`), `time`,
`accept_unsatisfied_refinement`, `solve_planner`, `mesh_planner`),
`iterations` (each: `index`, `record` = the full
run record document, `solver` (`iterations`, `initial_guess`), `transfer`,
`estimate`, `marking`, `refinement`, `refinement_options`,
`refinement_run` (a mesh run record document), `decisions`) and `final`
(`mesh_fingerprint`, `nodes`, `elements`, `estimate`), with the run-record
conventions (absent values `null`, `max_digits10` numbers).
`read_adaptive_history_json` checks the schema; `diff_records` compares two
histories leaf by leaf and ignores nested `timings`. Indicators and nodal
values are not written.

## Transient adaptive loop

`adapt_evolution` (`fem/adaptivity.hpp`) composes the same components for
an evolution. It splits the plan's interval into `windows` equal windows
(`adaptive_window_times`) and, per window: `evolve` on the current mesh,
`estimate_error` of the last frame at the window's end, then (except after
the last window, and only while the estimate exceeds `tolerance` when one
is set) mark, `refine_mesh` and move the last frame to the new mesh with
`transfer_field` as a nodal restart (`InitialConditions` with nodal
`Field`s; the velocity too when the initial conditions have one). The
restart's Dirichlet nodes are set to the problem's Dirichlet data (and its
time derivative) at the restart time, from `PreparedAssembly` on the new
mesh: interpolated boundary values would fail `evolve`'s compatibility
check.

```cpp
EvolutionAdaptOptions options;
options.windows = 6;
options.marker = std::make_shared<BulkMarking>(0.6);
auto history = adapt_evolution(mesh_result, problem, initial, plan, options);
history.evolution.frames.back();          // final state on history.mesh
write_json(out, history);                 // "pde2d.evolution_adaptive_history"
```

| `EvolutionAdaptOptions` field | Default | Meaning |
| --- | --- | --- |
| `windows` | 4 | equal windows of `[plan.time->start, plan.time->end]` |
| `estimator`, `marker` | gradient recovery, `BulkMarking{0.5}` | as in `AdaptOptions` |
| `tolerance` | 0 | refine after a window only when its estimate exceeds this (0: always) |
| `refinement` | each result's own options | `MeshOptions` of the marked refinement |
| `transfer` | `{}` | `TransferOptions` of the restart |
| `accept_unsatisfied_refinement` | false | evolve on a refined mesh that missed its size target; otherwise it is discarded and the next window stays on the current mesh |
| `output` | `{}` | `EvolutionOutput` of every window's `evolve` |

The plan must have time settings and uniform steps (a step controller
throws "adapt_evolution needs uniform time steps"); each window runs the
plan with that window's `start` and `end`, `plan.mesh` updated as in
`adapt` and `plan.analysis` refreshed for every new mesh. No planners run.
`EvolutionAdaptiveHistory` holds the `settings`, one `EvolutionWindow` per
window (bounds, the window's `RunRecord`, the `RestartRecord` that started
it: transfers, Dirichlet nodes and the largest correction of a transferred
value; the end-of-window `EstimateSummary`, `MarkingRecord`, requested
refinement options, `MeshReport`, mesh run record, whether the refinement
was accepted, and the `Marking`/`Refinement` decisions) and the final `mesh`,
`evolution` (of the last window) and `estimate`. The driver adds no
numerics: `tests/fem/transient_adaptivity.cpp` composes the loop by hand and
compares bit for bit, and shows on the `heat_moving_peak` reference problem
that the adapted error ends well below the unadapted coarse mesh's, at about
the accuracy of a uniform mesh with more nodes.

## Reference problems and plan comparisons

`fem/reference/problems.hpp` is a small library of complete problems with
exact or manufactured solutions (`ReferenceProblem`: `name`, `geometry`
with named regions, `problem`, `transient` (initial conditions and window)
for time-dependent ones, `exact` u(x, y, t), `notes` and suggested `mesh`
options). `reference_problems()` returns all of them;
`find_reference_problem(name)` one. Names are stable:

| Name | What it tests |
| --- | --- |
| `l_shape_corner` | Laplace on the L-shaped domain, u = r^(2/3) sin(2θ/3): corner singularity, H1 rate 2/3 on uniform meshes |
| `boundary_layer` | −εΔu + u_x + u_y = f, ε = 0.05: boundary layers at x = 1 and y = 1, nonsymmetric |
| `variable_anisotropic_diffusion` | variable anisotropic diffusion with a mixed term, Dirichlet and conormal flux conditions |
| `oblique_square` | −Δu + u = f with an oblique-derivative condition on three sides |
| `annulus_mixed` | Poisson on an annulus: Dirichlet outside, Robin on the hole |
| `heat_moving_peak` | u_t − Δu = f with a moving Gaussian peak and time-dependent Dirichlet data, t ∈ [0, 0.6] |
| `wave_standing` | u_tt − Δu = 0, standing wave with initial velocity, t ∈ [0, 0.5] |

`fem/reference/compare.hpp` runs plans on one of them and measures the
error for the work spent:

```cpp
auto reference = find_reference_problem("annulus_mixed");
auto c = compare_plans(reference, mesh_plan_from_user(reference.mesh),
                       {lu_plan, cg_plan});
c.entries[1].errors.l2;            // L2 and H1-seminorm error (error_norms)
c.entries[1].work.solver_iterations;
c.entries[1].record;               // the full RunRecord
auto m = compare_mesh_plans(reference, {coarse, fine}, plan);
write_json(out, c);                // "pde2d.plan_comparison", version 1
```

`compare_plans` meshes the geometry once with the mesh plan and solves with
every plan; `compare_mesh_plans` meshes with every mesh plan and solves with
one plan. Each solve gets the mesh plan it ran on as `plan.mesh` (and a
mesh-bound analysis refreshed), so its record describes what ran.
Stationary problems are compared at t = 0, transient ones at the end of the
reference window, which the plan's `start` and `end` must match exactly.
Each `ComparisonEntry` has its `index`, a `label` (`comparison_label`), the
`RunRecord`, `errors` (`l2`, `h1_seminorm`, `time`), `work` (nodes,
elements, assembly calls, linear solves, solver preparations and
iterations, time steps, mesh refinement steps and edge flips) and `timings`
(mesh and solve seconds). The JSON writes `reference`, `notes`, `kind`
(`solve plans` or `mesh plans`) and `entries` (each with the same groups and
the full run record under `record`); `read_plan_comparison_json` checks the
schema and `diff_records` skips the timings.

## Optional parallel execution and verification

Configure with `-DPDE2D_ENABLE_OPENMP=ON`, then select a policy explicitly:

```cpp
time.assembly.execution.backend=pde2d::ExecutionBackend::OpenMP;
time.assembly.execution.threads=4;
```

The default remains serial in either build. Element and boundary numerical work,
initial fields and prepared error diagnostics can run in parallel. Symbolic
preparation, ordered sparse reduction and mesh topology mutation remain serial.
Workers own separate outputs; deterministic reduction preserves serial results.
Requesting OpenMP in a serial-only build throws an explicit error.

`time.frame_stride` bounds retained frames (initial and final always included),
`store_reports=false` avoids retaining per-step reports, and `observer` receives
every accepted state synchronously. Borrowed observer data last only for the
callback. Defaults preserve every frame and report.

See [the independent verification guide](../archive/performance/verification-build-2026-11-06.md)
for isolated scikit-fem/SciPy comparisons, analytical convergence checks and
serial/2/4-worker tests.

## Native profiling and report output

Timings are always collected in returned native reports; no timer option or
external script is needed. The library stays quiet until explicitly asked to
format a summary. Include `fem/report.hpp` (also exported by `fem/fem.hpp`):

```cpp
#include "fem/report.hpp"
#include <iostream>

pde2d::fem::write_report(std::cout, motion);          // pde2d::fem::Evolution
pde2d::fem::write_report(std::cout, solution);        // StationarySolution or LinearSolution
pde2d::fem::write_report(std::cout, solution.report); // pde2d::fem::SolverReport
pde2d::fem::write_report(std::cout, motion.record);   // pde2d::fem::RunRecord
```

### Run records

`StationarySolution::record` and `Evolution::record` (`fem/record.hpp`) carry
what is needed to reproduce and compare a run, as fields:

- the plan that ran, with its decisions, mesh plan and analysis (if any);
- the inputs, by deterministic fingerprint (`pde2d::Fingerprint`, FNV-1a 64,
  explicit little-endian encoding): the mesh (`mesh_fingerprint`: node bit
  patterns, elements, boundary edges, tags), the problem
  (`problem_fingerprint`: the canonical SymEngine string of every `Equation`
  field, then each assignment's regions and condition fields, in assignment
  order) and, for evolutions, the initial value and velocity
  (`field_fingerprint`, expression or nodal);
- mesh provenance (`MeshOptions`, `MeshReport`, whether a boundary context was
  kept, and the mesh run record of the meshing call, `run`: operation,
  fingerprints, options, version, work counters, quality statistics and
  time; see `cpp/discretization/README.md`) when the solve was given a
  `MeshResult`;
- the stationary evaluation time, and for a stationary solve
  `stationary_solve`: the initial-guess use, the fingerprint of the guess
  as passed (`field_fingerprint`, kind `nodal`), the iterations, whether it
  converged and the final residual norms;
- `strategies`: what ran for the time integrator, step sequence, linear solver,
  size field, refinement criterion and the solve and mesh planners, each `built-in` (the options rebuild it), `described` (a
  `StrategyDescriptor` names it and its parameters) or `not replayable` (a
  factory without descriptor); `record.replayable()` is true when all are
  built-in;
- versions: library version and git commit at configure time
  (`pde2d::library_version()`, "unknown" and empty outside CMake or git),
  Eigen and SymEngine;
- execution policy and OpenMP availability, workload, work counters (with
  `solver_iterations`, summed over every linear solve), an
  evidence summary of the decisions, and the total wall-clock time
  (`timings.total_seconds`, the only field that differs between identical
  runs).

`write_json(out,record)` (`fem/record_json.hpp`) writes a versioned JSON
document: `"schema": "pde2d.run_record"`, `"schema_version": 1`, then
`versions`, `replayable`, `problem`, `mesh` (`fingerprint`, `provenance`
with `run`, a mesh run record document), `workload`, `stationary_time`,
`initial`, `stationary_solve` (`initial_guess`, `guess`, `outcome`:
`iterations`, `converged`, `residual_norm`, `relative_residual`), `plan`
(`assembly`, `solver`,
`time`, `decisions`, `mesh` (with its `planner`), `analysis` (with
`problem_fingerprint` and `mesh_fingerprint`), `planner`), `strategies`,
`execution`, `work`,
`evidence` and `timings`. Absent optional values are `null`. Numbers use the
classic locale and `max_digits10` digits, so parsed doubles are the run's bit
for bit; non-finite values are the strings `"nan"`, `"inf"`, `"-inf"`. Adding
a key keeps the schema version; renaming or removing one increments it.

```cpp
std::ofstream file("run.json");
pde2d::fem::write_json(file, solution.record);
std::ifstream in("run.json");
auto document = pde2d::fem::read_run_record_json(in);   // checks the schema
for (const auto &d : pde2d::fem::diff_records(old_record, new_record))
  std::cout << d.path << ": " << d.left << " -> " << d.right << '\n';
```

`diff_records` compares two records or parsed documents leaf by leaf (paths
such as `plan.decisions[2].choice`) and ignores `timings` (also nested, as
in adaptive histories and mesh run records) unless
`{.ignore_timings = false}`. It also skips `outcome` objects (what a solve
produced from its inputs: iterations and residuals) unless
`{.ignore_outcomes = false}`, so by default a diff compares inputs, settings
and work and a changed input shows as its fingerprint. Mesh options in
records include `refinement.criterion` (whether a user refinement criterion
was set) and `refinement.criterion_descriptor`.

These overloads return `std::ostream&`, emit one summary without per-step output,
and preserve the caller's formatting flags, precision, fill, width and locale.
`LinearSolution` prints its solver report. For `solve_stationary`, those timings
cover the linear solve, not preceding assembly and constraint elimination.
A `PreparedLinearSolver::solve` report covers that RHS only; preparation is
available separately through `preparation_timings()`.

`Evolution::timings` contains disjoint wall-clock phases in seconds:
`assembly_seconds`, `constraints_seconds`, `solver_seconds`, `observer_seconds`,
and `other_seconds`, all included in `total_seconds`. Observer time measures
only synchronous callback invocation; constructing/copying frames and retaining
output belong to `other_seconds`. The nested `timings.solver` contains solver
subphases and must not be added again to the evolution total. Counters include
initial acceleration solves for Newmark, although per-step reports do not.
`store_reports=false` retains all aggregate timings and work counters.
Observers still receive the initial state and every accepted step independently
of frame stride; callback exceptions propagate to the caller.

`Evolution::record` records mesh node, element and boundary-edge counts,
quadrature degree, time method/interval/step,
solver options, OpenMP build availability, and the selected `execution` policy.
Its `execution.threads` is
the requested count (zero means runtime default), **not** measured team size.
Small loops, nested execution and runtime limits may cause serial or smaller
teams. The formatter labels this distinction explicitly. The default selected
backend remains serial in both builds. These timings measure elapsed time,
not CPU consumption, and overlapping application activity can affect them.

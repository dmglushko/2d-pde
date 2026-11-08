# `cpp/fem` — scalar P1 finite elements

Header-only, optional module (CMake `PDE2D_BUILD_FEM=ON`; needs Eigen3 and
SymEngine). User-level documentation with equations, sign conventions and
examples is in [docs/fem/README.md](../../docs/fem/README.md). This page maps
the code.

## Pipeline

```
Problem (equation + boundary conditions)        problem.hpp
   │   symbolic coefficients (SymEngine Expr)    expression.hpp
   ├─ bind_boundary(mesh, problem) → regions, edge frames
   │                                             boundary_assignment.hpp
   ├─ analyze(problem[, mesh]) → ProblemProperties (each with Evidence)
   │                                             analysis.hpp, analysis_types.hpp
   ▼
SolvePlan {assembly, solver, time, decisions}   plan.hpp (plan_from_user)
   ▼
PreparedAssembly(mesh, problem)                 assembly.hpp
   ├─ bind_boundary (same binding as analysis)  boundary_assignment.hpp
   ├─ weak_form_coefficients(equation)          weak_form.hpp
   ├─ classify_edge_condition → EdgeForm        boundary_classification.hpp
   │    (DirichletEdge | NaturalEdge; sampled divisor sign check)
   └─ P1 geometry + local kernels               p1_element.hpp, quadrature.hpp
   ▼
   ├─ interior elements, boundary edges, scatter detail/assembly_*.hpp
   ▼
AssembledSystem {K, M1, M2, F, Dirichlet}       system.hpp (element independent)
   ▼
constrain() + PreparedLinearSolver              linear_solver.hpp
   │   LinearSolverBackend (LU / CG / GMRES / user), optional initial guess
   ▼
stationary: PreparedStationary(mesh, problem, plan).solve(t[, guess])
            solve_stationary() = one-shot wrapper                solver.hpp
evolve(mesh, problem, InitialConditions, opts | plan + output)  evolution.hpp
   ├─ StepController (default: TimeGrid)        step_control.hpp, time_grid.hpp
   │    next() → trial; judge() accepts or rejects; rejected → retry
   ├─ TimeIntegrator (BE / CN / Newmark / user) time_integration.hpp
   │    trial() / accept() / reject(); IntegratorServices: solve,
   │    start_dirichlet, assemble(t)
   └─ initial fields: Expr or nodal Vector      interpolation.hpp (Field)
   ▼
StationarySolution {values, report, record}     solver.hpp, record.hpp
Evolution {frames, reports, timings, metadata.decisions, record}
   → write_report()                             report.hpp, timing.hpp, decision.hpp
L2 / H1 errors against exact solutions          error_norms.hpp

Adaptivity (stationary): solve → estimate → mark → refine → transfer → seed
   estimate_error → ErrorEstimate {indicators, global}   estimation.hpp
   mark_bulk(indicators, θ) → element indices            estimation.hpp
   discretization::refine_mesh(result, marked, options)  triangulate.hpp
   transfer_field(old mesh, values, new mesh)            transfer.hpp
   PreparedStationary(new mesh, …).solve(t, transferred) solver.hpp
Transient restart: transfer_field(last frame) → InitialConditions{Vector}
```

One adaptive iteration (see `tests/fem/adaptive_loop.cpp`):

```cpp
auto plan = plan_from_user(AssemblyOptions{}, cg); // an iterative solver
auto u = solve_stationary(result.mesh, problem, plan).values;
auto estimate = estimate_error(result.mesh, problem, u);
auto marked = mark_bulk(estimate.indicators, 0.6);
auto finer = discretization::refine_mesh(result, marked, result.options);
auto start = transfer_field(result.mesh, u, finer.mesh,
                            {.max_outside_distance = 0.02}); // curved domain
auto next = PreparedStationary(finer.mesh, problem, plan)
                .solve(0, start.values); // the transfer seeds the solve
```

`fem.hpp` includes everything. Parallel loops use `pde2d::parallel_for`
from `cpp/execution.hpp`; meshes are `discretization::TriangleMesh`.

## Headers

| Header | Responsibility | Depends on |
| --- | --- | --- |
| `evidence.hpp` | `Evidence` (Proven / Sampled / Unknown), `Property<T>` | — |
| `decision.hpp` | `Decision` records (subject, choice, basis, evidence, source) | evidence |
| `expression.hpp` | `Expr`, symbols `x y t nx ny`, `require_symbols` by context, compiled `Evaluator` | SymEngine |
| `problem.hpp` | `Equation`, `BoundaryCondition` + factories, `Problem` | expression |
| `weak_form.hpp` | named weak-form coefficients (`weak::Term`), `weak_time_dependence` | problem |
| `quadrature.hpp` | triangle and edge rules by exactness degree | — |
| `p1_element.hpp` | P1 gradients, edge frames/normals, local kernels | weak_form |
| `boundary_classification.hpp` | general BC → `EdgeForm` (`DirichletEdge` or `NaturalEdge`), pure symbolic; shared sampled divisor sign check | p1_element, problem, quadrature |
| `boundary_assignment.hpp` | `bind_boundary`: validated region → condition binding and the frame (length, outward normal) of every boundary edge; shared by assembly and analysis | p1_element, problem |
| `system.hpp` | `SparseMatrix`, `Vector`, `DirichletValue`, `DirichletDerivative`, `AssembledSystem`, `time_order`, `numerically_symmetric` | Eigen |
| `assembly.hpp` | `AssemblyOptions`, `PreparedAssembly`, validation, parallel evaluation | all above, system |
| `detail/assembly_elements.hpp` | private: interior coefficients, quadrature points, element blocks | p1_element, weak_form |
| `detail/assembly_edges.hpp` | private: edge classification of the bound edges, named edge evaluators, edge blocks | boundary_assignment, boundary_classification |
| `detail/assembly_scatter.hpp` | private: deterministic serial scatter | system |
| `linear_solver.hpp` | Dirichlet lifting, `SolverOptions`, `LinearSolverBackend` (+ built-in LU/CG/GMRES, preconditioners, `solve_with_guess`), `PreparedLinearSolver`, initial guesses (`InitialGuess`), reuse, `solver_decisions` | system, timing, decision |
| `plan.hpp` | `SolvePlan` (assembly, solver, `TimePlan`, decisions), `plan_from_user`, `time_decisions` | assembly, linear_solver, step_control |
| `record.hpp` | `RunRecord` (plan, execution, workload, work, `EvidenceSummary`), `summarize_evidence` | plan, execution |
| `solver.hpp` | `PreparedStationary` (prepared assembly + reused linear solver, `StationaryStatistics`), `StationarySolution`, `solve_stationary` wrappers (includes linear_solver.hpp) | assembly, linear_solver, plan, record |
| `interpolation.hpp` | nodal interpolation of a field (rejects normals); `Field` (Expr or nodal Vector), `validate_field`, `nodal_values` | system, expression |
| `time_grid.hpp` | time stamps and step sizes | — |
| `time_integration.hpp` | `TimeIntegrator` interface (transactional `trial`/`accept`/`reject`), `IntegratorServices`, schemes, `TimeIntegratorFactory` | system |
| `step_control.hpp` | `StepController` interface (`judge` → `StepVerdict`), `UniformStepController` (TimeGrid), `step_size_consistent` rounding bound | time_grid, time_integration |
| `evolution.hpp` | `evolve()` driver (options or plan + `EvolutionOutput`), trial/accept/reject loop, frames, observer, accounting, output/step checks, decisions, record | all above |
| `timing.hpp` | timing structs, `timing::Accumulate` | — |
| `analysis_types.hpp` | `AnalysisOptions`, `Sign`, `Definiteness`, `BoundaryForm`, `ProblemProperties`, `BoundaryRegionProperties`, `SystemProperties`, `ClassifiedEdge` | evidence, weak_form, boundary_classification |
| `analysis.hpp` | `analyze()` → `ProblemProperties` / `SystemProperties` with evidence; `analyze_boundary_region` | analysis_types, boundary_assignment, detail/analysis_* |
| `detail/analysis_interior.hpp` | private: constant signs, sampled signs and definiteness, interior properties | analysis_types, weak_form, quadrature |
| `detail/analysis_regions.hpp` | private: mesh-free region judgement, per-region accumulation, Robin terms, nullspace | analysis_types, boundary_classification |
| `report.hpp` | text summaries of solves, evolutions, run records, decisions and analyses | evolution, solver, record, analysis |
| `error_norms.hpp` | prepared L2/H1 error norms (rejects normals) | system, quadrature |
| `estimation.hpp` | `ErrorEstimator` interface, `GradientRecoveryEstimator` (ZZ, H1 seminorm), `estimate_error`, Dörfler `mark_bulk` | p1_element, problem, system |
| `transfer.hpp` | `transfer_field`: P1 nodal field onto another mesh (point location + interpolation, documented outside rule) | discretization/point_location, system |

## Invariants

- **P1 only.** One scalar unknown per vertex; no element-order option.
  Higher order belongs in a separate module (see EXTENDING.md).
- **Preparation is symbolic, assembly is numeric.** Workers in parallel loops
  only call prepared `Evaluator`s; they never construct or copy `Expr`.
- **Determinism.** Scatter happens serially in element/edge order, so OpenMP
  and serial builds give bit-identical matrices.
- **Errors are exceptions** (`std::invalid_argument` for unsupported input,
  `std::domain_error` for non-finite values, `std::runtime_error` for
  solver failures). Nothing prints unless `write_report` is called.
- `*_detail` namespaces (and everything in `detail/`) are private. Tests and
  examples use public API only.
- **One judgement, one function.** Analysis and the solve share their
  checks: `bind_boundary` (region binding, edge frames),
  `weak_time_dependence` (caching), `divisor_keeps_strict_sign` on
  `edge_divisor_samples` (boundary divisors, Evidence::Sampled),
  `time_order` (integrator `check()`), `numerically_symmetric` (the CG
  backend). Do not re-implement one of them elsewhere.
- **Decisions are recorded.** Every choice that shapes a solve appears in
  `SolverReport::decisions` or `EvolutionMetadata::decisions`, copied from
  the `SolvePlan` that ran. Today all are user-selected (`plan_from_user`);
  nothing selects solvers automatically yet. A chooser returns a `SolvePlan`.
- **Results are reproducible.** `StationarySolution::record` and
  `Evolution::record` hold the plan, execution policy, OpenMP availability,
  workload, work counters and evidence summary as structured fields.
- **Steps are transactional.** Only accepted trials change integrator state,
  produce frames or reach the observer; the default controller accepts
  every trial, so the default path is unchanged.
- **Estimators report, they do not decide.** `estimate_error` returns
  indicators and a global value; marking and refinement are separate calls
  the user (or a future automatic loop, recording a `Decision`) makes.
- **Normals only on boundaries.** `nx`, `ny` are bound in boundary
  expressions; interior coefficients, interpolated and initial fields and
  exact solutions are checked with `require_symbols(..., field_symbols, ...)`.

See [EXTENDING.md](EXTENDING.md) for adding terms, boundary forms, schemes,
linear solver backends and element types.

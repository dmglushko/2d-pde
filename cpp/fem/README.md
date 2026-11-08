# `cpp/fem` — scalar P1 finite elements

Header-only, optional module (CMake `PDE2D_BUILD_FEM=ON`; needs Eigen3 and
SymEngine). User-level documentation with equations, sign conventions and
examples is in [docs/fem/README.md](../../docs/fem/README.md). This page maps
the code.

## Pipeline

```
Problem (equation + boundary conditions)        problem.hpp
   │   symbolic coefficients (SymEngine Expr)    expression.hpp
   ├─ analyze(problem[, mesh]) → ProblemProperties (each with Evidence)
   │                                             analysis.hpp, evidence.hpp
   ▼
PreparedAssembly(mesh, problem)                 assembly.hpp
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
   │   LinearSolverBackend (LU / CG / GMRES / user)
   ▼                                  stationary: solve_stationary() solver.hpp
evolve(mesh, problem, InitialConditions, opts)  evolution.hpp
   ├─ StepController (default: TimeGrid)        step_control.hpp, time_grid.hpp
   ├─ TimeIntegrator (BE / CN / Newmark / user) time_integration.hpp
   │    IntegratorServices: solve, start_dirichlet, assemble(t)
   └─ interpolate() initial fields              interpolation.hpp
   ▼
Evolution {frames, reports, timings, metadata.decisions}
   → write_report()                             report.hpp, timing.hpp, decision.hpp
L2 / H1 errors against exact solutions          error_norms.hpp

Adaptivity (stationary): solve → estimate → mark → refine → transfer
   estimate_error → ErrorEstimate {indicators, global}   estimation.hpp
   mark_bulk(indicators, θ) → element indices            estimation.hpp
   discretization::refine_mesh(result, marked, options)  triangulate.hpp
   transfer_field(old mesh, values, new mesh)            transfer.hpp
```

One adaptive iteration (see `tests/fem/adaptive_loop.cpp`):

```cpp
auto u = solve_stationary(result.mesh, problem).values;
auto estimate = estimate_error(result.mesh, problem, u);
auto marked = mark_bulk(estimate.indicators, 0.6);
auto finer = discretization::refine_mesh(result, marked, result.options);
auto start = transfer_field(result.mesh, u, finer.mesh,
                            {.max_outside_distance = 0.02}); // curved domain
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
| `system.hpp` | `SparseMatrix`, `Vector`, `DirichletValue`, `DirichletDerivative`, `AssembledSystem`, `time_order`, `numerically_symmetric` | Eigen |
| `assembly.hpp` | `AssemblyOptions`, `PreparedAssembly`, validation, parallel evaluation | all above, system |
| `detail/assembly_elements.hpp` | private: interior coefficients, quadrature points, element blocks | p1_element, weak_form |
| `detail/assembly_edges.hpp` | private: edge classification, named edge evaluators, edge blocks | boundary_classification |
| `detail/assembly_scatter.hpp` | private: deterministic serial scatter | system |
| `linear_solver.hpp` | Dirichlet lifting, `SolverOptions`, `LinearSolverBackend` (+ built-in LU/CG/GMRES, preconditioners), `PreparedLinearSolver`, reuse, `solver_decisions` | system, timing, decision |
| `solver.hpp` | `solve_stationary` (includes linear_solver.hpp) | assembly, linear_solver |
| `interpolation.hpp` | nodal interpolation of a field (rejects normals) | system, expression |
| `time_grid.hpp` | time stamps and step sizes | — |
| `time_integration.hpp` | `TimeIntegrator` interface, `IntegratorServices`, schemes | system |
| `step_control.hpp` | `StepController` interface, `UniformStepController` (TimeGrid) | time_grid, time_integration |
| `evolution.hpp` | `evolve()` driver, frames, observer, accounting, output/step checks, decisions | all above |
| `timing.hpp` | timing structs, `timing::Accumulate` | — |
| `analysis.hpp` | `analyze()` → `ProblemProperties` / `SystemProperties` with evidence | boundary_classification, weak_form, system, detail/assembly_* |
| `report.hpp` | text summaries of solves, evolutions, decisions and analyses | evolution, analysis |
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
  checks: `weak_time_dependence` (caching), `divisor_keeps_strict_sign` on
  `edge_divisor_samples` (boundary divisors, Evidence::Sampled),
  `time_order` (integrator `check()`), `numerically_symmetric` (the CG
  backend). Do not re-implement one of them elsewhere.
- **Decisions are recorded.** Every choice that shapes a solve appears in
  `SolverReport::decisions` or `EvolutionMetadata::decisions`. Today all are
  user-selected; nothing selects solvers automatically yet.
- **Estimators report, they do not decide.** `estimate_error` returns
  indicators and a global value; marking and refinement are separate calls
  the user (or a future automatic loop, recording a `Decision`) makes.
- **Normals only on boundaries.** `nx`, `ny` are bound in boundary
  expressions; interior coefficients, interpolated and initial fields and
  exact solutions are checked with `require_symbols(..., field_symbols, ...)`.

See [EXTENDING.md](EXTENDING.md) for adding terms, boundary forms, schemes,
linear solver backends and element types.

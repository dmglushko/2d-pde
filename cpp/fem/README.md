# `cpp/fem` — scalar P1 finite elements

Header-only, optional module (CMake `PDE2D_BUILD_FEM=ON`; needs Eigen3 and
SymEngine). User-level documentation with equations, sign conventions and
examples is in [docs/fem/README.md](../../docs/fem/README.md). This page maps
the code; [examples/fem/minimal.cpp](../../examples/fem/minimal.cpp) is the
shortest complete program (geometry, mesh, problem, solve and evolve).

## Pipeline

```
Problem (equation + boundary conditions)        problem.hpp
   │   symbolic coefficients (SymEngine Expr)    expression.hpp
   ▼
PreparedAssembly(mesh, problem, AssemblyOptions) assembly.hpp
   ├─ bind_boundary(mesh, problem) → regions, edge frames
   │                                             boundary_assignment.hpp
   ├─ weak_form_coefficients(equation)          weak_form.hpp
   ├─ classify_edge_condition → EdgeForm        boundary_classification.hpp
   │    (DirichletEdge | NaturalEdge | ObliqueEdge; sampled divisor signs)
   │    one unit per form: prepare/evaluate/scatter  detail/edge_forms/
   ├─ P1 geometry + local kernels               p1_element.hpp, quadrature.hpp
   └─ interior elements, boundary edges, scatter detail/assembly_*.hpp
   ▼
AssembledSystem {K, M1, M2, F, Dirichlet}       system.hpp (element independent)
   ▼
constrain() + PreparedLinearSolver              constraints.hpp, prepared_linear_solver.hpp
   │   LinearSolverBackend (LU / CG / GMRES / user), optional initial guess
   │   contracts: linear_solver_types.hpp; Eigen backends: detail/eigen_backends.hpp
   │   (linear_solver.hpp includes all of them)
   ▼
stationary: PreparedStationary(mesh, problem, assembly, solver).solve(t[, guess])
            solve_stationary() = one-shot wrapper                solver.hpp
evolve(mesh, problem, InitialConditions, TimeOptions)            evolution.hpp
   ├─ TimeGrid (stamps, last-step snapping)     time_grid.hpp
   ├─ TimeIntegrator (BE / CN / Newmark / user) time_integrator.hpp,
   │    one unit per built-in TimeMethod         detail/time_schemes/
   │    IntegratorServices: solve, start_dirichlet, assemble(t)
   └─ initial fields: Expr or nodal Vector      interpolation.hpp (Field)
   ▼
StationarySolution {values, report}             solver.hpp
Evolution {frames, reports, timings, metadata}  evolution.hpp
   → write_report()                             report.hpp, timing.hpp
L2 / H1 errors against exact solutions          error_norms.hpp
```

`fem.hpp` includes everything. Parallel loops use `pde2d::parallel_for`
from `cpp/execution.hpp`; meshes are `discretization::TriangleMesh`.

## Headers

| Header | Responsibility | Depends on |
| --- | --- | --- |
| `expression.hpp` | `Expr`, symbols `x y t nx ny`, `require_symbols` by context (`field_symbols`, `boundary_symbols`), compiled `Evaluator` | SymEngine |
| `problem.hpp` | `Equation`, `BoundaryCondition` + factories, `Problem` | expression |
| `weak_form.hpp` | weak-form term names (`weak::Term`, `WeakCoefficients<T>`, `WeakFlags`), coefficients by term, `weak_time_dependence` | problem |
| `quadrature.hpp` | triangle and edge rules by exactness degree | — |
| `p1_element.hpp` | P1 gradients, edge frames/normals, local kernels | weak_form |
| `boundary_classification.hpp` | general BC → `EdgeForm` (`DirichletEdge`, `NaturalEdge` or `ObliqueEdge`), pure symbolic; `derivative_vectors`; form and divisor names from each form's unit | detail/edge_forms, p1_element, problem |
| `detail/edge_forms/interface.hpp` | `restrict_to_edge`, `edge_divisor_samples`, `divisor_keeps_strict_sign` (public); private: the unit interface `Unit<Form>` (`EdgeFormUnit` concept), `EdgeSite`, `EdgeResult`, `ScatterTarget` | p1_element, quadrature, system |
| `detail/edge_forms/{dirichlet,natural,oblique}.hpp` | one unit per form: the public form struct and its name, divisors, prepared data, per-time evaluation and scatter. `oblique.hpp` holds the oblique derivation | interface (oblique: natural) |
| `detail/edge_forms/forms.hpp` | `EdgeForm` variant; `static_assert` that every alternative has its unit; `PreparedEdgeForm` | the units |
| `boundary_assignment.hpp` | `bind_boundary`: validated region → condition binding (owning snapshot of the equation and one condition per assignment; edges refer to them by index) and the frame (length, outward normal) of every boundary edge | p1_element, problem |
| `system.hpp` | `SparseMatrix`, `Vector`, `DirichletValue`, `DirichletDerivative`, `AssembledSystem`, `time_order`, `numerically_symmetric` | Eigen |
| `assembly.hpp` | `AssemblyOptions`, `PreparedAssembly`, validation, parallel evaluation | all above, system |
| `detail/assembly_elements.hpp` | private: interior coefficients, quadrature points, element blocks | p1_element, weak_form |
| `detail/assembly_edges.hpp` | private: classification and preparation of the bound edges, per-time evaluation, all through the form units | boundary_assignment, boundary_classification, edge_forms |
| `detail/assembly_scatter.hpp` | private: deterministic serial scatter (edges through their units) | system, edge_forms |
| `linear_solver_types.hpp` | Contracts only: `LinearSolver`, `Preconditioner`, `SolverValidation`, `SolverOptions` (defaulted `operator==`), `InitialGuess`, `SolverReport`, `LinearSolution`, `BackendSolution`, `LinearSolverBackend` (`prepare`, `solve`, `solve_with_guess`), `LinearSolverBackendFactory`, `make_linear_solver_backend`, report names (`linear_solver_name`, `linear_solver_method_name`, `preconditioner_name`, `solver_validation_name`, `initial_guess_name`) | system, timing |
| `constraints.hpp` | `constrain(A, b, dirichlet)`: Dirichlet lifting; the shared linear-system check | system |
| `prepared_linear_solver.hpp` | `PreparedLinearSolver` (validation, rank check, backend setup, true-residual check, reuse via `matches()`), `solve_linear` | linear_solver_types, constraints, detail/eigen_backends |
| `detail/eigen_backends.hpp` | private: built-in Eigen backends (SparseLU, CG, GMRES and their preconditioners), `with_builtin` (the only dispatch on `LinearSolver`), option validation, backend creation | linear_solver_types |
| `linear_solver.hpp` | Umbrella: includes `linear_solver_types.hpp`, `constraints.hpp` and `prepared_linear_solver.hpp` (include it for `solve_linear`) | the three |
| `solver.hpp` | `PreparedStationary` (prepared assembly + reused linear solver, `StationaryStatistics`, optional initial guess), `StationarySolution`, `solve_stationary` wrappers | assembly, constraints, prepared_linear_solver |
| `interpolation.hpp` | nodal interpolation of a field (rejects normals); `Field` (Expr or nodal Vector), `validate_field`, `nodal_values` | system, expression |
| `time_grid.hpp` | time stamps and step sizes | — |
| `time_integrator.hpp` | `TimeMethod`, `TimeIntegrator` interface, `IntegratorState`, `IntegratorServices`, `impose_compatible`, `TimeIntegratorFactory` | system |
| `detail/time_schemes/{backward_euler,crank_nicolson,newmark}.hpp` | one unit per built-in `TimeMethod`: `method`, `name`, `order`, `Integrator` (the public class, e.g. `NewmarkIntegrator`), `make()`; the classes take `method()`/`name()`/`order()` from their unit (`BuiltinScheme`) | time_schemes/interface |
| `detail/time_schemes/{interface,schemes}.hpp` | private: the `TimeSchemeUnit` concept, `BuiltinScheme`, `FirstOrder`; the unit list `Units` with a `static_assert` that every `TimeMethod` before `Custom` has its unit in order, and `with_time_method` (the only dispatch on `TimeMethod`) | time_integrator |
| `time_integration.hpp` | the built-in schemes and `time_method_name`, `time_method_order`, `make_time_integrator`, all read from the units | time_integrator, detail/time_schemes |
| `evolution.hpp` | `evolve()` driver, `TimeOptions` (method or user `integrator` factory, interval, step, assembly/solver options, output), `InitialConditions`, frames, observer, accounting, integrator output checks, `EvolutionMetadata` | all above |
| `timing.hpp` | timing structs, `timing::Accumulate` | — |
| `report.hpp` | text summaries of solves and evolutions | evolution, solver |
| `error_norms.hpp` | prepared L2/H1 error norms (rejects normals) | system, quadrature |

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
- **Domains may have holes.** Nothing in the FEM module assumes one
  boundary loop: regions are bound per edge wherever they lie, and each
  edge's outward normal comes from its triangle, so on a hole it points into
  the hole (`nx`, `ny` in conditions). `tests/fem/annulus.cpp` solves a
  manufactured problem with Dirichlet data outside and a Robin condition on
  the hole at second order in L2.
- **One unit per built-in time scheme.** `TimeMethod` values before
  `Custom` each have a unit in `detail/time_schemes/` (name, order,
  factory); `time_method_name`, `make_time_integrator` and the integrator
  classes read them through `with_time_method`, and `schemes.hpp` fails to
  compile for a value without its unit.
- **One unit per boundary form.** Generic code (classification names,
  assembly, scatter) reaches a form only through
  `edge_form_detail::Unit<Form>`; `forms.hpp` fails to compile for an
  `EdgeForm` alternative without its unit.
- **Bindings own their data.** `BoundaryBinding` snapshots the equation and
  conditions, so it outlives and ignores later changes to the `Problem`.
- **Normals only on boundaries.** `nx`, `ny` are bound in boundary
  expressions; interior coefficients, interpolated and initial fields and
  exact solutions are checked with `require_symbols(..., field_symbols, ...)`.
- **The `header_standalone` test** parses every public header in its own
  translation unit (syntax only, at build time).

See [EXTENDING.md](EXTENDING.md) for adding terms, boundary forms, schemes,
linear solver backends and element types.

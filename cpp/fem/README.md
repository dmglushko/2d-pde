# `cpp/fem` — scalar P1 finite elements

Header-only, optional module (CMake `PDE2D_BUILD_FEM=ON`; needs Eigen3 and
SymEngine). User-level documentation with equations, sign conventions and
examples is in [docs/fem/README.md](../../docs/fem/README.md). This page maps
the code.

## Pipeline

```
Problem (equation + boundary conditions)        problem.hpp
   │   symbolic coefficients (SymEngine Expr)    expression.hpp
   ▼
PreparedAssembly(mesh, problem)                 assembly.hpp
   ├─ weak_form_coefficients(equation)          weak_form.hpp
   ├─ classify_edge_condition(eq, bc, edge)     boundary_classification.hpp
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
   ├─ TimeGrid (stamps, last-step snapping)     time_grid.hpp
   ├─ TimeIntegrator (BE / CN / Newmark / user) time_integration.hpp
   └─ interpolate() initial fields              interpolation.hpp
   ▼
Evolution {frames, reports, timings}  → write_report()   report.hpp, timing.hpp
L2 / H1 errors against exact solutions          error_norms.hpp
```

`fem.hpp` includes everything. Parallel loops use `pde2d::parallel_for`
from `cpp/execution.hpp`; meshes are `discretization::TriangleMesh`.

## Headers

| Header | Responsibility | Depends on |
| --- | --- | --- |
| `expression.hpp` | `Expr`, symbols `x y t nx ny`, `require_symbols` by context, compiled `Evaluator` | SymEngine |
| `problem.hpp` | `Equation`, `BoundaryCondition` + factories, `Problem` | expression |
| `weak_form.hpp` | named weak-form coefficients (`weak::Term`) | problem |
| `quadrature.hpp` | triangle and edge rules by exactness degree | — |
| `p1_element.hpp` | P1 gradients, edge frames/normals, local kernels | weak_form |
| `boundary_classification.hpp` | general BC → Dirichlet or natural form, pure symbolic | p1_element, problem |
| `system.hpp` | `SparseMatrix`, `Vector`, `DirichletValue`, `AssembledSystem` | Eigen |
| `assembly.hpp` | `AssemblyOptions`, `PreparedAssembly`, validation, parallel evaluation | all above, system |
| `detail/assembly_elements.hpp` | private: interior coefficients, quadrature points, element blocks | p1_element, weak_form |
| `detail/assembly_edges.hpp` | private: edge classification, named edge evaluators, edge blocks | boundary_classification |
| `detail/assembly_scatter.hpp` | private: deterministic serial scatter | system |
| `linear_solver.hpp` | Dirichlet lifting, `SolverOptions`, `LinearSolverBackend` (+ built-in LU/CG/GMRES, preconditioners), `PreparedLinearSolver`, reuse | system, timing |
| `solver.hpp` | `solve_stationary` (includes linear_solver.hpp) | assembly, linear_solver |
| `interpolation.hpp` | nodal interpolation of a field (rejects normals) | system, expression |
| `time_grid.hpp` | time stamps and step sizes | — |
| `time_integration.hpp` | `TimeIntegrator` interface and schemes | system |
| `evolution.hpp` | `evolve()` driver, frames, observer, accounting | all above |
| `timing.hpp` | timing structs, `timing::Accumulate` | — |
| `report.hpp` | text summaries of solves and evolutions | evolution |
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
- **Normals only on boundaries.** `nx`, `ny` are bound in boundary
  expressions; interior coefficients, interpolated and initial fields and
  exact solutions are checked with `require_symbols(..., field_symbols, ...)`.

See [EXTENDING.md](EXTENDING.md) for adding terms, boundary forms, schemes,
linear solver backends and element types.

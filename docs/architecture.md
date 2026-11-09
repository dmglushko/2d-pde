# Architecture

Current structure of the project. Module-level detail lives next to the code:
[cpp/discretization/README.md](../cpp/discretization/README.md) and
[cpp/fem/README.md](../cpp/fem/README.md).

## Layers

```
            ┌────────────────────────────────────────────────┐
 tools/     │ mesh viewer (JS) · plot_mesh.py · render_*.mjs │  read files only
            └───────────────▲────────────────────────────────┘
                            │ PDE2D_MESH files, JSON results
 examples/  showcases, long waves, learning programs, benchmarks/
                            │ public headers only
 cpp/fem/   ────────────────┤  scalar P1 FEM (Eigen + SymEngine)   optional
                            │ TriangleMesh, validate_mesh_geometry
 cpp/discretization/ ───────┤  geometry, curves, meshing, mesh IO   core
                            │
 cpp/execution.hpp  ────────┘  ExecutionOptions, parallel_for (OpenMP opt-in)
```

Dependencies point downward only. `cpp/discretization` must never include
`cpp/fem`, and geometry-only users need neither Eigen nor SymEngine.
`svg_geometry.hpp` is the only discretization header that needs libxml2.

Namespaces: everything lives under `pde2d`. Geometry and meshing are in
`pde2d::discretization`, the FEM module in `pde2d::fem`, and execution
policy in `pde2d` itself. Private helpers live in `*_detail` namespaces and
are not used by tests or examples.

## Data flow

```
CurveGeometry ──discretize_boundary──▶ DiscretizedBoundary ──generate_mesh──▶ MeshResult
   (analytical curves, regions;          (polygon with holes                  .mesh  : TriangleMesh
    outer loop + holes)                   + curve context)
                                                                               .report: guarantees met?
MeshResult ──export_mesh/import_mesh_result──▶ .pde2d file   (docs/formats/pde2d-mesh.md)
MeshResult ──refine_mesh──▶ MeshResult        (resumes from the stored boundary context)
MeshResult + marked elements ──refine_mesh──▶ MeshResult   (report.marked: what was bisected)
RefinementOptions: max_edge_length and/or size_field h(x,y)  (size field is request-time, not in files)
TriangleMesh ──PointLocator──▶ element + barycentric        (outside rule: max_outside_distance)

TriangleMesh + Problem ──bind_boundary──▶ BoundaryBinding (regions, edge frames)
     │                                      shared by assembly and analysis
     ├──analyze──▶ ProblemProperties   (each property: value + Evidence + reason)
     └──PreparedAssembly──▶ AssembledSystem
options ──plan_from_user──▶ SolvePlan {assembly, solver, time, decisions}
SolvePlan ──PreparedStationary.solve(t[, guess]) / solve_stationary──▶ StationarySolution
SolvePlan + InitialConditions (Expr or nodal Vector) ──evolve──▶ Evolution
results carry RunRecord {plan, execution, workload, work, evidence}

Adaptive loop (stationary):
  solve_stationary ──▶ values ──estimate_error──▶ ErrorEstimate {indicators, global}
     ──mark_bulk──▶ marked elements ──refine_mesh(result, marked)──▶ MeshResult
     ──transfer_field(old mesh, values, new mesh)──▶ values on the new mesh
     ──PreparedStationary(new mesh, plan).solve(t, guess = transferred)──▶ values
Restart (transient): last frame ──transfer_field──▶ InitialConditions{Vector} ──evolve
Time steps: StepController.next ──▶ trial (TimeIntegrator.trial) ──judge──▶
     accept (commit, frame, observer) | reject (discard, next() proposes a retry)
```

Domains are multiply connected: an outer loop plus holes, carried
explicitly through every layer (`CurveGeometry` loops, `Boundary::holes`,
the boundary context's per-loop snapshot and `boundary_loops(mesh)`).
Ear clipping bridges holes into one ring and every loop's edges are CDT
constraints; validation accepts one connected domain whose boundary is
several closed loops. Files with holes use format version 5.

Inside meshing, refinement is one generic loop (`refine_with`) with two
seams in `detail/refinement_policy.hpp`: a size criterion (constant target,
size field or marked edges, all as one `SizeCriterion`) and a boundary
policy per `BoundaryTreatment`. Point location (`point_location.hpp`) is a
discretization facility; field transfer and error estimation are FEM
(`transfer.hpp`, `estimation.hpp`).

`AssembledSystem` and the sparse aliases live in the element-independent
`cpp/fem/system.hpp`; linear solvers (`linear_solver.hpp`) and time
integrators (`time_integration.hpp`) depend only on it. Both are strategies:
`SolverOptions::backend` plugs in a `LinearSolverBackend` and
`TimeOptions::integrator` a `TimeIntegrator`, with the enums as the built-in
convenience selectors. The step sequence is a third strategy:
`TimeOptions::step_controller` plugs in a `StepController`
(`step_control.hpp`; default `UniformStepController`, the fixed `TimeGrid`).
Steps are transactional: integrators produce a trial without committing
their state, the controller accepts or rejects it, and only accepted steps
are committed and published.

Preprocessing and adaptivity (see [vision.md](vision.md)) build on two
seams. `analysis.hpp` reports properties of a problem (time order,
symmetry, ellipticity, reaction sign, boundary forms, nullspace risk), each
with an `Evidence` level (Proven, Sampled, Unknown); it calls the same
functions the solve uses for the same judgement, so the two cannot
disagree. `decision.hpp` records choices (`Decision`: subject, choice,
basis, evidence, source) in `SolverReport::decisions` and
`EvolutionMetadata::decisions`, printed by `write_report`. The settings and
their decisions travel together as a `SolvePlan` (`plan.hpp`), the one
output type of a future automatic chooser and the one place to override
it. Today every recorded decision is user-selected (`plan_from_user`);
there is no automatic selection yet. Top-level results carry a `RunRecord`
(`record.hpp`) with the plan, execution policy, workload, work counters and
an evidence summary; `MeshResult` already carries its `MeshOptions` and
`MeshReport`. Analysis and assembly both start from `bind_boundary`
(`boundary_assignment.hpp`), so analysis no longer reaches into assembly
internals. Private implementation headers of both modules live in
`detail/` subdirectories (`*_detail` namespaces).

Each boundary edge form (Dirichlet, natural, oblique) is one unit in
`cpp/fem/detail/edge_forms/`: its name, divisors, preparation, per-time
evaluation, scatter and analysis judgement. Classification picks the form;
assembly, scatter and analysis call the unit through one interface, and an
`EdgeForm` alternative without its unit does not compile.

Boundary region tags connect the two halves: geometry regions become
`TriangleMesh::boundary_tags`, and `Problem::assign(condition, {"region"})`
attaches boundary conditions to them. Every tagged region needs exactly one
condition.

## Build configurations

| Configuration | Command | Needs |
| --- | --- | --- |
| Geometry only (default) | `tools/test.sh` | C++23, Boost headers; libxml2 and Node optional |
| FEM | `tools/test_fem.sh` (add `-DPDE2D_FETCH_SYMENGINE=ON` or `-DSymEngine_DIR=...`) | + Eigen3, SymEngine |
| OpenMP kernels | add `-DPDE2D_ENABLE_OPENMP=ON` | OpenMP |
| Sanitizers | add `-DPDE2D_SANITIZERS=address,undefined` | |
| Independent references | add `-DPDE2D_ENABLE_REFERENCES=ON` | `.venv` with tools/requirements-reference.txt |
| Benchmarks | add `-DPDE2D_BUILD_BENCHMARKS=ON` (never CTest tests) | |

## Tests

`tests/` mirrors the modules (`tests/core`, `tests/discretization`, `tests/fem`,
`tests/tools`); shared helpers are in `tests/support`. Each C++ test is a
standalone executable built on `tests/support/check.hpp` and registered
explicitly in its module directory's `CMakeLists.txt` (e.g.
`tests/fem/CMakeLists.txt`) with `pde2d_add_cpp_test`, defined in
`tests/CMakeLists.txt`, which states its dependencies (FEM, libxml2, fixture
files). Tests use public APIs; the only exception is `tests/<module>/internal/`,
which holds white-box tests of private data structures (currently the CDT
topology and broad phase). Oracles are exact or
manufactured solutions, independent reference implementations
(`tools/reference/`, scikit-fem) or invariants; tests must not re-implement
the algorithm under test.

## Generated and historical files

Generated by programs in this repository, regenerate rather than edit:
`docs/showcase/*` (meshing showcase), `docs/fem/showcase.html`,
`docs/fem/long-waves*`, `docs/performance/*.json|html`,
`examples/mesh_viewer/example_mesh.js`. `docs/archive/` holds dated reports that describe past snapshots of
the code. They are evidence, not documentation of the current API.

## Known limits and next steps

The intended direction is in [vision.md](vision.md).


- CDT refinement has no minimum-angle guarantee. Marked refinement only refines (no coarsening or node
  movement), and the only built-in estimator is gradient recovery.
- FEM is scalar, linear and P1. Extension points are in
  [cpp/fem/EXTENDING.md](../cpp/fem/EXTENDING.md) and
  [cpp/discretization/EXTENDING.md](../cpp/discretization/EXTENDING.md).
- Naming: the geometry/curve API uses camelCase member and curve functions
  (`setCenter`, `curveLength`). Meshing free functions and the whole FEM
  module use snake_case. New code follows snake_case. Renaming the public
  geometry API is deliberately deferred: it is a breaking change touching
  most examples and docs.
- Everything is header-only. If compile times grow, the SymEngine-heavy
  preparation (`assembly.hpp`) is the first candidate for a compiled library.

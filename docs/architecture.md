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
 cpp/provenance.hpp           Fingerprint, StrategyDescriptor, library_version
 cpp/json.hpp                 JsonValue, json_number, parse_json (records of both modules)
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
MeshResult.record: MeshRunRecord {operation, fingerprints, options, version,
     work (steps, flips, rebuilds), quality (angles, lengths, target ratio), time}
     ──write_json (mesh_record_json.hpp)──▶ "pde2d.mesh_run_record"
MeshResult ──export_mesh/import_mesh_result──▶ .pde2d file   (docs/formats/pde2d-mesh.md)
MeshResult ──refine_mesh──▶ MeshResult        (resumes from the stored boundary context)
MeshResult + marked elements ──refine_mesh──▶ MeshResult   (report.marked: what was bisected)
RefinementOptions: max_edge_length and/or size_field h(x,y), or a user
    RefinementCriterion (size_criterion.hpp)   (request-time, not in files)
MeshOptions::algorithm ──with_interior_algorithm──▶ unit {capabilities, generate, refine}
TriangleMesh ──PointLocator──▶ element + barycentric        (outside rule: max_outside_distance)

MeshResult ──(report.size_satisfied, or accept_unsatisfied(result))──▶ FEM entry points
TriangleMesh + Problem ──bind_boundary──▶ BoundaryBinding (owned snapshot: regions, conditions, edge frames)
     │                                      shared by assembly and analysis
     ├──analyze──▶ ProblemProperties   (each property: value + Evidence + reason)
     └──PreparedAssembly──▶ AssembledSystem
options ──plan_from_user──▶ SolvePlan {assembly, solver, time, decisions, mesh, analysis, planner}
SolvePlanner / MeshPlanner ──run_planner──▶ checked SolvePlan / MeshPlan   (UserPlanner = identity)
problem [+ mesh] ──make_analysis_record──▶ AnalysisRecord (properties + fingerprints + options)
MeshOptions ──mesh_plan_from_user──▶ MeshPlan {options, decisions}   (SolvePlan::mesh)
SolvePlan ──PreparedStationary.solve(t[, guess]) / solve_stationary──▶ StationarySolution
SolvePlan + InitialConditions (Expr or nodal Vector) ──evolve──▶ Evolution
results carry RunRecord {plan, inputs (fingerprints, mesh provenance with the
                         MeshRunRecord), stationary solve (guess fingerprint,
                         iterations, residual), strategies, versions, execution,
                         workload, work, evidence, timings}
RunRecord ──write_json──▶ versioned JSON ──read_run_record_json / diff_records

Adaptive loop (stationary), by hand or as adapt(result, problem, plan, AdaptOptions):
  PreparedStationary(result, plan).solve ──▶ values ──estimate_error──▶ ErrorEstimate
     ──MarkingStrategy (BulkMarking = mark_bulk)──▶ marked elements
     ──refine_mesh(result, marked)──▶ MeshResult
     ──transfer_field(old mesh, values, new mesh)──▶ values on the new mesh
     ──PreparedStationary(new result, plan).solve(t, guess = transferred)──▶ values
  optional AdaptOptions::solve_planner (before each solve), mesh_planner (after marking);
  plan.analysis refreshed and plan.mesh updated for each refined mesh
  adapt ──▶ AdaptiveHistory {iterations: RunRecord, transfer, estimate summary,
                             marking, refinement report, Decisions; stop; final state}
        (and AdaptiveSettings: tolerance, budget, warm start, transfer, ...)
        ──write_json (adaptivity_json.hpp)──▶ versioned JSON
Restart (transient): last frame ──transfer_field──▶ InitialConditions{Vector} ──evolve
Adaptive loop (transient), adapt_evolution(result, problem, initial, plan, options):
  windows of plan.time: evolve ──▶ estimate (last frame) ──▶ mark ──▶ refine
     ──transfer_field + Dirichlet data at the restart time──▶ nodal restart ──▶ evolve
  ──▶ EvolutionAdaptiveHistory {settings, windows: RunRecord, restart, estimate, ...}
Reference problems (fem/reference/problems.hpp: geometry, problem, exact solution)
  ──compare_plans(reference, mesh plan, {plans}) / compare_mesh_plans──▶
  PlanComparison {per plan: RunRecord, L2/H1 errors, work, timings} ──▶ JSON
Time steps: StepController.next ──▶ trial (TimeIntegrator.trial) ──judge──▶
     accept (commit, frame, observer) | reject (discard, next() proposes a retry)
```

Domains are multiply connected: an outer loop plus holes, carried
explicitly through every layer (`CurveGeometry` loops, `Boundary::holes`,
the boundary context's per-loop snapshot and `boundary_loops(mesh)`).
Ear clipping bridges holes into one ring and every loop's edges are CDT
constraints; validation accepts one connected domain whose boundary is
several closed loops. Files with holes use format version 5.

Inside meshing, each `InteriorAlgorithm` is one unit with declared
capabilities (`detail/interior_algorithms.hpp`, the only dispatch on the
algorithm; `interior_algorithm_capabilities` exposes them). Refinement is
one generic loop (`refine_with`, `detail/refinement_loop.hpp`) with two
seams: a size criterion (constant target, size field, a user
`RefinementCriterion` or marked edges, all as one `SizeCriterion`; public in
`size_criterion.hpp`) and a boundary policy per `BoundaryTreatment`
(`detail/refinement_policy.hpp`). Point location (`point_location.hpp`) is a
discretization facility; field transfer and error estimation are FEM
(`transfer.hpp`, `estimation.hpp`).

`AssembledSystem` and the sparse aliases live in the element-independent
`cpp/fem/system.hpp`; linear solvers and time integrators
(`time_integrator.hpp`, `time_integration.hpp`) depend only on it. Each
built-in `TimeMethod` is one unit in `detail/time_schemes/` (name, order,
factory, descriptor) behind one dispatch (`with_time_method`), like the
boundary edge forms and the interior algorithms. The linear solver layer is split
by role: contracts (`linear_solver_types.hpp`, which plans and records
include), Dirichlet lifting (`constraints.hpp`), the shared driver
(`prepared_linear_solver.hpp`) and the private Eigen backends
(`detail/eigen_backends.hpp`); `linear_solver.hpp` includes them all. Both are strategies:
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
disagree. `decision.hpp` records choices (`Decision`: typed
`DecisionSubject`, choice, basis, evidence, source) in
`SolverReport::decisions` and the run record, printed by `write_report`.
The settings and their decisions travel together as a `SolvePlan`
(`plan.hpp`), the one output type of a planner and the one place to
override it: setters change an option and its decision together, and every
solve validates at entry that each decision still matches its option
("Stale decision"). Each subject has one row in `decision_subject_table`
(stable name, owner: Solve, Time, Mesh or Adapt), and each plan kind one
binding table (`solve_plan_bindings`, `mesh_plan_bindings`) saying how to
read every owned subject's choice from the options; user decisions,
setters, validation and the owner check all derive from it. `MeshPlan`
(`fem/mesh_plan.hpp`) does the same for `MeshOptions`; it lives in `fem`
because decisions do, so discretization stays independent. Planners plug in
at `SolvePlanner` and `MeshPlanner` (`planner.hpp`; `UserPlanner` is the
identity), checked by `run_planner` and run by `adapt` between iterations.
The analysis a plan relies on (`AnalysisRecord`, made only by
`make_analysis_record`) carries the problem and mesh fingerprints and its
options; solves refuse evidence for other inputs ("Stale analysis") and
`adapt` refreshes it on refined meshes. Today every recorded decision is
user-selected (`plan_from_user`, `mesh_plan_from_user`); there is no
automatic selection yet. Plans and records include only contract headers
(`assembly_options.hpp`, `linear_solver_types.hpp`, `analysis_types.hpp`,
`mesh_plan.hpp`), never assembly, symbolic or edge-form headers; the
`header_standalone` test compiles every public header on its own. Top-level results carry a `RunRecord`
(`record.hpp`) with the plan (and its mesh plan, analysis and planner), input
fingerprints (mesh, problem, initial fields), mesh provenance when the
solve got a `MeshResult`, the strategies that ran and whether they are
replayable, library/dependency versions, execution policy, workload, work
counters and an evidence summary. `record_json.hpp` writes it as
versioned JSON and diffs two records. Fingerprints and strategy
descriptors come from `cpp/provenance.hpp`, shared by both modules
(`discretization::mesh_fingerprint`, `RefinementOptions::size_field_descriptor`);
the library version comes from a header CMake generates at configure time
(`cmake/pde2d_version.hpp.in`). Analysis and assembly both start from `bind_boundary`
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

Meshing results cross into FEM with their report: entry points that take a
`MeshResult` refuse one whose `report.size_satisfied` is false unless the
caller wraps it in `accept_unsatisfied(result)` (`fem/mesh_acceptance.hpp`),
and the meshing functions returning `MeshResult` are `[[nodiscard]]`.

The adaptive driver `adapt` (`fem/adaptivity.hpp`) is an optional
composition of the public solve, estimate, mark, refine and transfer
components; it adds no numerics (the hand-composed loop is its bit-for-bit
test oracle) and records each iteration and its marking and refinement
decisions in an `AdaptiveHistory`, together with the settings it ran with.
`adapt_evolution` does the same for evolutions, window by window, with a
nodal restart on each refined mesh.

Meshing keeps its own record: `MeshResult::record` (`MeshRunRecord`,
declared in `discretization/mesh_types.hpp`, filled by the meshing entry
points, serialized by `discretization/mesh_record_json.hpp`) holds the
operation, fingerprints, options, version, work counters, quality
statistics and time of the call that produced the mesh. It lives in
discretization so geometry-only users have it; the FEM run record embeds it
in its mesh provenance. Both serialize with the core JSON document
`cpp/json.hpp`.

Reference problems (`fem/reference/`) make "better is measured" concrete:
a small library of problems with exact solutions and `compare_plans` /
`compare_mesh_plans`, which run plans with the public solve calls and
report each one's errors next to its run record and work. They live in a
public subdirectory of the FEM module because they need the solver; nothing
in the solve path depends on them.

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
  movement), the only built-in estimator is gradient recovery and the only
  built-in marking strategy is Dörfler (`BulkMarking`). The transient
  driver refines between fixed time windows only (no re-solve of a window,
  no coarsening, uniform steps).
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

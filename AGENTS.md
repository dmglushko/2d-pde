# Agent guide

C++23 header-only library for 2D geometry, triangle meshing and scalar P1
finite elements, plus JavaScript/Python tools that read its files. Read
[docs/architecture.md](docs/architecture.md) first for the layer diagram and
data flow, and [docs/vision.md](docs/vision.md) for where the project is
heading and which directions to keep open.

## Where things are

Start with [docs/concepts.md](docs/concepts.md): the minimal path and the
optional layers. Module READMEs hold the header maps (checked by the
`docs_header_map` test).

| Path | Contents |
| --- | --- |
| `cpp/execution.hpp` | `ExecutionOptions`, `parallel_for` (serial/OpenMP) |
| `cpp/json.hpp`, `cpp/provenance.hpp` | JSON document of all records; fingerprints, strategy descriptors, library version |
| `cpp/discretization/` | geometry, curves, meshing, refinement, mesh records and IO: [README](cpp/discretization/README.md), [EXTENDING](cpp/discretization/EXTENDING.md) |
| `cpp/fem/` | problems, assembly, solvers, time integration, plans, records, adaptivity: [README](cpp/fem/README.md), [EXTENDING](cpp/fem/EXTENDING.md) |
| `cpp/fem/reference/` | reference problems with exact solutions and plan/planner comparisons |
| `tests/` | one directory per module, `headers/` (standalone headers), `tools/`, `support/` (shared helpers) |
| `examples/` | runnable programs (`fem/minimal.cpp` is the minimal path); built as CTest smoke tests |
| `benchmarks/` | opt-in timing programs; may use internals; never CTest tests |
| `tools/` | mesh viewer, plotting, independent references, `format.sh`, `test*.sh` |
| `docs/` | `concepts.md`, `architecture.md`, `vision.md`, user docs, performance; `archive/` = dated reports |

## Build and test

Run builds inside WSL/Linux (the checkout is on ext4; Windows git refuses it).

```sh
tools/test.sh                         # geometry only: no Eigen/SymEngine needed
tools/test_fem.sh -DPDE2D_FETCH_SYMENGINE=ON          # + FEM (fetches pinned SymEngine)
tools/test_fem.sh -DSymEngine_DIR=<prefix>/lib/cmake/symengine   # installed SymEngine
tools/test_fem.sh -DPDE2D_ENABLE_OPENMP=ON            # parallel kernels
tools/format.sh --check               # clang-format; run tools/format.sh to fix
ctest --test-dir build -R fem_assembly --output-on-failure   # one test
```

Set `PDE2D_BUILD_JOBS`/`PDE2D_TEST_JOBS` for parallelism and `PDE2D_BUILD_DIR`
to choose a build directory. Each C++ test binary prints `PASS/FAIL <case>`.

## Conventions

- Namespaces: `pde2d` (execution), `pde2d::discretization`, `pde2d::fem`.
  `*_detail` namespaces and `cpp/*/detail/` headers are private: tests and
  examples must use public API. Exceptions: benchmarks, and white-box tests in
  `tests/<module>/internal/` (e.g. `tests/discretization/internal/`), which
  must state what they check that the public API cannot observe.
- Extension seams that need no library edit: `SolverOptions::backend`
  (`LinearSolverBackend`), `TimeOptions::integrator` (`TimeIntegrator`),
  `TimeOptions::step_controller`, `RefinementOptions::criterion`
  (`RefinementCriterion`), `AdaptOptions::estimator`/`marker`/`adaptation`
  (`ErrorEstimator`, `MarkingStrategy`, `MeshAdaptation`) and
  `AdaptOptions::planners` (`SolvePlanner`, `MeshPlanner`). The enums
  `LinearSolver`/`TimeMethod` select the built-ins. An `InteriorAlgorithm` is a unit with declared
  capabilities; compatibility errors come from them, never from comparing
  enum values.
- Dependencies point downward: `fem` may include `discretization`, never the
  reverse. Only `svg_geometry.hpp` needs libxml2.
- Style: clang-format (LLVM, 80 columns). FEM and meshing free functions use
  snake_case. The geometry/curve API is camelCase (`setCenter`, `curveLength`)
  for compatibility; don't rename it piecemeal.
- Errors are exceptions with specific messages. Tests match message
  fragments, so keep messages stable or update the tests.
- Numerical code is deterministic: parallel loops write per-item slots and
  reductions/scatter run serially in a fixed order. Keep it that way. Serial
  and OpenMP results must be bit-identical.
- Symbolic (`SymEngine`) work happens only during preparation, never inside
  `parallel_for` bodies.
- One judgement, one function: when analysis and the solve decide the same
  thing (time dependence, divisor sign, time order, matrix symmetry), both
  call one shared public function. Analysis properties and `Decision`s state
  their `Evidence` (Proven / Sampled / Unknown) honestly.
- Conceptual weight: the minimal path of docs/concepts.md (geometry, mesh,
  `Problem`, solve/evolve) constructs no plan, record or decision, and
  `examples/fem/minimal.cpp` (a CTest smoke test) keeps it so. Prefer an
  optional field on an existing record over a new record type; a history
  has one entry type (`AdaptiveStep` serves both adaptive drivers); a
  change to the adaptive step goes into `adaptivity_core.hpp`, never into
  one driver.
- Boundary forms are `EdgeForm` alternatives dispatched with `std::visit`
  and a `static_assert` fallback; add an alternative, not a flag.

## Invariants that are easy to break

- **P1 only.** No element-order switch; higher order is a separate module.
- **Wire IDs are permanent:** `InteriorAlgorithm`, `BoundaryTreatment` and
  curve-kind IDs in `curve_io.hpp`. The `.pde2d` format is specified in
  [docs/formats/pde2d-mesh.md](docs/formats/pde2d-mesh.md) and parsed by
  C++, `tools/mesh_viewer/mesh.js` and `tools/plot_mesh.py`. Change all three
  together.
- **Sign convention:** `Equation` is `a_xx u_xx + a_xy u_xy + a_yy u_yy + ... = source`
  with default `a_xx = a_yy = -1` (that is, −Δu). Natural flux is
  `q_n = -n·A∇u`. See `cpp/fem/problem.hpp` and `weak_form.hpp`.
- **Initial conditions** are not part of `Problem`. They are passed to
  `evolve(mesh, problem, InitialConditions{value, velocity}, options)`;
  each is an `Expr` or a nodal `Vector` (`Field`). Newmark requires
  `velocity`.
- **Settings travel as a `SolvePlan`** (`cpp/fem/plan.hpp`) with their
  decisions; option-based entry points are `plan_from_user` wrappers and
  must stay bit-identical to the plan overloads. Results carry a
  `RunRecord` (`record.hpp`). A setting with a decision is one row in its
  plan's binding table (`solve_plan_bindings`, `mesh_plan_bindings`); never
  add a parallel list of subjects, and change options through `update` or a
  setter. `plan.analysis` comes only from `make_analysis_record` and is
  bound to the problem and mesh it describes.
- **Contract headers stay light:** `plan.hpp`, `record.hpp`, `mesh_plan.hpp`
  and `analysis_types.hpp` must not include `assembly.hpp`, symbolic or
  boundary-classification headers (the `header_standalone` test compiles
  every public header alone).
- **Time steps are transactional:** an integrator with state commits it in
  `accept()`, never in `trial()` (see `NewmarkIntegrator`).
- **Boundaries have loops:** an outer loop plus holes (`BoundaryCurve::loop`,
  `Boundary::holes`, `boundary_loops(mesh)`). Walk and close each loop on its
  own, outer first; meshes with holes are `.pde2d` version 5, one-loop meshes
  must keep writing versions 4/2 byte for byte.
- `MeshResult::report` must be checked: meshing can return a mesh that misses
  its size target. The meshing functions are `[[nodiscard]]`, and FEM entry
  points taking a `MeshResult` throw on an unsatisfied report unless given
  `accept_unsatisfied(result)`; keep new `MeshResult` overloads checked.
- **Decisions match options.** Solves throw "Stale decision" when a recorded
  `Decision` no longer matches its option. Change plan options with the
  `SolvePlan` setters (or update the decision), and give a new setting a
  `DecisionSubject` plus a case in `validate_decisions`.
- **Run record JSON is a format** (and so are the adaptive histories of
  both drivers, `adaptivity_json.hpp`, schema `pde2d.adaptive_history`
  version 2, the mesh run record,
  `mesh_record_json.hpp`, schema `pde2d.mesh_run_record`, and plan
  comparisons, `reference/compare.hpp`, schema `pde2d.plan_comparison`).
  `diff_records` skips `timings` and `outcome` groups by default; put a
  solve result that a changed input changes (iterations, residuals) under
  an `outcome` key. `record_json.hpp` keys and
  `DecisionSubject` names are stable; renaming or removing a key bumps
  `run_record_schema_version`. Fingerprint encodings carry a version tag
  (`pde2d.mesh.v1`, `pde2d.problem.v1`); changing one needs a new tag.
- **Time settings live in `TimePlan`.** `TimeOptions` derives from it; do not
  re-declare a time setting elsewhere.
- **Mesh run records never change meshes:** counters in `MeshWork` are
  incremented beside the algorithm (`Topology::work`, the refinement loop,
  curve rebuilds); meshes stay bit-identical. Every public meshing entry
  point completes `MeshResult::record` (`finish_mesh_run_record`).
- **Request-time options are not serialized:** `RefinementOptions::size_field`,
  `RefinementOptions::criterion` and `MeshReport::marked` have no `.pde2d` record. Do not add one without a
  format version (docs/formats/pde2d-mesh.md).
- **Refinement determinism:** a constant size field must give the scalar
  `max_edge_length` result bit for bit (`SizePriority` length tie-break).

## Tests

- Public headers may live in public subdirectories (`cpp/fem/reference/`);
  the `header_standalone` and `docs_header_map` checks glob everything but
  `detail/`, and the module README names them with their relative path.
- Reference problems (`fem/reference/problems.hpp`) are the oracle for
  "better is measured": judge an automatic choice with `compare_plans` /
  `compare_mesh_plans` against a simple default, and a planner with
  `compare_planners` (always against the `UserPlanner` baseline).
- Put a new test in the module directory and register it in that directory's
  `CMakeLists.txt` with `pde2d_add_cpp_test(NAME ... SOURCE ... [FEM] [LIBXML2] ...)`.
  White-box tests of `*_detail` code go in `tests/<module>/internal/` with an
  `_internal` test name; prefer a public-API check whenever behavior is
  observable.
- Use `tests/support/check.hpp` (`check`, `near`, `expect_throws`, `run`).
  Use `check_eigen.hpp` for vectors and `cdt.hpp` (`independent_cdt`) for
  meshes. Use `discretization/square_mesh.hpp` for
  structured meshes and `fem/error_norms.hpp` for L2/H1 errors.
- Oracles: exact or manufactured solutions, convergence rates, invariants, or
  independent implementations (`tools/reference/`, `tests/support/error_norms_reference.hpp`).
  Don't copy the algorithm under test into the test. To test caching, drive
  the same public components without the cache (see
  `tests/fem/reusable_evolution.cpp`).
- Exact work counters (`solver_reuses`, `preparations`) are performance
  contracts. Assert them as relations (`== time_steps - 1`), with a comment.

## Generated files (regenerate, don't hand-edit)

`docs/showcase/*`, `docs/fem/showcase.html`, `docs/fem/long-waves.html`,
`docs/fem/long-waves-timings.json`, `docs/performance/*.json|html`,
`examples/mesh_viewer/example_mesh.js`. The commands are in the README next
to each artifact. Files in `docs/archive/` are historical snapshots: don't
update them to match current code.

## Before finishing a change

1. `tools/format.sh --check`
2. `tools/test.sh`, and `tools/test_fem.sh` if `cpp/fem`, FEM tests or examples changed.
3. Update the module README or EXTENDING guide when you change a public API or
   an extension point, and this file when you change layout or conventions.
   Every public header must appear in its module README header map (the
   `docs_header_map` test fails otherwise).

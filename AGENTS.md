# Agent guide

C++23 header-only library for 2D geometry, triangle meshing and scalar P1
finite elements, plus JavaScript/Python tools that read its files. Read
[docs/architecture.md](docs/architecture.md) first for the layer diagram and
data flow, and [docs/vision.md](docs/vision.md) for where the project is
heading and which directions to keep open.

## Where things are

| Path | Contents |
| --- | --- |
| `cpp/execution.hpp` | `pde2d::ExecutionOptions`, `parallel_for` (serial/OpenMP) |
| `cpp/provenance.hpp` | `pde2d::Fingerprint` (portable FNV-1a), `StrategyDescriptor`, `library_version()` (reads the CMake-generated `pde2d_version.hpp`, from `cmake/pde2d_version.hpp.in`) |
| `cpp/discretization/` | geometry, curves, regions, boundary sampling, ear clipping/CDT meshing (size fields, marked refinement; criteria and boundary policies in `detail/refinement_policy.hpp`), point location (`point_location.hpp`), mesh validation, `.pde2d` and SVG IO. Private meshing helpers in `detail/`. [README](cpp/discretization/README.md), [EXTENDING](cpp/discretization/EXTENDING.md) |
| `cpp/fem/` | equations, boundary conditions, assembly (`system.hpp` + private `detail/assembly_*.hpp`), linear solver backends (`linear_solver.hpp`), time integration and step control (`step_control.hpp`), boundary region binding shared by assembly and analysis (`boundary_assignment.hpp`), boundary edge forms with one unit per form (`detail/edge_forms/`: Dirichlet, natural, oblique), problem analysis (`analysis.hpp`, types in `analysis_types.hpp`), decision records (`decision.hpp`), solve plans (`plan.hpp`), prepared stationary solves (`solver.hpp`), run records (`record.hpp`), error estimation and marking (`estimation.hpp`), field transfer between meshes (`transfer.hpp`). [README](cpp/fem/README.md) (header map), [EXTENDING](cpp/fem/EXTENDING.md) |
| `tests/` | `discretization/`, `fem/`, `core/`, `tools/` (Node tests), `support/` (shared helpers). Registered in each module directory's `CMakeLists.txt`; `pde2d_add_cpp_test` is defined in `tests/CMakeLists.txt` |
| `examples/` | runnable programs (learning, meshing showcase, FEM showcase, long waves). Built as CTest smoke tests |
| `benchmarks/` | opt-in timing programs; may use internals; never CTest tests |
| `tools/` | mesh viewer (`mesh_viewer/`), `plot_mesh.py`, `render_solutions.mjs`, independent references (`reference/`), `format.sh`, `test*.sh` |
| `docs/` | user docs (`fem/`, `learning/`, `showcase/`, `formats/`), current performance (`performance/`), `archive/` = dated historical reports |

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
  (`LinearSolverBackend`) and `TimeOptions::integrator` (`TimeIntegrator`).
  The enums `LinearSolver`/`TimeMethod` select the built-ins.
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
  `RunRecord` (`record.hpp`).
- **Time steps are transactional:** an integrator with state commits it in
  `accept()`, never in `trial()` (see `NewmarkIntegrator`).
- **Boundaries have loops:** an outer loop plus holes (`BoundaryCurve::loop`,
  `Boundary::holes`, `boundary_loops(mesh)`). Walk and close each loop on its
  own, outer first; meshes with holes are `.pde2d` version 5, one-loop meshes
  must keep writing versions 4/2 byte for byte.
- `MeshResult::report` must be checked: meshing can return a mesh that misses
  its size target.
- **Decisions match options.** Solves throw "Stale decision" when a recorded
  `Decision` no longer matches its option. Change plan options with the
  `SolvePlan` setters (or update the decision), and give a new setting a
  `DecisionSubject` plus a case in `validate_decisions`.
- **Run record JSON is a format.** `record_json.hpp` keys and
  `DecisionSubject` names are stable; renaming or removing a key bumps
  `run_record_schema_version`. Fingerprint encodings carry a version tag
  (`pde2d.mesh.v1`, `pde2d.problem.v1`); changing one needs a new tag.
- **Time settings live in `TimePlan`.** `TimeOptions` derives from it; do not
  re-declare a time setting elsewhere.
- **Request-time options are not serialized:** `RefinementOptions::size_field`
  and `MeshReport::marked` have no `.pde2d` record. Do not add one without a
  format version (docs/formats/pde2d-mesh.md).
- **Refinement determinism:** a constant size field must give the scalar
  `max_edge_length` result bit for bit (`SizePriority` length tie-break).

## Tests

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

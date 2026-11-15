# Agent guide

C++23 library (compiled, not header-only) for 2D geometry, triangle
meshing, their file formats, the problem description, its P1 finite
element assembly and its static and time-dependent solution: a
from-scratch rewrite for one linear scalar PDE with P1 finite elements. Read
[docs/decisions.md](docs/decisions.md) (the owner's decisions: your
specification), then [docs/architecture.md](docs/architecture.md) (modules,
types, data flow) and [docs/vision.md](docs/vision.md) (direction).

The graphical interface in `app/` is a separate program on top of the
library, and **its design is delegated to agents**: inside `app/` you
decide yourself and record the decision in [docs/app.md](docs/app.md).
Read [app/AGENTS.md](app/AGENTS.md) before working there. Everything
below about the owner deciding applies to the library.

## Architecture decisions belong to the owner

**Architecture decisions belong to the owner. Do not make a design choice
implicitly (types, interfaces, data flow, defaults, fallbacks, error
policy). Present options, trade-offs and a recommendation, and wait. If a
task needs a choice that docs/decisions.md doesn't cover, raise it instead
of picking.**

The project exists so the owner learns; a choice made silently is one the
owner never understood. Private details (helper names, loop structure)
are yours. Never contradict `docs/decisions.md`; if something there is
impossible or unclear, ask.

## Where things are

| Path | Contents |
| --- | --- |
| `include/pde2d/geometry/` | `point.hpp`, `predicates.hpp` (exact `orient`, `incircle`), `bounding_box.hpp` (`BoundingBox`), `curve.hpp` (the curve kinds with their core methods, the `CurveKind` concept, `Curve`, functions built on the core), `polygon.hpp`, `curve_geometry.hpp` (`Loop`, `CurveGeometry`), `shapes.hpp` |
| `src/geometry/` | their implementation; `predicates.cpp` is Shewchuk's adaptive arithmetic, compiled with `-fno-fast-math -ffp-contract=off` |
| `include/pde2d/mesh/` | `mesh.hpp` (output `Mesh`, with optional chains and links), `mesh_spec.hpp` (`LengthBound`, `MeshSpec`), `domain_size.hpp` (what relative lengths multiply), `curve_chain.hpp` (`CurveChain`, `CurveLink`, `default_corner_angle_degrees`), `boundary_polygon.hpp`, `working_mesh.hpp`, `report.hpp`, `components.hpp` (interfaces, improver roles, capabilities, `ImproverUse`, `PipelinePlan`, `plan_pipeline`), `delaunay.hpp` (shared flip routines), one header per component, `remesher.hpp`, `mesher.hpp` |
| `src/mesh/` | implementation; private headers `delaunay_internal.hpp`, `polygon_nodes.hpp`, `stage.hpp`, `local_improvers.hpp`, `edge_recovery.hpp`, `loop_chains.hpp` (joining a loop's curves into chains; shared by the discretizer and the Remesher), `boundary_split_policy.hpp` (the fixed, internal policy for splitting curved boundary edges), `boundary_split_steps.hpp` (its steps, a friend of `WorkingMesh`) and `mesh_validity.hpp` (the geometric validity check run by the `WorkingMesh` constructor) and `domain_size_policy.hpp` (the fixed definition of the domain size) |
| `include/pde2d/io/` | `svg.hpp` (SVG import/export of geometries, SVG pictures of meshes), `pde2d_mesh.hpp` (the old `.pde2d` format), `gmsh.hpp` (Gmsh import) |
| `src/io/` | implementation; libxml2 only in `svg_import.cpp`; private headers `svg_path.hpp` (path data parser), `text.hpp` (files and exact numbers) |
| `include/pde2d/fem/` | `expression.hpp` (`Expression`, `ExpressionFacts`, `Parameters`), `compiled_expression.hpp` (`CompiledExpression`), `problem.hpp` (`Equation`, `BoundaryCondition`, `InitialConditions`, `Problem`, `ConditionKind`, `classify`, the summaries), `meshed_problem.hpp` (`MeshedProblem` and its compiled parts), `csr_matrix.hpp` (`SparsityPattern`, `CsrMatrix`, `has_only_zero_entries`, `is_symmetric`), `assembled_problem.hpp` (`Operator`, `AssembledProblem`), `assembler.hpp` (`Assembler`) |
| `src/fem/` | implementation; private headers `symengine_expression.hpp` (the only file that includes SymEngine; `Expression::Impl`), `derivative.hpp` (`detail::derivative`, the only place derivatives are taken), `fact_summaries.hpp` (the summaries, written once for `Problem` and `MeshedProblem`) and `element_blocks.hpp` (assembly's pure element block functions); `assembler.cpp` holds the driver, patterns, colouring and `detail::AssemblyInternals` |
| `include/pde2d/solve/` | `linear_solver.hpp` (`LinearSolver`, `LinearSolverRequirements`, `PreparedLinearSolver`, `LinearSolveResult`), `sparse_lu.hpp`, `bicgstab.hpp`, `conjugate_gradient.hpp`, `simplicial_ldlt.hpp`, `solution.hpp` (`Solution`, `build_solution`), `static_solver.hpp` (`StaticSolver`, `StaticResult`), `time_method.hpp` (`TimeMethod`, `TimeMethodCapabilities`, `StartNeeds`, `StepMatrixWeights`, `TimeState`, `StepSolveInfo`), `implicit_euler.hpp`, `implicit_midpoint.hpp`, `newmark.hpp`, `step_controller.hpp`, `fixed_step.hpp`, `integrator.hpp` (`Integrator`, `IntegrationOutput`, `StepRecord`, `IntegrationResult`) |
| `src/solve/` | implementation; private headers `eigen_view.hpp` (`Eigen::Map` views of `CsrMatrix` and vectors), `iterative_solve.hpp` (the prepared solver shared by `BiCgStab` and `ConjugateGradient`), `requirements.hpp` (checking a solver's requirements against assembly's facts) and `vectors.hpp` (norms, residual, operator products); Eigen is included only in `eigen_view.hpp`, `iterative_solve.hpp` and the solver `.cpp` files (`sparse_lu.cpp`, `bicgstab.cpp`, `conjugate_gradient.cpp`, `simplicial_ldlt.cpp`); `time_methods.cpp` holds the three time methods |
| `app/` | the graphical interface (`pde2d_app`): `src/` (case format, component catalog, stages, runs in chunks, problem presets, local HTTP server), `web/` (the page, plain JS modules), `examples/` (cases), `Start pde2d app.cmd` and `start-app.sh` (double-click launcher), `tests/`, `third_party/cpp-httplib/`; its own guide is `app/AGENTS.md` and its design `docs/app.md` |
| `tests/<module>/` | one test program per topic, registered in that directory's `CMakeLists.txt`; `tests/io/data/` holds hand-written `.msh` files |
| `tests/support/` | `check.hpp` (test helper), `mesh_checks.hpp` (independent mesh oracles, including a brute-force overlap check in `check_valid_mesh`), `shapes.hpp` (test geometries) |
| `tools/` | `test.sh` (configure, build, ctest), `format.sh`, `mesh_viewer/` (the old browser viewer, unchanged except its README) |
| `examples/` | `mesh_viewer/produce_mesh.cpp` (meshes a plate with a hole, writes `.pde2d`), the viewer's startup example files (from the old code) |
| `docs/` | `decisions.md`, `architecture.md`, `vision.md`, `formats/pde2d-mesh.md` (the `.pde2d` format) |

The previous code is on the branch `archive/header-only-v1`. It is a
reference, not a source: only the predicates and curve math were ported.
Read it with `git show archive/header-only-v1:<path>`; do not check it out
in this checkout. The mesh viewer and its example files were copied from
it unchanged; keep them unchanged (the `.pde2d` format must stay readable
by them).

## Build and test

Run builds inside Linux/WSL with native tools (the checkout is on ext4).

```sh
tools/test.sh                          # configure, build, run all tests
tools/test.sh -DPDE2D_WARNINGS_AS_ERRORS=ON
tools/format.sh --check                # clang-format; tools/format.sh fixes
ctest --test-dir build -R mesh_pipeline --output-on-failure   # one test
```

Dependencies: Boost (tests only), libxml2 development files (`pde2d_io`),
Eigen 3.3 or later (`pde2d_solve`; header-only, found with
`find_package(Eigen3)`), SymEngine 0.14 (`pde2d_fem`; give its config
directory on the first
configure, for example `tools/test.sh
-DSymEngine_DIR=$HOME/.local/opt/symengine-0.14/lib/cmake/symengine`; it
is cached in the build directory), Node.js (optional: the viewer tests
are skipped without it).
`PDE2D_BUILD_EXAMPLES` (on by default) builds `examples/`.
`PDE2D_BUILD_APP` (on by default) builds the app (`build/app/pde2d_app`,
needs nlohmann/json); `pde2d::pde2d` is one target that links every
module.
`PDE2D_BUILD_DIR` picks the build directory, `PDE2D_BUILD_JOBS` the build
parallelism (capped at 4; run one build at a time), `PDE2D_TEST_JOBS` the
test parallelism. Every test program prints `PASS/FAIL <case>`.

## Conventions

- Names: snake_case for functions, variables and members; PascalCase for
  types; enumerators in snake_case.
- One CMake library per module (`pde2d_geometry`, `pde2d_mesh`,
  `pde2d_io`, `pde2d_fem`, `pde2d_solve`); the interface target
  `pde2d::pde2d` only bundles them, and nothing in the library depends on
  `app/`. Dependencies point one way:
  mesh may include geometry, io and fem may include mesh and geometry,
  solve may include fem (and below), never the reverse (and io and fem
  not each other, nor io and solve). Heavy dependencies (Boost, Eigen,
  SymEngine, libxml2) only in `.cpp` files, never in public headers.
- Public headers in `include/pde2d/<module>/`; private code and private
  headers in `src/<module>/`. Tests use only public headers. Narrow
  exception (accepted by the owner): a private helper that no public
  function reaches yet may be tested directly, if its header has no heavy
  dependency (today only `tests/fem/derivative.cpp`).
- Mesh components are virtual interfaces (`components.hpp`). Each
  component owns its options struct, set with designated initializers in
  its constructor, and validates it there. A component declares its
  capabilities or effects; compatibility is decided from those
  declarations, never from names or type checks.
- Steps an algorithm needs for correctness belong inside the component
  (Ruppert makes its own input Delaunay). Optional quality passes are
  improvers. Shared routines are plain functions (`delaunay.hpp`).
- Errors are exceptions with specific messages: `std::invalid_argument`
  for bad input or configuration, `std::runtime_error` for an algorithm
  that cannot finish. Report messages only mean "worked, but…". Tests
  match message fragments, so keep messages stable or update the tests.
- One way to do a common thing: every axis-aligned box uses
  `geometry::BoundingBox` and its functions (`bounding_box.hpp`), and a
  `MeshSpec` or `LengthBound` is always spelled with designated
  initializers (`{.max_edge_length = {.absolute = 0.5}}`, never `{0.5}`).
- Deterministic results: no dependence on hash iteration order or
  addresses; process in index order, break ties by index.
- Style: clang-format (LLVM, 80 columns).

## Invariants that are easy to break

- **A curve kind owns its core methods; nothing tests the kind by hand.**
  Mathematics that needs a kind's data is a method of the kind, listed in
  the `CurveKind` concept. What can be computed from the core is one
  function for all kinds. Code that must differ by kind in another module
  uses `std::visit(overloaded{...}, curve.kind())` with one handler per
  kind and no catch-all (`const auto &`), never a chain of `get_if`. A
  `get_if` is acceptable only as a shortcut for one kind in front of code
  that handles every kind.

- **Domain on the left.** `CurveGeometry` normalizes loops (even depth
  counterclockwise, odd depth clockwise); `BoundaryPolygon`, `WorkingMesh`
  and `Mesh` boundary edges all have the domain on their left. Triangles
  are counterclockwise.
- **Depth and parent are computed**, stored apart from `Loop`, never set by
  the user.
- **Every WorkingMesh is geometrically valid** when constructed (no
  folds, no crossing boundary edges, no piece inside another; exact
  predicates). Code that builds one, or a `Mesh` passed to `from_mesh` or
  `validate`, must produce a mesh that does not overlap itself.
- **WorkingMesh edge convention:** edge `i` of a triangle is opposite
  vertex `i`. Edits must leave the mesh valid or throw without changing
  it.
- **WorkingMesh may delete triangles and nodes** (only when a boundary
  split re-triangulates). They stay as tombstones until `compact()`, so
  code that loops over `triangles()` or `nodes()` during refinement must
  skip `is_triangle_deleted` / `is_node_deleted` entries, and queues must
  check before use. Boundary edges are never deleted. The `Remesher`
  compacts after the refiner.
- **Boundary edges carry region tags and optional curve links**
  `(chain, s0, s1)` in arc length; a refiner must update them when it
  splits (use `split_boundary_edge`, which applies the boundary split
  policy and may fail: check `BoundarySplitResult::split` and never retry
  an `is_unsplittable` edge). A linked edge is split at the arc-length
  middle on its chain, an unlinked one at the chord midpoint. The policy's
  steps are private (`detail::BoundarySplitSteps`); do not make them
  public.
- **The output `Mesh` carries its chains and links** when it came from the
  Mesher or Remesher (`to_mesh()` copies them, `from_mesh()` takes them
  back). Imported meshes have none. Code that builds or edits a `Mesh` by
  hand must keep links valid (`is_valid_link`) or drop them.
- **Improver roles are declared:** an `Improver` returns its
  `LocalImprover` from `as_local()` and its `FinalImprover` from
  `as_final()`, null for a role it lacks. Never test the type.
- **Refiners never name improvers.** They receive the resolved local
  improvers in `run`; improvers are configured only in the `Mesher` or
  `Remesher`, and a mode that cannot be honoured is a warning in the
  "pipeline" report, not an exception.
- **The Mesher never sees FEM data.** Its only external input is the
  `MeshSpec`.
- **Settings never know the domain.** A `LengthBound` (absolute and/or
  relative to `domain_size`) is resolved by the component that uses it,
  with `resolve` and `domain_size`; never write a resolved value back into
  a spec or options, and never pass the domain size between stages.
- **`MeshResult::target_met` must be checked**: a mesh can miss its target
  (for example after smoothing or without a refiner).
- **`.pde2d` keeps the old format exactly** (the viewer reads it
  unchanged). If the `Mesh` cannot fill something the format needs, ask
  the owner; never change the format on your own. Curve links have no
  place in it and are dropped on export.
- **Every derivative is taken by `detail::derivative`**
  (`src/fem/derivative.hpp`); never call SymEngine's `diff` elsewhere. It
  differentiates `abs` itself and checks its result against the
  allow-list.
- **SymEngine stays in `src/fem/symengine_expression.hpp`.** Public fem
  headers hold expressions through a pimpl (`Expression::Impl`); a `.cpp`
  that needs SymEngine includes that private header, never
  `<symengine/...>` directly. Symbolic work happens in one thread (the
  SymEngine build has no thread-safe reference counting).
- **Expressions accept only the allow-list** (`detail::unsupported` in
  `src/fem/expression.cpp`, listed in `expression.hpp`). Add a construct
  only with a test against the standard library in
  `tests/fem/expression.cpp`, a use in
  `tests/fem/expression_threads.cpp` and a test of its derivative in
  `tests/fem/derivative.cpp` (and in t in `tests/fem/meshed_problem.cpp`),
  after checking that SymEngine's
  `LambdaRealDoubleVisitor` evaluates it correctly and without shared
  state: compiled functions are called from several threads at once.
- **Facts are as written and summaries are never stored.** An
  `Expression`'s `facts()` come from its parsed form; `time_order`,
  `coefficients_depend_on_t` and `data_depends_on_t` are computed from them each
  time (`src/fem/fact_summaries.hpp`).
- **Assembly computes its own facts.** `AssembledProblem`'s
  `always_zero`, `depends_on_t`, `always_symmetric`, `time_order` and the
  right-hand side facts come from the terms assembly builds, never from
  `MeshedProblem`'s summaries. A term that depends on t goes to the time
  part, any other to the constant part; `assemble` with `previous` must
  stay bit for bit equal to a fresh assembly at the same `max_time_order`,
  also when `previous` had another one (`tests/fem/assembly.cpp`),
  including the number facts.
- **Two kinds of facts per matrix.** Formula facts (`depends_on_t`,
  `always_zero`, `always_symmetric`) hold for every t; number facts
  (`zero_now`, `symmetric_now`, read from the values of `free` with
  `has_only_zero_entries` / `is_symmetric`) hold at the assembled t only,
  and each is implied by its formula fact. The time order, reuse, "build
  or skip" and "prepare again" use only the formula facts (m2 = sin(t)
  is time order 2 also at t = 0); number facts only say whether a linear
  solver may take a matrix now, or that Newmark cannot start.
  `Assembler::time_order` must stay equal to `assemble(...).time_order()`.
- **`max_time_order` leaves terms out, it does not change them.** 1 drops
  M2 and U_D'', 0 also M1, D and U_D'; a dropped operator is a zero
  operator. Never evaluate `dirichlet_rate` or `dirichlet_acceleration`
  above the assembled time order. The `StaticSolver` assembles at 0.
- **Symmetry is a fact of the terms.** M1, M2, D, K, R, S are symmetric;
  C and T make A non-symmetric unless they are zero. T is zero when A is
  zero, or isotropic (`MeshedProblem::a_is_isotropic()`, decided
  symbolically in its constructor) with beta_tau, beta_x and beta_y zero.
  Never decide `always_symmetric` by comparing matrix values; values give
  only `symmetric_now`.
- **Element block functions are pure** (`src/fem/element_blocks.hpp`):
  no shared state, so only the driver in `assembler.cpp` would ever know
  about threads. Elements are processed colour by colour, in index order
  inside a colour; never sum into a slot in another order.
- **Assembly does no symbolic work.** Derivatives it needs (div A, U_D',
  U_D'') are compiled, and the isotropy of A decided, in the
  `MeshedProblem` constructor.
- **Eigen stays in `src/solve/`** (`eigen_view.hpp`,
  `iterative_solve.hpp` and the solver `.cpp` files); public headers use
  `fem::CsrMatrix` and `std::vector`/`std::span`.
- **The integrator decides from declarations.** A method's supported time
  orders come from `capabilities().supports(order)`, checked against
  `Assembler::time_order(meshed)` (formula facts, nothing evaluated);
  what is assembled at t0 comes from `start_needs(order)` (`nothing`: no
  assembly at t0 and no `start` call, `static_part`: `max_time_order` 0,
  `whole_problem`: 2); whether to prepare the linear solver again comes
  from dt and the operators' `depends_on_t` / `always_zero` for the
  non-zero weights of `step_matrix_weights(dt)`. Never test a method's
  type or name.
- **Linear solvers declare requirements; callers check them** before
  every `prepare` (`detail::check_requirements`): a symmetry requirement
  is met when the matrix is always symmetric by the facts or symmetric
  now by its values (`Operator::symmetric_now`; for a step matrix, every
  operator with a non-zero weight always symmetric or always zero, else
  `is_symmetric` of the step matrix, `detail::check_step_matrix_requirements`).
  A new caller that prepares a matrix must check too. Definiteness is not declared: a solver that needs it throws
  `std::runtime_error` when it fails, never returns an unchecked answer.
- **No fallbacks between methods or solvers**: one that cannot handle a
  problem throws with the reason (and may name alternatives); it never
  switches to another.
- **Time methods use `right_hand_side()` as it is**: it already holds the
  whole lifting (load − A_FD U_D − B_FD U_D' − M2_FD U_D''). The static
  solver assembles with `max_time_order` 0, where it is load − A_FD U_D.
- **One set of step weights per method serves all its orders**: a zero
  operator adds nothing (`detail::add_product` skips it), so a method
  tells the orders apart only by `always_zero` of M2 and B.
- **Never invert B or M2 in a step**: B can be singular (m1 = 0 with a
  dynamic condition) and either can be a zero operator. Only `Newmark`'s
  `start` solves with M2, and fails with a message naming the methods that
  need no acceleration (first when M2 is `zero_now` at t0, then when the
  solve fails).
- **Step times are anchor + k·dt**, not sums, and steps land exactly on
  snapshot times and t1, so a continued run (`integrate(meshed, t1, t2,
  output, &previous.final)`) matches one long run that lands on t1 (bit
  for bit for the first-order methods; `tests/solve/time_integration.cpp`).
- **Not yet implemented, on purpose** (see "Deferred" in decisions.md):
  adaptive time stepping, explicit time methods,
  handling pure Neumann problems, well-posedness
  checks, callables, anisotropic meshing, planners,
  the simple-options facade, Gmsh export, reading the old boundary
  context of `.pde2d` files as curve links. Do not add them unless a task
  asks.

## Reviewing

A review looks for problems a user of this library can actually meet:
wrong mathematics or signs, silent wrong answers, crashes, exceptions or
broken invariants on ordinary input, and numerical trouble that ordinary
problems trigger (small pivots, ill-conditioning, non-convergence,
cancellation at normal scales). Ordinary input means sizes and
coefficients of real engineering problems, meshes from our mesher or
Gmsh, and problems that make physical sense.

Do not report cases that need extreme magnitudes or constructions no real
problem produces: coordinates or radii of 1e9 or 1e200 with tiny arcs,
values near overflow or underflow, subnormals, time steps below the
spacing of doubles, adversarial meshes no mesher makes. If you notice one
whose fix is a line or two, mention it in one line at the end under
"Unlikely"; otherwise leave it out. Rank findings by how likely a real
user meets them, then by how bad the result is.

## Tests

- Register a test with
  `pde2d_add_test(NAME <module>_<topic> SOURCE <file> LIBS <library>)` in
  `tests/<module>/CMakeLists.txt`.
- Use `tests/support/check.hpp` (`check`, `near`, `expect_throws`, `run`)
  and, for meshes, `tests/support/mesh_checks.hpp`.
- Oracles: exact values, invariants (orientation, conformity, edges used
  once equal the boundary edges, area, Delaunay, minimum angle, edge
  lengths) or independent computations. Do not copy the algorithm under
  test into the test.
- `header_standalone` compiles every public header on its own.

## Before finishing a change

1. `tools/format.sh --check`
2. `tools/test.sh` (all tests pass)
3. Update `docs/architecture.md` when you change a public type, interface
   or the data flow, and this file when you change layout or conventions.
   Decisions go to the owner, not into the docs on your own.

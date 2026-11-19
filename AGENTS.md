# Agent guide

C++23 library (compiled, not header-only) for 2D geometry, triangle
meshing, their file formats, the problem description, its P1 finite
element assembly, its static and time-dependent solution, and error
estimation, marking, refinement, solution transfer and the adaptive
loops for static and time-dependent problems: a
from-scratch rewrite for one linear scalar PDE with P1 finite elements.
Read [docs/vision.md](docs/vision.md) (direction), then the sections of
[docs/decisions.md](docs/decisions.md) (the owner's decisions: your
specification; always its Process and Project sections) and
[docs/architecture.md](docs/architecture.md) (modules, types, data flow)
that cover the modules your task touches; both are
organised by module, so find the sections by their headings. Do not read
them whole unless the task spans every module.

The graphical interface in `app/` is a separate program on top of the
library, and **its design is delegated to agents**: inside `app/` you
decide yourself and record the decision in
[docs/app-decisions.md](docs/app-decisions.md) (the design as it is:
[docs/app.md](docs/app.md)).
Read [app/AGENTS.md](app/AGENTS.md) before working there.

**Library changes the app needs are delegated too** (the owner,
2026-11-19). An agent working on the app may change the library for what
the app needs (a new option, a callback, a function the app would
otherwise reimplement) and decides that design itself: keep this file's
conventions and invariants, add tests, update
[docs/architecture.md](docs/architecture.md), and record the decision
with its reason in [docs/decisions.md](docs/decisions.md), marked "(for
the app, by an agent)", and in
[docs/library-requests.md](docs/library-requests.md). Everything below
about the owner deciding applies to the rest of the library's work.

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

The table of files (which header holds which type, the private headers,
tests, tools) is in [docs/architecture.md](docs/architecture.md#files).
Look up the rows you need there; `ls` shows the rest.

The mesh viewer and its example files come from the previous code; keep
them unchanged (the `.pde2d` format must stay readable by them).

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
`find_package(Eigen3)`), SuiteSparse 7.12 with CHOLMOD and ParU and
OpenBLAS (`pde2d_solve`, the parallel direct solvers; CMake config files
of CHOLMOD and ParU, OpenBLAS found by `find_library`; OpenBLAS should
be the system BLAS, the pthreads build; CHOLMOD's config needs the C
language enabled), SymEngine 0.14 (`pde2d_fem`; give its config
directory on the first
configure, for example `tools/test.sh
-DSymEngine_DIR=$HOME/.local/opt/symengine-0.14/lib/cmake/symengine`; it
is cached in the build directory), OpenMP (`pde2d_fem` for assembly,
`pde2d_solve` for Eigen's parallel matrix-vector product, a public
requirement there so consumers that include Eigen match it; GCC's
libgomp, found with `find_package(OpenMP)`), Node.js (optional: the viewer tests
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
  `pde2d_io`, `pde2d_fem`, `pde2d_solve`, `pde2d_adapt`); the interface
  target `pde2d::pde2d` only bundles them, and nothing in the library
  depends on `app/`. Dependencies point one way: mesh may include
  geometry, io and fem may include mesh and geometry, solve may include
  fem (and below), adapt may include solve, fem, mesh and geometry, never
  the reverse (and io and fem not each other, nor io and solve, nor adapt
  and io; nothing includes adapt). Heavy dependencies (Boost, Eigen,
  SymEngine, SuiteSparse, OpenBLAS, libxml2, OpenMP's `<omp.h>`) only in
  `.cpp` files (and private headers), never in public headers.
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
  compacts after the refiner (in `run` and `refine`).
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
  improvers in `run` and `refine`; improvers are configured only in the
  `Mesher` or `Remesher`, and a mode that cannot be honoured is a warning
  in the "pipeline" report, not an exception.
- **One set of refiners for meshing and adaptivity.** Every `Refiner` has
  `run(mesh, spec, ...)` and `refine(mesh, marked, ...)`. The three
  bisection refiners are rules for one private core
  (`src/mesh/edge_bisection.*`); write bisection logic there, not in a
  refiner. Their `run` is the loop "refine the triangles with a long
  edge" around `refine`.
- **Parents survive only by declaration and by the run.** The Remesher
  returns `Parents` only if the refiner declares `keeps_parents` and its
  `refine` returned them, and every local improver and final pass of the
  plan declares `ImproverEffects::keeps_parents`; otherwise the pipeline
  report names who dropped them. A component that flips or moves nodes
  must not declare `keeps_parents`. The core drops them itself when the
  boundary split policy re-triangulates (only then can tombstones
  appear).
- **One measurement of the final mesh.** The Remesher measures its final
  mesh once (`measure`, `MeshProperties`); `target_met` and the angle
  warning in the pipeline report are read from it. Do not add a second
  place that computes a property of the final mesh (longest edge, angles,
  counts); read `properties`.
- **The Mesher never sees FEM data.** Its only external input is the
  `MeshSpec`.
- **Settings never know the domain.** A `LengthBound` (absolute and/or
  relative to `domain_size`) is resolved by the component that uses it,
  with `resolve` and `domain_size`; never write a resolved value back into
  a spec or options, and never pass the domain size between stages.
- **`MeshResult::target_met` must be checked**: a mesh can miss its target
  (for example after smoothing or without a refiner). It is
  `properties.longest_edge <= max_edge_length`, from the one measurement.
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
  no shared state, so only the driver in `assembler.cpp` knows about
  threads. Elements are processed colour by colour; inside a colour they
  run on `AssemblerOptions::threads` threads (`for_each_in_colour`), which
  is safe and bit for bit the same only because no two elements of a
  colour share a node, so each slot gets at most one addition per colour.
  Never sum into a slot in another order, never add a shared accumulator
  or atomics, and never let an exception leave an OpenMP thread (catch
  it; the colour rethrows the one of the lowest element index).
- **Threads never change a result.** Assembly and the iterative solvers
  give the same bits for every thread count (`tests/fem/assembly_threads.cpp`,
  `tests/solve/threads.cpp`); the count is an option of the component
  (default 1, 0 = `omp_get_num_procs()`, at most 1024; checked and
  resolved by `src/solve/threads.hpp` in the solvers), never a global.
  Every Eigen computation in `pde2d_solve` runs inside a
  `detail::EigenThreads` scope (`src/solve/eigen_threads.hpp`), which sets
  the calling thread's own OpenMP count: the iterative solvers their
  option, the Eigen direct solvers 1. Never call `Eigen::setNbThreads` (a
  global) in the library. The one accepted exception to "the same bits"
  are the parallel direct solvers (`SupernodalCholesky`,
  `MultifrontalLu`): their answers may differ in the last digits between
  thread counts (and between runs for the LU), never with 1 thread
  (`tests/solve/parallel_direct.cpp`). Every SuiteSparse call runs inside
  a `detail::FactorizationThreads` scope
  (`src/solve/factorization_threads.hpp`): it sets OpenBLAS's count (one
  for the whole process) and the calling thread's OpenMP count, and holds
  a process-wide lock so that two such solves never run at once. Never
  call `openblas_set_num_threads` elsewhere.
- **Assembly does no symbolic work.** Derivatives it needs (div A, U_D',
  U_D'') are compiled, and the isotropy of A decided, in the
  `MeshedProblem` constructor.
- **Eigen and SuiteSparse stay in `src/solve/`** (Eigen: `eigen_view.hpp`,
  `eigen_threads.hpp`, `iterative_solve.hpp` and the solver `.cpp` files;
  SuiteSparse: `suitesparse.hpp`, `supernodal_cholesky.cpp`,
  `multifrontal_lu.cpp`); public headers use `fem::CsrMatrix` and
  `std::vector`/`std::span`. Both are linked privately; OpenMP is a public
  requirement only because a consumer that includes Eigen must compile
  Eigen's inline functions with it (SuiteSparse's headers have no such
  inline functions).
- **The integrator decides from declarations.** A method's supported time
  orders come from `capabilities().supports(order)`, checked against
  `Assembler::time_order(meshed)` (formula facts, nothing evaluated);
  what is assembled at t0 comes from `start_needs(order)` (`nothing`: no
  assembly at t0 and no `start` call, `static_part`: `max_time_order` 0,
  `whole_problem`: 2); whether to prepare the linear solver again comes
  from dt and the operators' `depends_on_t` / `always_zero` for the
  non-zero weights of `step_matrix_weights(dt)`; whether `Solution`s
  carry accelerations, and whether a run given them skips the start,
  from `keeps_acceleration(order)`; whether a step's velocity quotient
  may serve as the acceleration at its end, from
  `velocity_quotient_is_acceleration(order)`. Never test a method's type
  or name.
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
- **Direct solvers check the backward error and refuse only exact
  singularity.** `SimplicialLdlt`, `SupernodalCholesky` and
  `MultifrontalLu` compare `LinearSolveResult::backward_error`
  (|b − Ax|∞ / (|A|∞ |x|∞ + |b|∞), from `detail::measure_residual`) with
  `residual_tolerance`, never the relative residual (`residual`, which a
  coefficient contrast makes large for an accurate answer). They throw on
  an exact zero pivot (the Cholesky on any pivot that is not positive);
  a tiny pivot is reported in `pivot_ratio`, never refused by a
  threshold (a pure Neumann problem gets one of its solutions); below
  `detail::small_pivot_ratio` (1e-10, the one place: `src/solve/pivot_ratio.hpp`)
  the result carries the warning of `detail::pivot_ratio_warning`, made
  once per prepared solver; its text names the solver and the threshold,
  never the ratio (that is `pivot_ratio`), so it is the same on every
  solve of a solver and an integrator that prepares at every step reports
  it once. ParU's ratio must
  include its singleton pivots, and no NaN may enter it: a zero row is
  refused before ParU sees the matrix, a zero singleton before dividing.
  A residual entry that is not finite gives a NaN backward error
  (`detail::max_abs` keeps NaN), which every check rejects.
- **The Cholesky factors −A for a negative diagonal, and says so.** When
  every diagonal entry is negative, `SupernodalCholesky` factors a negated
  copy and solves with −b; the checks use A and b as given, and every
  result carries the warning in `LinearSolveResult::messages`. A solver's
  warnings go only there (failures are exceptions); the `StaticSolver`
  copies them into its report and the `Integrator` logs every solve's
  through its private `LoggingSolver`, so a time method never has to pass
  them on. A new caller that solves must put them in its report. The
  Integrator's log (`SolveLog`) is shared per-run state behind a mutex
  (held only to add or copy, never during a solve): a time method may
  solve from several threads at once and keep a prepared solver beyond
  `integrate()`. The report's `max_backward_error` and `min_pivot_ratio`
  come from that log (every solve, `start()`'s too), never from
  `StepSolveInfo`, and are read with its messages in one snapshot (one
  lock), never under separate locks.
- **Public result structs grow at the end.** A new member of a public
  result struct (`LinearSolveResult`, `IntegrationResult`, and the
  others) goes after the last one, so that positional initializations
  keep their meaning: `LinearSolveResult` keeps values, iterations,
  residual, threads first, `IntegrationResult` final, snapshots,
  history, report (both tested positionally).
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
  solve fails). A run continued from a `Solution` with accelerations does
  not call it (so Newmark continues bit for bit, also where M2 is zero);
  keep the accelerations in every `Solution` the Integrator returns for a
  method that keeps them.
- **Step times are anchor + k·dt**, not sums, and steps land exactly on
  snapshot times and t1, so a continued run (`integrate(meshed, t1, t2,
  output, &previous.final)`) matches one long run that lands on t1 (bit
  for bit for the first-order methods; `tests/solve/time_integration.cpp`).
  A run stopped by `max_steps` continues the same stretch only when given
  its `IntegrationResult::anchor` (`StepAnchor`). A quotient of the last
  step divides by the step's own length (`FreeState::dt`), never by a
  difference of times (a landing keeps the nominal dt).
- **A triangle's refinement edge is the edge opposite its vertex 0**
  (the convention in `include/pde2d/mesh/mesh.hpp`). The bisection
  refiners read it from the vertex order and write it into every triangle
  a split makes (newest vertex first, `WorkingMesh::set_refinement_edge`),
  so it persists across refinement cycles without a field in `Mesh`. Code
  that copies a `Mesh` must keep each triangle's vertex order as it is
  (to_mesh/from_mesh, compact, the `.pde2d` round trip and refinement
  do); code that builds or edits triangles (flips, Ruppert, triangulators,
  the boundary split policy, Gmsh import) may choose vertex 0 freely
  inside the library. **The library delivers the order; users never call
  `set_longest_edges_as_refinement_edges`** (it is public only for meshes
  built by hand): `Remesher::run` (so the `Mesher`) and Gmsh import rotate
  every triangle to its longest edge, `Remesher::refine` only the
  triangles whose order an edit chose (the `WorkingMesh` remembers the
  order its constructor received or `set_refinement_edge` last gave; a
  triangle that differs was made or changed by an edit), all through
  `src/mesh/refinement_edges.*`, after the final passes. A bisection must
  order every triangle it makes with `set_refinement_edge`, or the
  Remesher will reorder it. `.pde2d` import keeps the file's order. Never
  reorder the vertices of an untouched triangle.
- **Refinement has one rule for boundary points.** A boundary edge is
  split only by `split_boundary_edge` (the boundary split policy), also in
  `refine`; if the policy re-triangulates, the parents are dropped and the
  report says why; an unsplittable edge leaves the triangles waiting for
  it unrefined. Never place a curve point another way or fall back to the
  chord midpoint. `Remesher::refine` validates its result and throws
  `std::runtime_error` if it overlaps itself. When parents are kept,
  every new node lies on an input edge, every output triangle names its
  input parent, and input nodes keep their indices: keep these when
  changing the core, the solution transfer relies on them.
- **The estimators' residual follows assembly's weak form.** The element
  residual is f - b . grad u_h - c u_h - m1 u_t - m2 u_tt (the A and div A
  parts cancel for P1), the jump is that of (A grad u_h) . n, and the
  boundary residual of a natural or dynamic edge is kappa_n / beta_n_hat
  times the condition's residual as written. Change them together with
  assembly's signs (`src/fem/element_blocks.cpp`). Every integral takes
  its rule from `quadrature.hpp`; an estimator's degrees are its options
  (`triangle_degree`, `edge_degree`, checked in its constructor), with
  defaults exact for coefficients and data of degree at most 1.
- **Drivers decide from declarations.** The `AdaptiveStaticSolver` and
  the `AdaptiveIntegrator` read `Adaptation::capabilities()`
  (`estimates_error`, `changes_mesh`): the cycle's estimate is that of
  the first adaptation that estimates, and at most one adaptation may
  change the mesh (all act on the same solved state). Never test an
  adaptation's or estimator's type.
- **A solution moves to a new mesh only by `transfer_solution`**
  (`src/adapt/solution_transfer.cpp`): interpolation through the parents
  when they are kept, else point location with the private
  `detail::PointLocator` (never `WorkingMesh::locate`, which can cycle:
  issue #22); Dirichlet nodes from the new `MeshedProblem`. Without
  parents a boundary node takes its old boundary point through its curve
  link, else on the old loop matched to its own (never the nearest point
  of another wall); keep that when changing the transfer. The
  `AdaptiveIntegrator` integrates in pieces with `Integrator::integrate`
  (its `max_steps` and `keep_before_last_step`), never re-implementing
  the step times, and continues a piece on the same problem from its
  `final` and its `anchor` (every piece on the same problem; after a
  transfer to a new mesh a new stretch starts), so that a run without
  refinement equals one integrate() call landing on the interval's
  adaptation times (the "every n steps" times are not landings: they are
  those of the uninterrupted run); an adaptation time near a snapshot
  time is that time. `before_last_step` is a `FreeState` (free unknowns,
  no boundary data evaluated), requested only for a piece that can end
  in an adaptation; at the Dirichlet nodes u_tt is the boundary data's
  acceleration at t, never a quotient that needs the rate at t − dt.
  Where only the method changes (after damping steps, before the
  adaptation time) the next piece continues the damping piece's
  `anchor` too, so damping steps never change the step times.
  The transfer moves accelerations like velocities. Damping steps after
  a transfer run only when `AdaptiveIntegratorOptions::damping_steps` asks
  for them, as their own piece with `Integrator::with_method`; never
  switch methods for any other reason (no fallback). After damping
  steps at time order 2 the state (and a snapshot at that time) carries
  the acceleration the damping method declares: its own if it
  `keeps_acceleration`, else its last step's velocity quotient if it
  declares `velocity_quotient_is_acceleration` (implicit Euler), so a
  method that keeps one continues without its start; any other damping
  method ahead of such a method is refused before the first step
  (`std::invalid_argument` naming both), never given a wrong
  acceleration.
- **Error control decides from declarations.** A step controller says
  whether it controls the error (`StepController::controls_error()`); a
  time method declares its estimate per time order
  (`TimeMethod::error_estimate`: order p, earlier steps read) and
  computes it in `local_error` from states it has, never with an extra
  solve. The Integrator refuses a method without one before the first
  step (no fallback). A controller that does not control the error takes
  the old loop exactly (FixedStep stays bit for bit as it was: no state
  copies, `judge` never called). Controllers are stateless; what a run
  remembers (next dt, the last accepted steps) is in the Integrator's
  loop and travels in `StepAnchor` (`next_dt`, `previous_steps`), which
  the AdaptiveIntegrator passes across pieces on one problem; after a
  transfer only `next_dt`. A rejected step is recorded with `accepted =
  false` and retaken from its start; `steps`, `max_steps`,
  `snapshot_every` and `keep_before_last_step` count accepted steps.
- **Not yet implemented, on purpose** (see "Deferred" in decisions.md):
  explicit time methods,
  handling pure Neumann problems, well-posedness
  checks, coarsening, L2 projection for the solution transfer,
  re-integrating a piece after refining, callables, anisotropic meshing,
  planners,
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
3. Update `docs/architecture.md` when you change a public type, interface,
   the data flow or the layout (its file table), and this file when you
   change conventions.
   Decisions go to the owner, not into the docs on your own.

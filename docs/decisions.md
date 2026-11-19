# Design decisions

The owner's record of architecture decisions for the rewrite. Each entry says
what was decided and why. Agents implement what is here. They do not decide
anything that is missing: they raise it with the owner. Proposals and open
questions are listed separately at the end.

Started 2026-11-11.

## Process

- **The owner decides the architecture.** Agents present the options with
  their trade-offs and a recommendation, then wait for an answer. *Why:* the
  project exists so the owner learns. Choices an agent makes silently are
  choices the owner never understood.
- **Design is done in discussion and implementation by subagents**, one
  agreed slice at a time.

- **Reviews look for problems that can happen** (decided 2026-11-15):
  wrong mathematics, silent wrong answers, crashes or broken invariants on
  ordinary input, and numerical trouble that ordinary problems trigger
  (small pivots, ill-conditioning, non-convergence). Cases with extreme
  magnitudes or constructions no real problem produces (coordinates of
  1e9 with tiny arcs, values near overflow or underflow, steps below the
  spacing of doubles) are not findings; they are fixed only when the fix
  is trivial. Reviews are run by GPT-6.2 Sol at high effort (since
  2026-11-17, after a side-by-side review; before: GPT-6.1 Sol).

## Project

- **Rewritten from scratch as a compiled C++ project, not header-only.**
  *Why:* the header-only design recompiled Eigen and SymEngine code in every
  translation unit, and the old code was no longer understood.
- **Layout:** public headers live in `include/pde2d/<module>/` and the
  implementation in `src/<module>/`. There is one CMake library per module:
  `pde2d_geometry` and `pde2d_mesh` now, `pde2d_io` and `pde2d_fem` later.
  Heavy dependencies (Eigen, SymEngine, libxml2) are included only from
  `.cpp` files, never from public headers. C++23, clang-format (LLVM style,
  80 columns). *Why:* code that uses the library then never compiles those
  dependencies, which fixes the slow builds.
- **Tests:** a tiny helper without dependencies (`check`, `near`,
  `expect_throws`), as in the old code, registered with CTest.
- **Exact predicates use Shewchuk's adaptive arithmetic** (decided
  2026-11-12), replacing Boost.Multiprecision. During the switch they are
  tested against the Boost version on many nearly degenerate inputs. Never
  compile them with `-ffast-math`. *Why:* the same exact answers, faster,
  and no dependency. Coordinates outside [2⁻²⁰⁰, 2²⁰⁰], where Shewchuk's
  arithmetic is not exact, go to a small exact integer fallback that is
  private to `predicates.cpp` (accepted 2026-11-12). *Why:* the predicates
  stay exact for every finite input, and the fallback stays isolated in one
  file. Linking a program with `-ffast-math` still flushes subnormals to
  zero process-wide; no single file can prevent that.
- **The graphical interface is delegated** (2026-11-14). It lives in
  `app/` as a separate program over the library's public headers;
  agents decide its design themselves and record it in
  [app.md](app.md). The library's own design stays with the owner, and
  the app may not change it. `pde2d::pde2d` links every module as one
  target; the modules stay separate libraries. *Why:* the owner wants a
  working interface to enter settings and see results without
  overseeing its details.
- **Only the predicates and curve math were ported** from the previous
  code (self-contained, well-tested pieces). Everything else is written
  fresh to this design. *Why:* the
  owner must not again inherit code they don't understand.
- **Scope: one linear scalar PDE with P1 elements.**
- **Input is symbolic only (SymEngine). No callables in v1, not even where no
  derivatives are needed.** *Why:* the weak form needs derivatives of the
  coefficients (for example ∂a_xx/∂x), and coefficient facts can be computed
  from expressions, while for a callable they would have to be promised by
  the user.

## Geometry

- **`CurveGeometry` is a set of closed loops of exact curves** (line, arc,
  Bézier). Each curve carries a boundary region tag, and the same region
  name may appear on several loops.
- **Loop roles come from nesting depth, which the program computes.** A loop
  of even depth is the outer boundary of a piece of the domain, a loop of odd
  depth is a hole, and islands are supported. The user never marks loops by
  hand. *Why:* containment has to be computed to validate a mark anyway, so a
  hand-written mark only adds a way for the input to contradict itself.
- **Computed loop data (depth, parent) is kept in separate private vectors
  next to the loops, not inside `Loop`.** *Why:* a `Loop` is input that the
  user builds, while depth and parent are computed from all loops together.
  Separate storage keeps it clear what is supplied and what is derived.
- **Loop direction is normalized when the geometry is built:** even depth
  counterclockwise, odd depth clockwise. *Why:* the domain is then always on
  the left, so one rule gives the outward normal on every loop.
- **Pre-built shapes (circle, rectangle, annulus, ...) are factory functions
  that return a `CurveGeometry`, not subclasses.** *Why:* they add no data,
  invariants or behaviour of their own. Rule: use a function when the result
  is fully described by the base type; create a new type only for its own
  data, invariants or behaviour.
- **Curves: each kind owns its core mathematics as methods; `Curve` is a
  value class around a `std::variant` of the kinds** (decided 2026-11-12).
  The core is what needs the kind's own data (point, derivatives, split,
  box, ...), stated once in the `CurveKind` concept and checked at compile
  time. Everything computable from the core is a function written once for
  all kinds. Code that differs by kind but belongs to another module (SVG)
  visits `curve.kind()` with one handler per kind and no catch-all. Calls
  are methods (`curve.point_at(t)`), not free functions. *Why:* a new kind
  is one struct with a known list of methods, and forgetting one, or
  forgetting a kind in another module, fails at compile time. Inheritance
  was rejected: the kinds are a closed set fixed by the file formats, and
  it would make every curve a pointer.
- **Region editing (split, merge, rename) stays as methods of
  `CurveGeometry`.** *Why:* these operations must preserve the class's
  invariants: unique names, closed loops and correct depths.
- **SVG import:** `<path>` elements give the curves. `data-region`, else
  `id`, gives the region name, and holes are found by depth. **The y-axis is
  flipped** (decided 2026-11-12): y points up, as in mathematics, so
  coefficients written in x and y mean what they say. `transform`
  attributes, basic shapes and `<use>` are rejected with a clear error;
  units and `viewBox` are ignored. Open runs are joined end to start when
  their ends meet (within 1e-9 of the drawing size). The remaining import
  and export details in the implementation were accepted on 2026-11-12.
- **Input/output** (library `pde2d_io`: SVG import and export, `.pde2d`
  import and export, Gmsh 4.1 and 2.2 ASCII import) is implemented.
  **`.pde2d` keeps the old format** (version 2 for one loop, version 5 for
  several), so the old viewer works unchanged; curve links are dropped on
  export because the format has no place for them. Gmsh boundary edges must
  belong to a physical line group, which gives their region. If the new mesh
  cannot fill the format, ask the owner before changing it.
- **Very flat SVG arcs become cubic Béziers** (accepted 2026-11-12): an arc
  that sweeps less than 0.01 rad is imported as a cubic through its exact
  end points. *Why:* `EllipticalArc` reaches its end points through its
  centre; for a radius of 1e8 over a chord of 1 the end lands about 1e-8
  off, above the closure tolerance. Such an arc comes back from export as a
  cubic. A proper end-point arc type is deferred.
- **Loop direction is decided exactly** by the public
  `geometry::polygon_orientation` (the `orient` sign of the turn at the
  lowest point), not by the sign of the area (accepted 2026-11-12). *Why:*
  the area underflows or overflows at extreme scales.
- **Gmsh import is strict** (accepted 2026-11-12): entity dimensions must be
  0 to 3, and every count must fit in what remains of the file before
  anything is allocated.
- **Imported meshes get a full geometric validity check** (decided
  2026-11-12), not only a connectivity check: counterclockwise triangles,
  conforming edges, no folds around interior nodes, no crossing boundary
  edges and no piece lying inside another. The test oracle checks overlaps
  independently. *Why:* a mesh can be connected correctly and still
  overlap itself.

## Meshing

- **The mesher never sees the `Problem`.** Its only external input is a
  `MeshSpec`. *Why:* this keeps meshing independent of the FEM code, and
  lets a future planner (preprocessing) or error estimator (adaptivity) talk
  to the mesher through a single contract.
- **v1 `MeshSpec` = maximum edge length.** It is a struct so that it can grow:
  later a metric tensor field for anisotropic meshes, with the isotropic size
  as a special case.
- **Length limits are `LengthBound{absolute, relative}`** (decided
  2026-11-12), both `std::optional<double>`: an absolute length, a multiple
  of the domain size, or both, and with both the smaller one applies. An
  empty value is "no bound" (not 0, not infinity). Used for
  `MeshSpec::max_edge_length` (at least one value required, else
  `std::invalid_argument`) and `ChordLengthOptions::max_curve_deviation`.
  Every set value must be finite and positive. *Why:* an absolute default
  like 0.02 means "very strict" or "never active" depending on the units;
  relative values give the same mesh at every scale.
- **Settings stay settings.** `MeshSpec` and options know nothing about the
  domain. Each component that uses a `LengthBound` resolves it itself, with
  the shared `resolve(bound, domain_size)` and `domain_size(...)`; no
  resolved spec type is passed between stages and `run()` signatures are
  unchanged. *Why:* the spec is what is asked for, the domain size is a fact
  about the input; and a component called on its own still behaves
  correctly.
- **The domain size is a fixed internal policy**, like the boundary split
  policy: currently the diagonal of the axis-aligned bounding box of the
  boundary (exact extremes of the curves). It is within a factor sqrt(2) of
  the diameter but changes when the domain is rotated; a different
  definition (for example the diameter) replaces the policy in one place.
  It is measured on the geometry's curves, on a mesh's chains when it has
  them, else on its boundary nodes, so every stage of a run sees the same
  number and refinement does not change it.
- **No caching of derived quantities** for now; they are recomputed (one
  pass over the boundary). Caching on `Mesh` is not possible while it is a
  public struct: GitHub issue #26.
- **Resolved values are reported:** `MeshResult::max_edge_length` and
  `domain_size`, and the metrics `domain_size`, `max_edge_length` (and
  `max_curve_deviation` when set) in the discretizer's and refiners'
  reports.
- **Stages run in a fixed order:** `BoundaryDiscretizer` →
  `InitialTriangulator` → `Refiner` (optional) → `Improver`s (a list).
- **Components are virtual interfaces.** Each component owns its own options
  struct, set with designated initializers. *Why:* stages run once per mesh,
  so the cost of a run-time call is negligible, and the code stays in `.cpp`
  files instead of headers.
- **The working mesh is separate from the output mesh.** The working mesh is
  mutable, knows triangle neighbours, and links each boundary edge to its
  source curve and parameter interval. The output mesh is plain arrays.
- **Compatibility is checked twice:** when the `Mesher` is constructed
  (component against component) and when a run starts (component against
  input), using capabilities that each component declares.
- **The refiner updates boundary edges as it works.** *Why:* region tags
  cannot be recovered from the triangles afterwards.
- **Curve links are optional per boundary edge.** When a link is present, a
  split boundary edge gets its new point on the true curve. When it is
  missing (imported meshes), the new point goes at the chord midpoint.
- **Merging curve junctions happens in the discretizer, not the
  geometry** (decided 2026-11-12). *Why:* the geometry stays exact and
  untouched; simplification is a meshing concern. It works with **chains**
  (decided 2026-11-12): the discretizer joins consecutive curves of one
  loop into a chain unless the region changes or the boundary turns by more
  than `corner_angle_degrees` (a discretizer option) at the junction. Each
  chain is sampled as one path by arc length. A boundary edge's link is
  `(chain, s₀, s₁)`, an arc-length interval, and a split puts the new point
  at `s = (s₀ + s₁)/2`, found by a binary search over the chain's
  cumulative curve lengths and length inversion inside that curve. Chains
  live on the meshing side (polygon, working mesh and, optionally, the
  output mesh), never in the geometry. The default `corner_angle_degrees`
  is 30° (decided 2026-11-12): it keeps the corners of rectangles and
  hexagons and merges smooth joins and fine polylines. The `Remesher` builds
  chains the same way when it links a mesh to a geometry.
- **Splitting a curved boundary edge goes through a replaceable policy**
  (decided 2026-11-12), a virtual interface like the other components. The
  v1 policy: split at the arc-length midpoint; if a triangle would invert,
  re-triangulate locally (insert the point, force the two new boundary
  edges by flips, delete what is now outside, including nodes in the
  cut-off region, which lie outside the true domain); if that also fails,
  skip the split, warn, and the target is not met. The working mesh may
  therefore delete nodes and triangles (tombstones until `compact()`). The
  inward step inserts the point, forces the two new edges, then deletes the
  cut-off region with its nodes, so no hole is left to re-triangulate. Only
  `WorkingMesh::split_boundary_edge` is public; the policy's individual
  steps are private. There is no chord fallback for
  linked edges. **The policy is internal, never a user input:** the
  library picks one fixed policy at compile time, and changing it means
  changing the library. *Why:* it is our own robustness machinery, not a
  meshing choice the user should have to understand.
- **Merging curves of the same kind at import is deferred** (decided
  2026-11-12).
- **Refining an existing mesh:** a separate `Remesher{refiner, improvers}`
  refines a mesh that came from anywhere, and the `Mesher` uses a `Remesher`
  internally.
- **v1 algorithms:**
  - boundary discretizer: every chord ≤ `max_edge_length`; the curve
    deviation is the component's own option, off by default (changed
    2026-11-12 from an absolute 0.02);
  - triangulators: ear clipping (bridging holes, run once per piece) and
    CDT;
  - refiners: Ruppert (keeps the mesh Delaunay internally, min-angle option)
    and longest-edge bisection (accepts a local improver);
  - improvers: Delaunay flips (whole mesh and local) and Laplacian
    smoothing.
- **Steps that an algorithm needs for correctness belong inside the
  component. Optional quality passes are improvers.** Ruppert's flips are of
  the first kind. Shared routines (for example `flip_to_delaunay`) are plain
  functions that several components may call.
- **Ruppert makes its own input Delaunay** when it isn't (for example after
  ear clipping) and reports how many flips that took.
- **Improver interfaces** (revised 2026-11-12): `Improver` is the common base
  that goes into the improver list (name, effects, use mode). It declares
  its roles itself: `as_local()` returns a `const LocalImprover*` and
  `as_final()` a `const FinalImprover*`, `nullptr` for a role it does not
  have. `LocalImprover::improve_around(mesh, touched)` is called by a
  refiner after each change and returns its number of changes;
  `FinalImprover::run(mesh, spec, report)` is a pass over the whole mesh;
  `spec` is a pointer, null when there is no size target
  (`Remesher::refine`; decided 2026-11-17 instead of a made-up spec with
  an infinite length, so a future pass that needs a size cannot be
  misled).
  *Why:* the object answers what it can do (a declaration, not a type check),
  the pointer hands over the object to call, and a class with only one of
  the two roles still fits in the list. A whole-mesh pass after every split
  would make refinement O(n²), hence the local role. Defaults: Delaunay
  flips `local_and_final`, Laplacian smoothing `final_only`.
- **Improvers are given only to the `Mesher`** (decided 2026-11-12). Each
  improver's options carry a use mode, `ImproverUse { local_and_final,
  local_only, final_only }`. When the pipeline is assembled, the refiner
  takes as local improvers those it permits whose mode allows local use; the
  refiner's own options never name improvers. Local improvers run in the
  order of the improver list, and final passes run in that order too.
  *Why:* every improver is configured in one place, and the refiner learns
  about improvers only from the assembled pipeline.
- **Compatibility is matched by declared capabilities, not by name lists.**
  Improvers declare their effects (flips edges, moves nodes, makes the mesh
  Delaunay). Refiners declare what they tolerate and what they already
  maintain. *Why:* new improvers work without editing refiners, and the
  declarations double as documentation.
- **A mode that cannot be honoured is a warning, not an error** (decided
  2026-11-12). The improver is then not used in that role, for example
  "delaunay_flips: Ruppert already maintains Delaunay, so it runs only as a
  final pass". *Why:* a preference that cannot be met must not stop the
  program.
- **A final pass that cannot change anything is also a warning, and still
  runs** (accepted 2026-11-12), for example Delaunay flips right after
  Ruppert. Each role that cannot be honoured or is pointless gives one
  warning.
- **These warnings go into an extra `pipeline` report** (decided
  2026-11-12), the first entry of every run's reports. It holds the
  resolved plan (which improvers run locally, which as final passes) and
  the warnings found when the pipeline was built. *Why:* the warnings are
  found in the constructor, before any report exists, and an improver that
  never runs has no report of its own to carry them. The plan is written
  as one readable message ("plan: refiner ruppert; local: none; final:
  delaunay_flips, laplacian"); `plan()` gives it to code. Every report,
  this one included, keeps its structured fields (`stage`, `time`,
  `metrics`) beside the messages, so times can be summed later.
- **Reports:** each stage report has common fields (`stage`, `time`,
  `messages`) plus named numeric `metrics`. Failures throw; messages only
  mean "worked, but…". The `Mesher` checks "target met" itself, against the
  `MeshSpec`, on the final mesh. *Why:* only the `Mesher` sees the final
  mesh, after the improvers, and the refiner is optional.
- **Interface details (accepted 2026-11-12):** every `run` receives a
  `StageReport&` that the component fills with metrics and messages (the
  `Mesher` sets the stage name and the time). `improve_around` returns its
  number of changes. Components are held as `std::shared_ptr<const T>`, so
  one object (for example `DelaunayFlips`) can be both a local and a
  whole-mesh improver, and a pipeline can be reused across runs.
- **A facade with simple options comes later.**

## Mesh (output)

- **Contents:** `nodes`, `Triangle{nodes}`, `BoundaryEdge{nodes, region}`
  and `region_names`. A boundary edge is oriented so that the domain is on
  its left, and its region is an integer id into `region_names`.
- **Interior edges and connected pieces are computed when needed**, not
  stored.
- **The output mesh may carry its curve links** (decided 2026-11-12): an
  optional set of chains plus an optional `(chain, s₀, s₁)` link per
  boundary edge, empty for imported meshes. The `Remesher` then refines a
  `Mesh` directly with its links; linking to a geometry is needed only for
  meshes that have none. *Why:* refining an existing mesh (for example in a
  future adaptivity loop) must not require relinking every time.
- **Normals come from the straight edges.** *Why:* the mesh must be
  self-contained, because an imported mesh has no geometry behind it.

## Problem

- **Equation:** `a_xx u_xx + a_xy u_xy + a_yy u_yy + b_x u_x + b_y u_y + c u
  + m1 u_t + m2 u_tt = f`. Coefficients may depend on x, y and t.
- **Named parameters** (for example `k = 1000`) are supported.
- **Boundary conditions:** `α u + β_n u_n + β_τ u_τ + β_x u_x + β_y u_y +
  γ u_t = g`. Internally the derivative terms are combined into one vector,
  evaluated pointwise using the normal of the straight edge.
- **Conditions are classified structurally per boundary condition** (that
  is, per region) as Dirichlet, natural (plus oblique) or dynamic. Edges get
  their class through their region id. For now, boundary-ODE conditions
  (γ ≠ 0 with no spatial derivative) and tangential-only conditions (b·n ≡ 0)
  are rejected with a clear error.
- **Corners:** Dirichlet wins over a natural condition. Between two
  Dirichlet conditions, an integer `priority` on each condition decides: the
  higher one wins, equal priorities are averaged, and the report records
  every corner where the competing values differ.
- **Expressions are our own `Expression` value type, built from text**
  (decided 2026-11-12), for example `Expression("1 + k*x*y")`. SymEngine
  is used only inside `.cpp` files; a parse error throws when the
  expression is built. *Why:* SymEngine stays out of public headers, like
  Eigen, and mistakes show up where they are written.
- **`Problem`, the conditions, `MeshedProblem` and later assembly and
  solvers live in one library, `pde2d_fem`** (decided 2026-11-12), which
  depends on `pde2d_mesh`.
- **Initial conditions are part of `Problem`** as optional expressions.
  Which ones are required follows from the time order. A missing initial
  condition is an empty `std::optional`, not NaN.
- **Unusual initial conditions warn, never throw** (decided 2026-11-12):
  an initial condition the time order does not need, and one that depends
  on t, are reported as warnings. *Why:* an unneeded one is harmless (for
  example after switching a problem from transient to steady), and one in
  t can be intended, for example to start the same problem at t = 0 or at
  t = 5.
- **Initial values are evaluated on request at a start time t₀**
  (decided 2026-11-12): `MeshedProblem` keeps the compiled initial
  conditions and provides `initial_values(t0)`; the time integrator calls
  it with its own start time. *Why:* the same problem can then start at
  t = 0 or at t = 5, and t₀ belongs to the time-integration job, not to
  the problem.
- **At Dirichlet nodes the boundary data wins over the initial
  condition** (decided 2026-11-12): initial values there come from g/α,
  not from the initial condition (and the initial velocity from the time
  derivative of g/α when the time order is 2). *Why:* the initial field
  then agrees with the boundary values the solution is held to.
- **Expressions accept only an allow-list of functions and operators**
  (decided 2026-11-12): functions we have checked and tested; anything
  else is rejected with a clear error, and new ones are added one at a
  time with a test. *Why:* SymEngine evaluates some constructs wrongly
  (`Xor`) or unsafely between threads (`loggamma`), and a `Piecewise`
  without a final unconditional branch reads out of bounds. `atan2` is
  left out (SymEngine's parser simplifies `atan2(x, x)` to π/4 whatever
  the sign of x), and every constant part must evaluate to a finite real
  number (`exp(1000)` and `asin(2)` are rejected).
- **A condition is not a number** (accepted 2026-11-12, to revisit: GitHub
  issue #28): comparisons and And/Or/Not may appear only as `Piecewise`
  conditions, so `2*(x < 0)` is rejected.
- **Every derivative is taken in one place** (decided 2026-11-12): one
  private derivative function, used for the initial velocity, U_D′, U_D″
  and later ∇·A. It differentiates `abs` itself (d|u| = sign(u)·du, with
  sign written as a `Piecewise` and 0 at the kink), since SymEngine
  cannot. Its result passes the same allow-list check. Each function on
  the allow-list must also have a tested derivative. *Why:* no scattered
  special cases, and a function without a derivative fails at one known
  point.
- **Facts are stored per coefficient** (`zero`, `depends_on_x/y/t`).
  Summaries are aggregates of these facts over the input as written:
  `time_order`, `coefficients_depend_on_t` (a coefficient multiplying u or
  a derivative) and `data_depends_on_t` (f or g). They describe stage 1
  only, not the assembled system, and are computed by functions, never
  stored. "Zero" means zero as written.
- **Equations and conditions can be written as one line** (decided
  2026-11-18): `parse_equation(text)` and `parse_condition(text)`
  (`equation_text.hpp`) read lines such as `u_t - div(k*grad(u)) = 1`,
  `-∇·(k∇u) = f` or `k*u_n + h*u = h*u_out` into the existing `Equation`
  and `BoundaryCondition`; `equation_text` and `condition_text` print
  them back in the coefficient form. Operators by name (grad, div, lap,
  dot) and with nabla (∇, ∇·, ∇², Δ), vectors and matrices; in a
  condition the normal n and tangent tau. The divergence form is expanded
  into the coefficient form (b = −∇k), with the coefficients'
  derivatives taken by the one derivative function; assembly, the
  solvers and the estimators are unchanged. A line that is not linear in
  u, has a third derivative or a space derivative of u_t, or a condition
  whose derivatives depend on the normal in another way (an anisotropic
  flux (A∇u)·n) is refused with a message quoting the part. *Why:* the
  owner wants to type the equation as it is written on paper; the
  coefficients need symbolic derivatives, which only the library may
  take, and keeping `Equation` as it is leaves every reader of it
  unchanged. Alternatives not taken: a `text` member in `Equation` (two
  forms that readers would have to choose between) and a divergence form
  that assembly integrates directly (a much larger change).
- **Well-posedness checks are deferred.**

## Problem combined with a mesh

It is called `MeshedProblem` (the owner left the name to Claude,
2026-11-12). It holds `std::shared_ptr<const Mesh>`. Building it:

1. matches region names in both directions: a condition naming an unknown
   region is an error, and so is a mesh region without a condition;
2. classifies the conditions;
3. decides the Dirichlet nodes, corners included;
4. compiles the expressions into fast numeric functions;
5. compiles the initial conditions, which are evaluated at the nodes on
   request at a start time t₀ (`initial_values(t0)`);
6. merges the facts.

It stops just before matrix assembly.

**`Problem`, the boundary conditions and `MeshedProblem` are the next
slice** (decided 2026-11-12); assembly comes after it.

**Accepted from its implementation** (2026-11-12, pending the review):
"zero as written" means zero in SymEngine's parsed form (`x - x` is zero,
`sin(x)^2 + cos(x)^2 - 1` is not); parameters count as unknown constants
in the facts; `Expression` has an explicit constructor from text;
`Problem` and its parts are plain structs with conditions in a map by
region name, checked only when `MeshedProblem` is built; the classes are
Dirichlet, natural and dynamic plus an `is_oblique` flag, and Dirichlet
also beats a dynamic condition at a corner; a dynamic condition makes the
time order at least 1; corner values are compared exactly and reported as
"may differ" when they depend on t; `MeshedProblem` provides the free-node
numbering and the Dirichlet values; derivatives for assembly (∇·A, g_t,
g_tt) are compiled in the assembly slice; `CompiledExpression` is public;
values that are not finite throw; SymEngine is a required dependency. The
mesh is validated again by `MeshedProblem` (GitHub issue #24).

**Decided after the review** (2026-11-12): `initial_values(t0)` and
`initial_velocities(t0)` return an empty `std::optional` when the time
order has no such values, never throw for that; one value per mesh node
otherwise. When the initial condition and the boundary data g/α disagree
at Dirichlet nodes, the report warns, like corners: compared exactly at
each node when neither depends on t, and reported once as "may differ"
otherwise. Only `pi` and `E` are constants; an unneeded initial condition
is ignored, not compiled; a start time t₀ that is not finite throws.

**Accepted from the follow-up** (2026-11-12): a test may include a
private header when no public function reaches the helper yet and the
header has no heavy dependency (the derivative tests); the "may differ"
count of initial-condition mismatches counts every Dirichlet node, like
corners; u_t is compared with d/dt(g/α) as an expression, so it "may
differ" only if that derivative depends on t; boundary data that is not
finite at a Dirichlet node throws when `MeshedProblem` is built; only the
Dirichlet region that wins a node is used; a mismatch message lists three
example nodes, and values that do not depend on t are evaluated at t = 0.
A `Piecewise` that jumps in t is differentiated branch by branch (the jump
contributes nothing).

## Assembly

Discussed and agreed on 2026-11-12. Of the architecture, only the items
marked "architecture" below are decided.

- **Architecture: assembly is a separate step.** An `Assembler` object has
  `assemble(meshed_problem, t, previous)`, where `previous` is an optional
  earlier result to reuse. It returns the assembled problem: the matrices
  and the facts, partly carried over from `MeshedProblem` and partly
  computed by assembly. *Why:* the problem description stays separate from
  its discretized form, and a result can be reused when only t changes.
- **Architecture: the first assembly slice has no options** (decided
  2026-11-12). Choices such as colouring or chunked assembly and other
  Dirichlet handlings may become options later.
- **Architecture: assembly fills a `StageReport`**, like the meshing
  stages (time, number of nonzeros, and so on).
- **Architecture: each role keeps a constant part and a total** (decided
  2026-11-12). For each operator of M₂U″ + B·U′ + A·U = b (and the
  right-hand side), the terms whose coefficients do not depend on t are
  assembled once into a constant part. At each t the total is the constant
  part copied, plus the t-dependent terms added on top. If nothing depends
  on t, the constant part is the total. *Why:* only coefficients that
  depend on t are evaluated again (grouping everything would recompute
  every term), while memory stays at most two arrays per role on one
  shared pattern (storing every term would need up to eight or more).
- **Architecture: the result holds `std::shared_ptr<const MeshedProblem>`**
  (decided 2026-11-12). It keeps the problem and its mesh alive, and
  `assemble` reuses `previous` only if it was built from the same problem
  object (identity, not equal contents); otherwise it throws
  `std::invalid_argument`.
- **Architecture: matrices in public headers are our own `CsrMatrix`**
  (row offsets, columns, values; decided 2026-11-12), never Eigen types.
  Assembly fills it through the precomputed pattern without Eigen; solvers
  view the same arrays with `Eigen::Map` inside their `.cpp` files, without
  copying. An opt-in adapter header (`as_eigen`) may be added when someone
  needs it, not now. *Why:* Eigen stays out of public headers (compile
  time, and our interface does not depend on Eigen or its version), and
  the solver library can be replaced without changing the public API.
- **Architecture: the thread count will be a constructor option of the
  `Assembler`, default 1** (decided 2026-11-12), added in a later slice.
  The serial path already goes colour by colour, so the thread count never
  changes the result.
- **Parallel assembly and iterative solvers are built now** (decided
  2026-11-17). Threads come from OpenMP (Eigen's sparse matrix-vector
  product uses it too). The thread count is an option of each component
  that uses threads: `AssemblerOptions{.threads}`, and `.threads` in the
  options of `ConjugateGradient` and `BiCgStab`; default 1, 0 means all
  hardware threads. `StaticSolver` and `Integrator` take `AssemblerOptions`
  in their constructors, since they create the assembler. *Why:* a setting
  read by one component is that component's option; no global state, and
  two runs can use different counts. Assembly and the iterative solvers
  give the same results for every thread count (colours in a fixed order;
  each row of a matrix-vector product summed by one thread), and tests
  check it bit for bit.
- **Details of the parallel slice, accepted 2026-11-17:** `threads` is a
  `std::size_t` from 0 to 1024 (more throws: it catches a negative count
  converted to unsigned); 0 means `omp_get_num_procs()` (the processors
  this process may use, not `OMP_NUM_THREADS`), capped at 1024. An
  exception inside a colour is rethrown after the colour, the one of the
  lowest element index. One parallel loop per colour, never more threads
  than the colour has elements. The solvers set the calling thread's own
  OpenMP count for Eigen (not the global `Eigen::setNbThreads`), the
  direct ones 1. Reports: assembly `threads` (asked) and `threads_used`
  (ran), `static_solver` `linear_threads`, `integrator`
  `assembly_threads` and `linear_threads`; `LinearSolveResult::threads`.
  Bad counts throw in the `StaticSolver` and `Integrator` constructors.
  OpenMP is a public requirement of `pde2d_solve` (a program that includes
  Eigen must match it; a review finding). The bit-for-bit guarantee
  assumes the default rounding mode. Building the sparsity pattern and
  the colours stays serial (it runs once per mesh): GitHub issue #37.
- **Parallel direct solvers come from SuiteSparse** (decided 2026-11-17;
  the owner left the library to the agents): new components named after
  the method, not the library (for example a supernodal Cholesky from
  CHOLMOD and a multifrontal LU from ParU), with `.threads` like the
  iterative solvers; the Eigen direct solvers stay. SuiteSparse stays in
  `.cpp` files, like Eigen. Their results may differ in the last digits
  between thread counts (and between runs, if the library schedules
  dynamically); the owner accepts that for these solvers, and their
  header comments say so. Their speed comes from a threaded BLAS
  (OpenBLAS).

- **Direct solvers check the backward error, not the relative residual**
  (decided 2026-11-17, after a review finding: a correct solve of a
  problem with a coefficient contrast of 10^6 was refused, because b − Ax
  computed in doubles has a rounding error of about ε‖A‖‖x‖): the check
  against `residual_tolerance` uses ‖b − Ax‖ / (‖A‖‖x‖ + ‖b‖), for
  `SupernodalCholesky`, `MultifrontalLu` and `SimplicialLdlt`; the result
  also reports it (`backward_error`), and `residual` stays the relative
  residual.
- **Direct solvers refuse only exact singularity** (decided 2026-11-17,
  after a review finding: the rule "smallest pivot below n·ε times the
  largest" refused a valid, very ill-conditioned problem): they throw on
  an exact zero pivot and the Cholesky when the matrix is not positive
  definite; the ratio of the smallest to the largest pivot is reported,
  not judged. A pure Neumann problem then gives one of its solutions
  without an error, as the Eigen solvers do (GitHub issue #32).
- **The Cholesky factors −A when A is negative definite, with a warning**
  (decided 2026-11-17): when every diagonal entry is negative (Δu = f
  written with a_xx = +1), `SupernodalCholesky` factors −A and solves
  with −b, which is exact, and says so as a warning ("worked, but…"); a
  matrix with diagonal entries of both signs is indefinite and refused as
  now. The licence question (CHOLMOD's supernodal module and ParU are GPL,
  so distributed programs that link them are GPL-3) is postponed: GitHub
  issue #38. Reusing the symbolic factorization when only the values
  change: GitHub issue #39.
- **A tiny pivot ratio is a warning; the integrator reports the worst
  solve** (decided 2026-11-18): a direct solver whose `pivot_ratio` is
  below 1e-10 adds a warning to `LinearSolveResult::messages` (the matrix
  is nearly singular, so the answer may have few correct digits); the
  answer is still returned. The integrator's report shows the largest
  `backward_error` and the smallest measured `pivot_ratio` over all its
  solves. *Why:* "refuse only exact singularity" otherwise returns a
  hopeless answer (one correct digit) without any sign but a number.
- **Known limits of the SuiteSparse solvers, documented** (2026-11-17):
  OpenBLAS's thread count is one setting for the whole process, so a
  program that changes it during one of these solves can have its setting
  overwritten; Ubuntu's ParU replaces the global `operator new` of every
  program that links it (an installed `std::new_handler` is skipped when
  memory runs out).
- **Weak form.** The second-order part is written as A : ∇²u with
  A = [[a_xx, a_xy/2], [a_xy/2, a_yy]] and integrated by parts once:
  ∫ (A:∇²u) v = −∫ ∇v·A∇u − ∫ (∇·A)·∇u v + ∮ v (A∇u)·n. The term ∇·A
  needs derivatives of the coefficients, which symbolic input provides:
  ∇·A = (∂ₓa_xx + ½∂ᵧa_xy, ½∂ₓa_xy + ∂ᵧa_yy). The formula holds where the
  coefficients are smooth in x and y; a `Piecewise` coefficient that jumps
  in space is not handled correctly (GitHub issue #33).
- **Matrices.** Interior: stiffness K (∫ ∇φᵢ·A∇φⱼ), convection C
  (∫ (b − ∇·A)·∇φⱼ φᵢ), reaction R (c), masses M₁, M₂ (m1, m2), load F (f).
  Boundary: Robin mass S, tangential block T, dynamic mass D, boundary load
  G. The system is M₂U″ + (M₁ − D)U′ + (−K + C + R − S + T)U = F − G. C and
  T are not symmetric.
- **Natural conditions** are substituted into the boundary term. On each
  straight edge, β_x and β_y are split into normal and tangential parts,
  giving β̂_n and β̂_τ; the condition is solved for u_n (this needs β̂_n ≠ 0)
  and substituted into (A∇u)·n = κ_n u_n + κ_τ u_τ, with κ_n = nᵀAn and
  κ_τ = nᵀAτ. u_τ is kept: it is the derivative along the edge, (U_b − U_a)/L
  for P1. *Consequence:* our Neumann condition is u_n = g, not the conormal
  flux, so with an anisotropic A even u_n = 0 contributes a T block.
- **Dirichlet values are imposed by elimination (lifting)**, during the
  scattering of each block, not afterwards: an entry whose row is a
  Dirichlet node is skipped; an entry whose column is a Dirichlet node goes
  to a separate sparse matrix A_FD; every other entry goes to A_FF. The
  right-hand side is b_F − A_FD U_D(t), plus −B_FD U_D′ − M₂,FD U_D″ for time
  derivatives, where B = M₁ − D is the whole damping operator (corrected
  2026-11-13: the boundary matrix D was missing from the lifting; found by
  an independent check). The derivatives come from differentiating g / α
  symbolically. A static solve uses only b_F − A_FD U_D(t). *Why:*
  elimination is exact and standard (deal.II, FEniCS); keeping A_FD lets a
  time-dependent g cost one matrix–vector product per step. Edge blocks
  touching a Dirichlet corner are routed the same way. The same routing
  applies to every matrix and to the load vector.
- **The eliminated system is shrunk to the free nodes** (decided
  2026-11-12): the unknowns are renumbered to free nodes only, with a map
  back to mesh nodes for the full solution. *Why:* with full size and
  identity rows, the mass matrices have empty Dirichlet rows, so M becomes
  singular and every time stepper (explicit ones need M⁻¹) would have to
  special-case those rows. Shrunk, time integration no longer sees those
  artificial rows. The mass matrix can still be singular for another
  reason: with m1 = 0 and a boundary γ ≠ 0, only boundary nodes carry a
  time derivative (noted 2026-11-13; implicit methods must cope with it).
- **Each stage owns its facts** (clarified 2026-11-12). Facts of a later
  stage are computed by that stage from what it actually builds, never
  read off an earlier stage's summary. In particular, whether the **final
  right-hand side** changes in time is a fact computed by assembly, from
  every term that feeds it: f, the Dirichlet values g/α, the lifting term
  A_FD·U_D (so also the left-hand side coefficients), and the natural
  boundary load (g, β and A on the boundary). A time-dependent Dirichlet
  value makes it time-dependent even when f is not. The fact means "may
  depend on t": it is decided from which terms depend on t, so a rare
  exact cancellation is not noticed.
- **Matrices are rebuilt per matrix, from the facts:** a matrix is
  recomputed in time only if a coefficient it uses depends on t (K only if
  A does, M₁ only if m1 does, and so on).
- **Element blocks are computed in full:** 3×3 per triangle, 2×2 per
  boundary edge, all entries, no symmetric shortcut. *Why:* correct for the
  non-symmetric matrices too; the cost is in quadrature, not in additions.
- **One pass over the triangles computes the blocks of every matrix**, so
  the area, gradients and coefficient values at quadrature points are
  computed once per triangle.
- **The sparsity pattern is precomputed from the mesh** and reused for
  every matrix and rebuild (one for A_FF, one for A_FD). It is built by one
  loop over the triangles (decided 2026-11-13): every pair of a triangle's
  nodes (p, q), including p = q, adds column q to row p when p is free, in
  A_FF if q is free and in A_FD if q is a Dirichlet node; then each row is
  sorted and duplicates are removed. Boundary edges add nothing: both nodes
  of an edge already share a triangle. An entry finds its slot by binary
  search in its row. Column k of A_FD is `dirichlet_nodes()[k]`.
- **The pattern and the colours live in the assembled result**, shared by
  its matrices and carried over through `previous` (decided 2026-11-13).
- **The Dirichlet tables stay in `MeshedProblem`** (confirmed 2026-11-13,
  after comparing with the owner's Poisson project, which keeps them in its
  assembled problem). *Why:* here `assemble` runs at every time step while
  the tables never depend on t, and `initial_values` needs them before any
  assembly. A refined mesh gets a new `MeshedProblem`, which recomputes
  them from the region tags.
- **Parallel assembly uses colouring:** triangles are grouped so that no two
  in a colour share a node; colours run one after another, with no atomics.
  *Why:* atomics make results depend on thread timing (floating-point
  addition is not associative) and are slow under contention; storing all
  element blocks first costs about 2.5× the matrix in memory. The colouring
  must itself be deterministic: greedy in triangle index order, each
  triangle taking the smallest colour not used by a triangle that shares a
  node with it; boundary edges are coloured the same way in their own pass
  (decided 2026-11-13). The method is fixed for now; making it a choice is
  GitHub issue #31.
- **Quadrature is internal and fixed at compile time**, never a user input:
  a 3-point rule on triangles (exact for degree 2) and 2-point
  Gauss–Legendre on edges. *Why:* it keeps P1's convergence rates and is
  exact for constant-coefficient mass matrices.
- **One table of quadrature rules, the degree chosen per computation**
  (decided 2026-11-17, for the error estimators): every rule lives in the
  public `fem/quadrature.hpp` (`triangle_rule(exact_degree)`,
  `edge_rule(exact_degree)`: the cheapest rule exact for that degree;
  triangles up to 5, edges up to 9). Each computation names the degree it
  needs in its code (assembly 2 on triangles, 3 on edges, unchanged);
  there is no global quadrature setting. *Why:* different integrals need
  different degrees (one setting wastes work or integrates too little, and
  would change assembly), and one table means one set of weights to get
  right. Whether an estimator offers its degree as an option is decided
  with the estimators. The adaptive Simpson integration of curve lengths
  in geometry is not a quadrature rule and stays there.
- **Eight choices of the assembly implementation, accepted 2026-11-14:**
  1. `Assembler::assemble(meshed_problem, t, previous)` takes `previous` as
     a plain pointer to const, null by default. It is only read during the
     call; the result does not keep it.
  2. The divergence of A is taken symbolically and compiled in the
     `MeshedProblem` constructor (`divergence_of_a()`), so assembly does no
     symbolic work.
  3. Matrix indices are signed (`std::ptrdiff_t`).
  4. `SparsityPattern` and `CsrMatrix` are plain structs with public
     members; a `CsrMatrix` holds its pattern through a shared pointer.
  5. An operator that is zero is still stored, as zeros on the shared
     pattern, with `is_zero` set. *Why:* every operator then has the same
     shape and the solver needs no special case.
  6. With `previous`, what is rebuilt is decided per term and per boundary
     region from the facts, not per whole matrix.
  7. The tangential boundary matrix T counts as non-zero whenever A is: it
     is not tested for cancelling to zero. *Replaced 2026-11-15* by the
     zero test for T under "Solvers and time integration".
  8. `dirichlet_rate` and `dirichlet_acceleration` are compiled only when
     the time order needs them (the rate for 1 or 2, the acceleration for
     2) and throw `std::invalid_argument` below that. *Why:* asking for
     them on a lower time order is a mistake in the caller, and a static
     problem must not be rejected because of a derivative it never uses.
- **`assemble` takes a `max_time_order`** (decided 2026-11-15, GitHub
  issue #35; built 2026-11-15): `assemble(meshed_problem, t, previous,
  max_time_order = 2)`. 2 builds everything; 1 skips M₂ and U_D″; 0
  builds only A (K, C, R, S, T) and the load, with the lifting load −
  A_FD·U_D, and evaluates only U_D. Skipped terms are stored as zero
  operators, so `time_order()` is at most `max_time_order`; terms that a
  lower-order `previous` did not build are built fresh. Other values
  throw `std::invalid_argument`. `StaticSolver` uses order 0 and
  `right_hand_side()` as it is. *Why:* a static solve must not evaluate
  boundary-data derivatives it drops (d/dt √t at t = 0), and it removes
  the static solver's own right-hand side.

## Solvers and time integration

Discussed in a side chat on 2026-11-12; the first slice was built on
2026-11-14.

- **Configuration in the constructor, the job in the call**, as for the
  `Mesher`: `Integrator(method, step_controller, linear_solver)` with
  `integrate(meshed_problem, t0, t1, output, initial)`, and
  `StaticSolver(linear_solver)` with `solve(meshed_problem, t = 0)`. Both
  assemble the problem themselves (reusing `previous` results).
- **The time method is a component, not an order number** (implicit Euler,
  Crank–Nicolson and BDF2 differ in more than their order). Implicit
  methods first: explicit ones need M⁻¹ and steps that shrink like h².
  A method declares which time orders it supports, checked against the
  problem's `time_order` before integrating, like the triangulators'
  declared hole support.
- **The problem never changes during integration.** The current state is
  a vector inside the integrator, read once from `initial_values(t0)`.
  A different starting state (for example a steady solution) is passed as
  an optional `initial` argument of `integrate`. A solver returns a
  `Solution` (node values plus their mesh).
- **The static solver drops the time terms** and solves A·U = b at the
  given t.
- **Linear solvers have a prepare step and a solve step**, like Eigen's
  `compute` and `solve`: a configured `LinearSolver` is `const` and
  reusable; `prepare(matrix)` returns a prepared solver tied to that
  matrix, which solves many right-hand sides. The integrator prepares
  again only when the operator or dt changes, which the facts tell it.
- **Step control is a component:** a `StepController` interface; the
  first slice has only `FixedStep{.dt}` (dt is configuration of the step
  control). Error-controlled adaptive stepping comes later without
  changing `integrate`; its options are tolerances, not step sizes.
- **Step controllers, time methods and linear solvers are virtual
  interfaces**, each with its own options in its constructor. *Why:* their
  kinds will keep growing, including the owner's own smart ones, and a new
  kind must not require editing library code (a variant suits closed sets
  of data kinds, like curves).
- **Reports stream:** each step's small record (t, dt, accepted, linear
  iterations, residual, time taken) updates one running summary and is
  then dropped; the summary becomes one `StageReport` at the end.
- **Output options belong in the call:** `keep_step_history` (off by
  default) keeps the small records; solution snapshots are taken at exact
  `snapshot_times` (the stepper lands on them) and/or every n steps; the
  final solution is always returned.
- **Second order in time is handled by methods that support it** (decided
  2026-11-13), not by refusing such problems or rewriting them as a
  first-order system. The first one is Newmark with average acceleration
  (β = 1/4, γ = 1/2): second-order accurate, one linear solve per step of
  the same size, no symmetry needed. Generalized-α may follow if damping
  of high frequencies is wanted.
- **First time methods** (decided 2026-11-13): `ImplicitEuler` and
  `ImplicitMidpoint` for time order 1, `Newmark` for time order 2, each a
  component behind the `TimeMethod` interface. The midpoint method
  evaluates the matrices and the right-hand side once, at the middle of
  the step; it is named for what it does, since "Crank–Nicolson" has two
  different meanings when the matrices change in time. The equation
  integrated is M(t)·U′ + A(t)·U = b(t) as assembled (no M′ term).
- **First linear solvers** (decided 2026-11-13): `SparseLu` (direct) and
  `BiCgStab` (iterative), both for any matrix, behind the `LinearSolver`
  interface. Conjugate gradients came with a symmetry fact from assembly
  (see below): the matrix is not symmetric in general and −K is
  negative definite.
- **A solve has an intermediate result and a separate building step**
  (decided 2026-11-13, as in the owner's Poisson project): the linear
  solver returns a `LinearSolveResult` (the vector over the free unknowns,
  iterations, residual); the plain function `build_solution(meshed_problem,
  free_values, t)` turns it into a `Solution` (a value for every mesh
  node, the mesh, the time), filling Dirichlet nodes from
  `dirichlet_value`. The integrator works on the short vector and builds a
  `Solution` only for snapshots and at the end.
- **Solvers live in their own library, `pde2d_solve`**, which depends on
  `pde2d_fem`; Eigen is used only inside its `.cpp` files.
- **Continuing a run:** `integrate(meshed, t1, t2, output, previous.final)`
  starts from an earlier result; no new problem object is made.
- **Choices of the first solver implementation, accepted 2026-11-15:**
  `prepare` takes `shared_ptr<const CsrMatrix>` and keeps it; `residual`
  is the relative residual ‖b − A·x‖/‖b‖ measured after every solve;
  `BiCgStab` defaults are tolerance 1e-10 and at most 1000 iterations,
  with a fixed Jacobi preconditioner and x = 0 as start;
  `FixedStep(FixedStepOptions{.dt})`; a method describes its step matrix
  as weights (mass·M₂ + damping·B + stiffness·A) and an evaluation
  fraction; `Solution` also carries `velocities` (filled for time order
  2, so a wave run can be continued); `initial` is refused on another
  mesh, with the wrong size, non-finite values or missing velocities;
  step times are anchor + k·dt and land on snapshot times and t1;
  `snapshot_times` strictly increasing inside [t0, t1], `snapshot_every =
  0` means never, one snapshot per step at most; the integrator's report
  is the stage `"integrator"` with step, dt, iteration, residual,
  assembly, prepare and snapshot metrics.
- **No fallbacks between methods** (decided 2026-11-15): a method or
  linear solver that cannot handle a problem throws with the reason; it
  never switches to another one. Choosing a method automatically belongs
  to a planner, later.
- **`Newmark` needs an invertible M₂ at t0** for its starting
  acceleration and throws otherwise, naming the cause and the methods
  that need no acceleration (decided 2026-11-15). The user never gives an
  initial acceleration.
- **`ImplicitEuler` and `ImplicitMidpoint` also integrate time order 2**
  (decided 2026-11-15), stepping U′ = V, M₂·V′ + B·V + A·U = rhs as a
  pair. Eliminating V₁ leaves one system of the usual size: weights
  (1/dt², 1/dt, 1) for Euler and (2/dt², 1/dt, ½) for midpoint. They need
  no acceleration, so they work when M₂ is singular or zero at some time
  (m2 = sin(t)); with M₂ = 0 the midpoint step is the first-order
  midpoint step. *Why:* Newmark cannot start then.
- **A time method declares the set of time orders it supports** (0, 1
  and/or 2; decided 2026-11-15), checked against the assembled
  `time_order()`: `ImplicitEuler` {0, 1, 2} (order 0 is a static solve at
  each step time), `ImplicitMidpoint` {1, 2} (at order 0 it would
  oscillate: U₁ = 2A⁻¹r − U₀), `Newmark` {2}. A problem whose assembled
  order is below what the user wrote (m1 = 0 and a boundary u_t term on a
  problem with A = 0, where the boundary term vanishes) is therefore
  refused only by methods that do not declare that order, with a message
  saying why.
- **Linear solvers declare requirements; matrices carry properties**
  (decided 2026-11-15). Assembly gives each operator a `symmetric` fact
  from its terms (M₁, M₂, D, K, R, S are symmetric; C and T are not unless
  zero). The step matrix is symmetric when every operator with a non-zero
  weight is. The integrator and the static solver check a solver's
  requirements against these facts before preparing and throw
  `std::invalid_argument` on a mismatch. Positive definiteness is not
  known from facts: a solver that needs it detects its failure and throws
  `std::runtime_error`. New solvers: `ConjugateGradient` and
  `SimplicialLdlt`, both requiring a symmetric matrix.
- **T is recognised as zero from the facts** (decided 2026-11-15,
  replacing the assembly choice "T counts as non-zero whenever A is"): on
  a natural or dynamic region, r = κ_τ − q·β̂_τ is zero when A is
  isotropic or zero (a_xy is zero and a_xx − a_yy simplifies to zero) and
  β_τ, β_x and β_y are zero. *Why:* otherwise every problem with
  diffusion would look non-symmetric and CG could never be used.

- **Choices of solver slice 2 accepted 2026-11-15** (as reported by its
  implementation): `Operator::symmetric` false unless assembly proves it;
  `MeshedProblem::a_is_isotropic()` public, distinct parameters count as
  unknown; time-order capabilities are three bools with `supports(order)`;
  every linear solver must declare `requirements()`; the requirement check
  is private and runs before every prepare; at order 0 Euler's starting
  solve at t0 is not counted in `prepares`; Newmark's failure is a
  `std::runtime_error`; the assembly report gains `max_time_order`;
  `ConjugateGradientOptions` is its own struct; the β̂ₙ ≠ 0 check runs on
  every natural or dynamic edge where A is not zero, even when all its
  terms vanish. `SimplicialLdlt` checks its relative residual against
  `SimplicialLdltOptions::residual_tolerance` (1e-8) and throws above it:
  a wrong answer is an error, not a warning.
- **Two kinds of facts per matrix** (decided 2026-11-15): facts from the
  formulas hold for every t (`Operator::is_zero` is renamed
  `always_zero`, `symmetric` becomes `always_symmetric`); facts from the
  numbers hold at the assembled time only (`zero_now`, `symmetric_now`,
  computed by assembly from the values; the same checks are plain
  functions on `CsrMatrix` for a step matrix). The time order, reuse and
  "build or skip" use only the formula facts: the time order must be one
  answer for the whole run (m2 = sin(t) is order 2 even at t = 0). A
  solver's symmetry requirement is met when the matrix is always
  symmetric or symmetric now (anisotropic diagonal A on a mesh with only
  axis-aligned boundary edges).
- **A time method declares what its start reads at t0** (decided
  2026-11-15): `enum class StartNeeds { nothing, static_part,
  whole_problem }`, from `TimeMethod::start_needs(time_order)`. Newmark:
  `whole_problem`; ImplicitEuler: `static_part` at order 0, otherwise
  `nothing`; ImplicitMidpoint: `nothing`. The integrator gets the time
  order from the formula facts without evaluating anything
  (`Assembler::time_order(meshed_problem)`), and assembles at t0 only what
  the start needs. Newmark stays as it is: it throws when M₂ at t0 is zero
  now or singular, naming the methods that need no acceleration.

- **The integrator starts from `MeshedProblem::initial_free_state(t0)`**
  (decided 2026-11-15, after a review finding): one function returning u
  and, for time order 2, u_t at the free nodes only, evaluating no
  boundary data. *Why:* reading the initial velocities at every node also
  evaluated the Dirichlet rate at t0, so u = √t on the boundary stopped a
  wave run whose method never uses it. The full-node `initial_values` and
  `initial_velocities` stay for other uses.
- **Accepted 2026-11-17 (slice 3 details):** the values of a step matrix
  are compared for symmetry only when the solver requires symmetry and the
  formula facts cannot promise it (one pass over the matrix per prepare);
  Newmark's start does nothing when there are no free unknowns (an empty
  M₂ is not "zero at t0").

## Adaptivity: marking and refinement

Asked for by the owner on 2026-11-15. Of the three parts of adaptive
refinement (estimate, mark, refine), the error estimation stays the
owner's; marking and refinement are built for it. Mesh refinement only: no
coarsening, moving nodes or enrichment for now.

- **Error indicators are one number per triangle** (decided 2026-11-15),
  the same length as `mesh.triangles`. Marking and refinement act on
  triangles.
- **A separate library, `pde2d_adapt`**, depending on `pde2d_mesh` only
  (since 2026-11-17 also on solve and fem, for the estimators and the loop).
- **Marking strategies are components behind one interface**: maximum
  (θ · the largest indicator), Dörfler or bulk (the smallest set holding a
  fraction θ of the total squared error), fixed fraction (a fraction of
  the triangles with the largest indicators) and equidistribution (above
  tol/√N). The details were left to the implementation, for the owner to
  accept.
- **Refinement methods are components behind one interface**: newest
  vertex bisection, and regular (red) refinement with bisection closure.
  Both keep the mesh conforming. The details were left to the
  implementation, for the owner to accept.
- **Red-green-blue warns at curved ears** (decided 2026-11-15;
  *replaced the same day* by the mesh properties below, which judge the
  final mesh instead of one method's triangles).
- **Mesh properties of the final mesh** (decided 2026-11-15): one plain
  function measures a mesh (counts, smallest and largest angle, shortest
  and longest edge, how many triangles have an angle below a warning
  angle), the same way facts describe a problem. The Mesher and the
  Remesher attach it to their result after the last improver, and the
  pipeline report warns when triangles fall below the warning angle (an
  option of the Mesher and Remesher, default 10°). Nothing is refused.
  *Why:* a warning about one method's triangles is inflexible, since an
  improver may fix them; only the final mesh tells. Reports, guarantees
  (`target_met`) and warnings must stay consistent: one place computes
  each property. This settles the open question on quality guarantees:
  measure the final mesh and warn.
- **Refinement and meshing share one set of refiners** (decided
  2026-11-15). Every refiner refines marked triangles. Bisection methods
  have two layers: a shared core that bisects marked edges and closes the
  mesh so no node hangs, and a per-method rule turning a marked triangle
  into edges (longest-edge bisection: its longest edge; newest vertex
  bisection: its refinement edge; red-green-blue: all three). "Refine
  until every edge is at most h" is a loop around it that marks the
  triangles with a longer edge. Newest vertex bisection and red-green-blue
  become refiners in the mesh module next to longest-edge bisection and
  Ruppert; the marking strategies stay in `pde2d_adapt`.
- **The Remesher drives adaptive refinement** (decided 2026-11-15):
  `Remesher::refine(mesh, marked_triangles)` next to `run(mesh, spec)`,
  with the same improvers and pipeline report. Its result carries parents
  (old triangle of each new triangle; old node or old edge of each new
  node) only when no component made them untrue; an improver that flips
  or moves nodes drops them, and a solution is then transferred by point
  location. Keeping the parents that stay valid is GitHub issue #36.
- **Details of the refiners, accepted 2026-11-17:** newest vertex
  bisection splits a marked triangle once (its refinement edge, two
  children); red-green-blue splits it into four. A red split that a curve
  point would fold is done with the bisection pattern instead and counted
  as `red_not_possible` (both patterns belong to red-green-blue; no
  fallback to another method). A cycle of refinement edges is broken by
  one split out of order, counted as `incompatible_splits`.
  `keeps_parents` is declared by each refiner and improver.
  `Remesher::refine` returns its own `RefinementResult` (mesh, reports,
  properties, optional parents; no `target_met`), throws without a
  refiner, and validates its output. `MeshResult::longest_edge` is gone:
  it is read from `properties`. The warning angle lies in [0°, 60°]
  (default 10°, 0 never warns). Newest vertex bisection and red-green-blue
  have options with `max_points`. With nothing marked, `refine` returns
  the mesh unchanged and the final passes do not run.
- **Newest vertex bisection and red-green-blue set the refinement edges
  in `run` only** (decided 2026-11-17): `run(spec)` first makes each
  triangle's longest edge its refinement edge, then refines; meshing
  starts from a vertex order nobody chose for bisection (a triangulator,
  Ruppert, flips), and from it the angles fall to about half of the
  starting mesh's smallest (L-shape, h = 0.12: 2.9° instead of 6.0°).
  `refine(marked)` never does: between cycles the stored vertex order is
  the method's memory (who sets the order on a starting mesh: see "The
  library delivers the refinement-edge order" below).
  Equally long edges are ordered by their node indices, the order of the
  bisection core, which both triangles of an edge agree on.
- **Error estimation and the adaptive loop are built too** (decided
  2026-11-17; on 2026-11-15 the estimation had stayed the owner's). The
  work goes in slices: (1) the three estimators and the adaptive loop for
  static problems; (2) the loop for time-dependent problems; (3) adaptive
  time steps. The details are left to the implementation, for the owner
  to accept.
- **Everything adaptive stays in `pde2d_adapt`** (decided 2026-11-17):
  estimators, marking and the adaptive loop. It may now depend on solve
  and fem (and below). Refiners stay in the mesh module, shared with
  meshing. *Why:* "adapt" covers all of the vision's adaptivity (later
  coarsening, moving nodes, revising the solver), not only mesh
  refinement.
- **Where a setting goes** (decided 2026-11-17): a setting read by one
  component only is that component's option (constructor); a setting of
  the loop a driver runs, the same for every job, is the driver's option
  (constructor); what describes one job (the problem, the time interval,
  the target) is an argument of the call. This is how it already is:
  `dt` in `FixedStepOptions`, the solver tolerance in `BiCgStabOptions`,
  t0 and t1 and the snapshots in `integrate(...)`, `MeshSpec` in
  `Mesher::run`.
- **An adaptive loop is a driver above the solvers** (decided
  2026-11-17). A general driver (name left to the implementation) takes a
  solver and one or more adaptations; mesh refinement is one adaptation,
  itself a component made of an estimator, a marker and the `Remesher`,
  each with its own options (θ of the marker, ...). Later kinds of
  adaptation plug in the same way. The job's target is an argument of
  the call: a tolerance for the estimated error, a maximum number of
  cycles, a node budget; for a time-dependent run also when to adapt:
  after a time interval and/or after a number of steps, whichever comes
  first (both optional, like a `LengthBound`). *Why:* `StaticSolver` and
  `Integrator` work on one `MeshedProblem`; a new mesh makes a new one,
  and the integrator already continues a run from a given state.
- **One estimator component serves static and time-dependent problems**
  (decided 2026-11-17): residual, Kelly and recovery (Zienkiewicz–Zhu),
  one indicator per triangle. One method; its input is the solution and,
  when the problem has time terms, u_t (and u_tt for time order 2). The
  residual estimator adds the time terms to its residual (and throws when
  a problem with time terms comes without them); Kelly and recovery ignore
  them. *Why:* the algorithm is the same in both cases; only the residual
  gains terms. The time error is not an estimator's job.
- **Adaptive time steps are another `StepController`** (decided
  2026-11-17), controlled by an estimate of each step's time error, with
  rejected steps; the `StepController` interface is extended when it is
  built.
- **Details of slice 1 (estimators and the static loop), accepted
  2026-11-17:** residual estimator η_T² = h_T²‖r‖² + Σ interior ½h_E‖J‖²
  + Σ natural/dynamic h_E‖R‖² with h the longest edge, r = f − b·∇u_h −
  c u_h − m1 u_t − m2 u_tt (the A part cancels for P1), R = κ_n/β̂_n times
  the condition's residual, not scaled by A; u_t is required when m1 ≠ 0
  or a region is dynamic, u_tt when m2 ≠ 0, from the estimator's own
  terms; total = √(Σ η_T²); Kelly = the residual estimator's edge terms;
  recovery = area-weighted nodal average of the gradients, in the energy
  norm of A (`.norm = energy`, or `gradient`; an indefinite or zero A
  throws). An `Adaptation` has two steps, `estimate` and `adapt`, so the
  loop tests its tolerance before refining; the loop's estimate comes from
  the first adaptation that declares it estimates the error, and at most
  one adaptation may change the mesh until solution transfer exists. The
  target: at least one of tolerance (absolute and/or relative to the first
  estimate, the smaller applies), `max_cycles`, `max_nodes`; the node
  budget refines, then discards a mesh over it and stops; the loop also
  stops with `nothing_changed` when nothing was marked or the refinement
  left the mesh unchanged. The loop takes a `MeshedProblem` and builds a
  new one per cycle; a static solution passes u_t = u_tt = 0; the result
  keeps every cycle. A bad indicator from an estimator is a
  `std::runtime_error`, a solution on another mesh `std::invalid_argument`.
- **Estimator quadrature degrees are options** (decided 2026-11-17), with
  the rules from the same table (`fem/quadrature.hpp`): residual and Kelly
  `triangle_degree` 4 and `edge_degree` 6 (the jumps use the edge degree
  too), recovery `triangle_degree` 3; a degree the table does not have
  throws in the constructor. *Why:* a setting read by one component only
  is that component's option. The defaults are exact for coefficients and
  data of degree at most 1 (with β̂_n constant).
- **Solution transfer is interpolation** (decided 2026-11-17): the new
  mesh's nodal values are the old u_h at the new nodes (old nodes keep
  theirs, a bisection midpoint gets the average of its edge's ends, a
  Dirichlet node gets g), through the parents when the refinement kept
  them and by point location otherwise; velocities the same way. Built
  with the time-dependent loop (slice 2).
- **The library delivers the refinement-edge order; the user never calls
  `set_longest_edges_as_refinement_edges`** (decided 2026-11-17, replacing
  "the user calls it once"): `Mesher::run` and `Remesher::run(spec)` end
  by making every triangle's longest edge its refinement edge, and Gmsh
  import does it too. `Remesher::refine` also ends with it, but only for
  the triangles whose order is not a bisection's (made or changed by
  improvers, for example flips); the triangles bisection made and the
  untouched ones keep their order, which is newest vertex bisection's
  memory between cycles. A `.pde2d` file keeps its order (it may be a
  saved mesh in the middle of adaptation). The function stays public for
  meshes built by hand.
- **Details of the refinement-edge change, accepted 2026-11-17:**
  `WorkingMesh` remembers each triangle's vertex order as constructed or
  as `set_refinement_edge` last left it, and `Remesher::refine` reorders
  (longest edge first) exactly the triangles whose order differs from that
  record: flips, insertions, re-triangulating boundary splits; moving a
  node keeps the order. `Remesher::run` reorders every triangle (a mesh
  just made is a starting mesh, also after newest vertex bisection). The
  pipeline report gains the metric `refinement_edges_set`. NVB and RGB
  `run` still set the longest edges before refining (for input that is
  not from the Mesher). `KellyEstimatorOptions{.edge_degree = 6}`; the
  jumps use the edge degree. A cycle of waiting refinement edges stays
  possible on a mesh mixing flipped and bisected triangles; it is
  conforming and reported (`incompatible_splits`).
- **Details of slice 2 (the adaptive loop in time), accepted 2026-11-18:**
  `transfer_solution` lives in `pde2d_adapt` and takes the new
  `MeshedProblem` (Dirichlet nodes get g and g_t); a new node on a curved
  boundary gets the average of its parent edge's ends (no extrapolation);
  without parents a private bucket-grid point location finds the old
  triangle, boundary nodes take their old boundary point on the same loop
  (through the curve links when they exist), a node outside the old mesh
  the nearest old boundary point. `Integrator::integrate` takes
  `max_steps`, for the "every n steps" rule. u_t comes from the last step
  ((u(t) − u(t − dt))/dt; for time order 2 the velocities and their
  quotient, u_tt at Dirichlet nodes from the boundary data). At each
  adaptation time the loop estimates and, above the tolerance, refines
  once and continues (no re-solving; a relative tolerance is relative to
  the first estimate). The schedule is "after an interval and/or n steps,
  whichever comes first", steps landing on snapshots count, an adaptation
  time near a snapshot time becomes it, no adaptation at t1. The node
  budget and `max_refinements` stop adapting for the rest of the run,
  which still reaches t1. Only `snapshot_times`, not `snapshot_every`.
  The result is a list of pieces plus the joined snapshots and steps. At
  most one adaptation changes the mesh. Public result structs grow at the
  end.
- **Newmark's acceleration is carried across pieces** (decided
  2026-11-18, after a review finding: restarting Newmark at a piece
  boundary solved M2 u_tt = … again and failed where M2 was singular at
  that time): `Solution` gains optional accelerations, filled by the
  integrator for a method that keeps u_tt; a run continued from such a
  solution does not call `start()` and uses them; the transfer moves them
  like the velocities. Continuation is then exact for Newmark too.
- **Damping steps after a transfer, as an explicit option** (decided
  2026-11-18): methods that do not damp (implicit midpoint, Newmark) keep
  the interpolation error a transfer leaves in the finest modes, so
  adaptive runs with them were worse than uniform ones. The adaptive loop
  gets an option for a number of steps of a damping method (implicit
  Euler) after each transfer before the chosen method continues (Rannacher
  smoothing). Configured by the user, never switched on by a failure, so
  it is not a fallback. A Newmark variant that damps only the highest
  frequencies (generalized-α), the better answer for waves: GitHub issue
  #40.
- **Details of the damping hand-over, accepted 2026-11-18:** a method
  declares `velocity_quotient_is_acceleration(order)` (implicit Euler at
  order 2), next to `keeps_acceleration`; after damping steps the state
  carries the damping method's own acceleration or that quotient, and any
  other damping method ahead of a method that keeps an acceleration is
  refused when the run starts (`std::invalid_argument` naming both). A run
  reports where its step grid ends (`IntegrationResult::anchor`,
  `StepAnchor{time, dt, steps}`) and `integrate` can continue on that grid
  (it throws if the anchor does not end at t0). The last step's real
  length is kept (`FreeState::dt`) and every quotient divides by it.
  `Integrator::method()` returns the method.
- **The anchor is passed across every piece on the same problem**
  (decided 2026-11-18, extending "only where the method changes"): a piece
  that continues on the same mesh continues the previous piece's step
  grid, so step times, and adaptation times from the "every n steps"
  rule, are those of one uninterrupted run.
- **Adaptive time steps: each method estimates its own error** (decided
  2026-11-18): a time method declares whether it can estimate a step's
  local error, and computes it cheaply from states it already has (no
  extra solves): implicit Euler from the change of u_t over the step,
  Newmark from the change of the acceleration (Zienkiewicz–Xie), the
  midpoint rule from the last steps. The error-controlled `StepController`
  refuses a method that does not declare it (no fallback). *Why:* the
  textbook practice and the cheapest; step doubling (about three solves a
  step) could come later as its own controller for such methods.
- **The error-controlled step, decided 2026-11-18:** the error is measured
  in the usual ODE-solver norm, each entry divided by atol + rtol·|u_i|,
  then the root mean square; options `{.atol, .rtol}`, a step is accepted
  when the norm is at most 1. The next step is dt · safety · (1/err)^(1/(p+1))
  for a method of order p, limited to a factor of 0.2 to 5 per step and to
  a minimum and maximum dt; a rejected step is taken again with the
  smaller dt (`StepRecord::accepted`). Because a new dt costs a new
  factorization, the controller keeps dt when the proposal is only a small
  change (inside 0.9 to 1.5 times the current dt); a rejection always
  shrinks it.
- **Details of slice 3 (adaptive time steps), accepted 2026-11-19:**
  `StepController` gains `controls_error()` (default false) and
  `judge(TakenStep) -> StepDecision{accept, next_dt, error_norm}`;
  `next_step(t)` gives only the first dt of an error-controlled run.
  Controllers are stateless: the next dt and the last accepted steps
  travel at the end of `StepAnchor` (`next_dt`, `previous_steps`).
  `TimeMethod` gains `error_estimate(order)` (order p, earlier steps read)
  and `local_error(...)`, with `PreviousStep`. Only the error in u is
  measured (not in the velocities). Implicit Euler without time terms
  declares no estimate. Steps without an estimate yet (the first ones of a
  run or after a transfer) are accepted at the current dt. The scale is
  atol + rtol·max(|u before|, |u after|), atol > 0, rtol ≥ 0. Defaults:
  atol 1e-6, rtol 1e-3, safety 0.9, factors 0.2 to 5, keep band 0.9 to
  1.5, min_dt 0, max_dt ∞; `initial_dt` is required. A step shortened to
  land on a snapshot does not shrink the following ones. A rejection
  always shrinks (a NaN error is a rejection) and retries at least at
  min_dt; a rejected step not longer than min_dt is a `std::runtime_error`,
  a method without an estimate a `std::invalid_argument` before the run.
  `steps` counts accepted steps, `prepares` and iterations every try;
  `StepRecord::error_norm`; the adaptive loop reports `rejected_steps`.
  `max_steps`, `snapshot_every`, `keep_before_last_step` and "every n
  steps" count accepted steps. After a transfer the next piece starts with
  the last proposed dt and no earlier steps. Damping steps use the same
  controller (their method needs an estimate). `FixedStep` ignores the new
  anchor fields. Estimates: Euler order 1 h(d₁ − d₀)/2, order 2
  h/2·(V₁ − V₀); midpoint order 2 from the velocities, h³/12·u‴ with u‴
  from the change of (V₁ − V₀)/h; Newmark dt²/12·(W₁ − W₀).
  `PreviousStep` stores the velocities too (decided 2026-11-19), so no
  estimate infers them from which method took a step.
- **The midpoint estimate at time order 1 compares with a prediction**
  (decided 2026-11-19, after a review finding: the u‴ estimate missed the
  error of averaging the state, h³/8 times the problem's rate times u″,
  and accepted a step 8 times over the tolerance): before the step an
  explicit extrapolation from earlier states predicts u₁ (no solve); the
  error is proportional to the computed u₁ minus the prediction (Milne's
  device, as in Gresho and Sani's adaptive trapezoidal rule). The
  prediction's own error depends only on u‴, so the difference sees both
  parts.
- **Follow-ups of the adaptive steps, as GitHub issues #41 to #43**
  (2026-11-19): an
  error-per-unit-step option (error proportional to the tolerance), a PI
  controller (fewer rejections on waves), and spreading the steps evenly
  before a landing (no short last step, fewer preparations).
- **Details of the marking strategies, accepted 2026-11-17:** the
  interface is `MarkingStrategy`, with `MaximumMarking`, `DorflerMarking`,
  `FixedFractionMarking` and `EquidistributionMarking`; `mark()` checks
  the indicators (one per triangle, none negative or NaN) and then calls
  the strategy's own selection. The result is the sorted triangle
  indices. An indicator of 0 is never marked. Defaults: maximum θ = 0.5,
  Dörfler θ = 0.5, fixed fraction 0.3; equidistribution has no default
  tolerance. Fixed fraction rounds the count to nearest. Dörfler divides
  the indicators by the largest before squaring (θ = 1 marks every
  positive indicator).

## Deferred

- Handling pure Neumann problems: to be discussed (the owner would
  ideally require mean zero): GitHub issue #32. Assembly builds the matrix
  as it is.
- Sharing compiled expressions between the `MeshedProblem`s of one
  `Problem` (adaptive refinement recompiles them per mesh): GitHub issue
  #29.
- Computing triangle and node adjacency once instead of in several places:
  GitHub issue #30. No `neighbours` field in `Mesh` for now.
- Anisotropic meshing (metric `MeshSpec`) and problem-aware planners.
- Callables, interior regions and materials, well-posedness checks.
- Robustness issues found in review, tracked as GitHub issues: a boundary
  split may overlap a nearby piece of the mesh; point location can cycle on
  some valid meshes; the inward split has no rollback if it fails half-way.
- Gmsh export; turning the boundary context of old `.pde2d` files into
  curve links.
- Natural conditions whose β̂_n is zero at some points although β_n is not
  zero as written (u_x = g on a circle): GitHub issue #27, with the
  well-posedness checks.
- An arc type stored by its end points (exact end points for flat arcs).
- The validity check also runs on meshes our own triangulators build, which
  the owner considers double work: GitHub issue #24.
- A curve-deviation bound that resolves small features of a large domain
  (relative to the chord or a turning angle, or local feature size):
  GitHub issue #25.
- Caching derived mesh quantities: GitHub issue #26.

## Open

- Nothing open right now.

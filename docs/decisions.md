# Design decisions

The owner's record of architecture decisions for the rewrite. Each entry says
what was decided and why. Agents implement what is here. They do not decide
anything that is missing: they raise it with the owner. Proposals and open
questions are listed separately at the end.

Started 2026-11-11. The previous header-only code is on the branch
`archive/header-only-v1`, kept for reference only.

## Process

- **The owner decides the architecture.** Agents present the options with
  their trade-offs and a recommendation, then wait for an answer. *Why:* the
  project exists so the owner learns. Choices an agent makes silently are
  choices the owner never understood.
- **Design is done in discussion and implementation by subagents**, one
  agreed slice at a time.

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
- **The archive is a reference, not a source.** Agents may port
  self-contained, well-tested pieces (the exact geometric predicates and the
  curve math). Everything else is written fresh to this design. *Why:* the
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
- **Region editing (split, merge, rename) stays as methods of
  `CurveGeometry`.** *Why:* these operations must preserve the class's
  invariants: unique names, closed loops and correct depths.
- **SVG import:** `<path>` elements give the curves. `id` (or `data-region`)
  gives the region name, and holes are found by depth.
- **Input/output** (SVG import and export, `.pde2d`, Gmsh import) is built on
  top of the core types once they exist. **`.pde2d` keeps the old format**,
  so the old viewer works unchanged. If the new mesh cannot fill the format,
  ask the owner before changing it.

## Meshing

- **The mesher never sees the `Problem`.** Its only external input is a
  `MeshSpec`. *Why:* this keeps meshing independent of the FEM code, and
  lets a future planner (preprocessing) or error estimator (adaptivity) talk
  to the mesher through a single contract.
- **v1 `MeshSpec` = maximum edge length.** It is a struct so that it can grow:
  later a metric tensor field for anisotropic meshes, with the isotropic size
  as a special case.
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
- **Refining an existing mesh:** a separate `Remesher{refiner, improvers}`
  refines a mesh that came from anywhere, and the `Mesher` uses a `Remesher`
  internally.
- **v1 algorithms:**
  - boundary discretizer: every chord ≤ `max_edge_length`; the curve
    deviation is the component's own option;
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
- **Two improver interfaces:** `Improver::run(mesh, spec)` is a pass over the
  whole mesh. `LocalImprover::improve_around(mesh, touched_triangles)` is
  called by a refiner after each change. One class may implement both. A
  local improver is handed to the refiner explicitly, as part of the
  refiner's options. *Why:* a whole-mesh pass after every split would make
  refinement O(n²).
- **Compatibility is matched by declared capabilities, not by name lists.**
  Improvers declare their effects (flips edges, moves nodes, makes the mesh
  Delaunay). Refiners declare what they tolerate and what they already
  maintain. A mismatch throws at construction with the reason, for example
  "Ruppert already maintains Delaunay". *Why:* new improvers work without
  editing refiners, and the declarations double as documentation.
- **Reports:** each stage report has common fields (`stage`, `time`,
  `messages`) plus named numeric `metrics`. Failures throw; messages only
  mean "worked, but…". The `Mesher` checks "target met" itself, against the
  `MeshSpec`, on the final mesh. *Why:* only the `Mesher` sees the final
  mesh, after the improvers, and the refiner is optional.
- **A facade with simple options comes later.**

## Mesh (output)

- **Contents:** `nodes`, `Triangle{nodes}`, `BoundaryEdge{nodes, region}`
  and `region_names`. A boundary edge is oriented so that the domain is on
  its left, and its region is an integer id into `region_names`.
- **Interior edges and connected pieces are computed when needed**, not
  stored.
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
- **Initial conditions are part of `Problem`** as optional expressions.
  Which ones are required follows from the time order. A missing initial
  condition is an empty `std::optional`, not NaN.
- **Facts are stored per coefficient** (`zero`, `depends_on_x/y/t`).
  Summaries such as `time_order`, "left-hand side depends on t" and
  "right-hand side depends on t" are computed from them by functions, never
  stored. "Zero" means zero as written.
- **Well-posedness checks are deferred.**

## Problem combined with a mesh

The name is still open; `MeshedProblem` is proposed. It holds
`std::shared_ptr<const Mesh>`. Building it:

1. matches region names in both directions: a condition naming an unknown
   region is an error, and so is a mesh region without a condition;
2. classifies the conditions;
3. decides the Dirichlet nodes, corners included;
4. compiles the expressions into fast numeric functions;
5. evaluates the initial conditions at the nodes;
6. merges the facts.

It stops just before matrix assembly.

## Deferred

- Handling pure Neumann problems: to be discussed.
- Anisotropic meshing (metric `MeshSpec`) and problem-aware planners.
- Callables, interior regions and materials, well-posedness checks.

## Open

- The name of the problem-plus-mesh object (`MeshedProblem` proposed).
- Everything from matrix assembly onwards (assembly, solvers, time
  integration) has not been discussed yet.

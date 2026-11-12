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
  `FinalImprover::run(mesh, spec, report)` is a pass over the whole mesh.
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

## Assembly

Discussed and agreed on 2026-11-12. The architecture (types, interfaces,
where assembly lives) has not been discussed yet.

- **Weak form.** The second-order part is written as A : ∇²u with
  A = [[a_xx, a_xy/2], [a_xy/2, a_yy]] and integrated by parts once:
  ∫ (A:∇²u) v = −∫ ∇v·A∇u − ∫ (∇·A)·∇u v + ∮ v (A∇u)·n. The term ∇·A
  needs derivatives of the coefficients, which symbolic input provides.
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
  right-hand side is b_F − A_FD U_D(t), plus −M₁,FD U_D′ − M₂,FD U_D″ for time
  derivatives, which come from differentiating g symbolically. *Why:*
  elimination is exact and standard (deal.II, FEniCS); keeping A_FD lets a
  time-dependent g cost one matrix–vector product per step. Edge blocks
  touching a Dirichlet corner are routed the same way. The same routing
  applies to every matrix and to the load vector.
- **"Right-hand side depends on t" also looks at the boundary data g**,
  since a time-dependent Dirichlet value makes the right-hand side
  time-dependent even when f is not.
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
  every matrix and rebuild (one for A_FF, one for A_FD). How it is built is
  still to be discussed.
- **Parallel assembly uses colouring:** triangles are grouped so that no two
  in a colour share a node; colours run one after another, with no atomics.
  *Why:* atomics make results depend on thread timing (floating-point
  addition is not associative) and are slow under contention; storing all
  element blocks first costs about 2.5× the matrix in memory. The colouring
  must itself be deterministic.
- **Quadrature is internal and fixed at compile time**, never a user input:
  a 3-point rule on triangles (exact for degree 2) and 2-point
  Gauss–Legendre on edges. *Why:* it keeps P1's convergence rates and is
  exact for constant-coefficient mass matrices.

## Deferred

- Handling pure Neumann problems: to be discussed.
- Anisotropic meshing (metric `MeshSpec`) and problem-aware planners.
- Callables, interior regions and materials, well-posedness checks.
- Robustness issues found in review, tracked as GitHub issues: a boundary
  split may overlap a nearby piece of the mesh; point location can cycle on
  some valid meshes; the inward split has no rollback if it fails half-way.
- Gmsh export; turning the boundary context of old `.pde2d` files into
  curve links.
- An arc type stored by its end points (exact end points for flat arcs).
- The validity check also runs on meshes our own triangulators build, which
  the owner considers double work: GitHub issue #24.
- A curve-deviation bound that resolves small features of a large domain
  (relative to the chord or a turning angle, or local feature size):
  GitHub issue #25.
- Caching derived mesh quantities: GitHub issue #26.

## Open

- The name of the problem-plus-mesh object (`MeshedProblem` proposed).
- Quality guarantees: an improver that runs after a refiner can break the
  refiner's guarantee (smoothing after Ruppert can lower the minimum
  angle). Whether to declare, warn and measure this is not decided yet.
- The architecture of the assembled problem and of assembly itself
  (types, interfaces, ownership), and the inheritance of the facts into it.
- Whether the eliminated system is renumbered to free nodes only or keeps
  its full size with identity rows for Dirichlet nodes.
- How the sparsity pattern is built.
- β̂_n can vanish at some points even when β_n is not zero as written (for
  example u_x = g on a circle), so the structural check alone may not be
  enough.
- Solvers and time integration have not been discussed yet.

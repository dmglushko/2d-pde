# Architecture

How the code is organized now. The decisions behind it are in
[decisions.md](decisions.md); where it is heading is in
[vision.md](vision.md).

## Modules

```
include/pde2d/geometry/   src/geometry/   ->  library pde2d_geometry
include/pde2d/mesh/       src/mesh/       ->  library pde2d_mesh
include/pde2d/io/         src/io/         ->  library pde2d_io
include/pde2d/fem/        src/fem/        ->  library pde2d_fem
```

`pde2d_mesh` depends on `pde2d_geometry`, and `pde2d_io` and `pde2d_fem`
each on `pde2d_mesh` (so on both), never the reverse. Public headers are in
`include/pde2d/<module>/`; everything in `src/` is private, including the
private headers there (`src/mesh/*.hpp`, `src/io/*.hpp`, `src/fem/*.hpp`).
`pde2d_geometry` and `pde2d_mesh` have no external dependencies;
`pde2d_io` links libxml2 privately (only `src/io/svg_import.cpp` includes
it), and `pde2d_fem` links SymEngine 0.14 privately (only through the
private header `src/fem/symengine_expression.hpp`). The tests use
Boost.Multiprecision (header-only) in one place:
`geometry_predicates_oracle` checks the adaptive predicates against an
arbitrary-precision evaluation. The viewer tests need Node.js and are
skipped without it.

Namespaces: `pde2d::geometry`, `pde2d::mesh`, `pde2d::io` and `pde2d::fem`.

## Geometry

| Type | Role |
| --- | --- |
| `Point` | a point or vector, with `+ - * dot cross norm distance` |
| `Line`, `QuadraticBezier`, `CubicBezier`, `EllipticalArc` | the kinds of exact curve, parameter `t` in [0, 1]; each owns its mathematics as methods (the core, listed in the `CurveKind` concept) |
| `Curve` | one curve of any kind, held by value (a class around a `std::variant` of the kinds); the same core methods, forwarded; `kind()` gives the variant |
| `BoundaryCurve` | a `Curve` and its region name |
| `Loop` | a closed list of `BoundaryCurve`s; built by the user |
| `CurveGeometry` | a validated set of loops with computed depth and parent |

Curve operations are of three sorts:

- **The core**, methods of every kind and of `Curve`: `validate`,
  `point_at`, `derivative_at`, `second_derivative_at`,
  `derivative_bounds`, `split`, `subcurve`, `reversed`, `transformed`,
  `bounding_box`, `exact_length`, `exact_length_parameter`. A method is in
  the core only if it needs the kind's own data. The `CurveKind` concept
  lists them and a `static_assert` checks every kind, so a kind that
  misses one does not compile.
- **Functions built on the core**, written once for every kind:
  `start_point`, `end_point`, `is_line`, `curve_length`,
  `parameter_at_length_fraction`, `bounding_box` of several curves.
  `curve_length` uses the kind's `exact_length` when it returns a value
  (lines, circular arcs) and integrates numerically otherwise.
- **Code that differs by kind but belongs to another module** (SVG path
  commands, mirroring for SVG): `std::visit` over `curve.kind()` with
  `overloaded{...}`, one handler per kind and no catch-all, so a new kind
  does not compile until it is handled there.

The formulas are ported from the archive. `orient` and `incircle`
are exact predicates: Shewchuk's adaptive-precision `orient2d` and
`incircle` (floating-point filter, then exact expansion arithmetic in
stages), with a slow exact-integer evaluation for the rare undecided case
whose coordinates lie outside [2^-200, 2^200], where the expansion
arithmetic could overflow or underflow. Outside that range the filter's
answer is used only if none of its products underflowed (an underflowed
product, magnified by a later one, can exceed the error bound); the
range check comes before the filter, so no stage runs unprotected. `predicates.cpp` is compiled with
`-fno-fast-math -ffp-contract=off` and refuses to compile under
`__FAST_MATH__` or with `FLT_EVAL_METHOD != 0` (x87 extended precision),
because the error-free transformations need every operation rounded on
its own. `polygon.hpp` has `signed_area` (relative to the first point, so a
small polygon far from the origin does not cancel), `polygon_orientation`
(exact: the turn at the lowest point, decided by `orient`; `CurveGeometry`
and `BoundaryPolygon` decide loop directions with it, because the sign of
an area underflows or overflows at extreme scales),
`locate_in_polygon` and `find_crossing` for polygons.
`bounding_box.hpp`: `BoundingBox{min, max}` with `merged`, `padded`,
`contains`, `width`, `height`, `diagonal`, and `bounding_box` of points.
The box of a curve is its core method `bounding_box()` (end points plus
the exact interior extremes: roots of the Bezier derivatives, axis
extremes of arcs); `CurveGeometry::bounding_box()` is the box of all its
curves. Every axis-aligned box in the library uses it (the
geometry's validation scale, SVG view boxes, the CDT's enclosing
triangle, the validity grid, the polygon sweep, curve linking).

`CurveGeometry` construction:

1. validate each curve and region name, and that every loop is closed
   (within 1e-9 of the geometry's size);
2. flatten every loop to a polyline close to the curves (deviation at most
   1e-6 of the size) and reject any crossing or touching with exact
   predicates;
3. depth of a loop = number of loops containing it; parent = the
   containing loop one level up. Depth and parent live in private vectors
   next to the loops, not in `Loop`;
4. normalize directions: even depth counterclockwise, odd depth clockwise,
   so the domain is always on the left.

Region editing (`rename_region`, `merge_region_into`, `split_region`) is
done by methods, because it must keep the names unique. Shapes (`circle`,
`ellipse`, `rectangle`, `annulus`, `rectangle_with_semicircle`, `polygon`,
`regular_polygon`) are functions returning a `CurveGeometry`.

## Meshing: data flow

```
CurveGeometry ──► BoundaryDiscretizer ──► BoundaryPolygon
                      (MeshSpec)               │ validate()
                                               ▼
                                     InitialTriangulator
                                               │
                                               ▼
                          WorkingMesh ──► Refiner (optional) ──► Improver ... ──► Mesh
                          └──────────────── Remesher ─────────────────────────┘
```

`Mesher{discretizer, triangulator, refiner, improvers}` runs the stages in
this fixed order. The last two run inside a `Remesher{refiner, improvers}`,
which can also start from a `Mesh` that came from anywhere, with or
without the geometry behind it. Improvers are given only to the `Mesher`
(or `Remesher`); the refiner learns which of them to call locally from
the assembled pipeline, never from its own options.

| Type | Role |
| --- | --- |
| `LengthBound{absolute, relative}` | a length limit: optional absolute length, optional multiple of the domain size; the smaller applies, none set means no limit; `resolve(bound, domain_size)`, `validate(bound, name)`; in `mesh_spec.hpp` |
| `MeshSpec{max_edge_length}` | the mesher's only external input besides the geometry; `max_edge_length` is a `LengthBound` with at least one value |
| `CurveChain` | consecutive exact curves treated as one path, measured by arc length `s`; cumulative lengths, `position(s)`, `point_at(s)` |
| `CurveLink{chain, s0, s1}` | which chain (index into the holder's chain list) and arc-length interval an edge approximates, `0 <= s0 < s1 <= length`; in `curve_chain.hpp`, with `default_corner_angle_degrees` (30) |
| `BoundaryPolygon` | loops of straight edges; per edge a region id and an optional `CurveLink`; the chains; region names |
| `WorkingMesh` | the mutable mesh the stages edit |
| `Mesh` | the output: `nodes`, `Triangle{nodes}`, `BoundaryEdge{nodes, region, link}`, `region_names`, `chains` (links and chains empty for an imported mesh); `validate(const Mesh &)` checks it like `WorkingMesh::from_mesh` |
| `ImproverUse` | `local_and_final`, `local_only`, `final_only`: in each improver's options |
| `PipelinePlan` | resolved at construction: indices of the local improvers and of the final passes, and the warnings |
| `StageReport` | `stage`, `time`, `messages`, `metrics` (name → number) |
| `MeshResult` | `mesh`, `reports` (the first is always "pipeline"), `target_met`, `longest_edge`, `max_edge_length` and `domain_size` (what the spec resolved to) |

### Lengths and the domain size

`domain_size(...)` (`domain_size.hpp`) is the length that relative values
multiply. Its definition is a private policy (`domain_size_policy.hpp`,
fixed by the library like the boundary split policy): currently the
diagonal of the axis-aligned bounding box (`geometry::bounding_box`) of
the boundary. It is measured on the geometry's curves, on a mesh's chains
when it has them (the same curves, so exactly the same number), else on
its boundary nodes (an imported mesh: splitting a boundary edge at its
midpoint keeps the box). Settings never know the domain: the discretizer
resolves `max_edge_length` and `max_curve_deviation` against
`domain_size(geometry)`, each refiner resolves `max_edge_length` against
`domain_size(mesh)`, and the `Remesher` does the same for `target_met`.
Each reports `domain_size` and the resolved `max_edge_length` (and
`max_curve_deviation`) as metrics. Nothing is cached (GitHub issue #26).

### Chains

The discretizer joins consecutive curves of a loop into a `CurveChain`
unless the region changes or the boundary turns by more than
`corner_angle_degrees` (default 30°, `default_corner_angle_degrees`) at
the junction. The
turning angle is the angle between the end tangent of one curve and the
start tangent of the next; where a derivative vanishes at that end, the
second derivative's direction is used, then the curve's chord; a junction
without a usable direction is a break. A loop without any break is one
chain from the start of its first curve, so that point (the seam) is
always a node and links never wrap around.

Each chain is sampled as one path: `ceil(length / h)` pieces of equal arc
length, raised while a chord is longer than `h` or a piece may deviate from
its chord by more than `max_curve_deviation` (when set). For a piece across
junctions the bound is the maximum over its curve pieces of
`(1 - u) d0 + u d1 + M u (1 - u) / 2` (`d0, d1` the distances of the curve
piece's ends from the chord, `M` its second-derivative bound), which is
`M / 8` on one curve and does not grow at a smooth junction. Junctions
inside a chain are not forced to be nodes.

A linked boundary edge is split at `s = (s0 + s1) / 2`: binary search over
the chain's cumulative lengths for the curve, then length inversion
(`parameter_at_length_fraction`) inside it. An edge lying on one straight
curve is split at its chord midpoint (the same point without the curve's
rounding); an unlinked edge too. Chains live only on the meshing side.
The chain building is shared private code (`src/mesh/loop_chains.*`), used
by the discretizer and by the `Remesher` when it links a mesh to a
geometry, so both build the same chains.

### WorkingMesh

- Triangles are counterclockwise. Edge `i` of a triangle is opposite its
  vertex `i` (from vertex `i+1` to `i+2`), so the triangle is on its left.
- `neighbour(t, i)` is the triangle across edge `i`, or `no_index`.
- An edge without a neighbour is exactly one boundary edge, oriented with
  the domain on its left, with a region id and an optional curve link.
  Boundary loops never touch, so each boundary node has one outgoing
  boundary edge.
- Edits (`flip`, `insert_in_triangle`, `split_interior_edge`,
  `split_boundary_edge`, `try_move_node`) keep all of this and return the
  triangles they touched. An edit that would
  invert a triangle throws and leaves the mesh unchanged.
- **Deletions.** Splitting a curved boundary edge may delete triangles and
  nodes. They stay as tombstones (`is_triangle_deleted`,
  `is_node_deleted`), so indices held by a refiner stay valid and new
  entries are always appended. `compact()` removes them and renumbers in
  index order; the `Remesher` calls it right after the refiner, so final
  passes never see tombstones, and `to_mesh()` never contains them.
  Boundary edges are never deleted.
- A split boundary edge keeps its region, and its link is split into two
  halves of the arc-length interval.
- The constructor validates everything (orientation, conformity, boundary
  edges matching the triangulation, no unused nodes, indices in range,
  links inside their chains), then the geometry (see "Geometric validity"
  below). So does `from_mesh()`, which the importers and the `Remesher`
  use, and `validate(const Mesh &)` runs the same checks on their own.
- `to_mesh()` lists boundary edges loop by loop in walking order, and
  copies the chains and each edge's link into the `Mesh`. `from_mesh()`
  takes them back, so a mesh refined once can be refined again on the
  true curves without the geometry.

### Geometric validity

A mesh can be connected correctly and still overlap itself, for example a
small triangle lying inside a big one with all six sides as boundary
edges. After the connectivity checks the constructor runs a geometric
check (private, `src/mesh/mesh_validity.*`). It rests on one identity:
when every triangle is counterclockwise and every interior edge is used
once in each direction, the number of triangles covering a point equals
the winding number of the boundary edges around it. So the triangles
overlap nowhere exactly when:

1. **no fold**: at every node, the triangles' angular sectors contain one
   reference direction exactly once (an interior node's triangles turn
   once around it, a boundary node's less than once; the reference is
   the outgoing boundary edge's direction there), decided with `orient`;
2. **no crossing boundary edges**: no two boundary edges cross or touch,
   except consecutive edges at their common node (this also rejects two
   nodes at the same place on the boundary and loops that touch);
3. **no piece inside another**: no node lies in or on a triangle that
   does not use it (exact `orient`), which also rejects a piece in the
   material of another (an island is fine only inside a hole).

Every decision uses exact predicates. Checks 2 and 3 use a uniform grid
over the nodes' bounding box with about one cell per triangle; each
triangle and boundary edge is listed in the cells its bounding box
touches. The cost is O(n) for meshes whose triangles are of similar size;
very long thin triangles or many boundary edges in one cell make it
slower (worst case quadratic) but never wrong. Errors name the node,
triangle or boundary edges ("Node 3 lies in triangle 0 but is not one of
its nodes ...", "Boundary edges 0 (0 -> 1) and 3 (3 -> 4) cross or touch
...", "The triangles around node 0 overlap: they turn around it 2 times
..."). Edits (flips, splits, moves) keep validity by construction and are
not re-checked.

### Splitting a boundary edge

`WorkingMesh::split_boundary_edge(k)` goes through the library's boundary
split policy, a private virtual interface (`src/mesh/boundary_split_policy.*`)
with one fixed implementation; it is not a user option. Its steps are
private too: static functions of `detail::BoundarySplitSteps`
(`src/mesh/boundary_split_steps.*`), which `WorkingMesh` names as a friend
so they can edit its internals. Only `split_boundary_edge`,
`is_unsplittable` and `BoundarySplitResult` are public. The steps, with
`a -> b` the edge, `c` the opposite vertex and `p` the split point:

1. **Direct** (`split_directly`): if `p` lies inside the angle at `c`, join
   it to `c` (two triangles replace one).
2. **Local re-triangulation.**
   - `p` outside the mesh (the curve bulges outwards): add the triangle
     `(a, p, b)` on the edge (`add_cap`), unless a neighbouring boundary
     edge crosses it.
   - `p` inside the mesh (the curve bulges inwards): first check the
     straight triangle `(a, b, p)`: no other boundary edge may cross it,
     no boundary node may lie in it, and no node on its new sides. Then
     insert `p` into the triangle containing it, force `a-p` and `p-b` by
     flips (`detail::recover_edge`, shared with the CDT), and cut off the
     region between them and the old edge with every node in it, which
     lies outside the true domain (`cut_off`). The region is
     bounded by forced edges, so deleting its triangles leaves no hole.
3. **Give up**: change nothing, mark the edge unsplittable
   (`is_unsplittable`) and return the reason. There is no chord fallback
   for linked edges.

Refiners never retry an unsplittable edge; they add a message to their
report and count `unsplittable_edges`, `retriangulated_splits` and
`deleted_nodes`. The target may then be missed, which `target_met` shows.

### Components and capabilities

Every stage is a virtual interface with `name()`. Each concrete component
takes its own options struct (designated initializers) in its constructor.

```cpp
class BoundaryDiscretizer { BoundaryPolygon run(geometry, spec, report); };
class InitialTriangulator { TriangulatorCapabilities capabilities();
                            WorkingMesh run(polygon, report); };
class Refiner             { RefinerCapabilities capabilities();
                            void run(mesh, spec,
                                     std::span<const LocalImprover *const> local,
                                     report); };
class Improver            { std::string name(); ImproverEffects effects();
                            ImproverUse use();
                            const LocalImprover *as_local();   // or null
                            const FinalImprover *as_final(); }; // or null
class LocalImprover       { std::string name();
                            size_t improve_around(mesh, touched); };
class FinalImprover       { void run(mesh, spec, report); };
```

Declarations:

- `ImproverEffects{flips_edges, moves_nodes, makes_delaunay}`;
- `RefinerCapabilities{tolerates_flips, tolerates_node_moves,
  maintains_delaunay}` (what the refiner accepts from local improvers,
  and what its output keeps);
- `TriangulatorCapabilities{supports_holes, supports_several_pieces}`;
- `ImproverUse` (from the improver's options) and the roles: `as_local()`
  and `as_final()` return the object to call in that role, or null for a
  role the improver does not have. `DelaunayFlips` derives from all three
  interfaces and returns itself from both; `LaplacianSmoothing` has only
  the final role. `LocalImprover` keeps `name()` because the refiner names
  each local improver in its report.

Checks:

1. **At construction** (`plan_pipeline`, called by `Remesher`, so also by
   `Mesher`): an improver runs locally if its mode allows it, it has a
   local version, and the refiner tolerates its effects (a Delaunay maker
   is refused by a refiner that maintains Delaunay). It runs as a final
   pass if its mode allows it and it has a final role. Both keep the list
   order. A mode that cannot be honoured is a **warning**, one per role,
   for example "delaunay_flips: ruppert already maintains Delaunay, so it
   runs only as a final pass" or "<name>: it has no final version, so it
   runs only locally"; so is a final pass that would make an already
   Delaunay mesh Delaunay (it still runs). Only a null component throws.
2. **When a run starts** (`Mesher::run`): the spec is valid, and the
   triangulator supports the geometry's holes and pieces.

The `Remesher` stores the plan (`plan()`, also on the `Mesher`) and a
"pipeline" `StageReport` (`pipeline_report()`) that every run copies to
the front of its reports. Its first message is the plan in words ("plan:
refiner ruppert; local: none; final: delaunay_flips,
laplacian_smoothing", or "plan: no refiner; ..."), followed by the
warnings; its metrics are `local_improvers` and `final_passes` (counts),
and its `time` is the time the planning took. Like every report it keeps
its structured fields, so times can be summed. The refiner calls
the local improvers after each change and reports the sum of their
returned counts as `local_improver_changes` (and one
`local_improver_changes.<name>` each).

Between stages the `Mesher` validates the `BoundaryPolygon` (no crossing
chords, orientation by depth, nesting), so a discretizer's mistake is
reported as such. After the last stage it checks the target: every edge of
the final mesh at most the resolved `max_edge_length` (`target_met`,
`longest_edge`, `MeshResult::max_edge_length`).
Failures are exceptions with specific messages; report messages only mean
"worked, but…".

### v1 components

| Component | File | Notes |
| --- | --- | --- |
| `ChordLengthDiscretizer` | `chord_length_discretizer.*` | chains sampled by equal arc length, chord ≤ h; options `max_curve_deviation` (a `LengthBound`, off by default), `corner_angle_degrees` (30) |
| `EarClipping` | `ear_clipping.*` | per piece: bridge holes, cut best-shaped ears first |
| `Cdt` | `cdt.*` | incremental Delaunay in an enclosing triangle, edge recovery by flips, outside removed, final flips |
| `Ruppert` | `ruppert.*` | makes its input Delaunay, splits encroached/long boundary edges, inserts circumcenters; options `min_angle_degrees`, `max_points`; accepts only local improvers without effects |
| `LongestEdgeBisection` | `longest_edge_bisection.*` | Rivara's longest-edge propagation path; option `max_points`; tolerates local flips and moves |
| `DelaunayFlips` | `delaunay_flips.*` | Lawson flips; both roles (`LocalImprover` and `FinalImprover`); option `use` (default `local_and_final`) |
| `LaplacianSmoothing` | `laplacian_smoothing.*` | interior nodes to neighbour average, inverting moves skipped; final role only; options `iterations`, `use` (default `final_only`) |

Shared routines that components call are plain functions in
`delaunay.hpp`: `is_locally_delaunay`, `flip_to_delaunay`,
`flip_to_delaunay_around`. Private shared code in `src/mesh/`:
`delaunay_internal.hpp` (Lawson with a touched list), `edge_recovery.*`
(forcing an edge by flips), `local_improvers.hpp` (calling and counting a
refiner's local improvers), `boundary_split_policy.*` and
`boundary_split_steps.*` (the policy and its steps), `loop_chains.*`
(joining a loop's curves into chains), `mesh_validity.*` (the geometric
validity check).

### Remesher and curve links

`Remesher::run(mesh, spec)` uses the mesh's own chains and links: a mesh
made by the `Mesher` or `Remesher` carries them, so refining it again puts
new boundary points on the true curves. An imported mesh has none, and its
boundary edges are split at chord midpoints.

`Remesher::run(mesh, geometry, spec, corner_angle_degrees = 30)` is for a
mesh without links (any links it has are replaced). It first joins the
geometry's loops into chains with the same code and rule as the
discretizer, then links each boundary edge to the chain of its region that
contains both end points (within 1e-9 of the mesh size), taking the short
way along the chain. An edge whose ends lie on two different chains
(across a corner, a region change or the start of a closed loop) stays
unlinked and is split at its chord midpoint; the "curve_links" report
(right after "pipeline") counts `chains`, `linked_edges` and
`unlinked_edges`. A mesh region missing from the geometry, a boundary node
off its region's curves, or an angle outside [0, 180] is an error.

## Input and output (`pde2d_io`)

```
SVG file ──import_svg──► CurveGeometry ──export_svg──► SVG file
                                         (Mesher)
.msh file ──import_gmsh──► Mesh ──export_pde2d──► .pde2d ──► tools/mesh_viewer
.pde2d file ──import_pde2d──►  └──export_mesh_svg──► SVG picture
```

Every function has a `std::istream&` / `std::ostream&` form and a
`std::filesystem::path` form. Problems in the content throw
`std::invalid_argument` with a specific message; a file that cannot be
opened, read or written throws `std::runtime_error`.

| Function | Header | Notes |
| --- | --- | --- |
| `import_svg` | `svg.hpp` | `<path>` elements; all path commands onto `Line`, `QuadraticBezier`, `CubicBezier`, `EllipticalArc` (arcs converted on the unit circle, so any radius works; a nearly flat arc, sweep below 0.01 rad, becomes a `CubicBezier` through exactly its end points); region = `data-region`, else `id`; subpaths joined into loops by end points (a run may be reversed); holes by depth; `<g>` entered, metadata ignored; transforms, shapes, nested `<svg>`, DTDs rejected; user units; y flipped, (x, y) → (x, −y), after the runs are joined (so error messages give file coordinates); `CurveGeometry` then normalizes the reversed loop directions |
| `export_svg` | `svg.hpp` | points written as (x, −y), so `import_svg(export_svg(g))` gives the same numbers back; one `<path>` per run of one region in a loop; arcs in pieces of at most 90°; id = region name if unique and a valid XML id, else `pde2d_run_<k>` with `data-region`; shortest exact numbers |
| `export_mesh_svg` | `svg.hpp` | nodes drawn at (x, −y), the right way up like `export_svg`; triangles (light fill, grey edges) and one coloured `<path>` per region with `data-region` and a `<title>`; no nodes, no links |
| `export_pde2d` | `pde2d_mesh.hpp` | the old `PDE2D_MESH` format (`docs/formats/pde2d-mesh.md`): version 2 for one loop, 5 with `RESULT 0` for several; outer loop (through the lexicographically smallest boundary node) first; links dropped |
| `import_pde2d` | `pde2d_mesh.hpp` | versions 2–5; v1 and untagged edges rejected; result trailers checked and ignored; validated like a `WorkingMesh` (with the geometric check); records read as they come, never allocated for a claimed count first; no links |
| `import_gmsh` | `gmsh.hpp` | MSH 4.1 and 2.2 ASCII; 3-node triangles only; boundary edges must lie on line elements of one physical group of dimension 1 (its name, or `physical_<tag>`); triangles made counterclockwise; unused nodes dropped; no links |

Imported meshes have no chains or links, so the `Remesher` splits their
boundary edges at chord midpoints unless it is given the geometry.
Private code: `svg_path.*` (the path data parser, the SVG arc
conversion and the y mirror `mirrored_y`), `text.hpp` (files, exact number parsing and printing).

The mesh viewer (`tools/mesh_viewer/`) is the previous code's viewer,
unchanged. `examples/mesh_viewer/produce_mesh` writes a mesh for it.

## Problems (`pde2d_fem`)

```
text ──► Expression ─┐
                     ├─► Problem ──┐
         Parameters ─┘             ├─► MeshedProblem ──► Assembler ──► AssembledProblem
                   Mesh (shared) ──┘                      (at t, optional previous)
```

| Type | Header | Role |
| --- | --- | --- |
| `Expression` | `expression.hpp` | a scalar expression in x, y, t and named parameters, built from text (`Expression("1 + k*x*y")`); immutable, copies share the parsed form; `text()`, `facts()`; `Expression()` is zero |
| `ExpressionFacts` | `expression.hpp` | `zero`, `depends_on_x/y/t`, as written |
| `Parameters` | `expression.hpp` | `std::map<std::string, double>`: name → value |
| `CompiledExpression` | `compiled_expression.hpp` | an `Expression` with its parameters substituted, compiled to a function `(x, y, t) -> double`; keeps the expression's facts; `CompiledExpression()` is zero |
| `Equation` | `problem.hpp` | the coefficients `a_xx a_xy a_yy b_x b_y c m1 m2 f` of `a_xx u_xx + a_xy u_xy + a_yy u_yy + b_x u_x + b_y u_y + c u + m1 u_t + m2 u_tt = f`; unset ones are zero |
| `BoundaryCondition` | `problem.hpp` | `alpha beta_n beta_tau beta_x beta_y gamma g` of `alpha u + beta_n u_n + beta_tau u_tau + beta_x u_x + beta_y u_y + gamma u_t = g`, and `priority` (default 0) |
| `InitialConditions` | `problem.hpp` | `std::optional<Expression>` `u` and `u_t` |
| `Problem` | `problem.hpp` | `equation`, `parameters`, `boundary_conditions` (region name → condition), `initial` |
| `ConditionKind` | `problem.hpp` | `dirichlet`, `natural`, `dynamic`; `classify(condition)` and `is_oblique(condition)` |
| `CompiledEquation`, `CompiledCondition` | `meshed_problem.hpp` | the same members as `CompiledExpression`s; a condition also has its `kind`, `oblique` and `priority` |
| `DirichletNode` | `meshed_problem.hpp` | a node and the regions whose `g / alpha` is averaged there (one, or two at an equal-priority corner) |
| `MeshedProblem` | `meshed_problem.hpp` | `Problem` + `std::shared_ptr<const Mesh>`, checked and compiled, up to assembly |
| `CompiledDivergence` | `meshed_problem.hpp` | `x`, `y`: the compiled divergence of A = [[a_xx, a_xy/2], [a_xy/2, a_yy]] (`MeshedProblem::divergence_of_a()`) |
| `SparsityPattern`, `CsrMatrix` | `csr_matrix.hpp` | compressed sparse rows: `rows`, `columns`, `row_offsets`, `column_indices` (signed `Index`, sorted and unique per row), `find`; a matrix is a shared pattern plus `values`, with `entry(i, j)` and `multiply(x)` |
| `Operator` | `assembled_problem.hpp` | one operator of the system: `free` (X_FF), `dirichlet` (X_FD), `depends_on_t`, `is_zero` |
| `AssembledProblem` | `assembled_problem.hpp` | the system M2 U'' + B U' + A U = b in the free unknowns at one t |
| `Assembler` | `assembler.hpp` | `assemble(meshed_problem, t, previous = nullptr)` |

```cpp
using namespace pde2d::fem;
Problem problem{
    .equation = {.a_xx = Expression("-k"), .a_yy = Expression("-k"),
                 .f = Expression("1")},
    .parameters = {{"k", 1000.0}},
    .boundary_conditions = {
        {"wall", {.alpha = Expression("1"), .g = Expression("0")}},
        {"inlet", {.beta_n = Expression("1"), .g = Expression("x*y")}}}};
auto meshed = std::make_shared<const MeshedProblem>(mesh, problem);
double a = meshed->equation().a_xx(0.5, 0.5, 0.0); // -1000
```

**Expressions.** SymEngine parses the text when an `Expression` is built
(`parse`, with `^` as power). The parsed form is then checked against the
allow-list (`detail::unsupported`, `src/fem/expression.cpp`), walking it
the way SymEngine's evaluator does: numbers (each must be finite as a
double, so `1e1000` and `10^400` are rejected), x, y, t, parameter names,
`pi` and `E`; `+ - * /` and powers; `sin cos tan asin acos atan sinh
cosh tanh exp log sqrt abs`; `Piecewise` whose last branch is `(value,
True)`, with conditions built from `< <= > >= == !=` (`Eq`, `Ne`) and
`And`, `Or`, `Not`, allowed only as `Piecewise` conditions. Anything else
throws `std::invalid_argument` naming it: a parse error, an unknown
function (`foo(x)`), a construct not on the list (`loggamma`, `Xor`,
`max`, `EulerGamma`; `atan2` is refused by name before parsing, because
SymEngine's parser turns `atan2(x, x)` into `pi/4` whatever the sign of
x), a complex value (`I`, `sqrt(-1)`), an infinite or undefined constant
(`oo`, `1/0`, `nan`), a constant part whose value is not a finite real
double (`exp(1000)`, `asin(2)`; evaluated once, also after parameters are
substituted), a condition used as a number
(`2*(x < 0)`), a `Piecewise` without a final unconditional branch (SymEngine's evaluator
would read past its branches). Constructs are added to the list one at a
time, each with a test against the standard library
(`tests/fem/expression.cpp`) and a tested derivative
(`tests/fem/derivative.cpp`, and in t through `MeshedProblem`). The facts are read off the parsed form:
`zero` when it is the number zero (SymEngine's parser already combines
like terms, so `x - x` is zero, but nothing more is simplified),
`depends_on_*` when the symbol is free in it. Free symbols other than x,
y, t are parameter names, unknown constants for the facts.
`CompiledExpression` checks the parameters (a plain symbol name other than
x, y, t, which the parser reads back as that symbol, so not `pi` or `E`; a
finite value), substitutes the values (an unknown name throws), checks the
substituted form again (`log(k)` with k = -1 is complex, `1/k` with k = 0
is infinite) and prepares SymEngine's `LambdaRealDoubleVisitor` (a tree of
`std::function`s; no LLVM in this build). A call does not check its
result. Calls may run in several threads at once: they only read the
prepared functions, and every allowed construct evaluates through standard
math functions without shared state (`tests/fem/expression_threads.cpp`,
also run under ThreadSanitizer and Helgrind). SymEngine itself is built
without thread-safe reference counting, so all symbolic work (building,
substituting, differentiating, compiling, dropping the last copy of an
expression) happens in one thread.

**Derivatives** are taken in one place, the private
`detail::derivative(expression, Variable::x | y | t)`
(`src/fem/derivative.hpp`). It runs SymEngine's differentiation with two
rules of its own: `abs`, which SymEngine leaves as an unevaluated
`Derivative`, becomes `d|u| = sign(u) du` with the sign written as
`Piecewise((1, 0 < u), (-1, u < 0), (0, True))` (0 at the kink), so
`d/dt abs(t - 1)` is `-Piecewise((1, 0 < 1 - t), (-1, 1 - t < 0), (0,
True))`; and a `Piecewise` is differentiated branch by branch with its
conditions kept, collapsing to the common value when all branches give
the same derivative. The expression itself keeps its `abs`; only its
derivative contains the sign. The result is checked against the
allow-list like text (`expression_from_form`), so a derivative that leaves
it (`d/dx 1e308*x^2` has an infinite number) throws. Every allowed
construct has a derivative inside the allow-list. It is used for
`d/dt (g / alpha)` (`time_derivative_of_quotient`), its second time
derivative, and the divergence of A (`divergence_of_a`), all in the
`MeshedProblem` constructor.

**Classification** (`classify`), by which coefficients are zero as
written: no spatial derivative and no `gamma` is Dirichlet (`alpha` must
be set: the value is `g / alpha`); a derivative with a normal part
(`beta_n`, `beta_x` or `beta_y`) is natural, or dynamic with `gamma`.
Rejected: no term at all, a boundary ODE (`gamma` without a spatial
derivative) and tangential only (`beta_tau` alone). `is_oblique`: `beta_tau`,
`beta_x` or `beta_y` is set.

**Summaries** are functions, computed from the facts every time, for a
`Problem` and for a `MeshedProblem` (one private template,
`src/fem/fact_summaries.hpp`): `time_order` (2 with `m2`, else 1 with `m1`
or any `gamma`, else 0), `coefficients_depend_on_t`, `data_depends_on_t` (`f` or
any `g`).

**`MeshedProblem` construction.** `mesh::validate` runs first, then:

1. region names are matched both ways (a condition for an unknown region
   or a mesh region without a condition is an error);
2. every condition is classified; `conditions()` is indexed by region id;
3. Dirichlet nodes: each boundary node has one incoming and one outgoing
   boundary edge; it is Dirichlet if either edge's region is. With one
   Dirichlet region it takes that one (Dirichlet beats natural and
   dynamic); with two, the higher `priority` wins and equal priorities are
   averaged. The other nodes are the free nodes, in index order: unknown
   `i` of the shrunk system is node `free_nodes()[i]`, and `free_index`
   goes back;
4. every expression is compiled with the parameters; an error names the
   coefficient ("Region 'wall', coefficient g: ..."). The
   time-independent winning boundary data is checked for finiteness at
   every Dirichlet node, independently of initial conditions. The
   competing values at Dirichlet corners are then compared: equal as written, never
   reported; otherwise, if one depends on t, reported as "may differ";
   otherwise evaluated at the node and reported when they differ;
5. the initial conditions that `time_order` requires are compiled (a
   missing one is an error; one that is not needed is ignored and one that
   depends on t is evaluated at the start time, each with a report
   message); for time order 1 or 2 so is `d/dt (g / alpha)` of every
   Dirichlet condition that depends on t, and for time order 2 its second
   derivative (with `detail::derivative`; a derivative outside the
   allow-list is an error). Each compiled initial
   condition is then compared with the boundary data that replaces it at
   the Dirichlet nodes, u with `g / alpha` and u_t with `d/dt (g /
   alpha)`, like the corners: when neither depends on t, both are
   evaluated at every Dirichlet node and compared exactly (the boundary
   data must be finite there, or it is an error), and a difference is one
   message with the count and the first three nodes; otherwise one message
   says they may differ;
6. the divergence of A is differentiated (`detail::divergence_of_a`,
   through `detail::derivative`) and compiled: `divergence_of_a()`;
7. the facts stay with each compiled function, and the summaries read
   them.

`dirichlet_value(k, t)` is the average of `g / alpha` over the regions of
`dirichlet_nodes()[k]` (with `std::midpoint`, so two large finite values
do not overflow; an error if not finite). `dirichlet_rate(k, t)` and
`dirichlet_acceleration(k, t)` average the first and second time
derivatives the same way; they exist for time order 1 or 2 and for time
order 2 respectively and throw `std::invalid_argument` below that. `initial_values(t0)` and
`initial_velocities(t0)` return `std::optional<std::vector<double>>`: u
and u_t at every mesh node at the start time t0 the time integrator
passes, computed on every call, or an empty optional when the time order
has no such values (no values for time order 0, no velocities below 2). At
a Dirichlet node they come from the boundary data (`dirichlet_value(k,
t0)`, and the average of `d/dt (g / alpha)` for the velocity), elsewhere
from the initial condition at `(x, y, t0)`. A t0 that is not finite (for
any time order) or a value that is not finite throws
`std::invalid_argument`:

```cpp
if (const auto u0 = meshed->initial_values(0.0)) // empty for time order 0
  for (std::size_t node = 0; node < u0->size(); ++node)
    use((*u0)[node]);
```

`report()` is a `mesh::StageReport` named "meshed_problem" with the time, the metrics
`regions`, `dirichlet_nodes`, `free_nodes`, `corners`,
`dirichlet_over_natural_corners`, `priority_corners`, `averaged_corners`,
`differing_corners`, `differing_initial_values` and
`differing_initial_velocities` (Dirichlet nodes where the initial
condition differs from the boundary data, or all of them when it may
differ), one message per differing corner, for example
"Corner at node 0 (0, 0): regions 'bottom' and 'left' (both priority 0)
give 0 and 1; their average 0.5 is used", one per initial condition
that is not needed or depends on t, and one per initial condition that
differs from the boundary data, for example "The initial condition u
differs from g / alpha at 4 of 5 Dirichlet nodes, where g / alpha is used:
node 0 (0, 0) has u = 1, g / alpha = 0; node 1 (0.5, 0) has u = 1, g /
alpha = 0.5; node 3 (0, 0.5) has u = 1, g / alpha = 0; and 1 more", or
may differ: "The initial condition u and g / alpha may differ at the 8
Dirichlet nodes, where g / alpha is used; one of them depends on t, so
whether they differ depends on the start time t0".

## Assembly (`pde2d_fem`)

```cpp
auto meshed = std::make_shared<const MeshedProblem>(mesh, problem);
const Assembler assembler;
AssembledProblem a = assembler.assemble(meshed, 0.0);
// static: a.stiffness().free * U = a.right_hand_side()
AssembledProblem b = assembler.assemble(meshed, 0.1, &a); // reuses a
```

The system in the free unknowns (`MeshedProblem::free_nodes()`, the
Dirichlet nodes eliminated) is

```
M2 U'' + B U' + A U = b,   B = M1 - D,   A = -K + C + R - S + T,
b = (F - G) - A_FD U_D - B_FD U_D' - M2_FD U_D''
```

`mass()`, `damping()` and `stiffness()` are `Operator`s, each with a
`free` matrix (free rows and columns) and a `dirichlet` matrix (free rows,
column k is `dirichlet_nodes()[k]`). All `free` matrices share one
`SparsityPattern` object and all `dirichlet` matrices another; a solver can
view the arrays with `Eigen::Map` (the index type is signed). `load()` is
F - G, `dirichlet_values()`, `dirichlet_rates()` and
`dirichlet_accelerations()` are U_D, U_D' and U_D'' (rates only for
`time_order() >= 1`, accelerations only for 2, otherwise empty), and
`right_hand_side()` is b (a lifting product is left out when its operator
is zero).

**Terms.** Interior, per triangle: K (a_xx, a_xy, a_yy), C (b and
`divergence_of_a()`), R (c), M1 (m1), M2 (m2), F (f). Boundary, per edge
of a natural or dynamic region: S (alpha), T (the tangential part), D
(gamma), G (g). On an edge with outward normal n and tangent tau (the
domain on the left), with κ_n = nᵀAn, κ_τ = nᵀAτ,
β̂_n = beta_n + (beta_x, beta_y)·n and β̂_τ = beta_tau + (beta_x, beta_y)·τ,
the condition is solved for u_n and substituted into (A∇u)·n = κ_n u_n +
κ_τ u_τ:

```
S_ij = ∫ κ_n alpha / β̂_n φ_i φ_j      T_ij = ∫ (κ_τ - κ_n β̂_τ / β̂_n) φ_i ∂τ φ_j
D_ij = ∫ κ_n gamma / β̂_n φ_i φ_j      G_i  = ∫ κ_n g / β̂_n φ_i
```

β̂_n = 0 at a quadrature point throws. Dirichlet regions have no boundary
term. Quadrature: barycentric (2/3, 1/6, 1/6) and rotations on triangles
(degree 2), 2-point Gauss–Legendre on edges.

**Facts.** Per term, from the facts of the coefficients feeding it
(zero as written, depends on t): K from A; C from b and div A; a boundary
term is zero when A is (κ = 0) or its own coefficient (alpha, gamma, g)
is, T only when A is; it depends on t when one of its non-zero factors
(including β̂_n, and β̂_τ for T) does. An operator `is_zero` when every
term of it is zero and `depends_on_t` when one of them depends on t;
`time_order()` is 2 with a non-zero mass, else 1 with a non-zero damping,
else 0 (lower than the problem's when a dynamic condition has A = 0).
`load_depends_on_t()` reads F and G; `right_hand_side_depends_on_t()`
adds the lifting products: a non-zero operator with Dirichlet nodes whose
values (g / alpha of a used region) or, for the stiffness, whose own terms
depend on t.

**Structure** (private, `detail::AssemblyInternals` in `assembler.cpp`,
shared through `previous`): the free index and the Dirichlet column of
every node; the two patterns, from one loop over the triangles (every node
pair (p, q) with p free adds column q to row p in the free or the
Dirichlet pattern; rows sorted, duplicates removed; boundary edges add
nothing); a greedy colouring of the triangles in index order (the smallest
colour not used by an earlier triangle sharing a node) and, in a separate
pass, of the boundary edges; the constant parts of the operators and load
that have terms depending on t.

**Building.** The driver loops over the colours, inside a colour in index
order. For each element it calls a pure block function
(`detail::triangle_blocks`, `detail::edge_blocks` in
`src/fem/element_blocks.cpp`), which computes the full 3 x 3 (2 x 2)
blocks of the requested terms, summed per operator and split into a
constant set and a time set; then it scatters them: a Dirichlet row is
skipped, a Dirichlet column goes to the `dirichlet` matrix, the rest to
`free`, each slot found by binary search in its row. A constant term goes
to the constant arrays, a term depending on t to the time arrays. Each
total is the constant part, or constant + time elementwise when the
operator has terms in t (the constant part is then kept for reuse). With
`previous` (same `MeshedProblem` object, else `std::invalid_argument`),
the structure and the constant parts are taken over and only the terms in
t are computed; an operator without such terms is copied from `previous`.
The result is the same bit for bit as a fresh assembly at that t.

**Report** ("assembly"): time; metrics `free_nodes`, `dirichlet_nodes`,
`free_nonzeros`, `dirichlet_nonzeros`, `triangle_colours`, `edge_colours`,
`used_previous`, and `mass_rebuilt`, `damping_rebuilt`,
`stiffness_rebuilt`, `load_rebuilt` (1 when something of it was computed,
0 when reused whole).

## Determinism

Results never depend on addresses or hash order (SymEngine's sets of
symbols are sorted by name before use): loops, curves, nodes,
triangles and boundary edges are processed in index order, queues are FIFO
or ordered by explicit keys with index tie-breaks, maps are ordered, and
`compact()` keeps the index order of what remains.
The same input gives the same mesh, bit for bit.

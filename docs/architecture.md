# Architecture

How the code is organized now. The decisions behind it are in
[decisions.md](decisions.md); where it is heading is in
[vision.md](vision.md).

## Modules

```
include/pde2d/geometry/   src/geometry/   ->  library pde2d_geometry
include/pde2d/mesh/       src/mesh/       ->  library pde2d_mesh
include/pde2d/io/         src/io/         ->  library pde2d_io
```

`pde2d_mesh` depends on `pde2d_geometry`, and `pde2d_io` on `pde2d_mesh`
(so on both), never the reverse. Public headers are in
`include/pde2d/<module>/`; everything in `src/` is private, including the
private headers there (`src/mesh/*.hpp`, `src/io/*.hpp`). `pde2d_geometry`
and `pde2d_mesh` have no external dependencies; `pde2d_io` links libxml2
privately (only `src/io/svg_import.cpp` includes it). The tests use
Boost.Multiprecision (header-only) in one place:
`geometry_predicates_oracle` checks the adaptive predicates against an
arbitrary-precision evaluation. The viewer tests need Node.js and are
skipped without it.

Namespaces: `pde2d::geometry`, `pde2d::mesh` and `pde2d::io`.

## Geometry

| Type | Role |
| --- | --- |
| `Point` | a point or vector, with `+ - * dot cross norm distance` |
| `Line`, `QuadraticBezier`, `CubicBezier`, `EllipticalArc` | exact curves, parameter `t` in [0, 1] |
| `Curve` | `std::variant` of the four curve kinds |
| `BoundaryCurve` | a `Curve` and its region name |
| `Loop` | a closed list of `BoundaryCurve`s; built by the user |
| `CurveGeometry` | a validated set of loops with computed depth and parent |

Free functions on `Curve` (`point_at`, `split`, `subcurve`, `reversed`,
`transformed`, `curve_length`, `parameter_at_length_fraction`,
`derivative_bounds`) are ported from the archive. `orient` and `incircle`
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
| `MeshSpec{max_edge_length}` | the mesher's only external input besides the geometry |
| `CurveChain` | consecutive exact curves treated as one path, measured by arc length `s`; cumulative lengths, `position(s)`, `point_at(s)` |
| `CurveLink{chain, s0, s1}` | which chain (index into the holder's chain list) and arc-length interval an edge approximates, `0 <= s0 < s1 <= length`; in `curve_chain.hpp`, with `default_corner_angle_degrees` (30) |
| `BoundaryPolygon` | loops of straight edges; per edge a region id and an optional `CurveLink`; the chains; region names |
| `WorkingMesh` | the mutable mesh the stages edit |
| `Mesh` | the output: `nodes`, `Triangle{nodes}`, `BoundaryEdge{nodes, region, link}`, `region_names`, `chains` (links and chains empty for an imported mesh); `validate(const Mesh &)` checks it like `WorkingMesh::from_mesh` |
| `ImproverUse` | `local_and_final`, `local_only`, `final_only`: in each improver's options |
| `PipelinePlan` | resolved at construction: indices of the local improvers and of the final passes, and the warnings |
| `StageReport` | `stage`, `time`, `messages`, `metrics` (name → number) |
| `MeshResult` | `mesh`, `reports` (the first is always "pipeline"), `target_met`, `longest_edge` |

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
its chord by more than `max_curve_deviation`. For a piece across
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
the final mesh at most `max_edge_length` (`target_met`, `longest_edge`).
Failures are exceptions with specific messages; report messages only mean
"worked, but…".

### v1 components

| Component | File | Notes |
| --- | --- | --- |
| `ChordLengthDiscretizer` | `chord_length_discretizer.*` | chains sampled by equal arc length, chord ≤ h; options `max_curve_deviation` (0.02), `corner_angle_degrees` (30) |
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

## Determinism

Results never depend on addresses or hash order: loops, curves, nodes,
triangles and boundary edges are processed in index order, queues are FIFO
or ordered by explicit keys with index tie-breaks, maps are ordered, and
`compact()` keeps the index order of what remains.
The same input gives the same mesh, bit for bit.

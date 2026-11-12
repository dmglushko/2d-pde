# Architecture

How the code is organized now. The decisions behind it are in
[decisions.md](decisions.md); where it is heading is in
[vision.md](vision.md).

## Modules

```
include/pde2d/geometry/   src/geometry/   ->  library pde2d_geometry
include/pde2d/mesh/       src/mesh/       ->  library pde2d_mesh
```

`pde2d_mesh` depends on `pde2d_geometry`, never the reverse. Public
headers are in `include/pde2d/<module>/`; everything in `src/` is private,
including the private headers there (`src/mesh/*.hpp`). The libraries have
no external dependencies. The tests use Boost.Multiprecision (header-only)
in one place: `geometry_predicates_oracle` checks the adaptive predicates
against an arbitrary-precision evaluation.

Namespaces: `pde2d::geometry` and `pde2d::mesh`.

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
arithmetic could overflow or underflow. `predicates.cpp` is compiled with
`-fno-fast-math -ffp-contract=off` and refuses to compile under
`__FAST_MATH__` or with `FLT_EVAL_METHOD != 0` (x87 extended precision),
because the error-free transformations need every operation rounded on
its own. `polygon.hpp` has `signed_area`,
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
without the geometry behind it.

| Type | Role |
| --- | --- |
| `MeshSpec{max_edge_length}` | the mesher's only external input besides the geometry |
| `BoundaryPolygon` | loops of straight edges; per edge a region id and an optional `CurveLink`; the linked curves; region names |
| `CurveLink{curve, t0, t1}` | which curve (index into the holder's curve list) and parameter interval an edge approximates |
| `WorkingMesh` | the mutable mesh the stages edit |
| `Mesh` | the output: `nodes`, `Triangle{nodes}`, `BoundaryEdge{nodes, region}`, `region_names` |
| `StageReport` | `stage`, `time`, `messages`, `metrics` (name → number) |
| `MeshResult` | `mesh`, `reports`, `target_met`, `longest_edge` |

### WorkingMesh

- Triangles are counterclockwise. Edge `i` of a triangle is opposite its
  vertex `i` (from vertex `i+1` to `i+2`), so the triangle is on its left.
- `neighbour(t, i)` is the triangle across edge `i`, or `no_index`.
- An edge without a neighbour is exactly one boundary edge, oriented with
  the domain on its left, with a region id and an optional curve link.
  Boundary loops never touch, so each boundary node has one outgoing
  boundary edge.
- Edits (`flip`, `insert_in_triangle`, `split_interior_edge`,
  `split_boundary_edge`, `try_move_node`) keep all of this. They never
  delete a triangle or node; they return the triangles they touched. An
  edit that would invert a triangle throws and leaves the mesh unchanged.
- A split boundary edge keeps its region, and its link is split into two
  halves of the parameter interval. The new point is the curve point at
  the middle parameter when the edge is linked to a curved curve, else the
  chord midpoint.
- The constructor validates everything (orientation, conformity, boundary
  edges matching the triangulation, no unused nodes, indices in range).
- `to_mesh()` lists boundary edges loop by loop in walking order.

### Components and capabilities

Every stage is a virtual interface with `name()`. Each concrete component
takes its own options struct (designated initializers) in its constructor.

```cpp
class BoundaryDiscretizer { BoundaryPolygon run(geometry, spec, report); };
class InitialTriangulator { TriangulatorCapabilities capabilities();
                            WorkingMesh run(polygon, report); };
class Refiner             { RefinerCapabilities capabilities();
                            const LocalImprover *local_improver();
                            void run(mesh, spec, report); };
class Improver            { ImproverEffects effects();
                            void run(mesh, spec, report); };
class LocalImprover       { ImproverEffects effects();
                            size_t improve_around(mesh, touched); };
```

Declarations:

- `ImproverEffects{flips_edges, moves_nodes, makes_delaunay}`;
- `RefinerCapabilities{tolerates_flips, tolerates_node_moves,
  maintains_delaunay}` (what the refiner accepts from its local improver,
  and what its output keeps);
- `TriangulatorCapabilities{supports_holes, supports_several_pieces}`.

Checks:

1. **At construction** (`check_compatibility`, called by `Remesher`, so
   also by `Mesher`): the local improver's effects must be tolerated by
   its refiner; an improver that makes the mesh Delaunay is rejected when
   the mesh already is (after a refiner that maintains Delaunay, or after
   another such improver with no flip or move in between), for example
   "ruppert already maintains Delaunay; delaunay_flips is redundant here".
2. **When a run starts** (`Mesher::run`): the spec is valid, and the
   triangulator supports the geometry's holes and pieces.

Between stages the `Mesher` validates the `BoundaryPolygon` (no crossing
chords, orientation by depth, nesting), so a discretizer's mistake is
reported as such. After the last stage it checks the target: every edge of
the final mesh at most `max_edge_length` (`target_met`, `longest_edge`).
Failures are exceptions with specific messages; report messages only mean
"worked, but…".

### v1 components

| Component | File | Notes |
| --- | --- | --- |
| `ChordLengthDiscretizer` | `chord_length_discretizer.*` | equal-arc-length pieces, chord ≤ h; option `max_curve_deviation` |
| `EarClipping` | `ear_clipping.*` | per piece: bridge holes, cut best-shaped ears first |
| `Cdt` | `cdt.*` | incremental Delaunay in an enclosing triangle, edge recovery by flips, outside removed, final flips |
| `Ruppert` | `ruppert.*` | makes its input Delaunay, splits encroached/long boundary edges, inserts circumcenters; options `min_angle_degrees`, `max_points` |
| `LongestEdgeBisection` | `longest_edge_bisection.*` | Rivara's longest-edge propagation path; options `local_improver`, `max_points` |
| `DelaunayFlips` | `delaunay_flips.*` | Lawson flips; both `Improver` and `LocalImprover` |
| `LaplacianSmoothing` | `laplacian_smoothing.*` | interior nodes to neighbour average, inverting moves skipped; option `iterations` |

Shared routines that components call are plain functions in
`delaunay.hpp`: `is_locally_delaunay`, `flip_to_delaunay`,
`flip_to_delaunay_around`.

### Remesher with geometry

`Remesher::run(mesh, geometry, spec)` first links each boundary edge to
the curve of its region that contains both end points (within 1e-9 of the
mesh size). Edges whose end points are on two different curves stay
unlinked and are split at chord midpoints; the "curve_links" report counts
them. A mesh region missing from the geometry, or a boundary node off its
region's curves, is an error.

## Determinism

Results never depend on addresses or hash order: loops, curves, nodes,
triangles and boundary edges are processed in index order, queues are FIFO
or ordered by explicit keys with index tie-breaks, and maps are ordered.
The same input gives the same mesh, bit for bit.

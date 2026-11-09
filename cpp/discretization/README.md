# Geometry and triangle meshing

The header-only C++23 API separates analytical geometry, boundary sampling, and
interior triangulation. A domain is one outer closed loop plus zero or more
inner loops (holes); every loop is simple and may be concave or curved. Every
triangle has three vertex indices (P1). See [Domains with holes](#domains-with-holes).

```cpp
#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "discretization/mesh_io.hpp"
using namespace pde2d::discretization;

RectangleGeometry shape(2.0, 1.0);
MeshOptions options;
options.boundary.max_edge_length = 0.4;
options.refinement.max_edge_length = 0.5;
auto result = generate_mesh(shape, options);
if (!result.report.size_satisfied)
    throw std::runtime_error(result.report.message);
export_mesh(result, "shape.pde2d");
auto restored = import_mesh_result("shape.pde2d");
options.refinement.max_edge_length = 0.3;
options.boundary_treatment = BoundaryTreatment::RefineOnCurves;
auto finer = refine_mesh(restored, options);
```

See [Extending analytical geometry](EXTENDING.md) for curve contracts, proven
interval bounds, format adapters, and stable serialization IDs.

## Free functions and options

`generate_mesh(CurveGeometry | Boundary | DiscretizedBoundary, MeshOptions = {})`
returns `MeshResult`: `.mesh`, optional `.boundary_context`, `.options`, and
`.report`. Include `triangulate.hpp` for the meshing entry points. Its private
helpers (`mesh_detail`) live in `detail/`: `ear_clipping.hpp` (seed
triangulation), `cdt_topology.hpp` (incremental constrained Delaunay topology,
flips, splits, priority queue of edges failing the size criterion,
`cdt_verified`), `curved_boundary.hpp` (splitting source intervals and
rebuilding on analytical curves) and `refinement_policy.hpp` (size criteria
and one policy per boundary treatment, see [EXTENDING](EXTENDING.md)).

`discretize_boundary(geometry, BoundaryOptions = {}, bool preserve_regions = true)`
returns `DiscretizedBoundary { boundary, context }`. Analytical source curves and
intervals survive sampling, so later refinement can place vertices on curves.
A raw `Boundary` has no analytical context.

| Option | Default | Meaning |
| --- | --- | --- |
| `algorithm` | `InteriorAlgorithm::ConstrainedDelaunay` | Or `EarClipping`. |
| `boundary_treatment` | `BoundaryTreatment::Fixed` | Or `SplitPolygonEdges`, `RefineOnCurves`. |
| `preserve_regions` | `true` | Keep tags and protect region junctions during sampling. |
| `boundary.preserve_corners` | `false` | Protect geometric corners during sampling. Region junctions remain protected when regions are preserved. |
| `boundary.max_edge_length` | `1.0` | Initial analytical boundary length bound. |
| `boundary.max_curve_deviation` | `0.02` | Initial absolute curve-to-chord deviation bound. |
| `refinement.max_edge_length` | `0.0` | Maximum triangle-edge length target; zero disables it. |
| `refinement.size_field` | empty | Optional `SizeField` (`double(Point2)`): a local length target. Not serialized. |
| `refinement.max_steps` | `10000` | Maximum midpoint insertion steps; zero allows none. |

Length/deviation constraints must be finite and nonnegative; zero disables each
positive constraint. A curved boundary rejects both boundary constraints set to
zero. `max_steps` is nonnegative. Boundary tolerances control initial sampling;
the refinement target controls all triangle edges.

Ear clipping triangulates the sampled polygon without interior refinement.
Selecting ear clipping with a positive interior target or a size field throws.
CDT starts with ear clipping and fully legalizes eligible interior edges under
the constrained Delaunay criterion. Size refinement inserts the midpoint of
the edge that fails its target worst and legalizes again. It is not classical
circumcenter quality refinement: there is no minimum-angle control or
guarantee, and no smoothing.

### Graded meshes: size fields

`refinement.size_field` gives a spatially varying target. An edge `(a, b)`
meets it when `|b - a| <= min(h(a), h((a+b)/2), h(b))` (times `1 + 1e-12`);
with a positive `max_edge_length` as well, the smaller target applies.
Without a field, edges are ranked by length; with one, by length divided by
the local target (ties by length), so a constant field gives exactly the
scalar result. Field values must be finite and positive, otherwise
refinement throws `Size field must return a finite positive length`.
`report.size_satisfied` states whether every edge met its local target; a
Fixed boundary edge that exceeds it returns `false` with "Fixed boundary edge
exceeds the local size target". The field is a request-time option:
`export_mesh` writes only the scalar options, so an imported result has an
empty field (see [the format](../../docs/formats/pde2d-mesh.md)).

```cpp
options.boundary_treatment = BoundaryTreatment::RefineOnCurves;
options.refinement.size_field = [](Point2 p) { return 0.05 + 0.3 * p.x; };
auto graded = generate_mesh(shape, options);
```

| Boundary treatment | During size refinement |
| --- | --- |
| `Fixed` | Keep every initial boundary vertex and edge. An oversized fixed edge makes the target impossible. |
| `SplitPolygonEdges` | Add midpoints on existing straight chords; keep the sampled polygon. |
| `RefineOnCurves` | Split source intervals and place new boundary vertices on analytical curves; requires context. |

`refine_mesh(TriangleMesh, MeshOptions = {}, optional<BoundaryContext> = nullopt)`
and `refine_mesh(MeshResult, MeshOptions)` run CDT refinement. The result overload
carries forward boundary context; pass the desired options explicitly.
`subdivide_mesh(TriangleMesh, MeshOptions = {}, optional<BoundaryContext> = nullopt)`
performs straight polygon subdivision without Delaunay legalization. Curve mode
is unsupported for subdivision.

### Marked refinement

`refine_mesh(MeshResult, std::vector<std::size_t> marked, MeshOptions)`
refines the marked triangles (indices into `result.mesh.elements`, for
example from `pde2d::fem::mark_bulk`). Each marked triangle selects its
longest edge and the edge is bisected with the CDT machinery, so the
neighbour across it is split too and the mesh stays conforming and Delaunay.
The boundary treatment decides boundary edges: `Fixed` selects the longest
interior edge instead (a triangle with only boundary edges is unrefinable),
`SplitPolygonEdges` splits the chord, `RefineOnCurves` puts the new vertex on
the source curve. Region tags and the boundary context carry over. Interior
selections are processed before boundary ones; a selected edge that an
earlier insertion flips away (or a curved rebuild retriangulates) is
*absorbed*, because its triangle already changed. A size target in the
options is enforced afterwards. `report.marked` (`MarkedRefinementReport`,
not serialized) counts marked triangles, selected, bisected and absorbed
edges and unrefinable triangles; `report.size_satisfied` is false if a
triangle was unrefinable, the budget ran out or the size target was missed.
The input should be a constrained Delaunay mesh (as `generate_mesh` and
`refine_mesh` return); otherwise the initial legalization may absorb
selections.

### Point location

`point_location.hpp`: `PointLocator(mesh, PointLocationOptions{})` builds a
box index over the triangles; `locate(p)` returns
`std::optional<PointLocation>` with `element`, `barycentric` (relative to
`mesh.elements[element]` in vertex order), `inside` and `distance`.
Containment uses exact predicates, with no tolerance; points on shared
edges or vertices go to the lowest-index triangle. Points outside the closed
mesh are accepted only within `max_outside_distance` (default 0) of the
boundary: they get the triangle owning the nearest boundary edge and its
unclamped barycentric coordinates (linear extrapolation, exact for affine
fields), `inside = false` and the distance. Farther points give
`std::nullopt`. `locate_point(mesh, p, options)` is the one-shot form. The
FEM module's `transfer_field` builds on it.

Always inspect `report.size_satisfied` and `report.message`. A fixed-boundary
conflict returns `false` with an explanation. Exhausting `max_steps` returns a
partial mesh with an explicit unmet-size report. Invalid inputs throw.
`report.refinement_steps` counts insertions; `report.delaunay_verified` describes
the CDT check, not a minimum-angle certificate.

## Domains with holes

A domain is an outer loop and zero or more holes. The single-outline
constructors and overloads are the zero-hole case and behave exactly as
before (same meshes, same files).

```cpp
AnnulusGeometry ring(1.0, 0.4);                 // regions "outer", "inner"
CurveGeometry plate({{"wall", outer_curves}},   // outer loop
                    {{{"cylinder", {circle}}},  // hole 1
                     {{"slot", slot_curves}}}); // hole 2
Boundary raw{outer_points, outer_tags, {{hole_points, hole_tags}}};
```

- **Representation.** `CurveGeometry(outer, holes)` and `setRegions(outer,
  holes)` take one `std::vector<CurveRegion>` per loop. Curves are stored loop
  by loop, outer first; `BoundaryCurve::loop` is 0 for the outer loop and
  `1 + i` for hole `i`, and `loopCount()` counts loops. `Boundary` (the
  polygon form) keeps the outer loop in `points`/`edge_tags` and adds
  `holes` (`BoundaryLoop {points, edge_tags}`); a hole with empty tags is
  untagged.
- **Regions** are global names: a region may have pieces on several loops,
  then it is disconnected (`pieces` lists one piece per loop run, joined
  across each loop's own stored seam). Runs and pieces never continue from one
  loop into the next. Editing (rename, move, split by curve ID or connected
  region, resize replay) works unchanged on hole curves; split pieces keep
  their loop.
- **Orientation** is neither normalized nor rejected: each loop may run either
  way and keeps the direction it was given, because stored direction defines
  region pieces and split fractions. Meshing, validation and FEM normals do
  not depend on it; mesh boundary edges follow each loop in its given
  direction.
- **Nesting** is validated on the sampled polygon (`discretize_boundary`,
  `generate_mesh(Boundary)`), with these messages: each loop simple (the
  single-loop messages), "Boundary loops intersect or touch" (any contact
  between two loops, including a shared vertex), "Hole is not inside the
  outer boundary", "Holes must not be nested". Analytical curves are not
  checked for contact beyond their sampling, as for single loops.
- **Sampling** (`discretize_boundary`) runs per loop: region junctions and
  corners are anchored per loop, a loop of one or two curves is still split
  into a polygon, and each chord keeps its source intervals.
  `BoundaryContext::polygon_snapshot` holds all loops one after another with
  `snapshot_loop_sizes` (empty for one loop), and `curves[i].loop` names the
  mesh boundary loop each source curve is sampled on.
- **Triangulation.** Ear clipping joins every hole to the outer ring by a
  bridge from the hole's rightmost vertex to the nearest visible ring vertex
  (holes in decreasing order of that vertex; visibility with the exact
  predicates, so the bridge touches no loop edge or earlier bridge), then
  clips the resulting weakly simple ring. CDT legalization and refinement
  start from that seed; every loop's edges are constrained, bridges are
  ordinary interior edges. `Fixed`, `SplitPolygonEdges` and `RefineOnCurves`
  treat hole edges like outer edges, so size targets, size fields and marked
  refinement put new hole vertices on the hole curves. Curve rebuilds
  traverse each loop separately.
- **Mesh boundary.** `TriangleMesh::boundary_edges` lists the edges of every
  loop; generated meshes list them loop by loop, outer first.
  `boundary_loops(mesh)` returns the loops as boundary-edge index lists
  (outer loop first: the one through the lexicographically least boundary
  vertex; holes by lowest edge index).
- **Point location** treats a hole as outside the mesh: a point in a hole
  gets `std::nullopt`, or with `max_outside_distance` the triangle of the
  nearest (hole) boundary edge. `transfer_field` works unchanged.
- **Files.** Meshes with holes are written as version 5 of the format (below);
  one-loop meshes keep versions 4 and 2.

## Geometry and boundary regions

Built-in defaults are circle radius 1, ellipse semi-axes 2 and 1, rectangle
2 by 1, capped rectangle 2 by 1 with a positive-x semicircle, regular hexagon
of side length 1, and an annulus with radii 1 and 0.5 (`AnnulusGeometry`,
regions `outer` and `inner`, `setRadii` requires inner < outer). `PolygonGeometry` takes ordered local vertices; its closing
edge is implicit. Translation uses `setCenter`, and `setRotation` takes
counterclockwise radians about that center. The cap diameter is the rectangle
height; total capped length is width + height/2.

Control sampling with `BoundaryOptions` passed to `discretize_boundary` or
`MeshOptions::boundary` passed to `generate_mesh`. Geometry overloads accept
`CurveGeometry`, including every built-in shape and SVG imports; supply a raw
`Boundary` for polygon-only input. `Geometry2D` holds transforms and has no
sampling interface. Regions can contain disconnected pieces on the same
outline. Empty tag vectors mean untagged edges; individual empty strings are
also untagged. Rectangle tags are `bottom`, `right`, `top`, `left` in local
coordinates. Disabling `preserve_regions` clears mesh tags and permits region
junctions to disappear. Set `boundary.preserve_corners` to retain geometric corners
independently; otherwise the enabled boundary bounds determine whether they can be removed.

Use names for whole-region operations and stable IDs for individual curves:

```cpp
RectangleGeometry g(4, 2);        // The same editing API supports SVG imports.
auto id = g.curveIdAt("bottom", 0);
g.renameRegion("bottom", "wall");
auto children = g.splitCurve(id, 0.4, "inlet", "wall");
g.moveCurve(children[0], "outlet");
g.mergeRegionInto("outlet", "wall");
g.splitRegion("wall", {0.25, 0.75}, {"first", "middle", "last"});
g.renameRegion("first", "middle"); // Existing names combine automatically.
```

The `splitRegion` operation requires a connected `wall` region. A region is the group
of all curves with that name, even if its pieces are disconnected. Renaming to
an existing name merges membership; moving a curve can create a region or
remove an emptied source region. Adjacent matching tags form one continuous
piece, including across the boundary's end/start. They retain separate curves
and IDs. Disconnected regions cannot be split by name: select an individual
curve ID instead. `regionGroups()` and `region(name)` report names, curve IDs,
and continuous pieces without evaluating lengths. `curves()` and `curve(id)`
expose the ordered records and local-coordinate geometry. `curveIdAt(name, i)`
resolves a position within a named group in stored boundary order; inspection
references and positional indices must be refreshed after edits.

`CurveId` is a strong, geometry-local handle assigned on import or construction.
Renaming, merging, and moving preserve IDs. Splitting a curve replaces it with
new child IDs and returns them in traversal order; the old ID fails lookup.
Region splitting preserves IDs of curves it only relabels and returns all
resulting selected curve IDs. Unselected curves retain their IDs. IDs are never
recycled for unrelated fragments. Copies inherit IDs and then evolve
independently; do not use an ID from another geometry. Mesh results serialize source IDs in a boundary-context snapshot for
continued refinement; they do not serialize editable geometry or its edit history.
Independent geometry imports assign their own IDs.

New split fractions measure cumulative arc length, across all selected curves.
Multiple fractions refer to the original selected length: `{0.25, 0.75}`
creates 25%, 50%, and 25% portions. Fractions must be finite, strictly increasing,
and inside (0, 1), with one more output name than cuts. Output names can match
existing names or each other. Cuts retain the exact underlying curves, using
numerical length inversion followed by exact subdivision. Lines/circular arcs
use analytic lengths; elliptical arcs and Beziers use adaptive speed integration
with a scale-relative tolerance (target roughly 1e-10 relative accuracy).
`length(id)` lazily caches each total curve length; `regionLength(name)`
sums a group's lengths, including disconnected pieces. Mesh resolution does
not affect these calculations. Zero-length selections, unresolved cuts near
junctions, numeric overflow, or failure to converge are rejected atomically.
Cuts within 2e-11 of an existing internal junction, relative to selected total
length, snap to that junction; cuts that would erase a requested portion fail.

A connected region starts where its tag changes from another tag and follows
stored curve direction. A region spanning the stored seam therefore starts
before the seam. A whole-loop region uses the stored first curve's start point
as its fraction-zero anchor. Neither merging nor renaming rotates the boundary.

Built-in dimension setters preserve editing history and reapply fractional cuts
to the resized geometry. Translation and rotation preserve lengths and IDs;
resizing recomputes lengths. A cut may cross an original segment junction:
surviving fragments keep their input-curve/output-interval identities, while
new fragments receive IDs. If an old fragment disappears, lookup fails; if the
same logical fragment returns on a later resize, its ID can become valid again.
If a later recorded edit needs a disappeared fragment, resizing fails with the
previous parameters and geometry intact. Structural changes such as changing
polygon side count or switching cap ends are rejected while edits exist when
source topology cannot be matched. For general polygons, vertex positions may
change with the same vertex count/order; that order defines source identity.

`setRegions(...)` explicitly replaces the geometry, clears edit history, and
assigns fresh IDs. It is not used by dimension setters. Region names are
validated immediately using the mesh format's ASCII token rules. All failed
editing operations leave the geometry unchanged.

Inspect boundary order through `curves()` and named groups through
`regionGroups()`. Positional operations count consecutive tagged runs; matching
end/start runs remain separate for these operations.
`splitRegion(regionIndex, segmentIndex, t, first, second)` still splits an entire
run at a curve's mathematical parameter, not an arc-length fraction.
`curveIdAt(regionIndex, segmentIndex)` resolves those positions to an ID.
`mergeRegions(first, count, name)` retains consecutive-range semantics without
wraparound. Use name-based merging for arbitrary groups and wraparound.


## Export and resume

`export_mesh(MeshResult, path)` writes version 4, including options, report, and
optional world-coordinate source-curve context for resumed curve refinement.
`import_mesh_result(path)` restores it. This snapshot is not an editable geometry
archive. `export_mesh(TriangleMesh, path)` writes version 2 with boundary tags.
A mesh with holes is written as version 5 by both overloads (boundary edges
grouped by loop, regrouped stably if needed; a raw mesh records no result).
`import_mesh(path)` reads topology only, dropping refinement context. Readers
accept versions 1 through 5 and reject holes in files older than version 5;
v1 edges are untagged. All use zero-based indices and
fixed `ORDER 1`. Nonempty tags use ASCII letters, digits, `_`, `.`, or `-`, with
the standalone `-` token reserved for untagged edges.

Validation checks indices, positive triangle area, shared-edge ownership and
orientation, exterior boundary records, one connected domain whose boundary is
one or more closed loops (Euler characteristic `V - E + F = 2 - loops`;
messages "Boundary is not a closed loop", "Disconnected mesh"), unused and
duplicate vertices, and crossing or overlapping edges. Conservative spatial
indexes prune edge-pair checks before robust predicates; overlapping boxes can
still yield quadratic work. Validation does not certify minimum angles. Meshing requires Boost
development headers for exact binary64 predicate fallbacks. The optional
[FEM module](../../docs/fem/README.md) assembles and solves scalar linear
equations on these meshes. See the [reference](../../docs/learning/reference.md)
and [rendered showcase](../../docs/showcase/README.md).

### SVG geometry import and export

Include `discretization/svg_geometry.hpp` and
`discretization/boundary_discretization.hpp`, and compile and link with
`pkg-config --cflags --libs libxml-2.0`. For example:

```cpp
namespace geo = pde2d::discretization;
auto geometry = geo::importSvg("boundary.svg",
    {2.0, geo::SvgScaleMode::Farthest, 64});
auto boundary = geo::discretize_boundary(geometry).boundary;
geo::exportSvg(geometry, "edited-boundary.svg");
```

Each `<path id="region" d="..."/>` supplies one tagged run per subpath (each
`M` starts a subpath; only `M` may follow `Z`). An optional
`data-region="name"` overrides the region name without changing the unique XML
ID. Several paths may carry the same region name, preserving disconnected
membership and runs crossing the stored boundary seam. Paths may
be unordered and reversed; their endpoints must join unambiguously into closed
loops. A single path may close with `Z`. Several closed loops become a domain
with holes: the loop with the largest (sampled) area is the outer loop, the
others are holes in file order; nesting is validated when the boundary is
sampled. Supported path commands are
absolute and relative `M`, `L`, `Q`, `C`, `A`, and `Z`, including repeated
argument groups. Coordinates retain the SVG coordinate direction. A single
optional `<circle id="origin" cx="..." cy="..."/>` sets the origin; otherwise
it is (0, 0). The marker radius is ignored.

`Factor` multiplies coordinates after subtracting the origin. `Nearest` and
`Farthest` make the corresponding distance from the origin equal to `scale`.
Distances are measured on the sampled polyline, with nearest measured to edges,
so increasing `SvgOptions::samples_per_curve` improves curved-distance
approximation. This option affects import scaling only; boundary sampling for
meshing uses `BoundaryOptions`.
`Nearest`/`Farthest` distances are measured over all loops. The importer
rejects zero distances, invalid paths, chains that do not close, transforms,
groups, other shapes, and unsupported attributes and commands. It does not apply viewBox, CSS, or units; use plain numeric coordinates
and export paths without presentation attributes. The triangulator validates
loop simplicity. libxml2 development headers and pkg-config are required for
SVG I/O and its tests. Include `discretization/boundary_discretization.hpp` for
the free sampling API in the example.

`exportSvg(const CurveGeometry&, const std::string& filename)` exports the
**current edited analytical geometry**, independently of mesh export (`.pde2d`).
An overload accepts `std::ostream&`. Both return `void`. Curves remain in stored
boundary order, with one path per consecutive named run of one loop; each loop
is exported as its own closed chain, outer loop first. Lines and quadratic
and cubic Beziers use `L`, `Q`, and `C`; elliptical arcs use `A`, split into
pieces of at most a quarter turn. Full ellipses/circles therefore use four
analytical arcs, never a polygon approximation. Segment subdivision changes
neither the curve locus nor its region membership. Coincident-end Beziers are
also subdivided analytically by de Casteljau until their endpoints are distinct.

Translation and rotation are baked into endpoints, control points, and ellipse
axes using the same world-coordinate convention as meshing. There is no Y-axis
flip, origin subtraction, origin marker, transform attribute, group, viewBox,
or presentation attribute. Default `importSvg` thus recovers world coordinates;
its resulting center and rotation are zero. Export uses locale-independent
decimal output with binary64 roundtrip precision; SVG endpoint-to-center arc
conversion is subject to floating-point roundoff. SVG viewing styles and canvas
layout are left to consumers.

For a unique region name starting with an ASCII letter or `_` (except reserved
`origin`), the path ID is the name, compatible with the original importer.
Otherwise export assigns collision-free `pde2d_run_N` IDs and writes the exact
name in `data-region`. This covers repeated names, digit/dot/hyphen-leading
names, and a region named `origin`. The importer now accepts this attribute;
older versions reject it. IDs always remain unique, including when a user has
named a region `pde2d_run_N`. Names retain CurveGeometry's ASCII validation.

SVG preserves shape, direction, order, and current region membership. It does
not archive geometry-local CurveIds, source-shape parameters, edit replay
history, or separate center/rotation settings. Import assigns fresh IDs, and
arc segmentation can increase the number of curves. Use imported IDs for
subsequent editing rather than retaining IDs from the exported object.

Export rejects empty, open, or disconnected boundaries, invalid/non-finite
curves, and numeric scales or degenerate curves the importer cannot resolve
(Bezier subdivision is limited to twelve levels). It checks continuity/closure using
the importer's tolerance; as with import, loop simplicity is validated by the
triangulator, not SVG I/O. Invalid geometry throws `std::invalid_argument`
before opening the destination or writing to a stream. Open/write/close failures
throw `std::runtime_error` (or a stream's configured I/O exception). A write
failure may leave partial output; file replacement is not atomic. Streams are
not flushed or closed by the stream overload; the caller owns those operations.

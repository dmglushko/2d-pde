# Geometry and triangle meshing

The header-only C++23 API separates analytical geometry, boundary sampling, and
interior triangulation. It supports one simple closed outline, including concave
polygons, without holes. Every triangle has three vertex indices (P1).

```cpp
#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "discretization/mesh_io.hpp"
using namespace discretization;

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

## Free functions and options

`generate_mesh(Geometry2D | Boundary | DiscretizedBoundary, MeshOptions = {})`
returns `MeshResult`: `.mesh`, optional `.boundary_context`, `.options`, and
`.report`. `mesh_generators.hpp` forwards to `triangulate.hpp`; the old five
mesh-generator classes and their setters have been removed.

`discretize_boundary(geometry, BoundaryOptions = {}, bool preserve_regions = true)`
returns `DiscretizedBoundary { boundary, context }`. Analytical source curves and
intervals survive sampling, so later refinement can place vertices on curves.
A raw `Boundary` has no analytical context.

| Option | Default | Meaning |
| --- | --- | --- |
| `algorithm` | `InteriorAlgorithm::ConstrainedDelaunay` | Or `EarClipping`. |
| `boundary_treatment` | `BoundaryTreatment::Fixed` | Or `SplitPolygonEdges`, `RefineOnCurves`. |
| `preserve_regions` | `true` | Keep tags and protect region junctions during sampling. |
| `boundary.max_edge_length` | `1.0` | Initial analytical boundary length bound. |
| `boundary.max_curve_deviation` | `0.02` | Initial absolute curve-to-chord deviation bound. |
| `refinement.max_edge_length` | `0.0` | Maximum triangle-edge length target; zero disables refinement. |
| `refinement.max_steps` | `10000` | Maximum midpoint insertion steps; zero allows none. |

Length/deviation constraints must be finite and nonnegative; zero disables each
positive constraint. A curved boundary rejects both boundary constraints set to
zero. `max_steps` is nonnegative. Boundary tolerances control initial sampling;
the refinement target controls all triangle edges.

Ear clipping triangulates the sampled polygon without interior refinement.
Selecting ear clipping with a positive interior target throws. CDT starts with
ear clipping and fully legalizes eligible interior edges under the constrained
Delaunay criterion. Size refinement inserts a longest-edge midpoint and fully
legalizes again. It is not classical circumcenter quality refinement: there is
no minimum-angle control or guarantee, smoothing, or local size field.

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

Always inspect `report.size_satisfied` and `report.message`. A fixed-boundary
conflict returns `false` with an explanation. Exhausting `max_steps` returns a
partial mesh with an explicit unmet-size report. Invalid inputs throw.
`report.refinement_steps` counts insertions; `report.delaunay_verified` describes
the CDT check, not a minimum-angle certificate.

## Geometry and boundary regions

Built-in defaults are circle radius 1, ellipse semi-axes 2 and 1, rectangle
2 by 1, capped rectangle 2 by 1 with a positive-x semicircle, and regular hexagon
of side length 1. `PolygonGeometry` takes ordered local vertices; its closing
edge is implicit. Translation uses `setCenter`, and `setRotation` takes
counterclockwise radians about that center. The cap diameter is the rectangle
height; total capped length is width + height/2.

Legacy segment-count arguments remain accepted but do not control meshing.
Use `BoundaryOptions` with `discretize_boundary` or `MeshOptions::boundary`.
Standalone `createBoundary()` and `discretizeBoundary(maxEdge, deviation)` remain
geometry conveniences. Regions can contain disconnected pieces on the same
outline. Empty tag vectors mean untagged edges; individual empty strings are
also untagged. Rectangle tags are `bottom`, `right`, `top`, `left` in local
coordinates. Disabling `preserve_regions` clears mesh tags and permits redundant
smooth region junctions to disappear; geometric corners remain protected.

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
independently; do not use an ID from another geometry. Version-3 mesh results serialize source IDs in a boundary-context snapshot for
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

For compatibility, `regions()` returns a snapshot of consecutive tagged runs
in boundary order; one disconnected name can occur multiple times, and matching
end/start runs remain separate in this positional view. The old
`splitRegion(regionIndex, segmentIndex, t, first, second)` still splits an entire
run at a curve's mathematical parameter, not an arc-length fraction.
`curveIdAt(regionIndex, segmentIndex)` resolves those positions to an ID.
`mergeRegions(first, count, name)` retains consecutive-range semantics without
wraparound. Use name-based merging for arbitrary groups and wraparound.


## Export and resume

`export_mesh(MeshResult, path)` writes version 3, including options, report, and
optional world-coordinate source-curve context for resumed curve refinement.
`import_mesh_result(path)` restores it. This snapshot is not an editable geometry
archive. `export_mesh(TriangleMesh, path)` writes version 2 with boundary tags.
`import_mesh(path)` reads topology only, dropping refinement context. Readers
accept versions 1, 2, and 3; v1 edges are untagged. All use zero-based indices and
fixed `ORDER 1`. Nonempty tags use ASCII letters, digits, `_`, `.`, or `-`, with
the standalone `-` token reserved for untagged edges.

Validation checks indices, positive triangle area, shared-edge ownership and
orientation, exterior boundary records, connected disk topology, unused and
duplicate vertices, and crossing or overlapping edges. The geometric edge-pair
check is quadratic; it does not certify minimum angles. Meshing requires Boost
development headers for exact binary64 predicate fallbacks. No PDE assembly, solver,
or error estimator is implemented. See the [reference](../../docs/learning/reference.md)
and [rendered showcase](../../docs/showcase/README.md).

### SVG import

Include `discretization/svg_geometry.hpp` and compile and link with
`pkg-config --cflags --libs libxml-2.0`. For example:

```cpp
auto geometry = discretization::importSvg("boundary.svg",
    {2.0, discretization::SvgScaleMode::Farthest, 64});
auto boundary = geometry.createBoundary();
```

Each `<path id="region" d="..."/>` supplies one tagged region. Paths may
be unordered and reversed; their endpoints must join unambiguously into one
closed loop. A single path may close with `Z`. Supported path commands are
absolute and relative `M`, `L`, `Q`, `C`, `A`, and `Z`, including repeated
argument groups. Coordinates retain the SVG coordinate direction. A single
optional `<circle id="origin" cx="..." cy="..."/>` sets the origin; otherwise
it is (0, 0). The marker radius is ignored.

`Factor` multiplies coordinates after subtracting the origin. `Nearest` and
`Farthest` make the corresponding distance from the origin equal to `scale`.
Distances are measured on the sampled polyline, with nearest measured to edges,
so increasing `samples_per_curve` improves curved-distance approximation.
The importer rejects zero distances, invalid or disconnected paths, transforms,
groups, other shapes, multiple subpaths, and unsupported attributes and
commands. It does not apply viewBox, CSS, or units; use plain numeric coordinates
and export paths without presentation attributes. The triangulator validates
loop simplicity. libxml2 development headers and pkg-config are required for
the SVG importer and its tests.

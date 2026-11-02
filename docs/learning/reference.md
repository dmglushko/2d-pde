# Source map and parameter reference

The public API is header-only C++23 in `discretization`. Use this beside the
[walkthrough](README.md) and [complete option table](../../cpp/discretization/README.md#free-functions-and-options).

| Source | Responsibility |
| --- | --- |
| [geometry_base.hpp](../../cpp/discretization/geometry_base.hpp) | `Point2`, `Boundary`, `TriangleMesh`, `Geometry2D`. |
| [geometry.hpp](../../cpp/discretization/geometry.hpp) | Built-in shapes, dimensions, transforms. |
| [curve_geometry.hpp](../../cpp/discretization/curve_geometry.hpp) | Named regions, stable `CurveId`, transactional edits and resizing. |
| [mesh_types.hpp](../../cpp/discretization/mesh_types.hpp) | `MeshOptions`, `BoundaryOptions`, `RefinementOptions`, `MeshResult`, `BoundaryContext`, `MeshReport`. |
| [boundary_discretization.hpp](../../cpp/discretization/boundary_discretization.hpp) | `discretize_boundary` and analytical sampling. |
| [triangulate.hpp](../../cpp/discretization/triangulate.hpp) | `generate_mesh`, `refine_mesh`, `subdivide_mesh`, `is_constrained_delaunay`. |
| [mesh_io.hpp](../../cpp/discretization/mesh_io.hpp) | `export_mesh`, `import_mesh`, `import_mesh_result`. |

## Parameters and methods

`MeshOptions{}` selects CDT, fixed boundary, and preserved regions. Initial
boundary bounds are length `1.0`, deviation `0.02`. Interior maximum edge length
is `0.0` (disabled), with an insertion budget of `10000`. Zero disables a positive
length/deviation constraint; curved sampling cannot disable both initial bounds.
Negative/nonfinite bounds and negative budgets are invalid.

`InteriorAlgorithm::EarClipping` triangulates without interior refinement and
rejects a positive refinement target. `ConstrainedDelaunay` fully legalizes the
triangulation; refinement inserts a longest-edge midpoint then legalizes again.
No minimum-angle parameter, local field, or smoothing is exposed. This is size
refinement, without the classical circumcenter quality-refinement guarantee.

`BoundaryTreatment::Fixed` keeps initial edges. `SplitPolygonEdges` subdivides
straight chords. `RefineOnCurves` adds points on source curves and needs retained
context. Boundary sampling tolerances and triangle-edge targets are independent.
A fixed boundary should be sampled finely enough for the later target.

`generate_mesh` accepts geometry, raw boundary, or discretized boundary.
`refine_mesh(result, options)` retains context; the raw-mesh overload accepts an
optional context. `subdivide_mesh(mesh, options, context)` subdivides straight
polygons without legalization and rejects curve mode. All return `MeshResult`.

Always check `.report.size_satisfied`: fixed-boundary conflicts and exhausted
step budgets return a mesh and explicit unmet-size message. A partial mesh is
usable topology, not evidence that the target succeeded. Invalid input throws.

## Geometry editing

Rectangle dimensions are full local lengths; ellipse A/B are semi-axis lengths.
Positive finite dimensions are required. Rotation is counterclockwise in radians,
translation defaults to `(0,0)`, and transforms leave stored dimensions intact.
Legacy segment counts do not control sampling. Stable curve IDs, named regions,
arc-length cuts, and edit replay on resizing remain unchanged; consult the
[full region-editing contract](../../cpp/discretization/README.md#geometry-and-boundary-regions).

## Format and limits

Exporting `MeshResult` writes v3, preserving options, report, and optional
analytical boundary context. `import_mesh_result` supports resuming curves.
Exporting raw `TriangleMesh` writes v2 (topology and tags). `import_mesh` accepts
v1/v2/v3 but returns only topology; v1 has no tags. Source IDs stored in a v3
snapshot do not recreate an editable geometry or its history. All elements are
three-node P1 triangles, indices start at zero, and `ORDER` must be `1`.

Import validates indices, finite coordinates, exact positive orientation,
edge ownership and direction, one connected disk, boundary records, unused or
duplicate vertices, and crossing or overlapping edges. It also verifies recorded
size/CDT guarantees and source-context consistency. Geometric validation includes
a quadratic edge-pair check; it does not certify a minimum angle. Geometry
accepts a single simple loop without holes. Floating-point coordinate precision
and finite work budgets limit near-degenerate cases. PDE assembly and solving
remain future work. Exact predicate fallbacks require Boost development headers.

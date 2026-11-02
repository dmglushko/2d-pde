# Read the 2D mesh code, one step at a time

Start with [first_mesh.cpp](../../examples/learning/first_mesh.cpp), a complete
C++23 program that constructs a rectangle, meshes it, exports a v3 result, and
imports that result. No external meshing library is needed.

```sh
g++ -std=c++23 -Wall -Wextra -Werror -Icpp examples/learning/first_mesh.cpp -o /tmp/first_mesh
/tmp/first_mesh /tmp/rectangle.pde2d
```

The optional argument selects the output path; otherwise it writes
`rectangle.pde2d` in the working directory.

## 1. Geometry describes the domain

`RectangleGeometry(2.0, 1.0)` stores full width and height in local coordinates.
`setCenter` translates the shape; `setRotation` rotates it counterclockwise in
radians about its center. Stored dimensions remain unchanged. Other built-ins
are circles, ellipses, capped rectangles, regular polygons, and general simple
polygons. The shape API supports one outline, including concavities, without
holes. See [geometry.hpp](../../cpp/discretization/geometry.hpp).

Named regions and stable curve IDs describe boundary pieces independently of
mesh resolution. The [region-editing API](../../cpp/discretization/README.md#geometry-and-boundary-regions)
still supports rename, move, merge, arc-length split, and edit replay on resize.
Changing meshing options does not change that editing contract.

## 2. Options choose sampling and triangulation

```cpp
discretization::MeshOptions options;
options.boundary.max_edge_length = 0.4;
options.refinement.max_edge_length = 0.5;
auto result = discretization::generate_mesh(geometry, options);
```

`MeshOptions{}` defaults to CDT with fixed boundary and region preservation.
Initial boundary length/deviation defaults are `1.0` and `0.02`; the interior
length target defaults to zero (disabled), with `max_steps=10000`. Zero disables
a positive length/deviation constraint, but a curved boundary needs at least one
enabled sampling bound. There is no minimum-angle control, smoothing, or local
size field. Read [mesh_types.hpp](../../cpp/discretization/mesh_types.hpp).

`discretize_boundary(geometry, boundary_options, preserve_regions)` samples the
analytical outline and retains source context. `generate_mesh` accepts geometry,
a raw `Boundary`, or that `DiscretizedBoundary`, returning a `MeshResult` by value.
A raw boundary has no analytical context. Read
[boundary_discretization.hpp](../../cpp/discretization/boundary_discretization.hpp).

## 3. Triangulation and refinement

`InteriorAlgorithm::EarClipping` triangulates the boundary polygon; a positive
interior target is rejected. `ConstrainedDelaunay` legalizes interior diagonals
and optionally inserts longest-edge midpoints followed by full legalization.
This is size refinement, without a classical circumcenter quality guarantee.
Read [triangulate.hpp](../../cpp/discretization/triangulate.hpp).

Boundary treatment is independent: `Fixed` keeps the sampled boundary,
`SplitPolygonEdges` splits its straight chords, and `RefineOnCurves` places new
boundary points on source curves. A fixed boundary needs sufficiently fine
initial sampling for the interior target. Curve mode requires retained context.
`preserve_regions=false` drops tags and allows redundant smooth region junctions
to disappear during initial sampling. Set `boundary.preserve_corners=true` to retain geometric corners.

Always inspect `result.report.size_satisfied` and `.message`. A conflicting fixed
boundary or exhausted step budget returns a mesh with an explicit unmet report;
invalid inputs throw. `refine_mesh(result, options)` resumes CDT refinement.
`subdivide_mesh(mesh, options, optional_context)` instead performs straight
polygon subdivision without legalization; it does not support curve mode.

## 4. Read the returned mesh

`result.mesh.nodes` holds coordinates. Each entry in `.elements` has three
zero-based indices into nodes. `.boundary_edges` contains exterior endpoint
pairs; `.boundary_tags` is an optional parallel vector of names. The first
outline point is not repeated at the end. Each triangle remains P1 after
refinement; PDE basis evaluation, assembly, and solving are future work.

## 5. Save and resume

`export_mesh(result, path)` writes v3 with options, report, and optional analytical
context. `import_mesh_result(path)` restores these for resumed curve refinement.
A raw `TriangleMesh` exports as v2, with topology and tags. `import_mesh(path)`
accepts v1/v2/v3 and returns topology only, dropping context. V1 has untagged
edges. All use fixed `ORDER 1`. A v3 boundary snapshot does not restore an
editable original geometry or its history.

The [I/O validator](../../cpp/discretization/mesh_io.hpp) checks local topology,
indices, positive areas, edge ownership and exterior records. Import success
does not prove global absence of overlap or good angles.

## Experiments and validation

Run the example, halve both size settings, rebuild, and compare counts. Then try
a coarse fixed boundary, chord splitting, and curve mode, reading the report each
time. Use the [questions](questions.md), [source map](reference.md),
[offline viewer](../../tools/mesh_viewer/README.md), and
[reproducible graphical showcase](../showcase/README.md).

The examples are compiled with C++23 and warnings as errors. The showcase README
records reproduction commands and dependency setup; the parent regression suite
is `./tools/test.sh`. Keep behavioral claims aligned with current headers when
changing this walkthrough.

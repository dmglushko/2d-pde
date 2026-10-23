# Source map and parameter reference

Use this beside the [walkthrough](README.md). Links point to this checkout's source, so a reader can confirm each statement. The public surface is header-only C++17 under the `discretization` namespace.

## Class and responsibility map

| Public name | Responsibility and first source stop |
| --- | --- |
| [`Point2`, `Boundary`, `TriangleMesh`](../../cpp/discretization/geometry.hpp) | Coordinate, one outline loop, and resulting mesh data. |
| [`Geometry2D`](../../cpp/discretization/geometry.hpp) | Base contract for `createBoundary`; common center and rotation. |
| [`CircleGeometry`, `EllipseGeometry`](../../cpp/discretization/geometry.hpp) | Polygonal approximations to curved outlines; see the [ellipse implementation](../../cpp/discretization/geometry.hpp). |
| [`RectangleGeometry`, `RectangleWithSemicircleGeometry`](../../cpp/discretization/geometry.hpp) | Four-corner rectangle or a rectangle with one polygonal semicircle cap; see the [cap implementation](../../cpp/discretization/geometry.hpp). |
| [`PolygonGeometry`, `RegularPolygonGeometry`](../../cpp/discretization/geometry.hpp) | User-supplied ordered local vertices or equal-sided polygon; see the [regular polygon implementation](../../cpp/discretization/geometry.hpp). |
| [`MeshGenerator`](../../cpp/discretization/mesh_generators.hpp) | Shared configuration and `createMesh(geometry)` entry point. Five concrete classes select algorithms in [the same header](../../cpp/discretization/mesh_generators.hpp). |
| [`MeshOptions`, `Triangulator2d`](../../cpp/discretization/triangulate.hpp) | Option defaults and implementation pipeline; see the [`generate` implementation](../../cpp/discretization/triangulate.hpp). |
| [`export_mesh`, `import_mesh`](../../cpp/discretization/mesh_io.hpp) | `.pde2d` write/read and mesh validation; see the [importer implementation](../../cpp/discretization/mesh_io.hpp). |

## Parameters you can change

| Setting | Current meaning and boundary |
| --- | --- |
| `setCenter(x,y)` | Translation of local shape coordinates; default `(0,0)`. Its setter does not itself validate finite values, but meshing checks boundary coordinates. |
| `setRotation(radians)` | Counterclockwise around the shape center; default `0`. Rejects a non-finite angle. |
| Rectangle `width`, `height` | Full lengths along local x and y; both finite and positive. Cap radius is `height/2`. |
| Circle `radius`; ellipse `A`, `B` | Radius and semi-axis lengths, finite and positive. Circle/ellipse boundary segments default to 64/96 and require at least 8. Cap segments default to 32 and require at least 4. More segments approximate curvature with more straight edges. |
| Regular polygon `sides`, `sideLength` | At least 3 sides and a finite positive side length. First vertex lies on local positive x before rotation. |
| `setMaxElementSize(h)` | Finite positive target in coordinate units; default `1`. The refinement test compares each triangle's **longest edge** against a target at its centroid. It is not an element-area setting. Plain ear clipping does not refine for it. |
| `setSizeField(f)` | A callable `f(x,y)` returning a finite positive local target, used only by `LocalSizeRefinementMeshGenerator`; capped above by `maxElementSize`. If unset, that generator uses the global target. |
| `setOrder(1 or 2)` | P1 three-node or P2 six-node connectivity; default `1`. Other values fail in `generate`. P2 construction adds straight-edge midpoint nodes after smoothing. |
| `setOptimizationIterations(n)` | Interior Laplacian smoothing passes; default `5`. Boundary nodes stay fixed, and a pass that would make a nonpositive-area triangle or violate its size target stops smoothing. `n<=0` skips smoothing; there is no upper-bound check. |
| `setMaxRefinementPasses(n)` | At most this many passes; default `12`, allowed range `0..20`. If the centroid/longest-edge target remains unmet after the passes, `generate` throws. |

Setters for mesher options mostly store values; validation in `generate` occurs when meshing. Refinement enforces the longest-edge target, and optional edge flips and smoothing reject changes that would violate it. This bound includes a small floating-point tolerance. It does not certify a minimum angle. A local size field must give consistent values for the same coordinates; it is sampled at triangle centroids rather than continuously over the region.

## Meshing methods

All five start with [boundary validation and ear clipping](../../cpp/discretization/triangulate.hpp), then the [refinement](../../cpp/discretization/triangulate.hpp), [flip](../../cpp/discretization/triangulate.hpp), and [smoothing](../../cpp/discretization/triangulate.hpp) stages indicated below. “First legal ear” means removing a three-vertex triangle from the current boundary loop that has no other loop vertex inside it. The quality score ranks candidate ears by area relative to squared edge lengths.

| Generator | Seed, refinement, flip | What to expect / limit |
| --- | --- | --- |
| [`EarClippingMeshGenerator`](../../cpp/discretization/mesh_generators.hpp) | First legal ear; no refinement or flips. | Simple baseline; `maxElementSize` and size field do not control triangle sizes. Smoothing is called but usually has no interior node to move. |
| [`QualityEarClippingMeshGenerator`](../../cpp/discretization/mesh_generators.hpp) | Best-score legal ear at each step; global target refinement; eligible interior edge flips. | Local choices may improve seed shapes, with no global optimality or minimum-angle guarantee. |
| [`UniformRefinementMeshGenerator`](../../cpp/discretization/mesh_generators.hpp) | First legal ear; global target refinement; no flips. | All regions use the same target; may make many triangles. “Uniform” refers to target size, not identical element shape or a fixed subdivision count. |
| [`DelaunayEdgeFlipMeshGenerator`](../../cpp/discretization/mesh_generators.hpp) | First legal ear; global target refinement; locally eligible interior edge flips. | Opposite-angle test improves local diagonals while boundary edges stay fixed. This is not a full constrained-Delaunay construction or a quality guarantee. |
| [`LocalSizeRefinementMeshGenerator`](../../cpp/discretization/mesh_generators.hpp) | First legal ear; centroid-sampled local target refinement; eligible interior flips. | Smaller requested sizes can concentrate triangles. Abrupt fields and finite passes may yield irregular transitions or a pass-limit error. |

[Refinement](../../cpp/discretization/triangulate.hpp) marks all three edges of a triangle whose longest edge exceeds its centroid target. Shared edge midpoint indices and one-, two-, or three-edge split patterns keep adjacent triangles conforming, and boundary edge records are split too. [Flipping](../../cpp/discretization/triangulate.hpp) skips exterior edges and non-convex quads; it stops when no eligible flip remains or reaches an iteration limit. [Smoothing](../../cpp/discretization/triangulate.hpp) averages interior neighbors and rejects a pass that would invert or flatten a triangle or violate its size target.

## Format and limits

The [writer](../../cpp/discretization/mesh_io.hpp) produces a text file with `PDE2D_MESH 1`, then `ORDER`, `NODES`, `ELEMENTS`, and `BOUNDARY_EDGES`. Indices start at zero. The [reader](../../cpp/discretization/mesh_io.hpp) accepts version 1 only, rejects malformed or truncated records, and uses [structural validation](../../cpp/discretization/mesh_io.hpp). The file does not carry shape parameters, mesher settings, physical tags, material IDs, boundary-condition labels, or a PDE solution. The boundary pairs list corner endpoints, including for P2 meshes.

The current validator checks local structure: valid indices, distinct nodes per triangle, positive finite corner triangle area, at most two triangle owners per edge with opposite traversal directions, exterior records matching one-owner edges, closed boundary degree, and shared P2 midpoint indices. P2 midpoints cannot be reused on distinct edges or as triangle corners. It does not check global triangle overlap, whether all nodes are used, whether a P2 midpoint lies geometrically at the edge center, or numerical mesh quality. These limitations matter if a file comes from outside this writer.

The shape API supports one simple closed outline, including concave polygons. It has no holes or disconnected pieces. The implementation uses fixed numeric tolerances for some boundary checks and finite pass/flip budgets, so near-degenerate input can fail. The P1/P2 labels describe mesh topology here; no PDE basis evaluation, assembly, solver, or error estimator is implemented in these headers.

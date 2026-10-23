# Built-in 2D geometry and triangle meshing

This subsystem creates a boundary from a built-in shape class, then creates a
triangle mesh from that geometry. It does not read CAD files. Shapes and mesh
generators are separate public objects so a later PDE constructor can accept a
configured `MeshGenerator` independently from the geometry.

## Example

```cpp
#include "discretization/geometry.hpp"
#include "discretization/mesh_generators.hpp"
#include "discretization/mesh_io.hpp"

using namespace discretization;

RegularPolygonGeometry shape(6, 2.0); // six equal sides, each of length 2
shape.setCenter(1.0, -0.5);
shape.setRotation(0.2);               // counterclockwise radians

LocalSizeRefinementMeshGenerator mesher;
mesher.setMaxElementSize(0.8);
mesher.setSizeField([](double x, double y) {
    return (x*x + y*y < 4.0) ? 0.25 : 0.8;
});

TriangleMesh mesh = mesher.createMesh(shape);
export_mesh(mesh, "shape.pde2d");

// Later, in another run or another part of the program:
TriangleMesh restored = import_mesh("shape.pde2d");
```

`shape` describes the boundary. `mesher` describes how to fill it with
triangles. `createMesh(shape)` combines them and returns `TriangleMesh`.
That separation is intended to let a future PDE object receive a configured
`MeshGenerator` object without knowing how the geometry was created.

All angles are counterclockwise radians. `setRotation` rotates around the
geometry center. `PolygonGeometry` vertex coordinates are local coordinates
relative to that center. The first vertex of a regular polygon lies in the
direction of its rotation angle. For a regular polygon, n=3 makes an
equilateral triangle and n=4 makes a square; both use the same `sideLength`
parameter. At rotation zero, the first vertex is on the positive x-axis.

`EllipseGeometry::setA` and `setB` set the semi-axis lengths. Thus A=2 and B=1
gives a width of 4 and height of 2 before rotation. `CircleGeometry::setRadius`
sets the radius. Circle orientation has no visual effect, but changes which
point is first in its polygonal boundary approximation.

`RectangleGeometry(width,height)` creates an axis-aligned rectangle centered on
its geometry center; `setRotation` then rotates it. Its width follows local x
and its height follows local y. `RectangleWithSemicircleGeometry(width,height)`
creates the same rectangle with one short end replaced by a semicircle whose
diameter is `height`. The total length along local x is `width + height/2`.
The semicircle is on the positive-x end by default; pass
`SemicircleEnd::NegativeX` to cap the other end, or rotate the whole geometry.
`setSemicircleSegments` controls how many straight boundary edges approximate
the curved cap.

`PolygonGeometry` handles an n-gon with different side lengths by taking an
ordered list of its vertices. The user supplies the local `(x,y)` coordinates;
the polygon must be simple, closed by connecting the last point to the first,
and have nonzero area. Concave outlines are supported. Holes and multiple
disconnected outlines are not represented in this first shape-only interface.

## The five mesh generator classes

Each class has the same `createMesh(const Geometry2D&)` interface and supports
the common setters `setMaxElementSize`, `setOptimizationIterations`, and
`setMaxRefinementPasses`. All return three-node P1 triangles.

1. **`EarClippingMeshGenerator`** triangulates the boundary polygon directly.
   It is the simplest baseline and performs no interior refinement. Element
   sizes and shapes depend on the polygon vertices and their order.
2. **`QualityEarClippingMeshGenerator`** chooses the best-shaped currently
   available ear at each step, then refines to the requested size and applies
   edge flips. It is a useful general starting choice, though a local choice
   cannot guarantee globally optimal triangles.
3. **`UniformRefinementMeshGenerator`** uses one target size throughout the
   domain and conforming midpoint splits. It is easy to control, but can create
   more elements than a spatially tailored mesh.
4. **`DelaunayEdgeFlipMeshGenerator`** refines to one target size, then flips
   eligible interior diagonals when the opposite-angle test indicates a locally
   non-Delaunay pair. Boundary edges stay fixed. This is a native local edge
   flipping method, not a full constrained-Delaunay mesher.
5. **`LocalSizeRefinementMeshGenerator`** takes `setSizeField(h)` where `h(x,y)`
   is the desired local size. It splits oversized triangles and adds matching
   edge nodes to their neighbors to avoid hanging edges, then flips eligible
   interior edges. This can concentrate elements in selected regions. Abrupt
   size changes can create less regular transition triangles.

`setMaxElementSize(h)` is a global cap in the geometry's coordinate units.
For local sizing, the function can request smaller elements; it cannot exceed
that cap. A smaller size usually means better geometric resolution and more
PDE unknowns. `setMaxRefinementPasses` limits work; if the requested size is
not reached, mesh creation reports an error rather than silently accepting it.

Each element stores exactly three vertex indices in a `std::array`. There is
no element-order option. Refinement adds new vertices and divides triangles;
each resulting triangle still has three vertices. Circle and ellipse boundaries are
polygonal approximations; control their boundary resolution with
`setBoundarySegments` on those geometry classes.

`export_mesh` saves the result in the project's own versioned `.pde2d` text
format. `import_mesh` reads that format and checks the topology before returning
a `TriangleMesh`. Nodes and connectivity use zero-based indices. Boundary-edge
records retain vertex endpoints and are available for later boundary-condition
assignment. The file preserves mesh coordinates, element connectivity,
and boundary edges. Its version-1 `ORDER 1` record is fixed; other orders are
rejected. It does not preserve the original shape parameters or mesh generator
settings.

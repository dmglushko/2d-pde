# Read the 2D mesh code, one step at a time

This guide follows the **implemented geometry and mesh path** in this checkout. Start with the complete [consumer program](../../examples/learning/first_mesh.cpp), then read the numbered stops below. You can run the program without a build system or external meshing library.

```cpp
#include "discretization/geometry.hpp"
#include "discretization/mesh_generators.hpp"
#include "discretization/mesh_io.hpp"

#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    const std::string path = argc > 1 ? argv[1] : "rectangle.pde2d";

    discretization::RectangleGeometry geometry(2.0, 1.0);
    discretization::UniformRefinementMeshGenerator mesher;
    mesher.setMaxElementSize(0.5);
    mesher.setOrder(1);

    const discretization::TriangleMesh mesh = mesher.createMesh(geometry);
    discretization::export_mesh(mesh, path);
    const discretization::TriangleMesh restored = discretization::import_mesh(path);

    std::cout << "order=" << restored.order
              << " nodes=" << restored.nodes.size()
              << " triangles=" << restored.elements.size()
              << " boundary_edges=" << restored.boundary_edges.size() << '\n';
}
```

Compile and run it under WSL Ubuntu:

```bash
cd /home/gldima/2d-pde
mkdir -p /tmp/2d-pde-learning
g++ -std=c++17 -Wall -Wextra -Werror -Icpp \
  examples/learning/first_mesh.cpp -o /tmp/2d-pde-learning/first_mesh
/tmp/2d-pde-learning/first_mesh /tmp/2d-pde-learning/rectangle.pde2d
head -n 8 /tmp/2d-pde-learning/rectangle.pde2d
```

The command line argument is the output mesh path. With no argument, the program writes `rectangle.pde2d` in your current directory. The build above was run under WSL; its result is recorded in [Validation](#validation). There is no root CMake target in this checkout.

## 1. Read the consumer program

The first three `#include` lines bring in the declarations and inline implementations for [geometry](../../cpp/discretization/geometry.hpp), [mesh generators](../../cpp/discretization/mesh_generators.hpp), and [mesh I/O](../../cpp/discretization/mesh_io.hpp). `main` is the entry point. The `discretization::` prefix selects names in this project's namespace, like a module qualifier in Python. `const` on `mesh` and `restored` means this program does not change them after construction.

Follow one call at a time:

```text
RectangleGeometry -> createBoundary() -> Boundary
mesher configuration -> createMesh(geometry) -> TriangleMesh
TriangleMesh -> export_mesh(path) -> .pde2d file -> import_mesh(path) -> TriangleMesh
```

**Pause:** What part of the program describes the region, and what part chooses how to fill it? Try changing only `setMaxElementSize(0.5)` to `0.25`, rebuilding, and comparing the counts. Then try `setOrder(2)`. [Check your reasoning](questions.md#answers).

## 2. Geometry becomes one boundary loop

[Geometry2D](../../cpp/discretization/geometry.hpp) is an abstract base class: it declares `createBoundary()` but cannot itself describe a shape. The concrete shape classes implement that method. [RectangleGeometry](../../cpp/discretization/geometry.hpp) stores a positive width and height; its boundary has four points in local coordinates around its center. The base class [transform](../../cpp/discretization/geometry.hpp) rotates those local coordinates counterclockwise in radians, then shifts them by the center. `Boundary` is a `std::vector<Point2>` forming one closed loop; the first point is **not** repeated at the end. The final edge closes the loop implicitly.

Other shapes in the same header are `CircleGeometry` (radius), `EllipseGeometry` (semi-axes `A` and `B`), `RegularPolygonGeometry` (side count and side length), `PolygonGeometry` (your ordered local vertices), and `RectangleWithSemicircleGeometry` (rectangle width and height, capped end, polygonal cap segments). Circle and ellipse use polygonal boundaries with configurable segment counts. The cap diameter equals the rectangle height; `SemicircleEnd::PositiveX` is the default. These classes describe a single outline; holes and disconnected regions have no representation here.

**Pause:** If you rotate a `RectangleGeometry(2, 1)` by 90 degrees, do `width()` and `height()` exchange values? Where does the transformed boundary actually get created? [Answers](questions.md#answers).

## 3. A configured mesher creates a mesh

[MeshGenerator](../../cpp/discretization/mesh_generators.hpp) owns a `MeshOptions` value. Setters change that value. Its `createMesh(const Geometry2D&) const` asks the geometry for a boundary, copies the options, chooses its subclass's algorithm, and passes both to [Triangulator2d::generate](../../cpp/discretization/triangulate.hpp). The geometry is passed by **const reference**: the call reads the existing object rather than taking ownership of it. The returned `TriangleMesh` is a value with its own vectors; it remains usable after the geometry and mesher go out of scope. An optional `setSizeField` stores a callable in the mesher; if that callable captures references, those referenced objects must still exist when `createMesh` calls it.

`generate` [validates the boundary](../../cpp/discretization/triangulate.hpp), takes its points as initial mesh nodes, ear clips the loop into triangles, records boundary edges, optionally refines and flips, smooths interior nodes, and optionally adds P2 edge nodes. The call can throw for invalid input or when the refinement pass limit is reached; it does not return a partly built mesh in that case.

The five public mesher classes and their actual steps are compared in [meshing methods](reference.md#meshing-methods). Choose one as an experiment, not as a guaranteed best method.

**Pause:** Why can the plain ear clipping class ignore a small `maxElementSize`? Why should a caller keep any objects referenced by a size field alive? [Answers](questions.md#answers).

## 4. Read the mesh as data

[TriangleMesh](../../cpp/discretization/geometry.hpp) is four fields, with zero-based indices:

| Field | Meaning |
| --- | --- |
| `nodes` | `Point2{x,y}` coordinates. Initial boundary points and any later split or P2 midpoint nodes live here. |
| `elements` | Each entry gives indices into `nodes`. P1 has three corner indices. P2 has those corners followed by edge midpoint indices for edges 01, 12, 20. |
| `boundary_edges` | Pairs of **corner endpoint** indices on the exterior, including for P2. Refinement may split these into more edges. |
| `order` | `1` for P1 or `2` for P2. This describes mesh connectivity, not an implemented PDE solver. |

For example, a P1 element `{2, 5, 7}` uses `nodes[2]`, `nodes[5]`, and `nodes[7]`. A P2 element `{2, 5, 7, 9, 10, 11}` adds the 2–5, 5–7, and 7–2 midpoints. The P2 midpoint indices are shared for a common edge by [add_quadratic_nodes](../../cpp/discretization/triangulate.hpp).

**Pause:** Does `boundary_edges` contain P2 midpoint indices? What does `elements.size()` count? [Answers](questions.md#answers).

## 5. Export, then import

[export_mesh](../../cpp/discretization/mesh_io.hpp) validates a mesh and writes the text format `PDE2D_MESH 1`, with `ORDER`, `NODES`, `ELEMENTS`, and `BOUNDARY_EDGES` sections. [import_mesh](../../cpp/discretization/mesh_io.hpp) parses those sections and validates the result. The file records the **resulting mesh**, not the original geometry object or mesher options. Changing a geometry after export cannot reconstruct the old settings from the file.

The I/O validator checks counts and indices, triangle orientation and nondegeneracy, edge ownership and exterior edge records, and shared P2 midpoint **indices**. It does not establish that every mesh is globally overlap-free, high quality, or suitable for a PDE. It also does not check that a P2 midpoint coordinate is geometrically halfway along its edge. See [format and limits](reference.md#format-and-limits).

**Pause:** What would you need to save separately if you wanted to regenerate a mesh with different `maxElementSize`? [Answers](questions.md#answers).

## Where to read next

1. Run and edit [first_mesh.cpp](../../examples/learning/first_mesh.cpp).
2. Follow the five steps above with [geometry.hpp](../../cpp/discretization/geometry.hpp) and [mesh_generators.hpp](../../cpp/discretization/mesh_generators.hpp) open.
3. Use the [reference map](reference.md) to choose a method and understand its parameters.
4. Answer the [reading questions](questions.md) before expanding their answers.
5. Only then read [triangulate.hpp](../../cpp/discretization/triangulate.hpp) and [mesh_io.hpp](../../cpp/discretization/mesh_io.hpp) in detail.

## Scope and maintenance

The code here implements geometry construction, triangle meshing, and mesh export/import. There is no PDE equation setup, element assembly, boundary-condition application, linear solve, or solution validation in these headers. Any later PDE integration is a plan, not a behavior of this example. The project also has a [discretization overview](../../cpp/discretization/README.md) and an [offline mesh viewer](../../tools/mesh_viewer/README.md).

When the public API changes: (1) update the example to use the new call, (2) rebuild and run it with the command above, (3) compare its generated file and [reference map](reference.md) with the changed headers, (4) check every local source link, and (5) update the validation note below. Keep behavioral claims tied to implementation, especially for meshing limits.

## Validation

Checked under WSL Ubuntu on 2026-10-02 against this checkout: `g++ -std=c++17 -Wall -Wextra -Werror -Icpp` compiled the example; running it and importing the exported file printed `order=1 nodes=81 triangles=128 boundary_edges=32`. The file began with `PDE2D_MESH 1`, `ORDER 1`, and `NODES 81`. All 77 local Markdown links resolved to existing files, and source links resolved.

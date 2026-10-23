# Questions for a first reading

Keep [first_mesh.cpp](../../examples/learning/first_mesh.cpp) and the [walkthrough](README.md) open. Try answering each question by finding the named symbol in source. Expand an answer only after you have a prediction.

1. In `main`, which object describes the region and which object decides the mesh method? What does `createMesh` return?
2. What does the `const Geometry2D&` parameter of `MeshGenerator::createMesh` mean for copying, mutation, and ownership during this call?
3. If `RectangleGeometry(2, 1)` is rotated by π/2, do its stored `width()` and `height()` change? Which function creates the final coordinates?
4. Why can `EarClippingMeshGenerator` produce long edges even after `setMaxElementSize(0.25)`?
5. If `setSizeField` receives a lambda that captures another object by reference, how long must that object live?
6. In a six-node P2 element, what do entries 3, 4, and 5 mean? Do `boundary_edges` store those midpoint indices?
7. What does `elements.size()` count? How would you find a triangle's coordinates?
8. Which parts of the original geometry and mesher can `import_mesh` recover from a `.pde2d` file? What must you save separately to remesh later?
9. A saved file passes `import_mesh`. Does that establish a good minimum angle, no global overlap, or a working PDE discretization?
10. If a local size field asks for `0.1` in one region and `1.0` elsewhere while `maxElementSize` is `0.5`, what targets are used at sampled centroids? What can happen at the pass limit?

## Answers

<details>
<summary>1. Geometry versus mesher</summary>

`RectangleGeometry` describes the boundary. `UniformRefinementMeshGenerator` chooses a method and holds its settings. `createMesh(geometry)` returns a `TriangleMesh` by value. Read [MeshGenerator](../../cpp/discretization/mesh_generators.hpp).
</details>

<details>
<summary>2. The const reference</summary>

`&` passes a reference rather than copying the geometry; `const` means the method reads it. The mesher does not own it. It asks for a new `Boundary`, then returns a mesh with its own vectors. Read [createMesh](../../cpp/discretization/mesh_generators.hpp).
</details>

<details>
<summary>3. Rotation</summary>

The stored lengths stay 2 and 1. `RectangleGeometry::createBoundary` makes local corners and calls `Geometry2D::transform`, which rotates and translates the coordinates. Read [rectangle](../../cpp/discretization/geometry.hpp) and [transform](../../cpp/discretization/geometry.hpp).
</details>

<details>
<summary>4. Plain ear clipping</summary>

The `EarClipping` algorithm skips `refine`, so the size setting is not enforced. Read the [pipeline condition](../../cpp/discretization/triangulate.hpp).
</details>

<details>
<summary>5. Size-field lifetime</summary>

The mesher stores the callable. Any object captured by reference must remain alive through every `createMesh` call that invokes the callable. Capturing a local value by value is often simpler. Read [setSizeField](../../cpp/discretization/mesh_generators.hpp) and [target](../../cpp/discretization/triangulate.hpp).
</details>

<details>
<summary>6. P2 indices</summary>

Entries 3, 4, and 5 refer to shared midpoints of corner edges 0–1, 1–2, and 2–0. `boundary_edges` contain only corner endpoint pairs. Read [TriangleMesh](../../cpp/discretization/geometry.hpp) and [P2 construction](../../cpp/discretization/triangulate.hpp).
</details>

<details>
<summary>7. Elements and coordinates</summary>

`elements.size()` counts triangle records, not nodes or PDE unknowns. For an element `e`, look up `nodes[e[0]]`, `nodes[e[1]]`, and `nodes[e[2]]` for its corner coordinates. Read [TriangleMesh](../../cpp/discretization/geometry.hpp).
</details>

<details>
<summary>8. What survives the file</summary>

The file saves order, mesh nodes, element connectivity, and boundary edge connectivity. It does not save the shape type, dimensions, center, rotation, chosen generator, size field, or other mesher settings. Save those separately if you want to regenerate a different mesh. Read [writer](../../cpp/discretization/mesh_io.hpp).
</details>

<details>
<summary>9. Validation scope</summary>

No. Import performs structural checks, not a global overlap or angle-quality proof. These headers do not implement PDE assembly or solving. Read [validate_mesh](../../cpp/discretization/mesh_io.hpp).
</details>

<details>
<summary>10. Local target and pass limit</summary>

At triangle centroids where the callable returns `0.1`, the target is `0.1`; where it returns `1.0`, the global cap makes it `0.5`. If the longest-edge criterion still fails after the allowed passes, `createMesh` throws. Read [target and refine](../../cpp/discretization/triangulate.hpp).
</details>

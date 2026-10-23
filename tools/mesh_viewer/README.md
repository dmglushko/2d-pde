# PDE2D mesh viewer

Open `tools/mesh_viewer/index.html` in a browser. It works offline and starts with a P1 example exported by this project's C++ mesher. Use **Open .pde2d** or drop another exported P1 file onto the canvas. No server or packages are needed.

## Generate your own mesh

From the project root (Linux / WSL):

```sh
g++ -std=c++17 -O2 -Wall -Wextra -Werror examples/mesh_viewer/produce_mesh.cpp -o /tmp/produce_mesh
/tmp/produce_mesh examples/mesh_viewer/rectangle_p1.pde2d
```

Then open `tools/mesh_viewer/index.html` and choose `examples/mesh_viewer/rectangle_p1.pde2d`. The example uses `Geometry2D`, `MeshGenerator::createMesh`, `export_mesh`, and `import_mesh` without touching solver code.

The bundled startup example is generated from the exported file. To update it after regenerating `rectangle_p1.pde2d`, run:

```sh
node -e 'const fs=require("fs"); fs.writeFileSync("examples/mesh_viewer/example_mesh.js", "window.exampleMesh = " + JSON.stringify(fs.readFileSync("examples/mesh_viewer/rectangle_p1.pde2d", "utf8")) + ";\n")'
```

## Inspecting

- Drag to pan, use the mouse wheel to zoom around the pointer, or press **Fit view**.
- Click a triangle for its zero-based connectivity, coordinates, three edges and adjacent triangles, signed area, orientation, mean-ratio quality, smallest corner angle, and longest/shortest edge ratio.
- Toggle node and triangle IDs independently. Quality measures use the three corner nodes.
- Red marks degenerate/inverted triangles and triangles with repeated indices or duplicate connectivity. Purple marks edges with boundary ownership mismatch, non-manifold connectivity, inconsistent shared-edge direction, or invalid boundary degree/records. Quality summaries exclude degenerate and inverted triangles. Collapsed triangles can be selected by clicking within six screen pixels of their edges when no triangle interior occupies the click.

## Verification

Run `node tools/mesh_viewer/test_mesh.cjs` from the project root. The tests read the real P1 export, check malformed data and defective connectivity, and exercise selection, panning, cancellation, and zoom with a mocked canvas. They do not verify browser rendering or pixel layout.

This is a visual diagnostic, not the C++ importer's full validity check. It accepts structurally readable files with some topology or orientation defects so they can be inspected. Malformed syntax, unsupported orders, non-three-node triangles, out-of-range node indices, and nonfinite coordinates are rejected without replacing the currently displayed mesh. The viewer does not check triangle overlap or detect all geometric self-intersections. With very large meshes, ID labels and full-canvas redraws can become slow; turn labels off to inspect dense meshes.

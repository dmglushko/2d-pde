# PDE2D mesh viewer

Open `tools/mesh_viewer/index.html` in a browser. It works offline and includes an example P2 mesh exported by this project's C++ mesher. Use **Open .pde2d** or drop another exported file onto the canvas. `examples/mesh_viewer/rectangle_p1.pde2d` is also provided for comparison. No server or packages are needed.

## Generate your own mesh

From the project root (Linux / WSL):

```sh
g++ -std=c++17 -O2 -Wall -Wextra -Werror examples/mesh_viewer/produce_mesh.cpp -o /tmp/produce_mesh
/tmp/produce_mesh examples/mesh_viewer/rectangle_p2.pde2d
```

Then open `tools/mesh_viewer/index.html` and choose `examples/mesh_viewer/rectangle_p2.pde2d`. Edit `produce_mesh.cpp` to select a different existing geometry, generator, size, or order. The example uses `Geometry2D`, `MeshGenerator::createMesh`, `export_mesh`, and `import_mesh` without touching solver code.

The bundled example displayed on startup is a JavaScript string generated from the actual exported file. If you regenerate the `.pde2d` file and want to update that startup copy, run:

```sh
python3 -c 'import json,pathlib; p=pathlib.Path("examples/mesh_viewer/rectangle_p2.pde2d"); pathlib.Path("examples/mesh_viewer/example_mesh.js").write_text("window.exampleMesh = " + json.dumps(p.read_text()) + ";\n")'
```

## Inspecting

- Drag to pan, use the mouse wheel to zoom around the pointer, or press **Fit view**.
- Click a triangle for its zero-based connectivity, coordinates, three edges and adjacent triangles, signed area, orientation, mean-ratio quality, smallest corner angle, and longest/shortest edge ratio.
- Toggle node and triangle IDs independently. P2 nodes appear as blue dots at the three mid-edge positions. The drawn triangle sides are straight. Quality is measured on the three corner nodes, so it is the geometry of this mesh, not an imagined curved P2 mapping.
- Red marks degenerate/inverted triangles and triangles with repeated indices or duplicate corner connectivity. Purple marks edges with boundary ownership mismatch, non-manifold connectivity, inconsistent shared-edge direction, invalid boundary degree/records, or P2 midpoint problems. Quality summaries exclude degenerate and inverted triangles. Collapsed triangles can be selected by clicking within six screen pixels of their edges when no triangle interior occupies the click.

## Verification

Run `node tools/mesh_viewer/test_mesh.cjs` from the project root if Node.js is available. The tests read both real exported examples, check malformed data and defective connectivity, and exercise selection, panning, cancellation, and zoom with a mocked canvas. They do not verify browser rendering or pixel layout.

This is a visual diagnostic, not the C++ importer's full validity check. It accepts structurally readable files with some topology or orientation defects so they can be inspected. Malformed section syntax, out-of-range node indices, nonfinite coordinates, and unsupported orders are rejected without replacing the currently displayed mesh. The viewer does not check triangle overlap or detect all geometric self-intersections. With very large meshes, ID labels and full-canvas redraws can become slow; turn labels off to inspect dense meshes.

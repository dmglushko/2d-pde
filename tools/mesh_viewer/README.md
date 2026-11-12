# PDE2D mesh viewer

Open `tools/mesh_viewer/index.html` in a browser. It works offline and starts with a P1 example exported by this project's C++ mesher. Use **Open .pde2d** or drop another exported P1 file onto the canvas. No server or packages are needed.

## Generate your own mesh

From the project root (Linux / WSL), after `tools/test.sh` has built the
project (examples are built by default, `PDE2D_BUILD_EXAMPLES`):

```sh
build/examples/produce_mesh /tmp/plate.pde2d /tmp/plate.svg
```

Then open `tools/mesh_viewer/index.html` and choose `/tmp/plate.pde2d`.
The example (`examples/mesh_viewer/produce_mesh.cpp`) meshes a plate with
a hole using `mesh::Mesher` and writes it with `io::export_pde2d`; the
optional SVG file is a picture of the same mesh (`io::export_mesh_svg`).
A plate with a hole has two boundary loops, so the file is version 5.

The bundled startup example (`examples/mesh_viewer/rectangle_native.pde2d`)
was exported by the previous code and is kept unchanged. To replace it with
a new export, write the file and run:

```sh
node -e 'const fs=require("fs"); fs.writeFileSync("examples/mesh_viewer/example_mesh.js", "window.exampleMesh = " + JSON.stringify(fs.readFileSync("examples/mesh_viewer/rectangle_native.pde2d", "utf8")) + ";\n")'
```

## File format

The viewer accepts `PDE2D_MESH` versions 1 through 5 with order 1 triangles; version 5 (meshes with holes) adds boundary loop records, see [the format](../../docs/formats/pde2d-mesh.md). The summary shows the number of boundary loops. Version 2 uses the same sections as version 1, but each `BOUNDARY_EDGES` record is `node_a node_b region`. Region names contain only ASCII letters, digits, `_`, `.`, and `-`; the single token `-` denotes an untagged edge. Version 1 edges are untagged.

Boundary colors are assigned deterministically by region name, with orange for untagged edges; the legend lists the regions present. Colors may coincide for different names. Purple diagnostic edges are drawn over boundary colors, even when the boundary toggle is off.

## Inspecting

- Drag to pan, use the mouse wheel to zoom around the pointer, or press **Fit view**.
- Click a triangle for its zero-based connectivity, coordinates, three edges and adjacent triangles, signed area, orientation, mean-ratio quality, smallest corner angle, and longest/shortest edge ratio.
- Toggle node and triangle IDs independently. Quality measures use the three corner nodes.
- Red marks degenerate/inverted triangles and triangles with repeated indices or duplicate connectivity. Purple marks edges with boundary ownership mismatch, non-manifold connectivity, inconsistent shared-edge direction, or invalid boundary degree/records. Quality summaries exclude degenerate and inverted triangles. Collapsed triangles can be selected by clicking within six screen pixels of their edges when no triangle interior occupies the click.

Triangle orientation uses a filtered determinant with an exact JavaScript BigInt
fallback, matching the native binary64 sign test. Thin triangles remain valid;
their shape is described by the quality metrics rather than an area cutoff.

## Verification

Run `node tools/mesh_viewer/test_mesh.cjs` from the project root (CTest
runs it as `viewer_test_mesh` when Node.js is found, and
`viewer_reads_export` checks a file written by the C++ exporter). The tests read the real P1 export, check malformed data, v3 metadata/provenance roundtrips (all curve kinds), and defective connectivity, and exercise selection, panning, cancellation, and zoom with a mocked canvas. They do not verify browser rendering or pixel layout.

This is a visual diagnostic, not the C++ importer's full validity check. It accepts structurally readable files with some topology or orientation defects so they can be inspected. Malformed syntax, unsupported orders, non-three-node triangles, out-of-range node indices, and nonfinite coordinates are rejected without replacing the currently displayed mesh. The viewer does not check triangle overlap or detect all geometric self-intersections. With very large meshes, ID labels and full-canvas redraws can become slow; turn labels off to inspect dense meshes.

## Full mesh results (v3)

The C++ exporter of this rewrite (`io::export_pde2d`) writes version 2,
or version 5 with `RESULT 0` for meshes with several boundary loops; it
has no settings or guarantees to record. Files of versions 3 and 4 written
by the previous code still open here, and `io::import_pde2d` reads them
(ignoring the trailer). Versions 1–2 have no saved settings or guarantees.
Version 3 implies corner protection, matching its original behavior.

Version 4 starts with the exact v2 `ORDER`, `NODES`, `ELEMENTS`, and tagged `BOUNDARY_EDGES` sections, then requires the following extension in this order. Version 3 has the same extension but omits the final `preserve_corners` option. Whitespace separates tokens; line breaks are cosmetic. All indices are zero based. Booleans are `0` or `1`.

```text
OPTIONS algorithm treatment preserve_regions boundary_max_edge boundary_max_deviation refinement_max_edge max_steps preserve_corners
REPORT size_satisfied delaunay_verified refinement_steps message_hex
CONTEXT present
[only when present = 1:]
PROTECTED_VERTICES count
vertex_index ...
POLYGON_SNAPSHOT count
x y ...
SOURCE_CURVES count
curve_id region kind geometry_values ...
EDGE_SOURCES count
span_count curve_index begin end [curve_index begin end ...]
[end of conditional sections]
END_RESULT
```

- `algorithm`: `0` ear clipping, `1` constrained Delaunay. `treatment`: `0` fixed, `1` split polygon edges, `2` refine on curves.
- Boundary length, deviation, and refinement length must be finite and nonnegative. Zero disables that objective. Curved geometry needs at least one boundary objective during initial sampling. `max_steps` is a nonnegative signed 32-bit integer.
- `message_hex` is the lowercase hexadecimal encoding of the report message's UTF-8 bytes, or `-` for empty. This safely preserves whitespace, newlines, and punctuation. No numerical callback is serialized.
- Protected vertex indices must be distinct boundary nodes. The viewer draws teal rings at these junctions, independently of node ID labels.
- Each source curve has a unique unsigned 64-bit ID (preserved as a decimal string by JavaScript), a region token with the v2 naming rules (`-` means empty), and a geometry kind. Geometry is an immutable world-coordinate snapshot, not transformed again on import.
- Geometry kinds and values: `0` line (`start_x start_y end_x end_y`); `1` quadratic Bézier (`start_x start_y control_x control_y end_x end_y`); `2` cubic Bézier (`start_x start_y control1_x control1_y control2_x control2_y end_x end_y`); `3` elliptical arc (`center_x center_y radius_x radius_y rotation start_angle sweep_angle`). Arc angles are radians, radii positive, and sweep nonzero with magnitude at most `2π`.
- `POLYGON_SNAPSHOT` records the polygon at sampling or the last curve update. C++ checks that its vertices are anchored to source curves and every current boundary edge lies on a snapshot segment. Chord splits preserve this snapshot; a curve update replaces it.
- `EDGE_SOURCES` count must equal `BOUNDARY_EDGES` count. Each record contains ordered parameter intervals; `curve_index` indexes `SOURCE_CURVES`, not stable IDs. Parameters lie in `[0,1]`, must differ, and may decrease for reverse traversal. Consecutive spans join on the source geometry. Chord vertices may lie off that geometry. Merely selecting curve mode with size refinement disabled does not relocate them. C++ additionally checks complete, nonoverlapping source coverage, directed connectivity, and protected region and geometric junctions. Preserved edge tags agree with every source interval they represent.
- Counts are capped at 50 million per section and in total for source spans. Coordinates, curve parameters, and tolerances must be finite. Invalid/truncated sections, unknown enum values, duplicate IDs/protected indices, endpoint mismatches, and trailing tokens are rejected. C++ additionally calls `validate_mesh_geometry`; the viewer retains its diagnostic handling of mesh topology defects.

The viewer presents **Mesh settings**, **Reported guarantees**, and **Viewer structural diagnostics** separately. Reported size satisfaction and Delaunay verification come from the saved mesher report; the viewer does not recompute them. A structurally clean mesh can still have an unsatisfied size target. Versions 1 and 2 display settings and guarantees as unavailable.

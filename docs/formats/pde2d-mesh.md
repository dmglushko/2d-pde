# `PDE2D_MESH` text format

Single source of truth for the mesh file format. Three independent parsers
implement it and must stay in sync with this page:

| Implementation | Reads | Writes |
| --- | --- | --- |
| `cpp/discretization/mesh_io.hpp` (+ `curve_io.hpp`) | v1–v4 | v2 (`TriangleMesh`), v4 (`MeshResult`) |
| `tools/mesh_viewer/mesh.js` (browser viewer, Node tests) | see file | — |
| `tools/plot_mesh.py` (matplotlib PNGs) | see file | — |

Changing the format means: bump the version, keep old versions readable,
update all three parsers, update this page, and add a fixture exercised by
both the C++ and the JavaScript tests (`tools/test_showcase.cjs` already
round-trips every showcase mesh written by the C++ examples).

## Lexical rules

- Whitespace-separated tokens. The C++ and JavaScript readers are token based,
  but `tools/plot_mesh.py` is line based, so writers must keep one record per
  line exactly as shown below.
- Indices are zero-based unsigned integers.
- Reals are finite decimal doubles, written with 17 significant digits.
- Region tags match `[A-Za-z0-9_.-]+` and are never `-`; the token `-`
  means "no tag" wherever a tag appears.
- Nothing may follow the final section.

## Sections

```
PDE2D_MESH <version>          version in 1..4
ORDER 1                       only P1 (three-node triangles) exists
NODES <n>
<x> <y>                       n lines
ELEMENTS <m>
3 <i> <j> <k>                 m lines; leading 3 = vertex count
BOUNDARY_EDGES <e>
<i> <j> [<tag>]               e lines; <tag> present from v2 on
```

Version 3 and 4 append a meshing result:

```
OPTIONS <algorithm> <boundary_treatment> <preserve_regions 0|1>
        <boundary.max_edge_length> <boundary.max_curve_deviation>
        <refinement.max_edge_length> <refinement.max_steps>
        [<boundary.preserve_corners 0|1>]          v4 only (v3 reads as 1)
REPORT <size_satisfied 0|1> <delaunay_verified 0|1> <refinement_steps>
       <message>                                   lowercase hex of the UTF-8
                                                   bytes, or - when empty
CONTEXT <0|1>                                      1 = boundary context follows
PROTECTED_VERTICES <p>
<node index>                                       p lines
POLYGON_SNAPSHOT <q>
<x> <y>                                            q lines
SOURCE_CURVES <c>
<curve id> <region tag or -> <kind> <parameters>   c lines
EDGE_SOURCES <e>                                   must equal BOUNDARY_EDGES
<k> (<curve> <begin> <end>){k}                     e lines; curve = index into
                                                   SOURCE_CURVES
END_RESULT
```

`<algorithm>` and `<boundary_treatment>` are the stable wire IDs of
`InteriorAlgorithm` (0 ear clipping, 1 constrained Delaunay) and
`BoundaryTreatment` (0 fixed, 1 split polygon edges, 2 refine on curves) in
`cpp/discretization/mesh_types.hpp`. **Never renumber them.**

## Curve kinds (`curve_io.hpp`)

| Kind | Type | Parameters |
| --- | --- | --- |
| 0 | `LineCurve` | start.x start.y end.x end.y |
| 1 | `QuadraticBezier` | start control end (x y each) |
| 2 | `CubicBezier` | start control1 control2 end (x y each) |
| 3 | `EllipticalArc` | center.x center.y radius_x radius_y rotation start_angle sweep_angle |

Kind IDs are wire format: never reuse or renumber them. A new curve type gets
the next free ID (see `cpp/discretization/EXTENDING.md`).

## Version history

| Version | Adds |
| --- | --- |
| 1 | nodes, P1 elements, untagged boundary edges |
| 2 | a region tag (or `-`) per boundary edge; written for raw meshes |
| 3 | `OPTIONS`, `REPORT`, optional boundary `CONTEXT` |
| 4 | `boundary.preserve_corners` in `OPTIONS`; written for `MeshResult` |

## Validation on import

Readers reject (C++ throws `std::runtime_error`): unknown versions or orders,
truncated data, non-finite reals, indices out of range, non-triangle elements,
invalid tags, content after the last section, meshes failing
`validate_mesh_geometry` (connected disk topology, duplicate/unused vertices,
crossing or overlapping edges), and recorded guarantees that do not hold
(e.g. `delaunay_verified` on a non-CDT mesh, or a size guarantee violated).

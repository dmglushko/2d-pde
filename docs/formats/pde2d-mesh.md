# `PDE2D_MESH` text format

Single source of truth for the mesh file format. Three independent parsers
implement it and must stay in sync with this page:

| Implementation | Reads | Writes |
| --- | --- | --- |
| `cpp/discretization/mesh_io.hpp` (+ `curve_io.hpp`) | v1–v5 | v2 (`TriangleMesh`), v4 (`MeshResult`); v5 for meshes with holes |
| `tools/mesh_viewer/mesh.js` (browser viewer, Node tests) | see file | — |
| `tools/plot_mesh.py` (matplotlib PNGs) | see file | — |

Changing the format means: bump the version, keep old versions readable,
update all three parsers, update this page, and add a fixture exercised by
both the C++ and the JavaScript tests (`tools/test_showcase.cjs` already
round-trips every showcase mesh written by the C++ examples; the version 5
files written by `tests/discretization/holes.cpp` are read by
`tests/tools/mesh_holes_viewer.cjs` and `tools/test_plot_mesh.py`).

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
PDE2D_MESH <version>          version in 1..5
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

### Version 5: meshes with holes

Version 5 is written exactly when the mesh boundary has more than one closed
loop (an outer loop and holes); one-loop meshes keep versions 4 and 2 byte for
byte. After `BOUNDARY_EDGES` (tagged as in v2) it adds:

```
BOUNDARY_LOOPS <k>                                 k >= 1 (writers: k >= 2)
<edge count>                                       k lines, each >= 3
RESULT <0|1>                                       1 = the v4 trailer follows
```

The boundary edges are listed loop by loop: group `i` is the next
`<edge count>` edges and must be exactly one closed loop of the mesh. Group 0
is the outer loop (the loop through the lexicographically least boundary
vertex); the holes follow. Writers regroup an ungrouped mesh stably (keeping
each loop's relative edge order) together with its tags and edge sources.
With `RESULT 1` the v4 trailer follows (`OPTIONS` with `preserve_corners`,
`REPORT`, `CONTEXT`, ..., `END_RESULT`), with one extra record after the
polygon snapshot when a context is present:

```
POLYGON_SNAPSHOT <q>
<x> <y>                                            q lines, loop after loop
POLYGON_LOOPS <k>                                  k >= 1
<vertex count>                                     k lines, each >= 3, sum q
```

`RESULT 0` (a raw `TriangleMesh`) ends the file; such a mesh records no
settings or guarantees. Source curves have no loop field: on import a curve
belongs to the loop whose edges reference it, and the C++ reader rejects a
curve referenced from two loops. Readers of versions 1–4 do not accept holes:
the C++ reader rejects a v1–v4 file whose boundary has several loops.

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
| 5 | `BOUNDARY_LOOPS`, `RESULT`, `POLYGON_LOOPS`; written for meshes with holes |

## Validation on import

The readers validate different amounts. Only a successful C++ import
establishes the geometric and recorded guarantees.

- **C++ `import_mesh_result`** (authoritative). Catch `std::exception` to
  handle every failure. The exception type depends on the stage:
  - parsing throws `std::runtime_error`: unknown versions or orders, a wrong
    section label, truncated or unparsable numbers, non-triangle elements,
    invalid tags, an out-of-range numeric scale, or content after the last
    section;
  - `validate_mesh_geometry` throws `std::invalid_argument`: out-of-range
    indices, non-finite or duplicate vertices, non-positive triangle area,
    duplicate triangles, a mesh that is not one connected domain bounded by
    closed loops, unused vertices, crossing or overlapping edges;
  - loop records throw `std::runtime_error`: holes in a v1–v4 file, or v5
    groups that are not the mesh's loops in order;
  - result validation throws `std::runtime_error`: recorded guarantees that do
    not hold (e.g. `delaunay_verified` on a non-CDT mesh or a violated size
    guarantee) and inconsistent boundary context.
- **JavaScript `parseMesh`** (viewer) checks syntax, versions, counts,
  numeric scale, the report encoding, the boundary-context structure
  (curve kinds and parameters, source spans that join, region consistency)
  and, in v5, that each `BOUNDARY_LOOPS` group is one closed loop and the
  `POLYGON_LOOPS` sizes add up. For v1–v4 it does not reject several loops.
  It does **not** re-check mesh topology (e.g. duplicate triangles) or the
  recorded size/Delaunay guarantees, so it displays what the file claims.
- **Python `tools/plot_mesh.py`** checks the header, `ORDER 1`, counts,
  finite coordinates, three-node elements, index bounds and the v5 loop sizes
  (at least three edges each, summing to the edge count). It ignores the
  result trailer entirely and does not validate topology.

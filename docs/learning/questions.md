# Questions for a first reading

Use [first_mesh.cpp](../../examples/learning/first_mesh.cpp) and the
[walkthrough](README.md). Predict an answer before opening the source.

1. What describes the shape, what configures triangulation, and what does `generate_mesh` return?
2. What does passing `const CurveGeometry&` mean for copying and ownership?
3. Does rotating a rectangle exchange its stored width and height?
4. Why does ear clipping reject a positive interior target?
5. How do the boundary length and refinement length differ?
6. What happens when an initial fixed boundary edge exceeds the refinement target?
7. What happens at `max_steps`, and which report fields must a caller inspect?
8. Why do chord splitting and curve refinement produce different circle outlines?
9. What does `preserve_regions=false` change? Does it change stable curve-ID editing?
10. Which export/import calls let you resume refinement on curves?
11. Does constrained Delaunay size refinement guarantee a minimum angle?
12. What do triangle and boundary-edge indices refer to?

## Answers

1. A geometry describes the outline; `MeshOptions` chooses behavior. The free function returns `MeshResult` with mesh, context, options, and report.
2. The function reads the geometry through a reference without taking ownership. The result owns its data and boundary snapshot.
3. No. Rotation changes world coordinates, leaving local dimensions intact.
4. Ear clipping has no interior-refinement stage. Use CDT or explicit straight polygon subdivision.
5. The boundary length controls initial analytical sampling; the refinement target tests every triangle edge. Both default and disable rules are listed in the [reference](reference.md).
6. The result has `size_satisfied=false` and an explanatory message. Sample the initial boundary more finely or allow splitting.
7. A partial mesh returns with an explicit unmet report. Inspect `size_satisfied` and `message`; `refinement_steps` counts work.
8. Chord midpoints remain on the original sampled polygon. Curve mode uses source intervals to put new boundary points on the analytical circle.
9. It omits tags and lets redundant smooth region junctions disappear during sampling. Corners remain; stable curve-ID editing is unchanged.
10. `export_mesh(result, path)` writes v3; `import_mesh_result(path)` retains context for `refine_mesh(result, options)`. Raw mesh export is v2; topology-only import drops context.
11. No. Longest-edge midpoint insertion and full legalization enforce size where feasible, without a classical circumcenter quality guarantee. There is no minimum-angle setting.
12. Each triangle has three indices into `mesh.nodes`; a boundary edge has two vertex endpoints. `elements.size()` counts triangles, not unknowns or nodes.

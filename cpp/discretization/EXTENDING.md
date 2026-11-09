# Extending analytical geometry

Curve mathematics lives in `curve_segment.hpp`. Shared length integration and
length-fraction inversion live in `curve_length.hpp`; meshing policy remains in
`boundary_discretization.hpp` and `triangulate.hpp` (with its private
`detail/ear_clipping.hpp`, `detail/cdt_topology.hpp`,
`detail/curved_boundary.hpp`, `detail/refinement_policy.hpp`,
`detail/refinement_loop.hpp` and `detail/interior_algorithms.hpp`); size
criteria are public in `size_criterion.hpp`. Every public header is listed
in the [README header map](README.md#headers) (checked by
`tests/tools/readme_headers.cmake`). SVG and PDE2D_MESH adapters
remain separate from the mathematical curve objects. The concrete types remain
aggregates, so existing brace initialization continues to work.

## Adding a curve type

Implement these operations beside its data, then add it to `CurveSegment`:

- `point(t)`, `derivative(t)`, and `secondDerivative(t)` use the local parameter.
- `derivativeBounds()` returns `DerivativeBounds { first, second }` satisfying
  `first >= sup(norm(C'(t)))` and `second >= sup(norm(C''(t)))` over all of `[0,1]`.
- `split(t)` returns two exact curves of the same concrete type, each with its
  own `[0,1]` parameter. It rejects nonfinite parameters and values outside `(0,1)`.
- `reversed()` represents `C(1-t)` exactly.
- `transformed(f)` supports the positive uniform scale, rotation, and translation
  in `CurveTransform` (after subtracting its optional origin). The public `transformCurve` wrapper validates the transform.
  General affine transforms and reflections are outside this contract.
- `validate()` checks finite data and representation-specific invariants.
- `exactLength(end)` and `exactLengthParameter(fraction)` return an optional
  exact shortcut, or `std::nullopt` to use the shared numerical algorithm.
- `isLinear()` identifies a representation that needs only one sampling panel
  for SVG distance scaling and may be meshed without a curved-boundary tolerance.
  Returning false conservatively is appropriate for general curved types.

The public variant wrappers only dispatch. A missing operation causes a compile
error rather than falling through to another curve's implementation. Geometry
IDs, region labels, edit replay, connectivity checks, and subdivision budgets
are not responsibilities of an individual curve.

## Bounds and subdivision

The mesher bounds length by `first` and endpoint-chord deviation by `second/8`.
Derivative values at finitely many points do not establish an interval bound.
Bézier bounds use the convex hull of derivative control points: a quadratic has
first-derivative controls `2(P1-P0), 2(P2-P1)` and constant second derivative
`2(P0-2P1+P2)`; a cubic has first-derivative controls `3(Pi+1-Pi)` and
second-derivative controls `6(P0-2P1+P2), 6(P1-2P2+P3)`. Maximum control-vector
norms bound the whole derivative curve by convexity of the norm. For an ellipse,
rotation preserves norms and `max(rx,ry)` bounds the sine/cosine vector norm;
multiplication by `abs(sweep)` and `sweep*sweep` gives the two bounds.

For a source interval of width `h`, local derivatives scale by `h` and `h*h`.
Bounds on split pieces must use this local parameterization and tighten under
repeated subdivision. Conservative bounds are acceptable; excessive looseness
costs subdivisions. Current floating-point scale checks and subdivision budgets
still apply; this refactor does not introduce interval arithmetic.

`tests/discretization/curve_operations.cpp` checks derivatives, reversal, transformations,
exact split identities, split derivative scaling, and shrinking bounds. Its
sampled inequalities are regression checks, not proofs of interval bounds. Add
analytically justified bounds and regression cases for interior extrema and
stationary endpoints for every new curve.

## Interchange adapters

PDE2D_MESH v3 IDs are stable (unchanged in v4 and v5): line `0`, quadratic `1`,
cubic `2`, arc `3`.
`curve_io.hpp` assigns each concrete type an explicit `Codec` specialization
with an ID and read/write functions. Never derive IDs from variant ordering,
renumber existing IDs, or change an existing record's field layout. Decoding
iterates the variant alternatives but selects by the codec ID; duplicate IDs
are rejected at compilation. Unknown IDs are rejected at runtime. Tests decode
and encode existing records with a reordered variant to guard this separation.

Add SVG `needsDistinctEndpoints` and `writeCommand` overloads in
`svg_geometry.hpp`; SVG syntax and representation restrictions belong there.
If a curve has no exact SVG representation, explicitly reject it or design a
separate approximation policy rather than silently changing geometry.

The JavaScript viewer's `sourceCurveFormats` registry records wire IDs, field
counts, parameter validation, and point evaluation together. Update it when
extending the format and add C++/viewer roundtrip coverage. C++ and JavaScript
remain separate implementations of the same small format; no code generator
or runtime plugin registry is needed for the current supported types.

## Shapes and meshing options

A shape made from existing curve types only needs to construct `CurveRegion`s,
as the existing shape classes do. It does not need mesher type dispatch. A
shape with holes passes one region list per hole to
`rebuildRegions(outer, holes)` (see `AnnulusGeometry`); resize replay
compares loops as well as regions and curve kinds.

## Boundary loops

Every layer carries the outer loop and the holes explicitly: `CurveGeometry`
(`BoundaryCurve::loop`), `Boundary::holes`, `BoundaryContext`
(`snapshot_loop_sizes`, `curves[i].loop`) and the mesh (`boundary_loops`). Code
that walks a boundary must walk each loop on its own and close it on itself;
the outer loop is first and holes keep their order. When adding a meshing
step that rebuilds the boundary (like `rebuild_curved`), traverse
`boundary_loops(mesh)` in order so hole numbering stays equal to
`BoundaryCurve::loop` (`validate_boundary_context` checks it). The seed
triangulation (`detail/ear_clipping.hpp`) bridges holes into one ring;
a different seed (for example a CDT with hole removal) only has to return
triangles of the same polygon, since all loop edges become constraints.

`mesh_types.hpp` explicitly assigns stable algorithm and boundary-treatment IDs
and centralizes value validation in `validateMeshOptionValues`. Add new values
to the corresponding exhaustive validation switch. The viewer has matching
named ID registries. Generation-specific compatibility checks remain at the
operation boundary: persisted subdivision results can carry combinations that
are not valid requests to `generate_mesh`.

## Adding an interior algorithm

Each `InteriorAlgorithm` is one unit in `detail/interior_algorithms.hpp`
(`EarClippingAlgorithm`, `ConstrainedDelaunayAlgorithm`), mirroring the
boundary policies; `with_interior_algorithm` is the only dispatch, and no
other code compares `MeshOptions::algorithm` with an enum value. Steps:

1. Append a wire ID to `InteriorAlgorithm` (`mesh_types.hpp`; never
   renumber), its report name in `interior_algorithm_name` and its case in
   `validInteriorAlgorithm`.
2. Write the unit: `id`; `capabilities` (`InteriorAlgorithmCapabilities`:
   `supports_size_target`, `supports_size_field`, `supports_size_criterion`,
   `supports_refinement`, `supports_marked_refinement`,
   `constrained_delaunay`, `requires_boundary_context`);
   `size_refinement_unsupported` and `refinement_unsupported`, the messages
   of requests its capabilities exclude (tests match them, keep them
   stable); `generate(seeded, options)` from `seeded_result` (the
   ear-clipped polygon); and `refine(result, options, criterion)` when it
   supports refinement. A unit that refines should run the generic loop
   (`refine_result`, `detail/refinement_loop.hpp`) so every size criterion
   and boundary policy works unchanged; topology operations belong in
   `detail/cdt_topology.hpp`, curve-aware boundary updates in
   `detail/curved_boundary.hpp`.
3. Add its case to `with_interior_algorithm`; the `InteriorAlgorithmUnit`
   concept fails to compile for an incomplete unit (a unit declaring
   `supports_refinement` must have `refine`).
4. Update both format readers and the viewer's ID registry
   (`tools/mesh_viewer/mesh.js`, `tools/plot_mesh.py`), the FEM mesh plan
   name if needed, and add tests: `interior_algorithm_capabilities` values,
   the excluded requests' messages and meshes of every boundary treatment
   (see `tests/discretization/refinement_criterion.cpp`).
5. Work counters: the generic loop and `Topology` count refinement steps,
   flips and rebuilds into `MeshResult::record.work` (`MeshWork`); a unit
   with its own topology work sets `Topology::work` (or counts into
   `result.record.work`) the same way. Counting must never change the mesh.
   The public entry points in `triangulate.hpp` complete the record
   (`finish_mesh_run_record`); a new entry point does the same.

## Refinement: size criteria and boundary policies

`detail/refinement_loop.hpp` runs one generic loop, `refine_with<Policy>`:
build the topology, legalize, ask the policy for a boundary conflict, then
repeatedly split the highest-priority edge that fails the size criterion
until none is left, the step budget runs out or midpoints stop resolving.
Two seams plug into it: size criteria (public, `size_criterion.hpp`) and
boundary policies (`detail/refinement_policy.hpp`):

- **Size criteria.** A `SizeCriterion` maps an edge's endpoints to
  `std::nullopt` (accepted) or a `SizePriority {tier, value, length}`
  (higher first; ties go to the first lexicographic edge).
  `RefinementCriterion` adds the Fixed-conflict message, an optional
  `split_hook` called after every split and a `StrategyDescriptor` for
  records. Built in: the constant target (`constant_size_criterion`,
  `{0, length, length}`, the historical order), a `SizeField`
  (`size_field_criterion`, `{0, length / target, length}`) and marked edges
  (tiers 2 and 1, falling back to the size criterion; private
  `marked_criterion`). A user criterion, for example a problem-aware target
  or an anisotropic metric, needs no library edit: set
  `RefinementOptions::criterion` (it replaces `max_edge_length` and
  `size_field`; compose them with `strictest_of`). `size_criterion(options)`
  in `detail/refinement_policy.hpp` turns options into the criterion; keep
  the constant-target path unchanged so existing results stay
  bit-identical.
- **Boundary policies.** One struct per `BoundaryTreatment`
  (`FixedBoundary`, `SplitPolygonBoundary`, `CurveBoundary`) holds
  everything that treatment decides: `validate` (required context),
  `prepare` (curve mode first moves chord vertices onto the curves),
  `conflict` (Fixed: a boundary edge failing the criterion makes the request
  impossible) and `split_boundary`. `with_boundary_policy` is the only
  dispatch. A new treatment needs a wire ID in `mesh_types.hpp`, its case in
  `validBoundaryTreatment` and `with_boundary_policy`, the format readers and
  viewer registry, and tests comparing it with the existing treatments.

Request-time options such as the size field and the refinement criterion
are not serialized
(`docs/formats/pde2d-mesh.md`). A new option that cannot be written must be
documented the same way rather than silently dropped. Tests:
`tests/discretization/size_field.cpp` (graded meshes; constant field equals
the scalar path bit for bit) and `marked_refinement.cpp`.

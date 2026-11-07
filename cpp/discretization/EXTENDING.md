# Extending analytical geometry

Curve mathematics lives in `curve_segment.hpp`. Shared length integration and
length-fraction inversion live in `curve_length.hpp`; meshing policy remains in
`boundary_discretization.hpp` and `triangulate.hpp`. SVG and PDE2D_MESH adapters
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

PDE2D_MESH v3 IDs are stable: line `0`, quadratic `1`, cubic `2`, arc `3`.
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
as the existing shape classes do. It does not need mesher type dispatch.

`mesh_types.hpp` explicitly assigns stable algorithm and boundary-treatment IDs
and centralizes value validation in `validateMeshOptionValues`. Add new values
to the corresponding exhaustive validation switch. The viewer has matching
named ID registries. Generation-specific compatibility checks remain at the
operation boundary: persisted subdivision results can carry combinations that
are not valid requests to `generate_mesh`.

When adding an algorithm, implement its dispatch and execution compatibility in
`triangulate.hpp`, update both format readers and UI choices, and add algorithm
combination tests. With the current two algorithms and three boundary policies,
this explicit dispatch is preferable to a polymorphic strategy hierarchy.

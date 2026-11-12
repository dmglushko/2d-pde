# Agent guide

C++23 library (compiled, not header-only) for 2D geometry and triangle
meshing, the first slice of a from-scratch rewrite that will grow into one
linear scalar PDE with P1 finite elements. Read
[docs/decisions.md](docs/decisions.md) (the owner's decisions: your
specification), then [docs/architecture.md](docs/architecture.md) (modules,
types, data flow) and [docs/vision.md](docs/vision.md) (direction).

## Architecture decisions belong to the owner

**Architecture decisions belong to the owner. Do not make a design choice
implicitly (types, interfaces, data flow, defaults, fallbacks, error
policy). Present options, trade-offs and a recommendation, and wait. If a
task needs a choice that docs/decisions.md doesn't cover, raise it instead
of picking.**

The project exists so the owner learns; a choice made silently is one the
owner never understood. Private details (helper names, loop structure)
are yours. Never contradict `docs/decisions.md`; if something there is
impossible or unclear, ask.

## Where things are

| Path | Contents |
| --- | --- |
| `include/pde2d/geometry/` | `point.hpp`, `predicates.hpp` (exact `orient`, `incircle`), `curve.hpp` (curve kinds and math), `polygon.hpp`, `curve_geometry.hpp` (`Loop`, `CurveGeometry`), `shapes.hpp` |
| `src/geometry/` | their implementation; `predicates.cpp` is Shewchuk's adaptive arithmetic, compiled with `-fno-fast-math -ffp-contract=off` |
| `include/pde2d/mesh/` | `mesh.hpp` (output `Mesh`), `mesh_spec.hpp`, `boundary_polygon.hpp`, `working_mesh.hpp`, `report.hpp`, `components.hpp` (interfaces, capabilities, `check_compatibility`), `delaunay.hpp` (shared flip routines), one header per component, `remesher.hpp`, `mesher.hpp` |
| `src/mesh/` | implementation; private headers `delaunay_internal.hpp`, `polygon_nodes.hpp`, `stage.hpp` |
| `tests/<module>/` | one test program per topic, registered in that directory's `CMakeLists.txt` |
| `tests/support/` | `check.hpp` (test helper), `mesh_checks.hpp` (independent mesh oracles), `shapes.hpp` (test geometries) |
| `tools/` | `test.sh` (configure, build, ctest), `format.sh` |
| `docs/` | `decisions.md`, `architecture.md`, `vision.md` |

The previous code is on the branch `archive/header-only-v1`. It is a
reference, not a source: only the predicates and curve math were ported.
Read it with `git show archive/header-only-v1:<path>`; do not check it out
in this checkout.

## Build and test

Run builds inside Linux/WSL with native tools (the checkout is on ext4).

```sh
tools/test.sh                          # configure, build, run all tests
tools/test.sh -DPDE2D_WARNINGS_AS_ERRORS=ON
tools/format.sh --check                # clang-format; tools/format.sh fixes
ctest --test-dir build -R mesh_pipeline --output-on-failure   # one test
```

`PDE2D_BUILD_DIR` picks the build directory, `PDE2D_BUILD_JOBS` the build
parallelism (capped at 4; run one build at a time), `PDE2D_TEST_JOBS` the
test parallelism. Every test program prints `PASS/FAIL <case>`.

## Conventions

- Names: snake_case for functions, variables and members; PascalCase for
  types; enumerators in snake_case.
- One CMake library per module (`pde2d_geometry`, `pde2d_mesh`; later
  `pde2d_io`, `pde2d_fem`). Dependencies point one way: mesh may include
  geometry, never the reverse. Heavy dependencies (Boost, later Eigen,
  SymEngine, libxml2) only in `.cpp` files, never in public headers.
- Public headers in `include/pde2d/<module>/`; private code and private
  headers in `src/<module>/`. Tests use only public headers.
- Mesh components are virtual interfaces (`components.hpp`). Each
  component owns its options struct, set with designated initializers in
  its constructor, and validates it there. A component declares its
  capabilities or effects; compatibility is decided from those
  declarations, never from names or type checks.
- Steps an algorithm needs for correctness belong inside the component
  (Ruppert makes its own input Delaunay). Optional quality passes are
  improvers. Shared routines are plain functions (`delaunay.hpp`).
- Errors are exceptions with specific messages: `std::invalid_argument`
  for bad input or configuration, `std::runtime_error` for an algorithm
  that cannot finish. Report messages only mean "worked, but…". Tests
  match message fragments, so keep messages stable or update the tests.
- Deterministic results: no dependence on hash iteration order or
  addresses; process in index order, break ties by index.
- Style: clang-format (LLVM, 80 columns).

## Invariants that are easy to break

- **Domain on the left.** `CurveGeometry` normalizes loops (even depth
  counterclockwise, odd depth clockwise); `BoundaryPolygon`, `WorkingMesh`
  and `Mesh` boundary edges all have the domain on their left. Triangles
  are counterclockwise.
- **Depth and parent are computed**, stored apart from `Loop`, never set by
  the user.
- **WorkingMesh edge convention:** edge `i` of a triangle is opposite
  vertex `i`. Edits never delete triangles or nodes, and must leave the
  mesh valid or throw without changing it.
- **Boundary edges carry region tags and optional curve links**; a refiner
  must update them when it splits (use `split_boundary_edge`). A linked
  edge is split at the curve point, an unlinked one at the chord midpoint.
- **The Mesher never sees FEM data.** Its only external input is the
  `MeshSpec`.
- **`MeshResult::target_met` must be checked**: a mesh can miss its target
  (for example after smoothing or without a refiner).
- **Not yet implemented, on purpose** (see "Deferred" in decisions.md):
  FEM, problems and boundary conditions, anisotropic meshing, planners,
  the simple-options facade, and IO (SVG, `.pde2d`, Gmsh) and the viewer.
  Do not add them unless a task asks.

## Tests

- Register a test with
  `pde2d_add_test(NAME <module>_<topic> SOURCE <file> LIBS <library>)` in
  `tests/<module>/CMakeLists.txt`.
- Use `tests/support/check.hpp` (`check`, `near`, `expect_throws`, `run`)
  and, for meshes, `tests/support/mesh_checks.hpp`.
- Oracles: exact values, invariants (orientation, conformity, edges used
  once equal the boundary edges, area, Delaunay, minimum angle, edge
  lengths) or independent computations. Do not copy the algorithm under
  test into the test.
- `header_standalone` compiles every public header on its own.

## Before finishing a change

1. `tools/format.sh --check`
2. `tools/test.sh` (all tests pass)
3. Update `docs/architecture.md` when you change a public type, interface
   or the data flow, and this file when you change layout or conventions.
   Decisions go to the owner, not into the docs on your own.

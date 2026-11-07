# Benchmarks

Routine profiling is native to the solver reports. For the standard wave
comparison, configure OpenMP and run:

```sh
./build/fem_long_waves --benchmark
```

Standalone experiments live here, rather than in the test suite or documentation.
Enable their CMake targets with `-DPDE2D_BUILD_BENCHMARKS=ON`:

| Target / script | Purpose | Additional dependency |
| --- | --- | --- |
| `benchmark_mesh_issue3` | Mesh refinement and geometric validation | None |
| `benchmark_geometry_queries` | Geometry queries, copying, sampling and SVG import | LibXml2 |
| `benchmark_prepared_assembly` | Constant and time-dependent assembly | FEM module |
| `benchmark_error_diagnostics` | Original versus prepared error norms | FEM module |
| `benchmark_evolution` | Solver preparation/reuse through evolution | FEM module |
| `benchmark_viewer_diagnostics.cjs` | Viewer JavaScript work in a Node VM | Node.js |
| `benchmark_long_waves.mjs` | Historical multi-revision wave comparison | Node.js and three binaries |

Run binaries from the repository root. The geometry benchmark writes
`build/geometry-benchmark.svg` by default, or accepts an output path. The viewer
script takes one or more viewer source directories, for example
`node benchmarks/benchmark_viewer_diagnostics.cjs tools/mesh_viewer`.

Benchmarks are opt-in and have no wall-time pass/fail thresholds. Tests remain
in `tests/`; current and historical measurements are indexed in
[the performance documentation](../docs/performance/README.md).

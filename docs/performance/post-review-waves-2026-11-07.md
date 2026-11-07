# Wave benchmarking after the five review fixes

Measured source: `83c8071671e22585387672b1536deba3b5850c6c`. All five fixes are included in the integration checkout. The comparison uses the original two 100-second waves on `[0,10]^2`: 441 vertices, 800 P1 triangles, degree-six quadrature, Newmark average acceleration, `dt=0.05`, 2,000 steps and 401 retained frames per case. SparseLU and default `Checked` validation are unchanged across runs. The new `AssumeValid` iterative policy is not used.

| Configuration | Total median (range), s | Driven integration, s | Neumann integration, s | Neumann diagnostics, s |
|---|---:|---:|---:|---:|
| OpenMP compiled OFF | 14.317 (14.262–15.165) | 6.047 | 0.456 | 7.594 |
| OpenMP build · serial | 14.627 (13.513–16.846) | 6.234 | 0.448 | 7.435 |
| OpenMP · 2 workers | 7.948 (7.800–8.433) | 3.324 | 0.447 | 3.990 |
| OpenMP · 4 workers | 6.491 (5.086–9.010) | 2.659 | 0.655 | 2.953 |

Three repetitions per configuration; retain every run. Integration columns exclude native observer diagnostics. Total includes mesh, diagnostic setup, integration, diagnostics and data export. Columns are nested and must not be summed. The chart provides the measured min/max ranges and additional phase views.

Machine: AMD Ryzen 9 6900HX with Radeon Graphics (16 logical CPUs), 6.18.33.2-microsoft-standard-WSL2. c++ (Ubuntu 15.2.0-16ubuntu1) 15.2.0. Release `-O3 -DNDEBUG -std=gnu++23`, with `-fopenmp` only for the OpenMP binary. `OMP_DYNAMIC=FALSE`, `OMP_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1`, `MKL_NUM_THREADS=1`; the PDE kernels explicitly request 2/4 threads. This keeps unrelated runtime defaults fixed. Actual team size is not measured.

The native benchmark rotates serial/2/4-worker order over three rounds. The separately compiled OFF runs follow that batch. All processes ran sequentially after builds and tests finished, without discarded warmups. Frequency and external machine load were not controlled; these are workstation measurements, not a general speed guarantee.

All 12 datasets passed complete case inventories, 401-frame/2,001-observation checks, finite diagnostics and numerical comparisons against the first serial run. The native benchmark also compares retained velocities. Energy/work and Neumann mean checks run inside each simulation. Both serial and OpenMP builds passed 35/35 CTests, including independent scikit-fem/SciPy comparisons, export-failure regressions and the benchmark CLI.

[Interactive comparison](post-review-waves-2026-11-07.html) · [Full profiles and provenance](post-review-waves-2026-11-07.json)

## Routine profiling

Every solve already populates native timing reports. No rerun, profiling option or external script is needed. Normal wave runs print the summary and write a `.timings.json` sidecar. Library callers can use:

```cpp
#include "fem/report.hpp"
auto result = fem::evolve(mesh, problem, initial, velocity, options);
fem::write_report(std::cout, result);
```

The report includes execution policy, workload, assembly, constraints, solver setup/solve/residual phases, native observer time, other time, and preparation/reuse counts. Aggregate timings remain available with `store_reports=false`. Stationary summaries currently cover linear solving, not outer assembly. Parent totals include their nested subphases.

## Reproduce the comparison

Configure the OpenMP build once, then use the native option:

```sh
./build/fem_long_waves --benchmark build/wave-benchmark 3 20 0.05
```

This saves every dataset and phase profile plus `benchmark.json` with medians and ranges. To assess compile-time OpenMP overhead, configure another build with `PDE2D_ENABLE_OPENMP=OFF` and repeat its normal command three times with unique output paths:

```sh
./build/serial/fem_long_waves build/openmp-off-1.json 20 0.05 1
```

Raw datasets for this session remain under `build/bench-post-review/`; binary SHA-256 values and every captured profile are retained in the JSON report. The historical multi-binary benchmark script is unnecessary for routine profiling or the built-in serial/OpenMP comparison.

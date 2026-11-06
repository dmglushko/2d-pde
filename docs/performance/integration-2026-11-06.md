# Integrated performance and verification work

Starting point: `695d785`. The [audit](audit-2026-11-06.md) preceded the
implementation and filed seven GitHub issues alongside the two existing ones.
Six implementation agents worked in isolated T3 worktrees; the parent integrated
their commits and fixes. Three fresh independent review agents examined the
combined implementation. Only OpenAI models were used.

## Changes and issue coverage

| Issue | Integrated result | Focused evidence |
| --- | --- | --- |
| [#3](https://github.com/dmglushko/2d-pde/issues/3) | Persistent straight-refinement topology, generation-checked size queue, local legalization, conservative spatial pruning and cheaper validated I/O/curved fallbacks | [Mesh report](mesh-issue3/README.md), stale/flip-created edge regressions and baseline comparison |
| [#4](https://github.com/dmglushko/2d-pde/issues/4) | Independent synchronized geometry length caches with documented read-only/copy concurrency | [Geometry report](geometry-issue48-2026-11-06.md), cold/hit/copy concurrency and TSan |
| [#5](https://github.com/dmglushko/2d-pde/issues/5) | Immutable prepared geometry/evaluators/quadrature and constant local contributions; correct time-dependent reevaluation | [Assembly report](assembly-2026-11-06.md), mutation/invalidation and legacy comparison |
| [#6](https://github.com/dmglushko/2d-pde/issues/6) | Owned prepared solvers; exact matrix/options reuse; stride/report/observer controls; stable nominal steps | [Solver report](solver6/README.md), initial/partial-step and changing-data tests |
| [#7](https://github.com/dmglushko/2d-pde/issues/7) | Prepared deterministic error diagnostics, every-step wave observations, cached viewer invariants, coalesced redraw and streaming lexer | [Diagnostics/viewer report](issue-7-diagnostics-viewers.md), C++ and Node regressions |
| [#8](https://github.com/dmglushko/2d-pde/issues/8) | Curve/region indexes, safeguarded length inversion, prepared sampling intervals and indexed SVG endpoints | [Geometry report](geometry-issue48-2026-11-06.md), extreme-scale inversion, ambiguity and byte-exact sampling tests |
| [#9](https://github.com/dmglushko/2d-pde/issues/9) | Incremental dependency-light builds, private fixtures/job controls, unchanged quadrature coverage with cached powers | [Build report](verification-build-2026-11-06.md), geometry-only consumer and incremental rebuild |
| [#10](https://github.com/dmglushko/2d-pde/issues/10) | Explicit optional OpenMP numerical kernels, analytical convergence and independent scikit-fem/SciPy references | [Verification guide](verification-build-2026-11-06.md) and [metrics](verification-results-2026-11-06.json) |
| [#11](https://github.com/dmglushko/2d-pde/issues/11) | v4 topology plotting with one parse per input | Fresh 19-mesh PNG/gallery roundtrip plus malformed topology tests |

## Final combined verification

On the combined checkout, Release builds with the installed SymEngine 0.14.0
passed **31/31 serial tests and 31/31 OpenMP-enabled tests**, including executed
references and 19 freshly exported v4 PNGs. The geometry-only wrapper passed
**17/17 tests** after configuring its existing cache with `BUILD_TESTING=OFF`;
the wrapper correctly reenabled testing. Eigen and SymEngine discovery were
disabled in that geometry configuration. No global Python dependencies changed;
the interpreter was explicitly set to the implementation agent's isolated venv.
Exact commands and the full logs are in the parent checkout's ignored `build/`:

```sh
PDE2D_BUILD_DIR=build tools/test_fem.sh \
  -DSymEngine_DIR=/tmp/pde2d-fem-build/_deps/symengine-build \
  -DPDE2D_FETCH_SYMENGINE=OFF -DPDE2D_ENABLE_REFERENCES=ON \
  -DPDE2D_REFERENCE_PYTHON=/home/gldima/.t3/worktrees/2d-pde/t3-9fca12b4/.venv/bin/python
PDE2D_BUILD_DIR=build/openmp tools/test_fem.sh -DPDE2D_ENABLE_OPENMP=ON \
  -DSymEngine_DIR=/tmp/pde2d-fem-build/_deps/symengine-build \
  -DPDE2D_FETCH_SYMENGINE=OFF -DPDE2D_ENABLE_REFERENCES=ON \
  -DPDE2D_REFERENCE_PYTHON=/home/gldima/.t3/worktrees/2d-pde/t3-9fca12b4/.venv/bin/python
PDE2D_BUILD_DIR=build/geometry-only tools/test.sh \
  -DCMAKE_DISABLE_FIND_PACKAGE_Eigen3=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_SymEngine=TRUE
```

The independent comparison uses scikit-fem 12.0.1 and SciPy 1.17.1. Maximum
assembly discrepancy was `3.553e-15`; stationary nodal discrepancy was
`1.333e-15`. Independent heat/wave ODE references passed first-order BE and
second-order CN/Newmark refinement checks. The OpenMP reference compared all
stationary and nine evolution results at serial/2/4-worker policies exactly.
The execution-policy regression also observes `omp_get_num_threads()` inside
workers, with dynamic teams disabled, and verifies actual teams of two/four plus
serial fallback below the work threshold.
See the metrics for step sizes, errors, versions and operation counts.

Independent FEM review additionally exercised concurrent prepared assembly,
caller mutation, nonmonotonic times, `log(t)` coefficients, dynamic Robin/moving
Dirichlet data (maximum nodal error `1.125e-14`), observer/storage and genuinely
shortened final steps. Geometry review compared 360 scaled polygon refinements,
100 edit/copy/replay cases, 50 samplings and 36 provenance/resume cases against
the baseline; outputs matched. It also checked 5,005 extreme-scale AABB queries
against exhaustive enumeration and 73,989 accepted tolerance contacts against
the broad phase. Neither core review found a confirmed correctness defect.

Four verification gaps were found and corrected by the parent:

- CMake passed a fixture argument to the sampling regression, activating its
  print-only mode. Only tests that consume directories now receive them. A
  deliberately corrupted signature reproduced the old false pass; the real
  regression now prints its assertion-success message under CTest.
- Test wrappers could preserve `BUILD_TESTING=OFF` and report success with zero
  tests. They now force testing on and use `--no-tests=error`. An empty CTest
  directory returned exit 8; the disabled-cache geometry run executed 17 tests.
- The external verifier could accept one time step per method, making its rate
  assertion vacuous. It now requires the exact nine-run method/step inventory
  and writes measured step sizes. The review's single-step substitute now fails.
- Moving-boundary convergence could hide NaNs in `std::max`. It now checks finite
  displacement/velocity, the endpoint, and positive finite errors explicitly.

The preparation integration also preserves value-only boundary inputs whose
unused time derivatives cannot be evaluated by SymEngine: unsupported derivatives
are deferred until requested and their errors are never silently replaced by zero.

Targeted geometry/mesh ASan, UBSan and TSan runs are recorded in their reports.
The final combined suites were ordinary Release runs, not a claim of full-engine
sanitizer coverage or a proof of race freedom.

## Parallel policy and retained limitations

Serial remains the runtime default even in an OpenMP build. Element/edge numeric
assembly, initial values and prepared error integration accept explicit policies.
Symbolic preparation and sparse reduction stay serial; workers write independent
outputs and reduction order is fixed. Missing OpenMP is an explicit error.
Time steps and mutation-heavy topology updates are sequential.

Spatial indexes prune validation candidates; those pruned validation loops stay
serial. This avoids adding a thread pool to small residual work without a measured
benefit. Curved boundary updates retain the global rebuild/legalization fallback,
triangle point location still scans conservative boxes, and ordered boundary
insertions and greedy simplification retain their ordering. Geometry cache
synchronization increases copy cost. Small-work threading can cost more than it
saves; optional execution is not a universal scaling claim.

## Long-wave reproduction

`tools/benchmark_long_waves.mjs` runs each policy as a separate sequential process,
keeps raw fields/profiles/logs, checks all retained states against serial, and
records binary hashes and environment. Run after builds and other tasks finish:

```sh
node tools/benchmark_long_waves.mjs /tmp/pde2d-baseline-long-waves-695d785 \
  build/fem_long_waves build/openmp/fem_long_waves build/wave-benchmark 3
```

The old run measured diagnostics after integration; the new observer measures
them inside integration. Compare `integration_excluding_observer_seconds` with
the old `integration_seconds`, and report diagnostics separately. Parent totals
include subphases. Every run uses the same 10-by-10 square, 441 nodes, 800 P1
triangles, degree-six assembly, `dt=.05`, 100 simulated seconds and 2,000 steps.
No correctness test asserts a wall time.

Three sequential repetitions per policy finished after all agents and compiler
jobs stopped. Hardware: AMD Ryzen 9 6900HX, 16 logical CPUs, Linux/WSL; GCC 15.2,
C++23, Release `-O3 -DNDEBUG`, no fast-math. `OMP_DYNAMIC=FALSE`, explicit 1/2/4
policies, and external BLAS thread limits of one were recorded. CPU frequency,
background host load and affinity were not controlled. These are observations
on this machine, not a scaling guarantee.

Median wall seconds (three samples; full ranges in the
[recorded profiles](waves-benchmark-2026-11-06.json)):

| Policy | Complete two-case execution | Driven integration, excluding diagnostics | Free integration, excluding diagnostics | Free exact/energy diagnostics |
| --- | ---: | ---: | ---: | ---: |
| Baseline `695d785`, serial | 89.471 | 39.871 | 41.396 | 8.091 |
| Prepared, serial | 13.578 | 5.793 | 0.425 | 7.145 |
| Prepared, 2 workers | 7.706 | 3.208 | 0.433 | 3.825 |
| Prepared, 4 workers | 4.995 | 2.132 | 0.433 | 2.290 |

The complete execution includes both cases, diagnostic setup, diagnostics and
JSON export; it excludes HTML packaging. The serial result is about 6.6 times
faster than the measured baseline; four workers are about 2.7 times faster than
prepared serial for this complete workload. The small cached free-integration
kernel shows no observed threading benefit. An [interactive chart](waves-benchmark.html)
separates these costs.

Each updated case recorded 2,003 assembly calls, 2,001 solves, **two rank checks
and two factor preparations**, 1,999 reuses, 2,001 observations and 401 retained
frames. The baseline repeated matrix preparation for each solve. Applying LU
factors still takes only about .03–.05 seconds across 2,001 solves. Removing
repeated preparation explains the large gain; dynamic source evaluation now
dominates driven assembly, while exact-solution diagnostics dominate the free
case and benefit from parallel element work.

All new serial/2/4-worker fields and diagnostics matched exactly across the
three rounds. Maximum nodal difference against the older, ten-digit serialized
baseline was `5.107e-10`. Updated maximum energy/work defects were below `4.5e-15`
and free mean drift below `2e-12`. No source was added to the Neumann case.
Full galleries were regenerated from computed states; their standalone timing
file represents serial sample 1, while the comparison above uses medians.

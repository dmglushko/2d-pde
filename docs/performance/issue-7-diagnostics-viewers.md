# Issue 7: prepared diagnostics and viewer work

Implementation baseline: 48b4fe7. Original diagnostic/viewer sources and the
read-only historical wave dataset were copied to `/tmp/pde2d-issue7-baseline`
before edits; the original dataset was never modified. Generated HTML and timing
profiles are temporary outputs under `/tmp/pde2d-issue7-output`. No tracked
artifacts were regenerated.

`PreparedErrorNorms` owns physical quadrature points/weights, basis gradients,
connectivity and exact/derivative evaluators. Construction does all symbolic
work and generates the reference rule once. Numeric workers borrow the prepared
object and write separate element contributions, reduced in element order.
The trailing default execution policy preserves the `error_norms(...)` API.
Prepared calls reject invalid values/time and unavailable execution backends.
The memory tradeoff is physical quadrature storage proportional to elements
and rule points, plus two scalars per element during each evaluation.

Long waves observes every accepted state (2,001 callbacks at default settings),
but retains 401 states, with one previous driven state for source work. Energy,
mean drift and source work checks run on all steps; exact errors run on retained
states. JSON uses 17 digits, preserves endpoints, and omits analytical metrics
for the driven case. Solver reports remain retained deliberately to compute the
largest residual truthfully; they contain no state trajectories. Counts for
steps, observations, frames and reports are exported.

Integration includes observer diagnostics, as does the evolution's other-work
bucket. `diagnostics_seconds` measures callback body work; subtraction gives the
reported observer-exclusive integration estimate (callback dispatch/timer
bookkeeping remains in integration). Setup is separate, as are data export and
HTML packaging. Nodal exact export evaluation is part of export time. These
quantities must not be added as independent phases. The gallery labels overlap.

Solution data is immutable: extrema are keyed by case and field and amplitude
series are cached at load time; normalized mesh coordinates and depth order are
also prepared once. Base projection keys include case, field, view and canvas
size. Canvas backing stores change only when size or DPR changes. Mesh projection
keys include pan/zoom/size, and successful mesh load invalidates the projection.
Pointer events update view state immediately but request only one draw per
animation frame. The parser counts tokens then scans them without retaining the
full token array; early section-size guards still run before array allocation.
This saves token retention, not a measured peak-memory quantity.

## Verification

- CMake Release/OpenMP build in `/tmp/pde2d-issue7-build`, GCC 15.2, C++23,
  `-DPDE2D_ENABLE_OPENMP=ON`, reused SymEngine package under
  `/tmp/pde2d-fem-build/_deps/symengine-build`, at most two compile jobs.
- `ctest --test-dir /tmp/pde2d-issue7-build -R '^fem_error_diagnostics$' --output-on-failure`:
  original-loop, compatibility, time-varying/spatial exact function, affine exact
  reproduction, analytical quadratic integral, size validation and bitwise
  serial/two-thread agreement. Also compiled/run directly without `-fopenmp` to
  check serial operation and explicit rejection of unavailable OpenMP.
- `node tools/mesh_viewer/test_mesh.cjs`: versions 1–4, uint64 IDs, malformed
  metadata/counts/truncation, upload preservation, selection, slow pan, zoom,
  cancellation, redraw coalescing, resize and fit.
- `node tests/viewer_diagnostics.cjs`: actual generated standalone script with a
  deterministic DOM/canvas; frame/case/field/map switches, fixed scale, canvas
  reuse, resized projection, speed/time playback, wrapping, pause and scrubbing.
- `fem_long_waves /tmp/pde2d-issue7-output/waves.json 20 .05`, then
  `node tests/long_waves_diagnostics.cjs /tmp/pde2d-issue7-output/waves.json /tmp/pde2d-issue7-baseline/long-waves.json`:
  401 frames, 2,001 observations, exact endpoints, aligned diagnostics, no driven
  error metrics, timing overlap. Largest old/new nodal difference 5.11e-10;
  energy/mean/work/norm differences below 6.65e-11 (old JSON has ten digits).
- Full and `--compact` HTML packaged to temporary files; showcase producer built
  and run. No browser rasterization assertions are made by the Node harness.

## Measurements

Single wall-clock samples on AMD Ryzen 9 6900HX (16 logical CPUs), Node 24.22.1,
Release C++ `-O3`, shared machine. These are not controlled speedup estimates;
compilation/other agents may contend. Benchmarks have no timing thresholds.

`tools/benchmark_error_diagnostics.cpp`, 800 triangles, degree 8, 40 evaluations,
identical checksum 28.3426:

| Work | Seconds |
| --- | ---: |
| Original rule/derivative rebuilding loop | 0.360562 |
| Prepared serial evaluation | 0.336280 |
| Prepared two-thread OpenMP evaluation | 0.172083 |

Preparation is outside prepared evaluation timing. Evaluating the exact function
still dominates the free-wave diagnostic path; preparation does not eliminate
that work. The full wave run measured driven integration 5.95143 s including
0.015644 s diagnostics and free integration 7.67625 s including 7.24358 s
diagnostics. Full-step energy/work defects: 8.33e-16 driven, 4.44e-15 free;
free mean drift 1.92e-12. These integration times include previously integrated
assembly/solver changes and are not attributed to this issue alone.

`node tests/viewer_diagnostics.cjs BASELINE.html CURRENT.html`, same historical
441-node / 800-triangle / 401-frame case, 20 surface redraws, no-op canvas:

| Work | Baseline | Current |
| --- | ---: | ---: |
| Initial script/setup seconds | 0.153805 | 0.273336 |
| 20 redraws seconds | 1.043935 | 0.040235 |
| Backing-store resets during redraws | 40 | 0 |

Load work intentionally increases to prepare caches for all cases. Measurements
include VM/JavaScript execution, not actual browser drawing or layout.

`node tools/benchmark_viewer_diagnostics.cjs /tmp/pde2d-issue7-baseline tools/mesh_viewer`,
398,128-byte v1 grid, 10,201 nodes, 20,000 triangles:

| Work | Baseline | Current |
| --- | ---: | ---: |
| Five imports seconds | 0.507257 | 0.467634 |
| Burst of 100 pointer moves + draw seconds | 2.180066 | 0.012473 |
| Draws for that burst | 100 | 1 |
| Calls projecting a point for that burst | 6,080,000 | 10,201 |

The burst models events arriving before an animation frame. Real events spread
across frames produce more draws. Parser measurement is throughput only; no
peak-memory claim is inferred from it. Baseline paths are temporary local
artifacts; benchmark scripts accept source directories/HTML paths explicitly.

Standalone benchmark compilation used:

```sh
c++ -std=c++23 -O3 -fopenmp -Icpp -isystem /usr/include/eigen3 \
  -isystem /tmp/pde2d-fem-build/_deps/symengine-src \
  -isystem /tmp/pde2d-fem-build/_deps/symengine-build \
  tools/benchmark_error_diagnostics.cpp \
  /tmp/pde2d-fem-build/_deps/symengine-build/symengine/libsymengine.a \
  -o /tmp/pde2d-issue7-output/benchmark-diagnostics
/tmp/pde2d-issue7-output/benchmark-diagnostics
```

The serial-only test used the same compile command without `-fopenmp`, replacing
benchmark source with `tests/fem_error_diagnostics.cpp`. The diagnostic regression
also checks a translated/sheared/scaled mesh and verifies prepared geometry
survives subsequent changes to the input mesh. Final full and compact wave HTML
and showcase HTML were executed by the Node harness as well as packaged.

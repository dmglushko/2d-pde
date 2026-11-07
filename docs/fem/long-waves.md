# Driven and source-free waves over 100 seconds

Open [the two-case wave gallery](long-waves.html). Both calculations use the
square `[0,10] × [0,10]`, wave speed 1, a 20-by-20 grid split into 800 P1
triangles (441 vertices), degree-six integration, and `dt=0.05 s` through
`t=100 s`. Each calculation takes 2,000 time steps. Saved frames are spaced
0.25 simulated seconds apart; diagnostics check every integration step.
The viewer opens at `t=20` and offers 1×, 4×, and 10× playback. The slider also
includes the initial state.

The driven case fixes `u=0` on all four sides, starts at rest, and uses

```
f(x,y,t) = 0.15 exp(-((x-3)^2+(y-5)^2)/1.2)
                 * (1-exp(-t^2/4)) * sin(0.8t).
```

This is a localized oscillating force with a smooth turn-on. No exact solution
is asserted: the error fields are disabled and the error norms display dashes.
The energy change is compared against the accumulated work of the source.

The Neumann case uses `du/dn=0` on every side and **identically zero source**.
Its initial velocity is zero and its initial displacement is

```
0.2 + cos(pi*x/10)*cos(pi*y/10)
    + 0.35*cos(2*pi*x/10)*cos(pi*y/10).
```

Its continuous exact solution multiplies the first cosine mode by
`cos(sqrt(2)*pi*t/10)` and the second by `cos(sqrt(5)*pi*t/10)`.
The gallery compares against this formula and exposes accumulated spatial and
temporal phase error. The mass-weighted mean and discrete energy are checked
for conservation throughout the run. The conserved discrete mean is that of
the initial P1 interpolant; it need not exactly equal the continuous mean 0.2.

## Time integration

Spatial assembly produces

```
M2 * acceleration + M1 * velocity + K * displacement = F(t).
```

Both cases use implicit Newmark average acceleration, with `beta=1/4` and
`gamma=1/2`. For a time step `h`, the code predicts

```
u_pred = u_n + h*v_n + (h*h/4)*a_n
v_pred = v_n + (h/2)*a_n
```

and solves the sparse linear system

```
[M2 + (h/2)*M1 + (h*h/4)*K] * a_next
    = F(t_next) - K*u_pred - M1*v_pred.
```

It then sets `u_next=u_pred+(h*h/4)*a_next` and
`v_next=v_pred+(h/2)*a_next`. Initial acceleration is obtained from the initial
mass-system solve. Dirichlet constraints are applied to each solve. Neumann
conditions enter the weak form naturally. Boundary `gamma*u_t` terms, when
present, contribute to `M1` in the same formula; these two examples have no
boundary damping.

This is second-order time integration. An undamped, unforced linear system with
constant symmetric matrices conserves its discrete quadratic energy under
this update, up to solve and roundoff errors. That checks the time stepping;
it does not remove P1 spatial dispersion. Accurate phase over a long run also
requires adequate spatial and temporal resolution.

## Execution profiles

[The recorded execution profile](long-waves-timings.json) contains wall-clock
seconds, separate from the 100 seconds of simulated time. It records mesh
creation, each case's diagnostic setup, integration, assembly, boundary
constraints, rank checks, factorization/preconditioner setup, solve/iterations,
residual checks, other evolution work, diagnostics, data export, and HTML
packaging. Parent totals include their subphases and must not be added to them.
These are measurements of a single run, not a controlled hardware benchmark.

`SolverReport::timings` exposes the solver breakdown.
`Evolution::timings` aggregates every assembly and solve, including the initial
acceleration solve, and records call counts. `reports` still contains only
per-time-step solve reports. Durations use `std::chrono::steady_clock`.
Prepared assembly reuses geometry, evaluators and constant contributions.
These constant-matrix wave runs need two solver preparations: the initial mass
solve and the Newmark effective system. The other 1,999 solves reuse the latter
factorization. The profile records preparation/reuse counts explicitly.
Diagnostics run inside the full-step observer, so their time is included in the
integration total. `integration_excluding_observer_seconds` subtracts the timed
diagnostics for comparison with the earlier postprocessing-based profile saved
in `docs/performance/baseline-long-waves-695d785.json`.

## Reproduce

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPDE2D_BUILD_FEM=ON \
  -DSymEngine_DIR=/path/to/lib/cmake/symengine
cmake --build build --target fem_long_waves -j2
./build/fem_long_waves build/long-waves.json
node tools/render_solutions.mjs build/long-waves.json docs/fem/long-waves.html
cp build/long-waves.timings.json docs/fem/long-waves-timings.json
```

Optional numerical arguments are grid subdivisions, time step and thread count:
`./build/fem_long_waves build/long-waves.json 20 0.05 1`.
To request two or four workers, configure `-DPDE2D_ENABLE_OPENMP=ON` and replace
the final argument. Serial is the default; parallel work is optional and can
cost more on small meshes. Each profile records the selected policy.
The full gallery keeps all 401 saved frames per example at full output
precision. `--compact` on the renderer creates the lighter inline version,
which samples at most 101 frames per example and quantizes plotted values to
16 bits. Error norms and conservation diagnostics retain full precision.

## Native timing reports and benchmarking

The [post-review benchmark](../performance/post-review-waves-2026-11-07.md)
records matched serial/OpenMP runs with all five review fixes applied, including
a separately compiled no-OpenMP binary and an interactive phase comparison.

Profiling is part of every solve: returned solver/evolution reports contain
phase timings and work counts by default. The wave executable prints the
report and saves a `.timings.json` sidecar on an ordinary run. There is no
separate profiling pass or timer option to enable. Durations measure wall
time, not the simulated interval.

Comparing execution policies requires solving the problem more than once.
The executable provides this as a single option in an OpenMP-enabled build:

```sh
./build/fem_long_waves --benchmark
# Optional: output directory, repetitions, subdivisions, time step.
./build/fem_long_waves --benchmark build/wave-benchmark 3 20 0.05
```

This runs both 100-second wave problems with serial execution and with 2 and
4 requested OpenMP workers. It uses the same executable, mesh, time integrator,
SparseLU solver, and default checked validation for every policy. It checks
every retained displacement, velocity, and conservation/error diagnostic
against the serial reference. The first run is serial; subsequent rounds rotate
the order. No warmup is discarded. OpenMP availability and requested worker
limits are checked before any simulations run; runtime settings may still
limit the actual teams used.

The default is three repetitions. `benchmark.json` records every run, median
and min/max total times, per-case integration/diagnostic times, settings and
compiler. Each raw run and its phase profile are retained alongside it.
Totals include diagnostic callbacks and export; phase totals are nested and
must not be summed twice. No wall-time threshold is a test assertion. Avoid
concurrent builds or other heavy work when collecting comparable measurements.

The serial policy in this comparison bypasses OpenMP kernels within the same
OpenMP-enabled binary. To measure compile-time OpenMP overhead separately,
also configure `-DPDE2D_ENABLE_OPENMP=OFF` in another build directory and run
its normal executable with the same `20 0.05 1` arguments. Building and running
the historical revision remains a separate advanced workflow supported by
`benchmarks/benchmark_long_waves.mjs`; it is unnecessary for routine profiling or
the serial/OpenMP comparison above.

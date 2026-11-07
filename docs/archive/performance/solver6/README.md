# Issue #6: reusable solver and evolution

`PreparedLinearSolver(matrix, options)` owns a compressed matrix snapshot and
options. `solve(rhs)` performs RHS validation, iteration/factor application and
a stable-norm true-residual check. `preparation_timings()` contains the single
rank check and matrix-dependent preparation; these are not repeated in each
solve report. `solve_linear` combines preparation and solve metrics for backward
compatibility. Preparation errors retain the existing nonconverged-report
behavior; invalid input still throws. Prepared solves are sequential.

Evolution keeps one prepared solver, compares exact constrained coefficients
and all solver options, and replaces it on a miss. Changing loads or prescribed
values does not itself invalidate factors. Changed operator coefficients or
constraint nodes do invalidate whenever the resulting matrix differs. Returning
to a previously discarded matrix prepares it again; this intentionally bounds
cache memory. Sparse constraint reconstruction and exact matrix comparison
remain per-step costs. No approximate matrix equality is used.

Full integration steps use the nominal step, with timestamps generated from the
origin; final remainders outside the rounding allowance use their actual shortened
duration. This avoids a distinct effective matrix for each timestamp ULP without
quantizing time-dependent coefficient evaluations.

`TimeOptions` appends `frame_stride`, `store_reports`, and `observer`. Stride must
be positive; initial/final frames are always retained. The observer sees initial
and every accepted step synchronously, including frames not retained. Its frame
reference is valid only during the call. Exceptions propagate immediately.
Reports still correspond only to steps. Timings include initial acceleration.
With reports disabled and stride exceeding step count, retained trajectory
storage is two frames. Initial evaluators are prepared serially, then numerical
nodal evaluations use `assembly.execution`; workers copy no symbolic objects.

Counters: `time_steps` counts accepted steps, `linear_solves` and
`solver.rhs_solves` include wave initialization; `solver.preparations`,
`rank_checks`, and `factorizations` count actual preparation attempts.
`factorizations` includes the combined SPD/preconditioner preparation phase for
iterative solvers. `solver_reuses` counts cache hits. Evolution adds preparation
and per-RHS metrics once; its reports contain only per-RHS metrics.

## Measurement

Captured baseline binary `/tmp/solver6-baseline` and output before editing;
post-change binary `/tmp/solver6-prepared`. Source is `benchmark.cpp`. GCC 15.2,
Release `-O3 -DNDEBUG -std=gnu++23`, Eigen system headers, installed SymEngine
at `/tmp/pde2d-fem-build/_deps/symengine-build`; AMD Ryzen 9 6900HX, WSL2,
serial execution. Mesh: 121 vertices / 200 triangles. LU, 200 nominal steps,
`dt=.01`, end 2, default tolerances and full storage. Raw single samples are
`baseline.txt` and `prepared.txt`. Concurrent machine activity was uncontrolled;
these are observations, not a speedup guarantee.

Baseline BE: total .303984 s, solver .0517044 s, rank .0257755 s, factor .022318 s.
Prepared BE: total .248660 s, solver .00204048 s, rank .000128743 s,
factor .000133641 s, 1 preparation, 199 reuses, 200 solves.
Baseline Newmark: total .312391 s, solver .0480132 s, rank .0254498 s,
factor .0197056 s. Prepared Newmark: total .299168 s, solver .00246492 s,
rank .000276310 s, factor .000213451 s, 2 preparations, 199 reuses, 201 solves.
Final vector norms agree at printed precision: BE 4.98327e-16, Newmark 3.67714.
Assembly remains the compatibility adapter and dominates these measurements;
no assembly optimization is claimed here.

Build with the contract's installed SymEngine arguments and at most two jobs.
The new `fem_reusable_evolution` target is discovered by the existing test glob.
It checks cached versus freshly assembled/prepared recurrences for LU/CG/GMRES,
all time methods, spatial/time-variable coefficients, nonlinear source and moving
Dirichlet plus dynamic boundary data, partial steps, call counts, snapshot and
option invalidation, storage and synchronous observer semantics. Existing tests
cover analytical time accuracy, large-origin times, tiny residuals, singularity,
CG restrictions and nonconvergence. OpenMP builds additionally execute parallel
initial evaluation equivalence/error propagation and observer caller-thread checks.

Starting implementation snapshot: `15bc00d5fe957b6fbc651dfa4841fb034ea1437d`.
Reproduce the prepared benchmark from the repository root:

```sh
c++ -O3 -DNDEBUG -std=gnu++23 -DSOLVER6_PREPARED -I. -Icpp \
  -isystem /usr/include/eigen3 \
  -isystem /tmp/pde2d-fem-build/_deps/symengine-src \
  -isystem /tmp/pde2d-fem-build/_deps/symengine-build \
  -isystem /tmp/pde2d-fem-build/_deps/symengine-src/symengine/utilities/cereal/include \
  benchmarks/benchmark_evolution.cpp \
  /tmp/pde2d-fem-build/_deps/symengine-build/symengine/libsymengine.a \
  -o /tmp/solver6-prepared
/tmp/solver6-prepared
```

Verification completed in this worktree: Release serial full CTest, **7/7
passed**; Release OpenMP (`PDE2D_ENABLE_OPENMP=ON`, `OMP_NUM_THREADS=2`) focused
`fem_reusable_evolution`, `fem_solver`, `fem_transient`, **3/3 passed**. Both use
the installed SymEngine configuration and builds limited to two compile jobs.
No sanitizer run or independent-library comparison was added for this scoped
solver change; the existing analytical convergence and transient tests ran.

A final-code rerun is saved separately as `prepared-final.txt` (binary
`/tmp/solver6-prepared-final`): BE total .227532 s / solver .00152493 s;
Newmark total .248667 s / solver .00182440 s. Preparation/reuse/solve counts and
printed final norms are unchanged. Differences between the two prepared samples
reinforce that these single timings should not be treated as controlled speedups.
The focused serial solver/reuse tests were rebuilt and passed again after the
last timing-accounting adjustment.

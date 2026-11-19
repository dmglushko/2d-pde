# Library requests from the app

Things the app (`app/`) would use but the library does not offer. Since
2026-11-19 the agent working on the app makes these library changes
itself (root AGENTS.md); each one done is recorded in
[decisions.md](decisions.md) (marked "for the app, by an agent") and
noted here as **Done**. Newest at the bottom.

## 1. Parallel assembly and parallel linear solves

**Asked by the owner** (2026-11-15): a sequential/parallel setting in the
app, for assembly and the solvers.

- **Assembly.** `Assembler` runs in one thread ("This slice runs in one
  thread", `assembler.hpp`). The element blocks are already pure and
  processed colour by colour (`element_blocks.hpp`, `assembler.cpp`), so
  elements of one colour could run in parallel with a result bit for bit
  equal to the sequential one. Possible shape: an `AssemblerOptions` with
  a thread count (1 = sequential), validated in the constructor.
- **Linear solvers.** Eigen's iterative solvers (`ConjugateGradient`,
  `BiCGSTAB`) use several threads for the sparse matrix-vector product
  when Eigen is built with OpenMP; the direct solvers (`SparseLU`,
  `SimplicialLDLT`) stay single-threaded. Possible shape: a thread count
  in the iterative solvers' options, or one global setting.
- **Compiled expressions** may already be called from several threads
  (`compiled_expression.hpp`), so evaluating coefficients in parallel
  needs no change there.
- **In the app:** the Settings dialog already shows "Sequential /
  Parallel" for assembly and for linear solves, disabled with a note
  that points here (2026-11-15). Once the library has the option, the
  setting becomes a catalog option like any other.
- **Done** (2026-11-18): the library has `AssemblerOptions::threads`, the
  iterative solvers' `threads` and the parallel direct solvers
  (`SupernodalCholesky`, `MultifrontalLu`). The app offers each as a
  case option (the solvers' own, and `solve.assembler.threads`); the
  disabled group is gone.

## 2. Progress and cancellation of long runs

`Integrator::integrate` and `StaticSolver::solve` give no progress and
cannot be stopped.

- **What the app does now** (2026-11-15): it integrates in chunks that
  end on snapshot times, each continuing from the previous chunk's final
  state (`integrate(..., &previous.final)`), in a background thread that
  holds the session mutex only during a chunk. The page shows the
  snapshots while the run goes on, a progress bar and Stop (docs/app.md,
  "Runs in chunks"). This needs no library change.
- **What is still missing:** progress and cancellation inside a chunk. A
  run with few snapshots and many steps per interval still answers only
  at each snapshot, and Stop waits for the chunk. (Newmark's chunks
  continue from the acceleration the `Solution` now carries, 2026-11-18,
  so they equal one call bit for bit.) A static solve has no progress at
  all.
- Possible shape: an optional callback in `IntegrationOutput` called after
  each step with the step record, returning whether to continue (or a
  cancellation token checked between steps).
- **The adaptive loop** (2026-11-17): the app's adaptive solve is one
  `AdaptiveStaticSolver::solve` call, so the page shows nothing until
  every cycle is solved and cannot stop it (the app must not run the
  loop itself: that would reimplement the library's stopping rules). A
  callback after each cycle (its solution, estimate and mesh, returning
  whether to go on) would give progress, Stop and the cycles as they
  come. The same holds for the adaptive loop in time (2026-11-18): the
  app's `adaptive_time` run is one `AdaptiveIntegrator::integrate` call
  (chunking it like a time run would restart its schedule, its relative
  tolerance and its budgets at every chunk); a callback after each step
  or each piece would give the time bar its snapshots as they come and
  Stop. Its result also keeps every adaptation point's estimate with all
  indicators (one array per triangle per adaptation time, also when the
  tolerance was met); an option to keep only the totals would bound a
  long run's memory (2026-11-19, review).
- **What chunks cost** (2026-11-16): each `integrate` call prepares the
  linear solver and assembles at its start again, so the app sizes chunks
  by their measured cost to keep that fixed cost a small share
  (docs/app.md, "Runs"). With a factorization of seconds, the cap on a
  chunk's length still leaves it up to half. A callback (above) would let
  one call run the whole integration with progress and Stop; failing
  that, an `integrate` that takes the previous call's prepared solver
  (when the step matrix is unchanged) would make chunks free.

## 3. Symbolic work from more than one thread

SymEngine's reference counts are not thread-safe in this build, so the
app serializes every request: a problem check (parsing expressions) waits
for a running solve (since runs go in chunks, for one chunk at most). Either a thread-safe SymEngine build, or a guarantee
that a given operation (for example integrating an already-built
`MeshedProblem`) never builds or drops `Expression`s, would let the app
check the problem while a solve runs. (Since 2026-11-16 progress, Stop, values,
probes and status no longer wait: they read only the runs' bookkeeping
under a short lock of their own, docs/app.md "Locks"; a problem check, a
mesh or a field still waits for the chunk.)

## 4. Option errors that name the option

A component's constructor reports a bad option only in its message text
(`RuppertOptions::min_angle_degrees must be in (0, 34]`). The app finds
the option name in the text to mark the right field. An exception type
that carries the option's name (or a fixed message prefix) would make
this exact.

## 5. Region edits on a `Mesh`

`CurveGeometry` has rename, merge and split of boundary regions; a `Mesh`
has none. The app renames, merges and splits (at a boundary node:
`split_at_node`) regions of an imported mesh itself (its region names
and boundary edge tags). Library functions for this (rename, merge and
split at a node on a `Mesh`, keeping chains and links valid) would keep
that logic in one place.

## 6. Problems that are not well posed

A problem with only natural conditions and no reaction term (pure
Neumann) has no unique solution; the static solve then returns a huge,
meaningless u with a large residual and no error. Well-posedness checks
are deferred in decisions.md. Until they exist, the app can only warn on
the symptoms (a residual far above the solver's tolerance, no Dirichlet
node).

## 7. A version the program can read

The app writes the library's version into every export's provenance. It
takes the CMake project version at compile time (`PDE2D_VERSION`), which
says nothing about the commit or local changes. A `pde2d::version()`
(and, if wanted, a build id set at configure time) would make the
provenance exact and the same for every program built on the library. (Since 2026-11-16 the app records the git
commit and a dirty flag at build time itself, `app/cmake/build_info.cmake`;
a library function would still serve every program.)

## 8. Point location and evaluation on a `Mesh`

Probes need the triangle that holds a point and the P1 value there. The
app has its own grid index and barycentric coordinates in double
(`app/src/field_eval.cpp`, `PointLocator`), with a tolerance on edges. A
library function (locate with the exact predicates, and evaluate a nodal
field at a point) would keep this geometry in one place, next to the
mesh's other invariants.

## 9. Norms of P1 functions

Comparisons report max |d|, the nodal RMS and a lumped L2 norm computed
in the app from triangle areas. The consistent-mass L2 norm (dᵀ M d) and
the H1 seminorm (dᵀ K d) are the usual measures of a finite element
error; they need the mass and stiffness matrices of the mesh alone,
which the app can only get by assembling a problem made for the purpose.
Functions in `pde2d_fem` that give these norms (or a pure mass and
Laplace matrix of a mesh) would make error studies standard.

## 10. Whether a linear solver iterates

The Solve result shows "Linear iterations" only when the run reports
some: a direct solver (SparseLU, SimplicialLDLT) always reports 0, and
nothing in `LinearSolverRequirements` or the solver's declarations says
whether it iterates, so the page cannot tell "a direct solve" from "an
iterative solve that needed none". A declared trait (for example
`iterative` next to the symmetry requirement) would let the catalog carry
it, as it carries `symmetric_matrix`, and the page show the figure for
exactly the solvers it applies to.

## 11. Splitting a region at a boundary point

The app's split tool turns a point the user clicked into
`CurveGeometry::split_region`'s fraction (`app/src/locate.cpp`): it finds
the nearest point of the exact curves and repeats the library's rules
for the region's run (where it starts in the normalized loop, the order
in which lengths are summed) so that a corner gives exactly the
library's junction. A library function that splits a region at a point
of a curve (loop, curve, parameter), or that gives the fraction of such
a point, would keep those rules in one place; so would one that gives
the nearest boundary point (curve and parameter) of a point.

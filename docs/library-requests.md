# Library requests from the app

Things the app (`app/`) would use but the library does not offer. Each
is a library change, so each is the owner's decision (see
[decisions.md](decisions.md)); nothing here is decided or started. The
app works without them. Newest at the bottom; when one is decided, move
it to decisions.md and note it here.

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
  at each snapshot, and Stop waits for the chunk. Newmark's chunks also
  recompute the acceleration at each start (the `Solution` carries no
  acceleration), so they match one call only up to rounding. A static
  solve has no progress at all.
- Possible shape: an optional callback in `IntegrationOutput` called after
  each step with the step record, returning whether to continue (or a
  cancellation token checked between steps).

## 3. Symbolic work from more than one thread

SymEngine's reference counts are not thread-safe in this build, so the
app serializes every request: a problem check (parsing expressions) waits
for a running solve (since runs go in chunks, for one chunk at most). Either a thread-safe SymEngine build, or a guarantee
that a given operation (for example integrating an already-built
`MeshedProblem`) never builds or drops `Expression`s, would let the app
check the problem while a solve runs.

## 4. Option errors that name the option

A component's constructor reports a bad option only in its message text
(`RuppertOptions::min_angle_degrees must be in (0, 34]`). The app finds
the option name in the text to mark the right field. An exception type
that carries the option's name (or a fixed message prefix) would make
this exact.

## 5. Region edits on a `Mesh`

`CurveGeometry` has rename, merge and split of boundary regions; a `Mesh`
has none. The app renames and merges regions of an imported mesh itself
(its region names and boundary edge tags). Library functions for this
(rename and merge on a `Mesh`, keeping chains and links valid) would keep
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
provenance exact and the same for every program built on the library.

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

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
- **In the app:** a Settings section would show the options from the
  catalog like any other component option; no app design is needed beyond
  a catalog entry.

## 2. Progress and cancellation of long runs

`Integrator::integrate` and `StaticSolver::solve` give no progress and
cannot be stopped. The app runs every request under one mutex (see 3),
so a long time integration blocks the whole app until it ends, and the
page can only show elapsed time.

- Possible shape: an optional callback in `IntegrationOutput` called after
  each step with the step record, returning whether to continue (or a
  cancellation token checked between steps).
- **In the app:** a progress bar (t / t1, steps) and a Stop button.

## 3. Symbolic work from more than one thread

SymEngine's reference counts are not thread-safe in this build, so the
app serializes every request: a problem check (parsing expressions) waits
for a running solve. Either a thread-safe SymEngine build, or a guarantee
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

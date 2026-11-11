# Vision

Where this project is heading. This page sets the direction and does not
prescribe designs: how each goal is reached is decided when the work is done.
The current state of the code is described in [architecture.md](architecture.md)
and the decisions behind it in [decisions.md](decisions.md).

## Scope

One **linear scalar** partial differential equation on a two-dimensional
domain, stationary or time dependent. Everything below stays inside that
scope.

Not in scope for now: systems of equations, nonlinear equations or boundary
conditions, stochastic equations. They may come later, but only once the
linear scalar case is well understood. Do not generalize the code toward them
in advance.

## Goals

### 1. General problem input

A user describes the whole problem, and the program accepts it without code
changes:

- **Geometry**: general two-dimensional domains, curved boundaries and named
  boundary regions.
- **The equation**: a general linear second-order operator with variable,
  possibly time-dependent coefficients.
- **Boundary conditions**: Dirichlet, Neumann, Robin, dynamic conditions and
  general oblique-derivative conditions.
- **Initial conditions**: general initial values, and initial velocities for
  second-order problems in time.
- **Several solvers**: a choice of linear solvers and time integration schemes.

Input that does not describe a well-posed problem the program can solve should
be rejected with an explanation, not solved anyway.

### 2. Problem-aware preprocessing

Before solving, the program uses the equation, boundary conditions, initial
conditions and geometry to make decisions a user would otherwise make by hand:

- generate a mesh tailored to the problem, which should do better than a
  generic mesh of similar size;
- choose the solver and time integration settings that suit the problem.

This is one of the two central and most interesting parts of the project.

### 3. Adaptive postprocessing

After solving, the program estimates the error and improves the result, then
repeats:

- change the discretization: mesh refinement or coarsening, moving mesh nodes
  (r-refinement), and possibly enriching the function space;
- revise the solver choice using what the solve revealed.

This is the other central part. Some of these ideas may turn out not to pay
off. Finding out which ones do is part of the work.

## Principles

- **Decisions are explained.** Every automatic choice records what it was
  based on and whether that basis was proven or only estimated, such as
  checked at sample points.
- **"Better" is measured.** Automatic choices are judged against simple
  defaults on reference problems with known solutions, in accuracy for the
  work spent. A choice that cannot be shown to help is not kept.
- **The user stays in control.** Every automatic decision can be inspected and
  overridden.
- **Correctness first.** Results stay deterministic, and an automatic choice
  never silently changes the problem being solved.

## For agents

- This page tells you which doors to keep open. It does not ask you to
  implement anything: build these features only when a task asks for them.
- When a change would make one of these goals harder, for example by fixing a
  value that preprocessing or adaptivity would need to choose, prefer a
  flexible form or raise the conflict instead of deciding it silently.
- Keep this page short and about direction. Design decisions belong to the
  owner and are recorded in [decisions.md](decisions.md); the current design
  is described in [architecture.md](architecture.md). Update this page when
  the direction changes.

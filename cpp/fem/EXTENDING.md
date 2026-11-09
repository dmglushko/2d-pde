# Extending the FEM module

Each extension point below lists the files to touch and the tests to add.
Keep the invariants in [README.md](README.md): symbolic work only during
preparation, deterministic serial scatter, explicit exceptions.

## A new time integration scheme

A user scheme needs no library change. Implement `TimeIntegrator` in your
own code and pass a factory: `options.integrator = [] { return
std::make_unique<MyScheme>(); };`. `evolve()` calls it once per evolution and
ignores `options.method`; reports and the run record's strategies use
`name()` (`method()` defaults to `TimeMethod::Custom`). Override
`descriptor()` so the record can say what ran (see "A strategy
descriptor"). See
`tests/fem/custom_integrator.cpp`.

1. Implement `TimeIntegrator`:
   - `order()`: 1 if the scheme needs only `u` at the start, 2 if it also
     needs `u_t` (then `evolve` requires `InitialConditions::velocity`);
   - `check()`: reject systems the scheme cannot integrate (e.g. nonzero
     second mass for a first-order scheme);
   - `start()`: optional start-up work (Newmark solves for the acceleration;
     use `services.start_dirichlet(DirichletDerivative::FirstDerivative)`
     for Dirichlet time derivatives; an integer order 0..2 still works);
   - `advance()`: form the linear system(s) for one step and call
     `services.solve(matrix, rhs, constraints, true)`. Do not solve directly:
     the driver supplies lifting, factorization reuse and accounting. A
     multistage scheme gets the system at a stage time from
     `services.assemble(time)`: same cached `PreparedAssembly`, counted in
     `assembly_calls`. Do not build your own assembly.
   - Return `u` and `v` with one finite entry per mesh node. `evolve()`
     checks this after `start()` and `advance()` and throws
     `std::runtime_error` otherwise.
   - Transactional steps: `evolve()` calls `trial()` (default: `advance()`)
     and then `accept()` or `reject()`. A scheme with internal state keeps
     the new state provisional in `trial()` and commits it in `accept()`
     (see `NewmarkIntegrator`: the trial acceleration), keeps `advance()` as
     `trial()` + `accept()` for drivers that never reject, and returns true
     from `transactional()`. A stateless scheme only needs
     `transactional() == true`. With the default `false`, `evolve()` throws
     when a controller rejects a trial.
2. Only for a built-in scheme (in `time_integration.hpp`): add the enum value
   to `TimeMethod`, a case to `make_time_integrator` and a name in
   `time_method_name`.
3. Tests (`tests/fem/`): a manufactured-solution convergence-rate test for
   the scheme's order, and an equivalence test that drives the integrator
   through an uncached reference driver (see the reusable-evolution test).

## A step controller (adaptive time stepping)

The step sequence is a strategy (`step_control.hpp`). Implement
`StepController` and set `options.step_controller = [] { return
std::make_unique<MyController>(...); };`; `evolve()` then ignores
`options.step`. No change to `evolution.hpp` is needed.

- `next()` returns `TimeStep{index, time, h}` or nothing at the end. The
  driver checks: consecutive indices from 1, strictly increasing finite
  stamps not after `TimeOptions::end`, finite `h > 0`, and the last stamp is
  exactly `end` (otherwise `std::invalid_argument`).
- `h` must equal `time` minus the previous stamp up to rounding:
  `step_size_consistent(step, previous, start, end)` is the documented bound
  (32 eps times the larger of the stamps and the interval length, plus h).
- `judge(StepOutcome)` sees every trial with the state before it and the
  trial state, which is where a temporal error estimator would look. Return
  `StepVerdict::Reject` to discard the trial; the next `next()` call must
  propose the retry (same index; typically a smaller h from the same
  previous stamp). Rejected trials publish no frames or observer calls and
  their step reports are dropped, but their work is counted
  (`EvolutionTimings::rejected_steps`, assemblies, solves). The driver caps
  retries: more than `TimeOptions::max_consecutive_rejections` (default
  `default_max_consecutive_rejections` = 100; 0 forbids rejections)
  rejected trials in a row throw `std::runtime_error`. The count restarts
  at every accepted step.
- `accepted(StepOutcome)` sees each accepted step with the states before
  and after it.
- `name()` appears in the run record's decisions ("time steps");
  `descriptor()` makes the controller described rather than "not
  replayable" in the record.
- Tests: a non-uniform sequence against a hand-computed recurrence on an
  exact semidiscrete solution, and the default path bit-identical (see
  `tests/fem/step_control.cpp`); for rejection, retried and halved trials
  against plain runs (`tests/fem/step_rejection.cpp`). A changing `h` changes the step matrix, so
  assert factorization counts as relations on the runs of equal `h`.

## An analysis property

`analysis.hpp` reports properties of a `Problem` as `Property<T>` (value,
`Evidence`, reason).

1. Add the field to `ProblemProperties` (or `SystemProperties` for a
   judgement on an assembled matrix) in `analysis_types.hpp` and fill it in
   `interior_properties` (`detail/analysis_interior.hpp`), the region
   accumulator (`detail/analysis_regions.hpp`) or
   `analyze(const AssembledSystem &)` (`analysis.hpp`). Boundary edges come
   from `bind_boundary`, the same binding assembly uses.
2. Evidence: `Proven` only for exact symbolic facts (constants, symbolic
   zeros, classification), `Sampled` for checks at quadrature points, edge
   samples, sample times or one assembled matrix, `Unknown` otherwise.
   Always say why in `reason`.
3. If the solve already makes the same judgement, move it into one public
   function and call it from both places (see `weak_time_dependence`,
   `divisor_keeps_strict_sign`, `time_order`, `numerically_symmetric`).
   Never copy the check.
4. Print it in `write_report(std::ostream &, const ProblemProperties &)`.
5. Tests (`tests/fem/analysis.cpp`): a problem whose property is known by
   construction, and the consequence the solver draws from it (singular
   solve, CG rejection, assembly exception, caching counters). A region
   property can also be tested on synthetic `ClassifiedEdge`s with
   `analyze_boundary_region` (`tests/fem/p1_kernels.cpp`).

## A decision record

`Decision` (`decision.hpp`) records a typed subject (`DecisionSubject`),
choice, basis, evidence and source. User choices use
`user_decision(DecisionSubject::X, choice, "Options::field")`.

1. A new setting gets a `DecisionSubject` value (append it, add its name in
   `decision_subject_name` and the `decision_subjects` list; names are
   stable, run record JSON uses them).
2. Teach `validate_decisions` (`plan.hpp`, or `mesh_plan.hpp` for a mesh
   setting) how to compute the choice text from the options, so a stale
   decision throws. Format numbers with `std::format("{}")` (shortest
   round trip) so every change of the option changes the text.
3. Add a `SolvePlan` setter that changes the option and calls
   `set_decision(decisions, decision_for(subject, choice, "Options::field",
   why))`.
4. If plan_from_user should record it, add it in `solver_decisions()` or
   `time_decisions()` (or `mesh_decisions()`); existing decision lists are
   asserted by tests, so prefer the setter for optional settings.

The plan's decisions are copied into every stationary `SolverReport` and
into `Evolution::record.plan.decisions`; `write_report` prints them and
`write_json` serializes them. A choice made by a strategy object
(integrator or step controller factory) stays empty in the plan and is
named by `evolve()` from `name()`; a nonempty choice must equal that name.

## A strategy descriptor (reproducible custom strategies)

Factories (`TimeOptions::integrator`, `step_controller`,
`SolverOptions::backend`, `RefinementOptions::size_field`) cannot be
replayed from a run record. Give each custom strategy a
`pde2d::StrategyDescriptor` (`cpp/provenance.hpp`): a stable name and its
parameters as text (`std::format("{}", value)` for numbers), enough for a
reader to rebuild it.

- Integrator / step controller / error estimator: override
  `descriptor()` on the object (`TimeIntegrator`, `StepController`,
  `ErrorEstimator`); built-in schemes and `UniformStepController` already
  return theirs.
- Linear solver backend: override `LinearSolverBackendFactory::descriptor()`
  or pass one to `make_linear_solver_backend(name, creator, descriptor)`.
- Size field: set `RefinementOptions::size_field_descriptor` next to the
  field.

`RunRecord::strategies` then says `described` with the descriptor; without
one it says `not replayable` and `record.replayable()` is false. Tests: the
record's strategy entry and its JSON (`tests/fem/run_records.cpp`).

## An automatic chooser (planner)

Not implemented; the seam is `SolvePlan` (`plan.hpp`), plus `MeshPlan`
(`mesh_plan.hpp`) for meshing. A chooser is a function from the problem,
mesh (or geometry) and analysis (`analyze`) to a plan:

1. Start from `plan_from_user(...)` defaults (or `mesh_plan_from_user`).
2. Change each setting it chooses with the setter and a `Rationale`:
   `plan.set_linear_solver(LinearSolver::ConjugateGradient,
   {"operator proven symmetric", properties.symmetric.evidence,
   DecisionSource::Automatic})`. Never edit an option without its
   decision: solves throw "Stale decision" at entry.
3. Store what it was based on: `plan.analysis = AnalysisRecord{options,
   properties}` and `plan.mesh = mesh_plan`.

The user overrides it by calling setters on the returned plan before
`solve_stationary(mesh, problem, plan)`, `PreparedStationary` or
`evolve(mesh, problem, initial, plan, output)` (pass the `MeshResult` to
check the mesh plan against it). The run record (`record.hpp`) then
carries the plan, its analysis and an evidence summary. Judge a chooser
against `plan_from_user` defaults on reference problems (docs/vision.md,
"Better is measured"), comparing records with `diff_records`.

## An error estimator

`estimation.hpp` defines `ErrorEstimator`: `name()` and
`estimate(mesh, problem, solution, time)` returning `ErrorEstimate`
(`indicators`, one nonnegative value per element in element order, and
`global`, conventionally `sqrt(sum eta_T^2)`). Pass it to
`estimate_error(..., estimator)`, which checks the result's size and
finiteness ("returned invalid indicators"). No library change is needed.

- State what norm the estimate measures and what it assumes, in the class
  comment (the built-in ZZ estimator: H1 seminorm, smooth solutions,
  ignores problem data).
- Parallel loops write per-element slots and accumulate serially in element
  order (see `GradientRecoveryEstimator`), so serial and OpenMP results stay
  bit-identical. Prepare symbolic data (for a residual estimator: the source
  and coefficients as `Evaluator`s) before the loop.
- A residual estimator needs element residuals and flux jumps across
  interior edges; reuse `weak_form_coefficients`, `p1::triangle_geometry`
  and the quadrature rules rather than re-deriving them.
- Tests (`tests/fem/estimation.cpp`): effectivity against the exact error
  of a manufactured solution over several refinements, concentration of the
  indicators where the exact element errors are largest, and serial/OpenMP
  equality.

Marking strategies are plain functions from indicators to ascending element
indices (`mark_bulk`); the result feeds
`discretization::refine_mesh(result, marked, options)`.

## A linear solver backend or preconditioner

`linear_solver.hpp` splits a solve into a shared driver
(`PreparedLinearSolver`: option and matrix validation, SparseQR rank check,
tolerances, true-residual check, timings, `matches()` reuse) and a
`LinearSolverBackend` that owns only `prepare(matrix, options)` (return `""`
or a failure message) and `solve(rhs, relative_tolerance)` (values,
iterations, optional failure).

- **Initial guess:** an iterative backend overrides
  `solve_with_guess(rhs, tolerance, guess)` and sets
  `BackendSolution::used_guess`; the default ignores the guess (right for
  direct factorizations) and reports `InitialGuess::Ignored`.
- **User backend, no library change:** implement `LinearSolverBackend` and
  set `SolverOptions::backend = make_linear_solver_backend("Name", creator)`
  (or your own `LinearSolverBackendFactory`). It replaces `method`, receives
  every option including `preconditioner`, and works with `solve_linear`,
  `solve_stationary` and `evolve`. Factories compare by identity, so keep one
  factory object to reuse factorizations. See `tests/fem/solver_backends.cpp`.
- **Built-in backend:** a class in `solver_detail` with `name`,
  `supports(Preconditioner)`, `prepare` and `solve`, a `LinearSolver` value
  and one case in `solver_detail::with_builtin` (the only dispatch).
- **Preconditioner:** add a `Preconditioner` value (and its name and the
  validation switch), construct it in the iterative backends that support
  it (`EigenIterativeBackend<Solver<SparseMatrix, P>>`) and extend their
  `supports()`. `Default` must keep its current meaning so results do not
  change.
- A new `SolverOptions` field needs nothing else for reuse: `operator==` is
  defaulted, so `matches()` compares every field.

## A new interior term

The weak form is linear in the coefficients of `weak::Term`.

1. Add the term to `Equation` (`problem.hpp`) and document its sign.
2. Add a `weak::Term` entry and its expression in `weak_form_coefficients`
   (integrate by parts there if needed, and record the derivation in the
   header comment).
3. Use it in `p1::accumulate_element_point` and extend `p1::active_blocks`
   if it writes a new matrix. Assembly caches it automatically when it is
   time independent.
4. Tests: an exact-solution test where the term matters, and a
   prepared-versus-one-shot assembly comparison with a time-dependent
   coefficient.

## A new boundary-condition form

`classify_edge_condition` (`boundary_classification.hpp`) reduces every
`BoundaryCondition` on one edge to an `EdgeForm` alternative
(`DirichletEdge`, `NaturalEdge` or `ObliqueEdge`). It is pure, so test it
directly with an `EdgeFrame`, without a mesh.

- A new *factory* for an existing form (e.g. a convenience constructor) only
  needs a function in `problem.hpp`.
- A genuinely new *form* is one unit, one variant entry and its tests.
  Everything the form does lives in one header
  `detail/edge_forms/<form>.hpp`; generic code (names, assembly, scatter,
  analysis) calls it only through `edge_form_detail::Unit<Form>`. The
  interface is documented in `detail/edge_forms/interface.hpp`; copy
  `natural.hpp` or `oblique.hpp` as a template:
  1. the public struct of restricted coefficients (namespace `pde2d::fem`)
     and `template <> struct Unit<MyEdge>` with `name`, `divisor_name`,
     `category` (an existing `BoundaryForm` report category, or a new value
     at the end of `boundary_form.hpp` plus its count in
     `BoundaryRegionProperties`), `divisor()`, a `Prepared` struct
     (`using Form = MyEdge`, `for_each` over its evaluators, which
     `AssemblyStatistics::evaluators` counts), `prepare()` (compile
     evaluators, cache time-independent parts), `evaluate()` (complete
     local contribution at one time, read-only), `scatter()` (serial, fixed
     order) and `judge()` (divisors to sample, time-dependent coefficients,
     Robin and tangential terms for analysis);
  2. include it in `detail/edge_forms/forms.hpp` and append the struct to
     `EdgeForm`. A `static_assert` there (the `EdgeFormUnit` concept) fails
     until the unit is complete;
  3. return it from `classify_edge_condition`, at the point where the data
     select it;
  4. tests: classification on `EdgeFrame`s, an exact or manufactured
     solution with a convergence rate, the local block against a hand
     computation (e.g. assembled minus a reference form, see
     `tests/fem/oblique_conditions.cpp`), time-dependent coefficients
     against the same problem with `t` substituted, and the analysis report
     (`tests/fem/analysis.cpp`, synthetic `ClassifiedEdge`s in
     `tests/fem/p1_kernels.cpp`).
- Divisors are checked with `EdgeSite::require_divisor_sign` in assembly and
  listed in `judge()` for analysis; both use `divisor_keeps_strict_sign` on
  `edge_divisor_samples`, so analysis and assembly agree. A form may have
  several divisors (the oblique form checks its conormal component, then its
  flux multiplier); check them in the same order in both places.
- A natural-type form (one that solves the condition for `q_n`) can reuse
  `natural_type_block` / `natural_type_evaluate` (`natural.hpp`) and add its
  own terms through the `Extra` hooks, as `oblique.hpp` does. With the
  default hooks the arithmetic is exactly that of `NaturalEdge`.

## A different element type (P2, quadrilaterals, systems)

Do not add an order switch to the P1 code. Add a sibling module: a
`p2_element.hpp`-style kernel header plus its own prepared assembler. The
pieces that are element independent and should be reused: `problem.hpp`,
`weak_form.hpp`, `boundary_classification.hpp` (edge-local symbolic
reduction), `quadrature.hpp`, `system.hpp` (produce an `AssembledSystem`),
`linear_solver.hpp`, `time_grid.hpp` and `time_integration.hpp`
(integrators only see `AssembledSystem`). `evolution.hpp` currently drives a
`PreparedAssembly`; a sibling module needs its own driver or a generalized
assembly seam there. Mesh files
are fixed at `ORDER 1` (docs/formats/pde2d-mesh.md); a higher-order mesh
needs a new format version.

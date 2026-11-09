# Extending the FEM module

Each extension point below lists the files to touch and the tests to add.
Keep the invariants in [README.md](README.md): symbolic work only during
preparation, deterministic serial scatter, explicit exceptions.

## A new time integration scheme

A user scheme needs no library change. Implement `TimeIntegrator` in your
own code and pass a factory: `options.integrator = [] { return
std::make_unique<MyScheme>(); };`. `evolve()` calls it once per evolution and
ignores `options.method`; reports and `Evolution::metadata.integrator` use
`name()` (override it; the default is "Custom", and `method()` defaults to
`TimeMethod::Custom`). See `tests/fem/custom_integrator.cpp`.

1. Implement `TimeIntegrator`:
   - `order()`: 1 if the scheme needs only `u` at the start, 2 if it also
     needs `u_t` (then `evolve` requires `InitialConditions::velocity`);
   - `check()`: reject systems the scheme cannot integrate (e.g. nonzero
     second mass for a first-order scheme);
   - `start()`: optional start-up work (Newmark solves for the acceleration;
     use `services.start_dirichlet(DirichletDerivative::FirstDerivative)`
     for Dirichlet time derivatives);
   - `advance()`: form the linear system(s) for one step and call
     `services.solve(matrix, rhs, constraints, true)`. Do not solve directly:
     the driver supplies lifting, factorization reuse and accounting. A
     multistage scheme gets the system at a stage time from
     `services.assemble(time)`: same cached `PreparedAssembly`, counted in
     `assembly_calls`. Do not build your own assembly.
   - Return `u` and `v` with one finite entry per mesh node. `evolve()`
     checks this after `start()` and `advance()` and throws
     `std::runtime_error` otherwise.
2. Only for a built-in scheme: one unit, as for boundary edge forms and
   interior algorithms. Copy `detail/time_schemes/crank_nicolson.hpp`:
   - a unit struct in `time_scheme_detail` deriving from
     `UnitBase<Self>` with `using Integrator = MyIntegrator;`,
     `static constexpr TimeMethod method`, `name` (stable: reports and
     tests use it) and `order`; `make()` comes from `UnitBase`;
   - the public integrator class deriving from
     `time_scheme_detail::BuiltinScheme<Unit[, FirstOrder]>`, which supplies
     `method()`, `name()` and `order()` from the unit (`FirstOrder` adds
     the first-order `check()`);
   - a `TimeMethod` value before `Custom`, the include in
     `detail/time_schemes/schemes.hpp` and the unit appended to `Units` in
     enum order. The `static_assert` there (`TimeSchemeUnit`, ids in order,
     `Custom` last) fails until the three agree. `time_method_name`,
     `time_method_order` and `make_time_integrator` then know the scheme;
     no switch to edit.
3. Tests (`tests/fem/`): a manufactured-solution convergence-rate test for
   the scheme's order, and an equivalence test that drives the integrator
   through an uncached reference driver (see the reusable-evolution test).

## A linear solver backend or preconditioner

The linear solver layer is split by role (`linear_solver.hpp` includes all
of it): the contracts in `linear_solver_types.hpp` (options, enums,
reports, `LinearSolverBackend`, the factory), Dirichlet lifting in
`constraints.hpp`, the shared driver in `prepared_linear_solver.hpp`
(`PreparedLinearSolver`: option and matrix validation, SparseQR rank check,
tolerances, true-residual check, timings, `matches()` reuse) and the private
Eigen backends in `detail/eigen_backends.hpp`. A backend owns only
`prepare(matrix, options)` (return `""` or a failure message) and
`solve(rhs, relative_tolerance)` (values, iterations, optional failure).

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
- **Built-in backend:** a class in `solver_detail`
  (`detail/eigen_backends.hpp`) with `name` (from
  `linear_solver_method_name`), `supports(Preconditioner)`, `prepare` and
  `solve`, a `LinearSolver` value with its name in
  `linear_solver_method_name` (`linear_solver_types.hpp`) and one case in
  `solver_detail::with_builtin` (the only dispatch).
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
   (`weak_form.hpp`;
   integrate by parts there if needed, and record the derivation in the
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
  `detail/edge_forms/<form>.hpp`; generic code (names, assembly, scatter)
  calls it only through `edge_form_detail::Unit<Form>`. The
  interface is documented in `detail/edge_forms/interface.hpp`; copy
  `natural.hpp` or `oblique.hpp` as a template:
  1. the public struct of restricted coefficients (namespace `pde2d::fem`)
     and `template <> struct Unit<MyEdge>` with `name`, `divisor_name`,
     `divisor()`, a `Prepared` struct
     (`using Form = MyEdge`, `for_each` over its evaluators, which
     `AssemblyStatistics::evaluators` counts), `prepare()` (compile
     evaluators, cache time-independent parts), `evaluate()` (complete
     local contribution at one time, read-only) and `scatter()` (serial,
     fixed order);
  2. include it in `detail/edge_forms/forms.hpp` and append the struct to
     `EdgeForm`. A `static_assert` there (the `EdgeFormUnit` concept) fails
     until the unit is complete;
  3. return it from `classify_edge_condition`, at the point where the data
     select it;
  4. tests: classification on `EdgeFrame`s, an exact or manufactured
     solution with a convergence rate, the local block against a hand
     computation (e.g. assembled minus a reference form, see
     `tests/fem/oblique_conditions.cpp`), time-dependent coefficients
     against the same problem with `t` substituted.
- Divisors are checked with `EdgeSite::require_divisor_sign` in assembly
  (`divisor_keeps_strict_sign` on `edge_divisor_samples`). A form may have
  several divisors (the oblique form checks its conormal component, then its
  flux multiplier).
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

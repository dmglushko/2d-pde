# Extending the FEM module

Each extension point below lists the files to touch and the tests to add.
Keep the invariants in [README.md](README.md): symbolic work only during
preparation, deterministic serial scatter, explicit exceptions.

## A new time integration scheme

1. Implement `TimeIntegrator` in `time_integration.hpp`:
   - `order()`: 1 if the scheme needs only `u` at the start, 2 if it also
     needs `u_t` (then `evolve` requires `InitialConditions::velocity`);
   - `check()`: reject systems the scheme cannot integrate (e.g. nonzero
     second mass for a first-order scheme);
   - `start()`: optional start-up work (Newmark solves for the acceleration;
     use `services.start_dirichlet(order)` for Dirichlet time derivatives);
   - `advance()`: form the linear system(s) for one step and call
     `services.solve(matrix, rhs, constraints, true)`. Do not solve directly:
     the driver supplies lifting, factorization reuse and accounting.
2. Add the enum value to `TimeMethod`, a case to `make_time_integrator`, and a
   name in `report.hpp`.
3. Tests (`tests/fem/`): a manufactured-solution convergence-rate test for
   the scheme's order, and an equivalence test that drives the integrator
   through an uncached reference driver (see the reusable-evolution test).

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
`BoundaryCondition` to Dirichlet or natural form on one edge. It is pure, so
test it directly with an `EdgeFrame`, without a mesh.

- A new *factory* for an existing form (e.g. a convenience constructor) only
  needs a function in `problem.hpp`.
- A genuinely new *form* (e.g. oblique derivatives that are not
  proportional to the flux) needs a new `EdgeCondition` kind, a new prepared
  edge evaluation path in `assembly.hpp` and a kernel in `p1_element.hpp`.

## A different element type (P2, quadrilaterals, systems)

Do not add an order switch to the P1 code. Add a sibling module: a
`p2_element.hpp`-style kernel header plus its own prepared assembler. The
pieces that are element independent and should be reused: `problem.hpp`,
`weak_form.hpp`, `boundary_classification.hpp` (edge-local symbolic
reduction), `quadrature.hpp`, `solver.hpp`, `time_grid.hpp` and
`time_integration.hpp` (integrators only see `AssembledSystem`). Mesh files
are fixed at `ORDER 1` (docs/formats/pde2d-mesh.md); a higher-order mesh
needs a new format version.

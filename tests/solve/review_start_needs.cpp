#include "solve_support.hpp"

#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/solve/fixed_step.hpp"
#include "pde2d/solve/implicit_euler.hpp"
#include "pde2d/solve/implicit_midpoint.hpp"
#include "pde2d/solve/integrator.hpp"

#include <memory>
#include <stdexcept>
#include <vector>

using namespace pde2d::fem;
using namespace pde2d::solve;
using pde2d::test::check;

namespace {

// A wave initially at rest with a square-root boundary input. The free
// initial values and velocities are all zero. Boundary rates at t0 are
// unnecessary: no t0 snapshot is requested, and both methods start with
// StartNeeds::nothing and evaluate lifting only at positive step times.
void starts_without_boundary_rate(std::shared_ptr<const TimeMethod> method) {
  const BoundaryCondition input{.alpha = Expression("1"),
                                .g = Expression("sqrt(t)")};
  const auto mesh = pde2d::test::unit_square(4);
  const auto problem = std::make_shared<const MeshedProblem>(
      mesh, Problem{.equation = {.a_xx = Expression("-1"),
                                 .a_yy = Expression("-1"),
                                 .m2 = Expression("1")},
                    .boundary_conditions = {{"bottom", input},
                                            {"right", input},
                                            {"top", input},
                                            {"left", input}},
                    .initial = {.u = Expression("0"), .u_t = Expression("0")}});
  const Integrator integrate(
      method, std::make_shared<FixedStep>(FixedStepOptions{.dt = 0.01}),
      std::make_shared<SparseLu>());
  const Solution initial{.mesh = mesh,
                         .time = 0.0,
                         .values = std::vector<double>(mesh->nodes.size(), 0.0),
                         .velocities =
                             std::vector<double>(mesh->nodes.size(), 0.0)};
  // Control: explicitly supplying the same free initial state succeeds.
  const auto control = integrate.integrate(problem, 0.0, 0.1, {}, &initial);
  check(control.report.metrics.at("assemblies") == 10.0,
        "control has no assembly at t0");
  // The same free initial state from Problem::initial. It used to throw
  // (d/dt (g / alpha) is not finite at t = 0): the integrator read the
  // initial velocities at every node, boundary rates included.
  const auto result = integrate.integrate(problem, 0.0, 0.1);
  check(result.final.values == control.final.values,
        "default and explicit free initial states agree");
  check(result.final.velocities == control.final.velocities,
        "default and explicit free initial velocities agree");
}

// initial_free_state is the free part of initial_values and
// initial_velocities when the boundary data is harmless, and it does not
// evaluate the boundary data at all when it is not.
void free_state_matches_full_vectors() {
  const BoundaryCondition smooth{.alpha = Expression("1"),
                                 .g = Expression("x + t")};
  const BoundaryCondition root{.alpha = Expression("1"),
                               .g = Expression("sqrt(t)")};
  const auto mesh = pde2d::test::unit_square(3);
  const auto make = [&](const BoundaryCondition &left) {
    return MeshedProblem(
        mesh,
        Problem{.equation = {.a_xx = Expression("-1"),
                             .a_yy = Expression("-1"),
                             .m2 = Expression("1")},
                .boundary_conditions = {{"bottom", smooth},
                                        {"right", {.beta_n = Expression("1")}},
                                        {"top", smooth},
                                        {"left", left}},
                .initial = {.u = Expression("x*y + t"),
                            .u_t = Expression("2*x - y")}});
  };
  const MeshedProblem p = make(smooth);
  const auto state = p.initial_free_state(0.5);
  const auto values = *p.initial_values(0.5);
  const auto velocities = *p.initial_velocities(0.5);
  check(state.has_value(), "time order 2 has an initial state");
  check(state->values.size() == p.free_nodes().size() &&
            state->velocities.size() == p.free_nodes().size(),
        "one value and one velocity per free node");
  for (std::size_t i = 0; i < p.free_nodes().size(); ++i) {
    const std::size_t node = p.free_nodes()[i];
    check(state->values[i] == values[node], "value at a free node");
    check(state->velocities[i] == velocities[node], "velocity at a free node");
  }
  // With u = sqrt(t) on the left the full velocities throw at t0 = 0 (the
  // boundary rate is infinite); the free state does not read it.
  const MeshedProblem q = make(root);
  pde2d::test::expect_throws<std::invalid_argument>(
      [&] { (void)q.initial_velocities(0.0); }, "not finite");
  check(q.initial_free_state(0.0).has_value(),
        "the free state ignores the boundary data");
}

} // namespace

int main() {
  return pde2d::test::run({
      {"Euler order 2 does not need boundary rates at t0",
       [] { starts_without_boundary_rate(std::make_shared<ImplicitEuler>()); }},
      {"initial_free_state is the free part and reads no boundary data",
       free_state_matches_full_vectors},
      {"midpoint order 2 does not need boundary rates at t0",
       [] {
         starts_without_boundary_rate(std::make_shared<ImplicitMidpoint>());
       }},
  });
}

#include "solve_support.hpp"

#include "pde2d/solve/bicgstab.hpp"
#include "pde2d/solve/fixed_step.hpp"
#include "pde2d/solve/implicit_euler.hpp"
#include "pde2d/solve/implicit_midpoint.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/newmark.hpp"
#include "pde2d/solve/static_solver.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

// Cases from the review of the first pde2d_solve commit (9cb76b6). The
// first four reproduced bugs there and now guard their fixes; the others
// check paths the first tests did not reach.

using namespace pde2d::fem;
using namespace pde2d::solve;
using pde2d::test::check;
using pde2d::test::near;
using pde2d::test::sparse;

namespace {

// A returned result meets its tolerance by an independent residual; a
// tolerance double precision cannot reach throws instead. 1e-10 must be
// reached; 1e-14 is at the edge of rounding for these matrices.
void true_residual() {
  for (const double tolerance : {1e-10, 1e-14})
    for (const std::size_t n : {32, 64, 128, 256}) {
    pde2d::test::Dense a(n, std::vector<double>(n, 0));
    for (std::size_t i = 0; i < n; ++i) {
      a[i][i] = 2;
      if (i > 0)
        a[i][i - 1] = -1;
      if (i + 1 < n)
        a[i][i + 1] = -1;
    }
    const auto matrix = sparse(a);
    LinearSolveResult result;
    try {
      result = BiCgStab({.tolerance = tolerance, .max_iterations = 10000})
                   .prepare(matrix)
                   ->solve(std::vector<double>(n, 1));
    } catch (const std::runtime_error &error) {
      std::cout << "n=" << n << ", tolerance=" << tolerance
                << ": threw: " << error.what() << '\n';
      check(tolerance < 1e-10, "a reachable tolerance is reached");
      check(std::string(error.what()).find("did not converge") !=
                std::string::npos,
            "an unreachable tolerance says it did not converge");
      continue;
    }
    // Independently compute the tridiagonal residual, in extended precision.
    long double squared = 0;
    for (std::size_t i = 0; i < n; ++i) {
      long double r = 1 - 2 * static_cast<long double>(result.values[i]);
      if (i > 0)
        r += result.values[i - 1];
      if (i + 1 < n)
        r += result.values[i + 1];
      squared += r * r;
    }
    const long double residual = std::sqrt(squared / n);
    std::cout << "n=" << n << ", reported residual=" << result.residual
              << ", independent residual=" << residual
              << ", tolerance=" << tolerance << '\n';
    check(residual <= tolerance, "a returned result meets its tolerance");
  }
}

void small_right_hand_side() {
  const auto result =
      BiCgStab().prepare(sparse({{1}}))->solve(std::vector<double>{1e-200});
  std::cout << "identity: x=" << result.values[0]
            << ", reported residual=" << result.residual << '\n';
  near(result.values[0] / 1e-200, 1, 1e-10, "identity solve, rescaled");
}

void zero_iterations() {
  const auto result =
      BiCgStab().prepare(sparse({{1}}))->solve(std::vector<double>{0});
  std::cout << "zero RHS: iterations=" << result.iterations << '\n';
  check(result.iterations == 0, "zero initial guess already solves zero RHS");
}

void sub_ulp_step() {
  // u_t = 1, u(1) = 0. Every nodal value must equal t - 1.
  const BoundaryCondition natural{.beta_n = Expression("1")};
  const auto meshed = std::make_shared<const MeshedProblem>(
      pde2d::test::unit_square(1),
      Problem{.equation = {.m1 = Expression("1"), .f = Expression("1")},
              .boundary_conditions = {{"bottom", natural},
                                      {"right", natural},
                                      {"top", natural},
                                      {"left", natural}},
              .initial = {.u = Expression("0")}});
  const double t0 = 1;
  const double t1 = std::nextafter(t0, 2.0);
  // A step of epsilon / 4 cannot change t = 1 in double precision; the
  // state must not change while the time stands still.
  const double dt = std::numeric_limits<double>::epsilon() / 4;
  const Integrator integrator(
      std::make_shared<ImplicitEuler>(),
      std::make_shared<FixedStep>(FixedStepOptions{.dt = dt}),
      std::make_shared<SparseLu>());
  pde2d::test::expect_throws<std::runtime_error>(
      [&] { (void)integrator.integrate(meshed, t0, t1); },
      "too small to change t");
}

void newmark_variable_operators() {
  // u=t^2 is exact in both space and in the Newmark kinematic formulas.
  // Thus this checks endpoint evaluation of all three operators, startup
  // acceleration, and the velocity update without a time truncation error.
  const BoundaryCondition natural{.beta_n = Expression("1")};
  const auto meshed = std::make_shared<const MeshedProblem>(
      pde2d::test::unit_square(1),
      Problem{.equation = {.c = Expression("3+t"),
                           .m1 = Expression("2+t^2"),
                           .m2 = Expression("1+t"),
                           .f = Expression("2*(1+t)+2*t*(2+t^2)+t^2*(3+t)")},
              .boundary_conditions = {{"bottom", natural},
                                      {"right", natural},
                                      {"top", natural},
                                      {"left", natural}},
              .initial = {.u = Expression("0"), .u_t = Expression("0")}});
  const auto solver = std::make_shared<pde2d::test::CountingSolver>();
  const auto result =
      Integrator(std::make_shared<Newmark>(),
                 std::make_shared<FixedStep>(FixedStepOptions{.dt = 0.3}),
                 solver)
          .integrate(meshed, 0, 1, {.snapshot_times = {0, 0.5, 0.95, 1}});
  for (const auto &snapshot : result.snapshots) {
    for (const double value : snapshot.values)
      near(value, snapshot.time * snapshot.time, 1e-12, "Newmark value");
    for (const double velocity : snapshot.velocities)
      near(velocity, 2 * snapshot.time, 1e-12, "Newmark velocity");
  }
  check(result.report.metrics.at("prepares") == 5,
        "variable operators prepared at each of five steps");
  check(solver->count() == 6, "five step matrices and the startup mass");
}

void shortened_step_prepares() {
  // u_t=1 with constant operators: a changed dt is the only reason to
  // prepare again. Snapshots require steps 0.3, 0.2, 0.3, 0.15, 0.05.
  const BoundaryCondition natural{.beta_n = Expression("1")};
  const auto meshed = std::make_shared<const MeshedProblem>(
      pde2d::test::unit_square(1),
      Problem{.equation = {.m1 = Expression("1"), .f = Expression("1")},
              .boundary_conditions = {{"bottom", natural},
                                      {"right", natural},
                                      {"top", natural},
                                      {"left", natural}},
              .initial = {.u = Expression("0")}});
  const auto solver = std::make_shared<pde2d::test::CountingSolver>();
  const auto result =
      Integrator(std::make_shared<ImplicitEuler>(),
                 std::make_shared<FixedStep>(FixedStepOptions{.dt = 0.3}),
                 solver)
          .integrate(meshed, 0, 1, {.snapshot_times = {0, 0.5, 0.95, 1}});
  for (const auto &snapshot : result.snapshots)
    for (const double value : snapshot.values)
      near(value, snapshot.time, 1e-12, "Euler value");
  check(solver->count() == 5, "each changed dt prepares again");
}

void midpoint_one_step() {
  // (1+t)u' + (1+t^2)u = 2+t^3, u(0)=2. A single step of length 1
  // evaluated at t=1/2 gives
  //   (3/2 + 5/8)u1 = 17/8 + (3/2 - 5/8)*2,
  // so u1=31/17. Averaging the endpoints gives a different answer.
  const BoundaryCondition natural{.beta_n = Expression("1")};
  const auto meshed = std::make_shared<const MeshedProblem>(
      pde2d::test::unit_square(1),
      Problem{.equation = {.c = Expression("1+t^2"),
                           .m1 = Expression("1+t"),
                           .f = Expression("2+t^3")},
              .boundary_conditions = {{"bottom", natural},
                                      {"right", natural},
                                      {"top", natural},
                                      {"left", natural}},
              .initial = {.u = Expression("2")}});
  const auto result =
      Integrator(std::make_shared<ImplicitMidpoint>(),
                 std::make_shared<FixedStep>(FixedStepOptions{.dt = 1}),
                 std::make_shared<SparseLu>())
          .integrate(meshed, 0, 1);
  for (const double value : result.final.values)
    near(value, 31.0 / 17.0, 1e-12, "midpoint one-step result");
}

} // namespace

int main() {
  std::cout << std::setprecision(17);
  return pde2d::test::run({
      {"BiCgStab checks its true residual", true_residual},
      {"BiCgStab solves a tiny nonzero right-hand side", small_right_hand_side},
      {"BiCgStab reports zero iterations for a zero RHS", zero_iterations},
      {"a step too small to change t throws", sub_ulp_step},
      {"Newmark evaluates time-dependent operators at the endpoint",
       newmark_variable_operators},
      {"shortened steps prepare the changed matrix", shortened_step_prepares},
      {"midpoint uses the midpoint rather than averaged endpoints",
       midpoint_one_step},
  });
}

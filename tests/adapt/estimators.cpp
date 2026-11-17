// The three error estimators: equivalence with the true energy error on
// smooth problems (effectivity in a band, recovery effectivity towards 1),
// the P1 rate, zero indicators for an exact P1 solution, the boundary
// residual of natural conditions, the time terms (hand computations on a
// tiny mesh), and invalid inputs.

#include "adapt/estimator_support.hpp"
#include "solve/solve_support.hpp"
#include "support/check.hpp"

#include "pde2d/adapt/kelly_estimator.hpp"
#include "pde2d/adapt/recovery_estimator.hpp"
#include "pde2d/adapt/residual_estimator.hpp"
#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/solve/sparse_lu.hpp"
#include "pde2d/solve/static_solver.hpp"

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

using namespace pde2d;
using namespace pde2d::fem;
using pde2d::adapt::ErrorEstimate;
using pde2d::adapt::KellyEstimator;
using pde2d::adapt::RecoveryEstimator;
using pde2d::adapt::RecoveryNorm;
using pde2d::adapt::ResidualEstimator;
using pde2d::test::check;
using pde2d::test::expect_throws;
using pde2d::test::near;

namespace {

Expression e(const char *text) { return Expression(text); }

std::map<std::string, BoundaryCondition> all(const BoundaryCondition &c) {
  return {{"bottom", c}, {"right", c}, {"top", c}, {"left", c}};
}

solve::Solution solve_static(const std::shared_ptr<const MeshedProblem> &p) {
  return solve::StaticSolver(std::make_shared<solve::SparseLu>())
      .solve(p)
      .solution;
}

// The nodal values of f on the problem's mesh, as a Solution at t.
solve::Solution
interpolant(const MeshedProblem &p,
            const std::function<double(double, double)> &f, double t = 0.0) {
  solve::Solution s{.mesh = p.mesh(), .time = t};
  for (const auto &n : p.mesh()->nodes)
    s.values.push_back(f(n.x, n.y));
  return s;
}

double rate(double coarse, double fine) { return std::log2(coarse / fine); }

struct Level {
  std::size_t n = 0;
  double error = 0, residual = 0, kelly = 0, recovery = 0, boundary_part = 0;
};

// Totals and the true error on unit_square(n) for n = 8, 16, 32, 64.
std::vector<Level>
levels(const Problem &problem, const test::Gradient &exact,
       const test::Weight &weight, const char *name) {
  std::vector<Level> out;
  for (std::size_t n : {8, 16, 32, 64}) {
    auto meshed =
        std::make_shared<const MeshedProblem>(test::unit_square(n), problem);
    const solve::Solution u = solve_static(meshed);
    Level l{.n = n};
    l.error = test::energy_error(*meshed->mesh(), u.values, exact, weight);
    const ErrorEstimate residual = ResidualEstimator().estimate(*meshed, u);
    l.residual = residual.total;
    l.boundary_part = residual.report.metrics.at("boundary_part");
    l.kelly = KellyEstimator().estimate(*meshed, u).total;
    l.recovery = RecoveryEstimator().estimate(*meshed, u).total;
    std::cout << "INFO " << name << " n = " << n << ": error " << l.error
              << ", effectivity residual " << l.residual / l.error
              << ", kelly " << l.kelly / l.error << ", recovery "
              << l.recovery / l.error << ", boundary part " << l.boundary_part
              << '\n';
    out.push_back(l);
  }
  return out;
}

void check_rates(const std::vector<Level> &ls) {
  for (std::size_t i = 1; i < ls.size(); ++i) {
    const Level &a = ls[i - 1], &b = ls[i];
    std::cout << "INFO rates n = " << b.n << ": error "
              << rate(a.error, b.error) << ", residual "
              << rate(a.residual, b.residual) << ", kelly "
              << rate(a.kelly, b.kelly) << ", recovery "
              << rate(a.recovery, b.recovery) << '\n';
    for (double r : {rate(a.error, b.error), rate(a.residual, b.residual),
                     rate(a.kelly, b.kelly), rate(a.recovery, b.recovery)})
      check(r > 0.9 && r < 1.15, "every total converges like h");
  }
}

// -Laplace u = 2 pi^2 sin(pi x) sin(pi y), u = 0 on the boundary.
void smooth_dirichlet() {
  const Problem problem{
      .equation = {.a_xx = e("-1"),
                   .a_yy = e("-1"),
                   .f = e("2*pi^2*sin(pi*x)*sin(pi*y)")},
      .boundary_conditions = all({.alpha = e("1"), .g = e("0")})};
  const double pi = std::numbers::pi;
  const auto ls = levels(
      problem,
      [pi](double x, double y) {
        return geometry::Point{pi * std::cos(pi * x) * std::sin(pi * y),
                               pi * std::sin(pi * x) * std::cos(pi * y)};
      },
      test::identity_weight, "sin sin");
  check_rates(ls);
  for (const Level &l : ls) {
    const double residual = l.residual / l.error, kelly = l.kelly / l.error;
    check(residual > 1.0 && residual < 10.0, "residual effectivity in a band");
    check(kelly > 0.3 && kelly < 10.0, "Kelly effectivity in a band");
  }
  // Equivalence: the effectivity settles (no drift with h).
  const double first = ls.front().residual / ls.front().error,
               last = ls.back().residual / ls.back().error;
  check(std::abs(last / first - 1.0) < 0.2, "residual effectivity is stable");
  // Recovery: effectivity tends to 1 on the structured mesh.
  double previous = 1e300;
  for (const Level &l : ls) {
    const double distance = std::abs(l.recovery / l.error - 1.0);
    check(distance < previous, "recovery effectivity approaches 1");
    previous = distance;
  }
  check(previous < 0.05, "recovery effectivity within 5% of 1");
}

// The assembly convergence problem (variable anisotropic A, convection,
// reaction) with Dirichlet, Robin and oblique Robin conditions: u =
// exp(x) cos(y). Exercises every term of the residual estimator; the
// boundary residual is of higher order (h^1.5) when its signs are right.
void smooth_mixed() {
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e("exp(x)*cos(y)")};
  const Problem problem{
      .equation =
          {.a_xx = e("-(1 + x^2)"),
           .a_xy = e("-x*y/2"),
           .a_yy = e("-(1 + y^2)"),
           .b_x = e("1"),
           .b_y = e("0.5"),
           .c = e("-1"),
           .f = e("exp(x)*cos(y)*(y^2 - x^2) + exp(x)*sin(y)*(x*y - 1)/2")},
      .boundary_conditions = {
          {"bottom", dirichlet},
          {"left", dirichlet},
          {"right",
           {.alpha = e("1"), .beta_n = e("1"), .g = e("2*exp(x)*cos(y)")}},
          {"top",
           {.alpha = e("2"),
            .beta_n = e("1"),
            .beta_x = e("0.5"),
            .g = e("2.5*exp(x)*cos(y) - exp(x)*sin(y)")}}}};
  const auto ls = levels(
      problem,
      [](double x, double y) {
        return geometry::Point{std::exp(x) * std::cos(y),
                               -std::exp(x) * std::sin(y)};
      },
      // |A| = -A.
      [](double x, double y) {
        return std::array<double, 3>{1 + x * x, x * y / 4, 1 + y * y};
      },
      "mixed");
  check_rates(ls);
  for (const Level &l : ls)
    check(l.residual / l.error > 0.5 && l.residual / l.error < 10.0,
          "residual effectivity in a band");
  for (std::size_t i = 1; i < ls.size(); ++i)
    check(rate(ls[i - 1].boundary_part, ls[i].boundary_part) > 1.3,
          "the boundary residual converges faster than the total");
  check(std::abs(ls.back().recovery / ls.back().error - 1.0) < 0.1,
        "recovery effectivity near 1");
}

// u = 1 + 2x + 3y is in the P1 space and solves
//   -(1 + x) u_xx - 0.5 u_xy - (2 + y) u_yy + u_x - u_y - u = f
// with f = u_x - u_y - u; Robin on the right, oblique Robin on the top
// (both satisfied by u), Dirichlet elsewhere. Every quadrature of assembly
// is exact here, so u_h = u to rounding and every indicator vanishes.
Problem linear_problem(const char *right_g) {
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e("1 + 2*x + 3*y")};
  return Problem{
      .equation = {.a_xx = e("-(1 + x)"),
                   .a_xy = e("-0.5"),
                   .a_yy = e("-(2 + y)"),
                   .b_x = e("1"),
                   .b_y = e("-1"),
                   .c = e("-1"),
                   .f = e("-2 - 2*x - 3*y")},
      .boundary_conditions = {
          {"bottom", dirichlet},
          {"left", dirichlet},
          // u_n = u_x = 2 on x = 1, as a Neumann condition.
          {"right", {.beta_n = e("1"), .g = e(right_g)}},
          // 2 u + u_n + 0.5 u_x = 2 u + 3 + 1 on y = 1.
          {"top",
           {.alpha = e("2"),
            .beta_n = e("1"),
            .beta_x = e("0.5"),
            .g = e("6 + 4*x + 6*y")}}}};
}

void exact_p1_solution() {
  auto meshed = std::make_shared<const MeshedProblem>(test::unit_square(4),
                                                      linear_problem("2"));
  const solve::Solution u = solve_static(meshed);
  check(test::max_nodal_error(*meshed->mesh(), u.values,
                              [](double x, double y) {
                                return 1 + 2 * x + 3 * y;
                              }) < 1e-12,
        "u_h is u");
  const ResidualEstimator residual;
  const KellyEstimator kelly;
  const RecoveryEstimator recovery, gradient({.norm = RecoveryNorm::gradient});
  for (const adapt::ErrorEstimator *estimator :
       {static_cast<const adapt::ErrorEstimator *>(&residual),
        static_cast<const adapt::ErrorEstimator *>(&kelly),
        static_cast<const adapt::ErrorEstimator *>(&recovery),
        static_cast<const adapt::ErrorEstimator *>(&gradient)}) {
    const ErrorEstimate estimate = estimator->estimate(*meshed, u);
    check(estimate.indicators.size() == meshed->mesh()->triangles.size(),
          "one indicator per triangle");
    check(estimate.total < 1e-10, estimator->name() + ": zero estimate");
  }
}

// The same problem with the Neumann data on the right off by delta and
// u_h the exact u (not solved): only the triangles on the right side have
// an indicator, h_E^2 (kappa_n / beta_n * delta)^2 with kappa_n = a_xx =
// -2 on x = 1 (h_E = |E| = 1/n).
void natural_boundary_residual() {
  const double delta = 0.25;
  const std::size_t n = 4;
  for (const char *g : {"2", "2.25"}) {
    auto meshed = std::make_shared<const MeshedProblem>(test::unit_square(n),
                                                        linear_problem(g));
    const solve::Solution u = interpolant(
        *meshed, [](double x, double y) { return 1 + 2 * x + 3 * y; });
    const bool off = std::string(g) != "2";
    for (const ErrorEstimate &estimate :
         {ResidualEstimator().estimate(*meshed, u),
          KellyEstimator().estimate(*meshed, u)}) {
      const auto &m = *meshed->mesh();
      std::size_t on_right = 0;
      for (std::size_t t = 0; t < m.triangles.size(); ++t) {
        std::size_t right_nodes = 0;
        for (std::size_t v : m.triangles[t].nodes)
          right_nodes += m.nodes[v].x == 1.0 ? 1 : 0;
        const bool has_right_edge = right_nodes == 2;
        on_right += has_right_edge ? 1 : 0;
        const double expected =
            off && has_right_edge ? 2.0 * delta / static_cast<double>(n) : 0.0;
        near(estimate.indicators[t], expected, 1e-12,
             estimate.report.stage + ": indicator of triangle " +
                 std::to_string(t));
      }
      check(on_right == n, "n triangles on the right side");
      near(estimate.report.metrics.at("jump_part"), 0.0, 1e-12, "no jumps");
    }
  }
}

// Two triangles of the unit square, T0 = (0,0),(1,0),(1,1) and
// T1 = (0,0),(1,1),(0,1), all nodes Dirichlet; -Laplace u + m1 u_t
// + m2 u_tt = 0 with u_h = x, so grad u_h is the same on both (no jump)
// and r = -m1 u_t - m2 u_tt. h_T = sqrt(2).
void time_terms() {
  const auto square = test::unit_square(1);
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e("x")};
  const std::vector<double> u_t{0, 1, 0, 1}; // x at nodes 0..3
  const std::vector<double> u_tt{1, 1, 1, 1};

  // m1 = 1: integral of x^2 over T0 is 1/4, over T1 1/12.
  {
    const Problem problem{
        .equation = {.a_xx = e("-1"), .a_yy = e("-1"), .m1 = e("1")},
        .boundary_conditions = all(dirichlet),
        .initial = {.u = e("x")}};
    const MeshedProblem meshed(square, problem);
    const solve::Solution u =
        interpolant(meshed, [](double x, double) { return x; }, 0.5);
    expect_throws<std::invalid_argument>(
        [&] { (void)ResidualEstimator().estimate(meshed, u); },
        "u_t is needed");
    const ErrorEstimate estimate =
        ResidualEstimator().estimate(meshed, u, {.u_t = u_t});
    near(estimate.indicators[0], std::sqrt(2.0 / 4), 1e-13, "T0");
    near(estimate.indicators[1], std::sqrt(2.0 / 12), 1e-13, "T1");
    near(estimate.total, std::sqrt(2.0 / 4 + 2.0 / 12), 1e-13, "total");
    // Kelly and recovery ignore the time terms (and need none).
    check(KellyEstimator().estimate(meshed, u).total == 0.0, "Kelly");
    check(RecoveryEstimator().estimate(meshed, u).total < 1e-15, "recovery");
  }
  // m1 = m2 = 1, u_tt = 1: r = -(x + 1); integral of (x + 1)^2 over T0 is
  // 17/12, over T1 11/12.
  {
    const Problem problem{
        .equation = {.a_xx = e("-1"),
                     .a_yy = e("-1"),
                     .m1 = e("1"),
                     .m2 = e("1")},
        .boundary_conditions = all(dirichlet),
        .initial = {.u = e("x"), .u_t = e("0")}};
    const MeshedProblem meshed(square, problem);
    const solve::Solution u =
        interpolant(meshed, [](double x, double) { return x; });
    expect_throws<std::invalid_argument>(
        [&] { (void)ResidualEstimator().estimate(meshed, u, {.u_t = u_t}); },
        "u_tt is needed");
    const ErrorEstimate estimate =
        ResidualEstimator().estimate(meshed, u, {.u_t = u_t, .u_tt = u_tt});
    near(estimate.indicators[0], std::sqrt(2.0 * 17 / 12), 1e-13, "T0");
    near(estimate.indicators[1], std::sqrt(2.0 * 11 / 12), 1e-13, "T1");
  }
  // A dynamic condition on the right, u_n + u_t = 1: with u_h = x, u_n = 1
  // and u_t = 1 on x = 1, so the condition's residual is -1 and
  // R = kappa_n / beta_n * (-1) = 1 (kappa_n = -1): h_E^2 * 1 = 1 on T0.
  {
    const Problem problem{
        .equation = {.a_xx = e("-1"), .a_yy = e("-1")},
        .boundary_conditions = {{"bottom", dirichlet},
                                {"top", dirichlet},
                                {"left", dirichlet},
                                {"right",
                                 {.beta_n = e("1"),
                                  .gamma = e("1"),
                                  .g = e("1")}}},
        .initial = {.u = e("x")}};
    const MeshedProblem meshed(square, problem);
    const solve::Solution u =
        interpolant(meshed, [](double x, double) { return x; });
    expect_throws<std::invalid_argument>(
        [&] { (void)ResidualEstimator().estimate(meshed, u); },
        "u_t is needed");
    const ErrorEstimate estimate =
        ResidualEstimator().estimate(meshed, u, {.u_t = u_t});
    near(estimate.indicators[0], 1.0, 1e-13, "T0 has the dynamic edge");
    near(estimate.indicators[1], 0.0, 1e-13, "T1");
    near(estimate.report.metrics.at("boundary_part"), 1.0, 1e-13, "boundary");
    // Kelly leaves gamma u_t out: u_n = g, so no residual.
    near(KellyEstimator().estimate(meshed, u, {.u_t = u_t}).total, 0.0, 1e-13,
         "Kelly");
  }
}

void invalid_inputs() {
  const Problem poisson{.equation = {.a_xx = e("-1"), .a_yy = e("-1")},
                        .boundary_conditions = all({.alpha = e("1")})};
  const auto square = test::unit_square(2);
  const MeshedProblem meshed(square, poisson);
  const MeshedProblem other(test::unit_square(2), poisson);
  const solve::Solution u =
      interpolant(meshed, [](double x, double y) { return x * y; });
  const ResidualEstimator residual;

  expect_throws<std::invalid_argument>(
      [&] { (void)residual.estimate(other, u); },
      "residual_estimator: the solution is not on the problem's mesh");
  solve::Solution short_u = u;
  short_u.values.pop_back();
  expect_throws<std::invalid_argument>(
      [&] { (void)residual.estimate(meshed, short_u); },
      "the solution has 8 values for 9 mesh nodes");
  solve::Solution nan_u = u;
  nan_u.values[3] = std::nan("");
  expect_throws<std::invalid_argument>(
      [&] { (void)KellyEstimator().estimate(meshed, nan_u); },
      "kelly_estimator: the solution is not finite at node 3");
  const std::vector<double> two{1, 2};
  expect_throws<std::invalid_argument>(
      [&] { (void)residual.estimate(meshed, u, {.u_t = two}); },
      "u_t has 2 values for 9 mesh nodes");
  expect_throws<std::invalid_argument>(
      [&] { (void)residual.estimate(meshed, u, {.u_tt = two}); },
      "u_tt has 2 values");

  // The energy norm needs a definite (or semidefinite) A.
  const Problem reaction{.equation = {.c = e("1"), .f = e("x")},
                         .boundary_conditions = all({.alpha = e("1")})};
  const MeshedProblem no_a(square, reaction);
  const solve::Solution v = interpolant(no_a, [](double x, double) { return x; });
  expect_throws<std::invalid_argument>(
      [&] { (void)RecoveryEstimator().estimate(no_a, v); },
      "the energy norm needs A, which is zero");
  check(RecoveryEstimator({.norm = RecoveryNorm::gradient})
                .estimate(no_a, v)
                .total >= 0.0,
        "the gradient norm needs no A");
  const Problem wave{.equation = {.a_xx = e("1"), .a_yy = e("-1")},
                     .boundary_conditions = all({.alpha = e("1")})};
  const MeshedProblem indefinite(square, wave);
  expect_throws<std::invalid_argument>(
      [&] { (void)RecoveryEstimator().estimate(indefinite, u); },
      "A is indefinite at");
  expect_throws<std::invalid_argument>(
      [] {
        (void)RecoveryEstimator({.norm = static_cast<RecoveryNorm>(7)});
      },
      "RecoveryEstimatorOptions::norm must be");

}

} // namespace

int main() {
  return test::run({
      {"smooth Dirichlet problem: effectivities and rates", smooth_dirichlet},
      {"smooth mixed problem: effectivities and rates", smooth_mixed},
      {"an exact P1 solution has zero indicators", exact_p1_solution},
      {"natural boundary residual", natural_boundary_residual},
      {"time terms of the residual estimator", time_terms},
      {"invalid inputs", invalid_inputs},
  });
}

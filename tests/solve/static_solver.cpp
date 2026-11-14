#include "solve_support.hpp"

#include "pde2d/solve/bicgstab.hpp"
#include "pde2d/solve/solution.hpp"
#include "pde2d/solve/sparse_lu.hpp"
#include "pde2d/solve/static_solver.hpp"

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace pde2d::fem;
using namespace pde2d::solve;
using pde2d::test::check;
using pde2d::test::expect_throws;
using pde2d::test::near;

namespace {

Expression e(const char *text) { return Expression(text); }

// Dirichlet u = 5 + y + t on the left, u_n = 0 elsewhere, on a 2 x 2 grid:
// the left nodes 0, 3, 6 are Dirichlet, the other six are free.
std::shared_ptr<const MeshedProblem> left_dirichlet() {
  const BoundaryCondition neumann{.beta_n = e("1")};
  return std::make_shared<const MeshedProblem>(
      pde2d::test::unit_square(2),
      Problem{.equation = {.a_xx = e("-1"), .a_yy = e("-1")},
              .boundary_conditions = {
                  {"left", {.alpha = e("1"), .g = e("5 + y + t")}},
                  {"bottom", neumann},
                  {"right", neumann},
                  {"top", neumann}}});
}

void builds_solution() {
  const auto meshed = left_dirichlet();
  check(meshed->free_nodes() == std::vector<std::size_t>{1, 2, 4, 5, 7, 8},
        "free nodes");
  const std::vector<double> free_values = {10, 20, 40, 50, 70, 80};
  const Solution s = build_solution(*meshed, free_values, 2.0);
  check(s.mesh == meshed->mesh(), "the problem's mesh object");
  check(s.time == 2.0, "the time");
  check(s.velocities.empty(), "no velocities");
  const std::vector<double> expected = {7.0, 10, 20, 7.5, 40, 50, 8.0, 70, 80};
  check(s.values == expected, "free values placed, Dirichlet values at t");
  expect_throws<std::invalid_argument>(
      [&] { (void)build_solution(*meshed, std::vector<double>(5, 0.0), 0.0); },
      "5 free values for 6 free nodes");
  expect_throws<std::invalid_argument>(
      [&] { (void)build_solution(*meshed, free_values, std::nan("")); },
      "not finite");
}

// The manufactured problem of tests/fem/assembly_convergence.cpp,
// u = exp(x) cos(y) with variable A, convection and reaction; Dirichlet on
// the bottom and the left, Robin on the right and an oblique Robin on the
// top.
double exact(double x, double y) { return std::exp(x) * std::cos(y); }

Problem mixed_problem() {
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e("exp(x)*cos(y)")};
  return {.equation =
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
}

void converges(std::shared_ptr<const LinearSolver> linear_solver,
               const char *name) {
  const StaticSolver solver(std::move(linear_solver));
  std::vector<double> errors;
  for (const std::size_t n : {8, 16, 32}) {
    const auto meshed = std::make_shared<const MeshedProblem>(
        pde2d::test::unit_square(n), mixed_problem());
    const StaticResult result = solver.solve(meshed);
    errors.push_back(
        pde2d::test::l2_error(*meshed->mesh(), result.solution.values, exact));
  }
  for (std::size_t i = 1; i < errors.size(); ++i) {
    const double rate = std::log2(errors[i - 1] / errors[i]);
    std::cout << name << ": L2 error " << errors[i - 1] << " -> " << errors[i]
              << ", rate " << rate << '\n';
    check(rate > 1.8 && rate < 2.3, "second order");
  }
}

// A problem of time order 1 whose Dirichlet data moves (U_D' = 1). The
// static solve at t drops the time terms, including the lifting of U_D':
// the solution is the harmonic function that equals the data, which P1
// reproduces exactly.
void drops_time_terms() {
  const BoundaryCondition dirichlet{.alpha = e("1"), .g = e("1 + x + 2*y + t")};
  const auto meshed = std::make_shared<const MeshedProblem>(
      pde2d::test::unit_square(4),
      Problem{.equation = {.a_xx = e("-1"), .a_yy = e("-1"), .m1 = e("1")},
              .boundary_conditions = {{"bottom", dirichlet},
                                      {"right", dirichlet},
                                      {"top", dirichlet},
                                      {"left", dirichlet}},
              .initial = {.u = e("0")}});
  check(time_order(*meshed) == 1, "time order 1");
  const StaticResult result =
      StaticSolver(std::make_shared<SparseLu>()).solve(meshed, 0.5);
  check(result.solution.time == 0.5, "the time");
  const double error = pde2d::test::max_nodal_error(
      *meshed->mesh(), result.solution.values,
      [](double x, double y) { return 1.5 + x + 2 * y; });
  near(error, 0.0, 1e-12, "the static solution at t = 0.5");

  check(result.reports.size() == 2, "two reports");
  check(result.reports[0].stage == "assembly", "assembly report");
  check(result.reports[1].stage == "static_solver", "solver report");
  check(result.reports[1].metrics.at("free_unknowns") == 9, "9 free unknowns");
  check(result.reports[1].metrics.at("residual") < 1e-12, "residual");
}

} // namespace

int main() {
  return pde2d::test::run({
      {"build_solution places free and Dirichlet values", builds_solution},
      {"static mixed problem converges at second order with SparseLu",
       [] { converges(std::make_shared<SparseLu>(), "sparse_lu"); }},
      {"static mixed problem converges at second order with BiCgStab",
       [] {
         converges(std::make_shared<BiCgStab>(BiCgStabOptions{
                       .tolerance = 1e-12, .max_iterations = 5000}),
                   "bicgstab");
       }},
      {"the static solver drops the time terms", drops_time_terms},
      {"the static solver rejects bad input",
       [] {
         expect_throws<std::invalid_argument>([] { StaticSolver(nullptr); },
                                              "linear solver is null");
         const StaticSolver solver(std::make_shared<SparseLu>());
         expect_throws<std::invalid_argument>(
             [&] { (void)solver.solve(nullptr); }, "problem is null");
         expect_throws<std::invalid_argument>(
             [&] { (void)solver.solve(left_dirichlet(), std::nan("")); },
             "not finite");
       }},
  });
}

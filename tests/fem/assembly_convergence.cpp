#include "assembly_support.hpp"

#include "pde2d/fem/assembler.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// Manufactured solution u = exp(x) cos(y) on the unit square for
//   a_xx u_xx + a_xy u_xy + a_yy u_yy + b . grad u + c u = f
// with variable A (so the div A term matters):
//   a_xx = -(1 + x^2), a_xy = -x y / 2, a_yy = -(1 + y^2), b = (1, 0.5),
//   c = -1.
// u_xx = u, u_yy = -u, u_xy = -exp(x) sin(y), so
//   f = exp(x) cos(y) (y^2 - x^2) + exp(x) sin(y) (x y - 1) / 2.
// The assembled system is solved with dense Gaussian elimination on Mesher
// meshes of decreasing size, and the L2 error must fall like h^2.

using namespace pde2d::fem;
using pde2d::test::check;

namespace {

Expression e(const char *text) { return Expression(text); }

double exact(double x, double y) { return std::exp(x) * std::cos(y); }

Equation equation() {
  return {.a_xx = e("-(1 + x^2)"),
          .a_xy = e("-x*y/2"),
          .a_yy = e("-(1 + y^2)"),
          .b_x = e("1"),
          .b_y = e("0.5"),
          .c = e("-1"),
          .f = e("exp(x)*cos(y)*(y^2 - x^2) + exp(x)*sin(y)*(x*y - 1)/2")};
}

BoundaryCondition dirichlet() {
  return {.alpha = e("1"), .g = e("exp(x)*cos(y)")};
}

std::map<std::string, BoundaryCondition> all_dirichlet() {
  return {{"bottom", dirichlet()},
          {"right", dirichlet()},
          {"top", dirichlet()},
          {"left", dirichlet()}};
}

// Dirichlet on the bottom and the left; Robin on the right (n = (1, 0),
// u + u_n = 2 u) and an oblique Robin on the top (n = (0, 1),
// 2 u + u_n + 0.5 u_x = 2.5 u - exp(x) sin(y)).
std::map<std::string, BoundaryCondition> mixed() {
  return {
      {"bottom", dirichlet()},
      {"left", dirichlet()},
      {"right", {.alpha = e("1"), .beta_n = e("1"), .g = e("2*exp(x)*cos(y)")}},
      {"top",
       {.alpha = e("2"),
        .beta_n = e("1"),
        .beta_x = e("0.5"),
        .g = e("2.5*exp(x)*cos(y) - exp(x)*sin(y)")}}};
}

struct Result {
  std::size_t nodes = 0;
  double l2_error = 0.0;
};

Result solve(double h, const std::map<std::string, BoundaryCondition> &bcs) {
  const auto mesh = pde2d::test::rectangle_mesh(1, 1, h);
  const auto problem = std::make_shared<const MeshedProblem>(
      mesh, Problem{.equation = equation(), .boundary_conditions = bcs});
  const auto assembled = Assembler().assemble(problem, 0.0);
  check(assembled.time_order() == 0, "static");
  const auto free_values = pde2d::test::solve(
      pde2d::test::dense(assembled.stiffness().free),
      {assembled.right_hand_side().begin(), assembled.right_hand_side().end()});
  std::vector<double> u(mesh->nodes.size(), 0.0);
  for (std::size_t i = 0; i < free_values.size(); ++i)
    u[problem->free_nodes()[i]] = free_values[i];
  for (std::size_t k = 0; k < problem->dirichlet_nodes().size(); ++k)
    u[problem->dirichlet_nodes()[k].node] = assembled.dirichlet_values()[k];

  // L2 error with the edge-midpoint rule on each triangle.
  double sum = 0.0;
  for (const auto &triangle : mesh->triangles) {
    const double area = pde2d::test::triangle_area(*mesh, triangle);
    for (std::size_t m = 0; m < 3; ++m) {
      const std::size_t a = triangle.nodes[m], b = triangle.nodes[(m + 1) % 3];
      const auto &p = mesh->nodes[a], &q = mesh->nodes[b];
      const double uh = (u[a] + u[b]) / 2;
      const double d = uh - exact((p.x + q.x) / 2, (p.y + q.y) / 2);
      sum += area / 3 * d * d;
    }
  }
  return {.nodes = mesh->nodes.size(), .l2_error = std::sqrt(sum)};
}

void converges(const std::map<std::string, BoundaryCondition> &bcs,
               const char *name) {
  std::vector<Result> results;
  for (const double h : {0.2, 0.1, 0.05})
    results.push_back(solve(h, bcs));
  for (std::size_t i = 1; i < results.size(); ++i) {
    // h is proportional to nodes^(-1/2).
    const double rate =
        -2.0 * std::log(results[i].l2_error / results[i - 1].l2_error) /
        std::log(static_cast<double>(results[i].nodes) /
                 static_cast<double>(results[i - 1].nodes));
    std::cout << name << ": " << results[i - 1].nodes << " -> "
              << results[i].nodes << " nodes, L2 error "
              << results[i - 1].l2_error << " -> " << results[i].l2_error
              << ", rate " << rate << '\n';
    check(rate > 1.7 && rate < 2.5, "second order");
  }
  check(results.back().l2_error < 1e-3, "small error");
}

} // namespace

int main() {
  return pde2d::test::run({
      {"Dirichlet problem converges at second order",
       [] { converges(all_dirichlet(), "Dirichlet"); }},
      {"mixed Dirichlet / Robin problem converges at second order",
       [] { converges(mixed(), "mixed"); }},
  });
}

#include "fem/solver.hpp"
#include "../examples/fem/square_mesh.hpp"
#include "../examples/fem/error_norms.hpp"
#include <iostream>
#include <numbers>
#include <stdexcept>

int main() {
  try {
    using namespace fem;
    const double pi = std::numbers::pi;
    const Expr exact = sin(pi * x) * sin(pi * y);
    Problem p;
    p.equation.source = 2 * pi * pi * exact;
    p.assign(dirichlet(0), {"bottom", "right", "top", "left"});
    ErrorNorms previous{};
    for (const int n : {4, 8, 16}) {
      auto mesh = square_mesh(n);
      auto solution = solve_stationary(mesh, p, AssemblyOptions{6});
      if (!solution.report.converged)
        throw std::runtime_error("Stationary solve failed");
      auto e = error_norms(mesh, solution.values, exact);
      std::cout << "n=" << n << " L2=" << e.l2 << " H1=" << e.h1_seminorm
                << '\n';
      if (n > 4 && (previous.l2 / e.l2 < 3.4 ||
                    previous.h1_seminorm / e.h1_seminorm < 1.85))
        throw std::runtime_error(
            "Manufactured Poisson solution lost expected refinement rates");
      previous = e;
    }
    // Mixed spatial derivatives and coefficient derivatives: a bilinear
    // solution exercises the mixed term; mesh refinement must converge to that
    // solution.
    p = Problem{};
    const Expr mixed = 1 + x + 2 * y + x * y;
    auto &eq = p.equation;
    eq.a_xx = -(1 + x * x);
    eq.a_xy = -0.4 * (1 + x * y);
    eq.a_yy = -(2 + y * y);
    eq.b_x = .3 + y;
    eq.b_y = -.2 + x;
    eq.c = 1;
    eq.source = eq.a_xx * diff(diff(mixed, x), x) +
                eq.a_xy * diff(diff(mixed, x), y) +
                eq.a_yy * diff(diff(mixed, y), y) + eq.b_x * diff(mixed, x) +
                eq.b_y * diff(mixed, y) + eq.c * mixed;
    const Expr flux =
        -nx * (eq.a_xx * diff(mixed, x) + .5 * eq.a_xy * diff(mixed, y)) -
        ny * (.5 * eq.a_xy * diff(mixed, x) + eq.a_yy * diff(mixed, y));
    p.assign(dirichlet(mixed), {"left", "right"});
    p.assign(flux_neumann(flux), {"bottom"});
    p.assign(flux_robin(2, .7, 2 * mixed + .7 * flux), {"top"});
    double prev = 1e9;
    for (int n : {4, 8, 16}) {
      auto mesh = square_mesh(n);
      auto s = solve_stationary(mesh, p, AssemblyOptions{8});
      auto e = error_norms(mesh, s.values, mixed);
      std::cout << "mixed n=" << n << " L2=" << e.l2 << '\n';
      if (e.l2 >= prev * .4)
        throw std::runtime_error(
            "Mixed-variable-coefficient solve does not converge quadratically");
      prev = e.l2;
    }
    std::cout << "FEM convergence checks passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

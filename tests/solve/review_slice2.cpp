#include "solve_support.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/solve/conjugate_gradient.hpp"
#include "pde2d/solve/fixed_step.hpp"
#include "pde2d/solve/implicit_euler.hpp"
#include "pde2d/solve/implicit_midpoint.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/simplicial_ldlt.hpp"
#include "pde2d/solve/sparse_lu.hpp"
#include "pde2d/solve/static_solver.hpp"

#include <algorithm>
#include <cmath>
#include <span>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Cases from the review of solver slice 2 (c72b876..97cc453). The first
// three reproduced defects there and now guard their fixes; the others
// check paths the slice's own tests did not reach.

using namespace pde2d::fem;
using namespace pde2d::solve;
using pde2d::test::check;
using pde2d::test::near;
using pde2d::test::sparse;

namespace {

Expression e(const std::string &text) { return Expression(text); }

std::shared_ptr<const MeshedProblem> meshed(const Problem &problem,
                                            std::size_t n = 4) {
  return std::make_shared<const MeshedProblem>(pde2d::test::unit_square(n),
                                               problem);
}

Integrator integrator(std::shared_ptr<const TimeMethod> method, double dt) {
  return Integrator(std::move(method),
                    std::make_shared<FixedStep>(FixedStepOptions{.dt = dt}),
                    std::make_shared<SparseLu>());
}

// SimplicialLdlt (no pivoting) used to return a wrong answer for a
// symmetric indefinite matrix with a tiny leading pivot, without throwing.
// The exact solution of [[eps, 1], [1, eps]] x = (1, 1) is x = (1, 1) /
// (1 + eps), about (1, 1).
void ldlt_indefinite_silent() {
  const double eps = 1e-20;
  const auto matrix = sparse({{eps, 1.0}, {1.0, eps}});
  bool threw = false;
  LinearSolveResult result;
  try {
    result = SimplicialLdlt().prepare(matrix)->solve(std::vector{1.0, 1.0});
  } catch (const std::runtime_error &error) {
    threw = true;
    std::cout << "simplicial_ldlt threw: " << error.what() << '\n';
  }
  if (!threw)
    std::cout << "simplicial_ldlt returned x = (" << result.values[0] << ", "
              << result.values[1] << "), residual " << result.residual << '\n';
  const auto lu = SparseLu().prepare(matrix)->solve(std::vector{1.0, 1.0});
  std::cout << "sparse_lu returned x = (" << lu.values[0] << ", "
            << lu.values[1] << "), residual " << lu.residual << '\n';
  check(threw || (std::abs(result.values[0] - 1.0) < 1e-8 &&
                  std::abs(result.values[1] - 1.0) < 1e-8),
        "LDL^T either throws or returns the right answer");
}

// The same through the StaticSolver: a symmetric indefinite A (Helmholtz)
// solved by LDL^T compared with LU, over a range of c. Reports the
// worst residual LDL^T returned without throwing.
void ldlt_helmholtz_sweep() {
  const BoundaryCondition zero{.alpha = e("1")};
  double worst = 0.0, worst_c = 0.0;
  for (int i = 1; i <= 400; ++i) {
    const double c = -1.0 * i;
    const auto m = std::make_shared<const MeshedProblem>(
        pde2d::test::unit_square(6),
        Problem{.equation = {.a_xx = e("-1"),
                             .a_yy = e("-1"),
                             .c = e(std::to_string(c)),
                             .f = e("1")},
                .boundary_conditions = {{"bottom", zero},
                                        {"right", zero},
                                        {"top", zero},
                                        {"left", zero}}});
    try {
      const auto r = StaticSolver(std::make_shared<SimplicialLdlt>()).solve(m);
      const double residual = r.reports[1].metrics.at("residual");
      if (residual > worst) {
        worst = residual;
        worst_c = c;
      }
    } catch (const std::runtime_error &) {
    }
  }
  std::cout << "helmholtz sweep: worst LDL^T residual returned " << worst
            << " at c = " << worst_c << '\n';
  const auto m = std::make_shared<const MeshedProblem>(
      pde2d::test::unit_square(6),
      Problem{.equation = {.a_xx = e("-1"),
                           .a_yy = e("-1"),
                           .c = e(std::to_string(worst_c)),
                           .f = e("1")},
              .boundary_conditions = {{"bottom", zero},
                                      {"right", zero},
                                      {"top", zero},
                                      {"left", zero}}});
  const auto lu = StaticSolver(std::make_shared<SparseLu>()).solve(m);
  const auto ldlt = StaticSolver(std::make_shared<SimplicialLdlt>()).solve(m);
  double difference = 0.0, size = 0.0;
  for (std::size_t k = 0; k < lu.solution.values.size(); ++k) {
    difference = std::max(
        difference, std::abs(lu.solution.values[k] - ldlt.solution.values[k]));
    size = std::max(size, std::abs(lu.solution.values[k]));
  }
  std::cout << "at c = " << worst_c << ": sparse_lu residual "
            << lu.reports[1].metrics.at("residual") << ", max |u_lu| " << size
            << ", max |u_lu - u_ldlt| " << difference << '\n';
  check(worst < 1e-8, "LDL^T never returns a large residual silently");
}

// At time order 0 an `initial` with values that are not finite used to be
// accepted without a word, although integrator.hpp lists "values that are
// not finite" among the errors.
void order_zero_non_finite_initial() {
  const BoundaryCondition natural{.beta_n = e("1")};
  const auto m = meshed(
      {.equation = {.a_xx = e("-1"), .a_yy = e("-1"), .c = e("1"), .f = e("1")},
       .boundary_conditions = {{"bottom", natural},
                               {"right", natural},
                               {"top", natural},
                               {"left", natural}}});
  Solution initial{
      .mesh = m->mesh(),
      .time = 0.0,
      .values = std::vector<double>(m->mesh()->nodes.size(),
                                    std::numeric_limits<double>::quiet_NaN()),
      .velocities = {}};
  bool threw = false;
  try {
    (void)integrator(std::make_shared<ImplicitEuler>(), 0.5)
        .integrate(m, 0.0, 1.0, {}, &initial);
  } catch (const std::invalid_argument &error) {
    threw = true;
    std::cout << "order 0, NaN initial: " << error.what() << '\n';
  }
  check(threw, "a non-finite initial is refused (as documented)");
}

// Test gap, expected to pass: a constant operator copied from a
// `previous` assembled at another level (the slice's cross-level loop
// uses a problem where every operator depends on t).
bool same(const Operator &a, const Operator &b) {
  return a.free.values == b.free.values &&
         a.dirichlet.values == b.dirichlet.values &&
         a.depends_on_t == b.depends_on_t && a.is_zero == b.is_zero &&
         a.symmetric == b.symmetric;
}
bool same(const AssembledProblem &a, const AssembledProblem &b) {
  const auto eq = [](std::span<const double> x, std::span<const double> y) {
    return std::ranges::equal(x, y);
  };
  return same(a.mass(), b.mass()) && same(a.damping(), b.damping()) &&
         same(a.stiffness(), b.stiffness()) && eq(a.load(), b.load()) &&
         eq(a.dirichlet_values(), b.dirichlet_values()) &&
         eq(a.dirichlet_rates(), b.dirichlet_rates()) &&
         eq(a.dirichlet_accelerations(), b.dirichlet_accelerations()) &&
         eq(a.right_hand_side(), b.right_hand_side()) &&
         a.load_depends_on_t() == b.load_depends_on_t() &&
         a.right_hand_side_depends_on_t() == b.right_hand_side_depends_on_t() &&
         a.time_order() == b.time_order();
}
void constant_operators_across_levels() {
  const BoundaryCondition dyn{
      .alpha = e("1"), .beta_n = e("1"), .gamma = e("2"), .g = e("x")};
  const BoundaryCondition d{.alpha = e("1"), .g = e("t*t*y")};
  const auto m = meshed({.equation = {.a_xx = e("-1"),
                                      .a_xy = e("0.3"),
                                      .a_yy = e("-2"),
                                      .c = e("1"),
                                      .m1 = e("1 + x"),
                                      .m2 = e("2 + y")},
                         .boundary_conditions = {{"bottom", dyn},
                                                 {"right", d},
                                                 {"top", dyn},
                                                 {"left", d}},
                         .initial = {.u = e("0"), .u_t = e("0")}});
  const Assembler assembler;
  for (int before = 0; before <= 2; ++before)
    for (int now = 0; now <= 2; ++now) {
      const auto first = assembler.assemble(m, 0.3, nullptr, before);
      const auto again = assembler.assemble(m, 1.1, &first, now);
      check(same(again, assembler.assemble(m, 1.1, nullptr, now)),
            "constant operators from level " + std::to_string(before) + " to " +
                std::to_string(now));
      for (int next = 0; next <= 2; ++next)
        check(same(assembler.assemble(m, 2.0, &again, next),
                   assembler.assemble(m, 2.0, nullptr, next)),
              "chained " + std::to_string(before) + std::to_string(now) +
                  std::to_string(next));
    }
}

double asymmetry(const pde2d::fem::CsrMatrix &a) {
  const auto &p = *a.pattern;
  double worst = 0.0, scale = 0.0;
  for (std::size_t i = 0; i < p.rows; ++i)
    for (auto k = p.row_offsets[i]; k < p.row_offsets[i + 1]; ++k) {
      const auto s = static_cast<std::size_t>(k);
      const auto j = static_cast<std::size_t>(p.column_indices[s]);
      const auto t = a.pattern->find(j, i);
      const double other = t ? a.values[*t] : 0.0;
      worst = std::max(worst, std::abs(a.values[s] - other));
      scale = std::max(scale, std::abs(a.values[s]));
    }
  return worst / scale;
}

// Expected to pass: isotropy with parameters and after expanding, with
// natural conditions on every side (T would show on every edge direction).
void isotropy_cases() {
  const BoundaryCondition robin{.alpha = e("1"), .beta_n = e("2")};
  const auto stiffness = [&](Equation equation, Parameters parameters) {
    const auto m = std::make_shared<const MeshedProblem>(
        pde2d::test::unit_square(3),
        Problem{.equation = std::move(equation),
                .parameters = std::move(parameters),
                .boundary_conditions = {{"bottom", robin},
                                        {"right", robin},
                                        {"top", robin},
                                        {"left", robin}}});
    return Assembler().assemble(m, 0.5, nullptr, 0);
  };
  const auto a1 = stiffness(
      {.a_xx = e("-(1 + k)^2"), .a_yy = e("-1 - 2*k - k^2")}, {{"k", 0.5}});
  check(a1.stiffness().symmetric, "parameter, isotropic after expanding");
  check(asymmetry(a1.stiffness().free) < 1e-14, "and A = A^T");
  // A non-constant isotropic A has div A: a convection, not symmetric.
  const auto a2 = stiffness({.a_xx = e("-(1 + x)"), .a_yy = e("-1 - x")}, {});
  check(!a2.stiffness().symmetric, "div A not zero: not symmetric");
  check(asymmetry(a2.stiffness().free) > 1e-6, "and A != A^T");
  // k1 = k2 = 1 numerically: conservatively not isotropic.
  const auto a3 = stiffness({.a_xx = e("-k1"), .a_yy = e("-k2")},
                            {{"k1", 1.0}, {"k2", 1.0}});
  std::cout << "a_xx = k1, a_yy = k2 (both 1): symmetric = "
            << a3.stiffness().symmetric << ", asymmetry "
            << asymmetry(a3.stiffness().free) << '\n';
}

// Expected to pass: the u_n check still runs when every edge term is zero
// (isotropic A, u_n x = 0 on the left side, where x = 0).
void beta_n_check_kept() {
  const BoundaryCondition natural{.beta_n = e("1")};
  const auto m =
      meshed({.equation = {.a_xx = e("-1"), .a_yy = e("-1")},
              .boundary_conditions = {{"bottom", natural},
                                      {"right", natural},
                                      {"top", natural},
                                      {"left", {.beta_n = e("x")}}}});
  for (const int level : {0, 1, 2}) {
    bool threw = false;
    try {
      (void)Assembler().assemble(m, 0.0, nullptr, level);
    } catch (const std::invalid_argument &error) {
      threw = std::string(error.what()).find("cannot be solved") !=
              std::string::npos;
    }
    check(threw, "u_n check at level " + std::to_string(level));
  }
}

} // namespace

int main() {
  return pde2d::test::run({
      {"LDL^T on a symmetric indefinite matrix throws or is right",
       ldlt_indefinite_silent},
      {"LDL^T never returns a large residual (Helmholtz sweep)",
       ldlt_helmholtz_sweep},
      {"order 0 refuses a non-finite initial", order_zero_non_finite_initial},
      {"constant operators reused across levels",
       constant_operators_across_levels},
      {"isotropy cases", isotropy_cases},
      {"the u_n check with every edge term zero", beta_n_check_kept},
  });
}

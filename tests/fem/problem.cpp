#include "pde2d/fem/problem.hpp"
#include "support/check.hpp"

#include <stdexcept>

using namespace pde2d::fem;
using pde2d::test::check;
using pde2d::test::expect_throws;

namespace {

Expression e(const char *text) { return Expression(text); }

// A problem with one region whose condition is `condition`.
Problem with_condition(BoundaryCondition condition) {
  return {.equation = {.a_xx = e("-1"), .a_yy = e("-1"), .f = e("1")},
          .boundary_conditions = {{"wall", std::move(condition)}}};
}

} // namespace

int main() {
  return pde2d::test::run({
      {"Dirichlet, natural and dynamic conditions",
       [] {
         check(classify({.alpha = e("1"), .g = e("x")}) ==
                   ConditionKind::dirichlet,
               "u = x");
         check(classify({.alpha = e("2 + x"), .g = e("0")}) ==
                   ConditionKind::dirichlet,
               "(2 + x) u = 0");
         check(classify({.beta_n = e("1")}) == ConditionKind::natural,
               "Neumann");
         check(classify({.alpha = e("h"), .beta_n = e("1"), .g = e("h*20")}) ==
                   ConditionKind::natural,
               "Robin");
         check(classify({.beta_x = e("1")}) == ConditionKind::natural, "u_x");
         check(classify({.beta_n = e("1"), .beta_tau = e("1")}) ==
                   ConditionKind::natural,
               "u_n + u_tau");
         check(classify({.beta_n = e("1"), .gamma = e("1")}) ==
                   ConditionKind::dynamic,
               "u_n + u_t");
         check(classify({.alpha = e("1"), .beta_y = e("y"), .gamma = e("t")}) ==
                   ConditionKind::dynamic,
               "oblique dynamic");
       }},
      {"oblique means a derivative that is not only normal",
       [] {
         check(!is_oblique({.alpha = e("1")}), "Dirichlet");
         check(!is_oblique({.alpha = e("1"), .beta_n = e("1")}), "Robin");
         check(is_oblique({.beta_x = e("1")}), "u_x");
         check(is_oblique({.beta_n = e("1"), .beta_y = e("1")}), "u_n + u_y");
         check(is_oblique({.beta_n = e("1"), .beta_tau = e("2")}),
               "u_n + 2 u_tau");
       }},
      {"classification reads zero as written",
       [] {
         check(classify({.alpha = e("1"), .beta_n = e("x - x")}) ==
                   ConditionKind::dirichlet,
               "x - x is zero");
         // k could be anything: a derivative term is present.
         check(classify({.alpha = e("1"), .beta_n = e("k")}) ==
                   ConditionKind::natural,
               "k is not zero");
       }},
      {"unsupported conditions are rejected",
       [] {
         expect_throws<std::invalid_argument>([] { classify({.g = e("1")}); },
                                              "has no term on its left-hand");
         expect_throws<std::invalid_argument>(
             [] { classify({.alpha = e("1"), .gamma = e("1")}); },
             "is a boundary ODE");
         expect_throws<std::invalid_argument>(
             [] { classify({.gamma = e("1"), .g = e("t")}); },
             "is a boundary ODE");
         for (const BoundaryCondition &tangential :
              {BoundaryCondition{.beta_tau = e("1")},
               BoundaryCondition{.alpha = e("1"), .beta_tau = e("x")},
               BoundaryCondition{.beta_tau = e("1"), .gamma = e("1")}})
           expect_throws<std::invalid_argument>([&] { classify(tangential); },
                                                "only a tangential derivative");
       }},
      {"time order",
       [] {
         check(time_order(with_condition({.alpha = e("1")})) == 0,
               "stationary");
         Problem heat = with_condition({.alpha = e("1")});
         heat.equation.m1 = e("-1");
         check(time_order(heat) == 1, "m1");
         Problem wave = heat;
         wave.equation.m2 = e("1");
         check(time_order(wave) == 2, "m2");
         Problem written_zero = with_condition({.alpha = e("1")});
         written_zero.equation.m2 = e("t - t");
         check(time_order(written_zero) == 0, "m2 = t - t");
         check(time_order(
                   with_condition({.beta_n = e("1"), .gamma = e("1")})) == 1,
               "a dynamic condition makes the problem time-dependent");
       }},
      {"which side depends on t",
       [] {
         const Problem constant =
             with_condition({.alpha = e("1"), .g = e("x*y")});
         check(!coefficients_depend_on_t(constant) &&
                   !data_depends_on_t(constant),
               "nothing depends on t");
         Problem c_of_t = constant;
         c_of_t.equation.c = e("t");
         check(coefficients_depend_on_t(c_of_t) && !data_depends_on_t(c_of_t),
               "c(t)");
         Problem f_of_t = constant;
         f_of_t.equation.f = e("sin(t)");
         check(!coefficients_depend_on_t(f_of_t) && data_depends_on_t(f_of_t),
               "f(t)");
         const Problem alpha_of_t =
             with_condition({.alpha = e("1 + t"), .g = e("1")});
         check(coefficients_depend_on_t(alpha_of_t) &&
                   !data_depends_on_t(alpha_of_t),
               "alpha(t)");
         const Problem beta_of_t =
             with_condition({.beta_n = e("1"), .beta_x = e("t")});
         check(coefficients_depend_on_t(beta_of_t), "beta_x(t)");
       }},
      {"boundary data g alone makes the right-hand side depend on t",
       [] {
         Problem problem = with_condition({.alpha = e("1"), .g = e("0")});
         problem.boundary_conditions["inlet"] = {.alpha = e("1"),
                                                 .g = e("sin(t)")};
         check(!problem.equation.f.facts().depends_on_t, "f is constant");
         check(data_depends_on_t(problem), "Dirichlet g(t)");
         check(!coefficients_depend_on_t(problem), "left-hand side constant");
         Problem natural = with_condition({.beta_n = e("1"), .g = e("t*x")});
         check(data_depends_on_t(natural), "natural g(t)");
       }},
  });
}

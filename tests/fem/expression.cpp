#include "fem/expression.hpp"
#include "support/check.hpp"
#include <iostream>
#include <limits>
#include <type_traits>

using pde2d_test::check;
using pde2d_test::expect_throws;
using namespace fem;
static_assert(std::is_same_v<Expr, SymEngine::Expression>);
static_assert(std::is_floating_point_v<decltype(fem::sin(1))>);
static_assert(std::is_floating_point_v<decltype(fem::pow(2, 3))>);
namespace {
void near(double a, double b) { pde2d_test::near(a, b, 1e-12); }
} // namespace
void expression_semantics() {
  Expr polynomial = pow(x + y, 3) + t * nx + ny;
  check(is_zero(diff(polynomial, x) - 3 * pow(x + y, 2)),
        "Symbolic derivative mismatch");
  check(is_zero(pow(x + y, 2) - x * x - 2 * x * y - y * y),
        "Algebraic expansion mismatch");
  check(!is_zero(x - y), "Nonzero expression lost");
  check(is_zero(Expr(0.0)) && is_zero(Expr(-0.0)),
        "Floating-point zero not recognized");
  check(!is_zero(Expr(1e-30)), "Small nonzero coefficient was treated as zero");
  Evaluator prepared(polynomial);
  check(prepared.expression() == polynomial,
        "Original expression not retained");
  for (int i = 0; i < 100; ++i) {
    EvaluationPoint p{.01 * i, .2, .3, .4, .5};
    near(prepared(p), std::pow(p.x + p.y, 3) + .3 * .4 + .5);
  }
  const auto copied = prepared;
  near(copied({1, 2, 3, 4, 5}), 44);
  Expr functions =
      fem::sin(x) + fem::cos(y) + fem::exp(t) + fem::sqrt(nx) + fem::log(ny);
  near(evaluate(functions, {.2, .3, .4, 4, 2}),
       std::sin(.2) + std::cos(.3) + std::exp(.4) + 2 + std::log(2));
  near(evaluate(fem::pow(2, x), {3}), 8);
  near(fem::sin(.3) + fem::cos(.2) + fem::exp(.1) + fem::sqrt(4) + fem::log(2) +
           fem::pow(2, 3),
       std::sin(.3) + std::cos(.2) + std::exp(.1) + 2 + std::log(2) + 8);
  near(evaluate(diff(fem::sin(x), x), {.4}), std::cos(.4));
  expect_throws([] { Evaluator invalid(Expr(SymEngine::symbol("unbound"))); });
  expect_throws([] { diff(x, x + y); });
  expect_throws([] { evaluate(1 / x, {}); });
  expect_throws([] { evaluate(fem::sqrt(x), {-1}); });
  expect_throws([] { evaluate(fem::exp(x), {1000}); });
  expect_throws(
      [] { evaluate(x, {std::numeric_limits<double>::quiet_NaN()}); });
}
int main() {
  return pde2d_test::run({{"expression_semantics", expression_semantics}});
}

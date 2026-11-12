// The library's private derivative function (src/fem/derivative.hpp),
// tested directly: assembly will differentiate in x and y, which no public
// function does yet. Derivatives in t through MeshedProblem are tested in
// meshed_problem.cpp.

#include "derivative.hpp"

#include "pde2d/fem/compiled_expression.hpp"
#include "pde2d/fem/expression.hpp"
#include "support/check.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

using namespace pde2d::fem;
using detail::derivative;
using detail::Variable;
using pde2d::test::check;
using pde2d::test::expect_throws;
using pde2d::test::near;

namespace {

// Points for one argument inside the domain of the function under test
// (asin and acos need (-1, 1), log and sqrt positive arguments); 0 is
// left out where a derivative has a kink there.
constexpr double unit_points[] = {-0.9, -0.5, -0.1, 0.3, 0.7};
constexpr double positive_points[] = {1e-3, 0.25, 1.0, 2.5, 10.0};

// The sign as the derivative of abs uses it: 0 at 0.
double sign(double v) { return v > 0 ? 1.0 : v < 0 ? -1.0 : 0.0; }

CompiledExpression derivative_in_x(const char *text) {
  return CompiledExpression(derivative(Expression(text), Variable::x), {});
}

// d/dx of `text` against the exact derivative `expected` at `points`,
// within a few units in the last place.
template <class Points, class F>
void compare(const char *text, const Points &points, F expected) {
  const CompiledExpression d = derivative_in_x(text);
  for (const double x : points) {
    const double want = expected(x);
    near(d(x, 0.0, 0.0), want, 1e-15 * std::max(1.0, std::abs(want)), text);
  }
}

} // namespace

int main() {
  return pde2d::test::run({
      {"every allowed function: d/dx against its exact derivative",
       [] {
         compare("sin(x)", unit_points, [](double x) { return std::cos(x); });
         compare("cos(x)", unit_points, [](double x) { return -std::sin(x); });
         compare("tan(x)", unit_points, [](double x) {
           return 1.0 / (std::cos(x) * std::cos(x));
         });
         compare("asin(x)", unit_points,
                 [](double x) { return 1.0 / std::sqrt(1.0 - x * x); });
         compare("acos(x)", unit_points,
                 [](double x) { return -1.0 / std::sqrt(1.0 - x * x); });
         compare("atan(x)", unit_points,
                 [](double x) { return 1.0 / (1.0 + x * x); });
         compare("sinh(x)", unit_points, [](double x) { return std::cosh(x); });
         compare("cosh(x)", unit_points, [](double x) { return std::sinh(x); });
         compare("tanh(x)", unit_points, [](double x) {
           return 1.0 / (std::cosh(x) * std::cosh(x));
         });
         compare("exp(x)", unit_points, [](double x) { return std::exp(x); });
         compare("E^x", unit_points, [](double x) { return std::exp(x); });
         compare("log(x)", positive_points, [](double x) { return 1.0 / x; });
         compare("ln(x)", positive_points, [](double x) { return 1.0 / x; });
         compare("log(x, 3)", positive_points,
                 [](double x) { return 1.0 / (x * std::log(3.0)); });
         compare("sqrt(x)", positive_points,
                 [](double x) { return 0.5 / std::sqrt(x); });
         compare("abs(x)", unit_points, [](double x) { return sign(x); });
         compare("x^2.5", positive_points,
                 [](double x) { return 2.5 * std::pow(x, 1.5); });
         compare("x**3", unit_points, [](double x) { return 3 * x * x; });
         compare("2^x", unit_points,
                 [](double x) { return std::exp2(x) * std::log(2.0); });
         compare("x^x", positive_points, [](double x) {
           return std::pow(x, x) * (1.0 + std::log(x));
         });
         compare("1/x", positive_points, [](double x) { return -1.0 / (x * x); });
         compare("pi*x", unit_points,
                 [](double) { return 3.141592653589793; });
         compare("3 - 2*x + x/4", unit_points, [](double) { return -1.75; });
         compare("Piecewise((x^2, x < 0), (sin(x), True))", unit_points,
                 [](double x) { return x < 0 ? 2 * x : std::cos(x); });
         // A compound condition is kept as it is.
         compare("Piecewise((x^3, And(x > -0.6, Or(x < 0, x > 0.5))), "
                 "(-x, True))",
                 unit_points, [](double x) {
                   return x > -0.6 && (x < 0 || x > 0.5) ? 3 * x * x : -1.0;
                 });
       }},
      {"the chain rule, in x and y",
       [] {
         compare("sin(x^2)", unit_points,
                 [](double x) { return 2 * x * std::cos(x * x); });
         compare("exp(-x^2/2)", unit_points,
                 [](double x) { return -x * std::exp(-x * x / 2); });
         compare("log(1 + x^2)", unit_points,
                 [](double x) { return 2 * x / (1 + x * x); });
         compare("abs(sin(x))", unit_points,
                 [](double x) { return sign(std::sin(x)) * std::cos(x); });
         const CompiledExpression dy(
             derivative(Expression("x*y^2 + cos(y*t)"), Variable::y), {});
         // 2 x y - t sin(y t) at (3, 2, 0.5).
         near(dy(3.0, 2.0, 0.5), 12.0 - 0.5 * std::sin(1.0), 1e-15, "d/dy");
       }},
      {"abs: sign(u) du, zero at the kink",
       [] {
         const CompiledExpression d = derivative_in_x("abs(x)");
         check(d(-2.0, 0.0, 0.0) == -1.0 && d(0.0, 0.0, 0.0) == 0.0 &&
                   d(3.0, 0.0, 0.0) == 1.0,
               "-1, 0, 1");
         const Expression form = derivative(Expression("abs(x)"), Variable::x);
         check(form.text() == "Piecewise((1, 0 < x), (-1, x < 0), (0, True))",
               form.text());
         check(derivative(Expression("abs(t - 1)"), Variable::t).text() ==
                   "-Piecewise((1, 0 < 1 - t), (-1, 1 - t < 0), (0, True))",
               "d/dt abs(t - 1)");
         // The expression keeps abs; only the derivative has the sign.
         check(derivative(Expression("x*abs(x)"), Variable::x).text() ==
                   "x*Piecewise((1, 0 < x), (-1, x < 0), (0, True)) + abs(x)",
               "product rule");
         const CompiledExpression shifted = derivative_in_x("abs(2*x - 1)");
         check(shifted(0.0, 0.0, 0.0) == -2.0 &&
                   shifted(0.5, 0.0, 0.0) == 0.0 &&
                   shifted(1.0, 0.0, 0.0) == 2.0,
               "d/dx |2x - 1| = 2 sign(2x - 1)");
         // abs inside abs, and inside a Piecewise condition (not
         // differentiated: a condition has no derivative).
         // At x = -0.5 the inner value is 0: the kink, where it is 0.
         compare("abs(abs(x) - 0.5)", unit_points, [](double x) {
           return sign(std::abs(x) - 0.5) * sign(x);
         });
         compare("Piecewise((x^2, abs(x) < 0.4), (x, True))", unit_points,
                 [](double x) { return std::abs(x) < 0.4 ? 2 * x : 1.0; });
       }},
      {"second derivatives",
       [] {
         const Expression d1 = derivative(Expression("x*abs(x)"), Variable::x);
         const Expression d2 = derivative(d1, Variable::x);
         // d/dx (|x| + x sign x) = 2 sign x, the derivative of sign being 0.
         check(d2.text() == "2*Piecewise((1, 0 < x), (-1, x < 0), (0, True))",
               d2.text());
         const CompiledExpression f(
             derivative(derivative(Expression("sin(x)*exp(x)"), Variable::x),
                        Variable::x),
             {});
         for (const double x : unit_points)
           near(f(x, 0.0, 0.0), 2 * std::cos(x) * std::exp(x), 1e-15,
                "(sin x e^x)'' = 2 cos x e^x");
         // A Piecewise whose branches all have the same derivative is that
         // derivative.
         check(derivative(Expression("Piecewise((x, y < 0), (x + 1, True))"),
                          Variable::x)
                       .text() == "1",
               "equal branches");
       }},
      {"parameters are constants; facts are those of the derivative",
       [] {
         const Expression d = derivative(Expression("k*x^2 + y"), Variable::x);
         const CompiledExpression f(d, {{"k", 3.0}});
         check(f(2.0, 5.0, 0.0) == 12.0, "2 k x");
         check(!d.facts().depends_on_y && d.facts().depends_on_x, "facts");
         check(derivative(Expression("sin(y) + t"), Variable::x).facts().zero,
               "no x: zero");
       }},
      {"a derivative outside the allow-list throws",
       [] {
         expect_throws<std::invalid_argument>(
             [] { (void)derivative(Expression("1e308*x^2"), Variable::x); },
             "contains a number that is not finite in double precision");
       }},
  });
}

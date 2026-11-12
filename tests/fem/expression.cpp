#include "pde2d/fem/compiled_expression.hpp"
#include "pde2d/fem/expression.hpp"
#include "support/check.hpp"

#include <cmath>
#include <stdexcept>

using namespace pde2d::fem;
using pde2d::test::check;
using pde2d::test::expect_throws;
using pde2d::test::near;

namespace {

bool same(const ExpressionFacts &a, const ExpressionFacts &b) {
  return a.zero == b.zero && a.depends_on_x == b.depends_on_x &&
         a.depends_on_y == b.depends_on_y && a.depends_on_t == b.depends_on_t;
}

void check_facts(const char *text, ExpressionFacts expected) {
  check(same(Expression(text).facts(), expected), text);
}

} // namespace

int main() {
  return pde2d::test::run({
      {"parse errors throw when the expression is built",
       [] {
         for (const char *text : {"", "   ", "exp(", "x y", "1 +"})
           expect_throws<std::invalid_argument>([&] { Expression e(text); },
                                                "Cannot parse the expression");
         expect_throws<std::invalid_argument>([] { Expression e("x +* 2"); },
                                              "'x +* 2'");
       }},
      {"unknown functions, infinities and complex values are rejected",
       [] {
         expect_throws<std::invalid_argument>(
             [] { Expression e("1 + foo(x)"); }, "unknown function 'foo'");
         for (const char *text : {"oo", "1/0", "nan", "x + oo"})
           expect_throws<std::invalid_argument>([&] { Expression e(text); },
                                                "infinite or undefined value");
         for (const char *text : {"I", "sqrt(-1)", "x + I*y"})
           expect_throws<std::invalid_argument>([&] { Expression e(text); },
                                                "has no real numeric value");
       }},
      {"the text is kept; the default expression is zero",
       [] {
         check(Expression("1 + k*x*y").text() == "1 + k*x*y", "text");
         const Expression zero;
         check(zero.text() == "0", "default text");
         check(same(zero.facts(), {.zero = true}), "default facts");
         const Expression copy = Expression("sin(t)");
         Expression other = copy;
         check(other.text() == "sin(t)" && other.facts().depends_on_t, "copy");
       }},
      {"zero means zero as written",
       [] {
         check_facts("0", {.zero = true});
         check_facts("0.0", {.zero = true});
         check_facts("-0", {.zero = true});
         // SymEngine combines like terms when it parses: still as written.
         check_facts("x - x", {.zero = true});
         check_facts("0*x*t", {.zero = true});
         // Nothing beyond that: an identity is not recognized.
         check_facts("sin(x)^2 + cos(x)^2 - 1", {.depends_on_x = true});
         // A parameter is an unknown constant, whatever its value later.
         check_facts("k", {});
         check_facts("k*x", {.depends_on_x = true});
       }},
      {"dependence on x, y and t",
       [] {
         check_facts("1 + k*x*y", {.depends_on_x = true, .depends_on_y = true});
         check_facts("exp(-t)*y", {.depends_on_y = true, .depends_on_t = true});
         check_facts("2.5", {});
         check_facts("pi*x", {.depends_on_x = true});
         check_facts("Piecewise((1, t < 1), (0, True))",
                     {.depends_on_t = true});
       }},
      {"compiled values against hand-computed ones",
       [] {
         const CompiledExpression a(Expression("1 + k*x*y"), {{"k", 1000.0}});
         check(a(0.5, 0.25, 7.0) == 126.0, "1 + 1000 * 0.125");
         const CompiledExpression b(Expression("x^2 + y - 3*t"), {});
         check(b(3.0, 2.0, 1.0) == 8.0, "9 + 2 - 3");
         const CompiledExpression c(Expression("2^n * x"), {{"n", 10.0}});
         check(c(0.5, 0.0, 0.0) == 512.0, "2^10 / 2");
         const CompiledExpression d(Expression("sin(x) * exp(-t)"), {});
         near(d(0.3, 0.0, 2.0), std::sin(0.3) * std::exp(-2.0), 1e-15,
              "sin(x) exp(-t)");
         const CompiledExpression e(
             Expression("Piecewise((1, x < 0), (2, True))"), {});
         check(e(-1.0, 0.0, 0.0) == 1.0 && e(1.0, 0.0, 0.0) == 2.0,
               "piecewise");
         const CompiledExpression zero;
         check(zero(1.0, 2.0, 3.0) == 0.0 && zero.facts().zero, "zero");
         // A call does not check its result.
         const CompiledExpression f(Expression("1/x"), {});
         check(std::isinf(f(0.0, 0.0, 0.0)), "1/0 is an infinity");
       }},
      {"compiled facts are the facts as written",
       [] {
         const CompiledExpression a(Expression("k*x"), {{"k", 0.0}});
         check(same(a.facts(), {.depends_on_x = true}), "k*x with k = 0");
         check(a(5.0, 0.0, 0.0) == 0.0, "value 0");
       }},
      {"parameters must be known, named plainly and finite",
       [] {
         expect_throws<std::invalid_argument>(
             [] { CompiledExpression c(Expression("k*x"), {}); },
             "uses 'k', which is not a parameter");
         expect_throws<std::invalid_argument>(
             [] { CompiledExpression c(Expression("a*b"), {{"a", 1.0}}); },
             "uses 'b'");
         for (const char *name : {"x", "y", "t"})
           expect_throws<std::invalid_argument>(
               [&] { CompiledExpression c(Expression("1"), {{name, 1.0}}); },
               "has the name of a variable");
         for (const char *name : {"pi", "E", "2k", "a b", "a+b", ""})
           expect_throws<std::invalid_argument>(
               [&] { CompiledExpression c(Expression("1"), {{name, 1.0}}); },
               "is not a plain symbol name");
         expect_throws<std::invalid_argument>(
             [] { CompiledExpression c(Expression("k"), {{"k", INFINITY}}); },
             "Parameter 'k' is not finite");
         // Unused parameters are allowed.
         const CompiledExpression c(Expression("x"), {{"unused", 1.0}});
         check(c(2.0, 0.0, 0.0) == 2.0, "unused parameter");
       }},
      {"a parameter value without a real result is rejected",
       [] {
         expect_throws<std::invalid_argument>(
             [] {
               CompiledExpression c(Expression("x + log(k)"), {{"k", -1.0}});
             },
             "has no real numeric value with the given parameters");
       }},
  });
}

#include "pde2d/fem/compiled_expression.hpp"
#include "pde2d/fem/expression.hpp"
#include "support/check.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

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

CompiledExpression compiled(const char *text) {
  return CompiledExpression(Expression(text), {});
}

// Sample points for one argument, inside the domain of every function used
// with them (asin and acos need [-1, 1]; log and sqrt get their own).
constexpr double unit_points[] = {-0.9, -0.5, -0.1, 0.0, 0.3, 0.7, 1.0};
constexpr double positive_points[] = {1e-3, 0.25, 1.0, 2.5, 10.0, 1e6};

// `text` uses x; compares it with `expected` at `points`, within a few
// units in the last place (SymEngine computes some functions through pow,
// sqrt(x) as pow(x, 0.5) for example).
template <class Points, class F>
void compare(const char *text, const Points &points, F expected) {
  const CompiledExpression f = compiled(text);
  for (const double x : points) {
    const double want = expected(x);
    near(f(x, 0.0, 0.0), want, 4e-16 * std::max(1.0, std::abs(want)), text);
  }
}

// A condition evaluated through Piecewise((1, condition), (0, True)).
double truth(const CompiledExpression &f, bool x, bool y, bool t) {
  return f(x ? 1.0 : -1.0, y ? 1.0 : -1.0, t ? 1.0 : -1.0);
}

// Checks a condition in x, y and t (each true when the variable is
// positive) against `expected` for all eight combinations.
template <class F> void truth_table(const char *condition, F expected) {
  const std::string text =
      std::string("Piecewise((1, ") + condition + "), (0, True))";
  const CompiledExpression f = compiled(text.c_str());
  for (const bool x : {false, true})
    for (const bool y : {false, true})
      for (const bool t : {false, true})
        check(truth(f, x, y, t) == (expected(x, y, t) ? 1.0 : 0.0), condition);
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
             "with the given parameters has no real numeric value");
       }},
      {"a Piecewise must end with an unconditional branch",
       [] {
         // Without (value, True) at the end SymEngine's evaluator reads
         // past its branches where no condition holds.
         for (const char *text :
              {"Piecewise((1, x < 0))", "Piecewise((1, x < 0), (2, y < 0))",
               // SymEngine drops a False branch, leaving none.
               "Piecewise((1, x < 0), (2, False))",
               // Checked inside other expressions and other Piecewise too.
               "1 + sin(Piecewise((1, x < 0)))",
               "Piecewise((1, x < 0), (Piecewise((2, y < 0)), True))",
               "Piecewise((Piecewise((2, y < 0)), x < 0), (1, True))"})
           expect_throws<std::invalid_argument>(
               [&] { Expression e(text); },
               "has a Piecewise without a final unconditional branch "
               "(value, True)");
         const CompiledExpression f =
             compiled("Piecewise((1, x < 0), (Piecewise((2, y < 0), (3, "
                      "True)), True))");
         check(f(-1.0, 0.0, 0.0) == 1.0 && f(0.0, -1.0, 0.0) == 2.0 &&
                   f(0.0, 0.0, 0.0) == 3.0,
               "nested Piecewise");
       }},
      {"functions and operators outside the allow-list are rejected",
       [] {
         // loggamma writes a global variable (not thread-safe); SymEngine
         // evaluates Xor wrongly; the others are simply not checked yet.
         const std::pair<const char *, const char *> cases[] = {
             {"loggamma(x)", "'loggamma'"},
             {"Piecewise((1, Xor(x < 0, y < 0)), (0, True))", "'Xor'"},
             {"gamma(x)", "'gamma'"},
             {"erf(x)", "'erf'"},
             {"erfc(x)", "'erfc'"},
             {"max(x, y)", "'max'"},
             {"min(x, y)", "'min'"},
             {"sign(x)", "'sign'"},
             {"floor(x)", "'floor'"},
             {"ceiling(x)", "'ceiling'"},
             {"cot(x)", "'cot'"},
             {"sec(x)", "'sec'"},
             {"csc(x)", "'csc'"},
             {"acot(x)", "'acot'"},
             {"asinh(x)", "'asinh'"},
             {"acosh(x)", "'acosh'"},
             {"atanh(x)", "'atanh'"},
             {"coth(x)", "'coth'"},
             {"lambertw(x)", "'lambertw'"},
             {"x*EulerGamma", "'EulerGamma'"},
             {"GoldenRatio", "'GoldenRatio'"},
             {"Catalan + x", "'Catalan'"}};
         for (const auto &[text, name] : cases)
           expect_throws<std::invalid_argument>(
               [&] { Expression e(text); },
               std::string("uses ") + name +
                   ", which is not among the supported functions and "
                   "operators");
         // Several unsupported constructs: all are named, sorted.
         expect_throws<std::invalid_argument>(
             [] { Expression e("max(x, 1) + loggamma(y)"); },
             "uses 'loggamma' and 'max', which are not among the supported");
         // Unknown names keep their own message.
         expect_throws<std::invalid_argument>([] { Expression e("Abs(x)"); },
                                              "unknown function 'Abs'");
       }},
      {"numbers that are not finite as doubles are rejected",
       [] {
         for (const char *text :
              {"1e1000", "x + 1e1000", "-1e1000*y", "1e308*10", "10^400",
               "x^(10^400)", "10^400/3 + t", "2.0^5000",
               "Piecewise((1, x < 1e1000), (0, True))",
               "Piecewise((1e1000, x < 0), (0, True))"})
           expect_throws<std::invalid_argument>(
               [&] { Expression e(text); },
               "contains a number that is not finite in double precision");
         // Tiny numbers become zero or stay finite: accepted.
         check(compiled("x + 1e-400")(2.0, 0.0, 0.0) == 2.0, "1e-400");
         check(compiled("x/10^400")(1.0, 0.0, 0.0) == 0.0, "1/10^400");
       }},
      {"parameter values that make a number infinite are rejected",
       [] {
         const std::pair<const char *, double> cases[] = {
             {"1/k", 0.0},      {"x + 1/k", 0.0},
             {"x/k", 0.0},      {"Piecewise((1/k, x < 0), (0, True))", 0.0},
             {"10*k*x", 1e308}, {"log(k)", 0.0}};
         for (const auto &[text, k] : cases)
           expect_throws<std::invalid_argument>(
               [&] { CompiledExpression c(Expression(text), {{"k", k}}); },
               "with the given parameters contains a number that is not "
               "finite in double precision");
         // A value that is infinite only at some points is a call result,
         // not checked (as for 1/x at x = 0).
         const CompiledExpression ok(Expression("1/(x - k)"), {{"k", 1.0}});
         check(std::isinf(ok(1.0, 0.0, 0.0)), "1/(x - k) at x = k");
       }},
      {"a condition is not a number",
       [] {
         for (const char *text : {"x < 0", "2*(x < 0)", "True", "1 + (x >= y)",
                                  "sin(And(x < 0, y < 0))"})
           expect_throws<std::invalid_argument>(
               [&] { Expression e(text); },
               "as a number; a condition may appear only in a Piecewise");
       }},
      {"the allowed functions against the standard library",
       [] {
         compare("sin(x)", unit_points, [](double x) { return std::sin(x); });
         compare("cos(x)", unit_points, [](double x) { return std::cos(x); });
         compare("tan(x)", unit_points, [](double x) { return std::tan(x); });
         compare("asin(x)", unit_points, [](double x) { return std::asin(x); });
         compare("acos(x)", unit_points, [](double x) { return std::acos(x); });
         compare("atan(x)", unit_points, [](double x) { return std::atan(x); });
         compare("sinh(x)", unit_points, [](double x) { return std::sinh(x); });
         compare("cosh(x)", unit_points, [](double x) { return std::cosh(x); });
         compare("tanh(x)", unit_points, [](double x) { return std::tanh(x); });
         compare("exp(x)", unit_points, [](double x) { return std::exp(x); });
         compare("abs(x)", unit_points, [](double x) { return std::abs(x); });
         compare("log(x)", positive_points,
                 [](double x) { return std::log(x); });
         compare("ln(x)", positive_points,
                 [](double x) { return std::log(x); });
         compare("sqrt(x)", positive_points,
                 [](double x) { return std::sqrt(x); });
         compare("x^2.5", positive_points,
                 [](double x) { return std::pow(x, 2.5); });
         compare("x**3", unit_points,
                 [](double x) { return std::pow(x, 3.0); });
         compare("2^x", unit_points, [](double x) { return std::exp2(x); });
         compare("1/x", positive_points, [](double x) { return 1.0 / x; });
         compare("pi*x", unit_points,
                 [](double x) { return 3.141592653589793 * x; });
         compare("E^x", unit_points, [](double x) { return std::exp(x); });
         // Every operator at once, in x, y and t.
         const CompiledExpression all = compiled("(x - 2*y)/(t + 3) * x^2");
         for (const double x : unit_points)
           for (const double y : positive_points) {
             const double t = 0.5 * x;
             const double want = (x - 2 * y) / (t + 3) * x * x;
             near(all(x, y, t), want, 1e-15 * std::max(1.0, std::abs(want)),
                  "(x - 2y)/(t + 3) x^2");
           }
       }},
      {"atan2(y, x) takes y first, like std::atan2",
       [] {
         const CompiledExpression f = compiled("atan2(y, x)");
         for (const double x : {-2.0, -0.5, 0.0, 0.5, 2.0})
           for (const double y : {-1.5, -0.25, 0.0, 0.25, 1.5})
             check(f(x, y, 0.0) == std::atan2(y, x), "atan2");
         check(compiled("atan2(1, 2)")(0.0, 0.0, 0.0) == std::atan2(1.0, 2.0),
               "constant arguments");
       }},
      {"comparisons and logic: full truth tables",
       [] {
         // Comparisons of x and y over values below, at and above.
         const std::pair<const char *, bool (*)(double, double)> comparisons[] =
             {{"x < y", [](double a, double b) { return a < b; }},
              {"x <= y", [](double a, double b) { return a <= b; }},
              {"x > y", [](double a, double b) { return a > b; }},
              {"x >= y", [](double a, double b) { return a >= b; }},
              {"x == y", [](double a, double b) { return a == b; }},
              {"Eq(x, y)", [](double a, double b) { return a == b; }},
              {"x != y", [](double a, double b) { return a != b; }},
              {"Ne(x, y)", [](double a, double b) { return a != b; }}};
         for (const auto &[condition, expected] : comparisons) {
           const std::string text =
               std::string("Piecewise((1, ") + condition + "), (0, True))";
           const CompiledExpression f = compiled(text.c_str());
           for (const double a : {-1.0, 0.0, 2.0})
             for (const double b : {-1.0, 0.0, 2.0})
               check(f(a, b, 0.0) == (expected(a, b) ? 1.0 : 0.0), condition);
           // NaN compares false, as in C++, and the final branch is taken.
           check(f(NAN, 0.0, 0.0) == (expected(NAN, 0.0) ? 1.0 : 0.0),
                 condition);
         }
         truth_table("And(x > 0, y > 0)",
                     [](bool x, bool y, bool) { return x && y; });
         truth_table("(x > 0) & (y > 0) & (t > 0)",
                     [](bool x, bool y, bool t) { return x && y && t; });
         truth_table("Or(x > 0, y > 0)",
                     [](bool x, bool y, bool) { return x || y; });
         truth_table("(x > 0) | (y > 0) | (t > 0)",
                     [](bool x, bool y, bool t) { return x || y || t; });
         truth_table("Not(x > 0)", [](bool x, bool, bool) { return !x; });
         truth_table("~(x > 0)", [](bool x, bool, bool) { return !x; });
         truth_table("Not(And(x > 0, y > 0))",
                     [](bool x, bool y, bool) { return !(x && y); });
         truth_table("Not(Or(x > 0, y > 0))",
                     [](bool x, bool y, bool) { return !(x || y); });
         truth_table("Or(And(x > 0, Not(y > 0)), And(Not(x > 0), y > 0))",
                     [](bool x, bool y, bool) { return x != y; });
         truth_table("And(Or(x > 0, t > 0), Not(y > 0))",
                     [](bool x, bool y, bool t) { return (x || t) && !y; });
       }},
  });
}

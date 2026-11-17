#include "pde2d/fem/compiled_expression.hpp"
#include "pde2d/fem/equation_text.hpp"
#include "support/check.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

using namespace pde2d::fem;
using pde2d::test::check;
using pde2d::test::expect_throws;
using pde2d::test::near;

namespace {

const Parameters parameters{{"k", 2.5},  {"k1", 3.0},    {"k2", 0.5},
                            {"h", 7.0},  {"u_out", 20.0}, {"c", 1.5},
                            {"q", -4.0}, {"n", 3.0}};

// The coefficient equals `expected` (an independent formula, written by
// hand) at a few points; a zero is zero as written.
void same(const Expression &actual, const std::string &expected,
          const std::string &what) {
  const Expression wanted(expected);
  if (wanted.facts().zero) {
    check(actual.facts().zero, what + " is zero (got " + actual.text() + ")");
    return;
  }
  const CompiledExpression a(actual, parameters), b(wanted, parameters);
  constexpr std::array<std::array<double, 3>, 4> points{
      {{0.3, -0.7, 0.0}, {1.1, 0.4, 0.25}, {-0.6, 2.0, 1.5}, {0.0, 0.5, 3.0}}};
  for (const auto &[x, y, t] : points)
    near(a(x, y, t), b(x, y, t), 1e-12 * (1 + std::abs(b(x, y, t))),
         what + " (" + actual.text() + ")");
}

struct Wanted {
  std::string a_xx = "0", a_xy = "0", a_yy = "0", b_x = "0", b_y = "0",
              c = "0", m1 = "0", m2 = "0", f = "0";
};

void equation_is(const Equation &e, const Wanted &w) {
  same(e.a_xx, w.a_xx, "a_xx");
  same(e.a_xy, w.a_xy, "a_xy");
  same(e.a_yy, w.a_yy, "a_yy");
  same(e.b_x, w.b_x, "b_x");
  same(e.b_y, w.b_y, "b_y");
  same(e.c, w.c, "c");
  same(e.m1, w.m1, "m1");
  same(e.m2, w.m2, "m2");
  same(e.f, w.f, "f");
}

struct WantedCondition {
  std::string alpha = "0", beta_n = "0", beta_tau = "0", beta_x = "0",
              beta_y = "0", gamma = "0", g = "0";
};

void condition_is(const BoundaryCondition &b, const WantedCondition &w) {
  same(b.alpha, w.alpha, "alpha");
  same(b.beta_n, w.beta_n, "beta_n");
  same(b.beta_tau, w.beta_tau, "beta_tau");
  same(b.beta_x, w.beta_x, "beta_x");
  same(b.beta_y, w.beta_y, "beta_y");
  same(b.gamma, w.gamma, "gamma");
  same(b.g, w.g, "g");
  check(b.priority == 0, "priority 0");
}

void refused(const char *text, const char *fragment, bool condition = false) {
  if (condition)
    expect_throws<std::invalid_argument>([&] { parse_condition(text); },
                                         fragment);
  else
    expect_throws<std::invalid_argument>([&] { parse_equation(text); },
                                         fragment);
}

} // namespace

int main() {
  return pde2d::test::run({
      {"the coefficient form as written",
       [] {
         equation_is(parse_equation("-u_xx - u_yy = 1"),
                     {.a_xx = "-1", .a_yy = "-1", .f = "1"});
         equation_is(
             parse_equation("2*u_xx + 3*u_xy + u_yx + 4*u_yy + x*u_x - y*u_y "
                            "+ c*u + u_t + 0.5*u_tt = sin(x)"),
             {.a_xx = "2",
              .a_xy = "4",
              .a_yy = "4",
              .b_x = "x",
              .b_y = "-y",
              .c = "c",
              .m1 = "1",
              .m2 = "0.5",
              .f = "sin(x)"});
       }},
      {"terms move between the sides",
       [] {
         equation_is(parse_equation("u_t = u_xx + u_yy + x*y"),
                     {.a_xx = "-1", .a_yy = "-1", .m1 = "1", .f = "x*y"});
         equation_is(parse_equation("3 + u = 2*u + 5"), {.c = "-1", .f = "2"});
         equation_is(parse_equation("(1 + x)*(u_x + 2) = 0"),
                     {.b_x = "1 + x", .f = "-2*(1 + x)"});
       }},
      {"the Laplacian by name and with nabla",
       [] {
         const Wanted laplace{.a_xx = "-1", .a_yy = "-1", .f = "1"};
         equation_is(parse_equation("-lap(u) = 1"), laplace);
         equation_is(parse_equation("-div(grad(u)) = 1"), laplace);
         equation_is(parse_equation("-Δu = 1"), laplace);        // Δ
         equation_is(parse_equation("-∇²u = 1"), laplace);  // ∇²
         equation_is(parse_equation("-∇·∇u = 1"), laplace);
         equation_is(parse_equation("-∇⋅(∇u) = 1"), laplace);
       }},
      {"the divergence form takes the derivatives of the coefficients",
       [] {
         // -div((1 + x^2 y) grad u): a = -(1 + x^2 y), b = -grad(1 + x^2 y).
         const Wanted wanted{.a_xx = "-(1 + x^2*y)",
                             .a_yy = "-(1 + x^2*y)",
                             .b_x = "-2*x*y",
                             .b_y = "-x^2",
                             .f = "1"};
         equation_is(parse_equation("-div((1 + x^2*y)*grad(u)) = 1"), wanted);
         equation_is(parse_equation("-∇·((1 + x^2*y)∇u) = 1"),
                     wanted);
         // A constant parameter has no derivative.
         equation_is(parse_equation("u_t - ∇·(k ∇u) = 0"),
                     {.a_xx = "-k", .a_yy = "-k", .m1 = "1"});
       }},
      {"a matrix coefficient and convection",
       [] {
         equation_is(
             parse_equation("-∇·([[k1, 0], [0, k2]] ∇u) + "
                            "(1, 0)·∇u = 1"),
             {.a_xx = "-k1", .a_yy = "-k2", .b_x = "1", .f = "1"});
         // div(A grad u) with A = [[x, y], [2, 3]]: a_xy = A_12 + A_21 and
         // b = div of A's columns.
         equation_is(parse_equation("div([[x, y], [2, 3]]*grad(u)) = 0"),
                     {.a_xx = "x",
                      .a_xy = "y + 2",
                      .a_yy = "3",
                      .b_x = "1",
                      .b_y = "0"});
         equation_is(parse_equation("dot((x, y), grad(u)) + u = 0"),
                     {.b_x = "x", .b_y = "y", .c = "1"});
         equation_is(parse_equation("div((x*u, y*u)) = 0"),
                     {.b_x = "x", .b_y = "y", .c = "2"});
       }},
      {"coefficients keep Expression's syntax",
       [] {
         equation_is(parse_equation("-Piecewise((k, x < 0), (1, True))*lap(u)"
                                    " = Piecewise((1, x <= 0.5), (0, True))"),
                     {.a_xx = "-Piecewise((k, x < 0), (1, True))",
                      .a_yy = "-Piecewise((k, x < 0), (1, True))",
                      .f = "Piecewise((1, x <= 0.5), (0, True))"});
         equation_is(parse_equation("2**2*u = x^2.0"),
                     {.c = "4", .f = "x^2"});
         // n is a parameter in the equation.
         equation_is(parse_equation("n*u = 1"), {.c = "n", .f = "1"});
         equation_is(parse_equation("u/k = 1"), {.c = "1/k", .f = "1"});
       }},
      {"what is not linear or not of the form is refused",
       [] {
         refused("u*u_x = 0", "'u*u_x' multiplies u by u");
         refused("sin(u) = 0", "applies 'sin' to 'u'");
         refused("1/u = 1", "divides by u or the normal");
         refused("u^2 = 1", "raises u");
         refused("div(grad(u_x)) = 0", "third derivative");
         refused("u_xxy = 0", "third derivative");
         refused("grad(u)*(1, 1) = 0", "multiplies two vectors");
         refused("dot(grad(u_t), (1, 1)) = 0", "space derivative of u_t");
         refused("u_xt = 0", "mixed derivative in space and time");
         refused("u_n = 0", "belongs in a boundary condition");
         refused("div(u) = 0", "takes the divergence of a number");
         refused("grad(u) = 0", "a vector on the left side");
         refused("(1, 2) + u = 0", "adds a number and a vector");
         refused("x = 1", "has no term with u");
         refused("u + 1", "has no '='");
         refused("u = 1 = 2", "more than one '='");
         refused("2u = 1", "a product needs '*'");
         refused("u + = 1", "where a number, a name, u or '(' is expected");
         refused("sin(x = 1", "not closed");
         refused("(1, 2, 3)*grad(u) = 0", "has 3 entries");
         refused("u = loggamma(x)", "not among the supported functions");
         refused("u = 1/0", "infinite or undefined");
         refused("u/(x - x) = 1", "divides by zero");
       }},
      {"messages name the line",
       [] {
         refused("u*u = 0", "Cannot read the equation 'u*u = 0': ");
         refused("u_xx = 0", "Cannot read the boundary condition 'u_xx = 0'",
                 true);
       }},
      {"conditions",
       [] {
         condition_is(parse_condition("u = 0"), {.alpha = "1"});
         condition_is(parse_condition("u = 300 + t"),
                      {.alpha = "1", .g = "300 + t"});
         condition_is(parse_condition("k*u_n + h*u = h*u_out"),
                      {.alpha = "h", .beta_n = "k", .g = "h*u_out"});
         condition_is(parse_condition("(k ∇u)·n = q"),
                      {.beta_n = "k", .g = "q"});
         condition_is(parse_condition("dot(n, grad(u)) + 2*u_tau = 0"),
                      {.beta_n = "1", .beta_tau = "2"});
         condition_is(parse_condition("tau·∇u + u_x = y"),
                      {.beta_tau = "1", .beta_x = "1", .g = "y"});
         condition_is(parse_condition("u_t + u_n = 0"),
                      {.beta_n = "1", .gamma = "1"});
         // A multiple of the identity plus a rotation: (A ∇u)·n = k u_n + u_tau.
         condition_is(parse_condition("([[k, 1], [-1, k]] ∇u)·n = 0"),
                      {.beta_n = "k", .beta_tau = "1"});
         condition_is(parse_condition("u_y + x*u_x = 0"),
                      {.beta_x = "x", .beta_y = "1"});
       }},
      {"conditions that are refused",
       [] {
         refused("u_xx = 0", "second derivative", true);
         refused("u_tt + u = 0", "second derivative", true);
         refused("dot(n, (u, 0)) = 0", "outside a derivative", true);
         refused("dot((1, 0), n) + u = 0", "outside a derivative", true);
         refused("([[k1, 0], [0, k2]] ∇u)·n = 0",
                 "not beta_n u_n + beta_tau u_tau", true);
         refused("dot(n, n)*u = 0", "by itself", true);
         refused("div(n) + u = 0", "differentiates the normal", true);
         refused("1 = 2", "no term with u", true);
       }},
      {"printed back",
       [] {
         const auto e1 = parse_equation("u_t - div((1 + x)*grad(u)) = sin(x)");
         const auto text = equation_text(e1);
         const auto again = parse_equation(text);
         equation_is(again, {.a_xx = "-(1 + x)",
                             .a_yy = "-(1 + x)",
                             .b_x = "-1",
                             .m1 = "1",
                             .f = "sin(x)"});
         check(equation_text(Equation{.f = Expression("1")}) == "0 = 1",
               "an empty left side");
         check(equation_text({.a_xx = Expression("-1"),
                              .a_yy = Expression("-1"),
                              .f = Expression("1")}) == "-u_xx - u_yy = 1",
               "the Laplacian: " +
                   equation_text({.a_xx = Expression("-1"),
                                  .a_yy = Expression("-1"),
                                  .f = Expression("1")}));
         check(equation_text({.c = Expression("-2*k"), .m1 = Expression("x"),
                              .f = Expression("0")}) == "-2*k*u + x*u_t = 0",
               "signs and products");
         const auto c1 = parse_condition("k*u_n + h*u - u_tau = h*u_out");
         const auto c2 = parse_condition(condition_text(c1));
         condition_is(c2, {.alpha = "h",
                           .beta_n = "k",
                           .beta_tau = "-1",
                           .g = "h*u_out"});
         // A coefficient that is a sum gets parentheses.
         const Equation sum{.b_x = Expression("1 + x"), .f = Expression("0")};
         equation_is(parse_equation(equation_text(sum)), {.b_x = "1 + x"});
         // Decimal numbers read back to the same values.
         const Equation decimals{.c = Expression("0.1"),
                                 .f = Expression("1e-7*x")};
         equation_is(parse_equation(equation_text(decimals)),
                     {.c = "0.1", .f = "1e-7*x"});
       }},
  });
}

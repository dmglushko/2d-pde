#pragma once

// Private: the one place where the library takes derivatives of
// expressions (docs/decisions.md, "Every derivative is taken in one
// place"). The interface does not depend on SymEngine.

#include "pde2d/fem/expression.hpp"

#include <array>

namespace pde2d::fem::detail {

enum class Variable { x, y, t };

// The derivative of `expression` with respect to `variable`, taken
// symbolically; parameters are constants. Every construct of the
// allow-list (expression.hpp) has a derivative inside the allow-list:
// - abs is differentiated here, since SymEngine cannot:
//   d|u| = sign(u) du, with sign(u) written as
//   Piecewise((1, 0 < u), (-1, u < 0), (0, True)), so 0 at the kink;
// - a Piecewise is differentiated branch by branch, its conditions kept
//   (a jump between branches has no derivative here); when every branch
//   gives the same derivative, the result is that derivative;
// - everything else follows SymEngine's rules.
// The result is checked like an expression built from text (its text is
// SymEngine's printed form) and throws std::invalid_argument like
// Expression(text), for example when a number becomes infinite
// (d/dx 1e308*x^2).
Expression derivative(const Expression &expression, Variable variable);

// d/dt (numerator / denominator), through derivative(). Throws like it.
Expression time_derivative_of_quotient(const Expression &numerator,
                                       const Expression &denominator);

// The divergence of the matrix A = [[a_xx, a_xy / 2], [a_xy / 2, a_yy]]
// of the second-order part: {d/dx a_xx + d/dy a_xy / 2,
// d/dx a_xy / 2 + d/dy a_yy}, through derivative(). Throws like it.
std::array<Expression, 2> divergence_of_a(const Expression &a_xx,
                                          const Expression &a_xy,
                                          const Expression &a_yy);

} // namespace pde2d::fem::detail

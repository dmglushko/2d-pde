#pragma once

#include "pde2d/fem/problem.hpp"

#include <string>
#include <string_view>

namespace pde2d::fem {

// The equation and the boundary conditions written as one line each, the
// way they are written on paper, read into the coefficients of Equation
// and BoundaryCondition (problem.hpp). Reading only expands the line:
// nothing downstream knows it was written as text.
//
// A line is `left = right`; terms with u may stand on either side. It is
// built from:
// - the unknown and its derivatives: u, u_x, u_y, u_xx, u_xy (or u_yx),
//   u_yy, u_t, u_tt;
// - coefficients: any expression Expression accepts (expression.hpp: x,
//   y, t, parameters, pi, E, the functions, Piecewise), combined with +,
//   -, *, / and ^ (or **);
// - vectors (a, b) or [a, b] and matrices [[a, b], [c, d]] (or ((a, b),
//   (c, d))), whose entries are such lines too;
// - operators by name: grad(s), div(v), lap(s) (the Laplacian) and
//   dot(v, w);
// - the same with nabla: ∇s, ∇·v (or ∇⋅v), ∇²s and Δs; a·b (or a⋅b) is
//   the dot product of two vectors and the product otherwise, and a
//   factor directly in front of ∇ or Δ multiplies it (k∇u, A ∇u).
// For example "u_t - div(k*grad(u)) = 1",
// "-∇·([[k1, 0], [0, k2]] ∇u) + (1, 0)·∇u = f", "-Δu + c*u = 0".
//
// Derivatives of the coefficients are taken symbolically (by the
// library's one derivative function): -div(k*grad(u)) with k = 1 + x
// gives a_xx = a_yy = -(1 + x), b_x = -1. A product of two matrices or of
// a matrix and a vector is the matrix product; a_xy is the coefficient of
// u_xy, so div(A grad u) gives a_xy = A_12 + A_21.
//
// Refused with std::invalid_argument, the message quoting the part that
// is wrong: a line that cannot be read, not exactly one '=', a part that
// is not linear in u (u*u_x, sin(u), 1/u, u^2), a third derivative of u,
// a space derivative of u_t or u_tt, a vector or matrix where a number is
// needed (or the reverse), a line without u, and an expression
// Expression refuses.

// a_xx u_xx + a_xy u_xy + a_yy u_yy + b_x u_x + b_y u_y + c u + m1 u_t
// + m2 u_tt = f, read from one line. Coefficients that come out zero are
// left unset (zero).
Equation parse_equation(std::string_view text);

// alpha u + beta_n u_n + beta_tau u_tau + beta_x u_x + beta_y u_y
// + gamma u_t = g, read from one line, with priority 0. Besides the
// above, a condition may use the outward normal n and the tangent tau (n
// turned counterclockwise by 90 degrees) as vectors, and u_n (n·∇u) and
// u_tau (tau·∇u); in a condition, n and tau are therefore not parameter
// names. For example "u = 0", "k*u_n + h*u = h*u_out",
// "(k ∇u)·n = q". Refused besides: a second derivative or u_tt, n or tau
// outside a derivative of u (n_x u), and derivatives that are not of the
// form beta_n u_n + beta_tau u_tau + beta_x u_x + beta_y u_y (the
// anisotropic flux (A ∇u)·n, whose coefficients would depend on n).
// Whether the condition is supported is classify()'s question.
BoundaryCondition parse_condition(std::string_view text);

// The equation as one line in the coefficient form, terms in the order
// above, zero ones left out, for example "-u_xx - u_yy + u_t = 1";
// "0 = f" when every coefficient on the left is zero. The coefficients
// are printed from their parsed form, powers with ^, so the text may
// differ from what was written (1 + x for x+1) but parse_equation reads it
// back to the same coefficients.
std::string equation_text(const Equation &equation);

// The condition as one line ("u_n + 2*u = 0"), like equation_text, without
// the priority.
std::string condition_text(const BoundaryCondition &condition);

} // namespace pde2d::fem

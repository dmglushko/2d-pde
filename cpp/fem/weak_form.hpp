#pragma once
// Interior weak-form coefficients of the scalar equation in problem.hpp.
//
// With A = [[a_xx, a_xy/2], [a_xy/2, a_yy]], the second-order part satisfies
//   a_xx u_xx + a_xy u_xy + a_yy u_yy = div(A grad u) - (div A) . grad u,
// where (div A)_j = sum_i d_i A_ij. Multiplying by a test function phi and
// integrating by parts gives the interior bilinear form
//   -int (A grad u).grad phi + int (b - div A).grad u phi + int c u phi
// and the boundary term  int (n.A grad u) phi = -int q_n phi, which boundary
// conditions supply (see boundary_classification.hpp).
//
// The weak form is linear in the nine coefficients below, which is what lets
// assembly cache every time-independent contribution separately.
#include "problem.hpp"
#include <array>
#include <cstddef>
#include <stdexcept>

namespace pde2d::fem {
namespace weak {
// Indices into WeakCoefficients. Order is part of no public format, but the
// local kernels in p1_element.hpp use these names, never raw numbers.
enum Term : std::size_t {
  DiffusionXX, // A_xx = a_xx
  DiffusionXY, // A_xy = A_yx = a_xy / 2
  DiffusionYY, // A_yy = a_yy
  DriftX,      // b_x - (div A)_x
  DriftY,      // b_y - (div A)_y
  Reaction,    // c
  Source,      // right-hand side f
  FirstMass,   // m1 (multiplies u_t)
  SecondMass,  // m2 (multiplies u_tt)
  Count
};
} // namespace weak

template <class T> using WeakCoefficients = std::array<T, weak::Count>;
using WeakFlags = WeakCoefficients<bool>;

// Interior coefficients cannot reference boundary normals.
inline void validate_interior_equation(const Equation &eq) {
  for (const auto *e : {&eq.a_xx, &eq.a_xy, &eq.a_yy, &eq.b_x, &eq.b_y, &eq.c,
                        &eq.source, &eq.m1, &eq.m2})
    require_symbols(*e, field_symbols, "Interior coefficients");
}

inline WeakCoefficients<Expr> weak_form_coefficients(const Equation &eq) {
  return {eq.a_xx,
          eq.a_xy / 2,
          eq.a_yy,
          eq.b_x - diff(eq.a_xx, x) - diff(eq.a_xy, y) / 2,
          eq.b_y - diff(eq.a_yy, y) - diff(eq.a_xy, x) / 2,
          eq.c,
          eq.source,
          eq.m1,
          eq.m2};
}

// Terms whose expression contains t. Conservative (t may cancel
// numerically); assembly caches every other term.
inline WeakFlags weak_time_dependence(const WeakCoefficients<Expr> &terms) {
  WeakFlags flags{};
  for (std::size_t k = 0; k < weak::Count; ++k)
    flags[k] = depends_on(terms[k], t);
  return flags;
}
} // namespace pde2d::fem

#pragma once

// Private: the warning of the direct solvers that measure a pivot ratio
// (SimplicialLdlt, SupernodalCholesky, MultifrontalLu; docs/decisions.md,
// "A tiny pivot ratio is a warning; the integrator reports the worst
// solve"). No heavy dependency: the three solvers' .cpp files include it.

#include <format>
#include <string>

namespace pde2d::solve::detail {

// A measured pivot ratio (LinearSolveResult::pivot_ratio) below this one
// gives a warning: the matrix is nearly singular, so the answer may have
// few correct digits. Fixed by the owner's decision, not an option.
inline constexpr double small_pivot_ratio = 1e-10;

// The warning for a factorization whose pivot ratio is `pivot_ratio`,
// naming `solver`; empty when the ratio is not below small_pivot_ratio or
// not measured (0). The ratio is a property of the factorization, so a
// prepared solver computes this once and gives the same text on every
// solve.
inline std::string pivot_ratio_warning(const std::string &solver,
                                       double pivot_ratio) {
  if (!(pivot_ratio > 0.0 && pivot_ratio < small_pivot_ratio))
    return {};
  return std::format(
      "{}: the matrix is nearly singular (pivot ratio {:.3g}, below {:g}), "
      "so the answer may have few correct digits; for example a problem "
      "without a Dirichlet condition whose Robin or reaction term is weak "
      "or absent",
      solver, pivot_ratio, small_pivot_ratio);
}

} // namespace pde2d::solve::detail

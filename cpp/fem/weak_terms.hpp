#pragma once
// Names of the interior weak-form terms (weak_form.hpp derives their
// expressions). Kept apart so contract headers that only report which terms
// depend on time (analysis_types.hpp) need no symbolic types.
#include <array>
#include <cstddef>

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
} // namespace pde2d::fem

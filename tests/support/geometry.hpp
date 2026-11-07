#pragma once
#include "discretization/geometry_base.hpp"
#include "support/check.hpp"
#include <cmath>

namespace pde2d_test {

inline double norm(pde2d::discretization::Point2 p) {
  return std::hypot(p.x, p.y);
}
inline double distance(pde2d::discretization::Point2 a,
                       pde2d::discretization::Point2 b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

// |a - b| <= tol * max(1, |a|, |b|) in the Euclidean norm.
inline bool points_near(pde2d::discretization::Point2 a,
                        pde2d::discretization::Point2 b, double tol = 1e-9) {
  return distance(a, b) <= tol * std::max({1., norm(a), norm(b)});
}

} // namespace pde2d_test

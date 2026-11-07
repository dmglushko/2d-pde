#pragma once
#include "support/check.hpp"
#include <Eigen/Dense>

namespace pde2d_test {

// ||a - b|| <= tol * max(1, ||b||) using the overflow-safe stable norm.
template <class A, class B>
void near_vector(const Eigen::MatrixBase<A> &a, const Eigen::MatrixBase<B> &b,
                 double tol,
                 std::source_location at = std::source_location::current()) {
  check(a.size() == b.size(), "near_vector: size mismatch", at);
  const double reference = std::max(1., b.derived().stableNorm());
  const double difference = (a - b).stableNorm();
  if (!(difference <= tol * reference))
    fail("near_vector: difference norm " + std::to_string(difference) +
             " exceeds " + std::to_string(tol * reference),
         at);
}

} // namespace pde2d_test

// From the review of the markers: theta = 1 promises every positive
// indicator, also when a tiny one vanishes from the floating-point total.
#include "support/check.hpp"

#include "pde2d/adapt/dorfler_marking.hpp"

#include <vector>

using namespace pde2d;
using namespace pde2d::test;

namespace {
void theta_one_keeps_small_positive_indicators() {
  mesh::Mesh m;
  m.triangles.resize(2);
  // Ordinary estimator magnitudes: a region with error and a region
  // already close to roundoff. Neither value is near underflow.
  const std::vector<double> indicators{1e-4, 1e-13};
  const auto marked = adapt::DorflerMarking({.theta = 1}).mark(m, indicators);
  std::cout << "INFO marked " << marked.size() << " of 2 positive indicators\n";
  check(marked == std::vector<std::size_t>{0, 1},
        "theta = 1 marks every positive indicator");
}
} // namespace

int main() {
  return run({{"theta_one_keeps_small_positive_indicators",
               theta_one_keeps_small_positive_indicators}});
}

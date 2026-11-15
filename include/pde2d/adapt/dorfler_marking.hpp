#pragma once

#include "pde2d/adapt/marking.hpp"

namespace pde2d::adapt {

struct DorflerMarkingOptions {
  // In [0, 1]: the fraction of the total squared error the marked set must
  // hold. 0 marks nothing, 1 every triangle with a positive indicator.
  double theta = 0.5;
};

// Dörfler (bulk) marking: the smallest set M of triangles with
//   sum over M of eta_T^2 >= theta * sum over all T of eta_T^2.
// Triangles are taken largest indicator first, equal indicators by
// triangle index, until the sum is reached. Both sums are added in that
// same order, so with theta = 1 the last positive indicator reaches the
// total exactly.
class DorflerMarking final : public MarkingStrategy {
public:
  // Throws std::invalid_argument if theta is not in [0, 1].
  explicit DorflerMarking(DorflerMarkingOptions options = {});
  const DorflerMarkingOptions &options() const { return options_; }
  std::string name() const override { return "dorfler"; }

private:
  std::vector<std::size_t>
  select(std::span<const double> indicators) const override;
  DorflerMarkingOptions options_;
};

} // namespace pde2d::adapt

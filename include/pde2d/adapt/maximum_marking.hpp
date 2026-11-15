#pragma once

#include "pde2d/adapt/marking.hpp"

namespace pde2d::adapt {

struct MaximumMarkingOptions {
  // In [0, 1]. 1 marks only the triangles with the largest indicator, 0
  // every triangle with a positive one.
  double theta = 0.5;
};

// The maximum strategy: marks every triangle T with
//   eta_T >= theta * max eta.
class MaximumMarking final : public MarkingStrategy {
public:
  // Throws std::invalid_argument if theta is not in [0, 1].
  explicit MaximumMarking(MaximumMarkingOptions options = {});
  const MaximumMarkingOptions &options() const { return options_; }
  std::string name() const override { return "maximum"; }

private:
  std::vector<std::size_t>
  select(std::span<const double> indicators) const override;
  MaximumMarkingOptions options_;
};

} // namespace pde2d::adapt

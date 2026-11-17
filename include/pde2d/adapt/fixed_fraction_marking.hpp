#pragma once

#include "pde2d/adapt/marking.hpp"

namespace pde2d::adapt {

struct FixedFractionMarkingOptions {
  // In [0, 1]: the fraction of all triangles to mark.
  double fraction = 0.3;
};

// Fixed fraction marking: the round(fraction * N) triangles with the
// largest indicators, N the number of triangles. round() is to the
// nearest integer, halves away from zero, so a fraction too small for one
// triangle marks none and fraction * N is never pushed up by its own
// rounding error (0.07 * 100 is 7.000000000000001 in doubles). Equal
// indicators are taken by triangle index. Triangles with a zero indicator
// are not marked even when the count would reach them (the shared rule of
// MarkingStrategy::mark), so fewer may be marked.
class FixedFractionMarking final : public MarkingStrategy {
public:
  // Throws std::invalid_argument if fraction is not in [0, 1].
  explicit FixedFractionMarking(FixedFractionMarkingOptions options = {});
  const FixedFractionMarkingOptions &options() const { return options_; }
  std::string name() const override { return "fixed_fraction"; }

private:
  std::vector<std::size_t>
  select(std::span<const double> indicators) const override;
  FixedFractionMarkingOptions options_;
};

} // namespace pde2d::adapt

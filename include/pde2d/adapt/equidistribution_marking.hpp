#pragma once

#include "pde2d/adapt/marking.hpp"

namespace pde2d::adapt {

struct EquidistributionMarkingOptions {
  // The total error tolerance tol; it has no meaningful default (it is in
  // the units of the indicators), so it must be set: finite and positive.
  double tolerance = 0.0;
};

// Equidistribution marking: marks every triangle T with
//   eta_T >= tolerance / sqrt(N),
// N the number of triangles. If every indicator were exactly at that
// bound, the total error sqrt(sum eta_T^2) would be the tolerance.
class EquidistributionMarking final : public MarkingStrategy {
public:
  // Throws std::invalid_argument if the tolerance is not finite and
  // positive (also when it was not set).
  explicit EquidistributionMarking(EquidistributionMarkingOptions options);
  const EquidistributionMarkingOptions &options() const { return options_; }
  std::string name() const override { return "equidistribution"; }

private:
  std::vector<std::size_t>
  select(std::span<const double> indicators) const override;
  EquidistributionMarkingOptions options_;
};

} // namespace pde2d::adapt

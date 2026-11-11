#pragma once

#include "pde2d/mesh/components.hpp"

#include <limits>

namespace pde2d::mesh {

struct ChordLengthOptions {
  // Upper bound on the distance between a curve and its chords, in the
  // geometry's length unit. Infinity (the default) means no bound beyond
  // the edge length. Must be positive.
  double max_curve_deviation = std::numeric_limits<double>::infinity();
};

// Splits every curve into ceil(length / max_edge_length) pieces of equal
// arc length, so every chord is at most max_edge_length. It adds pieces
// while a chord is longer after rounding, or while a piece may deviate from
// its chord by more than max_curve_deviation (an analytic bound from the
// second derivative). Each loop gets at least three points. Every edge is
// linked to its curve and parameter interval.
// Metrics: "points", "loops".
class ChordLengthDiscretizer final : public BoundaryDiscretizer {
public:
  explicit ChordLengthDiscretizer(ChordLengthOptions options = {});
  const ChordLengthOptions &options() const { return options_; }

  std::string name() const override { return "chord_length"; }
  BoundaryPolygon run(const geometry::CurveGeometry &geometry,
                      const MeshSpec &spec, StageReport &report) const override;

private:
  ChordLengthOptions options_;
};

} // namespace pde2d::mesh

#pragma once

#include "pde2d/mesh/components.hpp"

namespace pde2d::mesh {

struct ChordLengthOptions {
  // Upper bound on the distance between a curve and its chords, in the
  // geometry's length unit. Infinity means no bound beyond the edge
  // length. Must be positive.
  double max_curve_deviation = 0.02;
  // Consecutive curves of one loop are joined into one chain, sampled as a
  // single path, unless the region changes or the boundary turns by more
  // than this angle where they meet. In degrees, in [0, 180]: 0 breaks at
  // every turn, 180 joins every junction of one region.
  // PROVISIONAL: 30 degrees has not been confirmed by the owner yet
  // (docs/decisions.md, "Open").
  double corner_angle_degrees = 30.0;
};

// Joins the curves of each loop into chains (see corner_angle_degrees)
// and splits every chain into ceil(length / max_edge_length) pieces of
// equal arc length, so every chord is at most max_edge_length. It adds
// pieces while a chord is longer after rounding, or while a piece may
// deviate from its chord by more than max_curve_deviation (an analytic
// bound from the second derivatives and, for a piece across junctions,
// the distances of the junctions from the chord). Junctions inside a chain are
// not forced to be nodes; chain ends (region changes and corners) are. Each
// loop gets at least three points. Every edge is linked to its chain and
// arc-length interval.
//
// The turning angle at a junction is the angle between the end tangent of
// one curve and the start tangent of the next. Where a curve's derivative
// vanishes at that end, the direction of its second derivative is used
// (the direction the curve leaves or arrives in), and failing that the
// chord from its start to its end; a junction with no usable direction is
// a break. A loop without any break is one chain that starts and ends at
// the start of the loop's first curve, so that point is always a node.
// Metrics: "points", "loops", "chains".
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

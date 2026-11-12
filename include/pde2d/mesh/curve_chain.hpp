#pragma once

#include "pde2d/geometry/curve.hpp"
#include "pde2d/geometry/point.hpp"

#include <cstddef>
#include <vector>

namespace pde2d::mesh {

// Consecutive exact curves that the meshing side treats as one path. A
// position on the chain is its arc length s, from 0 at the start of the
// first curve to length() at the end of the last one. Curve i + 1 is
// expected to start where curve i ends (this is not checked; the geometry
// guarantees it for chains made from a CurveGeometry).
//
// A boundary discretizer builds chains from the loops of a geometry
// (joining curves where the boundary is smooth enough), and boundary edges
// link to an arc-length interval of a chain (CurveLink). Chains belong to
// meshing, never to the geometry.
class CurveChain {
public:
  // A curve and a parameter on it.
  struct Position {
    std::size_t curve = 0;
    double t = 0.0;
  };

  // Throws std::invalid_argument if the list is empty or a curve is
  // invalid or has zero length.
  explicit CurveChain(std::vector<geometry::Curve> curves);

  const std::vector<geometry::Curve> &curves() const { return curves_; }
  double length() const { return offsets_.back(); }
  // Arc length at the start of curve i, for i in [0, curves().size()];
  // offset(curves().size()) == length().
  double offset(std::size_t i) const { return offsets_.at(i); }

  // The curve containing arc length s (clamped to [0, length()]) and the
  // parameter there: a binary search over the cumulative lengths, then
  // length inversion inside the curve. At a junction it is the start of
  // the later curve; at length() the end of the last curve.
  Position position(double s) const;
  // The same, but at a junction the end of the earlier curve.
  Position position_from_before(double s) const;
  geometry::Point point_at(double s) const;

private:
  std::vector<geometry::Curve> curves_;
  std::vector<double> offsets_; // size curves_.size() + 1, increasing
};

} // namespace pde2d::mesh

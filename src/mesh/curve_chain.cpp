#include "pde2d/mesh/curve_chain.hpp"

#include <algorithm>
#include <stdexcept>

namespace pde2d::mesh {

CurveChain::CurveChain(std::vector<geometry::Curve> curves)
    : curves_(std::move(curves)) {
  if (curves_.empty())
    throw std::invalid_argument("A curve chain needs at least one curve");
  offsets_.reserve(curves_.size() + 1);
  offsets_.push_back(0.0);
  for (const auto &c : curves_) {
    geometry::validate(c);
    double l = geometry::curve_length(c);
    if (!(l > 0))
      throw std::invalid_argument("A curve chain cannot hold a zero-length "
                                  "curve");
    offsets_.push_back(offsets_.back() + l);
  }
}

CurveChain::Position CurveChain::position(double s) const {
  s = std::clamp(s, 0.0, length());
  // The last curve whose start is at or before s.
  auto it = std::upper_bound(offsets_.begin(), offsets_.end() - 1, s);
  std::size_t i = static_cast<std::size_t>(it - offsets_.begin()) - 1;
  i = std::min(i, curves_.size() - 1);
  double begin = offsets_[i], end = offsets_[i + 1];
  if (s <= begin)
    return {i, 0.0};
  if (s >= end)
    return {i, 1.0};
  double fraction = (s - begin) / (end - begin);
  const auto &curve = curves_[i];
  if (geometry::is_line(curve))
    return {i, fraction};
  if (!(fraction > 0 && fraction < 1)) // rounding at the ends
    return {i, fraction <= 0 ? 0.0 : 1.0};
  return {i, geometry::parameter_at_length_fraction(curve, fraction)};
}

CurveChain::Position CurveChain::position_from_before(double s) const {
  Position p = position(s);
  if (p.t == 0.0 && p.curve > 0)
    return {p.curve - 1, 1.0};
  return p;
}

geometry::Point CurveChain::point_at(double s) const {
  Position p = position(s);
  return geometry::point_at(curves_[p.curve], p.t);
}

} // namespace pde2d::mesh

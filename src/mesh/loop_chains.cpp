#include "loop_chains.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace pde2d::mesh::detail {

using geometry::Point;

namespace {

// The direction in which the curve leaves its start (at_end false) or
// arrives at its end (at_end true); zero if there is none.
Point tangent(const geometry::Curve &curve, bool at_end) {
  double t = at_end ? 1.0 : 0.0;
  auto bounds = curve.derivative_bounds();
  Point d = curve.derivative_at(t);
  if (geometry::norm(d) > 1e-9 * bounds.first)
    return d;
  // c(t) ~ c(0) + c''(0) t^2 / 2 near the start, and c(1) - c(1 - t) ~
  // -c''(1) t^2 / 2 near the end.
  Point d2 = curve.second_derivative_at(t);
  if (geometry::norm(d2) > 1e-9 * bounds.second)
    return at_end ? -1.0 * d2 : d2;
  return geometry::end_point(curve) - geometry::start_point(curve);
}

// Angle in [0, pi] by which the boundary turns from `before` to `after`;
// pi when either direction is missing.
double turning_angle(const geometry::Curve &before,
                     const geometry::Curve &after) {
  Point u = tangent(before, true), v = tangent(after, false);
  if (geometry::norm(u) == 0 || geometry::norm(v) == 0)
    return std::numbers::pi;
  return std::atan2(std::abs(geometry::cross(u, v)), geometry::dot(u, v));
}

} // namespace

void validate_corner_angle(double corner_angle_degrees, const char *what) {
  if (!(corner_angle_degrees >= 0 && corner_angle_degrees <= 180))
    throw std::invalid_argument(std::string(what) + " must be in [0, 180]");
}

std::vector<LoopChain> loop_chains(const geometry::Loop &loop,
                                   double corner_angle_degrees) {
  const double corner = corner_angle_degrees * std::numbers::pi / 180;
  const auto &curves = loop.curves;
  const std::size_t n = curves.size();
  // Junction j joins curve j to curve j + 1 (cyclically).
  std::vector<bool> breaks(n);
  bool any_break = false;
  for (std::size_t j = 0; j < n; ++j) {
    const auto &next = curves[(j + 1) % n];
    breaks[j] = curves[j].region != next.region ||
                turning_angle(curves[j].curve, next.curve) > corner;
    any_break = any_break || breaks[j];
  }
  std::vector<std::size_t> starts;
  for (std::size_t j = 0; j < n; ++j)
    if (breaks[(j + n - 1) % n])
      starts.push_back(j);
  if (!any_break)
    starts.push_back(0);
  std::vector<LoopChain> chains;
  for (std::size_t c = 0; c < starts.size(); ++c) {
    std::size_t end = any_break ? starts[(c + 1) % starts.size()] : 0;
    std::vector<geometry::Curve> chain_curves;
    std::size_t j = starts[c];
    do {
      chain_curves.push_back(curves[j].curve);
      j = (j + 1) % n;
    } while (j != end);
    std::size_t count = chain_curves.size();
    chains.push_back({starts[c], count, CurveChain(std::move(chain_curves))});
  }
  return chains;
}

} // namespace pde2d::mesh::detail

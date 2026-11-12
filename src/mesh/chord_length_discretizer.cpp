#include "pde2d/mesh/chord_length_discretizer.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <stdexcept>

namespace pde2d::mesh {

using geometry::Point;

namespace {

constexpr std::size_t max_pieces_per_chain = 10'000'000;

// The direction in which the curve leaves its start (at_end false) or
// arrives at its end (at_end true); zero if there is none.
Point tangent(const geometry::Curve &curve, bool at_end) {
  double t = at_end ? 1.0 : 0.0;
  auto bounds = geometry::derivative_bounds(curve);
  Point d = geometry::derivative_at(curve, t);
  if (geometry::norm(d) > 1e-9 * bounds.first)
    return d;
  // c(t) ~ c(0) + c''(0) t^2 / 2 near the start, and c(1) - c(1 - t) ~
  // -c''(1) t^2 / 2 near the end.
  Point d2 = geometry::second_derivative_at(curve, t);
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

double distance_to_segment(Point p, Point a, Point b) {
  Point d = b - a;
  double dd = geometry::dot(d, d);
  double t = dd > 0 ? std::clamp(geometry::dot(p - a, d) / dd, 0.0, 1.0) : 0;
  return geometry::distance(p, a + t * d);
}

// Upper bound on the distance from the chain between arc lengths sa < sb
// to the chord from A (its point at sa) to B (at sb). Take each curve
// piece c(u), u in [0, 1], with second-derivative bound M and end points
// at distances d0, d1 from the chord. c(u) lies within M u (1 - u) / 2 of
// the straight line between its ends, and that line's point at u lies
// within (1 - u) d0 + u d1 of the chord (distance to a segment is
// convex), so the piece is within
//   g(u) = (1 - u) d0 + u d1 + M u (1 - u) / 2
// of the chord, and the bound is the maximum of g. For a piece on a single
// curve d0 = d1 = 0 and it is the usual M / 8; for a smooth junction it is
// about the bound of one curve through it.
double piece_deviation(const CurveChain &chain, double sa, double sb, Point A,
                       Point B) {
  auto pa = chain.position(sa);
  auto pb = chain.position_from_before(sb);
  double worst = 0;
  for (std::size_t i = pa.curve; i <= pb.curve; ++i) {
    double t0 = i == pa.curve ? pa.t : 0.0, t1 = i == pb.curve ? pb.t : 1.0;
    if (!(t0 < t1))
      continue;
    const auto &curve = chain.curves()[i];
    double m =
        geometry::is_line(curve)
            ? 0.0
            : geometry::derivative_bounds(geometry::subcurve(curve, t0, t1))
                      .second /
                  2;
    double d0 = 0, d1 = 0;
    if (pa.curve != pb.curve) {
      d0 = distance_to_segment(geometry::point_at(curve, t0), A, B);
      d1 = distance_to_segment(geometry::point_at(curve, t1), A, B);
    }
    double g = std::max(d0, d1);
    if (m > 0) {
      double u = 0.5 + (d1 - d0) / (2 * m);
      if (u > 0 && u < 1)
        g = std::max(g, (1 - u) * d0 + u * d1 + m * u * (1 - u));
    }
    worst = std::max(worst, g);
  }
  return worst;
}

// One chain of a loop, with its region and current number of pieces.
struct Piece {
  CurveChain chain;
  std::size_t region;
  std::size_t count;
};

// Arc length of node k of n on a chain of length L. k = n gives L exactly.
double node_position(double L, std::size_t k, std::size_t n) {
  return L * (static_cast<double>(k) / static_cast<double>(n));
}

} // namespace

ChordLengthDiscretizer::ChordLengthDiscretizer(ChordLengthOptions options)
    : options_(options) {
  if (!(options_.max_curve_deviation > 0))
    throw std::invalid_argument(
        "ChordLengthOptions::max_curve_deviation must be positive");
  if (!(options_.corner_angle_degrees >= 0 &&
        options_.corner_angle_degrees <= 180))
    throw std::invalid_argument(
        "ChordLengthOptions::corner_angle_degrees must be in [0, 180]");
}

BoundaryPolygon
ChordLengthDiscretizer::run(const geometry::CurveGeometry &geometry,
                            const MeshSpec &spec, StageReport &report) const {
  validate(spec);
  const double h = spec.max_edge_length;
  const double tolerance = options_.max_curve_deviation;
  const double corner = options_.corner_angle_degrees * std::numbers::pi / 180;
  BoundaryPolygon polygon;
  polygon.region_names = geometry.region_names();
  std::map<std::string, std::size_t> region_id;
  for (std::size_t i = 0; i < polygon.region_names.size(); ++i)
    region_id[polygon.region_names[i]] = i;

  std::size_t points = 0;
  for (std::size_t l = 0; l < geometry.loops().size(); ++l) {
    const auto &curves = geometry.loops()[l].curves;
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
    // Chains start after each break, in curve order; without a break the
    // whole loop is one chain from the start of curve 0.
    std::vector<std::size_t> starts;
    for (std::size_t j = 0; j < n; ++j)
      if (breaks[(j + n - 1) % n])
        starts.push_back(j);
    if (!any_break)
      starts.push_back(0);
    std::vector<Piece> pieces;
    for (std::size_t c = 0; c < starts.size(); ++c) {
      std::size_t end = any_break ? starts[(c + 1) % starts.size()] : 0;
      std::vector<geometry::Curve> chain_curves;
      std::size_t j = starts[c];
      do {
        chain_curves.push_back(curves[j].curve);
        j = (j + 1) % n;
      } while (j != end);
      CurveChain chain(std::move(chain_curves));
      double count = std::ceil(chain.length() / h);
      if (!(count <= static_cast<double>(max_pieces_per_chain)))
        throw std::invalid_argument(
            "max_edge_length is too small for this geometry");
      pieces.push_back(
          {std::move(chain), region_id.at(curves[starts[c]].region),
           std::max<std::size_t>(1, static_cast<std::size_t>(count))});
    }
    // A loop needs at least three points: add pieces to the chain with the
    // longest pieces.
    auto total = [&] {
      std::size_t s = 0;
      for (const auto &p : pieces)
        s += p.count;
      return s;
    };
    while (total() < 3) {
      std::size_t best = 0;
      auto piece_length = [&](std::size_t k) {
        return pieces[k].chain.length() / static_cast<double>(pieces[k].count);
      };
      for (std::size_t k = 1; k < pieces.size(); ++k)
        if (piece_length(k) > piece_length(best))
          best = k;
      ++pieces[best].count;
    }

    PolygonLoop loop;
    loop.depth = geometry.depth(l);
    loop.parent = geometry.parent(l);
    for (auto &piece : pieces) {
      const CurveChain &chain = piece.chain;
      const double L = chain.length();
      std::vector<Point> q;
      while (true) {
        const std::size_t m = piece.count;
        q.resize(m + 1);
        for (std::size_t k = 0; k <= m; ++k)
          q[k] = chain.point_at(node_position(L, k, m));
        double worst = 0;
        bool chords_fit = true;
        for (std::size_t k = 0; k < m; ++k) {
          chords_fit = chords_fit && geometry::distance(q[k], q[k + 1]) <= h;
          worst = std::max(worst, piece_deviation(chain, node_position(L, k, m),
                                                  node_position(L, k + 1, m),
                                                  q[k], q[k + 1]));
        }
        double next = static_cast<double>(m);
        if (worst > tolerance)
          next = std::max(std::ceil(next * std::sqrt(worst / tolerance)),
                          next + 1);
        else if (!chords_fit)
          next += 1;
        else
          break;
        if (!(next <= static_cast<double>(max_pieces_per_chain)))
          throw std::invalid_argument(
              "max_curve_deviation is too small for this geometry");
        piece.count = static_cast<std::size_t>(next);
      }
      std::size_t chain_index = polygon.chains.size();
      for (std::size_t k = 0; k < piece.count; ++k) {
        loop.points.push_back(q[k]);
        loop.edges.push_back(
            {piece.region,
             CurveLink{chain_index, node_position(L, k, piece.count),
                       node_position(L, k + 1, piece.count)}});
      }
      polygon.chains.push_back(chain);
    }
    points += loop.points.size();
    polygon.loops.push_back(std::move(loop));
  }
  report.metrics["points"] = static_cast<double>(points);
  report.metrics["loops"] = static_cast<double>(polygon.loops.size());
  report.metrics["chains"] = static_cast<double>(polygon.chains.size());
  return polygon;
}

} // namespace pde2d::mesh

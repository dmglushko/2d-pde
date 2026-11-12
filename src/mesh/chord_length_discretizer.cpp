#include "pde2d/mesh/chord_length_discretizer.hpp"

#include "loop_chains.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace pde2d::mesh {

using geometry::Point;

namespace {

constexpr std::size_t max_pieces_per_chain = 10'000'000;

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
  detail::validate_corner_angle(options_.corner_angle_degrees,
                                "ChordLengthOptions::corner_angle_degrees");
}

BoundaryPolygon
ChordLengthDiscretizer::run(const geometry::CurveGeometry &geometry,
                            const MeshSpec &spec, StageReport &report) const {
  validate(spec);
  const double h = spec.max_edge_length;
  const double tolerance = options_.max_curve_deviation;
  BoundaryPolygon polygon;
  polygon.region_names = geometry.region_names();
  std::map<std::string, std::size_t> region_id;
  for (std::size_t i = 0; i < polygon.region_names.size(); ++i)
    region_id[polygon.region_names[i]] = i;

  std::size_t points = 0;
  for (std::size_t l = 0; l < geometry.loops().size(); ++l) {
    const auto &curves = geometry.loops()[l].curves;
    std::vector<Piece> pieces;
    for (auto &c : detail::loop_chains(geometry.loops()[l],
                                       options_.corner_angle_degrees)) {
      double count = std::ceil(c.chain.length() / h);
      if (!(count <= static_cast<double>(max_pieces_per_chain)))
        throw std::invalid_argument(
            "max_edge_length is too small for this geometry");
      pieces.push_back(
          {std::move(c.chain), region_id.at(curves[c.first].region),
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

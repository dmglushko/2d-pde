#include "pde2d/mesh/chord_length_discretizer.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace pde2d::mesh {

namespace {

constexpr std::size_t max_pieces_per_curve = 10'000'000;

// Parameters 0 = t_0 < ... < t_n = 1 splitting the curve into n pieces of
// equal arc length.
std::vector<double> equal_length_parameters(const geometry::Curve &curve,
                                            std::size_t n) {
  std::vector<double> t(n + 1);
  t[0] = 0;
  t[n] = 1;
  for (std::size_t k = 1; k < n; ++k) {
    double f = static_cast<double>(k) / static_cast<double>(n);
    t[k] = geometry::is_line(curve)
               ? f
               : geometry::parameter_at_length_fraction(curve, f);
  }
  return t;
}

// True if every chord between consecutive parameters is at most h.
bool chords_fit(const geometry::Curve &curve, const std::vector<double> &t,
                double h) {
  for (std::size_t k = 0; k + 1 < t.size(); ++k)
    if (geometry::distance(geometry::point_at(curve, t[k]),
                           geometry::point_at(curve, t[k + 1])) > h)
      return false;
  return true;
}

// Largest analytic bound on chord-to-curve distance over the pieces.
double max_deviation(const geometry::Curve &curve,
                     const std::vector<double> &t) {
  if (geometry::is_line(curve))
    return 0;
  double worst = 0;
  for (std::size_t k = 0; k + 1 < t.size(); ++k) {
    auto piece = geometry::subcurve(curve, t[k], t[k + 1]);
    worst = std::max(worst, geometry::derivative_bounds(piece).second / 8);
  }
  return worst;
}

} // namespace

ChordLengthDiscretizer::ChordLengthDiscretizer(ChordLengthOptions options)
    : options_(options) {
  if (!(options_.max_curve_deviation > 0))
    throw std::invalid_argument(
        "ChordLengthOptions::max_curve_deviation must be positive");
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
    // Piece counts: from the edge length, then raised until every chord is
    // at most h (rounding) and the deviation bound holds.
    std::vector<double> lengths;
    std::vector<std::size_t> counts;
    for (const auto &c : curves) {
      double length = geometry::curve_length(c.curve);
      lengths.push_back(length);
      double n = std::ceil(length / h);
      if (!(n <= static_cast<double>(max_pieces_per_curve)))
        throw std::invalid_argument(
            "max_edge_length is too small for this geometry");
      counts.push_back(std::max<std::size_t>(1, static_cast<std::size_t>(n)));
    }
    // A loop needs at least three points: add pieces to the curve with the
    // longest pieces.
    auto total = [&] {
      std::size_t s = 0;
      for (auto n : counts)
        s += n;
      return s;
    };
    while (total() < 3) {
      std::size_t best = 0;
      for (std::size_t k = 1; k < curves.size(); ++k)
        if (lengths[k] / static_cast<double>(counts[k]) >
            lengths[best] / static_cast<double>(counts[best]))
          best = k;
      ++counts[best];
    }
    PolygonLoop loop;
    loop.depth = geometry.depth(l);
    loop.parent = geometry.parent(l);
    for (std::size_t k = 0; k < curves.size(); ++k) {
      const auto &curve = curves[k].curve;
      std::vector<double> t = equal_length_parameters(curve, counts[k]);
      while (true) {
        double worst = max_deviation(curve, t);
        double n = static_cast<double>(counts[k]);
        if (worst > tolerance)
          n = std::max(std::ceil(n * std::sqrt(worst / tolerance)), n + 1);
        else if (!chords_fit(curve, t, h))
          n += 1;
        else
          break;
        if (!(n <= static_cast<double>(max_pieces_per_curve)))
          throw std::invalid_argument(
              "max_curve_deviation is too small for this geometry");
        counts[k] = static_cast<std::size_t>(n);
        t = equal_length_parameters(curve, counts[k]);
      }
      std::size_t curve_index = polygon.curves.size();
      polygon.curves.push_back(curve);
      std::size_t region = region_id.at(curves[k].region);
      for (std::size_t j = 0; j + 1 < t.size(); ++j) {
        loop.points.push_back(geometry::point_at(curve, t[j]));
        loop.edges.push_back({region, CurveLink{curve_index, t[j], t[j + 1]}});
      }
    }
    points += loop.points.size();
    polygon.loops.push_back(std::move(loop));
  }
  report.metrics["points"] = static_cast<double>(points);
  report.metrics["loops"] = static_cast<double>(polygon.loops.size());
  return polygon;
}

} // namespace pde2d::mesh

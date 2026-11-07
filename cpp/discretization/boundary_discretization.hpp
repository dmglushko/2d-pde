#pragma once
#include "mesh_types.hpp"
#include "mesh_validation.hpp"

namespace discretization {
struct DiscretizedBoundary {
  Boundary boundary;
  std::optional<BoundaryContext> context;
};
namespace mesh_detail {
inline void nonnegative(double value, const char *name) {
  validateNonnegative(value, name);
}
inline void validate_options(const MeshOptions &o) {
  validateMeshOptionValues(o);
  if (o.algorithm == InteriorAlgorithm::EarClipping &&
      o.refinement.max_edge_length > 0)
    throw std::invalid_argument(
        "Ear clipping has no interior refinement; use constrained Delaunay or "
        "refine_mesh with subdivision");
}
inline CurveSegment world_curve(const CurveSegment &segment,
                                const Geometry2D &geometry) {
  return transformCurve(segment, {1, geometry.rotation(), geometry.center()});
}

inline CurveSegment subcurve(CurveSegment c, double a, double b) {
  if (a > b) {
    c = subcurve(std::move(c), b, a);
    return reverseCurve(c);
  }
  if (b < 1)
    c = splitCurve(c, b).first;
  if (a > 0)
    c = splitCurve(c, a / b).second;
  return c;
}
inline std::pair<double, double> curve_bounds(const CurveSegment &piece) {
  auto b = curveDerivativeBounds(piece);
  return {b.first, b.second};
}
inline Point2 tangent(const CurveSegment &piece, bool end) {
  return curveDerivative(piece, end ? 1 : 0);
}
inline bool is_corner(const CurveSegment &left, const CurveSegment &right) {
  auto u = tangent(left, true), v = tangent(right, false);
  double ul = std::hypot(u.x, u.y), vl = std::hypot(v.x, v.y);
  if (ul == 0 || vl == 0)
    return true;
  u = {u.x / ul, u.y / ul};
  v = {v.x / vl, v.y / vl};
  return std::abs(u.x * v.y - u.y * v.x) > 1e-10 || u.x * v.x + u.y * v.y <= 0;
}
inline double point_segment_distance(Point2 p, Point2 a, Point2 b) {
  double dx = b.x - a.x, dy = b.y - a.y, len = std::hypot(dx, dy);
  if (len == 0)
    return distance(p, a);
  double u = std::clamp(
      ((p.x - a.x) * (dx / len) + (p.y - a.y) * (dy / len)) / len, 0.0, 1.0);
  return distance(p, {a.x + u * dx, a.y + u * dy});
}
} // namespace mesh_detail

inline DiscretizedBoundary
discretize_boundary(const CurveGeometry &geometry,
                    const BoundaryOptions &options = {},
                    bool preserve_regions = true) {
  using namespace mesh_detail;
  nonnegative(options.max_edge_length, "Boundary edge length");
  nonnegative(options.max_curve_deviation, "Curve deviation");
  BoundaryContext ctx;
  for (const auto &c : geometry.curves())
    ctx.curves.push_back({c.id, world_curve(c.segment, geometry), c.region});
  if (ctx.curves.empty())
    throw std::invalid_argument("Empty analytical boundary");
  Boundary b;
  std::vector<bool> protected_flags;
  double scale = 0;
  for (const auto &c : ctx.curves)
    scale = std::max(scale, curve_bounds(c.segment).first);
  if (!std::isfinite(scale))
    throw std::invalid_argument("Unsupported geometry scale");
  for (std::size_t i = 0; i < ctx.curves.size(); ++i) {
    const auto &c = ctx.curves[i];
    const auto &prev =
        ctx.curves[(i + ctx.curves.size() - 1) % ctx.curves.size()];
    curve_detail::validateCurve(c.segment);
    if (distance(curvePoint(prev.segment, 1), curvePoint(c.segment, 0)) >
        1e-12 * std::max(1.0, scale))
      throw std::invalid_argument("Disconnected analytical boundary");
    if (options.max_edge_length == 0 && options.max_curve_deviation == 0 &&
        !curveIsLinear(c.segment))
      throw std::invalid_argument(
          "Curved boundary needs an enabled length or deviation constraint");
    bool anchor =
        (options.preserve_corners && is_corner(prev.segment, c.segment)) ||
        (preserve_regions && prev.region != c.region);
    auto emit = [&](auto &&self, const CurveSegment &piece, double a, double z,
                    int depth) -> void {
      auto [length, second] = curve_bounds(piece);
      if (!std::isfinite(length) || !std::isfinite(second))
        throw std::invalid_argument("Nonfinite curve bound");
      bool split =
          // One or two source curves still need enough samples to form a
          // polygon when both user tolerances exceed the shape's dimensions.
          (ctx.curves.size() < 3 && z - a > 0.25) ||
          (options.max_edge_length > 0 && length > options.max_edge_length) ||
          (options.max_curve_deviation > 0 &&
           second / 8 > options.max_curve_deviation) ||
          distance(curvePoint(piece, 0), curvePoint(piece, 1)) == 0;
      if (split) {
        if (depth >= 30 || b.points.size() >= 1000000)
          throw std::runtime_error("Boundary discretization budget exceeded");
        auto halves = splitCurve(piece, .5);
        double mid = (a + z) / 2;
        self(self, halves.first, a, mid, depth + 1);
        self(self, halves.second, mid, z, depth + 1);
      } else {
        b.points.push_back(curvePoint(piece, 0));
        if (preserve_regions)
          b.edge_tags.push_back(c.region);
        ctx.edge_sources.push_back({{i, a, z}});
        protected_flags.push_back(a == 0 && anchor);
      }
    };
    emit(emit, c.segment, 0, 1, 0);
  }
  // Remove representation-only junctions when the combined chord still meets
  // both requested bounds. Each chord retains all source intervals it spans.
  struct IntervalData {
    double length, second;
    Point2 begin, end;
  };
  std::vector<std::vector<IntervalData>> intervalData;
  intervalData.reserve(ctx.edge_sources.size());
  for (const auto &spans : ctx.edge_sources) {
    auto &data = intervalData.emplace_back();
    for (const auto &span : spans) {
      auto piece =
          subcurve(ctx.curves[span.curve].segment, span.begin, span.end);
      auto bounds = curve_bounds(piece);
      data.push_back({bounds.first, bounds.second, curvePoint(piece, 0),
                      curvePoint(piece, 1)});
    }
  }
  bool changed = true;
  while (changed && b.points.size() > 3) {
    changed = false;
    for (std::size_t i = 0; i < b.points.size(); ++i) {
      if (protected_flags[i])
        continue;
      auto n = b.points.size(), p = (i + n - 1) % n, z = (i + 1) % n;
      auto a = b.points[p], end = b.points[z];
      if (distance(a, end) == 0)
        continue;
      double length = 0, error = 0;
      for (auto edge : {p, i})
        for (const auto &data : intervalData[edge]) {
          length += data.length;
          error = std::max(
              error, data.second / 8 +
                         std::max(point_segment_distance(data.begin, a, end),
                                  point_segment_distance(data.end, a, end)));
        }
      if ((options.max_edge_length > 0 &&
           length > options.max_edge_length * (1 + 1e-12)) ||
          (options.max_curve_deviation > 0 &&
           error > options.max_curve_deviation))
        continue;
      auto &spans = ctx.edge_sources[p];
      spans.insert(spans.end(), ctx.edge_sources[i].begin(),
                   ctx.edge_sources[i].end());
      auto &data = intervalData[p];
      data.insert(data.end(), intervalData[i].begin(), intervalData[i].end());
      intervalData.erase(intervalData.begin() + i);
      b.points.erase(b.points.begin() + i);
      ctx.edge_sources.erase(ctx.edge_sources.begin() + i);
      protected_flags.erase(protected_flags.begin() + i);
      if (preserve_regions)
        b.edge_tags.erase(b.edge_tags.begin() + i);
      changed = true;
      break;
    }
  }
  for (std::size_t i = 0; i < protected_flags.size(); ++i)
    if (protected_flags[i])
      ctx.protected_vertices.push_back(i);
  validate_polygon(b);
  ctx.polygon_snapshot = b.points;
  return {std::move(b), std::move(ctx)};
}
} // namespace discretization

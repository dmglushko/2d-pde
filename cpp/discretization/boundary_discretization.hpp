#pragma once
#include "mesh_types.hpp"
#include "mesh_validation.hpp"

namespace pde2d::discretization {
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
      size_refinement_requested(o.refinement))
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

// Samples every boundary loop (outer loop, then each hole) on its own:
// region junctions and corners are anchored per loop, and each chord keeps
// the source intervals it represents. The polygon (with holes) is validated
// before it is returned.
inline DiscretizedBoundary
discretize_boundary(const CurveGeometry &geometry,
                    const BoundaryOptions &options = {},
                    bool preserve_regions = true) {
  using namespace mesh_detail;
  nonnegative(options.max_edge_length, "Boundary edge length");
  nonnegative(options.max_curve_deviation, "Curve deviation");
  BoundaryContext ctx;
  for (const auto &c : geometry.curves())
    ctx.curves.push_back(
        {c.id, world_curve(c.segment, geometry), c.region, c.loop});
  if (ctx.curves.empty())
    throw std::invalid_argument("Empty analytical boundary");
  double scale = 0;
  for (const auto &c : ctx.curves)
    scale = std::max(scale, curve_bounds(c.segment).first);
  if (!std::isfinite(scale))
    throw std::invalid_argument("Unsupported geometry scale");
  // Curves are stored loop by loop; [first, last) is one loop.
  std::vector<std::pair<std::size_t, std::size_t>> ranges;
  for (std::size_t i = 0; i < ctx.curves.size(); ++i) {
    if (i == 0 || ctx.curves[i - 1].loop != ctx.curves[i].loop) {
      if (ctx.curves[i].loop != ranges.size())
        throw std::invalid_argument("Boundary loops must be stored in order");
      ranges.push_back({i, i});
    }
    ranges.back().second = i + 1;
  }
  struct LoopSamples {
    std::vector<Point2> points;
    std::vector<std::string> tags;
    std::vector<std::vector<CurveInterval>> sources;
    std::vector<bool> protected_flags;
  };
  std::vector<LoopSamples> loops;
  std::size_t sampled = 0;
  for (auto [first, last] : ranges) {
    auto &loop = loops.emplace_back();
    const std::size_t count = last - first;
    for (std::size_t i = first; i < last; ++i) {
      const auto &c = ctx.curves[i];
      const auto &prev = ctx.curves[first + (i - first + count - 1) % count];
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
      auto emit = [&](auto &&self, const CurveSegment &piece, double a,
                      double z, int depth) -> void {
        auto [length, second] = curve_bounds(piece);
        if (!std::isfinite(length) || !std::isfinite(second))
          throw std::invalid_argument("Nonfinite curve bound");
        bool split =
            // One or two source curves still need enough samples to form a
            // polygon when both user tolerances exceed the shape's
            // dimensions.
            (count < 3 && z - a > 0.25) ||
            (options.max_edge_length > 0 && length > options.max_edge_length) ||
            (options.max_curve_deviation > 0 &&
             second / 8 > options.max_curve_deviation) ||
            distance(curvePoint(piece, 0), curvePoint(piece, 1)) == 0;
        if (split) {
          if (depth >= 30 || sampled + loop.points.size() >= 1000000)
            throw std::runtime_error("Boundary discretization budget exceeded");
          auto halves = splitCurve(piece, .5);
          double mid = (a + z) / 2;
          self(self, halves.first, a, mid, depth + 1);
          self(self, halves.second, mid, z, depth + 1);
        } else {
          loop.points.push_back(curvePoint(piece, 0));
          if (preserve_regions)
            loop.tags.push_back(c.region);
          loop.sources.push_back({{i, a, z}});
          loop.protected_flags.push_back(a == 0 && anchor);
        }
      };
      emit(emit, c.segment, 0, 1, 0);
    }
    sampled += loop.points.size();
  }
  // Remove representation-only junctions when the combined chord still meets
  // both requested bounds. Each chord retains all source intervals it spans.
  struct IntervalData {
    double length, second;
    Point2 begin, end;
  };
  for (auto &loop : loops) {
    std::vector<std::vector<IntervalData>> intervalData;
    intervalData.reserve(loop.sources.size());
    for (const auto &spans : loop.sources) {
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
    while (changed && loop.points.size() > 3) {
      changed = false;
      for (std::size_t i = 0; i < loop.points.size(); ++i) {
        if (loop.protected_flags[i])
          continue;
        auto n = loop.points.size(), p = (i + n - 1) % n, z = (i + 1) % n;
        auto a = loop.points[p], end = loop.points[z];
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
        auto &spans = loop.sources[p];
        spans.insert(spans.end(), loop.sources[i].begin(),
                     loop.sources[i].end());
        auto &data = intervalData[p];
        data.insert(data.end(), intervalData[i].begin(), intervalData[i].end());
        intervalData.erase(intervalData.begin() + i);
        loop.points.erase(loop.points.begin() + i);
        loop.sources.erase(loop.sources.begin() + i);
        loop.protected_flags.erase(loop.protected_flags.begin() + i);
        if (preserve_regions)
          loop.tags.erase(loop.tags.begin() + i);
        changed = true;
        break;
      }
    }
  }
  Boundary b;
  std::size_t offset = 0;
  for (std::size_t k = 0; k < loops.size(); ++k) {
    auto &loop = loops[k];
    for (std::size_t i = 0; i < loop.protected_flags.size(); ++i)
      if (loop.protected_flags[i])
        ctx.protected_vertices.push_back(offset + i);
    ctx.edge_sources.insert(ctx.edge_sources.end(), loop.sources.begin(),
                            loop.sources.end());
    offset += loop.points.size();
    if (k == 0) {
      b.points = std::move(loop.points);
      b.edge_tags = std::move(loop.tags);
    } else {
      ctx.snapshot_loop_sizes.resize(k, b.points.size());
      ctx.snapshot_loop_sizes.push_back(loop.points.size());
      b.holes.push_back({std::move(loop.points), std::move(loop.tags)});
    }
  }
  validate_polygon(b);
  ctx.polygon_snapshot = boundary_points(b);
  return {std::move(b), std::move(ctx)};
}
} // namespace pde2d::discretization

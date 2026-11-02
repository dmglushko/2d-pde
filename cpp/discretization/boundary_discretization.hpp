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
  if (!std::isfinite(value) || value < 0)
    throw std::invalid_argument(
        std::string(name) +
        " must be finite and nonnegative; zero disables it");
}
inline void validate_options(const MeshOptions &o) {
  nonnegative(o.boundary.max_edge_length, "Boundary edge length");
  nonnegative(o.boundary.max_curve_deviation, "Curve deviation");
  nonnegative(o.refinement.max_edge_length, "Refinement edge length");
  if (o.algorithm != InteriorAlgorithm::EarClipping &&
      o.algorithm != InteriorAlgorithm::ConstrainedDelaunay)
    throw std::invalid_argument("Unknown interior algorithm");
  if (o.boundary_treatment != BoundaryTreatment::Fixed &&
      o.boundary_treatment != BoundaryTreatment::SplitPolygonEdges &&
      o.boundary_treatment != BoundaryTreatment::RefineOnCurves)
    throw std::invalid_argument("Unknown boundary treatment");
  if (o.refinement.max_steps < 0)
    throw std::invalid_argument("Refinement step budget must be nonnegative");
  if (o.algorithm == InteriorAlgorithm::EarClipping &&
      o.refinement.max_edge_length > 0)
    throw std::invalid_argument(
        "Ear clipping has no interior refinement; use constrained Delaunay or "
        "refine_mesh with subdivision");
}
inline CurveSegment world_curve(const CurveSegment &segment,
                                const Geometry2D &geometry) {
  auto transform = [&](Point2 p) {
    double c = std::cos(geometry.rotation()), s = std::sin(geometry.rotation());
    auto center = geometry.center();
    return Point2{center.x + c * p.x - s * p.y, center.y + s * p.x + c * p.y};
  };
  return std::visit(
      [&](auto c) -> CurveSegment {
        using T = std::decay_t<decltype(c)>;
        if constexpr (std::is_same_v<T, EllipticalArc>) {
          c.center = transform(c.center);
          c.rotation += geometry.rotation();
        } else {
          c.start = transform(c.start);
          c.end = transform(c.end);
          if constexpr (std::is_same_v<T, QuadraticBezier>)
            c.control = transform(c.control);
          if constexpr (std::is_same_v<T, CubicBezier>) {
            c.control1 = transform(c.control1);
            c.control2 = transform(c.control2);
          }
        }
        return c;
      },
      segment);
}
inline CurveSegment subcurve(CurveSegment c, double a, double b) {
  if (a > b) {
    c = subcurve(std::move(c), b, a);
    return std::visit(
        [](auto piece) -> CurveSegment {
          using T = std::decay_t<decltype(piece)>;
          if constexpr (std::is_same_v<T, EllipticalArc>) {
            piece.start_angle += piece.sweep_angle;
            piece.sweep_angle = -piece.sweep_angle;
          } else {
            std::swap(piece.start, piece.end);
            if constexpr (std::is_same_v<T, CubicBezier>)
              std::swap(piece.control1, piece.control2);
          }
          return piece;
        },
        c);
  }
  if (b < 1)
    c = splitCurve(c, b).first;
  if (a > 0)
    c = splitCurve(c, a / b).second;
  return c;
}
inline std::pair<double, double> curve_bounds(const CurveSegment &piece) {
  return std::visit(
      [](const auto &c) -> std::pair<double, double> {
        using T = std::decay_t<decltype(c)>;
        if constexpr (std::is_same_v<T, LineCurve>)
          return {distance(c.start, c.end), 0};
        else if constexpr (std::is_same_v<T, QuadraticBezier>)
          return {2 * std::max(distance(c.start, c.control),
                               distance(c.control, c.end)),
                  2 * std::hypot(c.start.x - 2 * c.control.x + c.end.x,
                                 c.start.y - 2 * c.control.y + c.end.y)};
        else if constexpr (std::is_same_v<T, CubicBezier>)
          return {
              3 * std::max({distance(c.start, c.control1),
                            distance(c.control1, c.control2),
                            distance(c.control2, c.end)}),
              6 * std::max(
                      std::hypot(c.start.x - 2 * c.control1.x + c.control2.x,
                                 c.start.y - 2 * c.control1.y + c.control2.y),
                      std::hypot(c.control1.x - 2 * c.control2.x + c.end.x,
                                 c.control1.y - 2 * c.control2.y + c.end.y))};
        else {
          double r = std::max(c.radius_x, c.radius_y),
                 w = std::abs(c.sweep_angle);
          return {r * w, r * w * w};
        }
      },
      piece);
}
inline Point2 tangent(const CurveSegment &piece, bool end) {
  return std::visit(
      [&](const auto &c) -> Point2 {
        using T = std::decay_t<decltype(c)>;
        if constexpr (std::is_same_v<T, EllipticalArc>) {
          double a = c.start_angle + (end ? c.sweep_angle : 0),
                 x = -c.radius_x * std::sin(a) * c.sweep_angle,
                 y = c.radius_y * std::cos(a) * c.sweep_angle;
          return {std::cos(c.rotation) * x - std::sin(c.rotation) * y,
                  std::sin(c.rotation) * x + std::cos(c.rotation) * y};
        } else {
          Point2 a = c.start, b = c.end;
          if constexpr (std::is_same_v<T, QuadraticBezier>) {
            a = end ? c.control : c.start;
            b = end ? c.end : c.control;
          }
          if constexpr (std::is_same_v<T, CubicBezier>) {
            a = end ? c.control2 : c.start;
            b = end ? c.end : c.control1;
          }
          return {b.x - a.x, b.y - a.y};
        }
      },
      piece);
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
discretize_boundary(const Geometry2D &geometry,
                    const BoundaryOptions &options = {},
                    bool preserve_regions = true) {
  using namespace mesh_detail;
  nonnegative(options.max_edge_length, "Boundary edge length");
  nonnegative(options.max_curve_deviation, "Curve deviation");
  const auto *analytical = dynamic_cast<const CurveGeometry *>(&geometry);
  if (!analytical) {
    Boundary b = geometry.createBoundary();
    if (!preserve_regions)
      b.edge_tags.clear();
    validate_polygon(b);
    return {std::move(b), std::nullopt};
  }
  BoundaryContext ctx;
  for (const auto &c : analytical->curves())
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
        !std::holds_alternative<LineCurve>(c.segment))
      throw std::invalid_argument(
          "Curved boundary needs an enabled length or deviation constraint");
    auto u = tangent(prev.segment, true), v = tangent(c.segment, false);
    double ul = std::hypot(u.x, u.y), vl = std::hypot(v.x, v.y);
    bool corner = ul == 0 || vl == 0;
    if (!corner) {
      u = {u.x / ul, u.y / ul};
      v = {v.x / vl, v.y / vl};
      corner =
          std::abs(u.x * v.y - u.y * v.x) > 1e-10 || u.x * v.x + u.y * v.y <= 0;
    }
    bool anchor = corner || (preserve_regions && prev.region != c.region);
    auto emit = [&](auto &&self, const CurveSegment &piece, double a, double z,
                    int depth) -> void {
      auto [length, second] = curve_bounds(piece);
      if (!std::isfinite(length) || !std::isfinite(second))
        throw std::invalid_argument("Nonfinite curve bound");
      bool split =
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
      auto spans = ctx.edge_sources[p];
      spans.insert(spans.end(), ctx.edge_sources[i].begin(),
                   ctx.edge_sources[i].end());
      double length = 0, error = 0;
      for (auto span : spans) {
        auto piece =
            subcurve(ctx.curves[span.curve].segment, span.begin, span.end);
        auto bounds = curve_bounds(piece);
        length += bounds.first;
        error = std::max(
            error,
            bounds.second / 8 +
                std::max(point_segment_distance(curvePoint(piece, 0), a, end),
                         point_segment_distance(curvePoint(piece, 1), a, end)));
      }
      if ((options.max_edge_length > 0 &&
           length > options.max_edge_length * (1 + 1e-12)) ||
          (options.max_curve_deviation > 0 &&
           error > options.max_curve_deviation))
        continue;
      ctx.edge_sources[p] = std::move(spans);
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

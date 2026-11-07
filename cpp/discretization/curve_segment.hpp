#pragma once

#include "geometry_base.hpp"
#include <optional>
#include <variant>

namespace pde2d::discretization {

// Bounds on Euclidean derivative norms over the ENTIRE local [0,1] interval.
// These are analytical upper bounds, not sampled estimates or exact maxima.
struct DerivativeBounds {
  double first, second;
};

// Subtract origin, apply positive uniform scale and rotation, then translate.
// General affine
// transforms are deliberately excluded: they need different arc handling.
struct CurveTransform {
  double scale = 1, rotation = 0;
  Point2 translation{}, origin{};
  void validate() const {
    if (!(scale > 0) || !std::isfinite(scale) || !std::isfinite(rotation) ||
        !std::isfinite(translation.x) || !std::isfinite(translation.y) ||
        !std::isfinite(origin.x) || !std::isfinite(origin.y))
      throw std::invalid_argument("Invalid curve transform");
  }
  Point2 operator()(Point2 p) const {
    p.x -= origin.x;
    p.y -= origin.y;
    const double c = std::cos(rotation), s = std::sin(rotation);
    return {translation.x + scale * (c * p.x - s * p.y),
            translation.y + scale * (s * p.x + c * p.y)};
  }
};
namespace curve_detail {
inline void validatePoint(Point2 p) {
  if (!std::isfinite(p.x) || !std::isfinite(p.y))
    throw std::invalid_argument("Non-finite curve coordinate");
}
inline void validateSplit(double t) {
  if (!(t > 0 && t < 1) || !std::isfinite(t))
    throw std::invalid_argument("Split parameter must be between 0 and 1");
}
inline Point2 mix(Point2 a, Point2 b, double t) {
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}
inline Point2 difference(Point2 a, Point2 b, double factor = 1) {
  return {factor * (a.x - b.x), factor * (a.y - b.y)};
}
inline double norm(Point2 p) { return std::hypot(p.x, p.y); }
inline double distance(Point2 a, Point2 b) { return norm(difference(a, b)); }
inline Point2 secondDifference(Point2 a, Point2 b, Point2 c, double factor) {
  return {factor * (a.x - 2 * b.x + c.x), factor * (a.y - 2 * b.y + c.y)};
}
} // namespace curve_detail

struct LineCurve {
  Point2 start, end;
  Point2 point(double t) const {
    return {(1 - t) * start.x + t * end.x, (1 - t) * start.y + t * end.y};
  }
  Point2 derivative(double) const {
    return curve_detail::difference(end, start);
  }
  Point2 secondDerivative(double) const { return {0, 0}; }
  DerivativeBounds derivativeBounds() const {
    return {curve_detail::distance(start, end), 0};
  }
  std::pair<LineCurve, LineCurve> split(double t) const {
    curve_detail::validateSplit(t);
    auto p = curve_detail::mix(start, end, t);
    return {{start, p}, {p, end}};
  }
  LineCurve reversed() const { return {end, start}; }
  LineCurve transformed(const CurveTransform &f) const {
    return {f(start), f(end)};
  }
  void validate() const {
    curve_detail::validatePoint(start);
    curve_detail::validatePoint(end);
  }
  std::optional<double> exactLength(double endParameter) const {
    return endParameter * curve_detail::distance(start, end);
  }
  std::optional<double> exactLengthParameter(double fraction) const {
    return fraction;
  }
  bool isLinear() const { return true; }
};
struct QuadraticBezier {
  Point2 start, control, end;
  Point2 point(double t) const {
    const double a = (1 - t) * (1 - t), b = 2 * t * (1 - t), d = t * t;
    return {a * start.x + b * control.x + d * end.x,
            a * start.y + b * control.y + d * end.y};
  }
  Point2 derivative(double t) const {
    return {2 * ((1 - t) * (control.x - start.x) + t * (end.x - control.x)),
            2 * ((1 - t) * (control.y - start.y) + t * (end.y - control.y))};
  }
  Point2 secondDerivative(double) const {
    return curve_detail::secondDifference(start, control, end, 2);
  }
  DerivativeBounds derivativeBounds() const {
    // Derivative control polygons enclose their entire Bezier curves.
    return {2 * std::max(curve_detail::distance(start, control),
                         curve_detail::distance(control, end)),
            curve_detail::norm(secondDerivative(0))};
  }
  std::pair<QuadraticBezier, QuadraticBezier> split(double t) const {
    curve_detail::validateSplit(t);
    auto a = curve_detail::mix(start, control, t),
         b = curve_detail::mix(control, end, t), p = curve_detail::mix(a, b, t);
    return {{start, a, p}, {p, b, end}};
  }
  QuadraticBezier reversed() const { return {end, control, start}; }
  QuadraticBezier transformed(const CurveTransform &f) const {
    return {f(start), f(control), f(end)};
  }
  void validate() const {
    curve_detail::validatePoint(start);
    curve_detail::validatePoint(control);
    curve_detail::validatePoint(end);
  }
  std::optional<double> exactLength(double) const { return std::nullopt; }
  std::optional<double> exactLengthParameter(double) const {
    return std::nullopt;
  }
  bool isLinear() const { return false; }
};
struct CubicBezier {
  Point2 start, control1, control2, end;
  Point2 point(double t) const {
    const double a = (1 - t) * (1 - t) * (1 - t), b = 3 * t * (1 - t) * (1 - t),
                 d = 3 * t * t * (1 - t), e = t * t * t;
    return {a * start.x + b * control1.x + d * control2.x + e * end.x,
            a * start.y + b * control1.y + d * control2.y + e * end.y};
  }
  Point2 derivative(double t) const {
    const double u = 1 - t;
    return {3 * (u * u * (control1.x - start.x) +
                 2 * u * t * (control2.x - control1.x) +
                 t * t * (end.x - control2.x)),
            3 * (u * u * (control1.y - start.y) +
                 2 * u * t * (control2.y - control1.y) +
                 t * t * (end.y - control2.y))};
  }
  Point2 secondDerivative(double t) const {
    auto a = curve_detail::secondDifference(start, control1, control2, 6),
         b = curve_detail::secondDifference(control1, control2, end, 6);
    return {(1 - t) * a.x + t * b.x, (1 - t) * a.y + t * b.y};
  }
  DerivativeBounds derivativeBounds() const {
    return {3 * std::max({curve_detail::distance(start, control1),
                          curve_detail::distance(control1, control2),
                          curve_detail::distance(control2, end)}),
            std::max(curve_detail::norm(secondDerivative(0)),
                     curve_detail::norm(secondDerivative(1)))};
  }
  std::pair<CubicBezier, CubicBezier> split(double t) const {
    curve_detail::validateSplit(t);
    auto a = curve_detail::mix(start, control1, t),
         b = curve_detail::mix(control1, control2, t),
         d = curve_detail::mix(control2, end, t);
    auto e = curve_detail::mix(a, b, t), f = curve_detail::mix(b, d, t),
         p = curve_detail::mix(e, f, t);
    return {{start, a, e, p}, {p, f, d, end}};
  }
  CubicBezier reversed() const { return {end, control2, control1, start}; }
  CubicBezier transformed(const CurveTransform &f) const {
    return {f(start), f(control1), f(control2), f(end)};
  }
  void validate() const {
    curve_detail::validatePoint(start);
    curve_detail::validatePoint(control1);
    curve_detail::validatePoint(control2);
    curve_detail::validatePoint(end);
  }
  std::optional<double> exactLength(double) const { return std::nullopt; }
  std::optional<double> exactLengthParameter(double) const {
    return std::nullopt;
  }
  bool isLinear() const { return false; }
};
// Center parameterization; angles in radians, including signed sweeps.
struct EllipticalArc {
  Point2 center;
  double radius_x, radius_y, rotation, start_angle, sweep_angle;
  Point2 rotate(Point2 p) const {
    return {std::cos(rotation) * p.x - std::sin(rotation) * p.y,
            std::sin(rotation) * p.x + std::cos(rotation) * p.y};
  }
  Point2 point(double t) const {
    const double a = start_angle + t * sweep_angle;
    auto p = rotate({radius_x * std::cos(a), radius_y * std::sin(a)});
    return {center.x + p.x, center.y + p.y};
  }
  Point2 derivative(double t) const {
    const double a = start_angle + t * sweep_angle;
    return rotate({-radius_x * std::sin(a) * sweep_angle,
                   radius_y * std::cos(a) * sweep_angle});
  }
  Point2 secondDerivative(double t) const {
    const double a = start_angle + t * sweep_angle,
                 w2 = sweep_angle * sweep_angle;
    return rotate({-radius_x * std::cos(a) * w2, -radius_y * std::sin(a) * w2});
  }
  DerivativeBounds derivativeBounds() const {
    const double r = std::max(radius_x, radius_y), w = std::abs(sweep_angle);
    return {r * w, r * w * w};
  }
  std::pair<EllipticalArc, EllipticalArc> split(double t) const {
    curve_detail::validateSplit(t);
    auto a = *this, b = *this;
    a.sweep_angle = sweep_angle * t;
    b.start_angle = start_angle + a.sweep_angle;
    b.sweep_angle = sweep_angle * (1 - t);
    return {a, b};
  }
  EllipticalArc reversed() const {
    auto c = *this;
    c.start_angle += sweep_angle;
    c.sweep_angle = -sweep_angle;
    return c;
  }
  EllipticalArc transformed(const CurveTransform &f) const {
    return {f(center),          radius_x * f.scale,
            radius_y * f.scale, rotation + f.rotation,
            start_angle,        sweep_angle};
  }
  void validate() const {
    curve_detail::validatePoint(center);
    if (!(radius_x > 0 && radius_y > 0) || !std::isfinite(radius_x) ||
        !std::isfinite(radius_y) || !std::isfinite(rotation) ||
        !std::isfinite(start_angle) || !std::isfinite(sweep_angle) ||
        sweep_angle == 0 || std::abs(sweep_angle) > 2 * std::numbers::pi)
      throw std::invalid_argument("Invalid elliptical arc");
  }
  std::optional<double> exactLength(double t) const {
    if (radius_x == radius_y)
      return t * std::abs(sweep_angle) * radius_x;
    return std::nullopt;
  }
  std::optional<double> exactLengthParameter(double f) const {
    if (radius_x == radius_y)
      return f;
    return std::nullopt;
  }
  bool isLinear() const { return false; }
};
using CurveSegment =
    std::variant<LineCurve, QuadraticBezier, CubicBezier, EllipticalArc>;

inline Point2 curvePoint(const CurveSegment &c, double t) {
  return std::visit([t](const auto &s) { return s.point(t); }, c);
}
inline Point2 curveDerivative(const CurveSegment &c, double t) {
  return std::visit([t](const auto &s) { return s.derivative(t); }, c);
}
inline Point2 curveSecondDerivative(const CurveSegment &c, double t) {
  return std::visit([t](const auto &s) { return s.secondDerivative(t); }, c);
}
inline DerivativeBounds curveDerivativeBounds(const CurveSegment &c) {
  return std::visit([](const auto &s) { return s.derivativeBounds(); }, c);
}
// Both exact pieces are reparameterized to [0,1]. On a source interval of
// width h, derivatives scale by h and h^2, respectively.
inline std::pair<CurveSegment, CurveSegment> splitCurve(const CurveSegment &c,
                                                        double t) {
  return std::visit(
      [t](const auto &s) -> std::pair<CurveSegment, CurveSegment> {
        auto p = s.split(t);
        return {p.first, p.second};
      },
      c);
}
inline CurveSegment reverseCurve(const CurveSegment &c) {
  return std::visit([](const auto &s) -> CurveSegment { return s.reversed(); },
                    c);
}
inline CurveSegment transformCurve(const CurveSegment &c,
                                   const CurveTransform &f) {
  f.validate();
  return std::visit(
      [&](const auto &s) -> CurveSegment { return s.transformed(f); }, c);
}
inline bool curveIsLinear(const CurveSegment &c) {
  return std::visit([](const auto &s) { return s.isLinear(); }, c);
}
} // namespace pde2d::discretization

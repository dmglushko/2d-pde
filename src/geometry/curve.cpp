#include "pde2d/geometry/curve.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>

// Ported from the archive (cpp/discretization/curve_segment.hpp and
// curve_length.hpp), with snake_case names and free functions over Curve.
namespace pde2d::geometry {
namespace {

void validate_point(Point p) {
  if (!is_finite(p))
    throw std::invalid_argument("Non-finite curve coordinate");
}

void validate_split(double t) {
  if (!(t > 0 && t < 1) || !std::isfinite(t))
    throw std::invalid_argument("Split parameter must be between 0 and 1");
}

Point mix(Point a, Point b, double t) {
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}

Point second_difference(Point a, Point b, Point c, double factor) {
  return {factor * (a.x - 2 * b.x + c.x), factor * (a.y - 2 * b.y + c.y)};
}

Point rotate(Point p, double angle) {
  const double c = std::cos(angle), s = std::sin(angle);
  return {c * p.x - s * p.y, s * p.x + c * p.y};
}

// --- Line -------------------------------------------------------------------

Point point_of(const Line &c, double t) {
  return {(1 - t) * c.start.x + t * c.end.x, (1 - t) * c.start.y + t * c.end.y};
}
Point first_of(const Line &c, double) { return c.end - c.start; }
Point second_of(const Line &, double) { return {0, 0}; }
DerivativeBounds bounds_of(const Line &c) {
  return {distance(c.start, c.end), 0};
}
std::pair<Curve, Curve> split_of(const Line &c, double t) {
  auto p = mix(c.start, c.end, t);
  return {Line{c.start, p}, Line{p, c.end}};
}
Curve reversed_of(const Line &c) { return Line{c.end, c.start}; }
Curve transformed_of(const Line &c, const CurveTransform &f) {
  return Line{f(c.start), f(c.end)};
}
void validate_of(const Line &c) {
  validate_point(c.start);
  validate_point(c.end);
}
std::optional<double> exact_length(const Line &c, double end) {
  return end * distance(c.start, c.end);
}
std::optional<double> exact_length_parameter(const Line &, double fraction) {
  return fraction;
}

// --- Quadratic Bezier -------------------------------------------------------

Point point_of(const QuadraticBezier &c, double t) {
  const double a = (1 - t) * (1 - t), b = 2 * t * (1 - t), d = t * t;
  return {a * c.start.x + b * c.control.x + d * c.end.x,
          a * c.start.y + b * c.control.y + d * c.end.y};
}
Point first_of(const QuadraticBezier &c, double t) {
  return {2 * ((1 - t) * (c.control.x - c.start.x) +
               t * (c.end.x - c.control.x)),
          2 * ((1 - t) * (c.control.y - c.start.y) +
               t * (c.end.y - c.control.y))};
}
Point second_of(const QuadraticBezier &c, double) {
  return second_difference(c.start, c.control, c.end, 2);
}
DerivativeBounds bounds_of(const QuadraticBezier &c) {
  // The derivative's control polygon encloses the whole derivative curve.
  return {2 * std::max(distance(c.start, c.control),
                       distance(c.control, c.end)),
          norm(second_of(c, 0))};
}
std::pair<Curve, Curve> split_of(const QuadraticBezier &c, double t) {
  auto a = mix(c.start, c.control, t), b = mix(c.control, c.end, t),
       p = mix(a, b, t);
  return {QuadraticBezier{c.start, a, p}, QuadraticBezier{p, b, c.end}};
}
Curve reversed_of(const QuadraticBezier &c) {
  return QuadraticBezier{c.end, c.control, c.start};
}
Curve transformed_of(const QuadraticBezier &c, const CurveTransform &f) {
  return QuadraticBezier{f(c.start), f(c.control), f(c.end)};
}
void validate_of(const QuadraticBezier &c) {
  validate_point(c.start);
  validate_point(c.control);
  validate_point(c.end);
}
std::optional<double> exact_length(const QuadraticBezier &, double) {
  return std::nullopt;
}
std::optional<double> exact_length_parameter(const QuadraticBezier &,
                                             double) {
  return std::nullopt;
}

// --- Cubic Bezier -----------------------------------------------------------

Point point_of(const CubicBezier &c, double t) {
  const double a = (1 - t) * (1 - t) * (1 - t), b = 3 * t * (1 - t) * (1 - t),
               d = 3 * t * t * (1 - t), e = t * t * t;
  return {a * c.start.x + b * c.control1.x + d * c.control2.x + e * c.end.x,
          a * c.start.y + b * c.control1.y + d * c.control2.y + e * c.end.y};
}
Point first_of(const CubicBezier &c, double t) {
  const double u = 1 - t;
  return {3 * (u * u * (c.control1.x - c.start.x) +
               2 * u * t * (c.control2.x - c.control1.x) +
               t * t * (c.end.x - c.control2.x)),
          3 * (u * u * (c.control1.y - c.start.y) +
               2 * u * t * (c.control2.y - c.control1.y) +
               t * t * (c.end.y - c.control2.y))};
}
Point second_of(const CubicBezier &c, double t) {
  auto a = second_difference(c.start, c.control1, c.control2, 6),
       b = second_difference(c.control1, c.control2, c.end, 6);
  return {(1 - t) * a.x + t * b.x, (1 - t) * a.y + t * b.y};
}
DerivativeBounds bounds_of(const CubicBezier &c) {
  return {3 * std::max({distance(c.start, c.control1),
                        distance(c.control1, c.control2),
                        distance(c.control2, c.end)}),
          std::max(norm(second_of(c, 0)), norm(second_of(c, 1)))};
}
std::pair<Curve, Curve> split_of(const CubicBezier &c, double t) {
  auto a = mix(c.start, c.control1, t), b = mix(c.control1, c.control2, t),
       d = mix(c.control2, c.end, t);
  auto e = mix(a, b, t), f = mix(b, d, t), p = mix(e, f, t);
  return {CubicBezier{c.start, a, e, p}, CubicBezier{p, f, d, c.end}};
}
Curve reversed_of(const CubicBezier &c) {
  return CubicBezier{c.end, c.control2, c.control1, c.start};
}
Curve transformed_of(const CubicBezier &c, const CurveTransform &f) {
  return CubicBezier{f(c.start), f(c.control1), f(c.control2), f(c.end)};
}
void validate_of(const CubicBezier &c) {
  validate_point(c.start);
  validate_point(c.control1);
  validate_point(c.control2);
  validate_point(c.end);
}
std::optional<double> exact_length(const CubicBezier &, double) {
  return std::nullopt;
}
std::optional<double> exact_length_parameter(const CubicBezier &, double) {
  return std::nullopt;
}

// --- Elliptical arc ---------------------------------------------------------

Point point_of(const EllipticalArc &c, double t) {
  const double a = c.start_angle + t * c.sweep_angle;
  return c.center +
         rotate({c.radius_x * std::cos(a), c.radius_y * std::sin(a)},
                c.rotation);
}
Point first_of(const EllipticalArc &c, double t) {
  const double a = c.start_angle + t * c.sweep_angle;
  return rotate({-c.radius_x * std::sin(a) * c.sweep_angle,
                 c.radius_y * std::cos(a) * c.sweep_angle},
                c.rotation);
}
Point second_of(const EllipticalArc &c, double t) {
  const double a = c.start_angle + t * c.sweep_angle,
               w2 = c.sweep_angle * c.sweep_angle;
  return rotate({-c.radius_x * std::cos(a) * w2, -c.radius_y * std::sin(a) * w2},
                c.rotation);
}
DerivativeBounds bounds_of(const EllipticalArc &c) {
  const double r = std::max(c.radius_x, c.radius_y),
               w = std::abs(c.sweep_angle);
  return {r * w, r * w * w};
}
std::pair<Curve, Curve> split_of(const EllipticalArc &c, double t) {
  auto a = c, b = c;
  a.sweep_angle = c.sweep_angle * t;
  b.start_angle = c.start_angle + a.sweep_angle;
  b.sweep_angle = c.sweep_angle * (1 - t);
  return {a, b};
}
Curve reversed_of(const EllipticalArc &c) {
  auto r = c;
  r.start_angle += c.sweep_angle;
  r.sweep_angle = -c.sweep_angle;
  return r;
}
Curve transformed_of(const EllipticalArc &c, const CurveTransform &f) {
  return EllipticalArc{f(c.center),          c.radius_x * f.scale,
                       c.radius_y * f.scale, c.rotation + f.rotation,
                       c.start_angle,        c.sweep_angle};
}
void validate_of(const EllipticalArc &c) {
  validate_point(c.center);
  if (!(c.radius_x > 0 && c.radius_y > 0) || !std::isfinite(c.radius_x) ||
      !std::isfinite(c.radius_y) || !std::isfinite(c.rotation) ||
      !std::isfinite(c.start_angle) || !std::isfinite(c.sweep_angle) ||
      c.sweep_angle == 0 || std::abs(c.sweep_angle) > 2 * std::numbers::pi)
    throw std::invalid_argument("Invalid elliptical arc");
}
std::optional<double> exact_length(const EllipticalArc &c, double t) {
  if (c.radius_x == c.radius_y)
    return t * std::abs(c.sweep_angle) * c.radius_x;
  return std::nullopt;
}
std::optional<double> exact_length_parameter(const EllipticalArc &c,
                                             double f) {
  if (c.radius_x == c.radius_y)
    return f;
  return std::nullopt;
}

// --- Length -----------------------------------------------------------------

double speed(const Curve &curve, double t) {
  return norm(derivative_at(curve, t));
}

// Adaptive Simpson integration of the speed. Tolerances scale with the
// curve, not with its position or an absolute floor.
double integrate_length(const Curve &curve, double end) {
  auto f = [&](double t) {
    double s = speed(curve, t);
    if (!std::isfinite(s))
      throw std::invalid_argument(
          "Curve length exceeds supported numeric scale");
    return s;
  };
  auto simpson = [](double a, double b, double fa, double fm, double fb) {
    return (b - a) * (fa / 6 + 2 * fm / 3 + fb / 6);
  };
  // Several initial panels keep coincident samples on a periodic curve
  // from looking like constant speed.
  double result = 0;
  for (int panel = 0; panel < 8; ++panel) {
    double a = end * panel / 8, b = end * (panel + 1) / 8, m = (a + b) / 2;
    double fa = f(a), fm = f(m), fb = f(b), whole = simpson(a, b, fa, fm, fb);
    double tolerance = 1e-11 * (b - a) * std::max({fa, fm, fb});
    auto adaptive = [&](auto &&self, double l, double r, double fl, double fc,
                        double fr, double estimate, double tol,
                        int depth) -> double {
      double mid = (l + r) / 2, f1 = f((l + mid) / 2), f2 = f((mid + r) / 2);
      double left = simpson(l, mid, fl, f1, fc),
             right = simpson(mid, r, fc, f2, fr), sum = left + right;
      if (!std::isfinite(sum))
        throw std::invalid_argument(
            "Curve length exceeds supported numeric scale");
      if (std::abs(sum - estimate) <= 15 * tol)
        return sum + (sum - estimate) / 15;
      if (depth == 0)
        throw std::runtime_error("Curve length integration did not converge");
      return self(self, l, mid, fl, f1, fc, left, tol / 2, depth - 1) +
             self(self, mid, r, fc, f2, fr, right, tol / 2, depth - 1);
    };
    result += adaptive(adaptive, a, b, fa, fm, fb, whole, tolerance, 30);
  }
  return result;
}

} // namespace

void CurveTransform::validate() const {
  if (!(scale > 0) || !std::isfinite(scale) || !std::isfinite(rotation) ||
      !is_finite(translation) || !is_finite(origin))
    throw std::invalid_argument("Invalid curve transform");
}

Point CurveTransform::operator()(Point p) const {
  return translation + scale * rotate(p - origin, rotation);
}

void validate(const Curve &curve) {
  std::visit([](const auto &c) { validate_of(c); }, curve);
}
Point point_at(const Curve &curve, double t) {
  return std::visit([t](const auto &c) { return point_of(c, t); }, curve);
}
Point derivative_at(const Curve &curve, double t) {
  return std::visit([t](const auto &c) { return first_of(c, t); }, curve);
}
Point second_derivative_at(const Curve &curve, double t) {
  return std::visit([t](const auto &c) { return second_of(c, t); }, curve);
}
Point start_point(const Curve &curve) { return point_at(curve, 0.0); }
Point end_point(const Curve &curve) { return point_at(curve, 1.0); }
DerivativeBounds derivative_bounds(const Curve &curve) {
  return std::visit([](const auto &c) { return bounds_of(c); }, curve);
}
bool is_line(const Curve &curve) {
  return std::holds_alternative<Line>(curve);
}

std::pair<Curve, Curve> split(const Curve &curve, double t) {
  validate_split(t);
  return std::visit([t](const auto &c) { return split_of(c, t); }, curve);
}

Curve subcurve(const Curve &curve, double t0, double t1) {
  if (!(t0 >= 0 && t0 < t1 && t1 <= 1))
    throw std::invalid_argument("Subcurve requires 0 <= t0 < t1 <= 1");
  if (const auto *arc = std::get_if<EllipticalArc>(&curve)) {
    auto r = *arc;
    r.start_angle = arc->start_angle + t0 * arc->sweep_angle;
    r.sweep_angle = arc->sweep_angle * (t1 - t0);
    return r;
  }
  Curve piece = curve;
  if (t1 < 1)
    piece = split(piece, t1).first;
  if (t0 > 0)
    piece = split(piece, t0 / t1).second;
  return piece;
}

Curve reversed(const Curve &curve) {
  return std::visit([](const auto &c) { return reversed_of(c); }, curve);
}

Curve transformed(const Curve &curve, const CurveTransform &transform) {
  transform.validate();
  return std::visit(
      [&](const auto &c) { return transformed_of(c, transform); }, curve);
}

double curve_length(const Curve &curve, double end) {
  if (!std::isfinite(end) || end < 0 || end > 1)
    throw std::invalid_argument("Length parameter must be in [0,1]");
  validate(curve);
  if (end == 0)
    return 0;
  auto exact =
      std::visit([end](const auto &c) { return exact_length(c, end); }, curve);
  double result = exact ? *exact : integrate_length(curve, end);
  if (!std::isfinite(result))
    throw std::invalid_argument("Curve length exceeds supported numeric scale");
  return result;
}

double parameter_at_length_fraction(const Curve &curve, double fraction) {
  if (!(fraction > 0 && fraction < 1) || !std::isfinite(fraction))
    throw std::invalid_argument(
        "Length fraction must be strictly between 0 and 1");
  double length = curve_length(curve);
  if (!(length > 0))
    throw std::invalid_argument("Cannot split a zero-length curve");
  auto exact = std::visit(
      [fraction](const auto &c) { return exact_length_parameter(c, fraction); },
      curve);
  if (exact)
    return *exact;
  double low = 0, high = 1;
  const bool from_end = fraction > .5;
  const double target = from_end ? 1 - fraction : fraction;
  double t = fraction;
  for (int i = 0; i < 144; ++i) {
    if (!(t > low && t < high))
      break;
    // Measure the smaller requested portion directly: subtracting nearly
    // equal lengths would lose precision near the end point.
    double ratio = from_end ? curve_length(split(curve, t).second) / length
                            : curve_length(curve, t) / length;
    double roundoff = 4 * std::numeric_limits<double>::epsilon() *
                      std::abs(t) * (speed(curve, t) / length);
    if (std::abs(ratio - target) <= 1e-11 * target + roundoff)
      return t;
    if (from_end ? ratio > target : ratio < target)
      low = t;
    else
      high = t;
    // Newton steps inside the bracket, with periodic bisection; after 16
    // attempts plain bisection.
    double derivative = speed(curve, t) / length;
    double candidate =
        t + (from_end ? ratio - target : target - ratio) / derivative;
    t = (i < 16 && i % 4 != 3 && std::isfinite(candidate) && candidate > low &&
         candidate < high)
            ? candidate
            : (low + high) / 2;
  }
  throw std::runtime_error("Curve length inversion did not converge");
}

} // namespace pde2d::geometry

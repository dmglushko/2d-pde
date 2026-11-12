#include "pde2d/geometry/curve.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>
#include <variant>
#include <vector>

// The formulas are ported from the archive
// (cpp/discretization/curve_segment.hpp and curve_length.hpp). Each kind's
// methods are together in its own section.
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

void validate_subcurve(double t0, double t1) {
  if (!(t0 >= 0 && t0 < t1 && t1 <= 1))
    throw std::invalid_argument("Subcurve requires 0 <= t0 < t1 <= 1");
}

// The piece between t0 and t1 of a kind that can be split anywhere.
template <class Kind> Kind subcurve_by_split(Kind piece, double t0, double t1) {
  validate_subcurve(t0, t1);
  if (t1 < 1)
    piece = piece.split(t1).first;
  if (t0 > 0)
    piece = piece.split(t0 / t1).second;
  return piece;
}

// Adds the kind's point at t to the box if t lies strictly inside (0, 1).
template <class Kind>
void add_interior(BoundingBox &box, const Kind &kind, double t) {
  if (t > 0 && t < 1)
    box = merged(box, kind.point_at(t));
}

template <class Kind> BoundingBox box_of_ends(const Kind &kind) {
  const Point a = kind.point_at(0.0);
  return merged(BoundingBox{a, a}, kind.point_at(1.0));
}

// Roots of a t^2 + b t + c (a linear or constant equation when a vanishes).
std::vector<double> quadratic_roots(double a, double b, double c) {
  std::vector<double> roots;
  if (a == 0) {
    if (b != 0)
      roots.push_back(-c / b);
    return roots;
  }
  const double disc = b * b - 4 * a * c;
  if (disc < 0)
    return roots;
  // The numerically stable form: one root from q, the other from c / q.
  const double q = -0.5 * (b + std::copysign(std::sqrt(disc), b));
  if (q != 0) {
    roots.push_back(q / a);
    roots.push_back(c / q);
  } else {
    roots.push_back(0.0); // b = 0 and disc = 0, so c = 0: a double root at 0
  }
  return roots;
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

// --- Line -------------------------------------------------------------------

Point Line::point_at(double t) const {
  return {(1 - t) * start.x + t * end.x, (1 - t) * start.y + t * end.y};
}
Point Line::derivative_at(double) const { return end - start; }
Point Line::second_derivative_at(double) const { return {0, 0}; }
DerivativeBounds Line::derivative_bounds() const {
  return {distance(start, end), 0};
}
std::pair<Line, Line> Line::split(double t) const {
  validate_split(t);
  auto p = mix(start, end, t);
  return {Line{start, p}, Line{p, end}};
}
Line Line::reversed() const { return Line{end, start}; }
Line Line::transformed(const CurveTransform &f) const {
  f.validate();
  return Line{f(start), f(end)};
}
void Line::validate() const {
  validate_point(start);
  validate_point(end);
}
std::optional<double> Line::exact_length(double to) const {
  return to * distance(start, end);
}
std::optional<double> Line::exact_length_parameter(double fraction) const {
  return fraction;
}
Line Line::subcurve(double t0, double t1) const {
  return subcurve_by_split(*this, t0, t1);
}
BoundingBox Line::bounding_box() const { return box_of_ends(*this); }

// --- Quadratic Bezier -------------------------------------------------------

Point QuadraticBezier::point_at(double t) const {
  const double a = (1 - t) * (1 - t), b = 2 * t * (1 - t), d = t * t;
  return {a * start.x + b * control.x + d * end.x,
          a * start.y + b * control.y + d * end.y};
}
Point QuadraticBezier::derivative_at(double t) const {
  return {2 * ((1 - t) * (control.x - start.x) + t * (end.x - control.x)),
          2 * ((1 - t) * (control.y - start.y) + t * (end.y - control.y))};
}
Point QuadraticBezier::second_derivative_at(double) const {
  return second_difference(start, control, end, 2);
}
DerivativeBounds QuadraticBezier::derivative_bounds() const {
  // The derivative's control polygon encloses the whole derivative curve.
  return {2 * std::max(distance(start, control), distance(control, end)),
          norm(second_derivative_at(0))};
}
std::pair<QuadraticBezier, QuadraticBezier>
QuadraticBezier::split(double t) const {
  validate_split(t);
  auto a = mix(start, control, t), b = mix(control, end, t), p = mix(a, b, t);
  return {QuadraticBezier{start, a, p}, QuadraticBezier{p, b, end}};
}
QuadraticBezier QuadraticBezier::reversed() const {
  return QuadraticBezier{end, control, start};
}
QuadraticBezier QuadraticBezier::transformed(const CurveTransform &f) const {
  f.validate();
  return QuadraticBezier{f(start), f(control), f(end)};
}
void QuadraticBezier::validate() const {
  validate_point(start);
  validate_point(control);
  validate_point(end);
}
std::optional<double> QuadraticBezier::exact_length(double) const {
  return std::nullopt;
}
std::optional<double> QuadraticBezier::exact_length_parameter(double) const {
  return std::nullopt;
}
QuadraticBezier QuadraticBezier::subcurve(double t0, double t1) const {
  return subcurve_by_split(*this, t0, t1);
}
BoundingBox QuadraticBezier::bounding_box() const {
  BoundingBox box = box_of_ends(*this);
  // x'(t) is linear: 2 [(p1 - p0) + t (p0 - 2 p1 + p2)].
  auto axis = [&](double p0, double p1, double p2) {
    const double d = p0 - 2 * p1 + p2;
    if (d != 0)
      add_interior(box, *this, (p0 - p1) / d);
  };
  axis(start.x, control.x, end.x);
  axis(start.y, control.y, end.y);
  return box;
}

// --- Cubic Bezier -----------------------------------------------------------

Point CubicBezier::point_at(double t) const {
  const double a = (1 - t) * (1 - t) * (1 - t), b = 3 * t * (1 - t) * (1 - t),
               d = 3 * t * t * (1 - t), e = t * t * t;
  return {a * start.x + b * control1.x + d * control2.x + e * end.x,
          a * start.y + b * control1.y + d * control2.y + e * end.y};
}
Point CubicBezier::derivative_at(double t) const {
  const double u = 1 - t;
  return {3 * (u * u * (control1.x - start.x) +
               2 * u * t * (control2.x - control1.x) +
               t * t * (end.x - control2.x)),
          3 * (u * u * (control1.y - start.y) +
               2 * u * t * (control2.y - control1.y) +
               t * t * (end.y - control2.y))};
}
Point CubicBezier::second_derivative_at(double t) const {
  auto a = second_difference(start, control1, control2, 6),
       b = second_difference(control1, control2, end, 6);
  return {(1 - t) * a.x + t * b.x, (1 - t) * a.y + t * b.y};
}
DerivativeBounds CubicBezier::derivative_bounds() const {
  return {
      3 * std::max({distance(start, control1), distance(control1, control2),
                    distance(control2, end)}),
      std::max(norm(second_derivative_at(0)), norm(second_derivative_at(1)))};
}
std::pair<CubicBezier, CubicBezier> CubicBezier::split(double t) const {
  validate_split(t);
  auto a = mix(start, control1, t), b = mix(control1, control2, t),
       d = mix(control2, end, t);
  auto e = mix(a, b, t), f = mix(b, d, t), p = mix(e, f, t);
  return {CubicBezier{start, a, e, p}, CubicBezier{p, f, d, end}};
}
CubicBezier CubicBezier::reversed() const {
  return CubicBezier{end, control2, control1, start};
}
CubicBezier CubicBezier::transformed(const CurveTransform &f) const {
  f.validate();
  return CubicBezier{f(start), f(control1), f(control2), f(end)};
}
void CubicBezier::validate() const {
  validate_point(start);
  validate_point(control1);
  validate_point(control2);
  validate_point(end);
}
std::optional<double> CubicBezier::exact_length(double) const {
  return std::nullopt;
}
std::optional<double> CubicBezier::exact_length_parameter(double) const {
  return std::nullopt;
}
CubicBezier CubicBezier::subcurve(double t0, double t1) const {
  return subcurve_by_split(*this, t0, t1);
}
BoundingBox CubicBezier::bounding_box() const {
  BoundingBox box = box_of_ends(*this);
  // x'(t) / 3 = a t^2 + b t + k with the coefficients below.
  auto axis = [&](double p0, double p1, double p2, double p3) {
    const double a = -p0 + 3 * p1 - 3 * p2 + p3;
    const double b = 2 * (p0 - 2 * p1 + p2);
    const double k = p1 - p0;
    for (double t : quadratic_roots(a, b, k))
      add_interior(box, *this, t);
  };
  axis(start.x, control1.x, control2.x, end.x);
  axis(start.y, control1.y, control2.y, end.y);
  return box;
}

// --- Elliptical arc ---------------------------------------------------------

Point EllipticalArc::point_at(double t) const {
  const double a = start_angle + t * sweep_angle;
  return center +
         rotate({radius_x * std::cos(a), radius_y * std::sin(a)}, rotation);
}
Point EllipticalArc::derivative_at(double t) const {
  const double a = start_angle + t * sweep_angle;
  return rotate({-radius_x * std::sin(a) * sweep_angle,
                 radius_y * std::cos(a) * sweep_angle},
                rotation);
}
Point EllipticalArc::second_derivative_at(double t) const {
  const double a = start_angle + t * sweep_angle,
               w2 = sweep_angle * sweep_angle;
  return rotate({-radius_x * std::cos(a) * w2, -radius_y * std::sin(a) * w2},
                rotation);
}
DerivativeBounds EllipticalArc::derivative_bounds() const {
  const double r = std::max(radius_x, radius_y), w = std::abs(sweep_angle);
  return {r * w, r * w * w};
}
std::pair<EllipticalArc, EllipticalArc> EllipticalArc::split(double t) const {
  validate_split(t);
  auto a = *this, b = *this;
  a.sweep_angle = sweep_angle * t;
  b.start_angle = start_angle + a.sweep_angle;
  b.sweep_angle = sweep_angle * (1 - t);
  return {a, b};
}
EllipticalArc EllipticalArc::reversed() const {
  auto r = *this;
  r.start_angle += sweep_angle;
  r.sweep_angle = -sweep_angle;
  return r;
}
EllipticalArc EllipticalArc::transformed(const CurveTransform &f) const {
  f.validate();
  return EllipticalArc{f(center),          radius_x * f.scale,
                       radius_y * f.scale, rotation + f.rotation,
                       start_angle,        sweep_angle};
}
void EllipticalArc::validate() const {
  validate_point(center);
  if (!(radius_x > 0 && radius_y > 0) || !std::isfinite(radius_x) ||
      !std::isfinite(radius_y) || !std::isfinite(rotation) ||
      !std::isfinite(start_angle) || !std::isfinite(sweep_angle) ||
      sweep_angle == 0 || std::abs(sweep_angle) > 2 * std::numbers::pi)
    throw std::invalid_argument("Invalid elliptical arc");
}
std::optional<double> EllipticalArc::exact_length(double t) const {
  if (radius_x == radius_y)
    return t * std::abs(sweep_angle) * radius_x;
  return std::nullopt;
}
std::optional<double> EllipticalArc::exact_length_parameter(double f) const {
  if (radius_x == radius_y)
    return f;
  return std::nullopt;
}
EllipticalArc EllipticalArc::subcurve(double t0, double t1) const {
  validate_subcurve(t0, t1);
  auto r = *this;
  r.start_angle = start_angle + t0 * sweep_angle;
  r.sweep_angle = sweep_angle * (t1 - t0);
  return r;
}
BoundingBox EllipticalArc::bounding_box() const {
  BoundingBox box = box_of_ends(*this);
  // x(a) = cx + rx cos(a) cos(phi) - ry sin(a) sin(phi) is extreme where
  // tan(a) = -ry sin(phi) / (rx cos(phi)); y likewise. Each gives two
  // angles, half a turn apart.
  const double cp = std::cos(rotation), sp = std::sin(rotation);
  const double angles[] = {std::atan2(-radius_y * sp, radius_x * cp),
                           std::atan2(radius_y * cp, radius_x * sp)};
  const double turn = 2 * std::numbers::pi;
  for (double base : angles)
    for (double a : {base, base + std::numbers::pi}) {
      // The parameter t with start + t * sweep = a (mod a full turn).
      double offset = std::fmod(a - start_angle, turn);
      if (sweep_angle < 0)
        offset = -offset;
      if (offset < 0)
        offset += turn;
      add_interior(box, *this, offset / std::abs(sweep_angle));
    }
  return box;
}

// --- Curve ------------------------------------------------------------------

void Curve::validate() const {
  std::visit([](const auto &k) { k.validate(); }, kind_);
}
Point Curve::point_at(double t) const {
  return std::visit([t](const auto &k) { return k.point_at(t); }, kind_);
}
Point Curve::derivative_at(double t) const {
  return std::visit([t](const auto &k) { return k.derivative_at(t); }, kind_);
}
Point Curve::second_derivative_at(double t) const {
  return std::visit([t](const auto &k) { return k.second_derivative_at(t); },
                    kind_);
}
DerivativeBounds Curve::derivative_bounds() const {
  return std::visit([](const auto &k) { return k.derivative_bounds(); }, kind_);
}
std::pair<Curve, Curve> Curve::split(double t) const {
  return std::visit(
      [t](const auto &k) {
        auto [first, second] = k.split(t);
        return std::pair<Curve, Curve>{first, second};
      },
      kind_);
}
Curve Curve::subcurve(double t0, double t1) const {
  return std::visit([&](const auto &k) { return Curve{k.subcurve(t0, t1)}; },
                    kind_);
}
Curve Curve::reversed() const {
  return std::visit([](const auto &k) { return Curve{k.reversed()}; }, kind_);
}
Curve Curve::transformed(const CurveTransform &transform) const {
  return std::visit(
      [&](const auto &k) { return Curve{k.transformed(transform)}; }, kind_);
}
BoundingBox Curve::bounding_box() const {
  return std::visit([](const auto &k) { return k.bounding_box(); }, kind_);
}
std::optional<double> Curve::exact_length(double end) const {
  return std::visit([end](const auto &k) { return k.exact_length(end); },
                    kind_);
}
std::optional<double> Curve::exact_length_parameter(double fraction) const {
  return std::visit(
      [fraction](const auto &k) { return k.exact_length_parameter(fraction); },
      kind_);
}

// --- Functions built on the core ---------------------------------------------

Point start_point(const Curve &curve) { return curve.point_at(0.0); }
Point end_point(const Curve &curve) { return curve.point_at(1.0); }
bool is_line(const Curve &curve) {
  return std::holds_alternative<Line>(curve.kind());
}

BoundingBox bounding_box(std::span<const Curve> curves) {
  if (curves.empty())
    throw std::invalid_argument("bounding_box: no curves");
  BoundingBox box = curves.front().bounding_box();
  for (const auto &c : curves)
    box = merged(box, c.bounding_box());
  return box;
}

namespace {

double speed(const Curve &curve, double t) {
  return norm(curve.derivative_at(t));
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

double curve_length(const Curve &curve, double end) {
  if (!std::isfinite(end) || end < 0 || end > 1)
    throw std::invalid_argument("Length parameter must be in [0,1]");
  curve.validate();
  if (end == 0)
    return 0;
  auto exact = curve.exact_length(end);
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
  auto exact = curve.exact_length_parameter(fraction);
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
    double ratio = from_end ? curve_length(curve.split(t).second) / length
                            : curve_length(curve, t) / length;
    double roundoff = 4 * std::numeric_limits<double>::epsilon() * std::abs(t) *
                      (speed(curve, t) / length);
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

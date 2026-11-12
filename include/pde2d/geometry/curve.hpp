#pragma once

#include "pde2d/geometry/bounding_box.hpp"
#include "pde2d/geometry/point.hpp"

#include <concepts>
#include <optional>
#include <span>
#include <utility>
#include <variant>

// Exact boundary curves. Every curve is parameterized over t in [0, 1] and
// runs from point_at(0) to point_at(1).
//
// There are four kinds of curve (Line, QuadraticBezier, CubicBezier,
// EllipticalArc). Each kind owns its mathematics as methods: the core
// listed in the CurveKind concept. A Curve holds one kind by value and has
// the same methods. Everything that can be computed from the core (length,
// end points, ...) is a function written once, below the Curve class.
namespace pde2d::geometry {

// Upper bounds on |c'(t)| and |c''(t)| over the whole interval [0, 1].
// They are analytic bounds, not sampled estimates.
struct DerivativeBounds {
  double first = 0.0, second = 0.0;
};

// A similarity transform: subtract origin, scale by a positive factor,
// rotate counterclockwise by rotation (radians), then translate. General
// affine maps are excluded because they would turn circles into ellipses
// with a different parameterization.
struct CurveTransform {
  double scale = 1.0;
  double rotation = 0.0;
  Point translation{};
  Point origin{};

  void validate() const; // throws std::invalid_argument
  Point operator()(Point p) const;
};

// The core methods, the same for every kind (K stands for the kind):
//
//   void validate() const;
//     Throws std::invalid_argument for non-finite data, non-positive radii,
//     a zero sweep or a sweep larger than a full turn.
//   Point point_at(double t) const;
//   Point derivative_at(double t) const;
//   Point second_derivative_at(double t) const;
//   DerivativeBounds derivative_bounds() const;
//   std::pair<K, K> split(double t) const;
//     Splits at t in (0, 1). Both pieces are exact curves of the same
//     kind, each reparameterized to [0, 1].
//   K subcurve(double t0, double t1) const;
//     The piece between 0 <= t0 < t1 <= 1, reparameterized to [0, 1].
//   K reversed() const;
//     The same curve traversed from end to start.
//   K transformed(const CurveTransform &transform) const;
//   BoundingBox bounding_box() const;
//     The smallest box containing the curve: its end points and the points
//     where x' or y' vanishes (roots of the Bezier derivatives, the axis
//     extremes of an arc), each evaluated with point_at.
//   std::optional<double> exact_length(double end) const;
//     The arc length from 0 to `end` where the kind has a closed formula
//     for it (lines, circular arcs); empty otherwise.
//   std::optional<double> exact_length_parameter(double fraction) const;
//     The parameter at that fraction of the length, where it is known in
//     closed form; empty otherwise.

// A straight segment.
struct Line {
  Point start, end;

  void validate() const;
  Point point_at(double t) const;
  Point derivative_at(double t) const;
  Point second_derivative_at(double t) const;
  DerivativeBounds derivative_bounds() const;
  std::pair<Line, Line> split(double t) const;
  Line subcurve(double t0, double t1) const;
  Line reversed() const;
  Line transformed(const CurveTransform &transform) const;
  BoundingBox bounding_box() const;
  std::optional<double> exact_length(double end) const;
  std::optional<double> exact_length_parameter(double fraction) const;
};

struct QuadraticBezier {
  Point start, control, end;

  void validate() const;
  Point point_at(double t) const;
  Point derivative_at(double t) const;
  Point second_derivative_at(double t) const;
  DerivativeBounds derivative_bounds() const;
  std::pair<QuadraticBezier, QuadraticBezier> split(double t) const;
  QuadraticBezier subcurve(double t0, double t1) const;
  QuadraticBezier reversed() const;
  QuadraticBezier transformed(const CurveTransform &transform) const;
  BoundingBox bounding_box() const;
  std::optional<double> exact_length(double end) const;
  std::optional<double> exact_length_parameter(double fraction) const;
};

struct CubicBezier {
  Point start, control1, control2, end;

  void validate() const;
  Point point_at(double t) const;
  Point derivative_at(double t) const;
  Point second_derivative_at(double t) const;
  DerivativeBounds derivative_bounds() const;
  std::pair<CubicBezier, CubicBezier> split(double t) const;
  CubicBezier subcurve(double t0, double t1) const;
  CubicBezier reversed() const;
  CubicBezier transformed(const CurveTransform &transform) const;
  BoundingBox bounding_box() const;
  std::optional<double> exact_length(double end) const;
  std::optional<double> exact_length_parameter(double fraction) const;
};

// Center parameterization: the point at t is
//   center + R(rotation) (radius_x cos a, radius_y sin a),
//   a = start_angle + t * sweep_angle.
// Angles are in radians; a negative sweep runs clockwise. A sweep of 2*pi
// is a full (closed) ellipse.
struct EllipticalArc {
  Point center;
  double radius_x = 1.0, radius_y = 1.0, rotation = 0.0;
  double start_angle = 0.0, sweep_angle = 0.0;

  void validate() const;
  Point point_at(double t) const;
  Point derivative_at(double t) const;
  Point second_derivative_at(double t) const;
  DerivativeBounds derivative_bounds() const;
  std::pair<EllipticalArc, EllipticalArc> split(double t) const;
  EllipticalArc subcurve(double t0, double t1) const;
  EllipticalArc reversed() const;
  EllipticalArc transformed(const CurveTransform &transform) const;
  BoundingBox bounding_box() const;
  std::optional<double> exact_length(double end) const;
  std::optional<double> exact_length_parameter(double fraction) const;
};

// What every kind of curve provides. A kind that misses a method does not
// compile (see the static_assert below).
template <class K>
concept CurveKind =
    requires(const K &c, double t, const CurveTransform &transform) {
      { c.validate() } -> std::same_as<void>;
      { c.point_at(t) } -> std::same_as<Point>;
      { c.derivative_at(t) } -> std::same_as<Point>;
      { c.second_derivative_at(t) } -> std::same_as<Point>;
      { c.derivative_bounds() } -> std::same_as<DerivativeBounds>;
      { c.split(t) } -> std::same_as<std::pair<K, K>>;
      { c.subcurve(t, t) } -> std::same_as<K>;
      { c.reversed() } -> std::same_as<K>;
      { c.transformed(transform) } -> std::same_as<K>;
      { c.bounding_box() } -> std::same_as<BoundingBox>;
      { c.exact_length(t) } -> std::same_as<std::optional<double>>;
      { c.exact_length_parameter(t) } -> std::same_as<std::optional<double>>;
    };

// Combines one handler per kind into the visitor for std::visit:
//   std::visit(overloaded{[](const Line &c) { ... },
//                         [](const QuadraticBezier &c) { ... }, ...},
//              curve.kind());
template <class... Handlers> struct overloaded : Handlers... {
  using Handlers::operator()...;
};

// One curve of any kind, held by value. Its methods are the core methods,
// forwarded to the kind it holds. A kind converts to a Curve implicitly:
// Curve c = Line{{0, 0}, {1, 0}};
class Curve {
public:
  using Kind = std::variant<Line, QuadraticBezier, CubicBezier, EllipticalArc>;

  template <CurveKind K> Curve(K kind) : kind_(std::move(kind)) {}

  // The kind held, for code that differs by kind and belongs to another
  // module (for example writing SVG path commands). Use std::visit with
  // one handler per kind and no catch-all, so that a new kind does not
  // compile until it is handled.
  const Kind &kind() const { return kind_; }

  void validate() const;
  Point point_at(double t) const;
  Point derivative_at(double t) const;
  Point second_derivative_at(double t) const;
  DerivativeBounds derivative_bounds() const;
  std::pair<Curve, Curve> split(double t) const;
  Curve subcurve(double t0, double t1) const;
  Curve reversed() const;
  Curve transformed(const CurveTransform &transform) const;
  BoundingBox bounding_box() const;
  std::optional<double> exact_length(double end) const;
  std::optional<double> exact_length_parameter(double fraction) const;

private:
  Kind kind_;
};

static_assert(CurveKind<Line> && CurveKind<QuadraticBezier> &&
              CurveKind<CubicBezier> && CurveKind<EllipticalArc>);

// Functions built on the core: written once, for every kind.

Point start_point(const Curve &curve);
Point end_point(const Curve &curve);
// True if the curve is of the kind Line.
bool is_line(const Curve &curve);

// The smallest box containing every curve. Throws std::invalid_argument
// for an empty list.
BoundingBox bounding_box(std::span<const Curve> curves);

// Arc length from t = 0 to t = end. Exact where the kind has a closed
// formula (lines, circular arcs), adaptive Simpson integration (relative
// tolerance about 1e-11) otherwise.
double curve_length(const Curve &curve, double end = 1.0);
// The parameter t at which the length from the start is the given fraction
// (strictly between 0 and 1) of the total length.
double parameter_at_length_fraction(const Curve &curve, double fraction);

} // namespace pde2d::geometry

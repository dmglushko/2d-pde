#pragma once

#include "pde2d/geometry/point.hpp"

#include <utility>
#include <variant>

// Exact boundary curves. Every curve is parameterized over t in [0, 1] and
// runs from point_at(c, 0) to point_at(c, 1). Ported from the archive
// (cpp/discretization/curve_segment.hpp and curve_length.hpp).
namespace pde2d::geometry {

// A straight segment.
struct Line {
  Point start, end;
};

struct QuadraticBezier {
  Point start, control, end;
};

struct CubicBezier {
  Point start, control1, control2, end;
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
};

using Curve = std::variant<Line, QuadraticBezier, CubicBezier, EllipticalArc>;

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

// Throws std::invalid_argument for non-finite data, non-positive radii, a
// zero sweep or a sweep larger than a full turn.
void validate(const Curve &curve);

Point point_at(const Curve &curve, double t);
Point derivative_at(const Curve &curve, double t);
Point second_derivative_at(const Curve &curve, double t);
Point start_point(const Curve &curve);
Point end_point(const Curve &curve);
DerivativeBounds derivative_bounds(const Curve &curve);
bool is_line(const Curve &curve);

// Splits at parameter t in (0, 1). Both pieces are exact curves of the
// same kind, each reparameterized to [0, 1].
std::pair<Curve, Curve> split(const Curve &curve, double t);
// The piece between parameters 0 <= t0 < t1 <= 1, reparameterized to [0, 1].
Curve subcurve(const Curve &curve, double t0, double t1);
// The same curve traversed from end to start.
Curve reversed(const Curve &curve);
Curve transformed(const Curve &curve, const CurveTransform &transform);

// Arc length from t = 0 to t = end. Exact for lines and circular arcs,
// adaptive Simpson integration (relative tolerance about 1e-11) otherwise.
double curve_length(const Curve &curve, double end = 1.0);
// The parameter t at which the length from the start is the given fraction
// (strictly between 0 and 1) of the total length.
double parameter_at_length_fraction(const Curve &curve, double fraction);

} // namespace pde2d::geometry

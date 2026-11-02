#pragma once

#include "geometry_base.hpp"
#include <algorithm>
#include <variant>
#include <type_traits>

namespace discretization {

// Elliptical arcs use a center parameterization, with angles in radians.
struct LineCurve { Point2 start, end; };
struct QuadraticBezier { Point2 start, control, end; };
struct CubicBezier { Point2 start, control1, control2, end; };
struct EllipticalArc { Point2 center; double radius_x, radius_y, rotation, start_angle, sweep_angle; };
using CurveSegment = std::variant<LineCurve, QuadraticBezier, CubicBezier, EllipticalArc>;

inline Point2 curvePoint(const CurveSegment& curve, double t) {
  return std::visit([t](const auto& c) -> Point2 {
    using T = std::decay_t<decltype(c)>;
    if constexpr(std::is_same_v<T,LineCurve>)
      return {(1-t)*c.start.x+t*c.end.x,(1-t)*c.start.y+t*c.end.y};
    else if constexpr(std::is_same_v<T,QuadraticBezier>) {
      const double a=(1-t)*(1-t), b=2*t*(1-t), d=t*t;
      return {a*c.start.x+b*c.control.x+d*c.end.x,a*c.start.y+b*c.control.y+d*c.end.y};
    } else if constexpr(std::is_same_v<T,CubicBezier>) {
      const double a=(1-t)*(1-t)*(1-t), b=3*t*(1-t)*(1-t), d=3*t*t*(1-t), e=t*t*t;
      return {a*c.start.x+b*c.control1.x+d*c.control2.x+e*c.end.x,a*c.start.y+b*c.control1.y+d*c.control2.y+e*c.end.y};
    } else {
      const double a=c.start_angle+t*c.sweep_angle, x=c.radius_x*std::cos(a), y=c.radius_y*std::sin(a);
      return {c.center.x+std::cos(c.rotation)*x-std::sin(c.rotation)*y,c.center.y+std::sin(c.rotation)*x+std::cos(c.rotation)*y};
    }
  },curve);
}

// Split without approximating the original curve (de Casteljau for Beziers).
inline std::pair<CurveSegment,CurveSegment> splitCurve(const CurveSegment& curve, double t) {
  if(!(t>0 && t<1) || !std::isfinite(t)) throw std::invalid_argument("Split parameter must be between 0 and 1");
  auto mix=[t](Point2 a,Point2 b) { return Point2{a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t}; };
  return std::visit([&](const auto& c)->std::pair<CurveSegment,CurveSegment> {
    using T=std::decay_t<decltype(c)>;
    if constexpr(std::is_same_v<T,LineCurve>) { auto p=mix(c.start,c.end); return {LineCurve{c.start,p},LineCurve{p,c.end}}; }
    else if constexpr(std::is_same_v<T,QuadraticBezier>) {
      auto a=mix(c.start,c.control), b=mix(c.control,c.end), p=mix(a,b);
      return {QuadraticBezier{c.start,a,p},QuadraticBezier{p,b,c.end}};
    } else if constexpr(std::is_same_v<T,CubicBezier>) {
      auto a=mix(c.start,c.control1), b=mix(c.control1,c.control2), d=mix(c.control2,c.end);
      auto e=mix(a,b), f=mix(b,d), p=mix(e,f);
      return {CubicBezier{c.start,a,e,p},CubicBezier{p,f,d,c.end}};
    } else {
      auto a=c,b=c; a.sweep_angle=c.sweep_angle*t;
      b.start_angle=c.start_angle+a.sweep_angle; b.sweep_angle=c.sweep_angle*(1-t);
      return {a,b};
    }
  },curve);
}

} // namespace discretization

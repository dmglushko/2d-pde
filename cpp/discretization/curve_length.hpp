#pragma once

#include "curve_segment.hpp"
#include <limits>

namespace discretization {
namespace curve_detail {

inline void validateCurve(const CurveSegment& segment) {
  auto point=[](Point2 p) {
    if(!std::isfinite(p.x)||!std::isfinite(p.y)) throw std::invalid_argument("Non-finite curve coordinate");
  };
  std::visit([&](const auto& c) {
    using T=std::decay_t<decltype(c)>;
    if constexpr(std::is_same_v<T,EllipticalArc>) {
      point(c.center);
      if(!(c.radius_x>0 && c.radius_y>0) || !std::isfinite(c.radius_x) || !std::isfinite(c.radius_y) ||
         !std::isfinite(c.rotation) || !std::isfinite(c.start_angle) || !std::isfinite(c.sweep_angle) ||
         c.sweep_angle==0 || std::abs(c.sweep_angle)>2*std::numbers::pi)
        throw std::invalid_argument("Invalid elliptical arc");
    } else {
      point(c.start); point(c.end);
      if constexpr(std::is_same_v<T,QuadraticBezier>) point(c.control);
      if constexpr(std::is_same_v<T,CubicBezier>) { point(c.control1); point(c.control2); }
    }
  },segment);
}

inline double speed(const CurveSegment& segment,double t) {
  return std::visit([t](const auto& c) {
    using T=std::decay_t<decltype(c)>;
    if constexpr(std::is_same_v<T,EllipticalArc>) {
      double a=c.start_angle+t*c.sweep_angle;
      return std::abs(c.sweep_angle)*std::hypot(c.radius_x*std::sin(a),c.radius_y*std::cos(a));
    } else if constexpr(std::is_same_v<T,LineCurve>) {
      return std::hypot(c.end.x-c.start.x,c.end.y-c.start.y);
    } else if constexpr(std::is_same_v<T,QuadraticBezier>) {
      return 2*std::hypot((1-t)*(c.control.x-c.start.x)+t*(c.end.x-c.control.x),
                          (1-t)*(c.control.y-c.start.y)+t*(c.end.y-c.control.y));
    } else {
      double u=1-t;
      return 3*std::hypot(u*u*(c.control1.x-c.start.x)+2*u*t*(c.control2.x-c.control1.x)+t*t*(c.end.x-c.control2.x),
                          u*u*(c.control1.y-c.start.y)+2*u*t*(c.control2.y-c.control1.y)+t*t*(c.end.y-c.control2.y));
    }
  },segment);
}

// Adaptive Simpson integration of speed. Tolerances scale with the curve,
// rather than with its position or an absolute world-unit floor.
inline double integrate(const CurveSegment& segment,double end) {
  auto f=[&](double t) {
    double s=speed(segment,t);
    if(!std::isfinite(s)) throw std::invalid_argument("Curve length exceeds supported numeric scale");
    return s;
  };
  auto simpson=[](double a,double b,double fa,double fm,double fb) { return (b-a)*(fa/6+2*fm/3+fb/6); };
  // Initial panels avoid treating coincident samples on a periodic curve as
  // evidence of constant speed.
  double result=0;
  for(int panel=0;panel<8;++panel) {
    double a=end*panel/8,b=end*(panel+1)/8,m=(a+b)/2;
    double fa=f(a),fm=f(m),fb=f(b),whole=simpson(a,b,fa,fm,fb);
    double tolerance=1e-11*(b-a)*std::max({fa,fm,fb});
    auto adaptive=[&](auto&& self,double l,double r,double fl,double fc,double fr,double estimate,double tol,int depth)->double {
      double mid=(l+r)/2, f1=f((l+mid)/2),f2=f((mid+r)/2);
      double left=simpson(l,mid,fl,f1,fc),right=simpson(mid,r,fc,f2,fr),sum=left+right;
      if(!std::isfinite(sum)) throw std::invalid_argument("Curve length exceeds supported numeric scale");
      if(std::abs(sum-estimate)<=15*tol) return sum+(sum-estimate)/15;
      if(depth==0) throw std::runtime_error("Curve length integration did not converge");
      return self(self,l,mid,fl,f1,fc,left,tol/2,depth-1)+self(self,mid,r,fc,f2,fr,right,tol/2,depth-1);
    };
    result+=adaptive(adaptive,a,b,fa,fm,fb,whole,tolerance,30);
  }
  if(!std::isfinite(result)) throw std::invalid_argument("Curve length exceeds supported numeric scale");
  return result;
}
} // namespace curve_detail

inline double curveLength(const CurveSegment& segment,double end=1) {
  if(!std::isfinite(end)||end<0||end>1) throw std::invalid_argument("Length parameter must be in [0,1]");
  curve_detail::validateCurve(segment);
  if(end==0) return 0;
  double result;
  if(const auto* line=std::get_if<LineCurve>(&segment)) result=end*std::hypot(line->end.x-line->start.x,line->end.y-line->start.y);
  else if(const auto* arc=std::get_if<EllipticalArc>(&segment); arc && arc->radius_x==arc->radius_y)
    result=end*std::abs(arc->sweep_angle)*arc->radius_x;
  else result=curve_detail::integrate(segment,end);
  if(!std::isfinite(result)) throw std::invalid_argument("Curve length exceeds supported numeric scale");
  return result;
}

inline double curveParameterAtLengthFraction(const CurveSegment& segment,double fraction) {
  if(!(fraction>0 && fraction<1)||!std::isfinite(fraction)) throw std::invalid_argument("Length fraction must be strictly between 0 and 1");
  double length=curveLength(segment);
  if(!(length>0)) throw std::invalid_argument("Cannot split a zero-length curve");
  if(std::holds_alternative<LineCurve>(segment)) return fraction;
  if(const auto* arc=std::get_if<EllipticalArc>(&segment); arc && arc->radius_x==arc->radius_y) return fraction;
  double low=0,high=1;
  const bool fromEnd=fraction>.5;
  const double target=fromEnd?1-fraction:fraction;
  for(int i=0;i<128;++i) {
    double t=(low+high)/2;
    if(!(t>low && t<high)) break;
    // Measure the smaller requested portion directly. Subtracting almost
    // equal total/prefix lengths would lose precision near the endpoint.
    double ratio=fromEnd ? curveLength(discretization::splitCurve(segment,t).second)/length : curveLength(segment,t)/length;
    double roundoff=4*std::numeric_limits<double>::epsilon()*std::abs(t)*(curve_detail::speed(segment,t)/length);
    if(std::abs(ratio-target)<=1e-11*target+roundoff) return t;
    if(fromEnd ? ratio>target : ratio<target) low=t; else high=t;
  }
  throw std::runtime_error("Curve length inversion did not converge");
}
} // namespace discretization

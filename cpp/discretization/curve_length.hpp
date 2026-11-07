#pragma once

#include "curve_segment.hpp"
#include <limits>

namespace discretization {
namespace curve_detail {

inline void validateCurve(const CurveSegment &segment) {
  std::visit([](const auto &c) { c.validate(); }, segment);
}
inline double speed(const CurveSegment &segment, double t) {
  return std::visit(
      [t](const auto &c) { return curve_detail::norm(c.derivative(t)); },
      segment);
}

// Adaptive Simpson integration of speed. Tolerances scale with the curve,
// rather than with its position or an absolute world-unit floor.
inline double integrate(const CurveSegment &segment, double end) {
  auto f = [&](double t) {
    double s = speed(segment, t);
    if (!std::isfinite(s))
      throw std::invalid_argument(
          "Curve length exceeds supported numeric scale");
    return s;
  };
  auto simpson = [](double a, double b, double fa, double fm, double fb) {
    return (b - a) * (fa / 6 + 2 * fm / 3 + fb / 6);
  };
  // Initial panels avoid treating coincident samples on a periodic curve as
  // evidence of constant speed.
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
  if (!std::isfinite(result))
    throw std::invalid_argument("Curve length exceeds supported numeric scale");
  return result;
}
} // namespace curve_detail

inline double curveLength(const CurveSegment &segment, double end = 1) {
  if (!std::isfinite(end) || end < 0 || end > 1)
    throw std::invalid_argument("Length parameter must be in [0,1]");
  curve_detail::validateCurve(segment);
  if (end == 0)
    return 0;
  double result;
  auto exact =
      std::visit([end](const auto &c) { return c.exactLength(end); }, segment);
  if (exact)
    result = *exact;
  else
    result = curve_detail::integrate(segment, end);
  if (!std::isfinite(result))
    throw std::invalid_argument("Curve length exceeds supported numeric scale");
  return result;
}

inline double curveParameterAtLengthFraction(const CurveSegment &segment,
                                             double fraction) {
  if (!(fraction > 0 && fraction < 1) || !std::isfinite(fraction))
    throw std::invalid_argument(
        "Length fraction must be strictly between 0 and 1");
  double length = curveLength(segment);
  if (!(length > 0))
    throw std::invalid_argument("Cannot split a zero-length curve");
  auto exact = std::visit(
      [fraction](const auto &c) { return c.exactLengthParameter(fraction); },
      segment);
  if (exact)
    return *exact;
  double low = 0, high = 1;
  const bool fromEnd = fraction > .5;
  const double target = fromEnd ? 1 - fraction : fraction;
  double t = fraction;
  for (int i = 0; i < 144; ++i) {
    if (!(t > low && t < high))
      break;
    // Measure the smaller requested portion directly. Subtracting almost
    // equal total/prefix lengths would lose precision near the endpoint.
    double ratio =
        fromEnd ? curveLength(discretization::splitCurve(segment, t).second) /
                      length
                : curveLength(segment, t) / length;
    double roundoff = 4 * std::numeric_limits<double>::epsilon() * std::abs(t) *
                      (curve_detail::speed(segment, t) / length);
    if (std::abs(ratio - target) <= 1e-11 * target + roundoff)
      return t;
    if (fromEnd ? ratio > target : ratio < target)
      low = t;
    else
      high = t;
    // Newton accelerates regular portions; keep the bracket and periodically
    // bisect even when a nearly stationary derivative gives a poor proposal.
    // After 16 attempts retain the original full 128-step bisection budget.
    double derivative = curve_detail::speed(segment, t) / length;
    double candidate =
        t + (fromEnd ? ratio - target : target - ratio) / derivative;
    t = (i < 16 && i % 4 != 3 && std::isfinite(candidate) && candidate > low &&
         candidate < high)
            ? candidate
            : (low + high) / 2;
  }
  throw std::runtime_error("Curve length inversion did not converge");
}
} // namespace discretization

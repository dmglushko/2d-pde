#include "pde2d/geometry/predicates.hpp"

#include <boost/multiprecision/cpp_int.hpp>

#include <algorithm>
#include <bit>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <stdexcept>

// Ported from the archive (cpp/discretization/predicates.hpp). Every finite
// double is a dyadic rational, so scaling all inputs by one common power of
// two turns them into integers, and the determinants can then be evaluated
// exactly with arbitrary-precision integers.
namespace pde2d::geometry {
namespace {

using Integer = boost::multiprecision::cpp_int;

int exponent(double x) {
  auto bits = std::bit_cast<std::uint64_t>(x);
  int e = static_cast<int>((bits >> 52) & 0x7ff);
  return e ? e - 1075 : -1074;
}

int common_exponent(std::initializer_list<double> values) {
  int e = 1024;
  for (double x : values) {
    if (!std::isfinite(x))
      throw std::invalid_argument("Predicate requires finite coordinates");
    if (x != 0)
      e = std::min(e, exponent(x));
  }
  return e == 1024 ? 0 : e;
}

Integer integer(double x, int scale) {
  if (x == 0)
    return 0;
  auto bits = std::bit_cast<std::uint64_t>(x);
  auto mantissa = bits & ((std::uint64_t{1} << 52) - 1);
  if ((bits >> 52) & 0x7ff)
    mantissa |= std::uint64_t{1} << 52;
  Integer n = mantissa;
  n <<= exponent(x) - scale;
  return bits >> 63 ? -n : n;
}

} // namespace

int orient(Point a, Point b, Point c) {
  double ax = a.x - c.x, ay = a.y - c.y, bx = b.x - c.x, by = b.y - c.y;
  double p = ax * by, q = ay * bx, d = p - q;
  double bound =
      16 * std::numeric_limits<double>::epsilon() * (std::abs(p) + std::abs(q));
  if (std::isfinite(d) && std::isfinite(bound) &&
      bound >= std::numeric_limits<double>::min() && std::abs(d) > bound)
    return d > 0 ? 1 : -1;
  int e = common_exponent({a.x, a.y, b.x, b.y, c.x, c.y});
  Integer x = integer(a.x, e) - integer(c.x, e),
          y = integer(a.y, e) - integer(c.y, e);
  Integer u = integer(b.x, e) - integer(c.x, e),
          v = integer(b.y, e) - integer(c.y, e);
  return Integer(x * v - y * u).sign();
}

int incircle(Point a, Point b, Point c, Point d) {
  double ax = a.x - d.x, ay = a.y - d.y, bx = b.x - d.x, by = b.y - d.y,
         cx = c.x - d.x, cy = c.y - d.y;
  double al = ax * ax + ay * ay, bl = bx * bx + by * by, cl = cx * cx + cy * cy;
  double det = al * (bx * cy - by * cx) - bl * (ax * cy - ay * cx) +
               cl * (ax * by - ay * bx);
  double permanent = al * (std::abs(bx * cy) + std::abs(by * cx)) +
                     bl * (std::abs(ax * cy) + std::abs(ay * cx)) +
                     cl * (std::abs(ax * by) + std::abs(ay * bx));
  double bound = 128 * std::numeric_limits<double>::epsilon() * permanent;
  if (std::isfinite(det) && std::isfinite(bound) &&
      bound >= std::numeric_limits<double>::min() && std::abs(det) > bound)
    return det > 0 ? 1 : -1;
  int e = common_exponent({a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y});
  Integer x = integer(a.x, e) - integer(d.x, e),
          y = integer(a.y, e) - integer(d.y, e);
  Integer u = integer(b.x, e) - integer(d.x, e),
          v = integer(b.y, e) - integer(d.y, e);
  Integer s = integer(c.x, e) - integer(d.x, e),
          t = integer(c.y, e) - integer(d.y, e);
  Integer det_exact = (x * x + y * y) * (u * t - v * s) -
                      (u * u + v * v) * (x * t - y * s) +
                      (s * s + t * t) * (x * v - y * u);
  return det_exact.sign();
}

bool on_segment(Point a, Point b, Point p) {
  return orient(a, b, p) == 0 && p.x >= std::min(a.x, b.x) &&
         p.x <= std::max(a.x, b.x) && p.y >= std::min(a.y, b.y) &&
         p.y <= std::max(a.y, b.y);
}

bool segments_intersect(Point a, Point b, Point c, Point d) {
  int p = orient(a, b, c), q = orient(a, b, d), r = orient(c, d, a),
      s = orient(c, d, b);
  return (p * q < 0 && r * s < 0) || (!p && on_segment(a, b, c)) ||
         (!q && on_segment(a, b, d)) || (!r && on_segment(c, d, a)) ||
         (!s && on_segment(c, d, b));
}

} // namespace pde2d::geometry

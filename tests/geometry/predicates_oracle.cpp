// Compares the adaptive (Shewchuk) predicates with an independent exact
// evaluation on many nearly degenerate inputs. The oracle is the previous
// implementation's exact path: every finite double is a dyadic rational,
// so scaling all inputs by one common power of two turns them into
// integers, and the determinants are evaluated with arbitrary-precision
// integers (Boost.Multiprecision). It has no floating-point filter, so
// every case goes through exact arithmetic.

#include "support/check.hpp"

#include "pde2d/geometry/predicates.hpp"

#include <boost/multiprecision/cpp_int.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <random>
#include <string>

using namespace pde2d::geometry;
using namespace pde2d::test;

namespace {

using Integer = boost::multiprecision::cpp_int;

int exponent(double x) {
  auto bits = std::bit_cast<std::uint64_t>(x);
  int e = static_cast<int>((bits >> 52) & 0x7ff);
  return e ? e - 1075 : -1074;
}

int common_exponent(std::initializer_list<double> values) {
  int e = 1024;
  for (double x : values)
    if (x != 0)
      e = std::min(e, exponent(x));
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

int exact_orient(Point a, Point b, Point c) {
  int e = common_exponent({a.x, a.y, b.x, b.y, c.x, c.y});
  Integer x = integer(a.x, e) - integer(c.x, e),
          y = integer(a.y, e) - integer(c.y, e);
  Integer u = integer(b.x, e) - integer(c.x, e),
          v = integer(b.y, e) - integer(c.y, e);
  return Integer(x * v - y * u).sign();
}

int exact_incircle(Point a, Point b, Point c, Point d) {
  int e = common_exponent({a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y});
  Integer x = integer(a.x, e) - integer(d.x, e),
          y = integer(a.y, e) - integer(d.y, e);
  Integer u = integer(b.x, e) - integer(d.x, e),
          v = integer(b.y, e) - integer(d.y, e);
  Integer s = integer(c.x, e) - integer(d.x, e),
          t = integer(c.y, e) - integer(d.y, e);
  Integer det = (x * x + y * y) * (u * t - v * s) -
                (u * u + v * v) * (x * t - y * s) +
                (s * s + t * t) * (x * v - y * u);
  return det.sign();
}

// A deterministic source of test data. Only integer draws are used (their
// results are fixed by the standard), so the cases are the same with every
// standard library.
struct Random {
  std::mt19937_64 engine{20261112};
  // Uniform in [lo, hi].
  double uniform(double lo, double hi) {
    double unit = static_cast<double>(engine() >> 11) * 0x1p-53;
    return lo + (hi - lo) * unit;
  }
  int integer(int lo, int hi) {
    return lo + static_cast<int>(engine() % static_cast<std::uint64_t>(
                                                 hi - lo + 1));
  }
  // x moved by up to `ulps` units in the last place, either way.
  double nudge(double x, int ulps) {
    int k = integer(-ulps, ulps);
    for (; k > 0; --k)
      x = std::nextafter(x, INFINITY);
    for (; k < 0; ++k)
      x = std::nextafter(x, -INFINITY);
    return x;
  }
  Point nudge(Point p, int ulps) { return {nudge(p.x, ulps), nudge(p.y, ulps)}; }
};

struct Tally {
  std::size_t cases = 0, zeros = 0;
};

void compare_orient(Point a, Point b, Point c, Tally &tally) {
  int expected = exact_orient(a, b, c);
  ++tally.cases;
  tally.zeros += expected == 0;
  // All six orders: the sign flips with odd permutations.
  int got[6] = {orient(a, b, c), orient(b, c, a), orient(c, a, b),
                -orient(b, a, c), -orient(a, c, b), -orient(c, b, a)};
  for (int g : got)
    if (g != expected)
      throw Failure("orient disagrees with the exact oracle at (" +
                    std::to_string(a.x) + ", " + std::to_string(a.y) +
                    "), (" + std::to_string(b.x) + ", " +
                    std::to_string(b.y) + "), (" + std::to_string(c.x) +
                    ", " + std::to_string(c.y) + ")");
}

void compare_incircle(Point a, Point b, Point c, Point d, Tally &tally) {
  // incircle's sign convention needs a counterclockwise a, b, c.
  int turn = exact_orient(a, b, c);
  if (turn == 0)
    return;
  if (turn < 0)
    std::swap(a, b);
  int expected = exact_incircle(a, b, c, d);
  ++tally.cases;
  tally.zeros += expected == 0;
  int got[3] = {incircle(a, b, c, d), incircle(b, c, a, d),
                incircle(c, a, b, d)};
  for (int g : got)
    if (g != expected)
      throw Failure("incircle disagrees with the exact oracle at d = (" +
                    std::to_string(d.x) + ", " + std::to_string(d.y) + ")");
}

// The point at parameter t on the line through a and b, rounded.
Point on_line(Point a, Point b, double t) {
  return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
}

constexpr double pi = 3.14159265358979323846;

} // namespace

int main() {
  return run({
      {"orient_near_lines",
       [] {
         Random r;
         Tally tally;
         for (double offset : {0.0, 1.0, 1e3, 1e8, 1e15, 1e30, -1e40})
           for (int k = 0; k < 4000; ++k) {
             double scale = std::ldexp(1.0, r.integer(-40, 40));
             Point a{offset + scale * r.uniform(-1, 1),
                     offset + scale * r.uniform(-1, 1)};
             Point b{offset + scale * r.uniform(-1, 1),
                     offset + scale * r.uniform(-1, 1)};
             Point c = r.nudge(on_line(a, b, r.uniform(-2, 3)), 3);
             compare_orient(a, b, c, tally);
           }
         check(tally.cases == 28000, "all cases ran");
       }},
      {"orient_exactly_collinear",
       [] {
         // Small integers times a power of two, plus an exact offset: the
         // points are exactly collinear, and a nudge of one ulp is not.
         Random r;
         Tally tally;
         for (int k = 0; k < 20000; ++k) {
           double unit = std::ldexp(1.0, r.integer(-60, 60));
           double offset = std::ldexp(r.integer(-1000, 1000), r.integer(0, 40));
           Point a{offset + unit * r.integer(-50, 50),
                   offset + unit * r.integer(-50, 50)};
           Point d{unit * r.integer(-9, 9), unit * r.integer(-9, 9)};
           int m = r.integer(-5, 5), n = r.integer(-5, 5);
           Point b{a.x + m * d.x, a.y + m * d.y};
           Point c{a.x + n * d.x, a.y + n * d.y};
           compare_orient(a, b, c, tally);
           compare_orient(a, b, r.nudge(c, 1), tally);
         }
         check(tally.zeros > 10000, "many exactly collinear cases");
       }},
      {"incircle_near_circles",
       [] {
         Random r;
         Tally tally;
         for (double offset : {0.0, 1.0, 1e4, 1e9, 1e15, 1e25})
           for (int k = 0; k < 3000; ++k) {
             // Not so small that the offset rounds the points together.
             double radius =
                 std::max(std::ldexp(1.0, r.integer(-30, 30)), offset * 0x1p-40);
             Point center{offset + radius * r.uniform(-3, 3),
                          offset + radius * r.uniform(-3, 3)};
             auto on_circle = [&](double angle) {
               return Point{center.x + radius * std::cos(angle),
                            center.y + radius * std::sin(angle)};
             };
             Point a = on_circle(r.uniform(0, 2 * pi)),
                   b = on_circle(r.uniform(0, 2 * pi)),
                   c = on_circle(r.uniform(0, 2 * pi));
             Point d = r.nudge(on_circle(r.uniform(0, 2 * pi)), 4);
             compare_incircle(a, b, c, d, tally);
           }
         check(tally.cases > 17000, "almost all triangles nondegenerate");
       }},
      {"incircle_exactly_cocircular",
       [] {
         // Integer points on x^2 + y^2 = 25 and 65^2, scaled by a power of
         // two and moved by an exact offset: exactly cocircular.
         const Point five[] = {{5, 0},  {4, 3},   {3, 4},   {0, 5},
                               {-3, 4}, {-4, 3},  {-5, 0},  {-4, -3},
                               {-3, -4}, {0, -5}, {3, -4},  {4, -3}};
         const Point sixty_five[] = {{65, 0},   {63, 16},  {60, 25},
                                     {56, 33},  {52, 39},  {39, 52},
                                     {33, -56}, {-25, 60}, {-16, -63}};
         Random r;
         Tally tally;
         for (int k = 0; k < 20000; ++k) {
           double unit = std::ldexp(1.0, r.integer(-50, 50));
           Point offset{std::ldexp(r.integer(-999, 999), r.integer(0, 30)),
                        std::ldexp(r.integer(-999, 999), r.integer(0, 30))};
           auto pick = [&] {
             Point p = k % 2 ? five[r.integer(0, 11)]
                             : sixty_five[r.integer(0, 8)];
             if (r.integer(0, 1))
               p.x = -p.x;
             return Point{offset.x + unit * p.x, offset.y + unit * p.y};
           };
           Point a = pick(), b = pick(), c = pick(), d = pick();
           compare_incircle(a, b, c, d, tally);
           compare_incircle(a, b, c, r.nudge(d, 1), tally);
         }
         check(tally.zeros > 5000, "many exactly cocircular cases");
       }},
      {"incircle_tiny_perturbations",
       [] {
         // Four corners of a square (cocircular), each moved by a few ulps,
         // at many scales and offsets: the tails of the differences are
         // nonzero, so the deepest adaptive stage runs.
         Random r;
         Tally tally;
         for (int k = 0; k < 20000; ++k) {
           double s = std::ldexp(1.0, r.integer(-20, 20));
           double offset = r.uniform(-1, 1) * std::ldexp(1.0, r.integer(0, 50));
           Point a = r.nudge(Point{offset, offset}, 2),
                 b = r.nudge(Point{offset + s, offset}, 2),
                 c = r.nudge(Point{offset + s, offset + s}, 2),
                 d = r.nudge(Point{offset, offset + s}, 2);
           compare_incircle(a, b, c, d, tally);
         }
         check(tally.cases > 19000, "cases ran");
       }},
      {"extreme_exponents",
       [] {
         // Coordinates outside the adaptive code's range: tiny, subnormal
         // and huge values, nearly collinear and nearly cocircular.
         Random r;
         Tally tally;
         for (int k = 0; k < 3000; ++k) {
           double s = std::ldexp(1.0, r.integer(-1070, 1000));
           Point a{s * r.integer(-9, 9), s * r.integer(-9, 9)};
           Point d{s * r.integer(-3, 3), s * r.integer(-3, 3)};
           Point b{a.x + 2 * d.x, a.y + 2 * d.y};
           compare_orient(a, b, r.nudge(Point{a.x + d.x, a.y + d.y}, 1), tally);
           Point q[4] = {{5 * s, 0}, {0, 5 * s}, {-3 * s, 4 * s},
                         {4 * s, -3 * s}};
           compare_incircle(q[0], q[1], q[2], r.nudge(q[3], 1), tally);
         }
         check(tally.cases > 4000, "cases ran");
       }},
      {"general_position",
       [] {
         Random r;
         Tally tally;
         for (int k = 0; k < 20000; ++k) {
           auto p = [&] { return Point{r.uniform(-1, 1), r.uniform(-1, 1)}; };
           Point a = p(), b = p(), c = p(), d = p();
           compare_orient(a, b, c, tally);
           compare_incircle(a, b, c, d, tally);
         }
       }},
  });
}

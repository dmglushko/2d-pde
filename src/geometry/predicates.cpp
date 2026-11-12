#include "pde2d/geometry/predicates.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <vector>

// Jonathan Richard Shewchuk's adaptive-precision orient2d and incircle
// ("Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric
// Predicates", 1997; predicates.c, public domain), ported to C++. The
// macros of the original are small inline functions here; the stages and
// error bounds are the original ones. For coordinates far outside the
// usual range (see in_exact_range) a slow integer evaluation takes over,
// so the answer is exact for every finite input.
//
// The code needs IEEE double arithmetic with every operation rounded on
// its own:
// - no -ffast-math (it would reorder and simplify the error-free
//   transformations away; the build compiles this file with
//   -fno-fast-math, and the check below stops a build that defines
//   __FAST_MATH__ anyway);
// - no contraction of a * b + c into a fused multiply-add (the build
//   compiles this file with -ffp-contract=off);
// - no extended intermediate precision. x87 arithmetic (80-bit registers)
//   would break the error-free transformations; on x86-64 the compiler
//   uses SSE2, where FLT_EVAL_METHOD is 0, which the check below demands.
#if defined(__FAST_MATH__)
#error "predicates.cpp must not be compiled with -ffast-math"
#endif
#if !defined(FLT_EVAL_METHOD) || FLT_EVAL_METHOD != 0
#error "predicates.cpp needs double arithmetic without extended precision"
#endif

namespace pde2d::geometry {
namespace {

static_assert(std::numeric_limits<double>::is_iec559 &&
                  std::numeric_limits<double>::digits == 53 &&
                  std::numeric_limits<double>::round_style ==
                      std::round_to_nearest,
              "Shewchuk's predicates need IEEE double with round to nearest");

// exactinit() of the original, evaluated at compile time. epsilon is half
// an ulp of 1, splitter is 2^ceil(53 / 2) + 1.
constexpr double epsilon = 0x1p-53;
constexpr double splitter = 0x1p27 + 1.0;
constexpr double resulterrbound = (3.0 + 8.0 * epsilon) * epsilon;
constexpr double ccwerrboundA = (3.0 + 16.0 * epsilon) * epsilon;
constexpr double ccwerrboundB = (2.0 + 12.0 * epsilon) * epsilon;
constexpr double ccwerrboundC = (9.0 + 64.0 * epsilon) * epsilon * epsilon;
constexpr double iccerrboundA = (10.0 + 96.0 * epsilon) * epsilon;
constexpr double iccerrboundB = (4.0 + 48.0 * epsilon) * epsilon;
constexpr double iccerrboundC = (44.0 + 576.0 * epsilon) * epsilon * epsilon;

// The expansion arithmetic is exact as long as no intermediate value
// overflows and no product underflows. Every input coordinate is an
// integer multiple of g = 2^(e - 52), where 2^e <= the smallest nonzero
// |coordinate|; incircle multiplies up to four such quantities, so its
// rounding errors are multiples of g^4, which stays a normal double while
// e >= -200. Products of up to four differences stay far below the
// overflow threshold while every |coordinate| <= 2^200. orient (degree 2)
// would allow a wider range; one range keeps the rule simple. Outside it,
// an undecided case goes to the integer fallback below.
constexpr double smallest_exact = 0x1p-200;
constexpr double largest_exact = 0x1p200;

void check_finite(std::initializer_list<double> values) {
  for (double x : values)
    if (!std::isfinite(x))
      throw std::invalid_argument("Predicate requires finite coordinates");
}

bool in_exact_range(std::initializer_list<double> values) {
  for (double x : values) {
    double m = std::abs(x);
    if (m != 0 && (m < smallest_exact || m > largest_exact))
      return false;
  }
  return true;
}

// --- integer fallback -------------------------------------------------------

// Every finite double is an integer times a power of two, so after scaling
// all inputs by the smallest power of two among them they are integers,
// and the determinant can be computed exactly with integers of a few
// thousand bits. This is slow, and runs only for coordinates outside the
// range above when the floating-point filter cannot decide (for example
// points converging on a sharp corner at the origin).
class BigInt {
public:
  BigInt() = default;

  // x / 2^scale, which must be an integer.
  static BigInt from_double(double x, int scale) {
    BigInt r;
    if (x == 0)
      return r;
    auto bits = std::bit_cast<std::uint64_t>(x);
    std::uint64_t mantissa = bits & ((std::uint64_t{1} << 52) - 1);
    if ((bits >> 52) & 0x7ff)
      mantissa |= std::uint64_t{1} << 52;
    int shift = exponent(x) - scale;
    std::size_t words = static_cast<std::size_t>(shift / 32),
                rest = static_cast<std::size_t>(shift % 32);
    r.mag_.assign(words, 0);
    // The mantissa shifted by `rest` bits fits in three 32-bit words.
    std::uint64_t low = mantissa << rest;
    std::uint64_t high = rest ? mantissa >> (64 - rest) : 0;
    r.mag_.push_back(static_cast<std::uint32_t>(low));
    r.mag_.push_back(static_cast<std::uint32_t>(low >> 32));
    r.mag_.push_back(static_cast<std::uint32_t>(high));
    r.trim();
    r.negative_ = bits >> 63;
    return r;
  }

  // The binary exponent of the lowest bit of a finite nonzero double.
  static int exponent(double x) {
    auto bits = std::bit_cast<std::uint64_t>(x);
    int e = static_cast<int>((bits >> 52) & 0x7ff);
    return e ? e - 1075 : -1074;
  }

  int sign() const { return mag_.empty() ? 0 : (negative_ ? -1 : 1); }

  friend BigInt operator-(BigInt a) {
    if (!a.mag_.empty())
      a.negative_ = !a.negative_;
    return a;
  }
  friend BigInt operator+(const BigInt &a, const BigInt &b) {
    BigInt r;
    if (a.negative_ == b.negative_) {
      r.mag_ = add(a.mag_, b.mag_);
      r.negative_ = a.negative_;
    } else if (compare(a.mag_, b.mag_) >= 0) {
      r.mag_ = subtract(a.mag_, b.mag_);
      r.negative_ = a.negative_;
    } else {
      r.mag_ = subtract(b.mag_, a.mag_);
      r.negative_ = b.negative_;
    }
    r.trim();
    return r;
  }
  friend BigInt operator-(const BigInt &a, const BigInt &b) { return a + -b; }
  friend BigInt operator*(const BigInt &a, const BigInt &b) {
    BigInt r;
    if (a.mag_.empty() || b.mag_.empty())
      return r;
    r.mag_.assign(a.mag_.size() + b.mag_.size(), 0);
    for (std::size_t i = 0; i < a.mag_.size(); ++i) {
      std::uint64_t carry = 0;
      for (std::size_t j = 0; j < b.mag_.size(); ++j) {
        std::uint64_t t =
            std::uint64_t{a.mag_[i]} * b.mag_[j] + r.mag_[i + j] + carry;
        r.mag_[i + j] = static_cast<std::uint32_t>(t);
        carry = t >> 32;
      }
      r.mag_[i + b.mag_.size()] = static_cast<std::uint32_t>(carry);
    }
    r.negative_ = a.negative_ != b.negative_;
    r.trim();
    return r;
  }

private:
  using Words = std::vector<std::uint32_t>; // least significant first

  void trim() {
    while (!mag_.empty() && mag_.back() == 0)
      mag_.pop_back();
    if (mag_.empty())
      negative_ = false;
  }
  static int compare(const Words &a, const Words &b) {
    if (a.size() != b.size())
      return a.size() < b.size() ? -1 : 1;
    for (std::size_t i = a.size(); i-- > 0;)
      if (a[i] != b[i])
        return a[i] < b[i] ? -1 : 1;
    return 0;
  }
  static Words add(const Words &a, const Words &b) {
    Words r(std::max(a.size(), b.size()) + 1, 0);
    std::uint64_t carry = 0;
    for (std::size_t i = 0; i + 1 < r.size(); ++i) {
      std::uint64_t t = carry;
      if (i < a.size())
        t += a[i];
      if (i < b.size())
        t += b[i];
      r[i] = static_cast<std::uint32_t>(t);
      carry = t >> 32;
    }
    r.back() = static_cast<std::uint32_t>(carry);
    return r;
  }
  // |a| >= |b| required.
  static Words subtract(const Words &a, const Words &b) {
    Words r(a.size(), 0);
    std::int64_t borrow = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
      std::int64_t t =
          std::int64_t{a[i]} - borrow - (i < b.size() ? std::int64_t{b[i]} : 0);
      borrow = t < 0;
      r[i] = static_cast<std::uint32_t>(t + (borrow << 32));
    }
    return r;
  }

  Words mag_;
  bool negative_ = false;
};

int common_exponent(std::initializer_list<double> values) {
  int e = 1024;
  for (double x : values)
    if (x != 0)
      e = std::min(e, BigInt::exponent(x));
  return e == 1024 ? 0 : e;
}

int integer_orient(Point a, Point b, Point c) {
  int e = common_exponent({a.x, a.y, b.x, b.y, c.x, c.y});
  auto I = [e](double x) { return BigInt::from_double(x, e); };
  BigInt acx = I(a.x) - I(c.x), acy = I(a.y) - I(c.y);
  BigInt bcx = I(b.x) - I(c.x), bcy = I(b.y) - I(c.y);
  return (acx * bcy - acy * bcx).sign();
}

int integer_incircle(Point a, Point b, Point c, Point d) {
  int e = common_exponent({a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y});
  auto I = [e](double x) { return BigInt::from_double(x, e); };
  BigInt adx = I(a.x) - I(d.x), ady = I(a.y) - I(d.y);
  BigInt bdx = I(b.x) - I(d.x), bdy = I(b.y) - I(d.y);
  BigInt cdx = I(c.x) - I(d.x), cdy = I(c.y) - I(d.y);
  BigInt det = (adx * adx + ady * ady) * (bdx * cdy - bdy * cdx) -
               (bdx * bdx + bdy * bdy) * (adx * cdy - ady * cdx) +
               (cdx * cdx + cdy * cdy) * (adx * bdy - ady * bdx);
  return det.sign();
}

// --- error-free transformations (the macros of predicates.c) ---------------

struct Pair {
  double hi, lo; // hi + lo exactly, |lo| <= ulp(hi) / 2
};

// |a| >= |b| required.
inline Pair fast_two_sum(double a, double b) {
  double x = a + b;
  double bvirt = x - a;
  return {x, b - bvirt};
}

inline Pair two_sum(double a, double b) {
  double x = a + b;
  double bvirt = x - a;
  double avirt = x - bvirt;
  double bround = b - bvirt;
  double around = a - avirt;
  return {x, around + bround};
}

// The roundoff of x = fl(a - b).
inline double two_diff_tail(double a, double b, double x) {
  double bvirt = a - x;
  double avirt = x + bvirt;
  double bround = bvirt - b;
  double around = a - avirt;
  return around + bround;
}

inline Pair two_diff(double a, double b) {
  double x = a - b;
  return {x, two_diff_tail(a, b, x)};
}

// a = hi + lo with both halves of at most 26 significant bits.
inline Pair split(double a) {
  double c = splitter * a;
  double abig = c - a;
  double hi = c - abig;
  return {hi, a - hi};
}

inline Pair two_product_presplit(double a, double b, Pair bs) {
  double x = a * b;
  Pair as = split(a);
  double err1 = x - as.hi * bs.hi;
  double err2 = err1 - as.lo * bs.hi;
  double err3 = err2 - as.hi * bs.lo;
  return {x, as.lo * bs.lo - err3};
}

inline Pair two_product(double a, double b) {
  return two_product_presplit(a, b, split(b));
}

inline Pair square(double a) {
  double x = a * a;
  Pair as = split(a);
  double err1 = x - as.hi * as.hi;
  double err3 = err1 - (as.hi + as.hi) * as.lo;
  return {x, as.lo * as.lo - err3};
}

// (a1 + a0) + (b1 + b0) and (a1 + a0) - (b1 + b0) as four-component
// expansions, smallest component first.
using Four = std::array<double, 4>;

inline Four two_two_sum(Pair a, Pair b) {
  Pair s = two_sum(a.lo, b.lo); // Two_One_Sum(a1, a0, b0)
  Pair t = two_sum(a.hi, s.hi); //   -> (_j, _0, x0)
  Pair u = two_sum(t.lo, b.hi); // Two_One_Sum(_j, _0, b1)
  Pair v = two_sum(t.hi, u.hi); //   -> (x3, x2, x1)
  return {s.lo, u.lo, v.lo, v.hi};
}

inline Four two_two_diff(Pair a, Pair b) {
  Pair s = two_diff(a.lo, b.lo); // Two_One_Diff(a1, a0, b0)
  Pair t = two_sum(a.hi, s.hi);
  Pair u = two_diff(t.lo, b.hi); // Two_One_Diff(_j, _0, b1)
  Pair v = two_sum(t.hi, u.hi);
  return {s.lo, u.lo, v.lo, v.hi};
}

// --- expansions -------------------------------------------------------------

// fast_expansion_sum_zeroelim: h = e + f, with zero components removed.
// e and f are nonoverlapping expansions, smallest component first; h needs
// room for elen + flen components. Unlike the original, it never reads
// past the end of e or f.
std::size_t sum(const double *e, std::size_t elen, const double *f,
                std::size_t flen, double *h) {
  // The next component to merge is the one of smaller magnitude.
  auto take_e = [](double enow, double fnow) {
    return (fnow > enow) == (fnow > -enow);
  };
  std::size_t ei = 0, fi = 0, hi = 0;
  double Q;
  if (take_e(e[0], f[0]))
    Q = e[ei++];
  else
    Q = f[fi++];
  if (ei < elen && fi < flen) {
    Pair s = take_e(e[ei], f[fi]) ? fast_two_sum(e[ei++], Q)
                                  : fast_two_sum(f[fi++], Q);
    Q = s.hi;
    if (s.lo != 0.0)
      h[hi++] = s.lo;
    while (ei < elen && fi < flen) {
      s = take_e(e[ei], f[fi]) ? two_sum(Q, e[ei++]) : two_sum(Q, f[fi++]);
      Q = s.hi;
      if (s.lo != 0.0)
        h[hi++] = s.lo;
    }
  }
  while (ei < elen) {
    Pair s = two_sum(Q, e[ei++]);
    Q = s.hi;
    if (s.lo != 0.0)
      h[hi++] = s.lo;
  }
  while (fi < flen) {
    Pair s = two_sum(Q, f[fi++]);
    Q = s.hi;
    if (s.lo != 0.0)
      h[hi++] = s.lo;
  }
  if (Q != 0.0 || hi == 0)
    h[hi++] = Q;
  return hi;
}

// scale_expansion_zeroelim: h = b * e, with zero components removed; h
// needs room for 2 * elen components.
std::size_t scale(const double *e, std::size_t elen, double b, double *h) {
  Pair bs = split(b);
  Pair p = two_product_presplit(e[0], b, bs);
  double Q = p.hi;
  std::size_t hi = 0;
  if (p.lo != 0.0)
    h[hi++] = p.lo;
  for (std::size_t i = 1; i < elen; ++i) {
    Pair product = two_product_presplit(e[i], b, bs);
    Pair s = two_sum(Q, product.lo);
    if (s.lo != 0.0)
      h[hi++] = s.lo;
    Pair t = fast_two_sum(product.hi, s.hi);
    Q = t.hi;
    if (t.lo != 0.0)
      h[hi++] = t.lo;
  }
  if (Q != 0.0 || hi == 0)
    h[hi++] = Q;
  return hi;
}

// A buffer with its used length, so the stages read like the original.
template <std::size_t N> struct Expansion {
  std::array<double, N> v{};
  std::size_t n = 0;
};

template <std::size_t N, std::size_t A>
void scale_into(Expansion<N> &out, const Expansion<A> &e, double b) {
  static_assert(N >= 2 * A, "scale result buffer too small");
  out.n = scale(e.v.data(), e.n, b, out.v.data());
}

template <std::size_t N, std::size_t A, std::size_t B>
void sum_into(Expansion<N> &out, const Expansion<A> &e, const Expansion<B> &f) {
  static_assert(N >= A + B, "sum result buffer too small");
  out.n = sum(e.v.data(), e.n, f.v.data(), f.n, out.v.data());
}

Expansion<4> four(const Four &x) { return {{x[0], x[1], x[2], x[3]}, 4}; }

template <std::size_t N> double estimate(const Expansion<N> &e) {
  double q = e.v[0];
  for (std::size_t i = 1; i < e.n; ++i)
    q += e.v[i];
  return q;
}

int sign(double x) { return (x > 0) - (x < 0); }

// --- orient2d ---------------------------------------------------------------

double orient2d_adapt(Point pa, Point pb, Point pc, double detsum) {
  double acx = pa.x - pc.x, bcx = pb.x - pc.x;
  double acy = pa.y - pc.y, bcy = pb.y - pc.y;

  Expansion<4> B =
      four(two_two_diff(two_product(acx, bcy), two_product(acy, bcx)));
  double det = estimate(B);
  double errbound = ccwerrboundB * detsum;
  if (det >= errbound || -det >= errbound)
    return det;

  double acxtail = two_diff_tail(pa.x, pc.x, acx);
  double bcxtail = two_diff_tail(pb.x, pc.x, bcx);
  double acytail = two_diff_tail(pa.y, pc.y, acy);
  double bcytail = two_diff_tail(pb.y, pc.y, bcy);
  if (acxtail == 0.0 && acytail == 0.0 && bcxtail == 0.0 && bcytail == 0.0)
    return det;

  errbound = ccwerrboundC * detsum + resulterrbound * std::abs(det);
  det += (acx * bcytail + bcy * acxtail) - (acy * bcxtail + bcx * acytail);
  if (det >= errbound || -det >= errbound)
    return det;

  Expansion<8> C1;
  sum_into(
      C1, B,
      four(two_two_diff(two_product(acxtail, bcy), two_product(acytail, bcx))));
  Expansion<12> C2;
  sum_into(
      C2, C1,
      four(two_two_diff(two_product(acx, bcytail), two_product(acy, bcxtail))));
  Expansion<16> D;
  sum_into(D, C2,
           four(two_two_diff(two_product(acxtail, bcytail),
                             two_product(acytail, bcxtail))));
  return D.v[D.n - 1];
}

// --- incircle ---------------------------------------------------------------

// The three rows (a, b, c) of the incircle determinant, each relative to
// d. The determinant is invariant under the cyclic rotation
// a -> b -> c -> a, and the original repeats every block three times with
// the roles rotated; here the blocks loop over i with j = i + 1 and
// k = i + 2 (mod 3), in the original order (a, then b, then c).
double incircle_adapt(Point pa, Point pb, Point pc, Point pd,
                      double permanent) {
  const Point p[3] = {pa, pb, pc};
  double dx[3], dy[3];
  for (int i = 0; i < 3; ++i) {
    dx[i] = p[i].x - pd.x;
    dy[i] = p[i].y - pd.y;
  }
  auto nx = [](int i) { return (i + 1) % 3; };
  auto pv = [](int i) { return (i + 2) % 3; };

  // cr[i] = dx[j] dy[k] - dx[k] dy[j]: bc, ca, ab of the original.
  Expansion<4> cr[3];
  for (int i = 0; i < 3; ++i)
    cr[i] = four(two_two_diff(two_product(dx[nx(i)], dy[pv(i)]),
                              two_product(dx[pv(i)], dy[nx(i)])));

  // adet, bdet, cdet: (dx^2 + dy^2) * cr.
  Expansion<32> rowdet[3];
  for (int i = 0; i < 3; ++i) {
    Expansion<8> xc, yc;
    Expansion<16> xxc, yyc;
    scale_into(xc, cr[i], dx[i]);
    scale_into(xxc, xc, dx[i]);
    scale_into(yc, cr[i], dy[i]);
    scale_into(yyc, yc, dy[i]);
    sum_into(rowdet[i], xxc, yyc);
  }
  Expansion<64> abdet;
  sum_into(abdet, rowdet[0], rowdet[1]);
  Expansion<1152> fin[2];
  int now = 0;
  sum_into(fin[now], abdet, rowdet[2]);

  double det = estimate(fin[now]);
  double errbound = iccerrboundB * permanent;
  if (det >= errbound || -det >= errbound)
    return det;

  double dxt[3], dyt[3];
  for (int i = 0; i < 3; ++i) {
    dxt[i] = two_diff_tail(p[i].x, pd.x, dx[i]);
    dyt[i] = two_diff_tail(p[i].y, pd.y, dy[i]);
  }
  if (dxt[0] == 0.0 && dxt[1] == 0.0 && dxt[2] == 0.0 && dyt[0] == 0.0 &&
      dyt[1] == 0.0 && dyt[2] == 0.0)
    return det;

  errbound = iccerrboundC * permanent + resulterrbound * std::abs(det);
  // The bound holds for the original order of evaluation: the three row
  // terms are summed first, (a + b) + c, and added to det once.
  double row[3];
  for (int i = 0; i < 3; ++i) {
    int j = nx(i), k = pv(i);
    row[i] =
        (dx[i] * dx[i] + dy[i] * dy[i]) * ((dx[j] * dyt[k] + dy[k] * dxt[j]) -
                                           (dy[j] * dxt[k] + dx[k] * dyt[j])) +
        2.0 * (dx[i] * dxt[i] + dy[i] * dyt[i]) *
            (dx[j] * dy[k] - dy[j] * dx[k]);
  }
  det += (row[0] + row[1]) + row[2];
  if (det >= errbound || -det >= errbound)
    return det;

  // Adds an expansion to the running total. Shewchuk's analysis bounds
  // the total at 1152 components; the check guards against a porting
  // mistake.
  auto add = [&](const auto &e) {
    if (fin[now].n + e.n > fin[now].v.size())
      throw std::logic_error("incircle: expansion buffer too small");
    fin[1 - now].n = sum(fin[now].v.data(), fin[now].n, e.v.data(), e.n,
                         fin[1 - now].v.data());
    now = 1 - now;
  };
  auto has_tail = [&](int i) { return dxt[i] != 0.0 || dyt[i] != 0.0; };

  // aa, bb, cc of the original: dx^2 + dy^2 exactly. The original
  // computes each only when a later block needs it.
  Expansion<4> sq[3];
  for (int i = 0; i < 3; ++i)
    sq[i] = four(two_two_sum(square(dx[i]), square(dy[i])));

  // First-order tail terms. axtbc etc. are kept for the second stage.
  Expansion<8> xt_cr[3], yt_cr[3];
  for (int i = 0; i < 3; ++i) {
    int j = nx(i), k = pv(i);
    if (dxt[i] != 0.0) {
      Expansion<8> t8b, t8c;
      Expansion<16> t16a, t16b, t16c;
      Expansion<32> t32a;
      Expansion<48> t48;
      scale_into(xt_cr[i], cr[i], dxt[i]);
      scale_into(t16a, xt_cr[i], 2.0 * dx[i]);
      scale_into(t8b, sq[k], dxt[i]);
      scale_into(t16b, t8b, dy[j]);
      scale_into(t8c, sq[j], dxt[i]);
      scale_into(t16c, t8c, -dy[k]);
      sum_into(t32a, t16a, t16b);
      sum_into(t48, t16c, t32a);
      add(t48);
    }
    if (dyt[i] != 0.0) {
      Expansion<8> t8b, t8c;
      Expansion<16> t16a, t16b, t16c;
      Expansion<32> t32a;
      Expansion<48> t48;
      scale_into(yt_cr[i], cr[i], dyt[i]);
      scale_into(t16a, yt_cr[i], 2.0 * dy[i]);
      scale_into(t8b, sq[j], dyt[i]);
      scale_into(t16b, t8b, dx[k]);
      scale_into(t8c, sq[k], dyt[i]);
      scale_into(t16c, t8c, -dx[j]);
      sum_into(t32a, t16a, t16b);
      sum_into(t48, t16c, t32a);
      add(t48);
    }
  }

  // Second- and higher-order tail terms.
  for (int i = 0; i < 3; ++i) {
    if (!has_tail(i))
      continue;
    int j = nx(i), k = pv(i);
    // bct and bctt of the original (rotated): the tail parts of cr[i].
    Expansion<8> crt;
    Expansion<4> crtt;
    if (has_tail(j) || has_tail(k)) {
      Expansion<4> u = four(
          two_two_sum(two_product(dxt[j], dy[k]), two_product(dx[j], dyt[k])));
      Expansion<4> v = four(two_two_sum(two_product(dxt[k], -dy[j]),
                                        two_product(dx[k], -dyt[j])));
      sum_into(crt, u, v);
      crtt = four(two_two_diff(two_product(dxt[j], dyt[k]),
                               two_product(dxt[k], dyt[j])));
    } else {
      crt.v[0] = 0.0;
      crt.n = 1;
      crtt.v[0] = 0.0;
      crtt.n = 1;
    }
    if (dxt[i] != 0.0) {
      Expansion<16> t16a, t16b, xtcrt;
      Expansion<32> t32a, t32b;
      Expansion<48> t48;
      Expansion<64> t64;
      Expansion<8> t8, xtcrtt;
      scale_into(t16a, xt_cr[i], dxt[i]);
      scale_into(xtcrt, crt, dxt[i]);
      scale_into(t32a, xtcrt, 2.0 * dx[i]);
      sum_into(t48, t16a, t32a);
      add(t48);
      if (dyt[j] != 0.0) {
        scale_into(t8, sq[k], dxt[i]);
        scale_into(t16a, t8, dyt[j]);
        add(t16a);
      }
      if (dyt[k] != 0.0) {
        scale_into(t8, sq[j], -dxt[i]);
        scale_into(t16a, t8, dyt[k]);
        add(t16a);
      }
      scale_into(t32a, xtcrt, dxt[i]);
      scale_into(xtcrtt, crtt, dxt[i]);
      scale_into(t16a, xtcrtt, 2.0 * dx[i]);
      scale_into(t16b, xtcrtt, dxt[i]);
      sum_into(t32b, t16a, t16b);
      sum_into(t64, t32a, t32b);
      add(t64);
    }
    if (dyt[i] != 0.0) {
      Expansion<16> t16a, t16b, ytcrt;
      Expansion<32> t32a, t32b;
      Expansion<48> t48;
      Expansion<64> t64;
      Expansion<8> ytcrtt;
      scale_into(t16a, yt_cr[i], dyt[i]);
      scale_into(ytcrt, crt, dyt[i]);
      scale_into(t32a, ytcrt, 2.0 * dy[i]);
      sum_into(t48, t16a, t32a);
      add(t48);
      scale_into(t32a, ytcrt, dyt[i]);
      scale_into(ytcrtt, crtt, dyt[i]);
      scale_into(t16a, ytcrtt, 2.0 * dy[i]);
      scale_into(t16b, ytcrtt, dyt[i]);
      sum_into(t32b, t16a, t16b);
      sum_into(t64, t32a, t32b);
      add(t64);
    }
  }
  return fin[now].v[fin[now].n - 1];
}

// The floating-point filters of orient2d and incircle accept their answer
// only if the bound is finite and at least the smallest normal double (the
// bound itself has not underflowed); otherwise the adaptive stages decide.
bool filter_decides(double det, double errbound) {
  return std::isfinite(det) && std::isfinite(errbound) &&
         errbound >= std::numeric_limits<double>::min() &&
         std::abs(det) > errbound;
}

// Shewchuk's error bounds hold only if no product underflowed: a product
// that drops below the smallest normal double loses absolute accuracy that
// a relative bound does not cover, and a later product can magnify that
// loss far beyond the bound (for example a tiny 2x2 minor times a huge
// lifted distance in incircle). Inside the exact range no product can
// underflow, so the plain product is used there; outside it, each product
// records whether it underflowed, and the filter's answer is then refused.
// Overflow needs no check: it makes the bound infinite or NaN, which
// filter_decides refuses.
struct PlainProduct {
  double operator()(double x, double y) const { return x * y; }
  bool underflowed() const { return false; }
};

struct CheckedProduct {
  double operator()(double x, double y) {
    double p = x * y;
    if (std::abs(p) < std::numeric_limits<double>::min() && x != 0 && y != 0)
      underflow = true;
    return p;
  }
  bool underflowed() const { return underflow; }
  bool underflow = false;
};

struct FilterResult {
  double det, detsum;
  bool decided;
};

template <class Product>
FilterResult orient_filter(Point a, Point b, Point c, Product &&mul) {
  double detleft = mul(a.x - c.x, b.y - c.y);
  double detright = mul(a.y - c.y, b.x - c.x);
  double det = detleft - detright;
  double detsum = std::abs(detleft) + std::abs(detright);
  bool decided =
      filter_decides(det, mul(ccwerrboundA, detsum)) && !mul.underflowed();
  return {det, detsum, decided};
}

template <class Product>
FilterResult incircle_filter(Point a, Point b, Point c, Point d,
                             Product &&mul) {
  double adx = a.x - d.x, bdx = b.x - d.x, cdx = c.x - d.x;
  double ady = a.y - d.y, bdy = b.y - d.y, cdy = c.y - d.y;
  double bdxcdy = mul(bdx, cdy), cdxbdy = mul(cdx, bdy);
  double alift = mul(adx, adx) + mul(ady, ady);
  double cdxady = mul(cdx, ady), adxcdy = mul(adx, cdy);
  double blift = mul(bdx, bdx) + mul(bdy, bdy);
  double adxbdy = mul(adx, bdy), bdxady = mul(bdx, ady);
  double clift = mul(cdx, cdx) + mul(cdy, cdy);
  double det = mul(alift, bdxcdy - cdxbdy) + mul(blift, cdxady - adxcdy) +
               mul(clift, adxbdy - bdxady);
  double permanent = mul(std::abs(bdxcdy) + std::abs(cdxbdy), alift) +
                     mul(std::abs(cdxady) + std::abs(adxcdy), blift) +
                     mul(std::abs(adxbdy) + std::abs(bdxady), clift);
  bool decided =
      filter_decides(det, mul(iccerrboundA, permanent)) && !mul.underflowed();
  return {det, permanent, decided};
}

} // namespace

// Inside the exact range: filter, then the adaptive stages. Outside it:
// the filter with checked products, then the integer evaluation.
int orient(Point a, Point b, Point c) {
  check_finite({a.x, a.y, b.x, b.y, c.x, c.y});
  if (in_exact_range({a.x, a.y, b.x, b.y, c.x, c.y})) {
    FilterResult f = orient_filter(a, b, c, PlainProduct{});
    if (f.decided)
      return sign(f.det);
    return sign(orient2d_adapt(a, b, c, f.detsum));
  }
  FilterResult f = orient_filter(a, b, c, CheckedProduct{});
  if (f.decided)
    return sign(f.det);
  return integer_orient(a, b, c);
}

int incircle(Point a, Point b, Point c, Point d) {
  check_finite({a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y});
  if (in_exact_range({a.x, a.y, b.x, b.y, c.x, c.y, d.x, d.y})) {
    FilterResult f = incircle_filter(a, b, c, d, PlainProduct{});
    if (f.decided)
      return sign(f.det);
    return sign(incircle_adapt(a, b, c, d, f.detsum));
  }
  FilterResult f = incircle_filter(a, b, c, d, CheckedProduct{});
  if (f.decided)
    return sign(f.det);
  return integer_incircle(a, b, c, d);
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

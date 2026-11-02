#pragma once

#include "curve_segment.hpp"
#include <istream>
#include <ostream>
#include <type_traits>

namespace discretization::curve_io {
// PDE2D_MESH v3 wire IDs. Never renumber or derive these from variant order.
// The primary template is intentionally undefined: new alternatives must
// explicitly opt into the format, independently of their geometry operations.
template <class T> struct Codec;
inline double real(std::istream &in) {
  double value;
  if (!(in >> value) || !std::isfinite(value))
    throw std::runtime_error("Invalid finite curve value");
  return value;
}
inline Point2 point(std::istream &in) {
  double x = real(in), y = real(in);
  return {x, y};
}
inline void point(std::ostream &out, Point2 p) {
  out << ' ' << p.x << ' ' << p.y;
}
template <> struct Codec<LineCurve> {
  static constexpr unsigned id = 0;
  static void write(std::ostream &out, const LineCurve &c) {
    point(out, c.start);
    point(out, c.end);
  }
  static LineCurve read(std::istream &in) {
    auto a = point(in), b = point(in);
    return {a, b};
  }
};
template <> struct Codec<QuadraticBezier> {
  static constexpr unsigned id = 1;
  static void write(std::ostream &out, const QuadraticBezier &c) {
    point(out, c.start);
    point(out, c.control);
    point(out, c.end);
  }
  static QuadraticBezier read(std::istream &in) {
    auto a = point(in), b = point(in), c = point(in);
    return {a, b, c};
  }
};
template <> struct Codec<CubicBezier> {
  static constexpr unsigned id = 2;
  static void write(std::ostream &out, const CubicBezier &c) {
    point(out, c.start);
    point(out, c.control1);
    point(out, c.control2);
    point(out, c.end);
  }
  static CubicBezier read(std::istream &in) {
    auto a = point(in), b = point(in), c = point(in), d = point(in);
    return {a, b, c, d};
  }
};
template <> struct Codec<EllipticalArc> {
  static constexpr unsigned id = 3;
  static void write(std::ostream &out, const EllipticalArc &c) {
    point(out, c.center);
    out << ' ' << c.radius_x << ' ' << c.radius_y << ' ' << c.rotation << ' '
        << c.start_angle << ' ' << c.sweep_angle;
  }
  static EllipticalArc read(std::istream &in) {
    auto p = point(in);
    double rx = real(in), ry = real(in), rotation = real(in), start = real(in),
           sweep = real(in);
    return {p, rx, ry, rotation, start, sweep};
  }
};
// This also makes compatibility testable with differently ordered variants.
template <class Variant, std::size_t... I>
consteval bool uniqueIds(std::index_sequence<I...>) {
  constexpr unsigned ids[] = {
      Codec<std::variant_alternative_t<I, Variant>>::id...};
  for (std::size_t a = 0; a < sizeof...(I); ++a)
    for (std::size_t b = a + 1; b < sizeof...(I); ++b)
      if (ids[a] == ids[b])
        return false;
  return true;
}
template <class Variant = CurveSegment, std::size_t I = 0>
Variant read(std::istream &in, unsigned id) {
  static_assert(uniqueIds<Variant>(
                    std::make_index_sequence<std::variant_size_v<Variant>>{}),
                "Duplicate curve wire ID");
  if constexpr (I < std::variant_size_v<Variant>) {
    using T = std::variant_alternative_t<I, Variant>;
    if (id == Codec<T>::id) {
      auto c = Codec<T>::read(in);
      c.validate();
      return c;
    }
    return read<Variant, I + 1>(in, id);
  } else
    throw std::runtime_error("Unknown source curve kind");
}
template <class Variant> void write(std::ostream &out, const Variant &curve) {
  std::visit(
      [&](const auto &c) {
        using T = std::decay_t<decltype(c)>;
        out << Codec<T>::id;
        Codec<T>::write(out, c);
      },
      curve);
}
} // namespace discretization::curve_io

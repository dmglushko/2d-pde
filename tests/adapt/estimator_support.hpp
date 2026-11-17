#pragma once

// Helpers for the estimator and adaptive loop tests: the true error of a
// P1 solution in an energy norm, computed independently of the estimators
// (exact gradients, a high-order rule, subdivision near a singular point).

#include "pde2d/fem/quadrature.hpp"
#include "pde2d/geometry/point.hpp"
#include "pde2d/mesh/mesh.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <optional>
#include <vector>

namespace pde2d::test {

using Gradient = std::function<geometry::Point(double, double)>;
// The symmetric weight W at a point, as (w_xx, w_xy, w_yy).
using Weight = std::function<std::array<double, 3>(double, double)>;

inline std::array<double, 3> identity_weight(double, double) {
  return {1.0, 0.0, 1.0};
}

namespace detail {

// The integral over triangle p of (grad u - g) . W (grad u - g), with g
// constant; subdivided `depth` times into four when a corner is
// `singular`.
inline double weighted_square(const std::array<geometry::Point, 3> &p,
                              const geometry::Point &g, const Gradient &exact,
                              const Weight &weight,
                              const std::optional<geometry::Point> &singular,
                              int depth) {
  bool at_singular = false;
  if (singular)
    for (const auto &q : p)
      at_singular = at_singular || (std::abs(q.x - singular->x) < 1e-14 &&
                                    std::abs(q.y - singular->y) < 1e-14);
  if (at_singular && depth > 0) {
    const auto mid = [](const geometry::Point &a, const geometry::Point &b) {
      return geometry::Point{(a.x + b.x) / 2, (a.y + b.y) / 2};
    };
    const auto m01 = mid(p[0], p[1]), m12 = mid(p[1], p[2]),
               m20 = mid(p[2], p[0]);
    double sum = 0.0;
    for (const auto &child :
         {std::array{p[0], m01, m20}, std::array{m01, p[1], m12},
          std::array{m20, m12, p[2]}, std::array{m01, m12, m20}})
      sum += weighted_square(child, g, exact, weight, singular, depth - 1);
    return sum;
  }
  const double area = ((p[1].x - p[0].x) * (p[2].y - p[0].y) -
                       (p[2].x - p[0].x) * (p[1].y - p[0].y)) /
                      2;
  const fem::TriangleRule rule = fem::triangle_rule(5);
  double sum = 0.0;
  for (const auto &q : rule.points) {
    const geometry::Point x = fem::point_at(q.barycentric, p);
    const geometry::Point e = exact(x.x, x.y);
    const double dx = e.x - g.x, dy = e.y - g.y;
    const auto [wxx, wxy, wyy] = weight(x.x, x.y);
    sum += q.weight * (dx * (wxx * dx + wxy * dy) + dy * (wxy * dx + wyy * dy));
  }
  return area * sum;
}

} // namespace detail

// sqrt of the sum over the triangles of the integral of
// (grad u - grad u_h) . W (grad u - grad u_h).
inline double
energy_error(const mesh::Mesh &m, const std::vector<double> &values,
             const Gradient &exact, const Weight &weight = identity_weight,
             std::optional<geometry::Point> singular = {}, int depth = 8) {
  double sum = 0.0;
  for (const auto &triangle : m.triangles) {
    std::array<geometry::Point, 3> p{};
    for (std::size_t i = 0; i < 3; ++i)
      p[i] = m.nodes[triangle.nodes[i]];
    const double det = (p[1].x - p[0].x) * (p[2].y - p[0].y) -
                       (p[2].x - p[0].x) * (p[1].y - p[0].y);
    geometry::Point g{};
    for (std::size_t i = 0; i < 3; ++i) {
      const auto &pj = p[(i + 1) % 3], &pk = p[(i + 2) % 3];
      const double u = values[triangle.nodes[i]];
      g.x += u * (pj.y - pk.y) / det;
      g.y += u * (pk.x - pj.x) / det;
    }
    sum += detail::weighted_square(p, g, exact, weight, singular, depth);
  }
  return std::sqrt(sum);
}

} // namespace pde2d::test

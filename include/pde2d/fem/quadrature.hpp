#pragma once

// The quadrature rules of the library, in one place. A computation asks
// for the polynomial degree it needs integrated exactly and gets the
// cheapest rule here that does it; nobody else stores points or weights.
// The degree is chosen in code by each computation (assembly: degree 2 on
// triangles, 3 on edges), never by a global setting: different integrals
// need different degrees.
//
// Every rule has its points inside the element and positive weights.

#include "pde2d/geometry/point.hpp"

#include <array>
#include <span>

namespace pde2d::fem {

// A point of a triangle rule: barycentric coordinates (they sum to 1) and
// a weight. The weights of a rule sum to 1, so
//   integral over T of f  ≈  area(T) * sum_q weight_q * f(x_q).
struct TrianglePoint {
  std::array<double, 3> barycentric{};
  double weight = 0.0;
};

struct TriangleRule {
  // The highest degree of polynomials it integrates exactly.
  int exact_degree = 0;
  std::span<const TrianglePoint> points{};
};

// A point of an edge rule: position s in [0, 1] along the edge (0 at its
// first node) and a weight. The weights sum to 1, so
//   integral over E of f  ≈  length(E) * sum_q weight_q * f(x(s_q)).
struct EdgePoint {
  double s = 0.0;
  double weight = 0.0;
};

struct EdgeRule {
  int exact_degree = 0;
  std::span<const EdgePoint> points{};
};

// The largest exact degrees available.
inline constexpr int max_triangle_degree = 5;
inline constexpr int max_edge_degree = 9;

// The rule with the fewest points that is exact for polynomials of degree
// `exact_degree` (0 <= exact_degree <= max_triangle_degree):
//   degree 0, 1: the centroid (1 point);
//   degree 2:    (2/3, 1/6, 1/6) and its rotations (3 points), the point
//                with 2/3 at vertex q coming q-th;
//   degree 3, 4: Dunavant's 6-point rule;
//   degree 5:    Radon's 7-point rule.
// Throws std::invalid_argument for a degree outside that range.
TriangleRule triangle_rule(int exact_degree);

// Gauss-Legendre on [0, 1] with the fewest points exact for degree
// `exact_degree` (n points are exact for degree 2n - 1; 0 <= exact_degree
// <= max_edge_degree, up to 5 points), the points in increasing s.
// Throws std::invalid_argument for a degree outside that range.
EdgeRule edge_rule(int exact_degree);

// The point of triangle (p0, p1, p2) with the given barycentric
// coordinates: l0 p0 + l1 p1 + l2 p2.
geometry::Point point_at(const std::array<double, 3> &barycentric,
                         const std::array<geometry::Point, 3> &nodes);

// The point at position s along the edge from a to b: a + s (b - a).
geometry::Point point_at(double s, const geometry::Point &a,
                         const geometry::Point &b);

} // namespace pde2d::fem

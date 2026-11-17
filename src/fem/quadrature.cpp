#include "pde2d/fem/quadrature.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::fem {

namespace {

// The three rotations of (1 - 2a, a, a), weight w each: the point with
// 1 - 2a at vertex q comes q-th.
void add_rotations(std::vector<TrianglePoint> &rule, double a, double w) {
  for (std::size_t q = 0; q < 3; ++q) {
    TrianglePoint p{.barycentric = {a, a, a}, .weight = w};
    p.barycentric[q] = 1.0 - 2.0 * a;
    rule.push_back(p);
  }
}

std::vector<TrianglePoint> centroid_rule() {
  return {{.barycentric = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}, .weight = 1.0}};
}

std::vector<TrianglePoint> three_point_rule() {
  // Written out (not 1 - 2a) so the coordinates are exactly 2/3 and 1/6.
  std::vector<TrianglePoint> rule;
  for (std::size_t q = 0; q < 3; ++q) {
    TrianglePoint p{.barycentric = {1.0 / 6.0, 1.0 / 6.0, 1.0 / 6.0},
                    .weight = 1.0 / 3.0};
    p.barycentric[q] = 2.0 / 3.0;
    rule.push_back(p);
  }
  return rule;
}

// D. A. Dunavant, "High degree efficient symmetrical Gaussian quadrature
// rules for the triangle", 1985: degree 4, 6 points.
std::vector<TrianglePoint> dunavant_rule() {
  std::vector<TrianglePoint> rule;
  add_rotations(rule, 0.445948490915964886, 0.223381589678011466);
  add_rotations(rule, 0.0915762135097707435, 0.109951743655321868);
  return rule;
}

// Radon's 7-point rule, degree 5, in closed form.
std::vector<TrianglePoint> radon_rule() {
  const double r = std::sqrt(15.0);
  std::vector<TrianglePoint> rule = {
      {.barycentric = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}, .weight = 9.0 / 40.0}};
  add_rotations(rule, (6.0 - r) / 21.0, (155.0 - r) / 1200.0);
  add_rotations(rule, (6.0 + r) / 21.0, (155.0 + r) / 1200.0);
  return rule;
}

// Gauss-Legendre with n points on [-1, 1] as (x, weight) for x >= 0,
// mapped to [0, 1]: s = (1 -+ x) / 2, weight / 2.
std::vector<EdgePoint> gauss_legendre(int n) {
  struct Node {
    double x, w;
  };
  std::vector<Node> half;
  switch (n) {
  case 1:
    half = {{0.0, 2.0}};
    break;
  case 2:
    half = {{1.0 / std::sqrt(3.0), 1.0}};
    break;
  case 3:
    half = {{0.0, 8.0 / 9.0}, {std::sqrt(3.0 / 5.0), 5.0 / 9.0}};
    break;
  case 4: {
    const double r = 2.0 / 7.0 * std::sqrt(6.0 / 5.0), s = std::sqrt(30.0);
    half = {{std::sqrt(3.0 / 7.0 - r), (18.0 + s) / 36.0},
            {std::sqrt(3.0 / 7.0 + r), (18.0 - s) / 36.0}};
    break;
  }
  case 5: {
    const double r = 2.0 * std::sqrt(10.0 / 7.0), s = 13.0 * std::sqrt(70.0);
    half = {{0.0, 128.0 / 225.0},
            {std::sqrt(5.0 - r) / 3.0, (322.0 + s) / 900.0},
            {std::sqrt(5.0 + r) / 3.0, (322.0 - s) / 900.0}};
    break;
  }
  default:
    throw std::logic_error("gauss_legendre: no rule with this many points");
  }
  // Increasing s: the negative x from the largest |x| down, then the
  // rest. s = 1/2 -+ x/2 (multiplying by 1/2 is exact).
  std::vector<EdgePoint> rule;
  for (auto it = half.rbegin(); it != half.rend(); ++it)
    if (it->x != 0.0)
      rule.push_back({.s = 0.5 - 0.5 * it->x, .weight = 0.5 * it->w});
  for (const Node &node : half)
    rule.push_back({.s = 0.5 + 0.5 * node.x, .weight = 0.5 * node.w});
  return rule;
}

void check_degree(int degree, int max, const char *function) {
  if (degree < 0 || degree > max)
    throw std::invalid_argument(std::string(function) + ": exact degree " +
                                std::to_string(degree) +
                                " is not available (0 to " +
                                std::to_string(max) + ")");
}

} // namespace

TriangleRule triangle_rule(int exact_degree) {
  check_degree(exact_degree, max_triangle_degree, "triangle_rule");
  // Built once (thread-safe initialization), never changed.
  static const std::vector<TrianglePoint> centroid = centroid_rule(),
                                          three = three_point_rule(),
                                          dunavant = dunavant_rule(),
                                          radon = radon_rule();
  if (exact_degree <= 1)
    return {.exact_degree = 1, .points = centroid};
  if (exact_degree == 2)
    return {.exact_degree = 2, .points = three};
  if (exact_degree <= 4)
    return {.exact_degree = 4, .points = dunavant};
  return {.exact_degree = 5, .points = radon};
}

EdgeRule edge_rule(int exact_degree) {
  check_degree(exact_degree, max_edge_degree, "edge_rule");
  static const std::array<std::vector<EdgePoint>, 5> rules = {
      gauss_legendre(1), gauss_legendre(2), gauss_legendre(3),
      gauss_legendre(4), gauss_legendre(5)};
  // n points are exact for degree 2n - 1.
  const int n = exact_degree / 2 + 1;
  return {.exact_degree = 2 * n - 1,
          .points = rules[static_cast<std::size_t>(n - 1)]};
}

geometry::Point point_at(const std::array<double, 3> &l,
                         const std::array<geometry::Point, 3> &p) {
  return {l[0] * p[0].x + l[1] * p[1].x + l[2] * p[2].x,
          l[0] * p[0].y + l[1] * p[1].y + l[2] * p[2].y};
}

geometry::Point point_at(double s, const geometry::Point &a,
                         const geometry::Point &b) {
  return {a.x + s * (b.x - a.x), a.y + s * (b.y - a.y)};
}

} // namespace pde2d::fem

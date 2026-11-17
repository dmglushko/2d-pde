// The quadrature rules against exact integrals of monomials:
//   over a triangle, of l0^a l1^b l2^c (barycentric coordinates):
//     area * 2 a! b! c! / (a + b + c + 2)!
//   over [0, 1], of s^k: 1 / (k + 1).
// Each rule must be exact up to its declared degree and not beyond.

#include "support/check.hpp"

#include "pde2d/fem/quadrature.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

using namespace pde2d::fem;
using namespace pde2d::test;

namespace {

double factorial(int n) {
  double f = 1.0;
  for (int i = 2; i <= n; ++i)
    f *= i;
  return f;
}

// The rule's sum for l0^a l1^b l2^c with weights summing to 1, against
// the exact integral divided by the area.
double triangle_error(const TriangleRule &rule, int a, int b, int c) {
  double sum = 0.0;
  for (const auto &p : rule.points)
    sum += p.weight * std::pow(p.barycentric[0], a) *
           std::pow(p.barycentric[1], b) * std::pow(p.barycentric[2], c);
  const double exact = 2.0 * factorial(a) * factorial(b) * factorial(c) /
                       factorial(a + b + c + 2);
  return std::abs(sum - exact);
}

double edge_error(const EdgeRule &rule, int k) {
  double sum = 0.0;
  for (const auto &p : rule.points)
    sum += p.weight * std::pow(p.s, k);
  return std::abs(sum - 1.0 / (k + 1));
}

// The largest error over all monomials of exactly degree d.
double triangle_error_of_degree(const TriangleRule &rule, int d) {
  double worst = 0.0;
  for (int a = 0; a <= d; ++a)
    for (int b = 0; a + b <= d; ++b)
      worst = std::max(worst, triangle_error(rule, a, b, d - a - b));
  return worst;
}

void check_triangle_rule(int requested) {
  const TriangleRule rule = triangle_rule(requested);
  const std::string name = "triangle_rule(" + std::to_string(requested) + ")";
  check(rule.exact_degree >= requested, name + " is exact for the degree");
  double weights = 0.0;
  for (const auto &p : rule.points) {
    check(p.weight > 0.0, name + ": positive weight");
    double sum = 0.0;
    for (double l : p.barycentric) {
      check(l > 0.0 && l < 1.0, name + ": point inside");
      sum += l;
    }
    near(sum, 1.0, 1e-15, name + ": barycentric coordinates sum to 1");
    weights += p.weight;
  }
  near(weights, 1.0, 1e-15, name + ": weights sum to 1");
  for (int d = 0; d <= rule.exact_degree; ++d)
    check(triangle_error_of_degree(rule, d) < 1e-15,
          name + ": exact for degree " + std::to_string(d));
  check(triangle_error_of_degree(rule, rule.exact_degree + 1) > 1e-6,
        name + ": not exact one degree higher (the declared degree is right)");
}

void check_edge_rule(int requested) {
  const EdgeRule rule = edge_rule(requested);
  const std::string name = "edge_rule(" + std::to_string(requested) + ")";
  check(rule.exact_degree >= requested, name + " is exact for the degree");
  check(static_cast<int>(rule.points.size()) * 2 - 1 == rule.exact_degree,
        name + ": n points, exact for 2n - 1");
  double weights = 0.0, previous = 0.0;
  for (const auto &p : rule.points) {
    check(p.weight > 0.0, name + ": positive weight");
    check(p.s > previous && p.s < 1.0, name + ": increasing points inside");
    previous = p.s;
    weights += p.weight;
  }
  near(weights, 1.0, 1e-15, name + ": weights sum to 1");
  for (int k = 0; k <= rule.exact_degree; ++k)
    check(edge_error(rule, k) < 1e-15,
          name + ": exact for degree " + std::to_string(k));
  check(edge_error(rule, rule.exact_degree + 1) > 1e-6,
        name + ": not exact one degree higher");
}

} // namespace

int main() {
  return run({
      {"triangle_rules_exact",
       [] {
         for (int d = 0; d <= max_triangle_degree; ++d)
           check_triangle_rule(d);
         check(triangle_rule(0).points.size() == 1, "degree 0: 1 point");
         check(triangle_rule(2).points.size() == 3, "degree 2: 3 points");
         check(triangle_rule(3).points.size() == 6, "degree 3: 6 points");
         check(triangle_rule(5).points.size() == 7, "degree 5: 7 points");
       }},
      {"three_point_rule_layout",
       [] {
         // Assembly relies on it: (2/3, 1/6, 1/6), 2/3 at vertex q for
         // the q-th point, weight 1/3.
         const auto rule = triangle_rule(2);
         for (std::size_t q = 0; q < 3; ++q) {
           near(rule.points[q].weight, 1.0 / 3.0, 0, "weight 1/3");
           for (std::size_t i = 0; i < 3; ++i)
             near(rule.points[q].barycentric[i], q == i ? 2.0 / 3.0 : 1.0 / 6.0,
                  0, "coordinate");
         }
       }},
      {"edge_rules_exact",
       [] {
         for (int d = 0; d <= max_edge_degree; ++d)
           check_edge_rule(d);
         const auto two = edge_rule(3);
         near(two.points[0].s, 0.5 - 0.5 / std::sqrt(3.0), 0,
              "2-point Gauss-Legendre");
       }},
      {"degree_out_of_range",
       [] {
         expect_throws<std::invalid_argument>([] { (void)triangle_rule(-1); },
                                              "triangle_rule: exact degree -1");
         expect_throws<std::invalid_argument>(
             [] { (void)triangle_rule(max_triangle_degree + 1); },
             "is not available");
         expect_throws<std::invalid_argument>(
             [] { (void)edge_rule(max_edge_degree + 1); },
             "edge_rule: exact degree");
       }},
      {"points_on_elements",
       [] {
         const std::array<pde2d::geometry::Point, 3> t{
             {{1, 1}, {3, 1}, {1, 5}}};
         const auto p = point_at({0.5, 0.25, 0.25}, t);
         near(p.x, 1.5, 1e-15, "x");
         near(p.y, 2.0, 1e-15, "y");
         const auto e = point_at(0.25, {0, 0}, {4, 8});
         near(e.x, 1.0, 1e-15, "edge x");
         near(e.y, 2.0, 1e-15, "edge y");
       }},
  });
}

#include "fem/quadrature.hpp"
#include "support/check.hpp"
#include <iostream>
#include <limits>
#include <array>
#include <vector>

using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
void near(long double a, long double b) { pde2d_test::near_rel(a, b, 3e-13L); }
} // namespace
void quadrature_rules() {
  check(pde2d::fem::triangle_quadrature().size() == 3,
        "Baseline must have three points");
  for (int degree = 0; degree <= 64; ++degree) {
    const auto edge = pde2d::fem::edge_quadrature(degree);
    const auto triangle = pde2d::fem::triangle_quadrature(degree);
    long double edge_sum = 0, triangle_sum = 0;
    for (auto p : edge) {
      check(p.coordinate > 0 && p.coordinate < 1 && p.weight > 0,
            "Invalid edge point");
      edge_sum += p.weight;
    }
    for (auto p : triangle) {
      for (double b : p.barycentric)
        check(b > 0 && b < 1, "Invalid barycentric coordinate");
      near(p.barycentric[0] + p.barycentric[1] + p.barycentric[2], 1);
      check(p.weight > 0, "Nonpositive triangle weight");
      triangle_sum += p.weight;
    }
    near(edge_sum, 1);
    near(triangle_sum, 1);
    // Recurrence caches preserve every degree/monomial check below.
    std::vector<std::array<long double, 65>> ep(edge.size()),
        xp(triangle.size()), yp(triangle.size());
    for (std::size_t k = 0; k < edge.size(); ++k) {
      ep[k][0] = 1;
      for (int i = 1; i <= degree; ++i)
        ep[k][i] = ep[k][i - 1] * edge[k].coordinate;
    }
    for (std::size_t k = 0; k < triangle.size(); ++k) {
      xp[k][0] = yp[k][0] = 1;
      for (int i = 1; i <= degree; ++i) {
        xp[k][i] = xp[k][i - 1] * triangle[k].barycentric[1];
        yp[k][i] = yp[k][i - 1] * triangle[k].barycentric[2];
      }
    }
    for (int i = 0; i <= degree; ++i) {
      long double integral = 0;
      for (std::size_t k = 0; k < edge.size(); ++k)
        integral += edge[k].weight * ep[k][i];
      near(integral, 1.L / (i + 1));
      for (int j = 0; j <= degree - i; ++j) {
        integral = 0;
        for (std::size_t k = 0; k < triangle.size(); ++k)
          integral += triangle[k].weight * xp[k][i] * yp[k][j];
        // Normalized reference-triangle integral: 2*i!*j!/(i+j+2)!.
        const long double exact =
            2 * std::exp(std::lgamma(static_cast<long double>(i + 1)) +
                         std::lgamma(static_cast<long double>(j + 1)) -
                         std::lgamma(static_cast<long double>(i + j + 3)));
        near(integral, exact);
      }
    }
  }
  for (int degree : {-1, 65, std::numeric_limits<int>::max(),
                     std::numeric_limits<int>::min()}) {
    expect_throws<std::invalid_argument>(
        [=] { pde2d::fem::edge_quadrature(degree); });
    expect_throws<std::invalid_argument>(
        [=] { pde2d::fem::triangle_quadrature(degree); });
  }
}
int main() { return pde2d_test::run({{"quadrature_rules", quadrature_rules}}); }

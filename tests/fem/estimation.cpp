// Error estimation and bulk marking. Oracles: the exact H1-seminorm error of
// manufactured solutions (error_norms and an element-wise quadrature of the
// exact gradient written out here), hand-computed Dorfler sets, and
// serial/OpenMP equality.
#include "discretization/geometry.hpp"
#include "discretization/square_mesh.hpp"
#include "discretization/triangulate.hpp"
#include "fem/error_norms.hpp"
#include "fem/estimation.hpp"
#include "fem/quadrature.hpp"
#include "fem/solver.hpp"
#include "support/check.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numbers>
#include <vector>
using namespace pde2d::fem;
using pde2d::discretization::Point2;
using pde2d::discretization::square_mesh;
using pde2d::discretization::TriangleMesh;
using pde2d_test::check;
using pde2d_test::expect_throws;

namespace {
Problem poisson(const Expr &exact) {
  Problem p;
  p.equation.source = -(diff(diff(exact, x), x) + diff(diff(exact, y), y));
  p.assign(dirichlet(exact), {"bottom", "right", "top", "left"});
  return p;
}

void effectivity_under_uniform_refinement() {
  const double pi = std::numbers::pi;
  const Expr exact = sin(pi * x) * sin(2 * pi * y) + x * x;
  const auto p = poisson(exact);
  double previous_estimate = 0, previous_error = 0;
  for (int n : {4, 8, 16, 32}) {
    auto mesh = square_mesh(n);
    auto u = solve_stationary(mesh, p, AssemblyOptions{6}).values;
    auto estimate = estimate_error(mesh, p, u);
    const double error = error_norms(mesh, u, exact).h1_seminorm;
    const double effectivity = estimate.global / error;
    std::cout << "n=" << n << " estimate=" << estimate.global
              << " error=" << error << " effectivity=" << effectivity << '\n';
    check(estimate.indicators.size() == mesh.elements.size());
    check(estimate.estimator == "gradient recovery (ZZ)");
    check(effectivity > 0.8 && effectivity < 1.25,
          "effectivity outside [0.8, 1.25]");
    if (previous_error > 0) {
      // Both roughly halve with h (first order in H1; n=4 is preasymptotic)
      // and the estimate tracks the error.
      const double rate_error = previous_error / error,
                   rate_estimate = previous_estimate / estimate.global;
      check(rate_error > 1.7 && rate_estimate > 1.7);
      check(std::abs(rate_estimate / rate_error - 1) < 0.1);
    }
    previous_estimate = estimate.global;
    previous_error = error;
  }
  // Unstructured CDT meshes lose the superconvergence of the structured
  // ones; the estimate must still stay close to the error.
  pde2d::discretization::RectangleGeometry square(1, 1);
  square.setCenter(.5, .5);
  for (double h : {.1, .05}) {
    pde2d::discretization::MeshOptions o;
    o.boundary_treatment =
        pde2d::discretization::BoundaryTreatment::SplitPolygonEdges;
    o.refinement.max_edge_length = h;
    auto mesh = pde2d::discretization::generate_mesh(square, o).mesh;
    auto u = solve_stationary(mesh, p, AssemblyOptions{6}).values;
    const double effectivity = estimate_error(mesh, p, u).global /
                               error_norms(mesh, u, exact).h1_seminorm;
    std::cout << "CDT h=" << h << " effectivity=" << effectivity << '\n';
    check(effectivity > 0.7 && effectivity < 1.4,
          "unstructured effectivity outside [0.7, 1.4]");
  }
}

// Exact element-wise H1-seminorm error for a Gaussian peak.
constexpr double k = 80, px = .3, py = .6;
std::array<double, 2> gaussian_gradient(double X, double Y) {
  const double g = std::exp(-k * ((X - px) * (X - px) + (Y - py) * (Y - py)));
  return {-2 * k * (X - px) * g, -2 * k * (Y - py) * g};
}
std::vector<double> exact_element_errors(const TriangleMesh &mesh,
                                         const Vector &u) {
  const auto rule = triangle_quadrature(8);
  std::vector<double> errors;
  for (const auto &t : mesh.elements) {
    auto a = mesh.nodes[t[0]], b = mesh.nodes[t[1]], c = mesh.nodes[t[2]];
    const double det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    const double gx = ((b.y - c.y) * u[t[0]] + (c.y - a.y) * u[t[1]] +
                       (a.y - b.y) * u[t[2]]) /
                      det,
                 gy = ((c.x - b.x) * u[t[0]] + (a.x - c.x) * u[t[1]] +
                       (b.x - a.x) * u[t[2]]) /
                      det;
    double sum = 0;
    for (const auto &q : rule) {
      const double X = q.barycentric[0] * a.x + q.barycentric[1] * b.x +
                       q.barycentric[2] * c.x,
                   Y = q.barycentric[0] * a.y + q.barycentric[1] * b.y +
                       q.barycentric[2] * c.y;
      auto g = gaussian_gradient(X, Y);
      sum += q.weight * det / 2 *
             ((gx - g[0]) * (gx - g[0]) + (gy - g[1]) * (gy - g[1]));
    }
    errors.push_back(std::sqrt(sum));
  }
  return errors;
}
double share_near_peak(const TriangleMesh &mesh, const std::vector<double> &eta,
                       double radius) {
  double near = 0, total = 0;
  for (std::size_t i = 0; i < mesh.elements.size(); ++i) {
    Point2 c;
    for (auto v : mesh.elements[i]) {
      c.x += mesh.nodes[v].x / 3;
      c.y += mesh.nodes[v].y / 3;
    }
    total += eta[i] * eta[i];
    if (std::hypot(c.x - px, c.y - py) < radius)
      near += eta[i] * eta[i];
  }
  return near / total;
}

void indicators_concentrate_at_peak() {
  const Expr exact = exp(-k * ((x - px) * (x - px) + (y - py) * (y - py)));
  const auto p = poisson(exact);
  auto mesh = square_mesh(32);
  auto u = solve_stationary(mesh, p, AssemblyOptions{8}).values;
  auto estimate = estimate_error(mesh, p, u);
  auto exact_errors = exact_element_errors(mesh, u);
  const double estimated_share = share_near_peak(mesh, estimate.indicators, .3),
               exact_share = share_near_peak(mesh, exact_errors, .3);
  std::cout << "share of squared error within 0.3 of the peak: estimate "
            << estimated_share << ", exact " << exact_share << '\n';
  check(exact_share > .95, "test premise: the error sits at the peak");
  check(estimated_share > .9, "indicators do not concentrate at the peak");
  // The elements the estimator ranks highest are mostly the ones with the
  // largest exact error.
  const std::size_t top = mesh.elements.size() / 10;
  auto ranked = [&](const std::vector<double> &v) {
    std::vector<std::size_t> order(v.size());
    std::iota(order.begin(), order.end(), std::size_t{0});
    std::stable_sort(order.begin(), order.end(),
                     [&](auto a, auto b) { return v[a] > v[b]; });
    order.resize(top);
    std::sort(order.begin(), order.end());
    return order;
  };
  auto a = ranked(estimate.indicators), b = ranked(exact_errors);
  std::vector<std::size_t> common;
  std::set_intersection(a.begin(), a.end(), b.begin(), b.end(),
                        std::back_inserter(common));
  std::cout << "top-10% overlap " << common.size() << " of " << top << '\n';
  check(common.size() > top * 6 / 10, "top-ranked elements disagree");
  // Bulk marking picks elements near the peak.
  auto marked = mark_bulk(estimate.indicators, .5);
  check(!marked.empty() && marked.size() < mesh.elements.size() / 5);
  double marked_exact = 0, total_exact = 0;
  for (auto e : exact_errors)
    total_exact += e * e;
  for (auto i : marked)
    marked_exact += exact_errors[i] * exact_errors[i];
  check(marked_exact > .3 * total_exact);
}

void bulk_marking() {
  // Squares 1, 9, 4, 0; total 14.
  const std::vector<double> eta{1, 3, 2, 0};
  check(mark_bulk(eta, .5) == std::vector<std::size_t>{1});
  check(mark_bulk(eta, .9) == std::vector<std::size_t>{1, 2});
  check(mark_bulk(eta, 1) == std::vector<std::size_t>{0, 1, 2});
  // Ties: lower index first.
  check(mark_bulk({1, 1, 1, 1}, .5) == std::vector<std::size_t>{0, 1});
  check(mark_bulk({0, 0}, .5).empty());
  check(mark_bulk({}, .5).empty());
  expect_throws<std::invalid_argument>([] { mark_bulk({1}, 0); },
                                       "fraction must be in (0, 1]");
  expect_throws<std::invalid_argument>([] { mark_bulk({1}, 1.5); }, "fraction");
  expect_throws<std::invalid_argument>([] { mark_bulk({-1}, .5); },
                                       "finite and nonnegative");
}

struct WrongSize final : ErrorEstimator {
  std::string name() const override { return "wrong size"; }
  ErrorEstimate estimate(const TriangleMesh &, const Problem &, const Vector &,
                         double) const override {
    return {{1.0}, 1.0, name()};
  }
};

void interface_and_validation() {
  auto mesh = square_mesh(4);
  Problem p = poisson(x * y);
  Vector u = Vector::Zero(static_cast<Eigen::Index>(mesh.nodes.size()));
  expect_throws<std::runtime_error>(
      [&] { estimate_error(mesh, p, u, 0, WrongSize{}); },
      "'wrong size' returned invalid indicators");
  expect_throws<std::invalid_argument>(
      [&] { estimate_error(mesh, p, Vector::Zero(2)); }, "one finite value");
  // P1 functions that are affine have zero recovered-gradient error.
  for (std::size_t i = 0; i < mesh.nodes.size(); ++i)
    u[static_cast<Eigen::Index>(i)] = 2 * mesh.nodes[i].x - mesh.nodes[i].y;
  auto zero = estimate_error(mesh, p, u);
  check(zero.global < 1e-13);
  if (!pde2d::openmp_available())
    return;
  auto fine = square_mesh(24);
  Vector v(static_cast<Eigen::Index>(fine.nodes.size()));
  for (std::size_t i = 0; i < fine.nodes.size(); ++i)
    v[static_cast<Eigen::Index>(i)] =
        std::sin(4 * fine.nodes[i].x) * std::exp(fine.nodes[i].y);
  GradientRecoveryEstimator parallel({pde2d::ExecutionBackend::OpenMP, 4, 1});
  auto a = estimate_error(fine, p, v),
       b = estimate_error(fine, p, v, 0, parallel);
  check(a.indicators == b.indicators && a.global == b.global,
        "serial and OpenMP estimates differ");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"effectivity_under_uniform_refinement",
        effectivity_under_uniform_refinement},
       {"indicators_concentrate_at_peak", indicators_concentrate_at_peak},
       {"bulk_marking", bulk_marking},
       {"interface_and_validation", interface_and_validation}});
}

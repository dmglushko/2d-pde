#include "pde2d/adapt/recovery_estimator.hpp"
#include "pde2d/fem/quadrature.hpp"

#include "estimator_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <stdexcept>

namespace pde2d::adapt {

namespace {

using geometry::Point;

// (G - grad u_h) is linear on a triangle, so its square times an A of
// degree at most 1 is a cubic.
constexpr int recovery_degree = 3;

// Relative size below which a negative determinant of A is rounding.
constexpr double definiteness_tolerance = 1e-12;

} // namespace

RecoveryEstimator::RecoveryEstimator(RecoveryEstimatorOptions options)
    : options_(options) {
  switch (options_.norm) {
  case RecoveryNorm::energy:
  case RecoveryNorm::gradient:
    return;
  }
  throw std::invalid_argument(
      "RecoveryEstimatorOptions::norm must be RecoveryNorm::energy or "
      "RecoveryNorm::gradient");
}

std::vector<double>
RecoveryEstimator::indicators(const fem::MeshedProblem &problem,
                              const solve::Solution &solution,
                              const TimeDerivatives & /*derivatives*/,
                              mesh::StageReport & /*report*/) const {
  const mesh::Mesh &mesh = *problem.mesh();
  const fem::CompiledEquation &e = problem.equation();
  const double t = solution.time;
  const bool energy = options_.norm == RecoveryNorm::energy;
  if (energy && e.a_xx.facts().zero && e.a_xy.facts().zero &&
      e.a_yy.facts().zero)
    throw std::invalid_argument(
        name() + ": the energy norm needs A, which is zero; use "
                 "RecoveryNorm::gradient");

  const std::vector<detail::TriangleData> data =
      detail::triangle_data(mesh, solution.values, name());

  // The recovered nodal gradients: area-weighted averages of the
  // gradients of the triangles around each node, summed in triangle order.
  std::vector<Point> recovered(mesh.nodes.size());
  std::vector<double> weight(mesh.nodes.size(), 0.0);
  for (std::size_t tri = 0; tri < data.size(); ++tri)
    for (std::size_t node : mesh.triangles[tri].nodes) {
      recovered[node].x += data[tri].area * data[tri].gradient.x;
      recovered[node].y += data[tri].area * data[tri].gradient.y;
      weight[node] += data[tri].area;
    }
  for (std::size_t i = 0; i < recovered.size(); ++i)
    if (weight[i] > 0.0) {
      recovered[i].x /= weight[i];
      recovered[i].y /= weight[i];
    }

  const fem::TriangleRule rule = fem::triangle_rule(recovery_degree);
  std::vector<double> eta(data.size());
  for (std::size_t tri = 0; tri < data.size(); ++tri) {
    const detail::TriangleData &d = data[tri];
    const auto &nodes = mesh.triangles[tri].nodes;
    double sum = 0.0;
    for (const fem::TrianglePoint &q : rule.points) {
      const auto &l = q.barycentric;
      const Point difference{
          l[0] * recovered[nodes[0]].x + l[1] * recovered[nodes[1]].x +
              l[2] * recovered[nodes[2]].x - d.gradient.x,
          l[0] * recovered[nodes[0]].y + l[1] * recovered[nodes[1]].y +
              l[2] * recovered[nodes[2]].y - d.gradient.y};
      double value = difference.x * difference.x + difference.y * difference.y;
      if (energy) {
        const Point p = fem::point_at(l, d.nodes);
        const double xx =
            detail::coefficient_value(e.a_xx, name(), "a_xx", p, t);
        const double xy =
            detail::coefficient_value(e.a_xy, name(), "a_xy", p, t) / 2;
        const double yy =
            detail::coefficient_value(e.a_yy, name(), "a_yy", p, t);
        const double scale =
            std::max({std::abs(xx), std::abs(xy), std::abs(yy)});
        if (xx * yy - xy * xy < -definiteness_tolerance * scale * scale)
          throw std::invalid_argument(std::format(
              "{}: A is indefinite at ({}, {}), t = {} (a_xx a_yy - a_xy^2 / "
              "4 < 0), so it has no energy norm; use RecoveryNorm::gradient",
              name(), p.x, p.y, t));
        value = std::abs(difference.x * (xx * difference.x + xy * difference.y) +
                         difference.y * (xy * difference.x + yy * difference.y));
      }
      sum += q.weight * value;
    }
    eta[tri] = std::sqrt(d.area * sum);
  }
  return eta;
}

} // namespace pde2d::adapt

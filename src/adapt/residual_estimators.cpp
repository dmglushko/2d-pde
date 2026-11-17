// The residual and the Kelly estimators: one computation of the residual
// terms (element residual, flux jumps, boundary residuals); Kelly takes
// the edge terms only and ignores the time terms.

#include "pde2d/adapt/kelly_estimator.hpp"
#include "pde2d/adapt/residual_estimator.hpp"
#include "pde2d/fem/quadrature.hpp"

#include "estimator_geometry.hpp"

#include <cmath>
#include <format>
#include <stdexcept>

namespace pde2d::adapt {

namespace {

using geometry::Point;

// The exact degrees asked of quadrature.hpp (see residual_estimator.hpp):
// with coefficients and data of degree at most 1, r and R_E are at most
// quadratic (c u_h, m1 u_t, alpha u_h) and J_E is linear (A linear times
// a constant gradient difference); the integrands are their squares.
constexpr int element_degree = 4;
constexpr int jump_degree = 2;
constexpr int boundary_degree = 4;

// The squares of the three kinds of terms, per triangle.
struct ResidualTerms {
  std::vector<double> element, jump, boundary;
};

// P1 interpolation of nodal values at barycentric coordinates.
double interpolate(std::span<const double> values,
                   const std::array<std::size_t, 3> &nodes,
                   const std::array<double, 3> &lambda) {
  return lambda[0] * values[nodes[0]] + lambda[1] * values[nodes[1]] +
         lambda[2] * values[nodes[2]];
}

bool a_is_zero(const fem::CompiledEquation &e) {
  return e.a_xx.facts().zero && e.a_xy.facts().zero && e.a_yy.facts().zero;
}

// n . A d with A = [[a_xx, a_xy / 2], [a_xy / 2, a_yy]] at p.
struct Flux {
  double xx = 0.0, xy = 0.0, yy = 0.0; // a_xx, a_xy / 2, a_yy
  double operator()(const Point &n, const Point &d) const {
    return n.x * (xx * d.x + xy * d.y) + n.y * (xy * d.x + yy * d.y);
  }
};

Flux flux_at(const fem::CompiledEquation &e, const std::string &who,
             const Point &p, double t) {
  return {.xx = detail::coefficient_value(e.a_xx, who, "a_xx", p, t),
          .xy = detail::coefficient_value(e.a_xy, who, "a_xy", p, t) / 2,
          .yy = detail::coefficient_value(e.a_yy, who, "a_yy", p, t)};
}

// with_element: the residual estimator (element residual and the time
// terms, from `derivatives`); otherwise Kelly (edge terms, no time terms).
ResidualTerms residual_terms(const fem::MeshedProblem &problem,
                             const solve::Solution &solution,
                             const TimeDerivatives &derivatives,
                             bool with_element, const std::string &who) {
  const mesh::Mesh &mesh = *problem.mesh();
  const fem::CompiledEquation &e = problem.equation();
  const double t = solution.time;
  const std::span<const double> u = solution.values;
  const std::size_t triangles = mesh.triangles.size();
  ResidualTerms terms{.element = std::vector<double>(triangles, 0.0),
                      .jump = std::vector<double>(triangles, 0.0),
                      .boundary = std::vector<double>(triangles, 0.0)};
  const std::vector<detail::TriangleData> data =
      detail::triangle_data(mesh, u, who);

  // The time terms this estimator evaluates: only the residual estimator,
  // and only for coefficients that are not zero as written.
  const bool use_u_t = with_element && !derivatives.u_t.empty();
  const bool use_u_tt = with_element && !derivatives.u_tt.empty();

  // Element residual r = f - b . grad u_h - c u_h - m1 u_t - m2 u_tt.
  if (with_element) {
    const fem::TriangleRule rule = fem::triangle_rule(element_degree);
    for (std::size_t tri = 0; tri < triangles; ++tri) {
      const detail::TriangleData &d = data[tri];
      const auto &nodes = mesh.triangles[tri].nodes;
      double sum = 0.0;
      for (const fem::TrianglePoint &q : rule.points) {
        const Point p = fem::point_at(q.barycentric, d.nodes);
        const auto c = [&](const fem::CompiledExpression &coefficient,
                           const char *label) {
          return detail::coefficient_value(coefficient, who, label, p, t);
        };
        double r = c(e.f, "f") - c(e.b_x, "b_x") * d.gradient.x -
                   c(e.b_y, "b_y") * d.gradient.y;
        if (!e.c.facts().zero)
          r -= c(e.c, "c") * interpolate(u, nodes, q.barycentric);
        if (!e.m1.facts().zero && use_u_t)
          r -= c(e.m1, "m1") *
               interpolate(derivatives.u_t, nodes, q.barycentric);
        if (!e.m2.facts().zero && use_u_tt)
          r -= c(e.m2, "m2") *
               interpolate(derivatives.u_tt, nodes, q.barycentric);
        sum += q.weight * r * r;
      }
      terms.element[tri] = d.diameter * d.diameter * d.area * sum;
    }
  }

  // Without A there is no flux: no jumps and no boundary residuals.
  if (a_is_zero(e))
    return terms;
  const detail::EdgeTopology topology = detail::edge_topology(mesh);

  // Jumps of the flux across interior edges, half to each side.
  {
    const fem::EdgeRule rule = fem::edge_rule(jump_degree);
    for (const detail::InteriorEdge &edge : topology.interior) {
      const Point a = mesh.nodes[edge.a], b = mesh.nodes[edge.b];
      const double length = std::hypot(b.x - a.x, b.y - a.y);
      const Point n{(b.y - a.y) / length, -(b.x - a.x) / length};
      const Point &g1 = data[edge.first].gradient,
                  &g2 = data[edge.second].gradient;
      const Point difference{g1.x - g2.x, g1.y - g2.y};
      double sum = 0.0;
      for (const fem::EdgePoint &q : rule.points) {
        const double jump =
            flux_at(e, who, fem::point_at(q.s, a, b), t)(n, difference);
        sum += q.weight * jump * jump;
      }
      const double half = 0.5 * length * length * sum; // 1/2 h_E ||J||^2
      terms.jump[edge.first] += half;
      terms.jump[edge.second] += half;
    }
  }

  // Residuals of natural and dynamic conditions.
  const fem::EdgeRule rule = fem::edge_rule(boundary_degree);
  for (std::size_t k = 0; k < mesh.boundary_edges.size(); ++k) {
    const mesh::BoundaryEdge &edge = mesh.boundary_edges[k];
    const fem::CompiledCondition &condition =
        problem.conditions()[edge.region];
    if (condition.kind == fem::ConditionKind::dirichlet)
      continue;
    const std::string &region = mesh.region_names[edge.region];
    const std::size_t tri = topology.boundary_triangle[k];
    const Point &gradient = data[tri].gradient;
    const auto [ia, ib] = edge.nodes;
    const Point a = mesh.nodes[ia], b = mesh.nodes[ib];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    // The domain is on the left: tau along the edge, n outward.
    const Point tau{(b.x - a.x) / length, (b.y - a.y) / length};
    const Point n{tau.y, -tau.x};
    const double u_n = gradient.x * n.x + gradient.y * n.y;
    const double u_tau = gradient.x * tau.x + gradient.y * tau.y;
    const bool use_gamma = use_u_t && !condition.gamma.facts().zero;
    double sum = 0.0;
    for (const fem::EdgePoint &q : rule.points) {
      const Point p = fem::point_at(q.s, a, b);
      const auto c = [&](const fem::CompiledExpression &coefficient,
                         const char *label) {
        return detail::coefficient_value(
            coefficient, std::format("{}: region '{}'", who, region),
            label, p, t);
      };
      const double kappa_n = flux_at(e, who, p, t)(n, n);
      const double beta_x = c(condition.beta_x, "beta_x");
      const double beta_y = c(condition.beta_y, "beta_y");
      const double beta_n = c(condition.beta_n, "beta_n");
      const double beta_n_hat = beta_n + beta_x * n.x + beta_y * n.y;
      if (beta_n_hat == 0.0)
        throw std::invalid_argument(std::format(
            "{}: region '{}': beta_n + beta_x n_x + beta_y n_y is zero at "
            "({}, {}) on the boundary edge from node {} to node {}, t = {}, "
            "so the condition cannot be solved for the normal derivative u_n",
            who, region, p.x, p.y, ia, ib, t));
      if (kappa_n == 0.0)
        continue; // no flux, so no residual
      const double u_here = (1.0 - q.s) * u[ia] + q.s * u[ib];
      double residual = c(condition.g, "g") -
                        c(condition.alpha, "alpha") * u_here -
                        beta_n * u_n -
                        c(condition.beta_tau, "beta_tau") * u_tau -
                        beta_x * gradient.x - beta_y * gradient.y;
      if (use_gamma)
        residual -= c(condition.gamma, "gamma") *
                    ((1.0 - q.s) * derivatives.u_t[ia] +
                     q.s * derivatives.u_t[ib]);
      const double flux_residual = kappa_n / beta_n_hat * residual;
      sum += q.weight * flux_residual * flux_residual;
    }
    terms.boundary[tri] += length * length * sum; // h_E ||R||^2
  }
  return terms;
}

double root_of_sum(const std::vector<double> &squares) {
  double sum = 0.0;
  for (double s : squares)
    sum += s;
  return std::sqrt(sum);
}

} // namespace

std::vector<double>
ResidualEstimator::indicators(const fem::MeshedProblem &problem,
                              const solve::Solution &solution,
                              const TimeDerivatives &derivatives,
                              mesh::StageReport &report) const {
  const fem::CompiledEquation &e = problem.equation();
  bool dynamic = false;
  for (const auto &condition : problem.conditions())
    dynamic = dynamic || condition.kind == fem::ConditionKind::dynamic;
  const bool needs_u_t = !e.m1.facts().zero || (dynamic && !a_is_zero(e));
  const bool needs_u_tt = !e.m2.facts().zero;
  if (needs_u_t && derivatives.u_t.empty())
    throw std::invalid_argument(
        name() + ": the problem has a term in u_t (m1 or a dynamic boundary "
                 "condition), so u_t is needed (zeros for a static "
                 "solution)");
  if (needs_u_tt && derivatives.u_tt.empty())
    throw std::invalid_argument(
        name() + ": the problem has a term in u_tt (m2), so u_tt is needed "
                 "(zeros for a static solution)");

  const ResidualTerms terms =
      residual_terms(problem, solution, derivatives, true, name());
  std::vector<double> eta(terms.element.size());
  for (std::size_t t = 0; t < eta.size(); ++t)
    eta[t] = std::sqrt(terms.element[t] + terms.jump[t] + terms.boundary[t]);
  report.metrics["element_part"] = root_of_sum(terms.element);
  report.metrics["jump_part"] = root_of_sum(terms.jump);
  report.metrics["boundary_part"] = root_of_sum(terms.boundary);
  return eta;
}

std::vector<double>
KellyEstimator::indicators(const fem::MeshedProblem &problem,
                           const solve::Solution &solution,
                           const TimeDerivatives & /*derivatives*/,
                           mesh::StageReport &report) const {
  const ResidualTerms terms =
      residual_terms(problem, solution, {}, false, name());
  std::vector<double> eta(terms.jump.size());
  for (std::size_t t = 0; t < eta.size(); ++t)
    eta[t] = std::sqrt(terms.jump[t] + terms.boundary[t]);
  report.metrics["jump_part"] = root_of_sum(terms.jump);
  report.metrics["boundary_part"] = root_of_sum(terms.boundary);
  return eta;
}

} // namespace pde2d::adapt

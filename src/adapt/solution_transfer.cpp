#include "pde2d/adapt/solution_transfer.hpp"

#include "point_location.hpp"

#include <chrono>
#include <cmath>
#include <format>
#include <span>
#include <stdexcept>
#include <vector>

namespace pde2d::adapt {

namespace {

constexpr const char *stage = "solution_transfer";

void check_values(std::span<const double> values, std::size_t nodes,
                  const char *what) {
  if (values.size() != nodes)
    throw std::invalid_argument(
        std::format("{}: the old solution has {} {} for {} nodes", stage,
                    values.size(), what, nodes));
  for (const double v : values)
    if (!std::isfinite(v))
      throw std::invalid_argument(std::format(
          "{}: the old solution has {} that are not finite", stage, what));
}

// Per new node, the old nodes and weights its value is made of.
using Plan = std::vector<detail::NodeWeights>;

Plan plan_from_parents(const mesh::Parents &parents, std::size_t old_nodes,
                       const mesh::Mesh &new_mesh, mesh::StageReport &report) {
  if (parents.nodes.size() != new_mesh.nodes.size() ||
      parents.triangles.size() != new_mesh.triangles.size())
    throw std::invalid_argument(std::format(
        "{}: the parents are for {} nodes and {} triangles, the new mesh "
        "has {} and {}",
        stage, parents.nodes.size(), parents.triangles.size(),
        new_mesh.nodes.size(), new_mesh.triangles.size()));
  Plan plan(new_mesh.nodes.size());
  std::size_t kept = 0, on_edges = 0;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    const auto [a, b] = parents.nodes[i];
    if (a > b || b >= old_nodes)
      throw std::invalid_argument(std::format(
          "{}: node {} has the parent {{{}, {}}}, not two old node indices "
          "a <= b below {}",
          stage, i, a, b, old_nodes));
    if (a == b) {
      plan[i] = {.nodes = {a, 0, 0}, .weights = {1.0, 0.0, 0.0}};
      ++kept;
    } else {
      plan[i] = {.nodes = {a, b, 0}, .weights = {0.5, 0.5, 0.0}};
      ++on_edges;
    }
  }
  report.metrics["kept_nodes"] = static_cast<double>(kept);
  report.metrics["edge_nodes"] = static_cast<double>(on_edges);
  report.messages.push_back(
      "by parents: old nodes keep their values, a node on an old edge gets "
      "the average of its ends");
  return plan;
}

Plan plan_by_location(const mesh::Mesh &old_mesh, const mesh::Mesh &new_mesh,
                      mesh::StageReport &report) {
  const detail::PointLocator locator(old_mesh);
  std::vector<bool> on_boundary(new_mesh.nodes.size(), false);
  for (const auto &edge : new_mesh.boundary_edges)
    for (const std::size_t v : edge.nodes)
      on_boundary.at(v) = true;
  Plan plan(new_mesh.nodes.size());
  std::size_t located = 0, nearest = 0;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    const geometry::Point p = new_mesh.nodes[i];
    if (!on_boundary[i]) {
      if (const auto weights = locator.in_triangle(p)) {
        plan[i] = *weights;
        ++located;
        continue;
      }
    }
    plan[i] = locator.nearest_on_boundary(p);
    ++nearest;
  }
  report.metrics["located"] = static_cast<double>(located);
  report.metrics["nearest_boundary"] = static_cast<double>(nearest);
  report.messages.push_back(
      "by point location (no parents): boundary nodes and nodes outside the "
      "old mesh take the value at their nearest old boundary point");
  return plan;
}

std::vector<double> apply(const Plan &plan, std::span<const double> old) {
  std::vector<double> values(plan.size());
  for (std::size_t i = 0; i < plan.size(); ++i) {
    const auto &[nodes, weights] = plan[i];
    // A single weight 1 copies the value exactly.
    if (weights[1] == 0.0 && weights[2] == 0.0 && weights[0] == 1.0)
      values[i] = old[nodes[0]];
    else
      values[i] = weights[0] * old[nodes[0]] + weights[1] * old[nodes[1]] +
                  weights[2] * old[nodes[2]];
  }
  return values;
}

} // namespace

SolutionTransfer transfer_solution(const solve::Solution &old,
                                   const fem::MeshedProblem &problem,
                                   const mesh::Parents *parents) {
  const auto start = std::chrono::steady_clock::now();
  if (!old.mesh)
    throw std::invalid_argument(
        std::format("{}: the old solution has no mesh", stage));
  const mesh::Mesh &old_mesh = *old.mesh;
  const mesh::Mesh &new_mesh = *problem.mesh();
  check_values(old.values, old_mesh.nodes.size(), "values");
  const bool velocities = !old.velocities.empty();
  if (velocities)
    check_values(old.velocities, old_mesh.nodes.size(), "velocities");
  if (!std::isfinite(old.time))
    throw std::invalid_argument(
        std::format("{}: the old solution's time is not finite", stage));

  SolutionTransfer result;
  mesh::StageReport &report = result.report;
  report.stage = stage;
  const Plan plan = parents ? plan_from_parents(*parents, old_mesh.nodes.size(),
                                                new_mesh, report)
                            : plan_by_location(old_mesh, new_mesh, report);

  solve::Solution &solution = result.solution;
  solution.mesh = problem.mesh();
  solution.time = old.time;
  solution.values = apply(plan, old.values);
  if (velocities)
    solution.velocities = apply(plan, old.velocities);
  const auto &dirichlet = problem.dirichlet_nodes();
  for (std::size_t k = 0; k < dirichlet.size(); ++k) {
    solution.values[dirichlet[k].node] = problem.dirichlet_value(k, old.time);
    if (velocities)
      solution.velocities[dirichlet[k].node] =
          problem.dirichlet_rate(k, old.time);
  }
  report.metrics["nodes"] = static_cast<double>(new_mesh.nodes.size());
  report.metrics["by_parents"] = parents ? 1.0 : 0.0;
  report.metrics["dirichlet_nodes"] = static_cast<double>(dirichlet.size());
  report.time = std::chrono::steady_clock::now() - start;
  return result;
}

} // namespace pde2d::adapt

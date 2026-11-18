#include "pde2d/adapt/solution_transfer.hpp"

#include "point_location.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <functional>
#include <limits>
#include <optional>
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

// The old boundary edges by their curve links, per chain, to find the
// edge that covers an arc-length position. A chain is used only when both
// meshes have it with the same length and number of curves (the chains a
// Remesher passes on).
class LinkIndex {
public:
  LinkIndex(const mesh::Mesh &old_mesh, const mesh::Mesh &new_mesh)
      : old_(old_mesh), by_chain_(old_mesh.chains.size()) {
    for (std::size_t e = 0; e < old_mesh.boundary_edges.size(); ++e)
      if (const auto &link = old_mesh.boundary_edges[e].link;
          link && link->chain < by_chain_.size())
        by_chain_[link->chain].push_back({link->s0, link->s1, e});
    for (std::size_t c = 0; c < by_chain_.size(); ++c) {
      const bool same =
          c < new_mesh.chains.size() &&
          old_mesh.chains[c].length() == new_mesh.chains[c].length() &&
          old_mesh.chains[c].curves().size() ==
              new_mesh.chains[c].curves().size();
      if (!same)
        by_chain_[c].clear();
      std::ranges::sort(by_chain_[c], {}, &Piece::s0);
    }
  }

  // The old boundary point at arc length s of chain `chain`: on the old
  // edge whose link covers s (the one starting at s when s is the end of
  // one and the start of the next: the same old node), weighted by arc
  // length along it; nothing when no edge covers s.
  std::optional<detail::BoundaryPoint> find(std::size_t chain, double s) const {
    if (chain >= by_chain_.size())
      return std::nullopt;
    const auto &pieces = by_chain_[chain];
    // The first piece with s0 >= s, or the one before it.
    auto it = std::ranges::lower_bound(pieces, s, {}, &Piece::s0);
    if (it == pieces.end() || it->s0 > s) {
      if (it == pieces.begin())
        return std::nullopt;
      --it;
    }
    if (!(it->s0 <= s && s <= it->s1))
      return std::nullopt;
    const auto &nodes = old_.boundary_edges[it->edge].nodes;
    detail::BoundaryPoint point{.edge = it->edge, .distance2 = 0.0};
    const double w = (s - it->s0) / (it->s1 - it->s0);
    if (s == it->s0)
      point.weights = {.nodes = {nodes[0], 0, 0}, .weights = {1.0, 0.0, 0.0}};
    else if (s == it->s1)
      point.weights = {.nodes = {nodes[1], 0, 0}, .weights = {1.0, 0.0, 0.0}};
    else
      point.weights = {.nodes = {nodes[0], nodes[1], 0},
                       .weights = {1.0 - w, w, 0.0}};
    return point;
  }

private:
  struct Piece {
    double s0, s1;
    std::size_t edge;
  };
  const mesh::Mesh &old_;
  std::vector<std::vector<Piece>> by_chain_;
};

Plan plan_by_location(const mesh::Mesh &old_mesh, const mesh::Mesh &new_mesh,
                      mesh::StageReport &report) {
  const detail::PointLocator locator(old_mesh);
  const LinkIndex links(old_mesh, new_mesh);
  constexpr auto none = std::numeric_limits<std::size_t>::max();
  // Per new node: its boundary loop (none inside), and its boundary edges
  // that start and end there.
  const std::vector<std::size_t> edge_loops = detail::boundary_loops(new_mesh);
  std::size_t new_loops = 0;
  std::vector<std::size_t> loop_of(new_mesh.nodes.size(), none);
  std::vector<std::size_t> starts(new_mesh.nodes.size(), none),
      ends(new_mesh.nodes.size(), none);
  for (std::size_t e = 0; e < new_mesh.boundary_edges.size(); ++e) {
    const auto &v = new_mesh.boundary_edges[e].nodes;
    loop_of.at(v[0]) = loop_of.at(v[1]) = edge_loops[e];
    starts[v[0]] = e;
    ends[v[1]] = e;
    new_loops = std::max(new_loops, edge_loops[e] + 1);
  }

  // 1. A boundary node with a curve link: the old edge of its chain that
  // covers its arc length (the edge starting there, else the one ending
  // there). Every boundary node also votes for the old loop of its old
  // boundary point; a point on the old boundary (through a link, or at
  // distance 0: an old node kept) is a sure vote.
  Plan plan(new_mesh.nodes.size());
  std::vector<std::optional<detail::BoundaryPoint>> nearest(plan.size());
  std::vector<bool> by_link(plan.size(), false);
  std::vector<std::vector<std::size_t>> sure(
      new_loops, std::vector<std::size_t>(locator.loops(), 0));
  std::vector<std::vector<std::size_t>> votes = sure;
  std::size_t linked = 0;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    if (loop_of[i] == none)
      continue;
    std::optional<detail::BoundaryPoint> point;
    for (const std::size_t e : {starts[i], ends[i]}) {
      if (point || e == none)
        continue;
      if (const auto &link = new_mesh.boundary_edges[e].link)
        point = links.find(link->chain, e == starts[i] ? link->s0 : link->s1);
    }
    if (point) {
      by_link[i] = true;
      plan[i] = point->weights;
      ++linked;
    } else {
      point = locator.nearest_on_boundary(new_mesh.nodes[i]);
    }
    nearest[i] = point;
    const std::size_t old_loop = locator.edge_loops()[point->edge];
    ++votes[loop_of[i]][old_loop];
    if (point->distance2 == 0.0)
      ++sure[loop_of[i]][old_loop];
  }
  // 2. Each new loop is the old loop with the most sure votes, or without
  // any, the most votes (ties: the lower old loop).
  std::vector<std::size_t> matched(new_loops, 0);
  for (std::size_t l = 0; l < new_loops; ++l) {
    const bool any_sure =
        std::ranges::any_of(sure[l], [](std::size_t n) { return n > 0; });
    const auto &count = any_sure ? sure[l] : votes[l];
    matched[l] = static_cast<std::size_t>(
        std::ranges::max_element(count, std::less<>{}) - count.begin());
  }

  // 3. The other boundary nodes: the nearest point of their matched old
  // loop; interior nodes: their old triangle, or outside the old mesh the
  // nearest old boundary point.
  std::size_t located = 0, nearest_boundary = 0, other_loop = 0;
  for (std::size_t i = 0; i < plan.size(); ++i) {
    if (by_link[i])
      continue;
    const geometry::Point p = new_mesh.nodes[i];
    if (loop_of[i] == none) {
      if (const auto weights = locator.in_triangle(p)) {
        plan[i] = *weights;
        ++located;
      } else {
        plan[i] = locator.nearest_on_boundary(p).weights;
        ++nearest_boundary;
      }
      continue;
    }
    const std::size_t loop = matched[loop_of[i]];
    if (locator.edge_loops()[nearest[i]->edge] == loop) {
      plan[i] = nearest[i]->weights;
    } else {
      plan[i] = locator.nearest_on_boundary(p, loop).weights;
      ++other_loop;
    }
    ++nearest_boundary;
  }
  report.metrics["located"] = static_cast<double>(located);
  report.metrics["by_link"] = static_cast<double>(linked);
  report.metrics["nearest_boundary"] = static_cast<double>(nearest_boundary);
  report.metrics["other_loop_nearer"] = static_cast<double>(other_loop);
  report.messages.push_back(
      "by point location (no parents): a boundary node takes the value at "
      "its old boundary point (through its curve link, else the nearest "
      "point of its own boundary loop), an interior node the old u_h at its "
      "position (outside the old mesh: at its nearest old boundary point)");
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

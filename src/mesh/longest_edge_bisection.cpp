#include "pde2d/mesh/longest_edge_bisection.hpp"

#include "pde2d/mesh/domain_size.hpp"

#include "local_improvers.hpp"

#include <algorithm>
#include <deque>
#include <stdexcept>
#include <tuple>

namespace pde2d::mesh {

namespace {

// A strict total order on edges: squared length, then node indices. It
// makes "the longest edge" unique, so propagation paths cannot cycle.
using EdgeKey = std::tuple<double, std::size_t, std::size_t>;

EdgeKey edge_key(const WorkingMesh &mesh, std::size_t t, int i) {
  const auto &v = mesh.triangles()[t];
  std::size_t a = v[static_cast<std::size_t>((i + 1) % 3)],
              b = v[static_cast<std::size_t>((i + 2) % 3)];
  geometry::Point d = mesh.nodes()[a] - mesh.nodes()[b];
  return {geometry::dot(d, d), std::min(a, b), std::max(a, b)};
}

int longest(const WorkingMesh &mesh, std::size_t t) {
  int best = 0;
  for (int i = 1; i < 3; ++i)
    if (edge_key(mesh, t, i) > edge_key(mesh, t, best))
      best = i;
  return best;
}

double longest_length(const WorkingMesh &mesh, std::size_t t) {
  const auto &v = mesh.triangles()[t];
  const auto &p = mesh.nodes();
  return std::max({geometry::distance(p[v[0]], p[v[1]]),
                   geometry::distance(p[v[1]], p[v[2]]),
                   geometry::distance(p[v[2]], p[v[0]])});
}

} // namespace

LongestEdgeBisection::LongestEdgeBisection(LongestEdgeBisectionOptions options)
    : options_(std::move(options)) {
  if (options_.max_points == 0)
    throw std::invalid_argument(
        "LongestEdgeBisectionOptions::max_points must be positive");
}

void LongestEdgeBisection::run(
    WorkingMesh &mesh, const MeshSpec &spec,
    std::span<const LocalImprover *const> local_improvers,
    StageReport &report) const {
  validate(spec);
  const double size = domain_size(mesh);
  const double h = resolve(spec.max_edge_length, size);
  report.metrics["domain_size"] = size;
  report.metrics["max_edge_length"] = h;
  std::size_t splits = 0, boundary_splits = 0, retriangulated = 0,
              deleted_nodes = 0, failed_splits = 0;
  detail::LocalImproverCalls local(local_improvers);
  std::deque<std::size_t> queue;
  // Sweeps repeat while one finds a long edge and makes progress: a local
  // improver may create long edges in triangles the refiner did not
  // touch, and a triangle whose path ends at an unsplittable boundary edge
  // is skipped.
  bool progress = true;
  while (progress) {
    progress = false;
    for (std::size_t t = 0; t < mesh.triangles().size(); ++t)
      if (!mesh.is_triangle_deleted(t) && longest_length(mesh, t) > h)
        queue.push_back(t);
    if (queue.empty())
      break;
    while (!queue.empty()) {
      std::size_t t = queue.front();
      queue.pop_front();
      if (mesh.is_triangle_deleted(t) || longest_length(mesh, t) <= h)
        continue;
      if (splits >= options_.max_points)
        throw std::runtime_error(
            "longest_edge_bisection: inserted max_points = " +
            std::to_string(options_.max_points) + " points");
      // Follow the longest-edge propagation path to its terminal edge.
      std::size_t u = t;
      std::vector<std::size_t> created;
      bool split = false;
      while (true) {
        int e = longest(mesh, u);
        std::size_t n = mesh.neighbour(u, e);
        if (n == no_index) {
          std::size_t k = mesh.boundary_edge_index(u, e);
          if (mesh.is_unsplittable(k))
            break;
          auto [a, b] = mesh.boundary_edges()[k].nodes;
          auto r = mesh.split_boundary_edge(k);
          if (!r.split) {
            ++failed_splits;
            report.messages.push_back(
                "boundary edge (" + std::to_string(a) + ", " +
                std::to_string(b) + ") of region '" +
                mesh.region_names()[mesh.boundary_edges()[k].region] +
                "' cannot be split: " + r.failure +
                "; it stays as it is, so the target may not be met near it");
            break;
          }
          created = std::move(r.touched);
          ++boundary_splits;
          retriangulated += r.retriangulated;
          deleted_nodes += r.deleted_nodes;
          split = true;
          break;
        }
        if (edge_key(mesh, n, longest(mesh, n)) == edge_key(mesh, u, e)) {
          const auto &v = mesh.triangles()[u];
          auto mid = geometry::midpoint(
              mesh.nodes()[v[static_cast<std::size_t>((e + 1) % 3)]],
              mesh.nodes()[v[static_cast<std::size_t>((e + 2) % 3)]]);
          auto c = mesh.split_interior_edge(u, e, mid);
          created.assign(c.begin(), c.end());
          split = true;
          break;
        }
        u = n;
      }
      if (!split)
        continue; // left for a later sweep, if anything else changes
      progress = true;
      ++splits;
      local.after_change(mesh, created);
      for (auto s : created)
        if (!mesh.is_triangle_deleted(s))
          queue.push_back(s);
      queue.push_back(t);
    }
  }
  report.metrics["splits"] = static_cast<double>(splits);
  report.metrics["boundary_splits"] = static_cast<double>(boundary_splits);
  report.metrics["retriangulated_splits"] = static_cast<double>(retriangulated);
  report.metrics["deleted_nodes"] = static_cast<double>(deleted_nodes);
  report.metrics["unsplittable_edges"] = static_cast<double>(failed_splits);
  local.record(report);
}

} // namespace pde2d::mesh

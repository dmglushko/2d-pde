#include "pde2d/mesh/longest_edge_bisection.hpp"

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

void LongestEdgeBisection::run(WorkingMesh &mesh, const MeshSpec &spec,
                               StageReport &report) const {
  validate(spec);
  const double h = spec.max_edge_length;
  std::size_t splits = 0, boundary_splits = 0, changes = 0;
  std::deque<std::size_t> queue;
  // Sweeps repeat until one finds no long edge: a local improver may
  // create long edges in triangles the refiner did not touch.
  while (true) {
    for (std::size_t t = 0; t < mesh.triangles().size(); ++t)
      if (longest_length(mesh, t) > h)
        queue.push_back(t);
    if (queue.empty())
      break;
    while (!queue.empty()) {
      std::size_t t = queue.front();
      queue.pop_front();
      if (longest_length(mesh, t) <= h)
        continue;
      if (splits >= options_.max_points)
        throw std::runtime_error(
            "longest_edge_bisection: inserted max_points = " +
            std::to_string(options_.max_points) + " points");
      // Follow the longest-edge propagation path to its terminal edge.
      std::size_t u = t;
      std::vector<std::size_t> created;
      while (true) {
        int e = longest(mesh, u);
        std::size_t n = mesh.neighbour(u, e);
        if (n == no_index) {
          auto c = mesh.split_boundary_edge(mesh.boundary_edge_index(u, e));
          created.assign(c.begin(), c.end());
          ++boundary_splits;
          break;
        }
        if (edge_key(mesh, n, longest(mesh, n)) == edge_key(mesh, u, e)) {
          const auto &v = mesh.triangles()[u];
          auto mid = geometry::midpoint(
              mesh.nodes()[v[static_cast<std::size_t>((e + 1) % 3)]],
              mesh.nodes()[v[static_cast<std::size_t>((e + 2) % 3)]]);
          auto c = mesh.split_interior_edge(u, e, mid);
          created.assign(c.begin(), c.end());
          break;
        }
        u = n;
      }
      ++splits;
      if (options_.local_improver)
        changes += options_.local_improver->improve_around(mesh, created);
      for (auto s : created)
        queue.push_back(s);
      queue.push_back(t);
    }
  }
  report.metrics["splits"] = static_cast<double>(splits);
  report.metrics["boundary_splits"] = static_cast<double>(boundary_splits);
  report.metrics["local_improver_changes"] = static_cast<double>(changes);
}

} // namespace pde2d::mesh

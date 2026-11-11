#include "pde2d/mesh/delaunay.hpp"

#include "delaunay_internal.hpp"

#include "pde2d/geometry/predicates.hpp"

#include <numeric>
#include <utility>
#include <vector>

namespace pde2d::mesh {

bool is_locally_delaunay(const WorkingMesh &mesh, std::size_t triangle,
                         int edge) {
  std::size_t u = mesh.neighbour(triangle, edge);
  if (u == no_index)
    return true;
  const auto &v = mesh.triangles()[triangle];
  const auto &w = mesh.triangles()[u];
  std::size_t b = v[static_cast<std::size_t>((edge + 1) % 3)];
  // The vertex of u that follows b is the one across the edge.
  std::size_t d = 0;
  for (std::size_t i = 0; i < 3; ++i)
    if (w[i] == b)
      d = w[(i + 1) % 3];
  const auto &p = mesh.nodes();
  return geometry::incircle(p[v[0]], p[v[1]], p[v[2]], p[d]) <= 0;
}

namespace detail {

std::size_t lawson(WorkingMesh &mesh, std::span<const std::size_t> seeds,
                   std::vector<std::size_t> *touched) {
  // Edges are kept as node pairs: flips reuse triangle indices, so a
  // (triangle, edge) pair may name a different edge later.
  std::vector<std::pair<std::size_t, std::size_t>> stack;
  auto push_edges = [&](std::size_t t) {
    const auto &v = mesh.triangles()[t];
    for (int i = 0; i < 3; ++i)
      if (mesh.neighbour(t, i) != no_index)
        stack.emplace_back(v[static_cast<std::size_t>((i + 1) % 3)],
                           v[static_cast<std::size_t>((i + 2) % 3)]);
  };
  for (auto it = seeds.rbegin(); it != seeds.rend(); ++it)
    push_edges(*it);
  if (touched)
    touched->insert(touched->end(), seeds.begin(), seeds.end());
  std::size_t flips = 0;
  while (!stack.empty()) {
    auto [a, b] = stack.back();
    stack.pop_back();
    auto [t, edge] = mesh.find_edge(a, b);
    if (t == no_index || is_locally_delaunay(mesh, t, edge))
      continue;
    // A non-Delaunay edge always has a strictly convex quadrilateral.
    auto pair = mesh.flip(t, edge);
    ++flips;
    for (auto s : pair) {
      push_edges(s);
      if (touched)
        touched->push_back(s);
    }
  }
  return flips;
}

} // namespace detail

std::size_t flip_to_delaunay(WorkingMesh &mesh) {
  std::vector<std::size_t> all(mesh.triangles().size());
  std::iota(all.begin(), all.end(), std::size_t{0});
  return detail::lawson(mesh, all, nullptr);
}

std::size_t flip_to_delaunay_around(WorkingMesh &mesh,
                                    std::span<const std::size_t> triangles) {
  return detail::lawson(mesh, triangles, nullptr);
}

} // namespace pde2d::mesh

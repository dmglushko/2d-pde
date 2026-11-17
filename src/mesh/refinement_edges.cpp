#include "refinement_edges.hpp"

#include <algorithm>
#include <tuple>

namespace pde2d::mesh::detail {

int longest_local_edge(const std::array<std::size_t, 3> &v,
                       const std::vector<geometry::Point> &nodes) {
  auto key = [&](std::size_t i) {
    const std::size_t a = v[(i + 1) % 3], b = v[(i + 2) % 3];
    const geometry::Point d = nodes[a] - nodes[b];
    return std::tuple{geometry::dot(d, d), std::min(a, b), std::max(a, b)};
  };
  std::size_t best = 0;
  for (std::size_t i = 1; i < 3; ++i)
    if (key(i) > key(best))
      best = i;
  return static_cast<int>(best);
}

std::size_t set_longest_edges_as_refinement_edges(WorkingMesh &mesh,
                                                  RefinementEdgeScope scope) {
  std::size_t rotated = 0;
  for (std::size_t t = 0; t < mesh.triangles().size(); ++t) {
    if (mesh.is_triangle_deleted(t))
      continue;
    if (scope == RefinementEdgeScope::edited_order &&
        RefinementEdgeRecord::order_is_as_set(mesh, t))
      continue;
    const int longest = longest_local_edge(mesh.triangles()[t], mesh.nodes());
    if (longest != 0)
      ++rotated;
    // Also for longest == 0: the order is now set.
    mesh.set_refinement_edge(t, longest);
  }
  return rotated;
}

bool RefinementEdgeRecord::order_is_as_set(const WorkingMesh &mesh,
                                           std::size_t t) {
  return t < mesh.set_orders_.size() &&
         mesh.set_orders_[t] == mesh.triangles_[t];
}

} // namespace pde2d::mesh::detail

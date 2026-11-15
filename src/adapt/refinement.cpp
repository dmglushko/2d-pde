// The two refinement methods (both thin: the work is in
// edge_refinement.cpp) and the initial refinement edges.

#include "edge_refinement.hpp"

#include "pde2d/adapt/newest_vertex_bisection.hpp"
#include "pde2d/adapt/red_green_blue_refinement.hpp"
#include "pde2d/adapt/refinement.hpp"

#include <stdexcept>
#include <string>

namespace pde2d::adapt {

RefinementResult
NewestVertexBisection::refine(const mesh::Mesh &mesh,
                              std::span<const std::size_t> marked) const {
  return detail::refine_marked_edges(mesh, marked, name(),
                                     detail::FullRefinement::bisections);
}

RefinementResult
RedGreenBlueRefinement::refine(const mesh::Mesh &mesh,
                               std::span<const std::size_t> marked) const {
  return detail::refine_marked_edges(mesh, marked, name(),
                                     detail::FullRefinement::red);
}

void set_longest_edges_as_refinement_edges(mesh::Mesh &mesh) {
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    auto &v = mesh.triangles[t].nodes;
    for (auto i : v)
      if (i >= mesh.nodes.size())
        throw std::invalid_argument(
            "set_longest_edges_as_refinement_edges: triangle " +
            std::to_string(t) + " uses a node index out of range");
    // The squared length of the edge opposite vertex i.
    auto opposite_length = [&](std::size_t i) {
      geometry::Point d =
          mesh.nodes[v[(i + 1) % 3]] - mesh.nodes[v[(i + 2) % 3]];
      return geometry::dot(d, d);
    };
    std::size_t first = 0;
    for (std::size_t i = 1; i < 3; ++i) {
      const double length = opposite_length(i), best = opposite_length(first);
      if (length > best || (length == best && v[i] < v[first]))
        first = i;
    }
    // A rotation keeps the triangle counterclockwise.
    v = {v[first], v[(first + 1) % 3], v[(first + 2) % 3]};
  }
}

} // namespace pde2d::adapt

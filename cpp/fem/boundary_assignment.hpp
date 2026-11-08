#pragma once
// Validated binding of mesh boundary regions to the conditions of a Problem,
// with the geometric frame (length, outward normal) of every boundary edge.
//
//   BoundaryBinding b = bind_boundary(mesh, problem);
//   for (const auto &edge : b.edges)  // mesh.boundary_edges order
//     classify_edge_condition(problem.equation, *edge.condition, edge.frame);
//
// Assembly (PreparedAssembly) and analysis (analyze(problem, mesh)) both start
// from this one binding, so they accept and reject the same assignments and
// see the same outward normals. The binding points into `problem`: keep the
// Problem alive (and unchanged) while it is used.
//
// Rules, with the exception messages tests match: every boundary edge has a
// nonempty region tag; every assigned region exists in the mesh ("Absent
// boundary region"); no region is assigned twice ("Overlapping boundary
// assignments"); every tagged region has a condition ("Missing boundary
// assignment").
#include "discretization/geometry_base.hpp"
#include "fem/p1_element.hpp"
#include "fem/problem.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
// One boundary edge with its region and condition.
struct BoundaryEdgeBinding {
  std::size_t edge;                 // index into mesh.boundary_edges
  std::array<std::size_t, 2> nodes; // endpoints, in mesh.boundary_edges order
  std::size_t region;               // index into BoundaryBinding::regions
  const BoundaryCondition *condition;
  p1::EdgeFrame frame; // from nodes[0] to nodes[1], outward normal
};
struct BoundaryBinding {
  // Assigned regions in the order of Problem::boundaries and their regions.
  std::vector<std::string> regions;
  // One entry per boundary edge, in mesh order.
  std::vector<BoundaryEdgeBinding> edges;
};

namespace boundary_detail {
// Interior vertex opposite each element edge, keyed by sorted endpoints.
using OppositeVertices =
    std::map<std::pair<std::size_t, std::size_t>, std::size_t>;
inline OppositeVertices
opposite_vertices(const discretization::TriangleMesh &mesh) {
  OppositeVertices opposite;
  for (const auto &element : mesh.elements)
    for (std::size_t k = 0; k < 3; ++k)
      opposite[std::minmax(element[k], element[(k + 1) % 3])] =
          element[(k + 2) % 3];
  return opposite;
}
} // namespace boundary_detail

// Bind every boundary edge of a geometrically valid mesh (see
// discretization::validate_mesh_geometry) to its region's condition.
inline BoundaryBinding bind_boundary(const discretization::TriangleMesh &mesh,
                                     const Problem &problem) {
  if (mesh.boundary_tags.size() != mesh.boundary_edges.size())
    throw std::invalid_argument(
        "Every boundary edge needs an explicit region tag and condition");
  std::set<std::string> present(mesh.boundary_tags.begin(),
                                mesh.boundary_tags.end());
  if (present.contains(""))
    throw std::invalid_argument("Empty boundary region tag");
  BoundaryBinding binding;
  std::map<std::string, std::pair<std::size_t, const BoundaryCondition *>>
      assigned;
  for (const auto &assignment : problem.boundaries) {
    if (assignment.regions.empty())
      throw std::invalid_argument("Empty boundary assignment");
    for (const auto &region : assignment.regions) {
      if (!present.contains(region))
        throw std::invalid_argument("Absent boundary region: " + region);
      if (!assigned
               .emplace(region, std::pair{binding.regions.size(),
                                          &assignment.condition})
               .second)
        throw std::invalid_argument("Overlapping boundary assignments: " +
                                    region);
      binding.regions.push_back(region);
    }
  }
  const auto opposite = boundary_detail::opposite_vertices(mesh);
  binding.edges.reserve(mesh.boundary_edges.size());
  for (std::size_t edge = 0; edge < mesh.boundary_edges.size(); ++edge) {
    const auto &tag = mesh.boundary_tags[edge];
    const auto found = assigned.find(tag);
    if (found == assigned.end())
      throw std::invalid_argument("Missing boundary assignment: " + tag);
    const std::array<std::size_t, 2> nodes{mesh.boundary_edges[edge].first,
                                           mesh.boundary_edges[edge].second};
    binding.edges.push_back(
        {edge, nodes, found->second.first, found->second.second,
         p1::edge_frame(
             mesh.nodes[nodes[0]], mesh.nodes[nodes[1]],
             mesh.nodes[opposite.at(std::minmax(nodes[0], nodes[1]))])});
  }
  return binding;
}
} // namespace pde2d::fem

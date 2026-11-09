#pragma once
// Deterministic fingerprint of a mesh: node coordinates (bit patterns),
// elements, boundary edges and boundary tags, in storage order. Two meshes
// have the same fingerprint when they are identical arrays, on every
// platform; renumbering nodes or moving one by one ulp changes it. Run
// records (cpp/fem/record.hpp) use it to identify the mesh of a solve.
#include "geometry_base.hpp"
#include "provenance.hpp"
#include <string>

namespace pde2d::discretization {
inline std::string mesh_fingerprint(const TriangleMesh &mesh) {
  pde2d::Fingerprint f;
  f.add("pde2d.mesh.v1").add(mesh.nodes.size());
  for (const auto &node : mesh.nodes)
    f.add(node.x).add(node.y);
  f.add(mesh.elements.size());
  for (const auto &element : mesh.elements)
    f.add(element[0]).add(element[1]).add(element[2]);
  f.add(mesh.boundary_edges.size());
  for (const auto &[a, b] : mesh.boundary_edges)
    f.add(a).add(b);
  f.add(mesh.boundary_tags.size());
  for (const auto &tag : mesh.boundary_tags)
    f.add(tag);
  return f.hex();
}
} // namespace pde2d::discretization

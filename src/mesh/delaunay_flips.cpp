#include "pde2d/mesh/delaunay_flips.hpp"

#include "pde2d/mesh/delaunay.hpp"

namespace pde2d::mesh {

void DelaunayFlips::run(WorkingMesh &mesh, const MeshSpec &,
                        StageReport &report) const {
  report.metrics["flips"] = static_cast<double>(flip_to_delaunay(mesh));
}

std::size_t DelaunayFlips::improve_around(
    WorkingMesh &mesh, std::span<const std::size_t> touched_triangles) const {
  return flip_to_delaunay_around(mesh, touched_triangles);
}

} // namespace pde2d::mesh

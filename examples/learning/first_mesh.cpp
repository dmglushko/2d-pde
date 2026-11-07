#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "discretization/mesh_io.hpp"

#include <iostream>
#include <string>

int main(int argc, char *argv[]) {
  const std::string path = argc > 1 ? argv[1] : "rectangle.pde2d";
  discretization::RectangleGeometry geometry(2.0, 1.0);
  discretization::MeshOptions options;
  options.boundary.max_edge_length = 0.4; // Fine enough for a fixed boundary.
  options.refinement.max_edge_length = 0.5;
  const auto result = discretization::generate_mesh(geometry, options);
  if (!result.report.size_satisfied) {
    std::cerr << result.report.message << '\n';
    return 1;
  }
  discretization::export_mesh(result, path); // v3 retains refinement context.
  const auto restored = discretization::import_mesh_result(path);
  std::cout << "nodes=" << restored.mesh.nodes.size()
            << " triangles=" << restored.mesh.elements.size()
            << " boundary_edges=" << restored.mesh.boundary_edges.size()
            << '\n';
}

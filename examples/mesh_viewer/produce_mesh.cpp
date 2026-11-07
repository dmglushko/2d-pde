#include "../../cpp/discretization/geometry.hpp"
#include "../../cpp/discretization/triangulate.hpp"
#include "../../cpp/discretization/mesh_io.hpp"

#include <exception>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: produce_mesh OUTPUT.pde2d\n";
    return 2;
  }
  try {
    discretization::RectangleGeometry geometry(3.0, 2.0);
    geometry.setRotation(0.12);
    discretization::MeshOptions options;
    options.boundary.max_edge_length = 0.35;
    options.refinement.max_edge_length = 0.42;
    auto result = discretization::generate_mesh(geometry, options);
    if (!result.report.size_satisfied) {
      std::cerr << result.report.message << '\n';
      return 1;
    }
    discretization::export_mesh(result, argv[1]);
    auto restored = discretization::import_mesh_result(argv[1]);
    std::cout << "Saved P1 mesh: " << restored.mesh.nodes.size() << " nodes, "
              << restored.mesh.elements.size() << " triangles, "
              << restored.mesh.boundary_edges.size() << " boundary edges\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

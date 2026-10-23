#include "../../cpp/discretization/geometry.hpp"
#include "../../cpp/discretization/mesh_generators.hpp"
#include "../../cpp/discretization/mesh_io.hpp"

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: produce_mesh OUTPUT.pde2d\n";
    return 2;
  }
  try {
    discretization::RectangleGeometry geometry(3.0, 2.0);
    geometry.setRotation(0.12);
    discretization::DelaunayEdgeFlipMeshGenerator generator;
    generator.setMaxElementSize(0.42);
    auto mesh = generator.createMesh(geometry);
    discretization::export_mesh(mesh, argv[1]);
    auto restored = discretization::import_mesh(argv[1]);
    std::cout << "Saved P1 mesh: " << restored.nodes.size()
              << " nodes, " << restored.elements.size() << " triangles, "
              << restored.boundary_edges.size() << " boundary edges\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

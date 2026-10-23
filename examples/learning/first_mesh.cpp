#include "discretization/geometry.hpp"
#include "discretization/mesh_generators.hpp"
#include "discretization/mesh_io.hpp"

#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    const std::string path = argc > 1 ? argv[1] : "rectangle.pde2d";

    discretization::RectangleGeometry geometry(2.0, 1.0);
    discretization::UniformRefinementMeshGenerator mesher;
    mesher.setMaxElementSize(0.5);
    mesher.setOrder(1);

    const discretization::TriangleMesh mesh = mesher.createMesh(geometry);
    discretization::export_mesh(mesh, path);
    const discretization::TriangleMesh restored = discretization::import_mesh(path);

    std::cout << "order=" << restored.order
              << " nodes=" << restored.nodes.size()
              << " triangles=" << restored.elements.size()
              << " boundary_edges=" << restored.boundary_edges.size() << '\n';
}

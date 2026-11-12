// Meshes a plate with a circular hole and writes it in the .pde2d format,
// which the mesh viewer (tools/mesh_viewer/index.html) opens.
//
// Usage: produce_mesh OUTPUT.pde2d [OUTPUT.svg]
// The optional second file is an SVG picture of the same mesh.

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/io/pde2d_mesh.hpp"
#include "pde2d/io/svg.hpp"
#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/mesher.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <exception>
#include <iostream>
#include <memory>
#include <numbers>

using namespace pde2d;

int main(int argc, char **argv) {
  if (argc != 2 && argc != 3) {
    std::cerr << "Usage: produce_mesh OUTPUT.pde2d [OUTPUT.svg]\n";
    return 2;
  }
  try {
    // A 3 x 2 plate ("wall" below and above, "inlet" left, "outlet" right)
    // with a hole of radius 0.4. The hole is found from nesting.
    using geometry::Line;
    geometry::Loop plate{{{Line{{0, 0}, {3, 0}}, "wall"},
                          {Line{{3, 0}, {3, 2}}, "outlet"},
                          {Line{{3, 2}, {0, 2}}, "wall"},
                          {Line{{0, 2}, {0, 0}}, "inlet"}}};
    geometry::Loop hole{
        {{geometry::EllipticalArc{{1, 1}, 0.4, 0.4, 0, 0, 2 * std::numbers::pi},
          "hole"}}};
    geometry::CurveGeometry geometry({plate, hole});

    mesh::Mesher mesher(std::make_shared<mesh::ChordLengthDiscretizer>(),
                        std::make_shared<mesh::Cdt>(),
                        std::make_shared<mesh::Ruppert>(
                            mesh::RuppertOptions{.min_angle_degrees = 25}));
    mesh::MeshResult result =
        mesher.run(geometry, {.max_edge_length = {.absolute = 0.2}});
    if (!result.target_met)
      std::cerr << "warning: the longest edge is " << result.longest_edge
                << ", above the target\n";

    io::export_pde2d(result.mesh, argv[1]);
    if (argc == 3)
      io::export_mesh_svg(result.mesh, argv[2]);
    mesh::Mesh restored = io::import_pde2d(argv[1]);
    std::cout << "Saved P1 mesh: " << restored.nodes.size() << " nodes, "
              << restored.triangles.size() << " triangles, "
              << restored.boundary_edges.size() << " boundary edges\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}

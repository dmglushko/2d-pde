#pragma once

#include "pde2d/mesh/mesh.hpp"

#include <cstddef>

namespace pde2d::mesh {

// The warning angle of the Mesher and Remesher, in degrees: the pipeline
// report warns about triangles with a smaller angle.
inline constexpr double default_warning_angle_degrees = 10.0;

// What a mesh is like, measured on the mesh itself (the way facts describe
// a problem). The Mesher and Remesher measure their final mesh once, after
// the last improver; target_met and the pipeline report's angle warning
// are read from this one measurement.
struct MeshProperties {
  std::size_t nodes = 0;
  std::size_t triangles = 0;
  // Over all angles of all triangles, in degrees.
  double smallest_angle_degrees = 0.0;
  double largest_angle_degrees = 0.0;
  // Over all edges of all triangles.
  double shortest_edge = 0.0;
  double longest_edge = 0.0;
  // The angle the count below was measured against, and how many
  // triangles have an angle strictly below it.
  double warning_angle_degrees = 0.0;
  std::size_t triangles_below_warning_angle = 0;
};

// Measures the mesh. It does not validate the mesh (see validate), but
// throws std::invalid_argument for a mesh without triangles, a node index
// out of range, or a warning angle outside [0, 60] (0 counts nothing; no
// triangle has all its angles above 60 degrees, so a larger angle would
// count every triangle).
MeshProperties measure(const Mesh &mesh, double warning_angle_degrees);

// Throws std::invalid_argument, naming `what`, unless the angle is in
// [0, 60]. The check measure() and the Mesher and Remesher options use.
void validate_warning_angle(double warning_angle_degrees, const char *what);

} // namespace pde2d::mesh

#pragma once

namespace pde2d::mesh {

// What the mesher is asked to produce. It is the mesher's only external
// input besides the geometry, and a struct so it can grow (for example a
// metric field for anisotropic meshes later).
struct MeshSpec {
  // Every edge of the final mesh should be at most this long. Must be
  // finite and positive.
  double max_edge_length = 0.0;
};

// Throws std::invalid_argument unless max_edge_length is finite and
// positive.
void validate(const MeshSpec &spec);

} // namespace pde2d::mesh

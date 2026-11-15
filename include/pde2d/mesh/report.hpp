#pragma once

#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/mesh_properties.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace pde2d::mesh {

// What one stage did. Failures are exceptions, never reports: a message
// here means "worked, but ...".
struct StageReport {
  std::string stage; // the component's name()
  std::chrono::duration<double> time{};
  std::vector<std::string> messages;
  std::map<std::string, double> metrics;
};

// What a refined mesh came from: the seam for transferring a solution to
// it. Indices on the left are those of the refined mesh, on the right
// those of the mesh that was refined.
struct Parents {
  // For each triangle, the input triangle it lies in. A child of a
  // triangle with a curved boundary edge may stick out of its parent (its
  // new boundary node is on the curve, not on the chord); it still names
  // that parent.
  std::vector<std::size_t> triangles;
  // For each node: {v, v} for input node v (which keeps its index), {a, b}
  // with a < b for a new node on the input edge a-b. A new node is the
  // midpoint of a-b, except on a boundary edge with a curve link, where it
  // is the curve point at the middle of the link's arc-length interval. So
  // (u[a] + u[b]) / 2 interpolates a P1 function u to every node.
  std::vector<std::array<std::size_t, 2>> nodes;
};

struct MeshResult {
  Mesh mesh;
  std::vector<StageReport> reports; // in stage order
  // The final mesh, measured once after the last improver (see
  // mesh_properties.hpp). properties.longest_edge is the longest edge.
  MeshProperties properties;
  // properties.longest_edge <= max_edge_length below. Check it: a mesh can
  // be returned that misses its target.
  bool target_met = false;
  // What MeshSpec::max_edge_length resolved to, and the domain size
  // (domain_size.hpp) it was resolved against.
  double max_edge_length = 0.0;
  double domain_size = 0.0;
};

// What Remesher::refine returns.
struct RefinementResult {
  Mesh mesh;
  std::vector<StageReport> reports; // in stage order
  // The final mesh, measured once after the last improver.
  MeshProperties properties;
  // Present only when no component made them untrue: the refiner kept
  // them (its capabilities, and its own edits in this run) and every
  // improver that ran keeps them (ImproverEffects::keeps_parents). When
  // absent, the pipeline report says which component dropped them; a
  // solution is then transferred by point location.
  std::optional<Parents> parents;
};

} // namespace pde2d::mesh

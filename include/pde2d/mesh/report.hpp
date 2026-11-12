#pragma once

#include "pde2d/mesh/mesh.hpp"

#include <chrono>
#include <map>
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

struct MeshResult {
  Mesh mesh;
  std::vector<StageReport> reports; // in stage order
  // Every edge of `mesh` is at most max_edge_length below. Checked on the
  // final mesh, after the improvers. Check it: a mesh can be returned that
  // misses its target.
  bool target_met = false;
  double longest_edge = 0.0;
  // What MeshSpec::max_edge_length resolved to, and the domain size
  // (domain_size.hpp) it was resolved against.
  double max_edge_length = 0.0;
  double domain_size = 0.0;
};

} // namespace pde2d::mesh

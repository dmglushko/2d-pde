#pragma once

#include <optional>
#include <string_view>

namespace pde2d::mesh {

// An upper bound on a length, given as an absolute length, as a multiple
// of the domain size (see domain_size.hpp), or both; with both, the
// smaller one applies. An empty value is no bound, so a LengthBound with
// neither value is no bound at all. These are settings only: they do not
// know the domain, and the component that uses one resolves it.
struct LengthBound {
  std::optional<double> absolute{}; // in the geometry's length unit
  std::optional<double> relative{}; // times the domain size
};

// The smaller of absolute and relative * domain_size among the values that
// are set; infinity when neither is.
double resolve(const LengthBound &bound, double domain_size);

// Throws std::invalid_argument, naming `name`, unless every value that is
// set is finite and positive.
void validate(const LengthBound &bound, std::string_view name);

// What the mesher is asked to produce. It is the mesher's only external
// input besides the geometry, and a struct so it can grow (for example a
// metric field for anisotropic meshes later).
struct MeshSpec {
  // Every edge of the final mesh should be at most this long. At least one
  // of its values must be set.
  LengthBound max_edge_length{};
};

// Throws std::invalid_argument unless max_edge_length has a value set and
// every set value is finite and positive.
void validate(const MeshSpec &spec);

} // namespace pde2d::mesh

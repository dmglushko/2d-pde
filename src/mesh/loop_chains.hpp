#pragma once

// Private to the mesh library.

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/curve_chain.hpp"

#include <cstddef>
#include <vector>

namespace pde2d::mesh::detail {

// One chain of a loop: `count` consecutive curves starting at curve
// `first` (cyclically), all of one region.
struct LoopChain {
  std::size_t first = 0;
  std::size_t count = 0;
  CurveChain chain;
};

// Joins consecutive curves of a loop into chains. A junction is a break
// where the region changes or the boundary turns by more than
// corner_angle_degrees (in [0, 180]). Chains start after each break, in
// curve order; a loop without any break is one chain from the start of
// curve 0 (a closed chain). The ChordLengthDiscretizer and the Remesher
// (when it links a mesh to a geometry) both use this, so they build the
// same chains.
std::vector<LoopChain> loop_chains(const geometry::Loop &loop,
                                   double corner_angle_degrees);

// Throws std::invalid_argument "<what> must be in [0, 180]" unless the
// angle is in [0, 180].
void validate_corner_angle(double corner_angle_degrees, const char *what);

} // namespace pde2d::mesh::detail

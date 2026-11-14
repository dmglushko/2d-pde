#pragma once

#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/mesh.hpp"

#include <memory>
#include <span>
#include <vector>

namespace pde2d::solve {

// A solution on a mesh at one time: a value for every mesh node.
struct Solution {
  std::shared_ptr<const mesh::Mesh> mesh{};
  double time = 0.0;
  // u, one per mesh node (indexed like Mesh::nodes).
  std::vector<double> values{};
  // u_t, one per mesh node, only for a solution of a problem of time order
  // 2 (the Integrator fills it, so that a run can be continued); otherwise
  // empty.
  std::vector<double> velocities{};
};

// Turns the values of the free unknowns (MeshedProblem::free_nodes()) into
// a Solution at time t: free node free_nodes()[i] gets free_values[i], and
// Dirichlet node dirichlet_nodes()[k] gets dirichlet_value(k, t). The
// velocities stay empty. Throws std::invalid_argument when the size of
// free_values is not the number of free nodes, for a t that is not
// finite, and when a Dirichlet value is not finite.
Solution build_solution(const fem::MeshedProblem &meshed_problem,
                        std::span<const double> free_values, double t);

} // namespace pde2d::solve

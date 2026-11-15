#include "pde2d/solve/solution.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace pde2d::solve {

Solution build_solution(const fem::MeshedProblem &meshed_problem,
                        std::span<const double> free_values, double t) {
  const auto &free_nodes = meshed_problem.free_nodes();
  if (free_values.size() != free_nodes.size())
    throw std::invalid_argument(
        "build_solution: " + std::to_string(free_values.size()) +
        " free values for " + std::to_string(free_nodes.size()) +
        " free nodes");
  if (!std::isfinite(t))
    throw std::invalid_argument("build_solution: t is not finite");

  Solution solution;
  solution.mesh = meshed_problem.mesh();
  solution.time = t;
  solution.values.assign(solution.mesh->nodes.size(), 0.0);
  for (std::size_t i = 0; i < free_nodes.size(); ++i)
    solution.values[free_nodes[i]] = free_values[i];
  // dirichlet_value throws when the value is not finite.
  const auto &dirichlet_nodes = meshed_problem.dirichlet_nodes();
  for (std::size_t k = 0; k < dirichlet_nodes.size(); ++k)
    solution.values[dirichlet_nodes[k].node] =
        meshed_problem.dirichlet_value(k, t);
  return solution;
}

} // namespace pde2d::solve

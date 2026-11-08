#pragma once
// Stationary P1 solve. The element-independent linear algebra (Dirichlet
// lifting, backends, PreparedLinearSolver) lives in linear_solver.hpp, which
// this header includes for compatibility.
#include "fem/assembly.hpp"
#include "fem/linear_solver.hpp"
#include <stdexcept>

namespace pde2d::fem {
inline LinearSolution solve_stationary(const discretization::TriangleMesh &mesh,
                                       const Problem &problem,
                                       const AssemblyOptions &assembly = {},
                                       const SolverOptions &solver = {}) {
  auto s = assemble(mesh, problem, 0, assembly);
  auto [a, b] = constrain(s.stiffness, s.load, s.dirichlet);
  auto solution = solve_linear(a, b, solver);
  if (!solution.report.converged)
    throw std::runtime_error("Stationary solve: " + solution.report.message);
  return solution;
}
} // namespace pde2d::fem

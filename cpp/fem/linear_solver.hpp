#pragma once
// Linear solves for assembled systems, independent of the element type.
// Compatibility facade: includes every part of the linear solver layer.
//
//   linear_solver_types.hpp     SolverOptions, LinearSolver, Preconditioner,
//                               reports, LinearSolverBackend (+ factory),
//                               report names, solver_decisions
//   constraints.hpp             constrain(A, b, dirichlet): Dirichlet lifting
//   prepared_linear_solver.hpp  PreparedLinearSolver, solve_linear
//   detail/eigen_backends.hpp   private built-in Eigen backends
//
// Headers that only need the contracts (plans, run records) include
// linear_solver_types.hpp.
#include "fem/constraints.hpp"
#include "fem/linear_solver_types.hpp"
#include "fem/prepared_linear_solver.hpp"

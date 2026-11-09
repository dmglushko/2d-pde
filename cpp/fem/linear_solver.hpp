#pragma once
// Linear solves for assembled systems, independent of the element type.
// Umbrella header: includes every part of the linear solver layer (the one
// to include for solve_linear and PreparedLinearSolver).
//
//   linear_solver_types.hpp     SolverOptions, LinearSolver, Preconditioner,
//                               reports, LinearSolverBackend (+ factory),
//                               report names
//   constraints.hpp             constrain(A, b, dirichlet): Dirichlet lifting
//   prepared_linear_solver.hpp  PreparedLinearSolver, solve_linear
//   detail/eigen_backends.hpp   private built-in Eigen backends
//
// Headers that only need the contracts include linear_solver_types.hpp.
#include "fem/constraints.hpp"
#include "fem/linear_solver_types.hpp"
#include "fem/prepared_linear_solver.hpp"

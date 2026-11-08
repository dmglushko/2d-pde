#pragma once
// Element-independent linear-algebra types shared by assembly, the linear
// solvers and time integration. Nothing here knows about P1 elements, so a
// different assembler (see EXTENDING.md) can produce the same AssembledSystem
// and reuse solver.hpp, time_integration.hpp and evolution.hpp unchanged.
#include <Eigen/Sparse>
#include <vector>

namespace pde2d::fem {
using SparseMatrix = Eigen::SparseMatrix<double>;
using Vector = Eigen::VectorXd;
struct DirichletValue {
  Eigen::Index node;
  double value;
};
// stiffness*u + first_mass*u_t + second_mass*u_tt = load, plus Dirichlet
// values (already reduced at shared corners) that constrain() imposes.
struct AssembledSystem {
  SparseMatrix stiffness, first_mass, second_mass;
  Vector load;
  std::vector<DirichletValue> dirichlet;
  double time;
};
} // namespace pde2d::fem

#pragma once
// Element-independent linear-algebra types shared by assembly, the linear
// solvers and time integration. Nothing here knows about P1 elements, so a
// different assembler (see EXTENDING.md) can produce the same AssembledSystem
// and reuse solver.hpp, time_integration.hpp and evolution.hpp unchanged.
#include <Eigen/Sparse>
#include <stdexcept>
#include <vector>

namespace pde2d::fem {
using SparseMatrix = Eigen::SparseMatrix<double>;
using Vector = Eigen::VectorXd;
// Which Dirichlet data an assembly produces: the prescribed values g, or
// their time derivatives g_t, g_tt (used to start second-order schemes).
enum class DirichletDerivative { Value, FirstDerivative, SecondDerivative };
// Compatibility with the former integer orders 0, 1 and 2.
inline DirichletDerivative dirichlet_derivative(int order) {
  if (order < 0 || order > 2)
    throw std::invalid_argument("Dirichlet derivative order must be in [0,2]");
  return static_cast<DirichletDerivative>(order);
}
inline int dirichlet_derivative_order(DirichletDerivative derivative) {
  switch (derivative) {
  case DirichletDerivative::Value:
    return 0;
  case DirichletDerivative::FirstDerivative:
    return 1;
  case DirichletDerivative::SecondDerivative:
    return 2;
  }
  throw std::invalid_argument("Dirichlet derivative order must be in [0,2]");
}

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

// Highest time derivative with a nonzero mass matrix: 2 (second_mass), 1
// (first_mass) or 0 (stationary). The time integrators' check() and
// analyze(const AssembledSystem &) use this one judgement.
inline int time_order(const AssembledSystem &system) {
  if (system.second_mass.norm() != 0)
    return 2;
  return system.first_mass.norm() != 0 ? 1 : 0;
}

// Relative skew-symmetric part at most `relative_tolerance`:
// ||A - A^T||_F <= relative_tolerance ||A||_F. The ConjugateGradient backend
// and analyze(const AssembledSystem &) share this judgement.
inline constexpr double symmetry_tolerance = 1e-12;
inline bool
numerically_symmetric(const SparseMatrix &matrix,
                      double relative_tolerance = symmetry_tolerance) {
  const SparseMatrix skew = matrix - SparseMatrix(matrix.transpose());
  return !(skew.norm() > relative_tolerance * matrix.norm());
}
} // namespace pde2d::fem

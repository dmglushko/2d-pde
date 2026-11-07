#pragma once

#include "expression.hpp"
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {

// m2*u_tt + m1*u_t + a_xx*u_xx + a_xy*u_xy + a_yy*u_yy
//     + b_x*u_x + b_y*u_y + c*u = source.
// The mixed derivative has no implicit factor of two.
struct Equation {
  Expr a_xx = -1, a_xy = 0, a_yy = -1;
  Expr b_x = 0, b_y = 0, c = 0, source = 0;
  Expr m1 = 0, m2 = 0;
};

// alpha*u + gamma*u_t + beta_flux*q_n + beta_normal*du_dn
//       + beta_x*u_x + beta_y*u_y = value.
// q_n = -n^T A grad(u), A = [[a_xx,a_xy/2],[a_xy/2,a_yy]].
// Thus q_n = du_dn for the default -Laplacian equation.
// nx,ny are the OUTWARD normal to the straight mesh edge, not the source curve.
struct BoundaryCondition {
  Expr alpha = 0, gamma = 0;
  Expr beta_flux = 0, beta_normal = 0, beta_x = 0, beta_y = 0;
  Expr value = 0;
};

inline BoundaryCondition dirichlet(Expr value) {
  BoundaryCondition bc;
  bc.alpha = 1;
  bc.value = std::move(value);
  return bc;
}
inline BoundaryCondition neumann(Expr value) {
  BoundaryCondition bc;
  bc.beta_normal = 1;
  bc.value = std::move(value);
  return bc;
}
inline BoundaryCondition robin(Expr alpha, Expr beta, Expr value) {
  BoundaryCondition bc;
  bc.alpha = std::move(alpha);
  bc.beta_normal = std::move(beta);
  bc.value = std::move(value);
  return bc;
}
inline BoundaryCondition dynamic_robin(Expr gamma, Expr alpha, Expr beta,
                                       Expr value) {
  auto bc = robin(std::move(alpha), std::move(beta), std::move(value));
  bc.gamma = std::move(gamma);
  return bc;
}
inline BoundaryCondition flux_neumann(Expr value) {
  BoundaryCondition bc;
  bc.beta_flux = 1;
  bc.value = std::move(value);
  return bc;
}
inline BoundaryCondition flux_robin(Expr alpha, Expr beta, Expr value) {
  BoundaryCondition bc;
  bc.alpha = std::move(alpha);
  bc.beta_flux = std::move(beta);
  bc.value = std::move(value);
  return bc;
}
inline BoundaryCondition dynamic_flux_robin(Expr gamma, Expr alpha, Expr beta,
                                            Expr value) {
  auto bc = flux_robin(std::move(alpha), std::move(beta), std::move(value));
  bc.gamma = std::move(gamma);
  return bc;
}

// The spatial problem: one equation and one condition per boundary region.
// Initial conditions are not part of it; transient problems pass them to
// evolve() as InitialConditions (evolution.hpp).
struct BoundaryAssignment {
  std::vector<std::string> regions;
  BoundaryCondition condition;
};
struct Problem {
  Equation equation;
  std::vector<BoundaryAssignment> boundaries;
  void assign(BoundaryCondition condition,
              std::initializer_list<std::string> regions) {
    boundaries.push_back({regions, std::move(condition)});
  }
};

} // namespace pde2d::fem

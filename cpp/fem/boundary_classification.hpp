#pragma once
// Symbolic reduction of a general BoundaryCondition on one straight boundary
// edge to the two forms the P1 assembler supports:
//
//   Dirichlet:  u = value/alpha                         (beta reduces to 0)
//   Natural:    beta q_n + alpha u + gamma u_t = value  (beta != 0)
//
// where q_n = -n.A grad u is the natural flux of the equation. A condition
// written with du/dn or u_x/u_y terms is accepted only when that derivative
// vector is exactly (symbolically, after restriction to the edge)
// proportional to the flux vector -A n; its projection is then added to beta.
// Oblique conditions that are not proportional are rejected.
//
// This header is pure: no mesh, no quadrature, no evaluation. Restricted
// expressions are written in the edge parameter s in [0,1], carried by the
// symbol `x` (point = a + s (b - a)); y, nx and ny are eliminated.
#include "p1_element.hpp"
#include "problem.hpp"
#include <stdexcept>

namespace pde2d::fem {
// SymEngine substitution is simultaneous: x on the right is the new edge
// coordinate s, not the original world coordinate. Restrict BEFORE deciding
// equivalence or choosing a nonzero flux component for division.
inline Expr restrict_to_edge(const Expr &e, const p1::EdgeFrame &edge) {
  const auto a = edge.a, b = edge.b;
  return SymEngine::expand(
      e.subs({{nx.get_basic(), Expr(edge.normal_x).get_basic()},
              {ny.get_basic(), Expr(edge.normal_y).get_basic()},
              {x.get_basic(), (Expr(a.x) + (b.x - a.x) * x).get_basic()},
              {y.get_basic(), (Expr(a.y) + (b.y - a.y) * x).get_basic()}}));
}

struct EdgeCondition {
  bool dirichlet = false;
  // Restricted coefficients of  beta q_n + alpha u + gamma u_t = value.
  Expr alpha, gamma, value, beta;
  // value/alpha on Dirichlet edges, otherwise 0.
  Expr prescribed;
};

inline EdgeCondition classify_edge_condition(const Equation &eq,
                                             const BoundaryCondition &bc,
                                             const p1::EdgeFrame &edge) {
  const auto on_edge = [&](const Expr &e) { return restrict_to_edge(e, edge); };
  EdgeCondition out;
  out.alpha = on_edge(bc.alpha);
  out.gamma = on_edge(bc.gamma);
  out.value = on_edge(bc.value);
  const Expr symbolic_flux_x = -nx * eq.a_xx - ny * eq.a_xy / 2;
  const Expr symbolic_flux_y = -nx * eq.a_xy / 2 - ny * eq.a_yy;
  const Expr symbolic_extra_x = bc.beta_normal * nx + bc.beta_x;
  const Expr symbolic_extra_y = bc.beta_normal * ny + bc.beta_y;
  const Expr flux_x = on_edge(symbolic_flux_x);
  const Expr flux_y = on_edge(symbolic_flux_y);
  const Expr extra_x = on_edge(symbolic_extra_x);
  const Expr extra_y = on_edge(symbolic_extra_y);
  Expr beta = on_edge(bc.beta_flux);
  if (!is_zero(extra_x) || !is_zero(extra_y)) {
    // Cancel symbolic identities before inserting rounded edge normals.
    // Otherwise even n and 10*n can acquire a nonzero determinant from
    // floating-point multiplication. Still restrict to this edge before
    // the exact-zero decision; do not admit approximate oblique data.
    const Expr determinant =
        on_edge(SymEngine::expand(symbolic_extra_x * symbolic_flux_y -
                                  symbolic_extra_y * symbolic_flux_x));
    if (!is_zero(determinant) || (is_zero(flux_x) && is_zero(flux_y)))
      throw std::invalid_argument(
          "Unsupported oblique/normal derivative boundary condition: "
          "derivative vector is not algebraically proportional to natural "
          "flux");
    // Project only AFTER proving exact proportionality. This quotient
    // avoids a removable 0/0 where one component vanishes but the flux
    // vector does not.
    beta = beta + (extra_x * flux_x + extra_y * flux_y) /
                      (flux_x * flux_x + flux_y * flux_y);
  }
  out.beta = beta;
  out.dirichlet = is_zero(beta);
  if (out.dirichlet && !is_zero(out.gamma))
    throw std::invalid_argument(
        "Derivative-free dynamic boundary conditions are unsupported");
  if (out.dirichlet && is_zero(out.alpha))
    throw std::invalid_argument("Boundary condition has no nonzero operator");
  out.prescribed = out.dirichlet ? out.value / out.alpha : Expr(0);
  return out;
}
} // namespace pde2d::fem

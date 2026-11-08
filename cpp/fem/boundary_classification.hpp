#pragma once
// Symbolic reduction of a general BoundaryCondition on one straight boundary
// edge to the forms the P1 assembler supports, one alternative of EdgeForm:
//
//   DirichletEdge:  u = value/alpha                         (beta reduces to 0)
//   NaturalEdge:    beta q_n + alpha u + gamma u_t = value  (beta != 0)
//
// where q_n = -n.A grad u is the natural flux of the equation. A condition
// written with du/dn or u_x/u_y terms is accepted only when that derivative
// vector is exactly (symbolically, after restriction to the edge)
// proportional to the flux vector -A n; its projection is then added to beta.
// Oblique conditions that are not proportional are rejected. A future
// oblique form becomes a new EdgeForm alternative; every std::visit over
// EdgeForm (here, in detail/assembly_edges.hpp and in
// detail/analysis_regions.hpp) then fails to compile until it handles it.
//
// Classification is pure: no mesh, no quadrature, no evaluation. Restricted
// expressions are written in the edge parameter s in [0,1], carried by the
// symbol `x` (point = a + s (b - a)); y, nx and ny are eliminated.
//
// The divisor of each form (Dirichlet alpha, natural beta) must keep one
// strict sign on the edge. That is checked numerically at the edge samples
// below (Evidence::Sampled), by assembly and analysis through the same
// function.
#include "p1_element.hpp"
#include "problem.hpp"
#include "quadrature.hpp"
#include <algorithm>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <variant>
#include <vector>

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

// u = prescribed on the edge; alpha and value are the restricted coefficients
// of  alpha u = value  (alpha is the divisor that must keep its sign).
struct DirichletEdge {
  Expr alpha, value;
  Expr prescribed; // value/alpha
};
// Restricted coefficients of  beta q_n + alpha u + gamma u_t = value
// (beta is the divisor that must keep its sign).
struct NaturalEdge {
  Expr alpha, gamma, value, beta;
};
// One alternative per supported boundary form.
using EdgeForm = std::variant<DirichletEdge, NaturalEdge>;

struct EdgeCondition {
  EdgeForm form;
  bool is_dirichlet() const noexcept {
    return std::holds_alternative<DirichletEdge>(form);
  }
};

namespace classification_detail {
template <class> inline constexpr bool unhandled_form = false;
}
// Report name of a form and of its divisor, the restricted expression that
// must keep one strict sign on the edge (used in error messages). Templates
// so that prepared (non-symbolic) data can name its form without
// constructing expressions.
template <class Form> constexpr const char *edge_form_name() {
  if constexpr (std::is_same_v<Form, DirichletEdge>)
    return "Dirichlet";
  else if constexpr (std::is_same_v<Form, NaturalEdge>)
    return "Natural";
  else
    static_assert(classification_detail::unhandled_form<Form>,
                  "Name every EdgeForm alternative");
}
template <class Form> constexpr const char *edge_divisor_name() {
  if constexpr (std::is_same_v<Form, DirichletEdge>)
    return "Dirichlet alpha";
  else if constexpr (std::is_same_v<Form, NaturalEdge>)
    return "Natural flux multiplier";
  else
    static_assert(classification_detail::unhandled_form<Form>,
                  "Name the divisor of every EdgeForm alternative");
}
inline const char *edge_form_name(const EdgeForm &form) {
  return std::visit(
      []<class Form>(const Form &) { return edge_form_name<Form>(); }, form);
}
inline const Expr &edge_divisor(const DirichletEdge &form) {
  return form.alpha;
}
inline const Expr &edge_divisor(const NaturalEdge &form) { return form.beta; }

// Edge coordinates where the divisor is sampled: both endpoints, the midpoint
// and every point of the edge rule of `quadrature_degree`, sorted.
inline std::vector<double> edge_divisor_samples(int quadrature_degree) {
  std::vector<double> samples{0, .5, 1};
  for (const auto &q : edge_quadrature(quadrature_degree))
    samples.push_back(q.coordinate);
  std::sort(samples.begin(), samples.end());
  return samples;
}

// True when divisor(s) is nonzero with one sign at every sample, evaluated in
// sample order. Evidence::Sampled: arbitrary functions may have unsampled
// roots between samples. Exceptions from `divisor` propagate.
template <class Divisor>
bool divisor_keeps_strict_sign(Divisor &&divisor,
                               std::span<const double> samples) {
  double previous = divisor(samples.front());
  for (double s : samples) {
    const double current = divisor(s);
    if (current == 0 || std::signbit(current) != std::signbit(previous))
      return false;
    previous = current;
  }
  return true;
}

inline EdgeCondition classify_edge_condition(const Equation &eq,
                                             const BoundaryCondition &bc,
                                             const p1::EdgeFrame &edge) {
  const auto on_edge = [&](const Expr &e) { return restrict_to_edge(e, edge); };
  Expr alpha = on_edge(bc.alpha);
  Expr gamma = on_edge(bc.gamma);
  Expr value = on_edge(bc.value);
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
  if (!is_zero(beta))
    return {NaturalEdge{std::move(alpha), std::move(gamma), std::move(value),
                        std::move(beta)}};
  if (!is_zero(gamma))
    throw std::invalid_argument(
        "Derivative-free dynamic boundary conditions are unsupported");
  if (is_zero(alpha))
    throw std::invalid_argument("Boundary condition has no nonzero operator");
  Expr prescribed = value / alpha;
  return {
      DirichletEdge{std::move(alpha), std::move(value), std::move(prescribed)}};
}
} // namespace pde2d::fem

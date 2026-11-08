#pragma once
// Symbolic reduction of a general BoundaryCondition on one straight boundary
// edge to the forms the P1 assembler supports, one alternative of EdgeForm:
//
//   DirichletEdge:  u = value/alpha                         (beta reduces to 0)
//   NaturalEdge:    beta q_n + alpha u + gamma u_t = value  (beta != 0)
//   ObliqueEdge:    beta q_n + tau du/ds + alpha u + gamma u_t = value
//
// where q_n = -n.A grad u is the natural flux of the equation and s the
// counterclockwise unit tangent of the edge. A condition written with du/dn
// or u_x/u_y terms is natural when that derivative vector is exactly
// (symbolically, after restriction to the edge) proportional to the flux
// vector -A n; its projection is then added to beta. Otherwise it is
// oblique: the total derivative vector is decomposed into a flux and a
// tangential part (derivation in detail/edge_forms/oblique.hpp), which needs
// a nonzero normal component of the conormal and of the derivative vector;
// purely tangential derivatives are rejected.
//
// Each form is one unit in detail/edge_forms/ (interface.hpp): its name,
// divisors, preparation, evaluation, scatter and analysis hook. A new form is
// a new unit plus its EdgeForm entry (forms.hpp); EXTENDING.md lists the
// steps.
//
// Classification is pure: no mesh, no quadrature, no evaluation. Restricted
// expressions are written in the edge parameter s in [0,1], carried by the
// symbol `x` (point = a + s (b - a)); y, nx and ny are eliminated.
//
// The divisors of each form (Dirichlet alpha, natural beta, oblique conormal
// component and beta) must keep one strict sign on the edge. That is checked
// numerically at the edge samples (edge_divisor_samples, Evidence::Sampled),
// by assembly and analysis through the same function.
#include "detail/edge_forms/forms.hpp"
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
struct EdgeCondition {
  EdgeForm form;
  bool is_dirichlet() const noexcept {
    return std::holds_alternative<DirichletEdge>(form);
  }
};

namespace classification_detail {
template <class> inline constexpr bool unhandled_form = false;
}
// Report name of a form and of its (first) divisor, the restricted
// expression that must keep one strict sign on the edge (used in error
// messages). Templates so that prepared (non-symbolic) data can name its form
// without constructing expressions. Each comes from the form's unit.
template <class Form> constexpr const char *edge_form_name() {
  return edge_form_detail::Unit<Form>::name;
}
template <class Form> constexpr const char *edge_divisor_name() {
  return edge_form_detail::Unit<Form>::divisor_name;
}
inline const char *edge_form_name(const EdgeForm &form) {
  return std::visit(
      []<class Form>(const Form &) { return edge_form_name<Form>(); }, form);
}
template <class Form>
  requires edge_form_detail::EdgeFormUnit<Form>
const Expr &edge_divisor(const Form &form) {
  return edge_form_detail::Unit<Form>::divisor(form);
}

// The derivative vectors of a condition, symbolic in nx and ny: the
// conormal c = -A n (q_n = c.grad u) and the other derivative terms
// d = beta_normal n + (beta_x, beta_y). The condition is natural on an edge
// where d is parallel to c, and oblique where it is not.
struct DerivativeVectors {
  Expr flux_x, flux_y, extra_x, extra_y;
  // det(d, c), expanded before any normal is inserted: symbolically zero
  // means d is parallel to c on every edge.
  Expr determinant() const {
    return SymEngine::expand(extra_x * flux_y - extra_y * flux_x);
  }
};
inline DerivativeVectors derivative_vectors(const Equation &eq,
                                            const BoundaryCondition &bc) {
  return {-nx * eq.a_xx - ny * eq.a_xy / 2, -nx * eq.a_xy / 2 - ny * eq.a_yy,
          bc.beta_normal * nx + bc.beta_x, bc.beta_normal * ny + bc.beta_y};
}
// True unless the condition is natural or Dirichlet on every edge, whatever
// its normal: d != 0 and det(d, c) is not symbolically zero. The mesh-free
// judgement of analyze(problem); classify_edge_condition decides per edge.
inline bool may_be_oblique(const Equation &eq, const BoundaryCondition &bc) {
  const auto v = derivative_vectors(eq, bc);
  return (!is_zero(v.extra_x) || !is_zero(v.extra_y)) &&
         !is_zero(v.determinant());
}

inline EdgeCondition classify_edge_condition(const Equation &eq,
                                             const BoundaryCondition &bc,
                                             const p1::EdgeFrame &edge) {
  const auto on_edge = [&](const Expr &e) { return restrict_to_edge(e, edge); };
  Expr alpha = on_edge(bc.alpha);
  Expr gamma = on_edge(bc.gamma);
  Expr value = on_edge(bc.value);
  const auto vectors = derivative_vectors(eq, bc);
  const Expr &symbolic_flux_x = vectors.flux_x;
  const Expr &symbolic_flux_y = vectors.flux_y;
  const Expr &symbolic_extra_x = vectors.extra_x;
  const Expr &symbolic_extra_y = vectors.extra_y;
  const Expr flux_x = on_edge(symbolic_flux_x);
  const Expr flux_y = on_edge(symbolic_flux_y);
  const Expr extra_x = on_edge(symbolic_extra_x);
  const Expr extra_y = on_edge(symbolic_extra_y);
  Expr beta = on_edge(bc.beta_flux);
  if (!is_zero(extra_x) || !is_zero(extra_y)) {
    // Cancel symbolic identities before inserting rounded edge normals.
    // Otherwise even n and 10*n can acquire a nonzero determinant from
    // floating-point multiplication. Still restrict to this edge before
    // the exact-zero decision; approximately proportional data are oblique,
    // never projected onto the flux.
    const Expr determinant = on_edge(vectors.determinant());
    if (is_zero(flux_x) && is_zero(flux_y))
      throw std::invalid_argument(
          "Unsupported oblique/normal derivative boundary condition: "
          "derivative vector is not algebraically proportional to natural "
          "flux (the natural flux vanishes on the edge)");
    if (!is_zero(determinant))
      return {edge_form_detail::Unit<ObliqueEdge>::decompose(
          std::move(alpha), std::move(gamma), std::move(value), beta,
          symbolic_flux_x, symbolic_flux_y, symbolic_extra_x, symbolic_extra_y,
          determinant, edge)};
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

#pragma once
// Edge form unit: general oblique-derivative edges (interface.hpp).
//
// The input condition (problem.hpp)
//
//   alpha u + gamma u_t + beta_flux q_n + beta_normal du/dn
//       + beta_x u_x + beta_y u_y = value,       q_n = -n.A grad u = c.grad u
//
// with the conormal c = -A n is, on one straight edge, alpha u + gamma u_t
// + D.grad u = value with the total derivative vector
// D = beta_flux c + beta_normal n + (beta_x, beta_y). When D is not
// (symbolically) parallel to c, classification returns this form instead of
// NaturalEdge.
//
// Decomposition. Let n be the outward unit normal of the straight mesh edge
// and s = (-n_y, n_x) its unit tangent: n turned counterclockwise, so s
// points along the boundary with the domain on its left (the positive
// orientation of the boundary), whatever the order of the edge's nodes. When
// the conormal is not tangential, n.c = -n.A n != 0, {c, s} is a basis and
//
//   D.grad u = beta q_n + tau du/ds,
//   beta = n.D / n.c = beta_flux + n.d / n.c,
//   tau  = s.D - beta s.c = (s.d n.c - n.d s.c) / n.c = det(c, d) / n.c,
//
// where d = D - beta_flux c are the derivative terms other than the flux.
// Both conditions below are required and are checked like the natural flux
// multiplier: symbolically zero on the edge rejects the edge, and otherwise
// each must keep one strict sign at the sampled edge points (Sampled):
//
//   "Oblique conormal component"  n.c  (always positive for an elliptic,
//                                       negative definite A), and
//   "Oblique flux multiplier"     beta (zero iff D is tangential: then the
//                                       condition carries no q_n at all and
//                                       is rejected as unsupported).
//
// Weak form. The interior form's boundary term is
// int (n.A grad u) phi = -int q_n phi. Solving the condition for q_n,
//
//   q_n = (value - alpha u - gamma u_t - tau du/ds) / beta,
//
// gives the natural form's terms (natural.hpp) plus
//
//   stiffness += int (tau/beta) du/ds phi_i ds.
//
// On a straight P1 edge from node a to node b, du/ds is constant:
// du/ds = o (u_b - u_a) / L with o = sign((b - a).s) and L the edge length.
// The term is therefore the nonsymmetric 2x2 block
//
//   K_ij += int r phi_i ds * o (j == b ? 1 : -1) / L,   r = tau/beta,
//
// which couples the two edge nodes in both rows (for constant r it is
// r/2 [[-o, o], [-o, o]]). Summed over a chain of edges with constant r the
// interior boundary nodes get r/2 (u_next - u_previous), a skew-symmetric
// central difference; the assembled stiffness is nonsymmetric whenever
// tau != 0.
//
// Corners. Each edge uses its own n, s and coefficients; the weak form has
// no extra point terms at corners between edges or at the ends of a region
// (du/ds is never integrated by parts along the boundary), so none are
// assembled.
//
// Convergence. For a smooth solution and an obliqueness that varies smoothly
// along the boundary, P1 converges at second order in L2 and first order in
// H1 (tests/fem/oblique_conditions.cpp). Where the oblique direction jumps
// at a corner the dual (adjoint) problem can have a corner singularity, as
// for oblique-derivative problems in polygons in general; L2 rates near 1.5
// were measured for some 45-degree jumps on the unit square, H1 stayed first
// order. This is not a defect of the assembly, but it is not predicted here.
//
// Well-posedness. Oblique-derivative problems are not well posed in general:
// int r u du/ds is skew only for constant r on a closed boundary path (it
// equals -1/2 int r_s u^2 plus endpoint terms otherwise), so the form may
// lose coercivity, and the nonsymmetric matrix rules out the conjugate
// gradient solver. Analysis reports the edges as BoundaryForm::Oblique and
// does not claim well-posedness (detail/analysis_regions.hpp).
#include "fem/detail/edge_forms/natural.hpp"

namespace pde2d::fem {
// Restricted coefficients of
//   beta q_n + tau du/ds + alpha u + gamma u_t = value,
// with s the counterclockwise unit tangent of the edge (s = (-n_y, n_x)), and
// conormal = n.(-A n). beta and conormal are the divisors that must keep
// their sign.
struct ObliqueEdge {
  Expr alpha, gamma, value, beta, tau;
  Expr conormal;
};
} // namespace pde2d::fem

namespace pde2d::fem::edge_form_detail {
// Add one quadrature point of  int r du/ds phi_i  at edge coordinate s with
// weight w (length included). orientation = sign((b - a).tangent).
inline void accumulate_tangential_point(LocalBlock &out, double s, double w,
                                        double r, double orientation,
                                        double length) {
  const std::array<double, 2> phi{1 - s, s};
  const std::array<double, 2> dphi{-orientation / length, orientation / length};
  for (std::size_t i = 0; i < 2; ++i)
    for (std::size_t j = 0; j < 2; ++j) {
      const auto ij = LocalBlock::index(i, j);
      out.stiffness[ij] = checked(out.stiffness[ij] + w * r * phi[i] * dphi[j]);
    }
}

template <> struct Unit<ObliqueEdge> {
  static constexpr const char *name = "Oblique";
  static constexpr const char *divisor_name = "Oblique flux multiplier";
  static constexpr const char *conormal_name = "Oblique conormal component";
  static constexpr BoundaryForm category = BoundaryForm::Oblique;
  static const Expr &divisor(const ObliqueEdge &form) { return form.beta; }

  // Classification (boundary_classification.hpp): the decomposition above
  // from the restricted coefficients, the symbolic (unrestricted) conormal c
  // and other derivative terms d, and the restricted det(d, c) != 0.
  static ObliqueEdge decompose(Expr alpha, Expr gamma, Expr value,
                               const Expr &beta_flux, const Expr &c_x,
                               const Expr &c_y, const Expr &d_x,
                               const Expr &d_y, const Expr &determinant,
                               const p1::EdgeFrame &edge) {
    // Cancel symbolic identities before inserting rounded normals.
    Expr conormal =
        restrict_to_edge(SymEngine::expand(nx * c_x + ny * c_y), edge);
    const Expr normal_d =
        restrict_to_edge(SymEngine::expand(nx * d_x + ny * d_y), edge);
    if (is_zero(conormal))
      throw std::invalid_argument(
          "Unsupported oblique derivative boundary condition: the conormal "
          "-A n is tangential to the edge (n.A n == 0)");
    // n.D = beta n.c; zero means D is tangential to the edge.
    const Expr normal_total =
        SymEngine::expand(beta_flux * conormal + normal_d);
    if (is_zero(normal_total))
      throw std::invalid_argument(
          "Unsupported oblique derivative boundary condition: the derivative "
          "is tangential to the edge (no normal component)");
    Expr beta = normal_total / conormal;
    // det(c, d) = -det(d, c).
    Expr tau = -determinant / conormal;
    return {std::move(alpha), std::move(gamma), std::move(value),
            std::move(beta),  std::move(tau),   std::move(conormal)};
  }

  struct Prepared {
    using Form = ObliqueEdge;
    EdgeCoefficients coefficients;
    Evaluator tau, conormal;
    EdgeVariation variable;
    // sign((b - a).s) of the edge, +1 or -1.
    double orientation;
    // Cached time-independent contribution (leading 2x2 block).
    LocalBlock block;
    template <class F> void for_each(F &&f) const {
      coefficients.for_each(f);
      f(tau);
      f(conormal);
    }
  };

  // natural_type_block hooks: the conormal check, tau at the endpoints and
  // the tangential term.
  struct Tangential {
    const Prepared &form;
    void validate(const EdgeSite &site, double time) const {
      site.require_divisor_sign(conormal_name, form.conormal, time);
    }
    void endpoint(const EvaluationPoint &p) const { (void)form.tau(p); }
    void point(LocalBlock &block, double s, double w, const EvaluationPoint &p,
               double bv, bool stiffness) const {
      // Prepared/assembled split as for alpha: tau varies with stiffness.
      if (stiffness)
        accumulate_tangential_point(block, s, w, checked(form.tau(p) / bv),
                                    form.orientation, length);
    }
    double length;
  };

  static Prepared prepare(const ObliqueEdge &c, const p1::EdgeFrame &frame,
                          const EdgeSite &site) {
    const auto varies = [](const Expr &e) { return depends_on(e, t); };
    const double tangent_x = -frame.normal_y, tangent_y = frame.normal_x;
    Prepared out{{Evaluator(c.alpha), Evaluator(c.gamma), Evaluator(c.value),
                  Evaluator(c.beta)},
                 Evaluator(c.tau),
                 Evaluator(c.conormal),
                 {.stiffness = varies(c.alpha) || varies(c.beta) ||
                               varies(c.tau) || varies(c.conormal),
                  .first_mass = varies(c.gamma) || varies(c.beta),
                  .load = varies(c.value) || varies(c.beta)},
                 (frame.b.x - frame.a.x) * tangent_x +
                             (frame.b.y - frame.a.y) * tangent_y >
                         0
                     ? 1.
                     : -1.,
                 {}};
    out.block = natural_type_block(out.coefficients, out.variable, divisor_name,
                                   site, 0, true, Tangential{out, site.length})
                    .block;
    return out;
  }

  static EdgeResult evaluate(const Prepared &form, const EdgeSite &site,
                             double time, DirichletDerivative) {
    return natural_type_evaluate(form.coefficients, form.variable, form.block,
                                 divisor_name, site, time,
                                 Tangential{form, site.length});
  }

  static void scatter(const EdgeResult &result, const EdgeNodes &nodes,
                      ScatterTarget &target) {
    scatter_block(result.block, nodes, target);
  }

  static EdgeJudgement judge(const ObliqueEdge &form) {
    return {category,
            {{conormal_name, &form.conormal}, {divisor_name, &form.beta}},
            {&form.alpha, &form.gamma, &form.value, &form.beta, &form.tau,
             &form.conormal},
            &form.alpha,
            &form.tau};
  }
};
} // namespace pde2d::fem::edge_form_detail

#pragma once
// Edge form unit: natural (flux Robin) edges (interface.hpp).
//
//   beta q_n + alpha u + gamma u_t = value,   q_n = -n.A grad u.
//
// Classification returns it when the derivative data are exactly
// (symbolically, after restriction to the edge) proportional to the natural
// flux; beta is then the projected flux multiplier. The weak form's boundary
// term  int (n.A grad u) phi = -int q_n phi  becomes, with
// q_n = (value - alpha u - gamma u_t)/beta,
//
//   stiffness += int alpha/beta u phi,  first mass += int gamma/beta u_t phi,
//   load      += int value/beta phi,
//
// a symmetric 2x2 edge block (p1::accumulate_edge_point). beta is the divisor
// and must keep one strict sign on the edge (sampled). The time-independent
// part is cached at preparation.
//
// natural_type_block is shared with the oblique unit (oblique.hpp), which
// adds a tangential term through its `extra` hooks; with NoExtraTerms the
// arithmetic is exactly that of this form.
#include "fem/detail/edge_forms/interface.hpp"

namespace pde2d::fem {
// Restricted coefficients of  beta q_n + alpha u + gamma u_t = value
// (beta is the divisor that must keep its sign).
struct NaturalEdge {
  Expr alpha, gamma, value, beta;
};
} // namespace pde2d::fem

namespace pde2d::fem::edge_form_detail {
// Restricted coefficients of  beta q_n + alpha u + gamma u_t = value.
struct EdgeCoefficients {
  Evaluator alpha, gamma, value, beta;
  template <class F> void for_each(F &&f) const {
    f(alpha);
    f(gamma);
    f(value);
    f(beta);
  }
};

// Which natural-boundary contributions depend on time.
struct EdgeVariation {
  bool stiffness = false, first_mass = false, load = false;
  bool any() const { return stiffness || first_mass || load; }
};

// Hooks of a natural-type form beyond the flux Robin terms.
struct NoExtraTerms {
  // Before the beta sign check.
  void validate(const EdgeSite &, double) const {}
  // At each edge endpoint, after alpha, gamma and value.
  void endpoint(const EvaluationPoint &) const {}
  // At each quadrature point after the Robin terms; bv = beta there.
  void point(LocalBlock &, double, double, const EvaluationPoint &, double,
             bool) const {}
};

// cache_only=true computes the time-independent part at preparation;
// otherwise only the time-dependent part (added to the cache by evaluate).
template <class Extra = NoExtraTerms>
EdgeResult natural_type_block(const EdgeCoefficients &coefficients,
                              const EdgeVariation &variable,
                              const char *divisor_name, const EdgeSite &site,
                              double time, bool cache_only,
                              const Extra &extra = {}) {
  EdgeResult result;
  const auto &[alpha, gamma, value, beta] = coefficients;
  if (!cache_only || !variable.any()) {
    extra.validate(site, time);
    site.require_divisor_sign(divisor_name, beta, time);
    for (int k = 0; k < 2; ++k) {
      const auto p = site.at(k, time);
      (void)alpha(p);
      (void)gamma(p);
      (void)value(p);
      extra.endpoint(p);
    }
  }
  // Prepared terms are the time-independent ones, assembled terms the
  // time-dependent ones.
  const auto active = [&](bool varies) {
    return cache_only ? !varies : varies;
  };
  const bool stiffness = active(variable.stiffness),
             first_mass = active(variable.first_mass),
             load = active(variable.load);
  if (!stiffness && !first_mass && !load)
    return result;
  for (const auto &q : site.rule) {
    const auto p = site.at(q.coordinate, time);
    const double bv = beta(p);
    const double av = stiffness ? checked(alpha(p) / bv) : 0,
                 gv = first_mass ? checked(gamma(p) / bv) : 0,
                 f = load ? checked(value(p) / bv) : 0,
                 w = site.length * q.weight;
    p1::accumulate_edge_point(result.block, q.coordinate, w, av, gv, f);
    extra.point(result.block, q.coordinate, w, p, bv, stiffness);
  }
  return result;
}

// Cached block when nothing varies, else the cache plus the varying part.
template <class Extra = NoExtraTerms>
EdgeResult natural_type_evaluate(const EdgeCoefficients &coefficients,
                                 const EdgeVariation &variable,
                                 const LocalBlock &cached,
                                 const char *divisor_name, const EdgeSite &site,
                                 double time, const Extra &extra = {}) {
  if (!variable.any())
    return {cached, {}};
  auto result = natural_type_block(coefficients, variable, divisor_name, site,
                                   time, false, extra);
  add_cached(result.block, cached);
  return result;
}

template <> struct Unit<NaturalEdge> {
  static constexpr const char *name = "Natural";
  static constexpr const char *divisor_name = "Natural flux multiplier";
  static constexpr BoundaryForm category = BoundaryForm::Natural;
  static const Expr &divisor(const NaturalEdge &form) { return form.beta; }

  struct Prepared {
    using Form = NaturalEdge;
    EdgeCoefficients coefficients;
    EdgeVariation variable;
    // Cached time-independent contribution (leading 2x2 block).
    LocalBlock block;
    template <class F> void for_each(F &&f) const { coefficients.for_each(f); }
  };

  static Prepared prepare(const NaturalEdge &c, const p1::EdgeFrame &,
                          const EdgeSite &site) {
    const auto varies = [](const Expr &e) { return depends_on(e, t); };
    Prepared out{{Evaluator(c.alpha), Evaluator(c.gamma), Evaluator(c.value),
                  Evaluator(c.beta)},
                 {.stiffness = varies(c.alpha) || varies(c.beta),
                  .first_mass = varies(c.gamma) || varies(c.beta),
                  .load = varies(c.value) || varies(c.beta)},
                 {}};
    out.block = natural_type_block(out.coefficients, out.variable, divisor_name,
                                   site, 0, true)
                    .block;
    return out;
  }

  static EdgeResult evaluate(const Prepared &form, const EdgeSite &site,
                             double time, DirichletDerivative) {
    return natural_type_evaluate(form.coefficients, form.variable, form.block,
                                 divisor_name, site, time);
  }

  static void scatter(const EdgeResult &result, const EdgeNodes &nodes,
                      ScatterTarget &target) {
    scatter_block(result.block, nodes, target);
  }

  static EdgeJudgement judge(const NaturalEdge &form) {
    return {category,
            {{divisor_name, &form.beta}},
            {&form.alpha, &form.gamma, &form.value, &form.beta},
            &form.alpha,
            nullptr};
  }
};
} // namespace pde2d::fem::edge_form_detail

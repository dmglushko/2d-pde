#pragma once
// Edge form unit: Dirichlet edges (interface.hpp).
//
//   alpha u = value  on the edge, so u = prescribed = value/alpha.
//
// Classification returns it when every derivative coefficient reduces to
// zero on the edge. alpha is the divisor and must keep one strict sign on the
// edge (sampled). Assembly evaluates the prescribed value, or its first or
// second time derivative, at the two edge nodes; the scatter records the
// first value of every node and rejects a different value at a shared
// corner ("Contradictory corner Dirichlet values"). Values are eliminated
// later by constrain() (system.hpp).
#include "fem/detail/edge_forms/interface.hpp"
#include <exception>
#include <new>

namespace pde2d::fem {
// u = prescribed on the edge; alpha and value are the restricted coefficients
// of  alpha u = value  (alpha is the divisor that must keep its sign).
struct DirichletEdge {
  Expr alpha, value;
  Expr prescribed; // value/alpha
};
} // namespace pde2d::fem

namespace pde2d::fem::edge_form_detail {
// One prescribed Dirichlet time derivative. When the symbolic library could
// not form it, `error` keeps the exception, rethrown only if it is requested.
struct PreparedDerivative {
  Evaluator evaluator;
  std::exception_ptr error{};
};
// Prescribed Dirichlet value g and its time derivatives g_t and g_tt.
struct DirichletData {
  PreparedDerivative value, first_derivative, second_derivative;
  const PreparedDerivative &derivative(DirichletDerivative d) const {
    return d == DirichletDerivative::Value             ? value
           : d == DirichletDerivative::FirstDerivative ? first_derivative
                                                       : second_derivative;
  }
  PreparedDerivative &derivative(DirichletDerivative d) {
    return d == DirichletDerivative::Value             ? value
           : d == DirichletDerivative::FirstDerivative ? first_derivative
                                                       : second_derivative;
  }
  template <class F> void for_each(F &&f) const {
    f(value.evaluator);
    f(first_derivative.evaluator);
    f(second_derivative.evaluator);
  }
};

template <> struct Unit<DirichletEdge> {
  static constexpr const char *name = "Dirichlet";
  static constexpr const char *divisor_name = "Dirichlet alpha";
  static const Expr &divisor(const DirichletEdge &form) { return form.alpha; }

  struct Prepared {
    using Form = DirichletEdge;
    Evaluator alpha, value;
    DirichletData prescribed;
    template <class F> void for_each(F &&f) const {
      f(alpha);
      f(value);
      prescribed.for_each(f);
    }
  };

  static Prepared prepare(const DirichletEdge &c, const p1::EdgeFrame &,
                          const EdgeSite &) {
    Prepared out{Evaluator(c.alpha),
                 Evaluator(c.value),
                 {{Evaluator(c.prescribed)}, {Evaluator(0)}, {Evaluator(0)}}};
    // Prepare supported derivatives serially, but preserve the value-only
    // API when the symbolic library cannot evaluate an unused derivative.
    // Re-throw only if that derivative is actually requested. Workers never
    // construct/copy symbolic expressions or mutate a lazy cache.
    for (auto derivative : {DirichletDerivative::FirstDerivative,
                            DirichletDerivative::SecondDerivative}) {
      auto &prepared = out.prescribed.derivative(derivative);
      try {
        Expr g = c.prescribed;
        for (int k = 0; k < dirichlet_derivative_order(derivative); ++k)
          g = diff(g, t);
        prepared.evaluator = Evaluator(g);
      } catch (const std::bad_alloc &) {
        throw;
      } catch (...) {
        prepared.error = std::current_exception();
      }
    }
    return out;
  }

  // Values of the requested derivative at the two edge nodes.
  static EdgeResult evaluate(const Prepared &form, const EdgeSite &site,
                             double time, DirichletDerivative derivative) {
    site.require_divisor_sign(divisor_name, form.alpha, time);
    EdgeResult result;
    const auto &prescribed = form.prescribed.derivative(derivative);
    if (prescribed.error)
      std::rethrow_exception(prescribed.error);
    for (std::size_t k = 0; k < result.values.size(); ++k)
      result.values[k] =
          prescribed.evaluator(site.at(static_cast<double>(k), time));
    for (const auto &q : site.rule)
      (void)form.value(site.at(q.coordinate, time));
    return result;
  }

  static void scatter(const EdgeResult &result, const EdgeNodes &nodes,
                      ScatterTarget &target) {
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      double g = result.values[i];
      auto [it, inserted] = target.prescribed.emplace(nodes[i], g);
      if (!inserted &&
          std::abs(it->second - g) >
              target.boundary_tolerance *
                  std::max({1., std::abs(it->second), std::abs(g)}))
        throw std::invalid_argument("Contradictory corner Dirichlet values");
    }
  }
};
} // namespace pde2d::fem::edge_form_detail

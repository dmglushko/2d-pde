#pragma once
// Private part of PreparedAssembly (assembly.hpp): boundary edges.
//
// Preparation takes the validated region binding (boundary_assignment.hpp),
// reduces each edge's boundary condition to one EdgeForm
// alternative (boundary_classification.hpp), compiles its restricted
// coefficients into the matching prepared alternative and caches the
// time-independent natural contribution. evaluate() computes the remaining
// part at one time; it only reads prepared data. Dispatch is std::visit over
// PreparedEdgeForm, so a new form does not compile until every step handles
// it.
#include "discretization/geometry_base.hpp"
#include "fem/boundary_assignment.hpp"
#include "fem/boundary_classification.hpp"
#include "fem/detail/assembly_elements.hpp"
#include "fem/expression.hpp"
#include "fem/p1_element.hpp"
#include "fem/problem.hpp"
#include "fem/quadrature.hpp"
#include "fem/system.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <new>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace pde2d::fem::assembly_detail {
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

// Which natural-boundary contributions depend on time.
struct EdgeVariation {
  bool stiffness = false, first_mass = false, load = false;
  bool any() const { return stiffness || first_mass || load; }
};

// Prepared counterpart of each EdgeForm alternative (`Form` names it).
struct PreparedDirichletEdge {
  using Form = DirichletEdge;
  Evaluator alpha, value;
  DirichletData prescribed;
  template <class F> void for_each(F &&f) const {
    f(alpha);
    f(value);
    prescribed.for_each(f);
  }
};
struct PreparedNaturalEdge {
  using Form = NaturalEdge;
  EdgeCoefficients coefficients;
  EdgeVariation variable;
  // Cached time-independent contribution (leading 2x2 block).
  LocalBlock block;
  template <class F> void for_each(F &&f) const { coefficients.for_each(f); }
};
using PreparedEdgeForm =
    std::variant<PreparedDirichletEdge, PreparedNaturalEdge>;

struct PreparedEdge {
  std::array<std::size_t, 2> nodes;
  double length, normal_x, normal_y;
  PreparedEdgeForm form;

  bool is_dirichlet() const noexcept {
    return std::holds_alternative<PreparedDirichletEdge>(form);
  }
  std::size_t evaluator_count() const {
    std::size_t count = 0;
    std::visit(
        [&](const auto &f) { f.for_each([&](const Evaluator &) { ++count; }); },
        form);
    return count;
  }
};
struct EdgeResult {
  LocalBlock block;
  // Dirichlet values at the two edge nodes.
  std::array<double, 2> values{};
};

// Add the time-independent natural contribution cached at preparation.
inline void add_cached(LocalBlock &out, const LocalBlock &cached) {
  for (std::size_t k = 0; k < out.stiffness.size(); ++k) {
    out.stiffness[k] += cached.stiffness[k];
    out.first[k] += cached.first[k];
  }
  for (std::size_t k = 0; k < out.load.size(); ++k)
    out.load[k] += cached.load[k];
}

class PreparedBoundary {
public:
  std::vector<PreparedEdge> edges;
  std::vector<EdgeQuadraturePoint> rule;
  // Edge coordinates where the Dirichlet alpha or natural flux multiplier
  // must keep one strict sign (edge_divisor_samples).
  std::vector<double> divisor_samples;

  // Set up the edge rule (validates the quadrature degree).
  void set_rule(int quadrature_degree) {
    rule = edge_quadrature(quadrature_degree);
    divisor_samples = edge_divisor_samples(quadrature_degree);
  }

  // Classify and prepare every bound edge (after set_rule()).
  void prepare(const Problem &problem, const BoundaryBinding &binding) {
    for (const auto &bound : binding.edges) {
      const auto &frame = bound.frame;
      const auto c =
          classify_edge_condition(problem.equation, *bound.condition, frame);
      PreparedEdge e{bound.nodes, frame.length, frame.normal_x, frame.normal_y,
                     std::visit(
                         [](const auto &form) -> PreparedEdgeForm {
                           return prepare(form);
                         },
                         c.form)};
      if (auto *natural = std::get_if<PreparedNaturalEdge>(&e.form))
        natural->block = natural_block(e, *natural, 0, true).block;
      edges.push_back(std::move(e));
    }
  }

  // Complete contribution of one edge at `time`: Dirichlet values of the
  // requested derivative, or the cached plus time-dependent natural block.
  EdgeResult evaluate(const PreparedEdge &e, double time,
                      DirichletDerivative derivative) const {
    return std::visit(
        [&]<class Form>(const Form &form) -> EdgeResult {
          if constexpr (std::is_same_v<Form, PreparedDirichletEdge>)
            return dirichlet_values(e, form, time, derivative);
          else if constexpr (std::is_same_v<Form, PreparedNaturalEdge>) {
            if (!form.variable.any())
              return {form.block, {}};
            auto result = natural_block(e, form, time);
            add_cached(result.block, form.block);
            return result;
          } else
            static_assert(classification_detail::unhandled_form<Form>,
                          "Assemble every prepared edge form");
        },
        e.form);
  }

private:
  static PreparedDirichletEdge prepare(const DirichletEdge &c) {
    PreparedDirichletEdge out{
        Evaluator(c.alpha),
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
  static PreparedNaturalEdge prepare(const NaturalEdge &c) {
    const auto varies = [](const Expr &e) { return depends_on(e, t); };
    return {{Evaluator(c.alpha), Evaluator(c.gamma), Evaluator(c.value),
             Evaluator(c.beta)},
            {.stiffness = varies(c.alpha) || varies(c.beta),
             .first_mass = varies(c.gamma) || varies(c.beta),
             .load = varies(c.value) || varies(c.beta)},
            {}};
  }

  static EvaluationPoint at(const PreparedEdge &e, double s, double time) {
    return EvaluationPoint{s, 0, time, e.normal_x, e.normal_y};
  }
  // Endpoint/midpoint/quadrature sign sampling, shared with analysis. This is
  // not a proof that arbitrary symbolic functions have no unsampled roots.
  template <class Prepared>
  void require_divisor_sign(const PreparedEdge &e, const Evaluator &divisor,
                            double time) const {
    if (!divisor_keeps_strict_sign(
            [&](double s) { return divisor(at(e, s, time)); }, divisor_samples))
      throw std::invalid_argument(
          std::string(edge_divisor_name<typename Prepared::Form>()) +
          " vanishes or changes sign on boundary");
  }

  EdgeResult dirichlet_values(const PreparedEdge &e,
                              const PreparedDirichletEdge &form, double time,
                              DirichletDerivative derivative) const {
    require_divisor_sign<PreparedDirichletEdge>(e, form.alpha, time);
    EdgeResult result;
    const auto &prescribed = form.prescribed.derivative(derivative);
    if (prescribed.error)
      std::rethrow_exception(prescribed.error);
    for (std::size_t k = 0; k < result.values.size(); ++k)
      result.values[k] =
          prescribed.evaluator(at(e, static_cast<double>(k), time));
    for (const auto &q : rule)
      (void)form.value(at(e, q.coordinate, time));
    return result;
  }

  // cache_only=true computes the time-independent part at preparation;
  // otherwise only the time-dependent part (added to the cache by evaluate).
  EdgeResult natural_block(const PreparedEdge &e,
                           const PreparedNaturalEdge &form, double time,
                           bool cache_only = false) const {
    EdgeResult result;
    const auto &[alpha, gamma, value, beta] = form.coefficients;
    const auto &variable = form.variable;
    if (!cache_only || !variable.any()) {
      require_divisor_sign<PreparedNaturalEdge>(e, beta, time);
      for (int k = 0; k < 2; ++k) {
        (void)alpha(at(e, k, time));
        (void)gamma(at(e, k, time));
        (void)value(at(e, k, time));
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
    for (const auto &q : rule) {
      const auto p = at(e, q.coordinate, time);
      const double bv = beta(p);
      const double av = stiffness ? checked(alpha(p) / bv) : 0,
                   gv = first_mass ? checked(gamma(p) / bv) : 0,
                   f = load ? checked(value(p) / bv) : 0,
                   w = e.length * q.weight;
      p1::accumulate_edge_point(result.block, q.coordinate, w, av, gv, f);
    }
    return result;
  }
};
} // namespace pde2d::fem::assembly_detail

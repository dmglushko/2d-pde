#pragma once
// The uniform interface of boundary edge forms.
//
// classify_edge_condition (boundary_classification.hpp) reduces the
// BoundaryCondition of one straight boundary edge to one EdgeForm
// alternative (forms.hpp). Everything a form does after that lives in ONE
// unit, the specialization edge_form_detail::Unit<Form> in its own header
// next to this one (dirichlet.hpp, natural.hpp, oblique.hpp). Generic code
// (classification names, PreparedAssembly's boundary, the serial scatter and
// the region analysis) only calls through this interface:
//
//   template <> struct Unit<MyEdge> {
//     static constexpr const char *name;          // edge_form_name
//     static constexpr const char *divisor_name;  // edge_divisor_name
//     static constexpr BoundaryForm category;     // analysis report category
//     static const Expr &divisor(const MyEdge &); // edge_divisor
//     struct Prepared {                           // numeric data only
//       using Form = MyEdge;
//       template <class F> void for_each(F &&) const;  // every Evaluator
//     };
//     // Compile evaluators, cache time-independent parts (serial).
//     static Prepared prepare(const MyEdge &, const p1::EdgeFrame &,
//                             const EdgeSite &);
//     // Complete local contribution at `time`; read-only, so parallel
//     // workers may call it concurrently.
//     static EdgeResult evaluate(const Prepared &, const EdgeSite &,
//                                double time, DirichletDerivative);
//     // Serial scatter of that contribution in a fixed order.
//     static void scatter(const EdgeResult &, const EdgeNodes &,
//                         ScatterTarget &);
//     // What region analysis learns from the edge.
//     static EdgeJudgement judge(const MyEdge &);
//   };
//
// EdgeFormUnit checks this shape. forms.hpp static_asserts it for every
// EdgeForm alternative, so a new alternative without its unit does not
// compile.
//
// Also here (public, namespace pde2d::fem): edge restriction and the sampled
// divisor sign contract shared by classification, assembly and analysis.
#include "fem/boundary_form.hpp"
#include "fem/expression.hpp"
#include "fem/p1_element.hpp"
#include "fem/quadrature.hpp"
#include "fem/system.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <map>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
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
} // namespace pde2d::fem

namespace pde2d::fem::edge_form_detail {
using p1::checked;
using p1::LocalBlock;

// Defined once per EdgeForm alternative by its unit header.
template <class Form> struct Unit;

using EdgeNodes = std::array<std::size_t, 2>;

// Where a unit evaluates: one boundary edge (length, outward normal) and the
// edge rule and divisor samples shared by all edges. Restricted expressions
// use x as the edge coordinate s in [0, 1].
struct EdgeSite {
  double length, normal_x, normal_y;
  const std::vector<EdgeQuadraturePoint> &rule;
  const std::vector<double> &divisor_samples;

  EvaluationPoint at(double s, double time) const {
    return EvaluationPoint{s, 0, time, normal_x, normal_y};
  }
  // Endpoint/midpoint/quadrature sign sampling, shared with analysis. This is
  // not a proof that arbitrary symbolic functions have no unsampled roots.
  void require_divisor_sign(const char *name, const Evaluator &divisor,
                            double time) const {
    if (!divisor_keeps_strict_sign(
            [&](double s) { return divisor(at(s, time)); }, divisor_samples))
      throw std::invalid_argument(std::string(name) +
                                  " vanishes or changes sign on boundary");
  }
};

// Local contribution of one edge: the leading 2x2 block of a LocalBlock and
// its load, or the prescribed values at the two edge nodes.
struct EdgeResult {
  LocalBlock block;
  // Dirichlet values at the two edge nodes.
  std::array<double, 2> values{};
};

// Add a time-independent contribution cached at preparation.
inline void add_cached(LocalBlock &out, const LocalBlock &cached) {
  for (std::size_t k = 0; k < out.stiffness.size(); ++k) {
    out.stiffness[k] += cached.stiffness[k];
    out.first[k] += cached.first[k];
  }
  for (std::size_t k = 0; k < out.load.size(); ++k)
    out.load[k] += cached.load[k];
}

// One nonzero matrix entry; zeros are skipped, nonfinite values throw.
inline void add_entry(std::vector<Eigen::Triplet<double>> &v, std::size_t i,
                      std::size_t j, double a) {
  if (checked(a) != 0)
    v.emplace_back(i, j, a);
}

// The global system under construction (assembly_scatter.hpp). Units append
// in their fixed local order; the edge order is the mesh order.
struct ScatterTarget {
  std::vector<Eigen::Triplet<double>> &stiffness, &first;
  Vector &load;
  // First prescribed value of every Dirichlet node.
  std::map<Eigen::Index, double> &prescribed;
  double boundary_tolerance;
};

// Scatter the leading 2x2 stiffness/first-mass block and load of an edge.
inline void scatter_block(const LocalBlock &block, const EdgeNodes &nodes,
                          ScatterTarget &target) {
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    target.load[nodes[i]] = checked(target.load[nodes[i]] + block.load[i]);
    for (std::size_t j = 0; j < nodes.size(); ++j) {
      const auto ij = LocalBlock::index(i, j);
      add_entry(target.stiffness, nodes[i], nodes[j], block.stiffness[ij]);
      add_entry(target.first, nodes[i], nodes[j], block.first[ij]);
    }
  }
}

// A restricted expression that must keep one strict sign on the edge.
struct NamedDivisor {
  const char *name;
  const Expr *expression;
};
// What region analysis (detail/analysis_regions.hpp) learns from one
// classified edge. Pointers refer into the judged form.
struct EdgeJudgement {
  BoundaryForm category;
  // Checked in this order at every sample time, exactly as assembly does.
  std::vector<NamedDivisor> divisors;
  // Restricted coefficients whose t dependence makes the edge time dependent.
  std::vector<const Expr *> coefficients;
  // Robin coefficient of a natural-type edge (nullspace judgement), or null.
  const Expr *robin = nullptr;
  // Coefficient of a nonsymmetric tangential-derivative term, or null.
  const Expr *tangential = nullptr;
};

template <class Form>
concept EdgeFormUnit =
    requires(const Form &form, const p1::EdgeFrame &frame, const EdgeSite &site,
             const typename Unit<Form>::Prepared &prepared,
             const EdgeResult &result, const EdgeNodes &nodes,
             ScatterTarget &target, double time) {
      sizeof(Unit<Form>);
      { Unit<Form>::name } -> std::convertible_to<const char *>;
      { Unit<Form>::divisor_name } -> std::convertible_to<const char *>;
      { Unit<Form>::category } -> std::convertible_to<BoundaryForm>;
      { Unit<Form>::divisor(form) } -> std::same_as<const Expr &>;
      requires std::same_as<typename Unit<Form>::Prepared::Form, Form>;
      {
        Unit<Form>::prepare(form, frame, site)
      } -> std::same_as<typename Unit<Form>::Prepared>;
      {
        Unit<Form>::evaluate(prepared, site, time, DirichletDerivative::Value)
      } -> std::same_as<EdgeResult>;
      Unit<Form>::scatter(result, nodes, target);
      { Unit<Form>::judge(form) } -> std::same_as<EdgeJudgement>;
    };
} // namespace pde2d::fem::edge_form_detail

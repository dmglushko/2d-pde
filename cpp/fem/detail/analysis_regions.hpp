#pragma once
// Private part of analysis (analysis.hpp): boundary regions. Per-edge
// judgements (form counts, time dependence, divisor signs checked exactly as
// assembly checks them) are accumulated per region; Robin terms feed the
// problem-level nullspace judgement.
#include "fem/analysis_types.hpp"
#include "fem/boundary_classification.hpp"
#include "fem/detail/analysis_interior.hpp"
#include "fem/expression.hpp"
#include "fem/problem.hpp"
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace pde2d::fem::analysis_detail {
// Robin coefficients of natural edges, for the nullspace judgement.
struct RobinTerms {
  bool all_zero = true, some_nonzero_constant = false;
  void add(const Expr &alpha) {
    if (is_zero(alpha))
      return;
    all_zero = false;
    const auto s = constant_sign(alpha);
    some_nonzero_constant = some_nonzero_constant || (s && *s != 0);
  }
};

inline void finish_nullspace(ProblemProperties &p, const RobinTerms &robin) {
  if (p.has_dirichlet.value && p.has_dirichlet.evidence == Evidence::Proven)
    p.constant_nullspace = {false, Evidence::Proven,
                            "Dirichlet data fix the constant"};
  else if (known_nonzero(p.reaction))
    p.constant_nullspace = {false, p.reaction.evidence,
                            "reaction c is nonzero: " + p.reaction.reason};
  else if (robin.some_nonzero_constant)
    p.constant_nullspace = {false, Evidence::Proven,
                            "a Robin coefficient is a nonzero constant"};
  else if (p.has_dirichlet.evidence == Evidence::Proven &&
           known_zero(p.reaction) && robin.all_zero)
    p.constant_nullspace = {true,
                            weakest(Evidence::Proven, p.reaction.evidence),
                            "only natural conditions without Robin terms and "
                            "c == 0: the stationary operator is singular"};
  else
    p.constant_nullspace = {false, Evidence::Unknown,
                            "boundary or reaction terms are not known"};
}

// Mesh-free boundary judgement from the structure of the condition alone.
inline BoundaryRegionProperties region_without_mesh(const std::string &region,
                                                    const BoundaryCondition &bc,
                                                    RobinTerms &robin) {
  BoundaryRegionProperties r;
  r.region = region;
  r.time_dependent = {false, Evidence::Proven, "t does not occur"};
  for (const auto *e : {&bc.alpha, &bc.gamma, &bc.beta_flux, &bc.beta_normal,
                        &bc.beta_x, &bc.beta_y, &bc.value})
    if (depends_on(*e, t))
      r.time_dependent = {true, Evidence::Proven, "t occurs"};
  const bool no_derivatives = is_zero(bc.beta_flux) &&
                              is_zero(bc.beta_normal) && is_zero(bc.beta_x) &&
                              is_zero(bc.beta_y);
  const auto flux = constant_sign(bc.beta_flux);
  if (no_derivatives) {
    // Same order of checks as classify_edge_condition.
    if (!is_zero(bc.gamma) || is_zero(bc.alpha))
      r.form = {BoundaryForm::Rejected, Evidence::Proven,
                !is_zero(bc.gamma) ? "derivative-free dynamic condition"
                                   : "no nonzero operator"};
    else
      r.form = {BoundaryForm::Dirichlet, Evidence::Proven,
                "no derivative terms"};
    const auto alpha = constant_sign(bc.alpha);
    r.divisor_keeps_sign =
        alpha && *alpha != 0
            ? Property<bool>{true, Evidence::Proven, "alpha is constant"}
            : Property<bool>{false, Evidence::Unknown,
                             "alpha is not constant; analyze with a mesh"};
  } else if (is_zero(bc.beta_normal) && is_zero(bc.beta_x) &&
             is_zero(bc.beta_y) && flux && *flux != 0) {
    r.form = {BoundaryForm::Natural, Evidence::Proven,
              "nonzero constant flux multiplier"};
    r.divisor_keeps_sign = {true, Evidence::Proven, "beta is constant"};
    robin.add(bc.alpha);
  } else {
    r.form = {BoundaryForm::Natural, Evidence::Unknown,
              "depends on the edges; analyze with a mesh"};
    r.divisor_keeps_sign = {false, Evidence::Unknown,
                            "depends on the edges; analyze with a mesh"};
    robin.all_zero = robin.all_zero && is_zero(bc.alpha);
  }
  return r;
}

// Accumulates the per-edge judgements of one region.
class RegionAccumulator {
  BoundaryRegionProperties r_;
  std::string rejected_, failure_;
  bool all_constant_ = true, unknown_ = false, time_dependent_ = false;

public:
  explicit RegionAccumulator(std::string region) {
    r_.region = std::move(region);
  }
  void reject(const std::string &message) {
    ++r_.edges;
    if (rejected_.empty())
      rejected_ = message;
  }
  // One classified edge: count it, check its divisor as assembly does.
  void add(const EdgeCondition &c, const p1::EdgeFrame &frame,
           const std::vector<double> &edge_samples,
           const AnalysisOptions &options, RobinTerms &robin) {
    ++r_.edges;
    std::visit(
        [&]<class Form>(const Form &form) {
          if constexpr (std::is_same_v<Form, DirichletEdge>) {
            ++r_.dirichlet_edges;
            time_dependent_ = time_dependent_ || depends_on(form.alpha, t) ||
                              depends_on(form.value, t);
          } else if constexpr (std::is_same_v<Form, NaturalEdge>) {
            ++r_.natural_edges;
            robin.add(form.alpha);
            for (const auto *e :
                 {&form.alpha, &form.gamma, &form.value, &form.beta})
              time_dependent_ = time_dependent_ || depends_on(*e, t);
          } else
            static_assert(classification_detail::unhandled_form<Form>,
                          "Analyze every EdgeForm alternative");
          check_divisor(edge_divisor(form), edge_divisor_name<Form>(), frame,
                        edge_samples, options);
        },
        c.form);
  }
  BoundaryRegionProperties finish(std::size_t samples_per_edge) {
    r_.time_dependent = {time_dependent_, Evidence::Proven,
                         time_dependent_ ? "t occurs on some edge"
                                         : "t does not occur"};
    if (!rejected_.empty()) {
      r_.form = {BoundaryForm::Rejected, Evidence::Proven, rejected_};
      r_.divisor_keeps_sign = {false, Evidence::Unknown,
                               "some edge could not be classified"};
      return r_;
    }
    const auto form = r_.dirichlet_edges == r_.edges ? BoundaryForm::Dirichlet
                      : r_.natural_edges == r_.edges ? BoundaryForm::Natural
                                                     : BoundaryForm::Mixed;
    r_.form = {form, Evidence::Proven,
               "symbolic classification of " + std::to_string(r_.edges) +
                   " edges"};
    if (!failure_.empty())
      r_.divisor_keeps_sign = {false, Evidence::Sampled, failure_};
    else if (unknown_)
      r_.divisor_keeps_sign = {true, Evidence::Unknown,
                               "some divisor could not be evaluated"};
    else if (all_constant_)
      r_.divisor_keeps_sign = {true, Evidence::Proven,
                               "constant nonzero divisors"};
    else
      r_.divisor_keeps_sign = {
          true, Evidence::Sampled,
          sampled_reason("divisors", samples_per_edge * r_.edges)};
    return r_;
  }

private:
  void check_divisor(const Expr &divisor, const char *name,
                     const p1::EdgeFrame &frame,
                     const std::vector<double> &edge_samples,
                     const AnalysisOptions &options) {
    if (const auto s = constant_sign(divisor); s && *s != 0)
      return;
    all_constant_ = false;
    const Evaluator f(divisor);
    for (double time : options.sample_times)
      try {
        if (!divisor_keeps_strict_sign(
                [&](double s) {
                  return f({s, 0, time, frame.normal_x, frame.normal_y});
                },
                edge_samples) &&
            failure_.empty())
          failure_ =
              std::string(name) +
              " vanishes or changes sign on boundary at t=" + number(time);
      } catch (const std::domain_error &) {
        unknown_ = true;
      }
  }
};

// Accumulate the edges of one region in order; Robin terms go to `robin`.
inline BoundaryRegionProperties
accumulate_region(std::string region, std::span<const ClassifiedEdge> edges,
                  const std::vector<double> &edge_samples,
                  const AnalysisOptions &options, RobinTerms &robin) {
  RegionAccumulator accumulator(std::move(region));
  for (const auto &edge : edges) {
    if (const auto *rejected = std::get_if<std::string>(&edge.condition)) {
      accumulator.reject(*rejected);
      continue;
    }
    // As for classification, an input error while judging the edge (e.g. a
    // divisor that cannot be compiled) makes the region Rejected.
    try {
      accumulator.add(std::get<EdgeCondition>(edge.condition), edge.frame,
                      edge_samples, options, robin);
    } catch (const std::invalid_argument &error) {
      accumulator.reject(error.what());
    }
  }
  return accumulator.finish(edge_samples.size() * options.sample_times.size());
}
} // namespace pde2d::fem::analysis_detail

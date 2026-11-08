#pragma once
// Problem analysis: properties of a Problem that preprocessing can base
// decisions on (docs/vision.md, goal 2), each with its Evidence.
//
//   ProblemProperties p = analyze(problem, mesh);    // symbolic + sampled
//   ProblemProperties q = analyze(problem);          // symbolic only
//   SystemProperties  s = analyze(assembled_system); // one assembled system
//
// Symbolic judgements (constant coefficients, symbolic zeros, boundary
// classification) are Proven. Variable coefficients are sampled at the
// interior quadrature points of the mesh at AnalysisOptions::sample_times
// (Sampled). Without a mesh, or when sampling fails, the evidence is Unknown.
//
// Analysis reuses the judgements of the solve itself so the two cannot
// disagree: weak_form_coefficients / weak_time_dependence (weak_form.hpp),
// classify_edge_condition and divisor_keeps_strict_sign on
// edge_divisor_samples (boundary_classification.hpp, as in assembly),
// time_order and numerically_symmetric (system.hpp, as in the integrators'
// check() and the ConjugateGradient backend).
//
// Sign convention (problem.hpp): the operator is elliptic when A is negative
// definite, as for the default a_xx = a_yy = -1 (that is, -Laplace u), and
// the reaction c is coercive when c >= 0.
#include "discretization/mesh_validation.hpp"
#include "fem/boundary_classification.hpp"
#include "fem/detail/assembly_edges.hpp"
#include "fem/detail/assembly_elements.hpp"
#include "fem/evidence.hpp"
#include "fem/expression.hpp"
#include "fem/problem.hpp"
#include "fem/quadrature.hpp"
#include "fem/system.hpp"
#include "fem/weak_form.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <locale>
#include <map>
#include <new>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace pde2d::fem {
struct AnalysisOptions {
  // Interior sample points and edge divisor samples; use the assembly's
  // quadrature degree to check exactly what assembly checks.
  int quadrature_degree = 2;
  // Times at which time-dependent coefficients are sampled.
  std::vector<double> sample_times{0.};
};

// Sign of a scalar coefficient over the samples (or exactly, if constant).
enum class Sign {
  Zero,
  Positive,
  Negative,
  NonNegative,
  NonPositive,
  Indefinite
};
// Of the symmetric diffusion matrix A over the samples.
enum class Definiteness {
  Zero,
  PositiveDefinite,
  NegativeDefinite,
  PositiveSemidefinite,
  NegativeSemidefinite,
  Indefinite
};
// Of all edges of one boundary region (EdgeForm per edge). Rejected: the
// condition cannot be classified on some edge (reason holds the message).
enum class BoundaryForm { Dirichlet, Natural, Mixed, Rejected };

inline const char *sign_name(Sign sign) {
  switch (sign) {
  case Sign::Zero:
    return "Zero";
  case Sign::Positive:
    return "Positive";
  case Sign::Negative:
    return "Negative";
  case Sign::NonNegative:
    return "NonNegative";
  case Sign::NonPositive:
    return "NonPositive";
  case Sign::Indefinite:
    return "Indefinite";
  }
  return "Invalid";
}
inline const char *definiteness_name(Definiteness d) {
  switch (d) {
  case Definiteness::Zero:
    return "Zero";
  case Definiteness::PositiveDefinite:
    return "PositiveDefinite";
  case Definiteness::NegativeDefinite:
    return "NegativeDefinite";
  case Definiteness::PositiveSemidefinite:
    return "PositiveSemidefinite";
  case Definiteness::NegativeSemidefinite:
    return "NegativeSemidefinite";
  case Definiteness::Indefinite:
    return "Indefinite";
  }
  return "Invalid";
}
inline const char *boundary_form_name(BoundaryForm form) {
  switch (form) {
  case BoundaryForm::Dirichlet:
    return "Dirichlet";
  case BoundaryForm::Natural:
    return "Natural";
  case BoundaryForm::Mixed:
    return "Mixed";
  case BoundaryForm::Rejected:
    return "Rejected";
  }
  return "Invalid";
}

struct BoundaryRegionProperties {
  std::string region;
  // Edge counts (zero without a mesh).
  std::size_t edges = 0, dirichlet_edges = 0, natural_edges = 0;
  Property<BoundaryForm> form;
  // The Dirichlet alpha / natural flux multiplier keeps one strict sign on
  // every edge, checked exactly as assembly checks it.
  Property<bool> divisor_keeps_sign;
  // t occurs in the (restricted) condition.
  Property<bool> time_dependent;
};

struct ProblemProperties {
  // 2 if m2 is nonzero, else 1 if m1 is nonzero, else 0 (stationary).
  Property<int> time_order;
  Property<Sign> first_mass, second_mass;
  // Weak-form terms (weak::Term) that contain t; assembly re-evaluates these.
  Property<WeakFlags> time_dependent;
  // b - div A == 0: the interior operator is self-adjoint, so stiffness
  // matrices are symmetric (natural boundary terms and Dirichlet lifting
  // keep that).
  Property<bool> symmetric;
  Property<Definiteness> diffusion;
  Property<Sign> reaction;
  // Some boundary edge is Dirichlet.
  Property<bool> has_dirichlet;
  // Constants are in the nullspace of the stationary operator: no Dirichlet
  // edge, c == 0 and no Robin term (alpha == 0 on natural edges). A
  // stationary solve is then singular.
  Property<bool> constant_nullspace;
  // In order of Problem::boundaries and their regions.
  std::vector<BoundaryRegionProperties> boundaries;
};

// Properties of one assembled system (at system.time).
struct SystemProperties {
  Property<int> time_order;
  Property<bool> symmetric_stiffness;
};

namespace analysis_detail {
using Samples = std::vector<EvaluationPoint>;

// Shortest round-trip-free text of a double, independent of the locale.
inline std::string number(double value) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << value;
  return out.str();
}

inline Evidence weakest(Evidence a, Evidence b) {
  return static_cast<int>(a) > static_cast<int>(b) ? a : b;
}

// -1, 0 or 1 when `e` is a real numeric constant after expansion.
inline std::optional<int> constant_sign(const Expr &e) {
  const Expr expanded = SymEngine::expand(e);
  if (!SymEngine::is_a_Number(*expanded.get_basic()))
    return std::nullopt;
  const auto &number =
      SymEngine::down_cast<const SymEngine::Number &>(*expanded.get_basic());
  if (number.is_zero())
    return 0;
  if (number.is_positive())
    return 1;
  if (number.is_negative())
    return -1;
  return std::nullopt; // complex
}

inline Sign sign_of_range(double lo, double hi) {
  if (lo == 0 && hi == 0)
    return Sign::Zero;
  if (lo > 0)
    return Sign::Positive;
  if (hi < 0)
    return Sign::Negative;
  if (lo >= 0)
    return Sign::NonNegative;
  if (hi <= 0)
    return Sign::NonPositive;
  return Sign::Indefinite;
}
inline Definiteness definiteness_of_range(double lo, double hi) {
  switch (sign_of_range(lo, hi)) {
  case Sign::Zero:
    return Definiteness::Zero;
  case Sign::Positive:
    return Definiteness::PositiveDefinite;
  case Sign::Negative:
    return Definiteness::NegativeDefinite;
  case Sign::NonNegative:
    return Definiteness::PositiveSemidefinite;
  case Sign::NonPositive:
    return Definiteness::NegativeSemidefinite;
  case Sign::Indefinite:
    break;
  }
  return Definiteness::Indefinite;
}

// Known to be nonzero somewhere (exactly or at a sample).
inline bool known_nonzero(const Property<Sign> &p) {
  return p.evidence != Evidence::Unknown && p.value != Sign::Zero;
}
inline bool known_zero(const Property<Sign> &p) {
  return p.evidence != Evidence::Unknown && p.value == Sign::Zero;
}

inline std::string sampled_reason(const std::string &what,
                                  std::size_t samples) {
  return what + " sampled at " + std::to_string(samples) + " points";
}

// Sign of one coefficient: exact when constant, else over the samples.
inline Property<Sign> scalar_sign(const Expr &e, const Samples &samples,
                                  const std::string &what) {
  if (const auto s = constant_sign(e))
    return {*s == 0  ? Sign::Zero
            : *s > 0 ? Sign::Positive
                     : Sign::Negative,
            Evidence::Proven, what + " is constant"};
  if (samples.empty())
    return {Sign::Indefinite, Evidence::Unknown,
            what + " is not constant and was not sampled"};
  const Evaluator f(e); // unbound symbols are input errors, as in assembly
  double lo = std::numeric_limits<double>::infinity(), hi = -lo;
  try {
    for (const auto &point : samples) {
      const double v = f(point);
      lo = std::min(lo, v);
      hi = std::max(hi, v);
    }
  } catch (const std::domain_error &error) {
    return {Sign::Indefinite, Evidence::Unknown,
            what + " could not be sampled: " + error.what()};
  }
  return {sign_of_range(lo, hi), Evidence::Sampled,
          sampled_reason(what, samples.size())};
}

inline Property<Definiteness>
diffusion_definiteness(const WeakCoefficients<Expr> &w,
                       const Samples &samples) {
  const auto &axx = w[weak::DiffusionXX], &axy = w[weak::DiffusionXY],
             &ayy = w[weak::DiffusionYY];
  const auto det = constant_sign(axx * ayy - axy * axy),
             trace = constant_sign(axx + ayy);
  if (constant_sign(axx) && constant_sign(axy) && constant_sign(ayy) && det &&
      trace) {
    Definiteness d = Definiteness::Indefinite;
    if (*det > 0)
      d = *trace > 0 ? Definiteness::PositiveDefinite
                     : Definiteness::NegativeDefinite;
    else if (*det == 0)
      d = *trace > 0   ? Definiteness::PositiveSemidefinite
          : *trace < 0 ? Definiteness::NegativeSemidefinite
                       : Definiteness::Zero;
    return {d, Evidence::Proven, "A is constant"};
  }
  if (samples.empty())
    return {Definiteness::Indefinite, Evidence::Unknown,
            "A is not constant and was not sampled"};
  const Evaluator fxx(axx), fxy(axy), fyy(ayy);
  // Extreme eigenvalues of A over all samples.
  double lo = std::numeric_limits<double>::infinity(), hi = -lo;
  try {
    for (const auto &point : samples) {
      const double a = fxx(point), b = fxy(point), d = fyy(point);
      const double centre = (a + d) / 2, radius = std::hypot((a - d) / 2, b);
      lo = std::min(lo, centre - radius);
      hi = std::max(hi, centre + radius);
    }
  } catch (const std::domain_error &error) {
    return {Definiteness::Indefinite, Evidence::Unknown,
            std::string("A could not be sampled: ") + error.what()};
  }
  return {definiteness_of_range(lo, hi), Evidence::Sampled,
          sampled_reason("eigenvalues of A", samples.size())};
}

// Interior quadrature points of every element at every sample time.
inline Samples interior_samples(const discretization::TriangleMesh &mesh,
                                const AnalysisOptions &options) {
  Samples samples;
  const auto rule = triangle_quadrature(options.quadrature_degree);
  for (const auto &element : mesh.elements) {
    const auto a = mesh.nodes[element[0]], b = mesh.nodes[element[1]],
               c = mesh.nodes[element[2]];
    for (const auto &q : rule) {
      const auto &phi = q.barycentric;
      for (double time : options.sample_times)
        samples.push_back({phi[0] * a.x + phi[1] * b.x + phi[2] * c.x,
                           phi[0] * a.y + phi[1] * b.y + phi[2] * c.y, time});
    }
  }
  return samples;
}

inline void validate(const AnalysisOptions &options) {
  for (double time : options.sample_times)
    if (!std::isfinite(time))
      throw std::invalid_argument("Invalid analysis sample time");
  (void)triangle_quadrature(options.quadrature_degree);
}

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

// Everything that does not need boundary edges.
inline ProblemProperties interior_properties(const Equation &equation,
                                             const Samples &samples) {
  validate_interior_equation(equation);
  const auto w = weak_form_coefficients(equation);
  ProblemProperties p;
  p.first_mass = scalar_sign(equation.m1, samples, "m1");
  p.second_mass = scalar_sign(equation.m2, samples, "m2");
  if (known_nonzero(p.second_mass))
    p.time_order = {2, p.second_mass.evidence, "m2 is nonzero"};
  else if (!known_zero(p.second_mass))
    p.time_order = {2, Evidence::Unknown, "m2 is not known to vanish"};
  else if (known_nonzero(p.first_mass))
    p.time_order = {1, weakest(p.first_mass.evidence, p.second_mass.evidence),
                    "m2 vanishes and m1 is nonzero"};
  else if (known_zero(p.first_mass))
    p.time_order = {0, weakest(p.first_mass.evidence, p.second_mass.evidence),
                    "m1 and m2 vanish"};
  else
    p.time_order = {1, Evidence::Unknown, "m1 is not known to vanish"};

  p.time_dependent = {weak_time_dependence(w), Evidence::Proven,
                      "terms whose expression contains t"};

  const auto drift_x = scalar_sign(w[weak::DriftX], samples, "b_x - (div A)_x");
  const auto drift_y = scalar_sign(w[weak::DriftY], samples, "b_y - (div A)_y");
  if (known_zero(drift_x) && known_zero(drift_y))
    p.symmetric = {true, weakest(drift_x.evidence, drift_y.evidence),
                   "b - div A vanishes (" + drift_x.reason + ", " +
                       drift_y.reason + ")"};
  else if (known_nonzero(drift_x) || known_nonzero(drift_y)) {
    const auto &d = known_nonzero(drift_x) ? drift_x : drift_y;
    p.symmetric = {false, d.evidence, "b - div A is nonzero: " + d.reason};
  } else
    p.symmetric = {false, Evidence::Unknown,
                   "b - div A is not known to vanish"};

  p.diffusion = diffusion_definiteness(w, samples);
  p.reaction = scalar_sign(equation.c, samples, "c");
  return p;
}

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
} // namespace analysis_detail

// Symbolic analysis without a mesh: boundary forms are judged from the
// structure of each condition; variable coefficients stay Unknown.
inline ProblemProperties analyze(const Problem &problem) {
  using namespace analysis_detail;
  auto p = interior_properties(problem.equation, {});
  RobinTerms robin;
  bool dirichlet = false, unknown = false;
  for (const auto &assignment : problem.boundaries)
    for (const auto &region : assignment.regions) {
      auto r = region_without_mesh(region, assignment.condition, robin);
      dirichlet = dirichlet || (r.form.value == BoundaryForm::Dirichlet &&
                                r.form.evidence == Evidence::Proven);
      unknown = unknown || r.form.evidence != Evidence::Proven ||
                r.form.value == BoundaryForm::Rejected;
      p.boundaries.push_back(std::move(r));
    }
  if (dirichlet)
    p.has_dirichlet = {true, Evidence::Proven,
                       "a condition has no derivative "
                       "terms"};
  else if (unknown)
    p.has_dirichlet = {false, Evidence::Unknown,
                       "boundary forms depend on the edges"};
  else
    p.has_dirichlet = {false, Evidence::Proven, "every condition is natural"};
  finish_nullspace(p, robin);
  return p;
}

// Analysis on a mesh: boundary conditions are classified edge by edge as
// assembly classifies them, and variable coefficients are sampled. Throws
// the assembly's exceptions for invalid meshes and boundary assignments.
inline ProblemProperties analyze(const Problem &problem,
                                 const discretization::TriangleMesh &mesh,
                                 const AnalysisOptions &options = {}) {
  using namespace analysis_detail;
  validate(options);
  discretization::validate_mesh_geometry(mesh);
  auto p =
      interior_properties(problem.equation, interior_samples(mesh, options));
  const auto assigned = assembly_detail::region_conditions(mesh, problem);
  const auto opposite = assembly_detail::opposite_vertices(mesh);
  const auto edge_samples = edge_divisor_samples(options.quadrature_degree);
  std::vector<RegionAccumulator> regions;
  std::map<std::string, std::size_t> index;
  for (const auto &assignment : problem.boundaries)
    for (const auto &region : assignment.regions) {
      index.emplace(region, regions.size());
      regions.emplace_back(region);
    }
  RobinTerms robin;
  for (std::size_t edge = 0; edge < mesh.boundary_edges.size(); ++edge) {
    const auto &tag = mesh.boundary_tags[edge];
    const auto condition = assigned.find(tag);
    if (condition == assigned.end())
      throw std::invalid_argument("Missing boundary assignment: " + tag);
    const auto [a, b] = mesh.boundary_edges[edge];
    const auto frame =
        p1::edge_frame(mesh.nodes[a], mesh.nodes[b],
                       mesh.nodes[opposite.at(std::minmax(a, b))]);
    auto &region = regions[index.at(tag)];
    try {
      region.add(
          classify_edge_condition(problem.equation, *condition->second, frame),
          frame, edge_samples, options, robin);
    } catch (const std::invalid_argument &error) {
      region.reject(error.what());
    }
  }
  std::size_t dirichlet_edges = 0;
  bool rejected = false;
  for (auto &region : regions) {
    p.boundaries.push_back(
        region.finish(edge_samples.size() * options.sample_times.size()));
    dirichlet_edges += p.boundaries.back().dirichlet_edges;
    rejected =
        rejected || p.boundaries.back().form.value == BoundaryForm::Rejected;
  }
  if (dirichlet_edges > 0)
    p.has_dirichlet = {true, Evidence::Proven,
                       std::to_string(dirichlet_edges) + " Dirichlet edges"};
  else if (rejected)
    p.has_dirichlet = {false, Evidence::Unknown,
                       "some edges could not be classified"};
  else
    p.has_dirichlet = {false, Evidence::Proven,
                       "every boundary edge is "
                       "natural"};
  finish_nullspace(p, robin);
  return p;
}

// One assembled system, with the judgements the integrators and the
// ConjugateGradient backend apply.
inline SystemProperties analyze(const AssembledSystem &system) {
  const std::string at =
      "assembled at t=" + analysis_detail::number(system.time);
  SystemProperties s;
  s.time_order = {time_order(system), Evidence::Sampled, at};
  s.symmetric_stiffness = {numerically_symmetric(system.stiffness),
                           Evidence::Sampled,
                           at + ", relative skew tolerance " +
                               analysis_detail::number(symmetry_tolerance)};
  return s;
}
} // namespace pde2d::fem

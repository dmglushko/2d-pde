#pragma once
// Private part of analysis (analysis.hpp): interior coefficients. Signs and
// definiteness are exact for constants (Evidence::Proven) and otherwise
// sampled at the interior quadrature points of a mesh (Evidence::Sampled).
#include "discretization/geometry_base.hpp"
#include "fem/analysis_types.hpp"
#include "fem/expression.hpp"
#include "fem/problem.hpp"
#include "fem/quadrature.hpp"
#include "fem/weak_form.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::fem::analysis_detail {
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

} // namespace pde2d::fem::analysis_detail

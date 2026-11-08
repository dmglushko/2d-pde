#pragma once
// Result and option types of problem analysis (analysis.hpp): properties of
// a Problem or an assembled system, each a Property<T> with its Evidence.
// Kept apart from the analysis itself so code that only reads or prints
// properties (report.hpp, a future automatic chooser) does not need it.
//
// Sign convention (problem.hpp): the operator is elliptic when A is negative
// definite, as for the default a_xx = a_yy = -1 (that is, -Laplace u), and
// the reaction c is coercive when c >= 0.
#include "fem/boundary_classification.hpp"
#include "fem/evidence.hpp"
#include "fem/p1_element.hpp"
#include "fem/weak_form.hpp"
#include <cstddef>
#include <string>
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

// One boundary edge as region analysis sees it: the classification of its
// condition on the edge (classify_edge_condition) or the message that
// rejected it, and its frame. analyze(problem, mesh) builds these from
// bind_boundary; analyze_boundary_region accepts them directly.
struct ClassifiedEdge {
  std::variant<EdgeCondition, std::string> condition;
  p1::EdgeFrame frame;
};
} // namespace pde2d::fem

#pragma once
// Problem analysis: properties of a Problem that preprocessing can base
// decisions on (docs/vision.md, goal 2), each with its Evidence.
//
//   ProblemProperties p = analyze(problem, mesh);    // symbolic + sampled
//   ProblemProperties q = analyze(problem);          // symbolic only
//   SystemProperties  s = analyze(assembled_system); // one assembled system
//   BoundaryRegionProperties r = analyze_boundary_region(name, edges);
//
// Symbolic judgements (constant coefficients, symbolic zeros, boundary
// classification) are Proven. Variable coefficients are sampled at the
// interior quadrature points of the mesh at AnalysisOptions::sample_times
// (Sampled). Without a mesh, or when sampling fails, the evidence is Unknown.
//
// Analysis reuses the judgements of the solve itself so the two cannot
// disagree: bind_boundary (boundary_assignment.hpp, region binding and edge
// frames), weak_form_coefficients / weak_time_dependence (weak_form.hpp),
// classify_edge_condition and divisor_keeps_strict_sign on
// edge_divisor_samples (boundary_classification.hpp, as in assembly),
// time_order and numerically_symmetric (system.hpp, as in the integrators'
// check() and the ConjugateGradient backend).
//
// The types are in analysis_types.hpp; the interior and region judgements in
// detail/analysis_interior.hpp and detail/analysis_regions.hpp.
#include "discretization/mesh_validation.hpp"
#include "fem/analysis_types.hpp"
#include "fem/boundary_assignment.hpp"
#include "fem/boundary_classification.hpp"
#include "fem/detail/analysis_interior.hpp"
#include "fem/detail/analysis_regions.hpp"
#include "fem/problem.hpp"
#include "fem/system.hpp"
#include <cstddef>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {

// Symbolic analysis without a mesh: boundary forms are judged from the
// structure of each condition; variable coefficients stay Unknown.
inline ProblemProperties analyze(const Problem &problem) {
  using namespace analysis_detail;
  auto p = interior_properties(problem.equation, {});
  RobinTerms robin;
  bool dirichlet = false, unknown = false, oblique = false;
  for (const auto &assignment : problem.boundaries)
    for (const auto &region : assignment.regions) {
      auto r = region_without_mesh(region, problem.equation,
                                   assignment.condition, robin, oblique);
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
  if (oblique && p.symmetric.value)
    p.symmetric = {false, Evidence::Unknown,
                   "a boundary condition may be oblique on some edges "
                   "(nonsymmetric tangential term); analyze with a mesh"};
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
  const auto binding = bind_boundary(mesh, problem);
  const auto edge_samples = edge_divisor_samples(options.quadrature_degree);
  // Classify every edge as assembly does; a rejection is a property here.
  std::vector<std::vector<ClassifiedEdge>> classified(binding.regions.size());
  for (const auto &edge : binding.edges) {
    auto &region = classified[edge.region];
    try {
      region.push_back(
          {classify_edge_condition(binding.equation, binding.condition(edge),
                                   edge.frame),
           edge.frame});
    } catch (const std::invalid_argument &error) {
      region.push_back({std::string(error.what()), edge.frame});
    }
  }
  RobinTerms robin;
  std::size_t tangential_edges = 0;
  std::vector<BoundaryRegionProperties> regions;
  for (std::size_t r = 0; r < binding.regions.size(); ++r)
    regions.push_back(accumulate_region(binding.regions[r], classified[r],
                                        edge_samples, options, robin,
                                        tangential_edges));
  std::size_t dirichlet_edges = 0;
  bool rejected = false;
  for (auto &region : regions) {
    dirichlet_edges += region.dirichlet_edges;
    rejected = rejected || region.form.value == BoundaryForm::Rejected;
    p.boundaries.push_back(std::move(region));
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
  // The tangential term of oblique edges is nonsymmetric (its pair entries
  // differ by the edge mean of tau/beta); not sampled here, hence Unknown.
  if (tangential_edges > 0 && p.symmetric.value)
    p.symmetric = {false, Evidence::Unknown,
                   std::to_string(tangential_edges) +
                       " oblique boundary edges add a nonsymmetric "
                       "tangential-derivative term; interior: " +
                       p.symmetric.reason};
  finish_nullspace(p, robin);
  return p;
}

// The judgement of one boundary region from its classified edges (in edge
// order), exactly as analyze(problem, mesh) forms it for each region.
inline BoundaryRegionProperties
analyze_boundary_region(std::string region,
                        std::span<const ClassifiedEdge> edges,
                        const AnalysisOptions &options = {}) {
  analysis_detail::validate(options);
  analysis_detail::RobinTerms robin;
  std::size_t tangential_edges = 0;
  return analysis_detail::accumulate_region(
      std::move(region), edges, edge_divisor_samples(options.quadrature_degree),
      options, robin, tangential_edges);
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

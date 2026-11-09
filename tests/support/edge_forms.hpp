#pragma once
// Edge forms per boundary region, from the public binding and classification
// (bind_boundary, classify_edge_condition): how many edges of each assigned
// region classification reduces to each EdgeForm alternative. Throws what
// classification throws for an edge it rejects.
#include "fem/boundary_assignment.hpp"
#include "fem/boundary_classification.hpp"
#include <cstddef>
#include <string>
#include <variant>
#include <vector>

namespace pde2d_test {
struct RegionForms {
  std::string region;
  std::size_t edges = 0, dirichlet = 0, natural = 0, oblique = 0;
};
// One entry per assigned region, in bind_boundary order.
inline std::vector<RegionForms>
region_forms(const pde2d::discretization::TriangleMesh &mesh,
             const pde2d::fem::Problem &problem) {
  using namespace pde2d::fem;
  const auto binding = bind_boundary(mesh, problem);
  std::vector<RegionForms> regions;
  for (const auto &name : binding.regions)
    regions.push_back({name});
  for (const auto &edge : binding.edges) {
    auto &r = regions[edge.region];
    ++r.edges;
    const auto form = classify_edge_condition(
        binding.equation, binding.condition(edge), edge.frame);
    if (std::holds_alternative<DirichletEdge>(form.form))
      ++r.dirichlet;
    else if (std::holds_alternative<NaturalEdge>(form.form))
      ++r.natural;
    else if (std::holds_alternative<ObliqueEdge>(form.form))
      ++r.oblique;
  }
  return regions;
}
} // namespace pde2d_test

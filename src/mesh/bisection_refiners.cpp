// The three bisection refiners. Each is its rule (edge_bisection.hpp) and
// its options; the work is in edge_bisection.cpp.

#include "edge_bisection.hpp"

#include "pde2d/mesh/longest_edge_bisection.hpp"
#include "pde2d/mesh/newest_vertex_bisection.hpp"
#include "pde2d/mesh/red_green_blue_refinement.hpp"

#include <stdexcept>
#include <string>

namespace pde2d::mesh {

namespace {

using Rule = detail::BisectionRule;

void check_max_points(std::size_t max_points, const std::string &options) {
  if (max_points == 0)
    throw std::invalid_argument(options + "::max_points must be positive");
}

// A marked triangle: its longest edge; first bisection: the longest edge.
constexpr Rule longest_edge_rule{.marks = Rule::Marks::longest_edge,
                                 .first = Rule::FirstBisection::longest_edge,
                                 .red = false};
// A marked triangle: its refinement edge; first bisection: there.
constexpr Rule newest_vertex_rule{.marks = Rule::Marks::refinement_edge,
                                  .first =
                                      Rule::FirstBisection::refinement_edge,
                                  .red = false};
// A marked triangle: all three edges, red where all three are split.
constexpr Rule red_green_blue_rule{.marks = Rule::Marks::all_edges,
                                   .first =
                                       Rule::FirstBisection::refinement_edge,
                                   .red = true};

} // namespace

// --- LongestEdgeBisection ---------------------------------------------------

LongestEdgeBisection::LongestEdgeBisection(LongestEdgeBisectionOptions options)
    : options_(options) {
  check_max_points(options_.max_points, "LongestEdgeBisectionOptions");
}

void LongestEdgeBisection::run(
    WorkingMesh &mesh, const MeshSpec &spec,
    std::span<const LocalImprover *const> local_improvers,
    StageReport &report) const {
  detail::EdgeBisection(name(), longest_edge_rule, options_.max_points)
      .run(mesh, spec, local_improvers, report);
}

std::optional<Parents> LongestEdgeBisection::refine(
    WorkingMesh &mesh, std::span<const std::size_t> marked_triangles,
    std::span<const LocalImprover *const> local_improvers,
    StageReport &report) const {
  return detail::EdgeBisection(name(), longest_edge_rule, options_.max_points)
      .refine(mesh, marked_triangles, local_improvers, report);
}

// --- NewestVertexBisection --------------------------------------------------

NewestVertexBisection::NewestVertexBisection(
    NewestVertexBisectionOptions options)
    : options_(options) {
  check_max_points(options_.max_points, "NewestVertexBisectionOptions");
}

void NewestVertexBisection::run(
    WorkingMesh &mesh, const MeshSpec &spec,
    std::span<const LocalImprover *const> local_improvers,
    StageReport &report) const {
  detail::EdgeBisection(name(), newest_vertex_rule, options_.max_points)
      .run(mesh, spec, local_improvers, report);
}

std::optional<Parents> NewestVertexBisection::refine(
    WorkingMesh &mesh, std::span<const std::size_t> marked_triangles,
    std::span<const LocalImprover *const> local_improvers,
    StageReport &report) const {
  return detail::EdgeBisection(name(), newest_vertex_rule, options_.max_points)
      .refine(mesh, marked_triangles, local_improvers, report);
}

// --- RedGreenBlueRefinement -------------------------------------------------

RedGreenBlueRefinement::RedGreenBlueRefinement(
    RedGreenBlueRefinementOptions options)
    : options_(options) {
  check_max_points(options_.max_points, "RedGreenBlueRefinementOptions");
}

void RedGreenBlueRefinement::run(
    WorkingMesh &mesh, const MeshSpec &spec,
    std::span<const LocalImprover *const> local_improvers,
    StageReport &report) const {
  detail::EdgeBisection(name(), red_green_blue_rule, options_.max_points)
      .run(mesh, spec, local_improvers, report);
}

std::optional<Parents> RedGreenBlueRefinement::refine(
    WorkingMesh &mesh, std::span<const std::size_t> marked_triangles,
    std::span<const LocalImprover *const> local_improvers,
    StageReport &report) const {
  return detail::EdgeBisection(name(), red_green_blue_rule, options_.max_points)
      .refine(mesh, marked_triangles, local_improvers, report);
}

} // namespace pde2d::mesh

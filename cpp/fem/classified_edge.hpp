#pragma once
// The input of region analysis (analyze_boundary_region, analysis.hpp): one
// boundary edge with the classification of its condition. Kept apart from
// the analysis result types (analysis_types.hpp) because it carries the
// symbolic edge forms of boundary_classification.hpp.
#include "fem/boundary_classification.hpp"
#include "fem/p1_element.hpp"
#include <string>
#include <variant>

namespace pde2d::fem {
// One boundary edge as region analysis sees it: the classification of its
// condition on the edge (classify_edge_condition) or the message that
// rejected it, and its frame. analyze(problem, mesh) builds these from
// bind_boundary; analyze_boundary_region accepts them directly.
struct ClassifiedEdge {
  std::variant<EdgeCondition, std::string> condition;
  p1::EdgeFrame frame;
};
} // namespace pde2d::fem

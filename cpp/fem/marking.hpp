#pragma once
// Marking strategies of adaptive refinement: which elements to refine, from
// the indicators of an ErrorEstimate (estimation.hpp). The result feeds
// discretization::refine_mesh(result, marked, options).
//
//   BulkMarking dorfler(0.6);                       // default strategy
//   auto marked = mark_elements(dorfler, estimate, mesh);
//
// A strategy only decides; it does not refine. A user strategy implements
// MarkingStrategy (name(), descriptor(), mark()) and needs no library
// change; the adaptive driver (adaptivity.hpp) records its name and
// descriptor in every iteration's "marking" decision.
#include "discretization/mesh_validation.hpp"
#include "fem/estimation.hpp"
#include "provenance.hpp"
#include <algorithm>
#include <cmath>
#include <format>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::fem {
class MarkingStrategy {
public:
  virtual ~MarkingStrategy() = default;
  // Report name, e.g. "bulk (Dorfler)".
  virtual std::string name() const = 0;
  // Stable description (name + parameters) for records. Without one the
  // strategy is "not replayable" from a record.
  virtual std::optional<pde2d::StrategyDescriptor> descriptor() const {
    return std::nullopt;
  }
  // Element indices (into mesh.elements) to refine. `estimate` has one
  // indicator per element (estimate_error checked it). Must be
  // deterministic.
  virtual std::vector<std::size_t>
  mark(const ErrorEstimate &estimate,
       const discretization::TriangleMesh &mesh) const = 0;
};

// Dorfler (bulk) marking with mark_bulk: the smallest set of elements, in
// decreasing indicator order, whose squared indicators reach `fraction` of
// the total. The default strategy of adapt().
class BulkMarking final : public MarkingStrategy {
public:
  double fraction = 0.5;
  BulkMarking() = default;
  explicit BulkMarking(double fraction) : fraction(fraction) {}
  std::string name() const override { return "bulk (Dorfler)"; }
  std::optional<pde2d::StrategyDescriptor> descriptor() const override {
    return pde2d::StrategyDescriptor{
        "Bulk", {{"fraction", std::format("{}", fraction)}}};
  }
  std::vector<std::size_t>
  mark(const ErrorEstimate &estimate,
       const discretization::TriangleMesh &) const override {
    return mark_bulk(estimate.indicators, fraction);
  }
};

// Runs a strategy and checks its result: ascending, distinct indices of
// elements of `mesh` (std::runtime_error "Marking strategy '<name>' returned
// invalid element indices" otherwise).
inline std::vector<std::size_t>
mark_elements(const MarkingStrategy &strategy, const ErrorEstimate &estimate,
              const discretization::TriangleMesh &mesh) {
  if (estimate.indicators.size() != mesh.elements.size())
    throw std::invalid_argument(
        "Marking needs one error indicator per element");
  auto marked = strategy.mark(estimate, mesh);
  for (std::size_t k = 0; k < marked.size(); ++k)
    if (marked[k] >= mesh.elements.size() ||
        (k > 0 && marked[k] <= marked[k - 1]))
      throw std::runtime_error("Marking strategy '" + strategy.name() +
                               "' returned invalid element indices");
  return marked;
}
} // namespace pde2d::fem

#pragma once

// Private to the mesh library.

#include "pde2d/mesh/components.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace pde2d::mesh::detail {

// Calls a refiner's local improvers after each change and counts what
// they did, for the refiner's report.
class LocalImproverCalls {
public:
  explicit LocalImproverCalls(std::span<const LocalImprover *const> improvers)
      : improvers_(improvers), counts_(improvers.size(), 0) {}

  // Calls every local improver, in order, on the touched triangles.
  void after_change(WorkingMesh &mesh, std::span<const std::size_t> touched) {
    for (std::size_t i = 0; i < improvers_.size(); ++i)
      counts_[i] += improvers_[i]->improve_around(mesh, touched);
  }

  // The changes made so far, by all of them.
  std::size_t total() const {
    std::size_t sum = 0;
    for (auto c : counts_)
      sum += c;
    return sum;
  }

  // "local_improver_changes" (the sum) and one
  // "local_improver_changes.<name>" per local improver.
  void record(StageReport &report) const {
    std::size_t total = 0;
    for (std::size_t i = 0; i < improvers_.size(); ++i) {
      total += counts_[i];
      report.metrics["local_improver_changes." + improvers_[i]->name()] +=
          static_cast<double>(counts_[i]);
    }
    report.metrics["local_improver_changes"] = static_cast<double>(total);
  }

private:
  std::span<const LocalImprover *const> improvers_;
  std::vector<std::size_t> counts_;
};

} // namespace pde2d::mesh::detail

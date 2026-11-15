#pragma once

#include "pde2d/mesh/mesh.hpp"

#include <cstddef>
#include <span>
#include <string>
#include <vector>

// Marking: the second of the three parts of adaptive refinement
// (estimate -> mark -> refine). The error estimate is the owner's; it
// gives one indicator per triangle. A marking strategy picks the triangles
// to refine from those indicators.
namespace pde2d::adapt {

// The common interface of the marking strategies. Each strategy owns its
// options struct, set with designated initializers in its constructor and
// validated there (std::invalid_argument).
//
// mark() is not virtual: it checks the indicators and applies the rules
// every strategy shares, then asks the strategy (select()) which triangles
// it wants. So the shared rules are written once and no strategy can skip
// them.
class MarkingStrategy {
public:
  virtual ~MarkingStrategy() = default;
  virtual std::string name() const = 0;

  // The marked triangles of `mesh`, in increasing index order, without
  // repetitions. `indicators` holds one error indicator per triangle
  // (indicators[t] belongs to mesh.triangles[t]); each must be finite and
  // non-negative. Throws std::invalid_argument for a wrong length or a bad
  // value, naming the triangle. A triangle whose indicator is zero is
  // never marked, so all zeros mark nothing. Deterministic: equal
  // indicators are ordered by triangle index.
  [[nodiscard]] std::vector<std::size_t>
  mark(const mesh::Mesh &mesh, std::span<const double> indicators) const;

private:
  // Called by mark() with valid indicators, at least one of them
  // positive. Returns the triangles the strategy picks, in any order and
  // possibly with zero indicators; mark() drops those and sorts.
  virtual std::vector<std::size_t>
  select(std::span<const double> indicators) const = 0;
};

} // namespace pde2d::adapt

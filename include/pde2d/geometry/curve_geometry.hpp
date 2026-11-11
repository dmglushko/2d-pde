#pragma once

#include "pde2d/geometry/curve.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pde2d::geometry {

// One exact curve of a loop and the boundary region it belongs to.
struct BoundaryCurve {
  Curve curve;
  std::string region;
};

// A closed loop: curve i ends where curve i + 1 starts, and the last curve
// ends where the first starts. This is what the user builds.
struct Loop {
  std::vector<BoundaryCurve> curves;
};

// Region names use ASCII letters, digits, '_', '.' and '-', and must not be
// empty or a lone "-" (reserved by the .pde2d file format).
bool is_valid_region_name(std::string_view name);

// A domain described by closed loops of exact curves.
//
// The constructor validates the loops and computes, from containment, each
// loop's depth (how many other loops contain it) and parent (the innermost
// loop that contains it). A loop of even depth is the outer boundary of a
// piece of the domain, a loop of odd depth is a hole; a loop of depth 2 is
// an island inside a hole. The loops are then normalized: even depth
// counterclockwise, odd depth clockwise, so the domain is always on the
// left of every curve. Loops keep their input order; a loop that had the
// wrong direction has its curves reversed (and their order with them).
//
// Invariants: every loop is closed, no loop intersects or touches itself
// or another loop, every curve has a valid region name.
class CurveGeometry {
public:
  // Throws std::invalid_argument with the loop and curve index when the
  // input is invalid.
  explicit CurveGeometry(std::vector<Loop> loops);

  const std::vector<Loop> &loops() const { return loops_; }
  std::size_t depth(std::size_t loop) const;
  // The innermost loop containing this one; empty for depth 0.
  std::optional<std::size_t> parent(std::size_t loop) const;

  // Distinct region names, in order of first appearance along the stored
  // (normalized) loops.
  std::vector<std::string> region_names() const;
  double region_length(const std::string &region) const;

  // Region editing. Each one keeps the invariants above and region names
  // unique, or throws std::invalid_argument and leaves the geometry
  // unchanged.
  //
  // Gives region `from` the new name `to`, which must not exist yet.
  void rename_region(const std::string &from, const std::string &to);
  // Moves every curve of `source` into the existing region `target`.
  void merge_region_into(const std::string &source, const std::string &target);
  // Splits a connected region in two at the given fraction (strictly
  // between 0 and 1) of its length, measured along the normalized loop
  // direction from the start of the region. A region that covers a whole
  // loop starts at the start of the loop's first curve. A curve that
  // contains the split point is split into two curves. Each new name must
  // be the old name or a name that does not exist yet.
  void split_region(const std::string &region, double fraction,
                    const std::string &first, const std::string &second);

  // The same geometry moved by a similarity transform.
  CurveGeometry transformed(const CurveTransform &transform) const;

private:
  std::vector<Loop> loops_;
  // Computed from all loops together, so kept apart from the input.
  std::vector<std::size_t> depth_;
  std::vector<std::optional<std::size_t>> parent_;
};

} // namespace pde2d::geometry

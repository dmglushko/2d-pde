#pragma once
// Size criteria of mesh refinement: what decides that an edge is too long
// and which failing edge is split first. Every refinement (constant
// max_edge_length, a SizeField, marked refinement, a user criterion) runs
// one generic loop that splits the highest-priority failing edge at its
// midpoint until no edge fails, the step budget runs out or coordinates stop
// resolving.
//
//   RefinementCriterion c;
//   c.size = [](Point2 a, Point2 b) -> std::optional<SizePriority> {
//     double d = std::hypot(b.x - a.x, b.y - a.y), h = my_target(a, b);
//     if (d > h) return SizePriority{0, d / h, d};
//     return std::nullopt;             // the edge meets the criterion
//   };
//   c.descriptor = pde2d::StrategyDescriptor{"MyCriterion", {{"h", "0.1"}}};
//   options.refinement.criterion = c; // generate_mesh / refine_mesh
//
// A user criterion replaces RefinementOptions::max_edge_length and
// size_field (setting both throws); combine the built-in targets with your
// own through constant_size_criterion, size_field_criterion and
// strictest_of. Like the size field, a criterion is a request-time option:
// .pde2d files do not store it.
#include "geometry_base.hpp"
#include "provenance.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::discretization {
namespace criterion_detail {
// Euclidean length of the edge (a, b), as the refinement loop measures it.
inline double edge_length(Point2 a, Point2 b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
} // namespace criterion_detail

// Refinement priority of an edge that fails a size criterion: higher tiers
// go first, then larger values, then longer edges; equal priorities go to
// the first lexicographic edge. The constant target uses {0, length,
// length}. A size field uses {0, length / target, length}: for a constant
// field the length tie-break restores exactly the constant-target order,
// even where the division rounds two lengths to one ratio. Tiers 1 and 2
// are used by marked refinement (selected boundary and interior edges); a
// user criterion normally uses tier 0.
struct SizePriority {
  int tier = 0;
  double value = 0;
  double length = 0;
  bool operator==(const SizePriority &) const = default;
  bool operator<(const SizePriority &b) const {
    if (tier != b.tier)
      return tier < b.tier;
    if (value != b.value)
      return value < b.value;
    return length < b.length;
  }
};
// Every size criterion plugs into the refinement loop through this one
// interface: nullopt means the edge between the two points meets it,
// otherwise its splitting priority. It must be deterministic (a function of
// the two points only): the loop re-evaluates edges as the mesh changes.
using SizeCriterion =
    std::function<std::optional<SizePriority>(Point2, Point2)>;

// Local target edge length at a point, for spatially graded meshes. An edge
// (a, b) meets the field when its length is at most
// min(h(a), h((a+b)/2), h(b)); values must be finite and positive. A size
// field is a request-time option: .pde2d files do not store it (a callable
// cannot be serialized), so an imported result has an empty field.
using SizeField = std::function<double(Point2)>;

// A complete refinement criterion: the size test plus what the generic loop
// reports and records about it.
struct RefinementCriterion {
  SizeCriterion size;
  // Report message when a Fixed boundary edge fails the criterion (the
  // request is then impossible; MeshReport::size_satisfied is false).
  std::string fixed_conflict = "Fixed boundary edge fails the refinement "
                               "criterion";
  // Called with the endpoints of every edge the loop splits.
  std::function<void(Point2, Point2)> split_hook{};
  // What the criterion is, for run records and mesh plans (a callable cannot
  // be compared or serialized). Without one a user criterion is recorded as
  // "not replayable".
  std::optional<pde2d::StrategyDescriptor> descriptor{};
  bool active() const { return static_cast<bool>(size); }
};

// Constant target h: edges longer than h (times 1 + 1e-12) fail, ranked by
// length. Zero or negative h gives an empty (inactive) criterion.
inline SizeCriterion constant_size_criterion(double h) {
  if (!(h > 0))
    return {};
  return [h](Point2 a, Point2 b) -> std::optional<SizePriority> {
    double d = criterion_detail::edge_length(a, b);
    if (d > h * (1 + 1e-12))
      return SizePriority{0, d, d};
    return std::nullopt;
  };
}

// h(p) of a size field, checked finite and positive.
inline double size_field_value(const SizeField &field, Point2 p) {
  const double h = field(p);
  if (!std::isfinite(h) || !(h > 0))
    throw std::invalid_argument("Size field must return a finite positive "
                                "length");
  return h;
}
// Local target of the edge (a, b): min(h(a), h(midpoint), h(b)), further
// limited by a positive constant max_edge_length.
inline double local_size_target(const SizeField &field, double max_edge_length,
                                Point2 a, Point2 b) {
  Point2 mid{std::midpoint(a.x, b.x), std::midpoint(a.y, b.y)};
  double target =
      std::min({size_field_value(field, a), size_field_value(field, mid),
                size_field_value(field, b)});
  if (max_edge_length > 0)
    target = std::min(target, max_edge_length);
  return target;
}
// A size field (and an optional constant limit): edges longer than their
// local target fail, ranked by length relative to the target (ties by
// length), so a constant field gives exactly the constant-target order.
inline SizeCriterion size_field_criterion(SizeField field,
                                          double max_edge_length = 0) {
  if (!field)
    return constant_size_criterion(max_edge_length);
  return [field = std::move(field),
          max_edge_length](Point2 a, Point2 b) -> std::optional<SizePriority> {
    const double target = local_size_target(field, max_edge_length, a, b),
                 d = criterion_detail::edge_length(a, b);
    if (d > target * (1 + 1e-12))
      return SizePriority{0, d / target, d};
    return std::nullopt;
  };
}
// An edge meets both criteria or fails with the higher of their priorities.
// An empty criterion accepts every edge.
inline SizeCriterion strictest_of(SizeCriterion first, SizeCriterion second) {
  if (!first)
    return second;
  if (!second)
    return first;
  return [first = std::move(first), second = std::move(second)](
             Point2 a, Point2 b) -> std::optional<SizePriority> {
    auto p = first(a, b), q = second(a, b);
    if (!p)
      return q;
    if (!q)
      return p;
    return *p < *q ? q : p;
  };
}
} // namespace pde2d::discretization

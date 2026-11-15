#pragma once

// Private to the mesh library: the edge bisection shared by the three
// bisection refiners (LongestEdgeBisection, NewestVertexBisection,
// RedGreenBlueRefinement). They differ only in the rule below; the rest
// (marked edges, the closure that keeps the mesh conforming, the division
// of the triangles, boundary points from the boundary split policy, the
// parents, the "until every edge is at most h" loop) is written once here.

#include "pde2d/mesh/components.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <string>

namespace pde2d::mesh::detail {

// The per-method part.
struct BisectionRule {
  // Which edges of a marked triangle are marked.
  enum class Marks {
    refinement_edge, // edge 0 (newest vertex bisection)
    longest_edge,    // its longest edge (longest-edge bisection)
    all_edges        // all three (red-green-blue)
  };
  // Where a triangle is bisected that no bisection of the current step
  // made (an input triangle of the step, or one the boundary split policy
  // re-triangulated). A triangle a bisection made is always bisected at
  // its edge 0, opposite the new node, which the bisection puts first.
  enum class FirstBisection {
    refinement_edge, // edge 0, as the mesh stores it
    longest_edge     // the longest edge, measured at that moment
  };
  Marks marks = Marks::refinement_edge;
  FirstBisection first = FirstBisection::refinement_edge;
  // A triangle whose three edges were all halved gets the four red
  // children (similar to it) instead of the bisection pattern.
  bool red = false;
};

// One bisection refiner: its name (for reports and messages), its rule and
// its point budget.
class EdgeBisection {
public:
  EdgeBisection(std::string name, BisectionRule rule, std::size_t max_points)
      : name_(std::move(name)), rule_(rule), max_points_(max_points) {}

  // Refiner::refine. One step:
  //  1. mark the edges of the marked triangles (the rule);
  //  2. split every marked edge, conforming: an edge is split only once
  //     it is the bisection edge of the triangles on both its sides; a
  //     triangle on one side that must first be bisected at another edge
  //     has that edge split first (recursively). This is the closure: no
  //     node ever hangs. Interior edges are split at their midpoint,
  //     boundary edges by WorkingMesh::split_boundary_edge (the boundary
  //     split policy: the curve point of a linked edge, the chord midpoint
  //     otherwise). The new node becomes vertex 0 of every triangle the
  //     split makes, so each child's refinement edge is an edge of its
  //     parent;
  //  3. with rule.red, the triangles whose three edges were all split get
  //     the red division (one flip of the bisection pattern);
  //  4. the local improvers run once on every triangle the step touched.
  // If the triangles waiting on an edge form a cycle (refinement edges
  // that are not compatible, which bisection itself never makes from
  // longest-edge refinement edges), the edge is split anyway: the mesh
  // stays conforming, but those triangles are not divided by the bisection
  // pattern; the metric "incompatible_splits" counts this. An unsplittable
  // boundary edge leaves the triangles waiting on it unrefined (report
  // message). Parents are kept unless the boundary split policy
  // re-triangulates.
  std::optional<Parents>
  refine(WorkingMesh &mesh, std::span<const std::size_t> marked,
         std::span<const LocalImprover *const> local_improvers,
         StageReport &report) const;

  // Refiner::run: refine() repeated on the triangles that have an edge
  // longer than max_edge_length (resolved against domain_size(mesh)),
  // until there are none or a step splits nothing.
  void run(WorkingMesh &mesh, const MeshSpec &spec,
           std::span<const LocalImprover *const> local_improvers,
           StageReport &report) const;

private:
  std::string name_;
  BisectionRule rule_;
  std::size_t max_points_;
};

// Throws std::invalid_argument, naming the refiner, for a marked index out
// of range or of a deleted triangle. Shared by every refiner's refine().
void check_marked(const WorkingMesh &mesh, std::span<const std::size_t> marked,
                  const std::string &refiner);

} // namespace pde2d::mesh::detail

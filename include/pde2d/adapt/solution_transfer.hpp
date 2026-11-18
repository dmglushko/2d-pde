#pragma once

#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/solve/solution.hpp"

namespace pde2d::adapt {

// A solution moved to a new mesh, and how.
struct SolutionTransfer {
  // On the new problem's mesh (the same object), at the old solution's
  // time, with velocities when the old one had them.
  solve::Solution solution{};
  // "solution_transfer": the time taken; the metrics nodes, by_parents
  // (1 or 0), kept_nodes and edge_nodes (with parents: new nodes that are
  // old nodes, new nodes on an old edge), located (without parents: nodes
  // inside an old triangle), nearest_boundary (without parents: nodes
  // given the value at their nearest old boundary point) and
  // dirichlet_nodes; a message saying which way it went.
  mesh::StageReport report{};
};

// Moves a P1 solution to a new mesh by interpolation (docs/decisions.md,
// "Solution transfer is interpolation"): every node of the new mesh gets
// the old u_h at its position, the velocities (time order 2) the same way,
// and the Dirichlet nodes of `problem` their boundary data at the
// solution's time instead (dirichlet_value, and dirichlet_rate for the
// velocities).
//
// With `parents` (Remesher::refine's, from old to new mesh): a node with
// parent {v, v} keeps old node v's value exactly; a node with parent
// {a, b} gets (u[a] + u[b]) / 2. That is the old u_h at the edge's
// midpoint, also for a node on a curved boundary edge, which lies on the
// curve off the old chord, outside the old mesh on a convex curve (where
// u_h has no value) or inside it on a concave one: it gets the value at
// the chord point it was split from, its nearest point of the old
// boundary to second order (exactly on a circle). So a refined boundary
// carries the old boundary trace and never an extrapolated value.
//
// Without parents (an improver flipped or moved nodes, or the mesh was
// not made by refinement): by point location in the old mesh, a bucket
// grid with exact containment (no walk, so GitHub issue #22 does not
// arise). A boundary node of the new mesh gets the old u_h at its
// nearest point of the old boundary, as with parents; another node gets
// the old u_h of the old triangle that contains it (the lowest index
// when it lies on an edge or node), or, if none does (a node moved
// outside the old mesh near a curved boundary), at its nearest point of
// the old boundary. A node at an old node's position keeps its value
// exactly.
//
// Interpolation never leaves the range of the old values on a triangle or
// boundary edge: no new extremes. On a refinement (with or without
// parents) a P1 function is reproduced: the new u_h equals the old one
// inside the old mesh.
//
// Throws std::invalid_argument when the old solution has no mesh, values
// or velocities of the wrong size or not finite, or parents that do not
// fit the two meshes (sizes, a node parent {a, b} with a > b or an index
// out of range), and passes on dirichlet_value's and dirichlet_rate's
// exceptions.
[[nodiscard]] SolutionTransfer
transfer_solution(const solve::Solution &old, const fem::MeshedProblem &problem,
                  const mesh::Parents *parents = nullptr);

} // namespace pde2d::adapt

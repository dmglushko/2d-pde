#pragma once

#include "pde2d/fem/assembled_problem.hpp"
#include "pde2d/fem/meshed_problem.hpp"

#include <memory>

namespace pde2d::fem {

// Assembles a MeshedProblem at a time t into an AssembledProblem with P1
// elements (docs/decisions.md, "Assembly"):
//
// - one sparsity pattern for the free columns and one for the Dirichlet
//   columns, from one loop over the triangles; every matrix shares them;
// - element blocks in full (3 x 3 per triangle, 2 x 2 per boundary edge),
//   with a 3-point rule on triangles (exact for degree 2) and 2-point
//   Gauss-Legendre on edges; one pass over the triangles computes the
//   blocks of every matrix that is built;
// - natural and dynamic conditions are solved for u_n on each straight
//   edge and substituted into (A grad u).n (this needs beta_n plus the
//   normal part of (beta_x, beta_y) to be nonzero);
// - Dirichlet nodes are eliminated while scattering: their rows are
//   skipped and their columns go to the operators' `dirichlet` matrices;
// - triangles (and, in their own pass, boundary edges) are coloured
//   greedily in index order so that no two of one colour share a node,
//   and are processed colour by colour, in index order inside a colour.
//   This slice runs in one thread; the result never depends on that.
//
// `max_time_order` says which time terms are built (docs/decisions.md,
// "assemble takes a max_time_order"):
// - 2: everything;
// - 1: everything but M2 and U_D'';
// - 0: only A (K, C, R, S, T) and the load, with the lifting
//   load - A_FD U_D: M2, M1 and D are left out and only U_D is evaluated
//   (never U_D' or U_D''), so a static solve does not need derivatives
//   of the boundary data that it drops.
// An operator left out is a zero operator (is_zero), so time_order() is at
// most max_time_order.
//
// With `previous` (an earlier result for the same MeshedProblem object,
// by pointer identity), the patterns, the colours and every constant part
// are reused and only the terms whose coefficients depend on t are built
// again; the result is bit for bit the same as without `previous`. An
// operator that `previous` did not build (it was assembled at a lower
// max_time_order) is built here in full.
//
// Throws std::invalid_argument for a null problem, a t that is not
// finite, a max_time_order other than 0, 1 or 2, a `previous` built from
// another MeshedProblem object, a
// coefficient that is not finite at a quadrature point (naming it), and a
// natural or dynamic condition whose beta_n + (beta_x, beta_y).n is zero
// at a quadrature point of one of its edges (it cannot be solved for u_n).
//
// The report (stage "assembly") has the time and the metrics free_nodes,
// dirichlet_nodes, free_nonzeros, dirichlet_nonzeros, triangle_colours,
// edge_colours, used_previous (0 or 1), max_time_order and, for mass,
// damping, stiffness and load, <name>_rebuilt: 1 when something of it was
// computed in this call, 0 when it was reused from `previous` as a whole
// or left out by max_time_order.
class Assembler {
public:
  // No options in this slice.
  Assembler() = default;

  [[nodiscard]] AssembledProblem
  assemble(std::shared_ptr<const MeshedProblem> meshed_problem, double t,
           const AssembledProblem *previous = nullptr,
           int max_time_order = 2) const;
};

} // namespace pde2d::fem

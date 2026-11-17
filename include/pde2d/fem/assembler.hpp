#pragma once

#include "pde2d/fem/assembled_problem.hpp"
#include "pde2d/fem/meshed_problem.hpp"

#include <cstddef>
#include <memory>

namespace pde2d::fem {

struct AssemblerOptions {
  // How many threads compute the elements of one colour at once: 1 runs
  // in the calling thread without OpenMP, 0 means all hardware threads
  // (the processors available to this process, omp_get_num_procs; not
  // OMP_NUM_THREADS). At most 1024. The result is bit for bit the same
  // for every count.
  std::size_t threads = 1;
};

// Assembles a MeshedProblem at a time t into an AssembledProblem with P1
// elements (docs/decisions.md, "Assembly"):
//
// - one sparsity pattern for the free columns and one for the Dirichlet
//   columns, from one loop over the triangles; every matrix shares them;
// - element blocks in full (3 x 3 per triangle, 2 x 2 per boundary edge),
//   with the rules of quadrature.hpp: exact degree 2 on triangles (3
//   points) and 3 on edges (2-point Gauss-Legendre); one pass over the
//   triangles computes the
//   blocks of every matrix that is built;
// - natural and dynamic conditions are solved for u_n on each straight
//   edge and substituted into (A grad u).n (this needs beta_n plus the
//   normal part of (beta_x, beta_y) to be nonzero);
// - Dirichlet nodes are eliminated while scattering: their rows are
//   skipped and their columns go to the operators' `dirichlet` matrices;
// - triangles (and, in their own pass, boundary edges) are coloured
//   greedily in index order so that no two of one colour share a node,
//   and are processed colour by colour. The elements of one colour are
//   shared among AssemblerOptions::threads threads (OpenMP); they share no
//   node, so every matrix slot and load entry receives at most one
//   addition per colour, and the additions to it come in colour order
//   whatever the thread count: the result is bit for bit the same for
//   every count. Called from inside an OpenMP parallel region, it may
//   run in one thread (nested parallelism is off by default); the result
//   is the same.
//
// `max_time_order` says which time terms are built (docs/decisions.md,
// "assemble takes a max_time_order"):
// - 2: everything;
// - 1: everything but M2 and U_D'';
// - 0: only A (K, C, R, S, T) and the load, with the lifting
//   load - A_FD U_D: M2, M1 and D are left out and only U_D is evaluated
//   (never U_D' or U_D''), so a static solve does not need derivatives
//   of the boundary data that it drops.
// An operator left out is a zero operator (always_zero), so time_order() is at
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
// With several threads, the elements of a colour are all computed before
// an error is passed on, and it is the error of the element the serial
// order meets first (the lowest index in the first colour that has one),
// so the exception and its message do not depend on the thread count.
//
// The report (stage "assembly") has the time and the metrics free_nodes,
// dirichlet_nodes, free_nonzeros, dirichlet_nonzeros, triangle_colours,
// edge_colours, used_previous (0 or 1), max_time_order, threads (the
// count used, 0 resolved to the hardware threads) and, for mass,
// damping, stiffness and load, <name>_rebuilt: 1 when something of it was
// computed in this call, 0 when it was reused from `previous` as a whole
// or left out by max_time_order.
class Assembler {
public:
  // Throws std::invalid_argument for more than 1024 threads.
  explicit Assembler(AssemblerOptions options = {});
  const AssemblerOptions &options() const { return options_; }

  [[nodiscard]] AssembledProblem
  assemble(std::shared_ptr<const MeshedProblem> meshed_problem, double t,
           const AssembledProblem *previous = nullptr,
           int max_time_order = 2) const;

  // The time order that assemble(meshed_problem, t) gives (with the
  // default max_time_order 2) at any t, from the formula facts alone:
  // no coefficient is evaluated, so it can be asked before anything is
  // assembled (d/dt sqrt(t) is infinite at t = 0). Like
  // AssembledProblem::time_order, it can be lower than the order as
  // written (fem::time_order(MeshedProblem)), when a boundary u_t term
  // vanishes because A is zero.
  [[nodiscard]] int time_order(const MeshedProblem &meshed_problem) const;

private:
  AssemblerOptions options_;
};

} // namespace pde2d::fem

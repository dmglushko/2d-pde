#pragma once

#include "pde2d/fem/csr_matrix.hpp"
#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/report.hpp"

#include <memory>
#include <span>
#include <vector>

namespace pde2d::fem {

class Assembler;

namespace detail {
// What a later assemble() reuses: patterns, colours, constant parts. Defined
// in src/fem/assembler.cpp.
struct AssemblyInternals;
} // namespace detail

// One operator of the assembled system, split by column kind. Rows are the
// free unknowns (MeshedProblem::free_nodes()).
struct Operator {
  // Free rows x free columns (n_free x n_free).
  CsrMatrix free{};
  // Free rows x Dirichlet columns; column k is dirichlet_nodes()[k].
  CsrMatrix dirichlet{};
  // Two kinds of facts (docs/decisions.md, "Two kinds of facts per
  // matrix"). The facts from the formulas hold for every t: they come
  // from the terms assembly builds, never from the values, and decide the
  // time order, reuse and what is built. The facts from the numbers hold
  // at the assembled time only: they come from the values of `free`, and
  // only say whether a linear solver may take the matrix now.

  // From the formulas: some term of it has a coefficient that depends on
  // t (as written).
  bool depends_on_t = false;
  // From the formulas: no term contributes, at any t. Every coefficient
  // feeding it is zero as written, or assembly left it out
  // (Assembler::assemble's max_time_order). Its matrices then hold the
  // pattern with zero values.
  bool always_zero = false;
  // From the formulas: the free part is symmetric at every t. M2, M1, D,
  // K, R and S are symmetric, C and T only when they are zero
  // (docs/decisions.md, "Linear solvers declare requirements; matrices
  // carry properties"). A zero operator, or one with only Dirichlet
  // columns, is symmetric. Up to rounding: the blocks are computed in full,
  // so (i, j) and (j, i) can differ in the last bits.
  bool always_symmetric = false;

  // From the numbers: every value of `free` is zero at the assembled time
  // (has_only_zero_entries). True when always_zero is; it can also be true
  // for one t only (m2 = sin(t) at t = 0).
  bool zero_now = false;
  // From the numbers: `free` is symmetric at the assembled time, up to
  // rounding (is_symmetric with its default tolerance). True when
  // always_symmetric is; it can also be true when the formulas cannot
  // promise it (an anisotropic diagonal A, whose tangential term T
  // vanishes on axis-aligned boundary edges).
  bool symmetric_now = false;
};

// The system assembly builds from a MeshedProblem at one time t, in the
// free unknowns U (the Dirichlet nodes are eliminated):
//
//   M2 U'' + B U' + A U = b,  B = M1 - D,  A = -K + C + R - S + T,
//   b = (F - G) - A_FD U_D - B_FD U_D' - M2_FD U_D''
//
// with the interior matrices K (a_xx, a_xy, a_yy), C (b - div A), R (c),
// M1 (m1), M2 (m2) and load F (f), and the boundary matrices of the
// natural and dynamic conditions S (alpha), T (tangential part), D (gamma)
// and load G (g); docs/decisions.md, "Assembly", has the weak form. An
// operator's `free` part is X_FF and its `dirichlet` part X_FD.
//
// Built only by Assembler::assemble. The facts here are computed by
// assembly from what it builds, never copied from the MeshedProblem.
// Internally every operator and the load keep a constant part (the terms
// whose coefficients do not depend on t) that a later assemble() reuses
// through `previous`, together with the patterns and the colouring.
class AssembledProblem {
public:
  const std::shared_ptr<const MeshedProblem> &meshed_problem() const {
    return meshed_problem_;
  }
  // The t it was assembled at.
  double time() const { return time_; }

  const Operator &mass() const { return mass_; }           // M2
  const Operator &damping() const { return damping_; }     // B = M1 - D
  const Operator &stiffness() const { return stiffness_; } // A

  // F - G on the free rows, before lifting.
  std::span<const double> load() const { return load_; }
  // U_D(t), one per Dirichlet node (MeshedProblem::dirichlet_value).
  std::span<const double> dirichlet_values() const { return values_; }
  // U_D'(t) when time_order() >= 1, else empty (MeshedProblem::
  // dirichlet_rate).
  std::span<const double> dirichlet_rates() const { return rates_; }
  // U_D''(t) when time_order() == 2, else empty.
  std::span<const double> dirichlet_accelerations() const {
    return accelerations_;
  }
  // b = load - A_FD U_D - B_FD U_D' - M2_FD U_D'' (a product is left out
  // when its operator is zero).
  std::span<const double> right_hand_side() const { return right_hand_side_; }

  // F - G changes with t: a term of it has a coefficient that depends on t.
  bool load_depends_on_t() const { return load_depends_on_t_; }
  // b changes with t: the load does, or a lifting product has a nonzero
  // operator and Dirichlet nodes, and that operator or its Dirichlet data
  // depends on t. The data U_D, U_D', U_D'' count as depending on t when
  // g / alpha of a region used at a Dirichlet node does.
  bool right_hand_side_depends_on_t() const { return rhs_depends_on_t_; }
  // 2 if mass() is not zero, else 1 if damping() is not zero, else 0. It
  // can be lower than the problem's time_order (a dynamic condition whose
  // D is zero because A is), and it is at most the max_time_order it was
  // assembled with. Decided from the formula facts (always_zero), so it is
  // one answer for every t: m2 = sin(t) gives 2 also at t = 0.
  // Assembler::time_order gives the same without assembling.
  int time_order() const;

  // Stage "assembly": the time taken and metrics (see Assembler).
  const mesh::StageReport &report() const { return report_; }

private:
  friend class Assembler;
  AssembledProblem() = default;

  std::shared_ptr<const MeshedProblem> meshed_problem_;
  double time_ = 0.0;
  Operator mass_, damping_, stiffness_;
  std::vector<double> load_;
  std::vector<double> values_, rates_, accelerations_;
  std::vector<double> right_hand_side_;
  bool load_depends_on_t_ = false;
  bool rhs_depends_on_t_ = false;
  mesh::StageReport report_;
  std::shared_ptr<const detail::AssemblyInternals> internals_;
};

} // namespace pde2d::fem

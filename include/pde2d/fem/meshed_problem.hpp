#pragma once

#include "pde2d/fem/compiled_expression.hpp"
#include "pde2d/fem/problem.hpp"
#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/report.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace pde2d::fem {

// The equation's coefficients as numeric functions (members as in
// Equation).
struct CompiledEquation {
  CompiledExpression a_xx{}, a_xy{}, a_yy{};
  CompiledExpression b_x{}, b_y{};
  CompiledExpression c{};
  CompiledExpression m1{}, m2{};
  CompiledExpression f{};
};

// The divergence of the matrix A = [[a_xx, a_xy / 2], [a_xy / 2, a_yy]] of
// the equation's second-order part, which the weak form needs (assembly's
// convection term):
//   x = d/dx a_xx + d/dy a_xy / 2,   y = d/dx a_xy / 2 + d/dy a_yy.
// Its facts are those of the derivatives as computed (a_xx = x + t gives
// x = 1, which does not depend on t).
struct CompiledDivergence {
  CompiledExpression x{}, y{};
};

// One region's boundary condition, classified and compiled (members as in
// BoundaryCondition).
struct CompiledCondition {
  ConditionKind kind = ConditionKind::dirichlet;
  bool oblique = false; // is_oblique()
  int priority = 0;
  CompiledExpression alpha{};
  CompiledExpression beta_n{}, beta_tau{}, beta_x{}, beta_y{};
  CompiledExpression gamma{};
  CompiledExpression g{};
};

// A mesh node whose value is given by Dirichlet conditions. `regions` are
// the region ids (into Mesh::region_names) whose g / alpha is averaged
// there, in increasing order: one region, or two at a corner between two
// Dirichlet regions of equal priority.
struct DirichletNode {
  std::size_t node = 0;
  std::vector<std::size_t> regions{};
};

// A Problem combined with a mesh, ready for assembly. The constructor:
//
// 1. matches region names both ways: a boundary condition for a region the
//    mesh does not have is an error, and so is a mesh region without a
//    condition;
// 2. classifies every condition (classify());
// 3. decides the Dirichlet nodes: a node is Dirichlet if one of its two
//    boundary edges has a Dirichlet condition (Dirichlet beats natural and
//    dynamic); at a corner between two Dirichlet regions the higher
//    priority wins and equal priorities are averaged;
// 4. compiles every expression with the problem's parameters;
// 5. compiles the initial conditions that time_order() requires (u for 1,
//    u and u_t for 2; a missing one is an error), the time derivative
//    d/dt (g / alpha) of every Dirichlet condition for time order 1 or 2,
//    and its second derivative for time order 2. An
//    initial condition that is not needed is ignored, and one that depends
//    on t is evaluated at the start time; both get a report message. Each
//    compiled initial condition is compared with the boundary data that
//    replaces it at the Dirichlet nodes (u with g / alpha, u_t with
//    d/dt (g / alpha)): when neither depends on t, exactly at every node
//    (the boundary data must then be finite there), and a difference is
//    reported with the number of nodes and the first three; otherwise they
//    are reported once as possibly different;
// 6. compiles the divergence of A (divergence_of_a()) and decides whether
//    A is isotropic (a_is_isotropic());
// 7. keeps each expression's facts with its compiled function, so the
//    summaries below are computed from the compiled problem.
//
// Errors throw std::invalid_argument, with the coefficient or region they
// concern. The mesh is checked with mesh::validate first.
//
// report() is a mesh::StageReport ("meshed_problem"): the time taken, the
// metrics regions, dirichlet_nodes, free_nodes, corners (boundary nodes
// where the region changes), dirichlet_over_natural_corners,
// priority_corners, averaged_corners, differing_corners,
// differing_initial_values and differing_initial_velocities (the Dirichlet
// nodes where u or u_t differs from the boundary data, or all of them when
// they may differ), one message for every corner between two Dirichlet
// regions whose values differ (or depend on t, unless both conditions are
// the same as written), one for every initial condition that is not needed
// or depends on t, and one for every initial condition that differs or may
// differ from the boundary data.
// The initial state over the unknowns only, indexed like
// MeshedProblem::free_nodes(): what a time integrator starts from.
struct FreeInitialState {
  std::vector<double> values;     // u
  std::vector<double> velocities; // u_t; empty below time order 2
};

class MeshedProblem {
public:
  MeshedProblem(std::shared_ptr<const mesh::Mesh> mesh, const Problem &problem);

  const std::shared_ptr<const mesh::Mesh> &mesh() const { return mesh_; }
  const Problem &problem() const { return problem_; }

  const CompiledEquation &equation() const { return equation_; }
  const CompiledDivergence &divergence_of_a() const { return divergence_; }
  // A is a multiple of the identity (or zero): a_xy is zero as written and
  // a_xx - a_yy simplifies to zero (SymEngine's expand; named parameters
  // are unknown constants, so a_xx = k1, a_yy = k2 is not isotropic).
  // Assembly uses it to recognise the tangential boundary matrix T as
  // zero.
  bool a_is_isotropic() const { return a_is_isotropic_; }
  // One per mesh region, indexed like Mesh::region_names.
  const std::vector<CompiledCondition> &conditions() const {
    return conditions_;
  }

  // The Dirichlet nodes, by increasing node index.
  const std::vector<DirichletNode> &dirichlet_nodes() const {
    return dirichlet_nodes_;
  }
  // The value at dirichlet_nodes()[k] at time t: the average of g / alpha
  // over its regions. Throws std::invalid_argument if it is not finite.
  double dirichlet_value(std::size_t k, double t) const;
  // d/dt and d2/dt2 of the value at dirichlet_nodes()[k] at time t: the
  // average over its regions of d/dt (g / alpha) and d2/dt2 (g / alpha),
  // like dirichlet_value. They are compiled only when the time order needs
  // them: the rate for time order 1 or 2, the acceleration for 2. Throws
  // std::invalid_argument for a lower time order, for a k that does not
  // exist and for a value that is not finite.
  double dirichlet_rate(std::size_t k, double t) const;
  double dirichlet_acceleration(std::size_t k, double t) const;

  // The other nodes, by increasing index: the unknowns of the shrunk
  // system. Unknown i is mesh node free_nodes()[i].
  const std::vector<std::size_t> &free_nodes() const { return free_nodes_; }
  // The unknown of a mesh node, or nothing for a Dirichlet node.
  std::optional<std::size_t> free_index(std::size_t node) const;

  // u at every mesh node (indexed like Mesh::nodes) at the start time t0,
  // or nothing for time order 0, which has no initial values. At a
  // Dirichlet node it is the boundary data, dirichlet_value at t0;
  // elsewhere the initial condition u at (x, y, t0). Computed on every
  // call. Throws std::invalid_argument for a t0 that is not finite (also
  // for time order 0) or a value that is not finite.
  std::optional<std::vector<double>> initial_values(double t0) const;
  // u_t at every mesh node at t0, or nothing below time order 2. At a
  // Dirichlet node it is d/dt (g / alpha) at t0, averaged over the node's
  // regions like dirichlet_value; elsewhere the initial condition u_t at
  // (x, y, t0). Throws like initial_values.
  std::optional<std::vector<double>> initial_velocities(double t0) const;
  // The initial conditions u and (for time order 2) u_t at the free nodes
  // only, at t0; nothing for time order 0. Unlike initial_values and
  // initial_velocities it evaluates no boundary data, so Dirichlet data
  // whose time derivative is not finite at t0 (u = sqrt(t) from t0 = 0)
  // does not stop a run that never uses it. Throws like initial_values.
  std::optional<FreeInitialState> initial_free_state(double t0) const;

  const mesh::StageReport &report() const { return report_; }

private:
  std::shared_ptr<const mesh::Mesh> mesh_;
  Problem problem_;
  CompiledEquation equation_;
  std::vector<CompiledCondition> conditions_;
  std::vector<DirichletNode> dirichlet_nodes_;
  std::vector<std::size_t> free_nodes_;
  // Per node: its unknown, or the largest std::size_t for a Dirichlet node.
  std::vector<std::size_t> free_index_;
  // The initial conditions the time order needs, compiled.
  std::optional<CompiledExpression> initial_u_;
  std::optional<CompiledExpression> initial_u_t_;
  CompiledDivergence divergence_;
  bool a_is_isotropic_ = false;
  // Per region, for time order 1 or 2: d/dt (g / alpha) of a Dirichlet
  // condition, zero otherwise. Empty for time order 0.
  std::vector<CompiledExpression> dirichlet_rates_;
  // Per region, for time order 2: d2/dt2 (g / alpha), like the rates.
  std::vector<CompiledExpression> dirichlet_accelerations_;
  mesh::StageReport report_;
};

// The summaries of problem.hpp, computed from the facts kept with the
// compiled functions.
int time_order(const MeshedProblem &problem);
bool coefficients_depend_on_t(const MeshedProblem &problem);
bool data_depends_on_t(const MeshedProblem &problem);

} // namespace pde2d::fem

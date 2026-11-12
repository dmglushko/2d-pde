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
// 5. evaluates the initial conditions at the nodes (required ones by
//    time_order(); a missing one, or one that is not needed, is an error);
// 6. keeps each expression's facts with its compiled function, so the
//    summaries below are computed from the compiled problem.
//
// Errors throw std::invalid_argument, with the coefficient or region they
// concern. The mesh is checked with mesh::validate first.
//
// report() is a mesh::StageReport ("meshed_problem"): the time taken, the
// metrics regions, dirichlet_nodes, free_nodes, corners (boundary nodes
// where the region changes), dirichlet_over_natural_corners,
// priority_corners, averaged_corners and differing_corners, and one
// message for every corner between two Dirichlet regions whose values
// differ (or depend on t, unless both conditions are the same as written).
class MeshedProblem {
public:
  MeshedProblem(std::shared_ptr<const mesh::Mesh> mesh, const Problem &problem);

  const std::shared_ptr<const mesh::Mesh> &mesh() const { return mesh_; }
  const Problem &problem() const { return problem_; }

  const CompiledEquation &equation() const { return equation_; }
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

  // The other nodes, by increasing index: the unknowns of the shrunk
  // system. Unknown i is mesh node free_nodes()[i].
  const std::vector<std::size_t> &free_nodes() const { return free_nodes_; }
  // The unknown of a mesh node, or nothing for a Dirichlet node.
  std::optional<std::size_t> free_index(std::size_t node) const;

  // u and u_t at every mesh node, when the time order requires them.
  const std::optional<std::vector<double>> &initial_u() const {
    return initial_u_;
  }
  const std::optional<std::vector<double>> &initial_u_t() const {
    return initial_u_t_;
  }

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
  std::optional<std::vector<double>> initial_u_;
  std::optional<std::vector<double>> initial_u_t_;
  mesh::StageReport report_;
};

// The summaries of problem.hpp, computed from the facts kept with the
// compiled functions.
int time_order(const MeshedProblem &problem);
bool lhs_depends_on_t(const MeshedProblem &problem);
bool rhs_depends_on_t(const MeshedProblem &problem);

} // namespace pde2d::fem

// bind_boundary owns a snapshot of the problem (boundary_assignment.hpp).
// Oracles: a binding made before the Problem is destroyed or changed must
// equal, field by field and in its classification, a binding of an
// identical problem built separately; and assembly from a binding-time
// problem equals assembly of the unchanged problem bit for bit.
#include "discretization/square_mesh.hpp"
#include "fem/assembly.hpp"
#include "fem/boundary_assignment.hpp"
#include "fem/boundary_classification.hpp"
#include "fem/fingerprint.hpp"
#include "support/check.hpp"
#include <memory>
#include <optional>

using pde2d::discretization::square_mesh;
using pde2d_test::check;
namespace {
using namespace pde2d::fem;

Problem mixed() {
  Problem p;
  p.equation.source = 1 + x;
  p.assign(dirichlet(x * y), {"bottom", "top"});
  p.assign(robin(2, 1, y), {"left", "right"});
  return p;
}

// Classification and indices of every edge, compared between bindings.
void same_binding(const BoundaryBinding &a, const BoundaryBinding &b) {
  check(a.regions == b.regions && a.edges.size() == b.edges.size() &&
            a.conditions.size() == b.conditions.size(),
        "Shape");
  check(expression_text(a.equation.source) ==
            expression_text(b.equation.source),
        "Equation snapshot");
  for (std::size_t k = 0; k < a.edges.size(); ++k) {
    const auto &ea = a.edges[k];
    const auto &eb = b.edges[k];
    check(ea.edge == eb.edge && ea.nodes == eb.nodes &&
              ea.region == eb.region && ea.assignment == eb.assignment &&
              ea.frame.normal_x == eb.frame.normal_x &&
              ea.frame.normal_y == eb.frame.normal_y,
          "Edge binding");
    const auto ca =
        classify_edge_condition(a.equation, a.condition(ea), ea.frame);
    const auto cb =
        classify_edge_condition(b.equation, b.condition(eb), eb.frame);
    check(ca.is_dirichlet() == cb.is_dirichlet() &&
              ca.form.index() == cb.form.index(),
          "Classification");
    check(expression_text(a.condition(ea).value) ==
              expression_text(b.condition(eb).value),
          "Condition snapshot");
  }
}

void survives_destroyed_problem() {
  const auto mesh = square_mesh(3);
  std::optional<BoundaryBinding> binding;
  {
    auto problem = std::make_unique<Problem>(mixed());
    binding = bind_boundary(mesh, *problem);
  } // the Problem is gone
  same_binding(*binding, bind_boundary(mesh, mixed()));
  // Assignment indices follow Problem::boundaries.
  for (const auto &edge : binding->edges) {
    const auto &tag = mesh.boundary_tags[edge.edge];
    check(edge.assignment == (tag == "bottom" || tag == "top" ? 0u : 1u),
          "Assignment index");
    check(binding->condition(edge).alpha == Expr(edge.assignment ? 2 : 1),
          "Condition by index");
  }
  // A copy is independent of the original.
  auto copy = *binding;
  binding.reset();
  same_binding(copy, bind_boundary(mesh, mixed()));
}

void unchanged_by_mutated_problem() {
  const auto mesh = square_mesh(3);
  auto problem = mixed();
  const auto binding = bind_boundary(mesh, problem);
  problem.boundaries[0].condition = neumann(5);
  problem.boundaries.push_back({{"nowhere"}, dirichlet(0)});
  problem.equation.source = 0;
  same_binding(binding, bind_boundary(mesh, mixed()));

  // Assembly prepared from the original problem is unaffected as well.
  auto changing = mixed();
  const PreparedAssembly prepared(mesh, changing);
  changing.equation.source = 7;
  changing.boundaries[1].condition = dirichlet(0);
  const auto a = prepared.assemble(0);
  const auto b = assemble(mesh, mixed(), 0);
  check(SparseMatrix(a.stiffness - b.stiffness).norm() == 0 &&
            a.load == b.load && a.dirichlet.size() == b.dirichlet.size(),
        "Prepared assembly keeps its own problem");
}
} // namespace
int main() {
  return pde2d_test::run(
      {{"survives_destroyed_problem", survives_destroyed_problem},
       {"unchanged_by_mutated_problem", unchanged_by_mutated_problem}});
}

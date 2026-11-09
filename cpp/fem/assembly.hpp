#pragma once
// Global P1 assembly with immutable preparation.
//
//   PreparedAssembly prepared(mesh, problem, options);   // symbolic work once
//   AssembledSystem s = prepared.assemble(time);         // numeric work only
//
// Preparation validates the mesh and boundary assignment, compiles every
// coefficient into an Evaluator, classifies each boundary edge
// (boundary_classification.hpp) and caches every time-independent local
// contribution. assemble() evaluates only time-dependent terms, using the P1
// kernels in p1_element.hpp, optionally in parallel (execution.hpp), and
// scatters in a fixed element/edge order so results are deterministic.
//
// Boundary regions are bound to conditions by bind_boundary
// (boundary_assignment.hpp), shared with analysis. The private pieces live
// in detail/: interior elements
// (assembly_elements.hpp), boundary edges (assembly_edges.hpp) and the serial
// scatter (assembly_scatter.hpp). The element-independent result types are in
// system.hpp.
#include "discretization/mesh_validation.hpp"
#include "execution.hpp"
#include "fem/detail/assembly_edges.hpp"
#include "fem/detail/assembly_elements.hpp"
#include "fem/detail/assembly_scatter.hpp"
#include "fem/problem.hpp"
#include "fem/system.hpp"
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace pde2d::fem {
struct AssemblyOptions {
  int quadrature_degree = 2;
  double boundary_tolerance = 1e-10;
  pde2d::ExecutionOptions execution{};
};
// Immutable preparation counters. Per-assembly interior evaluations count
// successful full calls; boundary validation/evaluation is deliberately
// separate.
struct AssemblyStatistics {
  std::size_t elements = 0, edges = 0, physical_quadrature_points = 0,
              interior_constant_evaluations = 0,
              interior_evaluations_per_assembly = 0, evaluators = 0;
};

namespace assembly_detail {
// Validation, the order of preparation and parallel evaluation; the element,
// edge and scatter work is delegated to the detail/ headers.
class PreparedData {
public:
  discretization::TriangleMesh mesh;
  Problem problem;
  AssemblyOptions options;
  AssemblyStatistics statistics;
  PreparedInterior interior;
  PreparedBoundary boundary;

  PreparedData(const discretization::TriangleMesh &input, const Problem &p,
               const AssemblyOptions &o)
      : mesh(input), problem(p), options(o) {
    pde2d::validate_execution(options.execution);
    if (!std::isfinite(options.boundary_tolerance) ||
        options.boundary_tolerance < 0)
      throw std::invalid_argument("Invalid FEM time or boundary tolerance");
    discretization::validate_mesh_geometry(mesh);
    boundary.set_rule(options.quadrature_degree);
    if (mesh.nodes.size() >
        static_cast<std::size_t>(std::numeric_limits<int>::max()))
      throw std::invalid_argument("Mesh exceeds sparse matrix index capacity");
    interior.compile(problem.equation);
    interior.prepare(mesh, options.quadrature_degree);
    boundary.prepare(bind_boundary(mesh, problem));
    statistics.elements = interior.elements.size();
    statistics.edges = boundary.edges.size();
    statistics.physical_quadrature_points =
        interior.counts.physical_quadrature_points;
    statistics.interior_constant_evaluations =
        interior.counts.constant_evaluations;
    statistics.interior_evaluations_per_assembly =
        interior.counts.evaluations_per_assembly;
    statistics.evaluators = interior.coefficients.size();
    for (const auto &edge : boundary.edges)
      statistics.evaluators += edge.evaluator_count();
  }

  AssembledSystem assemble(double time, DirichletDerivative derivative) const {
    if (!std::isfinite(time))
      throw std::invalid_argument("Invalid FEM assembly time");
    (void)dirichlet_derivative_order(derivative); // rejects invalid values
    // Workers only borrow prepared evaluators and write their own numeric slot.
    // Evaluator CSE is disabled, so calls do not write visitor scratch storage.
    const auto &elements = interior.elements;
    std::vector<LocalBlock> blocks(elements.size());
    pde2d::parallel_for(elements.size(), options.execution, [&](std::size_t i) {
      blocks[i] = interior.block(elements[i], time, interior.dynamic);
    });
    const auto &edges = boundary.edges;
    std::vector<EdgeResult> results(edges.size());
    pde2d::parallel_for(edges.size(), options.execution, [&](std::size_t i) {
      results[i] = boundary.evaluate(edges[i], time, derivative);
    });
    return scatter(mesh.nodes.size(), time, elements, blocks, edges, results,
                   options.boundary_tolerance);
  }
};
} // namespace assembly_detail

class PreparedAssembly {
  std::shared_ptr<const assembly_detail::PreparedData> data_;

public:
  PreparedAssembly(const discretization::TriangleMesh &mesh,
                   const Problem &problem, const AssemblyOptions &options = {})
      : data_(std::make_shared<const assembly_detail::PreparedData>(
            mesh, problem, options)) {}
  // `derivative` selects the Dirichlet data: the values, or their first or
  // second time derivative (used to start Newmark).
  AssembledSystem
  assemble(double time = 0,
           DirichletDerivative derivative = DirichletDerivative::Value) const {
    return data_->assemble(time, derivative);
  }
  // Compatibility: integer order 0, 1 or 2.
  AssembledSystem assemble(double time, int dirichlet_derivative_order) const {
    if (!std::isfinite(time))
      throw std::invalid_argument("Invalid FEM assembly time");
    return assemble(time, dirichlet_derivative(dirichlet_derivative_order));
  }
  const discretization::TriangleMesh &mesh() const { return data_->mesh; }
  const Problem &problem() const { return data_->problem; }
  const AssemblyOptions &options() const { return data_->options; }
  const AssemblyStatistics &statistics() const { return data_->statistics; }
};

// One-shot convenience: prepare and assemble at a single time.
inline AssembledSystem assemble(const discretization::TriangleMesh &mesh,
                                const Problem &problem, double time = 0,
                                const AssemblyOptions &options = {}) {
  return PreparedAssembly(mesh, problem, options).assemble(time);
}
} // namespace pde2d::fem

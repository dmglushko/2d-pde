#pragma once
// Private part of PreparedAssembly (assembly.hpp): boundary edges.
//
// Preparation takes the validated region binding (boundary_assignment.hpp),
// reduces each edge's boundary condition to one EdgeForm
// alternative (boundary_classification.hpp) and lets that form's unit
// (detail/edge_forms/) compile its restricted coefficients and cache its
// time-independent contribution. evaluate() asks the unit for the complete
// contribution at one time; it only reads prepared data. The generic code
// here never names a form: it calls through edge_form_detail::Unit.
#include "discretization/geometry_base.hpp"
#include "fem/boundary_assignment.hpp"
#include "fem/boundary_classification.hpp"
#include "fem/detail/assembly_elements.hpp"
#include "fem/detail/edge_forms/forms.hpp"
#include "fem/expression.hpp"
#include "fem/p1_element.hpp"
#include "fem/problem.hpp"
#include "fem/quadrature.hpp"
#include "fem/system.hpp"
#include <array>
#include <cstddef>
#include <variant>
#include <vector>

namespace pde2d::fem::assembly_detail {
using edge_form_detail::add_cached;
using edge_form_detail::EdgeResult;
using edge_form_detail::EdgeSite;
using edge_form_detail::PreparedEdgeForm;
using edge_form_detail::unit_of;
using PreparedDirichletEdge = edge_form_detail::Unit<DirichletEdge>::Prepared;
using PreparedNaturalEdge = edge_form_detail::Unit<NaturalEdge>::Prepared;
using PreparedObliqueEdge = edge_form_detail::Unit<ObliqueEdge>::Prepared;

struct PreparedEdge {
  std::array<std::size_t, 2> nodes;
  double length, normal_x, normal_y;
  PreparedEdgeForm form;

  bool is_dirichlet() const noexcept {
    return std::holds_alternative<PreparedDirichletEdge>(form);
  }
  std::size_t evaluator_count() const {
    std::size_t count = 0;
    std::visit(
        [&](const auto &f) { f.for_each([&](const Evaluator &) { ++count; }); },
        form);
    return count;
  }
};

class PreparedBoundary {
public:
  std::vector<PreparedEdge> edges;
  std::vector<EdgeQuadraturePoint> rule;
  // Edge coordinates where every form's divisors must keep one strict sign
  // (edge_divisor_samples).
  std::vector<double> divisor_samples;

  // Set up the edge rule (validates the quadrature degree).
  void set_rule(int quadrature_degree) {
    rule = edge_quadrature(quadrature_degree);
    divisor_samples = edge_divisor_samples(quadrature_degree);
  }

  // Classify and prepare every bound edge (after set_rule()).
  void prepare(const BoundaryBinding &binding) {
    for (const auto &bound : binding.edges) {
      const auto &frame = bound.frame;
      const auto c = classify_edge_condition(binding.equation,
                                             binding.condition(bound), frame);
      const EdgeSite site{frame.length, frame.normal_x, frame.normal_y, rule,
                          divisor_samples};
      edges.push_back(
          {bound.nodes, frame.length, frame.normal_x, frame.normal_y,
           std::visit(
               [&]<class Form>(const Form &form) -> PreparedEdgeForm {
                 return edge_form_detail::Unit<Form>::prepare(form, frame,
                                                              site);
               },
               c.form)});
    }
  }

  // Complete contribution of one edge at `time`: Dirichlet values of the
  // requested derivative, or the cached plus time-dependent block.
  EdgeResult evaluate(const PreparedEdge &e, double time,
                      DirichletDerivative derivative) const {
    const EdgeSite site{e.length, e.normal_x, e.normal_y, rule,
                        divisor_samples};
    return std::visit(
        [&]<class Prepared>(const Prepared &form) {
          return unit_of<Prepared>::evaluate(form, site, time, derivative);
        },
        e.form);
  }
};
} // namespace pde2d::fem::assembly_detail

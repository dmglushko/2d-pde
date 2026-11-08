#pragma once
// Private part of PreparedAssembly (assembly.hpp): interior elements.
//
// Preparation compiles the weak-form coefficients, places the physical
// quadrature points of every triangle and caches each time-independent
// contribution. block() evaluates the remaining terms at one time; it only
// reads prepared data, so parallel workers may call it concurrently.
#include "discretization/geometry_base.hpp"
#include "fem/expression.hpp"
#include "fem/p1_element.hpp"
#include "fem/problem.hpp"
#include "fem/quadrature.hpp"
#include "fem/weak_form.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

namespace pde2d::fem::assembly_detail {
using p1::checked;
using p1::LocalBlock;

struct PhysicalPoint {
  EvaluationPoint point;
  std::array<double, LocalBlock::nodes> phi;
  double weight;
  WeakCoefficients<double> constant{};
};
struct PreparedElement {
  std::array<std::size_t, LocalBlock::nodes> nodes;
  p1::TriangleGeometry geometry;
  std::vector<PhysicalPoint> points;
  // Sum of every time-independent weak-form term.
  LocalBlock constant;
};

// Work counters reported through AssemblyStatistics.
struct InteriorCounts {
  std::size_t physical_quadrature_points = 0, constant_evaluations = 0,
              evaluations_per_assembly = 0;
};

class PreparedInterior {
public:
  std::vector<Evaluator> coefficients;
  // Coefficients that depend on time and are evaluated by every assembly.
  WeakFlags dynamic{};
  std::vector<PreparedElement> elements;
  InteriorCounts counts;

  // Validate and compile the interior coefficients.
  void compile(const Equation &equation) {
    validate_interior_equation(equation);
    const auto expressions = weak_form_coefficients(equation);
    for (const auto &expression : expressions)
      coefficients.emplace_back(expression);
    dynamic = weak_time_dependence(expressions);
  }

  // Prepare every element (after compile()).
  void prepare(const discretization::TriangleMesh &mesh,
               int quadrature_degree) {
    const auto triangle_rule = triangle_quadrature(quadrature_degree);
    for (const auto &element : mesh.elements) {
      const auto a = mesh.nodes[element[0]], b = mesh.nodes[element[1]],
                 c = mesh.nodes[element[2]];
      PreparedElement e{element, p1::triangle_geometry(a, b, c), {}, {}};
      const double twice_area = e.geometry.twice_area;
      for (const auto &q : triangle_rule) {
        const auto &phi = q.barycentric;
        PhysicalPoint point{{phi[0] * a.x + phi[1] * b.x + phi[2] * c.x,
                             phi[0] * a.y + phi[1] * b.y + phi[2] * c.y, 0},
                            phi,
                            twice_area / 2 * q.weight,
                            {}};
        for (std::size_t k = 0; k < weak::Count; ++k)
          if (!dynamic[k]) {
            point.constant[k] = coefficients[k](point.point);
            ++counts.constant_evaluations;
          } else
            ++counts.evaluations_per_assembly;
        ++counts.physical_quadrature_points;
        e.points.push_back(point);
      }
      // The weak form is linear in its coefficients. Cache each
      // time-independent contribution, even inside a partly dynamic matrix.
      WeakFlags fixed;
      for (std::size_t k = 0; k < weak::Count; ++k)
        fixed[k] = !dynamic[k];
      e.constant = block(e, 0, fixed);
      elements.push_back(std::move(e));
    }
  }

  // Contribution of the `enabled` terms at `time`.
  LocalBlock block(const PreparedElement &e, double time,
                   const WeakFlags &enabled) const {
    LocalBlock out;
    if (std::none_of(enabled.begin(), enabled.end(), [](bool v) { return v; }))
      return out;
    const auto active = p1::active_blocks(enabled);
    for (const auto &q : e.points) {
      WeakCoefficients<double> v{};
      auto point = q.point;
      point.t = time;
      for (std::size_t k = 0; k < weak::Count; ++k)
        if (enabled[k])
          v[k] = dynamic[k] ? coefficients[k](point) : q.constant[k];
      p1::accumulate_element_point(out, v, q.phi, e.geometry, q.weight, active);
    }
    return out;
  }
};
} // namespace pde2d::fem::assembly_detail

#pragma once
#include "discretization/mesh_validation.hpp"
#include "execution.hpp"
#include "fem/expression.hpp"
#include "fem/quadrature.hpp"
#include "fem/system.hpp"
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace pde2d::fem {

struct ErrorNorms {
  double l2 = 0, h1_seminorm = 0;
};

// Own the geometry, rule and compiled exact fields. Calls borrow evaluators;
// workers never construct or copy symbolic expressions (including RCPs).
// The exact solution is a field in x, y and t; boundary normals are rejected.
class PreparedErrorNorms {
  struct Point {
    EvaluationPoint location;
    std::array<double, 3> basis;
    double weight;
  };
  struct Element {
    std::array<std::size_t, 3> nodes;
    std::array<double, 3> gx, gy;
    std::vector<Point> points;
  };
  std::size_t node_count_;
  Evaluator u_, ux_, uy_;
  static const Expr &exact_field(const Expr &exact) {
    require_symbols(exact, field_symbols, "Exact solution");
    return exact;
  }
  std::vector<Element> elements_;

public:
  PreparedErrorNorms(const discretization::TriangleMesh &mesh,
                     const Expr &exact, int degree = 8)
      : node_count_(mesh.nodes.size()), u_(exact_field(exact)),
        ux_(diff(exact, x)), uy_(diff(exact, y)) {
    discretization::validate_mesh_geometry(mesh);
    const auto rule = triangle_quadrature(degree);
    elements_.reserve(mesh.elements.size());
    for (const auto &tri : mesh.elements) {
      const auto a = mesh.nodes[tri[0]], b = mesh.nodes[tri[1]],
                 c = mesh.nodes[tri[2]];
      const double det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
      Element e{{static_cast<std::size_t>(tri[0]),
                 static_cast<std::size_t>(tri[1]),
                 static_cast<std::size_t>(tri[2])},
                {(b.y - c.y) / det, (c.y - a.y) / det, (a.y - b.y) / det},
                {(c.x - b.x) / det, (a.x - c.x) / det, (b.x - a.x) / det},
                {}};
      for (const auto &q : rule) {
        EvaluationPoint p;
        for (int j = 0; j < 3; ++j) {
          p.x += q.barycentric[j] * mesh.nodes[tri[j]].x;
          p.y += q.barycentric[j] * mesh.nodes[tri[j]].y;
        }
        e.points.push_back({p, q.barycentric, std::abs(det) * .5 * q.weight});
      }
      elements_.push_back(std::move(e));
    }
  }
  const Evaluator &exact_evaluator() const { return u_; }
  ErrorNorms evaluate(const Vector &values, double time = 0,
                      const pde2d::ExecutionOptions &execution = {}) const {
    if (values.size() != static_cast<Eigen::Index>(node_count_) ||
        !values.allFinite() || !std::isfinite(time))
      throw std::invalid_argument("Invalid error norm values or time");
    std::vector<ErrorNorms> contributions(elements_.size());
    pde2d::parallel_for(elements_.size(), execution, [&](std::size_t i) {
      const auto &e = elements_[i];
      auto &sum = contributions[i];
      double dx = 0, dy = 0;
      for (int j = 0; j < 3; ++j) {
        dx += e.gx[j] * values[e.nodes[j]];
        dy += e.gy[j] * values[e.nodes[j]];
      }
      for (const auto &q : e.points) {
        auto p = q.location;
        p.t = time;
        double uh = 0;
        for (int j = 0; j < 3; ++j)
          uh += q.basis[j] * values[e.nodes[j]];
        const double error = uh - u_(p), ex = dx - ux_(p), ey = dy - uy_(p);
        sum.l2 += q.weight * error * error;
        sum.h1_seminorm += q.weight * (ex * ex + ey * ey);
      }
    });
    ErrorNorms result;
    for (const auto &e : contributions) {
      result.l2 += e.l2;
      result.h1_seminorm += e.h1_seminorm;
    }
    result.l2 = std::sqrt(result.l2);
    result.h1_seminorm = std::sqrt(result.h1_seminorm);
    return result;
  }
};
inline ErrorNorms error_norms(const discretization::TriangleMesh &mesh,
                              const Vector &values, const Expr &exact,
                              double time = 0, int degree = 8,
                              const pde2d::ExecutionOptions &execution = {}) {
  return PreparedErrorNorms(mesh, exact, degree)
      .evaluate(values, time, execution);
}
} // namespace pde2d::fem

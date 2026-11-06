#pragma once

#include "discretization/mesh_validation.hpp"
#include "execution.hpp"
#include "problem.hpp"
#include "quadrature.hpp"
#include <Eigen/Sparse>
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <set>
#include <stdexcept>
#include <vector>

namespace fem {
using SparseMatrix = Eigen::SparseMatrix<double>;
using Vector = Eigen::VectorXd;
struct AssemblyOptions {
  int quadrature_degree = 2;
  double boundary_tolerance = 1e-10;
  pde2d::ExecutionOptions execution{};
};
struct DirichletValue {
  Eigen::Index node;
  double value;
};
struct AssembledSystem {
  SparseMatrix stiffness, first_mass, second_mass;
  Vector load;
  std::vector<DirichletValue> dirichlet;
  double time;
};

namespace assembly_detail {
inline bool zero(const Expr &e) { return fem::is_zero(e); }
inline double checked(double v) {
  if (!std::isfinite(v))
    throw std::domain_error("Nonfinite FEM assembly contribution");
  return v;
}
inline Expr edge_expression(const Expr &e, double normal_x, double normal_y,
                            discretization::Point2 a,
                            discretization::Point2 b) {
  // SymEngine substitution is simultaneous: x on the right is the new edge
  // coordinate s, not the original world coordinate. Restrict BEFORE deciding
  // equivalence or choosing a nonzero flux component for division.
  return SymEngine::expand(
      e.subs({{nx.get_basic(), Expr(normal_x).get_basic()},
              {ny.get_basic(), Expr(normal_y).get_basic()},
              {x.get_basic(), (Expr(a.x) + (b.x - a.x) * x).get_basic()},
              {y.get_basic(), (Expr(a.y) + (b.y - a.y) * x).get_basic()}}));
}
} // namespace assembly_detail

// Owned preparation is shared immutably; assemble() only allocates numeric
// outputs.
namespace assembly_detail {
inline bool time_dependent(const Expr &e) {
  for (const auto &s : SymEngine::free_symbols(*e.get_basic()))
    if (SymEngine::eq(*s, *t.get_basic()))
      return true;
  return false;
}
// Immutable preparation counters. Per-assembly interior evaluations count
// successful full calls; boundary validation/evaluation is deliberately
// separate.
struct PreparationStatistics {
  std::size_t elements = 0, edges = 0, physical_quadrature_points = 0,
              interior_constant_evaluations = 0,
              interior_evaluations_per_assembly = 0, evaluators = 0;
};
struct LocalBlock {
  std::array<double, 9> stiffness{}, first{}, second{};
  std::array<double, 3> load{};
};
struct PhysicalPoint {
  EvaluationPoint point;
  std::array<double, 3> phi;
  double weight;
  std::array<double, 9> constant{};
};
struct PreparedElement {
  std::array<std::size_t, 3> nodes;
  std::array<double, 3> gx, gy;
  std::vector<PhysicalPoint> points;
  LocalBlock constant;
};
struct PreparedEdge {
  std::array<std::size_t, 2> nodes;
  double length, normal_x, normal_y;
  std::array<Evaluator, 4> evaluators;
  std::array<Evaluator, 3> prescribed;
  std::array<std::exception_ptr, 3> prescribed_errors;
  bool dirichlet, constant;
  std::array<bool, 3> variable;
  LocalBlock block;
};
struct EdgeResult {
  LocalBlock block;
  std::array<double, 2> values{};
};
class PreparedData {
public:
  discretization::TriangleMesh mesh;
  Problem problem;
  AssemblyOptions options;
  PreparationStatistics statistics;
  std::vector<Evaluator> coefficients;
  std::array<bool, 9> dynamic{};
  std::vector<PreparedElement> elements;
  std::vector<PreparedEdge> edges;
  std::vector<EdgeQuadraturePoint> edge_rule;
  std::vector<double> divisor_samples;
  PreparedData(const discretization::TriangleMesh &input, const Problem &p,
               const AssemblyOptions &o)
      : mesh(input), problem(p), options(o) {
    pde2d::validate_execution(options.execution);
    if (!std::isfinite(options.boundary_tolerance) ||
        options.boundary_tolerance < 0)
      throw std::invalid_argument("Invalid FEM time or boundary tolerance");
    discretization::validate_mesh_geometry(mesh);
    const auto triangle_rule = triangle_quadrature(options.quadrature_degree);
    edge_rule = edge_quadrature(options.quadrature_degree);
    divisor_samples = {0, .5, 1};
    for (const auto &q : edge_rule)
      divisor_samples.push_back(q.coordinate);
    std::sort(divisor_samples.begin(), divisor_samples.end());
    if (mesh.nodes.size() >
        static_cast<std::size_t>(std::numeric_limits<int>::max()))
      throw std::invalid_argument("Mesh exceeds sparse matrix index capacity");
    const auto &eq = problem.equation;
    for (const auto *e : {&eq.a_xx, &eq.a_xy, &eq.a_yy, &eq.b_x, &eq.b_y, &eq.c,
                          &eq.source, &eq.m1, &eq.m2})
      for (const auto &s : SymEngine::free_symbols(*e->get_basic()))
        if (SymEngine::eq(*s, *nx.get_basic()) ||
            SymEngine::eq(*s, *ny.get_basic()))
          throw std::invalid_argument(
              "Interior coefficients cannot depend on boundary normals");
    const std::array<Expr, 9> expressions{
        eq.a_xx,
        eq.a_xy / 2,
        eq.a_yy,
        eq.b_x - diff(eq.a_xx, x) - diff(eq.a_xy, y) / 2,
        eq.b_y - diff(eq.a_yy, y) - diff(eq.a_xy, x) / 2,
        eq.c,
        eq.source,
        eq.m1,
        eq.m2};
    for (std::size_t k = 0; k < 9; ++k) {
      coefficients.emplace_back(expressions[k]);
      dynamic[k] = time_dependent(expressions[k]);
    }
    std::map<std::pair<std::size_t, std::size_t>, std::size_t> opposite;
    for (const auto &element : mesh.elements) {
      const auto a = mesh.nodes[element[0]], b = mesh.nodes[element[1]],
                 c = mesh.nodes[element[2]];
      const double twice_area =
          checked((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
      if (twice_area <= 0)
        throw std::invalid_argument("Unrepresentable triangle area");
      PreparedElement e{element,
                        {(b.y - c.y) / twice_area, (c.y - a.y) / twice_area,
                         (a.y - b.y) / twice_area},
                        {(c.x - b.x) / twice_area, (a.x - c.x) / twice_area,
                         (b.x - a.x) / twice_area},
                        {},
                        {}};
      for (int k = 0; k < 3; ++k)
        opposite[std::minmax(element[k], element[(k + 1) % 3])] =
            element[(k + 2) % 3];
      for (const auto &q : triangle_rule) {
        const auto &phi = q.barycentric;
        PhysicalPoint point{{phi[0] * a.x + phi[1] * b.x + phi[2] * c.x,
                             phi[0] * a.y + phi[1] * b.y + phi[2] * c.y, 0},
                            phi,
                            twice_area / 2 * q.weight,
                            {}};
        for (int k = 0; k < 9; ++k)
          if (!dynamic[k]) {
            point.constant[k] = coefficients[k](point.point);
            ++statistics.interior_constant_evaluations;
          } else
            ++statistics.interior_evaluations_per_assembly;
        ++statistics.physical_quadrature_points;
        e.points.push_back(point);
      }
      // The weak form is linear in these nine coefficients. Cache each
      // time-independent contribution, even inside a partly dynamic matrix.
      std::array<bool, 9> fixed;
      for (int k = 0; k < 9; ++k)
        fixed[k] = !dynamic[k];
      e.constant = element_block(e, 0, fixed);
      elements.push_back(std::move(e));
    }
    if (mesh.boundary_tags.size() != mesh.boundary_edges.size())
      throw std::invalid_argument(
          "Every boundary edge needs an explicit region tag and condition");
    std::set<std::string> present(mesh.boundary_tags.begin(),
                                  mesh.boundary_tags.end());
    if (present.contains(""))
      throw std::invalid_argument("Empty boundary region tag");
    std::map<std::string, const BoundaryCondition *> assigned;
    for (const auto &assignment : problem.boundaries) {
      if (assignment.regions.empty())
        throw std::invalid_argument("Empty boundary assignment");
      for (const auto &region : assignment.regions) {
        if (!present.contains(region))
          throw std::invalid_argument("Absent boundary region: " + region);
        if (!assigned.emplace(region, &assignment.condition).second)
          throw std::invalid_argument("Overlapping boundary assignments: " +
                                      region);
      }
    }

    for (std::size_t edge = 0; edge < mesh.boundary_edges.size(); ++edge) {
      const auto condition = assigned.find(mesh.boundary_tags[edge]);
      if (condition == assigned.end())
        throw std::invalid_argument("Missing boundary assignment: " +
                                    mesh.boundary_tags[edge]);
      const auto &bc = *condition->second;
      const std::array<std::size_t, 2> nodes{mesh.boundary_edges[edge].first,
                                             mesh.boundary_edges[edge].second};
      const auto a = mesh.nodes[nodes[0]], b = mesh.nodes[nodes[1]];
      const auto inside =
          mesh.nodes[opposite.at(std::minmax(nodes[0], nodes[1]))];
      const double dx = b.x - a.x, dy = b.y - a.y,
                   length = checked(std::hypot(dx, dy));
      if (length == 0)
        throw std::invalid_argument("Zero boundary edge length");
      double normal_x = dy / length, normal_y = -dx / length;
      if (normal_x * (inside.x - a.x) + normal_y * (inside.y - a.y) > 0) {
        normal_x = -normal_x;
        normal_y = -normal_y;
      }
      const auto on_edge = [&](const Expr &e) {
        return assembly_detail::edge_expression(e, normal_x, normal_y, a, b);
      };
      const Expr alpha = on_edge(bc.alpha), gamma = on_edge(bc.gamma),
                 value = on_edge(bc.value);
      const Expr flux_x = on_edge(-nx * eq.a_xx - ny * eq.a_xy / 2);
      const Expr flux_y = on_edge(-nx * eq.a_xy / 2 - ny * eq.a_yy);
      const Expr extra_x = on_edge(bc.beta_normal * nx + bc.beta_x);
      const Expr extra_y = on_edge(bc.beta_normal * ny + bc.beta_y);
      Expr beta = on_edge(bc.beta_flux);
      if (!assembly_detail::zero(extra_x) || !assembly_detail::zero(extra_y)) {
        if (!assembly_detail::zero(extra_x * flux_y - extra_y * flux_x) ||
            (assembly_detail::zero(flux_x) && assembly_detail::zero(flux_y)))
          throw std::invalid_argument(
              "Unsupported oblique/normal derivative boundary condition: "
              "derivative vector is not algebraically proportional to natural "
              "flux");
        // Project only AFTER proving exact proportionality. This quotient
        // avoids a removable 0/0 where one component vanishes but the flux
        // vector does not.
        beta = beta + (extra_x * flux_x + extra_y * flux_y) /
                          (flux_x * flux_x + flux_y * flux_y);
      }

      const bool dirichlet = zero(beta);
      if (dirichlet && !zero(gamma))
        throw std::invalid_argument(
            "Derivative-free dynamic boundary conditions are unsupported");
      if (dirichlet && zero(alpha))
        throw std::invalid_argument(
            "Boundary condition has no nonzero operator");
      const Expr g = dirichlet ? value / alpha : Expr(0);
      PreparedEdge e{
          nodes,
          length,
          normal_x,
          normal_y,
          {Evaluator(alpha), Evaluator(gamma), Evaluator(value),
           Evaluator(beta)},
          {Evaluator(g), Evaluator(0), Evaluator(0)},
          {},
          dirichlet,
          !time_dependent(alpha) && !time_dependent(gamma) &&
              !time_dependent(value) && !time_dependent(beta),
          {time_dependent(alpha) || time_dependent(beta),
           time_dependent(gamma) || time_dependent(beta),
           time_dependent(value) || time_dependent(beta)},
          {}};
      // Prepare supported derivatives serially, but preserve the value-only
      // API when the symbolic library cannot evaluate an unused derivative.
      // Re-throw only if that derivative is actually requested. Workers never
      // construct/copy symbolic expressions or mutate a lazy cache.
      if (e.dirichlet)
        for (int order = 1; order < 3; ++order) {
          try {
            Expr derivative = g;
            for (int k = 0; k < order; ++k)
              derivative = diff(derivative, t);
            e.prescribed[order] = Evaluator(derivative);
          } catch (const std::bad_alloc &) {
            throw;
          } catch (...) {
            e.prescribed_errors[order] = std::current_exception();
          }
        }
      if (!e.dirichlet)
        e.block = edge_block(e, 0, 0, true).block;
      edges.push_back(std::move(e));
    }
    statistics.elements = elements.size();
    statistics.edges = edges.size();
    statistics.evaluators = coefficients.size() + 7 * edges.size();
  }
  LocalBlock element_block(const PreparedElement &e, double time,
                           const std::array<bool, 9> &enabled) const {
    LocalBlock out;
    if (std::none_of(enabled.begin(), enabled.end(), [](bool v) { return v; }))
      return out;
    const std::array<bool, 4> active{std::any_of(enabled.begin(),
                                                 enabled.begin() + 6,
                                                 [](bool v) { return v; }),
                                     enabled[7], enabled[8], enabled[6]};
    for (const auto &q : e.points) {
      std::array<double, 9> v{};
      auto point = q.point;
      point.t = time;
      for (int k = 0; k < 9; ++k)
        if (enabled[k])
          v[k] = dynamic[k] ? coefficients[k](point) : q.constant[k];
      const auto &phi = q.phi;
      const auto &gx = e.gx;
      const auto &gy = e.gy;
      const double w = q.weight;
      for (int i = 0; i < 3; ++i) {
        if (active[3])
          out.load[i] = checked(out.load[i] + w * v[6] * phi[i]);
        for (int j = 0; j < 3; ++j) {
          int ij = 3 * i + j;
          if (active[0])
            out.stiffness[ij] =
                checked(out.stiffness[ij] +
                        w * (-v[0] * gx[i] * gx[j] -
                             v[1] * (gx[i] * gy[j] + gy[i] * gx[j]) -
                             v[2] * gy[i] * gy[j] +
                             phi[i] * (v[3] * gx[j] + v[4] * gy[j]) +
                             v[5] * phi[i] * phi[j]));
          if (active[1])
            out.first[ij] = checked(out.first[ij] + w * v[7] * phi[i] * phi[j]);
          if (active[2])
            out.second[ij] =
                checked(out.second[ij] + w * v[8] * phi[i] * phi[j]);
        }
      }
    }
    return out;
  }
  EdgeResult edge_block(const PreparedEdge &e, double time, int order,
                        bool cache_only = false) const {
    EdgeResult result;
    const auto &alpha = e.evaluators[0];
    const auto &gamma = e.evaluators[1];
    const auto &value = e.evaluators[2];
    const auto &beta = e.evaluators[3];
    auto at = [&](double s) {
      return EvaluationPoint{s, 0, time, e.normal_x, e.normal_y};
    };
    // Preserve endpoint/midpoint/quadrature sign sampling. This is not a proof
    // that arbitrary symbolic functions have no unsampled roots.
    const auto &divisor = e.dirichlet ? alpha : beta;
    if (!cache_only || std::none_of(e.variable.begin(), e.variable.end(),
                                    [](bool v) { return v; })) {
      double previous = divisor(at(divisor_samples.front()));
      for (double s : divisor_samples) {
        const double current = divisor(at(s));
        if (current == 0 || std::signbit(current) != std::signbit(previous))
          throw std::invalid_argument(
              std::string(e.dirichlet ? "Dirichlet alpha"
                                      : "Natural flux multiplier") +
              " vanishes or changes sign on boundary");
        previous = current;
      }
    }
    if (e.dirichlet) {
      if (e.prescribed_errors[order])
        std::rethrow_exception(e.prescribed_errors[order]);
      for (int k = 0; k < 2; ++k)
        result.values[k] = e.prescribed[order](at(k));
      for (const auto &q : edge_rule)
        (void)value(at(q.coordinate));
    } else {
      if (!cache_only || e.constant)
        for (int k = 0; k < 2; ++k) {
          (void)alpha(at(k));
          (void)gamma(at(k));
          (void)value(at(k));
        }
      for (const auto &q : edge_rule) {
        const auto p = at(q.coordinate);
        const auto active = [&](int k) {
          return cache_only ? !e.variable[k] : e.variable[k];
        };
        if (!active(0) && !active(1) && !active(2))
          continue;
        const double bv = beta(p);
        const double av = active(0) ? checked(alpha(p) / bv) : 0,
                     gv = active(1) ? checked(gamma(p) / bv) : 0,
                     f = active(2) ? checked(value(p) / bv) : 0,
                     w = e.length * q.weight;
        const std::array<double, 2> phi{1 - q.coordinate, q.coordinate};
        for (int i = 0; i < 2; ++i) {
          result.block.load[i] = checked(result.block.load[i] + w * f * phi[i]);
          for (int j = 0; j < 2; ++j) {
            result.block.stiffness[3 * i + j] = checked(
                result.block.stiffness[3 * i + j] + w * av * phi[i] * phi[j]);
            result.block.first[3 * i + j] = checked(
                result.block.first[3 * i + j] + w * gv * phi[i] * phi[j]);
          }
        }
      }
    }
    return result;
  }
  AssembledSystem assemble(double time, int order) const {
    if (!std::isfinite(time))
      throw std::invalid_argument("Invalid FEM time or boundary tolerance");
    if (order < 0 || order > 2)
      throw std::invalid_argument(
          "Dirichlet derivative order must be in [0,2]");
    // Workers only borrow prepared evaluators and write their own numeric slot.
    // Evaluator CSE is disabled, so calls do not write visitor scratch storage.
    std::vector<LocalBlock> blocks(elements.size());
    pde2d::parallel_for(elements.size(), options.execution, [&](std::size_t i) {
      blocks[i] = element_block(elements[i], time, dynamic);
    });
    std::vector<EdgeResult> boundary(edges.size());
    pde2d::parallel_for(edges.size(), options.execution, [&](std::size_t i) {
      const auto &e = edges[i];
      if (e.constant && !e.dirichlet)
        boundary[i].block = e.block;
      else {
        boundary[i] = edge_block(e, time, order);
        if (!e.dirichlet) {
          for (int k = 0; k < 9; ++k) {
            boundary[i].block.stiffness[k] += e.block.stiffness[k];
            boundary[i].block.first[k] += e.block.first[k];
          }
          for (int k = 0; k < 3; ++k)
            boundary[i].block.load[k] += e.block.load[k];
        }
      }
    });
    const Eigen::Index count = mesh.nodes.size();
    AssembledSystem result{SparseMatrix(count, count),
                           SparseMatrix(count, count),
                           SparseMatrix(count, count),
                           Vector::Zero(count),
                           {},
                           time};
    std::vector<Eigen::Triplet<double>> stiffness, first, second;
    stiffness.reserve(9 * elements.size() + 4 * edges.size());
    first.reserve(stiffness.capacity());
    second.reserve(9 * elements.size());
    auto add = [](auto &v, std::size_t i, std::size_t j, double a) {
      if (checked(a) != 0)
        v.emplace_back(i, j, a);
    };
    for (std::size_t n = 0; n < elements.size(); ++n) {
      const auto &e = elements[n];
      const auto &a = blocks[n];
      const auto &b = e.constant;
      for (int i = 0; i < 3; ++i) {
        result.load[e.nodes[i]] =
            checked(result.load[e.nodes[i]] + a.load[i] + b.load[i]);
        for (int j = 0; j < 3; ++j) {
          int ij = 3 * i + j;
          add(stiffness, e.nodes[i], e.nodes[j],
              a.stiffness[ij] + b.stiffness[ij]);
          add(first, e.nodes[i], e.nodes[j], a.first[ij] + b.first[ij]);
          add(second, e.nodes[i], e.nodes[j], a.second[ij] + b.second[ij]);
        }
      }
    }
    // Stable element/edge ordering also fixes sparse duplicate summation order.
    std::map<Eigen::Index, double> prescribed;
    for (std::size_t n = 0; n < edges.size(); ++n) {
      const auto &e = edges[n];
      const auto &a = boundary[n];
      for (int i = 0; i < 2; ++i) {
        if (e.dirichlet) {
          double g = a.values[i];
          auto [it, inserted] = prescribed.emplace(e.nodes[i], g);
          if (!inserted &&
              std::abs(it->second - g) >
                  options.boundary_tolerance *
                      std::max({1., std::abs(it->second), std::abs(g)}))
            throw std::invalid_argument(
                "Contradictory corner Dirichlet values");
        } else {
          result.load[e.nodes[i]] =
              checked(result.load[e.nodes[i]] + a.block.load[i]);
          for (int j = 0; j < 2; ++j) {
            add(stiffness, e.nodes[i], e.nodes[j],
                a.block.stiffness[3 * i + j]);
            add(first, e.nodes[i], e.nodes[j], a.block.first[3 * i + j]);
          }
        }
      }
    }
    for (const auto &[node, value] : prescribed)
      result.dirichlet.push_back({node, value});
    result.stiffness.setFromTriplets(stiffness.begin(), stiffness.end());
    result.first_mass.setFromTriplets(first.begin(), first.end());
    result.second_mass.setFromTriplets(second.begin(), second.end());
    for (const auto *matrix :
         {&result.stiffness, &result.first_mass, &result.second_mass})
      for (int k = 0; k < matrix->nonZeros(); ++k)
        checked(matrix->valuePtr()[k]);
    return result;
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
  AssembledSystem assemble(double time = 0,
                           int dirichlet_derivative_order = 0) const {
    return data_->assemble(time, dirichlet_derivative_order);
  }
  const discretization::TriangleMesh &mesh() const { return data_->mesh; }
  const Problem &problem() const { return data_->problem; }
  const AssemblyOptions &options() const { return data_->options; }
  const assembly_detail::PreparationStatistics &statistics() const {
    return data_->statistics;
  }
};
namespace assembly_detail {
inline AssembledSystem assemble_impl(const discretization::TriangleMesh &mesh,
                                     const Problem &problem, double time,
                                     const AssemblyOptions &options,
                                     int order = 0) {
  return PreparedAssembly(mesh, problem, options).assemble(time, order);
}
} // namespace assembly_detail
inline AssembledSystem assemble(const discretization::TriangleMesh &mesh,
                                const Problem &problem, double time = 0,
                                const AssemblyOptions &options = {}) {
  return assembly_detail::assemble_impl(mesh, problem, time, options);
}
} // namespace fem

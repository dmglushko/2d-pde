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
#include "boundary_classification.hpp"
#include "discretization/mesh_validation.hpp"
#include "execution.hpp"
#include "p1_element.hpp"
#include "problem.hpp"
#include "quadrature.hpp"
#include "weak_form.hpp"
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

namespace pde2d::fem {
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
// stiffness*u + first_mass*u_t + second_mass*u_tt = load, plus Dirichlet
// values (already reduced at shared corners) that constrain() imposes.
struct AssembledSystem {
  SparseMatrix stiffness, first_mass, second_mass;
  Vector load;
  std::vector<DirichletValue> dirichlet;
  double time;
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
using p1::checked;
using p1::LocalBlock;
struct PhysicalPoint {
  EvaluationPoint point;
  std::array<double, 3> phi;
  double weight;
  WeakCoefficients<double> constant{};
};
struct PreparedElement {
  std::array<std::size_t, 3> nodes;
  p1::TriangleGeometry geometry;
  std::vector<PhysicalPoint> points;
  LocalBlock constant;
};
struct PreparedEdge {
  std::array<std::size_t, 2> nodes;
  double length, normal_x, normal_y;
  std::array<Evaluator, 4> evaluators; // alpha, gamma, value, beta
  // Prescribed Dirichlet value and its first/second time derivatives.
  std::array<Evaluator, 3> prescribed;
  std::array<std::exception_ptr, 3> prescribed_errors;
  bool dirichlet, constant;
  // Time dependence of the stiffness, first-mass and load contributions.
  std::array<bool, 3> variable;
  LocalBlock block;
};
struct EdgeResult {
  LocalBlock block;
  std::array<double, 2> values{};
};

// Map every boundary region present in the mesh to its single condition.
// Regions without an assignment are reported per edge by the caller.
inline std::map<std::string, const BoundaryCondition *>
region_conditions(const discretization::TriangleMesh &mesh,
                  const Problem &problem) {
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
  return assigned;
}

class PreparedData {
public:
  discretization::TriangleMesh mesh;
  Problem problem;
  AssemblyOptions options;
  AssemblyStatistics statistics;
  std::vector<Evaluator> coefficients;
  WeakFlags dynamic{};
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
    edge_rule = edge_quadrature(options.quadrature_degree);
    divisor_samples = {0, .5, 1};
    for (const auto &q : edge_rule)
      divisor_samples.push_back(q.coordinate);
    std::sort(divisor_samples.begin(), divisor_samples.end());
    if (mesh.nodes.size() >
        static_cast<std::size_t>(std::numeric_limits<int>::max()))
      throw std::invalid_argument("Mesh exceeds sparse matrix index capacity");
    validate_interior_equation(problem.equation);
    const auto expressions = weak_form_coefficients(problem.equation);
    for (std::size_t k = 0; k < weak::Count; ++k) {
      coefficients.emplace_back(expressions[k]);
      dynamic[k] = depends_on(expressions[k], t);
    }
    const auto opposite = prepare_elements();
    prepare_edges(opposite);
    statistics.elements = elements.size();
    statistics.edges = edges.size();
    statistics.evaluators = coefficients.size() + 7 * edges.size();
  }

  // Interior vertex opposite each element edge, keyed by sorted endpoints.
  using OppositeVertices =
      std::map<std::pair<std::size_t, std::size_t>, std::size_t>;

  OppositeVertices prepare_elements() {
    const auto triangle_rule = triangle_quadrature(options.quadrature_degree);
    OppositeVertices opposite;
    for (const auto &element : mesh.elements) {
      const auto a = mesh.nodes[element[0]], b = mesh.nodes[element[1]],
                 c = mesh.nodes[element[2]];
      PreparedElement e{element, p1::triangle_geometry(a, b, c), {}, {}};
      const double twice_area = e.geometry.twice_area;
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
        for (std::size_t k = 0; k < weak::Count; ++k)
          if (!dynamic[k]) {
            point.constant[k] = coefficients[k](point.point);
            ++statistics.interior_constant_evaluations;
          } else
            ++statistics.interior_evaluations_per_assembly;
        ++statistics.physical_quadrature_points;
        e.points.push_back(point);
      }
      // The weak form is linear in its coefficients. Cache each
      // time-independent contribution, even inside a partly dynamic matrix.
      WeakFlags fixed;
      for (std::size_t k = 0; k < weak::Count; ++k)
        fixed[k] = !dynamic[k];
      e.constant = element_block(e, 0, fixed);
      elements.push_back(std::move(e));
    }
    return opposite;
  }

  void prepare_edges(const OppositeVertices &opposite) {
    const auto assigned = region_conditions(mesh, problem);
    for (std::size_t edge = 0; edge < mesh.boundary_edges.size(); ++edge) {
      const auto condition = assigned.find(mesh.boundary_tags[edge]);
      if (condition == assigned.end())
        throw std::invalid_argument("Missing boundary assignment: " +
                                    mesh.boundary_tags[edge]);
      const std::array<std::size_t, 2> nodes{mesh.boundary_edges[edge].first,
                                             mesh.boundary_edges[edge].second};
      const auto frame = p1::edge_frame(
          mesh.nodes[nodes[0]], mesh.nodes[nodes[1]],
          mesh.nodes[opposite.at(std::minmax(nodes[0], nodes[1]))]);
      const auto c =
          classify_edge_condition(problem.equation, *condition->second, frame);
      const auto varies = [](const Expr &e) { return depends_on(e, t); };
      PreparedEdge e{nodes,
                     frame.length,
                     frame.normal_x,
                     frame.normal_y,
                     {Evaluator(c.alpha), Evaluator(c.gamma),
                      Evaluator(c.value), Evaluator(c.beta)},
                     {Evaluator(c.prescribed), Evaluator(0), Evaluator(0)},
                     {},
                     c.dirichlet,
                     !varies(c.alpha) && !varies(c.gamma) && !varies(c.value) &&
                         !varies(c.beta),
                     {varies(c.alpha) || varies(c.beta),
                      varies(c.gamma) || varies(c.beta),
                      varies(c.value) || varies(c.beta)},
                     {}};
      // Prepare supported derivatives serially, but preserve the value-only
      // API when the symbolic library cannot evaluate an unused derivative.
      // Re-throw only if that derivative is actually requested. Workers never
      // construct/copy symbolic expressions or mutate a lazy cache.
      if (e.dirichlet)
        for (int order = 1; order < 3; ++order) {
          try {
            Expr derivative = c.prescribed;
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
  }

  LocalBlock element_block(const PreparedElement &e, double time,
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

  // cache_only=true computes the time-independent part at preparation;
  // otherwise only the time-dependent part (added to the cache by assemble).
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
        p1::accumulate_edge_point(result.block, q.coordinate, w, av, gv, f);
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
    return scatter(time, blocks, boundary);
  }

  AssembledSystem scatter(double time, const std::vector<LocalBlock> &blocks,
                          const std::vector<EdgeResult> &boundary) const {
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
  // dirichlet_derivative_order selects the Dirichlet data: 0 = values,
  // 1 = first time derivative, 2 = second (used to start Newmark).
  AssembledSystem assemble(double time = 0,
                           int dirichlet_derivative_order = 0) const {
    return data_->assemble(time, dirichlet_derivative_order);
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

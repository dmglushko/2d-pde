#pragma once
// Private part of PreparedAssembly (assembly.hpp): boundary edges.
//
// Preparation reduces each edge's boundary condition to Dirichlet or natural
// form (boundary_classification.hpp), compiles the restricted coefficients
// and caches the time-independent natural contribution. block() evaluates the
// remaining part at one time; it only reads prepared data.
#include "discretization/geometry_base.hpp"
#include "fem/boundary_classification.hpp"
#include "fem/detail/assembly_elements.hpp"
#include "fem/expression.hpp"
#include "fem/p1_element.hpp"
#include "fem/problem.hpp"
#include "fem/quadrature.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <map>
#include <new>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::fem::assembly_detail {
// Restricted coefficients of  beta q_n + alpha u + gamma u_t = value.
struct EdgeCoefficients {
  Evaluator alpha, gamma, value, beta;
  template <class F> void for_each(F &&f) const {
    f(alpha);
    f(gamma);
    f(value);
    f(beta);
  }
};

// One prescribed Dirichlet time derivative. When the symbolic library could
// not form it, `error` keeps the exception, rethrown only if it is requested.
struct PreparedDerivative {
  Evaluator evaluator;
  std::exception_ptr error{};
};
// Prescribed Dirichlet value g and its time derivatives g_t and g_tt.
struct DirichletData {
  PreparedDerivative value, first_derivative, second_derivative;
  // Derivative of time order 0, 1 or 2 (validated by the caller).
  const PreparedDerivative &derivative(int order) const {
    return order == 0 ? value : order == 1 ? first_derivative
                                           : second_derivative;
  }
  PreparedDerivative &derivative(int order) {
    return order == 0 ? value : order == 1 ? first_derivative
                                           : second_derivative;
  }
  template <class F> void for_each(F &&f) const {
    f(value.evaluator);
    f(first_derivative.evaluator);
    f(second_derivative.evaluator);
  }
};

// Which natural-boundary contributions depend on time.
struct EdgeVariation {
  bool stiffness = false, first_mass = false, load = false;
  bool any() const { return stiffness || first_mass || load; }
};

struct PreparedEdge {
  std::array<std::size_t, 2> nodes;
  double length, normal_x, normal_y;
  EdgeCoefficients coefficients;
  DirichletData prescribed;
  bool dirichlet;
  EdgeVariation variable;
  // Cached time-independent natural contribution (leading 2x2 block).
  LocalBlock block;

  std::size_t evaluator_count() const {
    std::size_t count = 0;
    const auto add = [&](const Evaluator &) { ++count; };
    coefficients.for_each(add);
    prescribed.for_each(add);
    return count;
  }
};
struct EdgeResult {
  LocalBlock block;
  // Dirichlet values at the two edge nodes.
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

// Add the time-independent natural contribution cached at preparation.
inline void add_cached(LocalBlock &out, const LocalBlock &cached) {
  for (std::size_t k = 0; k < out.stiffness.size(); ++k) {
    out.stiffness[k] += cached.stiffness[k];
    out.first[k] += cached.first[k];
  }
  for (std::size_t k = 0; k < out.load.size(); ++k)
    out.load[k] += cached.load[k];
}

class PreparedBoundary {
public:
  std::vector<PreparedEdge> edges;
  std::vector<EdgeQuadraturePoint> rule;
  // Edge coordinates where the Dirichlet alpha or natural flux multiplier
  // must keep one strict sign.
  std::vector<double> divisor_samples;

  // Set up the edge rule (validates the quadrature degree).
  void set_rule(int quadrature_degree) {
    rule = edge_quadrature(quadrature_degree);
    divisor_samples = {0, .5, 1};
    for (const auto &q : rule)
      divisor_samples.push_back(q.coordinate);
    std::sort(divisor_samples.begin(), divisor_samples.end());
  }

  // Classify and prepare every boundary edge (after set_rule()).
  void prepare(const discretization::TriangleMesh &mesh,
               const Problem &problem, const OppositeVertices &opposite) {
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
      PreparedEdge e{
          nodes,
          frame.length,
          frame.normal_x,
          frame.normal_y,
          {Evaluator(c.alpha), Evaluator(c.gamma), Evaluator(c.value),
           Evaluator(c.beta)},
          {{Evaluator(c.prescribed)}, {Evaluator(0)}, {Evaluator(0)}},
          c.dirichlet,
          {.stiffness = varies(c.alpha) || varies(c.beta),
           .first_mass = varies(c.gamma) || varies(c.beta),
           .load = varies(c.value) || varies(c.beta)},
          {}};
      // Prepare supported derivatives serially, but preserve the value-only
      // API when the symbolic library cannot evaluate an unused derivative.
      // Re-throw only if that derivative is actually requested. Workers never
      // construct/copy symbolic expressions or mutate a lazy cache.
      if (e.dirichlet)
        for (int order = 1; order < 3; ++order) {
          auto &prepared = e.prescribed.derivative(order);
          try {
            Expr derivative = c.prescribed;
            for (int k = 0; k < order; ++k)
              derivative = diff(derivative, t);
            prepared.evaluator = Evaluator(derivative);
          } catch (const std::bad_alloc &) {
            throw;
          } catch (...) {
            prepared.error = std::current_exception();
          }
        }
      if (!e.dirichlet)
        e.block = block(e, 0, 0, true).block;
      edges.push_back(std::move(e));
    }
  }

  // cache_only=true computes the time-independent part at preparation;
  // otherwise only the time-dependent part (added to the cache by evaluate).
  EdgeResult block(const PreparedEdge &e, double time, int order,
                   bool cache_only = false) const {
    EdgeResult result;
    const auto &[alpha, gamma, value, beta] = e.coefficients;
    auto at = [&](double s) {
      return EvaluationPoint{s, 0, time, e.normal_x, e.normal_y};
    };
    // Preserve endpoint/midpoint/quadrature sign sampling. This is not a proof
    // that arbitrary symbolic functions have no unsampled roots.
    const auto &divisor = e.dirichlet ? alpha : beta;
    if (!cache_only || !e.variable.any()) {
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
      const auto &prescribed = e.prescribed.derivative(order);
      if (prescribed.error)
        std::rethrow_exception(prescribed.error);
      for (std::size_t k = 0; k < result.values.size(); ++k)
        result.values[k] = prescribed.evaluator(at(static_cast<double>(k)));
      for (const auto &q : rule)
        (void)value(at(q.coordinate));
    } else {
      if (!cache_only || !e.variable.any())
        for (int k = 0; k < 2; ++k) {
          (void)alpha(at(k));
          (void)gamma(at(k));
          (void)value(at(k));
        }
      // Prepared terms are the time-independent ones, assembled terms the
      // time-dependent ones.
      const auto active = [&](bool variable) {
        return cache_only ? !variable : variable;
      };
      const bool stiffness = active(e.variable.stiffness),
                 first_mass = active(e.variable.first_mass),
                 load = active(e.variable.load);
      if (!stiffness && !first_mass && !load)
        return result;
      for (const auto &q : rule) {
        const auto p = at(q.coordinate);
        const double bv = beta(p);
        const double av = stiffness ? checked(alpha(p) / bv) : 0,
                     gv = first_mass ? checked(gamma(p) / bv) : 0,
                     f = load ? checked(value(p) / bv) : 0,
                     w = e.length * q.weight;
        p1::accumulate_edge_point(result.block, q.coordinate, w, av, gv, f);
      }
    }
    return result;
  }

  // Complete contribution of one edge at `time`: Dirichlet values of the
  // requested derivative order, or the cached plus time-dependent natural
  // block.
  EdgeResult evaluate(const PreparedEdge &e, double time, int order) const {
    if (!e.variable.any() && !e.dirichlet)
      return {e.block, {}};
    auto result = block(e, time, order);
    if (!e.dirichlet)
      add_cached(result.block, e.block);
    return result;
  }
};
} // namespace pde2d::fem::assembly_detail

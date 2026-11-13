#include "pde2d/fem/meshed_problem.hpp"

#include "derivative.hpp"
#include "fact_summaries.hpp"
#include "symengine_expression.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <format>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::fem {

namespace {

constexpr std::size_t none = std::numeric_limits<std::size_t>::max();

std::string in_quotes(const std::string &name) { return "'" + name + "'"; }

std::string node_text(const mesh::Mesh &mesh, std::size_t node) {
  const auto &p = mesh.nodes[node];
  return std::format("node {} ({}, {})", node, p.x, p.y);
}

// Compiles one expression; an error names the coefficient it belongs to.
CompiledExpression compile(const Expression &expression,
                           const Parameters &parameters,
                           const std::string &label) {
  try {
    return CompiledExpression(expression, parameters);
  } catch (const std::invalid_argument &e) {
    throw std::invalid_argument(label + ": " + e.what());
  }
}

// Step 1: every condition names a mesh region and every mesh region has a
// condition.
void match_regions(const mesh::Mesh &mesh, const Problem &problem) {
  for (const auto &[name, condition] : problem.boundary_conditions) {
    bool found = false;
    for (const auto &region : mesh.region_names)
      found = found || region == name;
    if (!found) {
      std::string names;
      for (const auto &region : mesh.region_names)
        names += (names.empty() ? "" : ", ") + in_quotes(region);
      throw std::invalid_argument(
          "There is a boundary condition for region " + in_quotes(name) +
          ", but the mesh has no region of that name (mesh regions: " +
          (names.empty() ? std::string("none") : names) + ")");
    }
  }
  for (const auto &region : mesh.region_names)
    if (!problem.boundary_conditions.contains(region))
      throw std::invalid_argument("Mesh region " + in_quotes(region) +
                                  " has no boundary condition");
}

// A corner between two Dirichlet regions, kept from step 3 until the
// values can be compared after compiling.
struct DirichletCorner {
  std::size_t node = 0;
  std::size_t first = 0, second = 0; // region ids, first < second
  bool averaged = false;             // else the higher priority won
};

double condition_value(const CompiledCondition &condition,
                       const geometry::Point &p, double t) {
  return condition.g(p.x, p.y, t) / condition.alpha(p.x, p.y, t);
}

double checked_value(const mesh::Mesh &mesh, const CompiledCondition &condition,
                     std::size_t region, std::size_t node, double t) {
  const double value = condition_value(condition, mesh.nodes[node], t);
  if (!std::isfinite(value))
    throw std::invalid_argument(std::format(
        "The Dirichlet value g / alpha of region {} is not finite at {}, "
        "t = {}",
        in_quotes(mesh.region_names[region]), node_text(mesh, node), t));
  return value;
}

// The average of value(r) over `regions`, computed without overflow:
// (a + b) / 2 is infinite for a = b = 1e308, std::midpoint is not. A node
// has one region or two; more are averaged as a sum of value / n.
template <class Value>
double average(const std::vector<std::size_t> &regions, Value value) {
  if (regions.size() == 1)
    return value(regions[0]);
  if (regions.size() == 2)
    return std::midpoint(value(regions[0]), value(regions[1]));
  double sum = 0.0;
  for (const std::size_t r : regions)
    sum += value(r) / static_cast<double>(regions.size());
  return sum;
}

bool depends_on_t(const BoundaryCondition &condition) {
  return condition.alpha.facts().depends_on_t ||
         condition.g.facts().depends_on_t;
}

// The report message for a corner whose competing values differ, or
// nothing when they are the same.
std::optional<std::string>
corner_message(const mesh::Mesh &mesh, const Problem &problem,
               const std::vector<CompiledCondition> &conditions,
               const DirichletCorner &corner) {
  const auto &name_a = mesh.region_names[corner.first];
  const auto &name_b = mesh.region_names[corner.second];
  const auto &source_a = problem.boundary_conditions.at(name_a);
  const auto &source_b = problem.boundary_conditions.at(name_b);
  if (detail::same_as_written(source_a.alpha, source_b.alpha) &&
      detail::same_as_written(source_a.g, source_b.g))
    return std::nullopt;
  const auto &a = conditions[corner.first];
  const auto &b = conditions[corner.second];
  // The winner first when one region wins.
  const bool b_wins = !corner.averaged && b.priority > a.priority;
  const auto &winner_name = b_wins ? name_b : name_a;
  const auto &loser_name = b_wins ? name_a : name_b;
  const int winner_priority = b_wins ? b.priority : a.priority;
  const int loser_priority = b_wins ? a.priority : b.priority;
  const std::string where = "Corner at " + node_text(mesh, corner.node);

  if (depends_on_t(source_a) || depends_on_t(source_b)) {
    if (corner.averaged)
      return std::format("{}: regions {} and {} (both priority {}) have "
                         "values that depend on t and may differ; their "
                         "average is used",
                         where, in_quotes(name_a), in_quotes(name_b),
                         a.priority);
    return std::format("{}: region {} (priority {}) wins over region {} "
                       "(priority {}); their values depend on t and may "
                       "differ",
                       where, in_quotes(winner_name), winner_priority,
                       in_quotes(loser_name), loser_priority);
  }
  // Only the values that are used must be finite: a losing value is only
  // reported.
  const auto &p = mesh.nodes[corner.node];
  const double value_a =
      corner.averaged || !b_wins
          ? checked_value(mesh, a, corner.first, corner.node, 0.0)
          : condition_value(a, p, 0.0);
  const double value_b =
      corner.averaged || b_wins
          ? checked_value(mesh, b, corner.second, corner.node, 0.0)
          : condition_value(b, p, 0.0);
  if (value_a == value_b)
    return std::nullopt;
  if (corner.averaged)
    return std::format("{}: regions {} and {} (both priority {}) give {} and "
                       "{}; their average {} is used",
                       where, in_quotes(name_a), in_quotes(name_b), a.priority,
                       value_a, value_b, std::midpoint(value_a, value_b));
  return std::format("{}: region {} (priority {}) gives {} and wins over "
                     "region {} (priority {}), which gives {}",
                     where, in_quotes(winner_name), winner_priority,
                     b_wins ? value_b : value_a, in_quotes(loser_name),
                     loser_priority, b_wins ? value_a : value_b);
}

// Step 5 for one initial condition: compiled when the time order needs
// it. A missing one that is needed is an error; one that is not needed,
// or one that depends on t, is a report message.
std::optional<CompiledExpression>
compile_initial(const Problem &problem,
                const std::optional<Expression> &condition, bool needed,
                const std::string &name, const char *evaluated_by,
                std::vector<std::string> &messages) {
  const int order = time_order(problem);
  if (needed && !condition)
    throw std::invalid_argument(
        std::format("The problem has time order {}, so it needs the initial "
                    "condition {}",
                    order, name));
  if (!needed) {
    if (condition)
      messages.push_back(std::format("The problem has time order {}, so the "
                                     "initial condition {} is not needed; it "
                                     "is ignored",
                                     order, name));
    return std::nullopt;
  }
  if (condition->facts().depends_on_t)
    messages.push_back(std::format("The initial condition {} depends on t; "
                                   "{}(t0) evaluates it at the start time t0",
                                   name, evaluated_by));
  return compile(*condition, problem.parameters, "Initial condition " + name);
}

void check_start_time(double t0) {
  if (!std::isfinite(t0))
    throw std::invalid_argument(
        std::format("The start time t0 = {} is not finite", t0));
}

// An initial condition at the free nodes at t0; the Dirichlet nodes are
// filled by the caller.
std::vector<double> at_free_nodes(const mesh::Mesh &mesh,
                                  const std::vector<std::size_t> &free_nodes,
                                  const CompiledExpression &condition,
                                  const char *name, double t0) {
  std::vector<double> values(mesh.nodes.size(), 0.0);
  for (const std::size_t node : free_nodes) {
    const auto &p = mesh.nodes[node];
    const double value = condition(p.x, p.y, t0);
    if (!std::isfinite(value))
      throw std::invalid_argument(
          std::format("The initial condition {} is not finite at {}, t = {}",
                      name, node_text(mesh, node), t0));
    values[node] = value;
  }
  return values;
}

// The average over the regions of `entry` at t of a compiled time
// derivative of g / alpha (one per region); `what` names it in errors.
double averaged_derivative(const mesh::Mesh &mesh,
                           const std::vector<CompiledExpression> &derivatives,
                           const DirichletNode &entry, double t,
                           const char *what) {
  const auto &p = mesh.nodes[entry.node];
  const double value = average(entry.regions, [&](std::size_t r) {
    const double derivative = derivatives[r](p.x, p.y, t);
    if (!std::isfinite(derivative))
      throw std::invalid_argument(std::format(
          "The {} of g / alpha of region {} is not finite at {}, t = {}", what,
          in_quotes(mesh.region_names[r]), node_text(mesh, entry.node), t));
    return derivative;
  });
  if (!std::isfinite(value))
    throw std::invalid_argument(
        std::format("The average {} of g / alpha is not finite at {}, t = {}",
                    what, node_text(mesh, entry.node), t));
  return value;
}

constexpr const char *rate_name = "time derivative";
constexpr const char *acceleration_name = "second time derivative";

// How many differing nodes a mismatch message lists.
constexpr std::size_t listed_mismatches = 3;

// Compares an initial condition with the boundary data that replaces it at
// the Dirichlet nodes (`data(k)` at dirichlet_nodes[k]) and adds a report
// message when they differ. When neither depends on t both are evaluated
// at t = 0 (any t gives the same values) and compared exactly at every
// node; otherwise they are reported once as possibly different. Returns
// the number of nodes reported: those that differ, or every Dirichlet node
// when they may differ.
template <class Data>
std::size_t compare_with_boundary_data(const mesh::Mesh &mesh,
                                       const std::vector<DirichletNode> &nodes,
                                       const CompiledExpression &initial,
                                       bool data_depends_on_t, Data data,
                                       const char *initial_name,
                                       const char *data_name,
                                       std::vector<std::string> &messages) {
  if (nodes.empty())
    return 0;
  if (initial.facts().depends_on_t || data_depends_on_t) {
    messages.push_back(std::format(
        "The initial condition {} and {} may differ at the {} Dirichlet "
        "nodes, where {} is used; one of them depends on t, so whether they "
        "differ depends on the start time t0",
        initial_name, data_name, nodes.size(), data_name));
    return nodes.size();
  }
  std::size_t differing = 0;
  std::string examples;
  for (std::size_t k = 0; k < nodes.size(); ++k) {
    const auto &p = mesh.nodes[nodes[k].node];
    const double initial_value = initial(p.x, p.y, 0.0);
    const double data_value = data(k);
    if (initial_value == data_value)
      continue;
    if (++differing <= listed_mismatches)
      examples +=
          std::format("{}{} has {} = {}, {} = {}", differing == 1 ? "" : "; ",
                      node_text(mesh, nodes[k].node), initial_name,
                      initial_value, data_name, data_value);
  }
  if (differing == 0)
    return 0;
  if (differing > listed_mismatches)
    examples += std::format("; and {} more", differing - listed_mismatches);
  messages.push_back(std::format(
      "The initial condition {} differs from {} at {} of {} Dirichlet "
      "nodes, where {} is used: {}",
      initial_name, data_name, differing, nodes.size(), data_name, examples));
  return differing;
}

} // namespace

MeshedProblem::MeshedProblem(std::shared_ptr<const mesh::Mesh> mesh,
                             const Problem &problem)
    : mesh_(std::move(mesh)), problem_(problem) {
  const auto start = std::chrono::steady_clock::now();
  if (!mesh_)
    throw std::invalid_argument("MeshedProblem needs a mesh, not null");
  const mesh::Mesh &m = *mesh_;
  mesh::validate(m);
  const std::size_t regions = m.region_names.size();

  // 1. Region names, both ways.
  match_regions(m, problem_);

  // 2. Classification, per region id.
  conditions_.resize(regions);
  for (std::size_t r = 0; r < regions; ++r) {
    const auto &name = m.region_names[r];
    const auto &source = problem_.boundary_conditions.at(name);
    try {
      conditions_[r].kind = classify(source);
    } catch (const std::invalid_argument &e) {
      throw std::invalid_argument("Region " + in_quotes(name) + ": " +
                                  e.what());
    }
    conditions_[r].oblique = is_oblique(source);
    conditions_[r].priority = source.priority;
  }

  // 3. Dirichlet nodes. In a valid mesh every boundary node has exactly
  // one incoming and one outgoing boundary edge.
  std::vector<std::size_t> in_region(m.nodes.size(), none);
  std::vector<std::size_t> out_region(m.nodes.size(), none);
  for (const auto &edge : m.boundary_edges) {
    out_region[edge.nodes[0]] = edge.region;
    in_region[edge.nodes[1]] = edge.region;
  }
  const auto dirichlet = [&](std::size_t r) {
    return conditions_[r].kind == ConditionKind::dirichlet;
  };
  std::vector<DirichletCorner> dirichlet_corners;
  std::size_t corners = 0, over_natural = 0, by_priority = 0, averaged = 0;
  free_index_.assign(m.nodes.size(), none);
  for (std::size_t node = 0; node < m.nodes.size(); ++node) {
    const std::size_t a = in_region[node], b = out_region[node];
    if (a == none) { // interior node
      free_index_[node] = free_nodes_.size();
      free_nodes_.push_back(node);
      continue;
    }
    if (a != b)
      ++corners;
    if (!dirichlet(a) && !dirichlet(b)) {
      free_index_[node] = free_nodes_.size();
      free_nodes_.push_back(node);
      continue;
    }
    DirichletNode result{.node = node, .regions = {}};
    if (a == b || !dirichlet(a) || !dirichlet(b)) {
      result.regions = {dirichlet(a) ? a : b};
      if (a != b)
        ++over_natural;
    } else {
      DirichletCorner corner{.node = node,
                             .first = std::min(a, b),
                             .second = std::max(a, b),
                             .averaged = false};
      const int pa = conditions_[a].priority, pb = conditions_[b].priority;
      if (pa == pb) {
        corner.averaged = true;
        result.regions = {corner.first, corner.second};
        ++averaged;
      } else {
        result.regions = {pa > pb ? a : b};
        ++by_priority;
      }
      dirichlet_corners.push_back(corner);
    }
    dirichlet_nodes_.push_back(std::move(result));
  }

  // 4. Compiling. The parameters are checked once here, so that an error
  // in them is not reported as an error of the first coefficient.
  detail::check_parameters(problem_.parameters);
  const auto &parameters = problem_.parameters;
  const auto &e = problem_.equation;
  const auto equation_coefficient = [&](const Expression &expression,
                                        const char *name) {
    return compile(expression, parameters,
                   std::string("Equation coefficient ") + name);
  };
  equation_ = {.a_xx = equation_coefficient(e.a_xx, "a_xx"),
               .a_xy = equation_coefficient(e.a_xy, "a_xy"),
               .a_yy = equation_coefficient(e.a_yy, "a_yy"),
               .b_x = equation_coefficient(e.b_x, "b_x"),
               .b_y = equation_coefficient(e.b_y, "b_y"),
               .c = equation_coefficient(e.c, "c"),
               .m1 = equation_coefficient(e.m1, "m1"),
               .m2 = equation_coefficient(e.m2, "m2"),
               .f = equation_coefficient(e.f, "f")};
  for (std::size_t r = 0; r < regions; ++r) {
    const auto &name = m.region_names[r];
    const auto &source = problem_.boundary_conditions.at(name);
    const auto coefficient = [&](const Expression &expression,
                                 const char *coefficient_name) {
      return compile(expression, parameters,
                     "Region " + in_quotes(name) + ", coefficient " +
                         coefficient_name);
    };
    auto &target = conditions_[r];
    target.alpha = coefficient(source.alpha, "alpha");
    target.beta_n = coefficient(source.beta_n, "beta_n");
    target.beta_tau = coefficient(source.beta_tau, "beta_tau");
    target.beta_x = coefficient(source.beta_x, "beta_x");
    target.beta_y = coefficient(source.beta_y, "beta_y");
    target.gamma = coefficient(source.gamma, "gamma");
    target.g = coefficient(source.g, "g");
  }
  // The corner values can be compared now that the conditions are
  // compiled.
  std::size_t differing = 0;
  for (const auto &corner : dirichlet_corners)
    if (auto message = corner_message(m, problem_, conditions_, corner)) {
      ++differing;
      report_.messages.push_back(std::move(*message));
    }

  // 5. Initial conditions, compiled; they are evaluated on request at a
  // start time. At a Dirichlet node the initial velocity comes from
  // d/dt (g / alpha), and assembly needs that rate for time order 1 or 2
  // and the second derivative for time order 2; both are differentiated
  // here (symbolic work stays in the constructor).
  const int order = time_order(problem_);
  initial_u_ = compile_initial(problem_, problem_.initial.u, order >= 1, "u",
                               "initial_values", report_.messages);
  initial_u_t_ = compile_initial(problem_, problem_.initial.u_t, order >= 2,
                                 "u_t", "initial_velocities", report_.messages);
  if (order >= 1) {
    dirichlet_rates_.resize(regions);
    if (order >= 2)
      dirichlet_accelerations_.resize(regions);
    for (std::size_t r = 0; r < regions; ++r) {
      const auto &name = m.region_names[r];
      const auto &source = problem_.boundary_conditions.at(name);
      // Zero (the default) when g / alpha does not depend on t.
      if (!dirichlet(r) || !depends_on_t(source))
        continue;
      const std::string label = "Region " + in_quotes(name) + ", ";
      Expression rate;
      try {
        rate = detail::time_derivative_of_quotient(source.g, source.alpha);
      } catch (const std::invalid_argument &error) {
        throw std::invalid_argument(
            label + "time derivative of g / alpha: " + error.what());
      }
      dirichlet_rates_[r] =
          compile(rate, parameters, label + "time derivative of g / alpha");
      if (order < 2)
        continue;
      Expression acceleration;
      try {
        acceleration = detail::derivative(rate, detail::Variable::t);
      } catch (const std::invalid_argument &error) {
        throw std::invalid_argument(
            label + "second time derivative of g / alpha: " + error.what());
      }
      dirichlet_accelerations_[r] =
          compile(acceleration, parameters,
                  label + "second time derivative of g / alpha");
    }
  }
  // The boundary data wins over the initial condition at Dirichlet nodes;
  // the report says where they disagree. g / alpha depends on t when alpha
  // or g does; its time derivative when the derivative does.
  const auto used_data_depends_on_t = [&](auto region_depends_on_t) {
    return std::ranges::any_of(dirichlet_nodes_, [&](const auto &entry) {
      return std::ranges::any_of(entry.regions, region_depends_on_t);
    });
  };
  std::size_t differing_values = 0, differing_velocities = 0;
  if (initial_u_)
    differing_values = compare_with_boundary_data(
        m, dirichlet_nodes_, *initial_u_,
        used_data_depends_on_t([&](std::size_t r) {
          return depends_on_t(
              problem_.boundary_conditions.at(m.region_names[r]));
        }),
        [&](std::size_t k) { return dirichlet_value(k, 0.0); }, "u",
        "g / alpha", report_.messages);
  if (initial_u_t_)
    differing_velocities = compare_with_boundary_data(
        m, dirichlet_nodes_, *initial_u_t_,
        used_data_depends_on_t([&](std::size_t r) {
          return dirichlet_rates_[r].facts().depends_on_t;
        }),
        [&](std::size_t k) {
          return averaged_derivative(m, dirichlet_rates_, dirichlet_nodes_[k],
                                     0.0, rate_name);
        },
        "u_t", "d/dt (g / alpha)", report_.messages);

  // 6. The divergence of A, for assembly's convection term.
  {
    std::array<Expression, 2> divergence;
    try {
      divergence = detail::divergence_of_a(e.a_xx, e.a_xy, e.a_yy);
    } catch (const std::invalid_argument &error) {
      throw std::invalid_argument(
          std::string(
              "Equation: the divergence of A (from a_xx, a_xy, a_yy): ") +
          error.what());
    }
    divergence_ = {
        .x = compile(divergence[0], parameters,
                     "Equation: the x component of the divergence of A"),
        .y = compile(divergence[1], parameters,
                     "Equation: the y component of the divergence of A")};
  }

  // 7. Facts: each CompiledExpression carries the facts of the expression
  // it was compiled from, so the summaries below read the compiled problem.

  report_.stage = "meshed_problem";
  report_.metrics = {
      {"regions", static_cast<double>(regions)},
      {"dirichlet_nodes", static_cast<double>(dirichlet_nodes_.size())},
      {"free_nodes", static_cast<double>(free_nodes_.size())},
      {"corners", static_cast<double>(corners)},
      {"dirichlet_over_natural_corners", static_cast<double>(over_natural)},
      {"priority_corners", static_cast<double>(by_priority)},
      {"averaged_corners", static_cast<double>(averaged)},
      {"differing_corners", static_cast<double>(differing)},
      {"differing_initial_values", static_cast<double>(differing_values)},
      {"differing_initial_velocities",
       static_cast<double>(differing_velocities)}};
  report_.time = std::chrono::steady_clock::now() - start;
}

double MeshedProblem::dirichlet_value(std::size_t k, double t) const {
  if (k >= dirichlet_nodes_.size())
    throw std::invalid_argument(
        std::format("Dirichlet node {} does not exist (there are {})", k,
                    dirichlet_nodes_.size()));
  const auto &entry = dirichlet_nodes_[k];
  const double value = average(entry.regions, [&](std::size_t r) {
    return checked_value(*mesh_, conditions_[r], r, entry.node, t);
  });
  if (!std::isfinite(value))
    throw std::invalid_argument(std::format(
        "The average Dirichlet value g / alpha is not finite at {}, t = {}",
        node_text(*mesh_, entry.node), t));
  return value;
}

double MeshedProblem::dirichlet_rate(std::size_t k, double t) const {
  if (dirichlet_rates_.empty())
    throw std::invalid_argument(std::format(
        "The time derivative of the Dirichlet values is not available for "
        "time order {}; it is compiled for time order 1 or 2",
        time_order(*this)));
  if (k >= dirichlet_nodes_.size())
    throw std::invalid_argument(
        std::format("Dirichlet node {} does not exist (there are {})", k,
                    dirichlet_nodes_.size()));
  return averaged_derivative(*mesh_, dirichlet_rates_, dirichlet_nodes_[k], t,
                             rate_name);
}

double MeshedProblem::dirichlet_acceleration(std::size_t k, double t) const {
  if (dirichlet_accelerations_.empty())
    throw std::invalid_argument(std::format(
        "The second time derivative of the Dirichlet values is not available "
        "for time order {}; it is compiled for time order 2",
        time_order(*this)));
  if (k >= dirichlet_nodes_.size())
    throw std::invalid_argument(
        std::format("Dirichlet node {} does not exist (there are {})", k,
                    dirichlet_nodes_.size()));
  return averaged_derivative(*mesh_, dirichlet_accelerations_,
                             dirichlet_nodes_[k], t, acceleration_name);
}

std::optional<std::vector<double>>
MeshedProblem::initial_values(double t0) const {
  check_start_time(t0);
  if (!initial_u_)
    return std::nullopt;
  auto values = at_free_nodes(*mesh_, free_nodes_, *initial_u_, "u", t0);
  // The boundary data wins over the initial condition.
  for (std::size_t k = 0; k < dirichlet_nodes_.size(); ++k)
    values[dirichlet_nodes_[k].node] = dirichlet_value(k, t0);
  return values;
}

std::optional<std::vector<double>>
MeshedProblem::initial_velocities(double t0) const {
  check_start_time(t0);
  if (!initial_u_t_)
    return std::nullopt;
  auto values = at_free_nodes(*mesh_, free_nodes_, *initial_u_t_, "u_t", t0);
  for (const auto &entry : dirichlet_nodes_)
    values[entry.node] =
        averaged_derivative(*mesh_, dirichlet_rates_, entry, t0, rate_name);
  return values;
}

std::optional<std::size_t> MeshedProblem::free_index(std::size_t node) const {
  if (node >= free_index_.size())
    throw std::invalid_argument(std::format(
        "Node {} does not exist (the mesh has {})", node, free_index_.size()));
  if (free_index_[node] == none)
    return std::nullopt;
  return free_index_[node];
}

int time_order(const MeshedProblem &problem) {
  return detail::time_order_of(problem.equation(), problem.conditions());
}

bool coefficients_depend_on_t(const MeshedProblem &problem) {
  return detail::coefficients_depend_on_t_of(problem.equation(),
                                             problem.conditions());
}

bool data_depends_on_t(const MeshedProblem &problem) {
  return detail::data_depends_on_t_of(problem.equation(), problem.conditions());
}

} // namespace pde2d::fem

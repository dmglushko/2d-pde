#include "pde2d/fem/meshed_problem.hpp"

#include "fact_summaries.hpp"
#include "symengine_expression.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <format>
#include <limits>
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
                       value_a, value_b, (value_a + value_b) / 2);
  return std::format("{}: region {} (priority {}) gives {} and wins over "
                     "region {} (priority {}), which gives {}",
                     where, in_quotes(winner_name), winner_priority,
                     b_wins ? value_b : value_a, in_quotes(loser_name),
                     loser_priority, b_wins ? value_a : value_b);
}

std::optional<std::vector<double>>
initial_values(const mesh::Mesh &mesh, const Problem &problem,
               const std::optional<Expression> &condition, bool needed,
               const std::string &name) {
  const int order = time_order(problem);
  if (needed && !condition)
    throw std::invalid_argument(
        std::format("The problem has time order {}, so it needs the initial "
                    "condition {}",
                    order, name));
  if (!needed && condition)
    throw std::invalid_argument(
        std::format("The problem has time order {}, so it takes no initial "
                    "condition {}, but it is set",
                    order, name));
  if (!condition)
    return std::nullopt;
  if (condition->facts().depends_on_t)
    throw std::invalid_argument("The initial condition " + name +
                                " depends on t; an initial condition may "
                                "depend only on x, y and parameters");
  const auto function =
      compile(*condition, problem.parameters, "Initial condition " + name);
  std::vector<double> values;
  values.reserve(mesh.nodes.size());
  for (std::size_t node = 0; node < mesh.nodes.size(); ++node) {
    const auto &p = mesh.nodes[node];
    const double value = function(p.x, p.y, 0.0);
    if (!std::isfinite(value))
      throw std::invalid_argument("The initial condition " + name +
                                  " is not finite at " + node_text(mesh, node));
    values.push_back(value);
  }
  return values;
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

  // 5. Initial conditions at the nodes.
  const int order = time_order(problem_);
  initial_u_ = initial_values(m, problem_, problem_.initial.u, order >= 1, "u");
  initial_u_t_ =
      initial_values(m, problem_, problem_.initial.u_t, order >= 2, "u_t");

  // 6. Facts: each CompiledExpression carries the facts of the expression
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
      {"differing_corners", static_cast<double>(differing)}};
  report_.time = std::chrono::steady_clock::now() - start;
}

double MeshedProblem::dirichlet_value(std::size_t k, double t) const {
  if (k >= dirichlet_nodes_.size())
    throw std::invalid_argument(
        std::format("Dirichlet node {} does not exist (there are {})", k,
                    dirichlet_nodes_.size()));
  const auto &entry = dirichlet_nodes_[k];
  double sum = 0.0;
  for (const std::size_t r : entry.regions)
    sum += checked_value(*mesh_, conditions_[r], r, entry.node, t);
  return sum / static_cast<double>(entry.regions.size());
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

bool lhs_depends_on_t(const MeshedProblem &problem) {
  return detail::lhs_depends_on_t_of(problem.equation(), problem.conditions());
}

bool rhs_depends_on_t(const MeshedProblem &problem) {
  return detail::rhs_depends_on_t_of(problem.equation(), problem.conditions());
}

} // namespace pde2d::fem

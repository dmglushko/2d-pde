#include "pde2d/fem/assembler.hpp"

#include "element_blocks.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <format>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::fem {

namespace {

using detail::EdgeTerms;
using detail::Target;
using detail::TriangleTerms;
using Index = SparsityPattern::Index;

constexpr std::size_t none = std::numeric_limits<std::size_t>::max();

} // namespace

namespace detail {

// The values of one operator: one per slot of the free pattern and one per
// slot of the Dirichlet pattern.
struct OperatorValues {
  std::vector<double> free, dirichlet;
};

// What a later assemble() reuses through `previous`. Immutable once built
// and shared between the results of one MeshedProblem.
struct AssemblyInternals {
  std::shared_ptr<const SparsityPattern> free_pattern, dirichlet_pattern;
  // Per mesh node: its free unknown or its Dirichlet column, else none.
  std::vector<std::size_t> free_index, dirichlet_index;
  // Element indices by colour, in index order inside a colour.
  std::vector<std::vector<std::size_t>> triangle_colours, edge_colours;
  // The constant parts of the operators and the load that have terms
  // depending on t (empty otherwise: then the total is the constant part).
  OperatorValues mass, damping, stiffness;
  std::vector<double> load;
};

} // namespace detail

namespace {

using detail::OperatorValues;
using Internals = detail::AssemblyInternals;

// ---------------------------------------------------------------------
// Facts: which terms are zero and which depend on t, as written.

struct TermFacts {
  bool zero = true;
  bool depends_on_t = false;
};

TermFacts facts_of(std::initializer_list<const CompiledExpression *> parts) {
  TermFacts result;
  for (const auto *part : parts)
    if (!part->facts().zero) {
      result.zero = false;
      result.depends_on_t = result.depends_on_t || part->facts().depends_on_t;
    }
  return result;
}

// A product of factors is zero when one of them is; it depends on t when it
// is not zero and one of the factors (or `extra`) depends on t.
TermFacts product(std::initializer_list<TermFacts> factors,
                  bool extra_depends_on_t = false) {
  TermFacts result{.zero = false, .depends_on_t = extra_depends_on_t};
  for (const auto &factor : factors) {
    result.zero = result.zero || factor.zero;
    result.depends_on_t = result.depends_on_t || factor.depends_on_t;
  }
  if (result.zero)
    result.depends_on_t = false;
  return result;
}

struct RegionFacts {
  TermFacts s, t, d, g; // all zero for a Dirichlet region
};

struct Plan {
  TermFacts k, c, r, m1, m2, f;
  std::vector<RegionFacts> regions;
  TermFacts mass, damping, stiffness, load; // per operator
};

TermFacts merged(std::initializer_list<TermFacts> terms) {
  TermFacts result;
  for (const auto &term : terms) {
    result.zero = result.zero && term.zero;
    result.depends_on_t = result.depends_on_t || term.depends_on_t;
  }
  return result;
}

Plan make_plan(const MeshedProblem &problem) {
  const auto &e = problem.equation();
  const auto &divergence = problem.divergence_of_a();
  Plan plan;
  plan.k = facts_of({&e.a_xx, &e.a_xy, &e.a_yy});
  plan.c = facts_of({&e.b_x, &e.b_y, &divergence.x, &divergence.y});
  plan.r = facts_of({&e.c});
  plan.m1 = facts_of({&e.m1});
  plan.m2 = facts_of({&e.m2});
  plan.f = facts_of({&e.f});
  plan.mass = plan.m2;
  plan.damping = plan.m1;
  plan.stiffness = merged({plan.k, plan.c, plan.r});
  plan.load = plan.f;
  for (const auto &condition : problem.conditions()) {
    RegionFacts region;
    if (condition.kind != ConditionKind::dirichlet) {
      // Every boundary term carries kappa_n / beta_n_hat or kappa_tau.
      const TermFacts kappa = plan.k;
      const TermFacts beta_n_hat =
          facts_of({&condition.beta_n, &condition.beta_x, &condition.beta_y});
      const TermFacts beta_tau_hat =
          facts_of({&condition.beta_tau, &condition.beta_x, &condition.beta_y});
      const auto with = [&](const CompiledExpression &coefficient) {
        return product({kappa, facts_of({&coefficient})},
                       beta_n_hat.depends_on_t);
      };
      region.s = with(condition.alpha);
      region.d = with(condition.gamma);
      region.g = with(condition.g);
      // kappa_tau - kappa_n beta_tau_hat / beta_n_hat.
      region.t =
          product({kappa}, !beta_tau_hat.zero && (beta_tau_hat.depends_on_t ||
                                                  beta_n_hat.depends_on_t));
    }
    plan.damping = merged({plan.damping, region.d});
    plan.stiffness = merged({plan.stiffness, region.s, region.t});
    plan.load = merged({plan.load, region.g});
    plan.regions.push_back(region);
  }
  return plan;
}

// Where a term goes in this call: a constant term is skipped when its
// constant part is reused.
Target target(const TermFacts &term, bool reuse_constant) {
  if (term.zero)
    return Target::skip;
  if (term.depends_on_t)
    return Target::time;
  return reuse_constant ? Target::skip : Target::constant;
}

// ---------------------------------------------------------------------
// Structure: numbering, patterns, colours.

std::shared_ptr<const SparsityPattern>
make_pattern(std::size_t rows, std::size_t columns,
             std::vector<std::vector<Index>> &entries) {
  auto pattern = std::make_shared<SparsityPattern>();
  pattern->rows = rows;
  pattern->columns = columns;
  pattern->row_offsets.reserve(rows + 1);
  pattern->row_offsets.push_back(0);
  for (auto &row : entries) {
    std::ranges::sort(row);
    const auto [first, last] = std::ranges::unique(row);
    row.erase(first, last);
    pattern->column_indices.insert(pattern->column_indices.end(), row.begin(),
                                   row.end());
    pattern->row_offsets.push_back(
        static_cast<Index>(pattern->column_indices.size()));
  }
  return pattern;
}

// Greedy colouring in element index order: each element takes the
// smallest colour not used by an earlier element that shares a node with
// it. Returns the elements of each colour, in index order.
template <std::size_t N>
std::vector<std::vector<std::size_t>>
colour(std::size_t node_count,
       const std::vector<std::array<std::size_t, N>> &elements) {
  std::vector<std::vector<std::size_t>> by_node(node_count);
  for (std::size_t i = 0; i < elements.size(); ++i)
    for (const std::size_t node : elements[i])
      by_node[node].push_back(i);
  std::vector<std::size_t> colour_of(elements.size(), none);
  std::vector<std::vector<std::size_t>> colours;
  std::vector<bool> used;
  for (std::size_t i = 0; i < elements.size(); ++i) {
    used.assign(colours.size() + 1, false);
    for (const std::size_t node : elements[i])
      for (const std::size_t j : by_node[node])
        if (j < i)
          used[colour_of[j]] = true;
    const auto c =
        static_cast<std::size_t>(std::ranges::find(used, false) - used.begin());
    if (c == colours.size())
      colours.emplace_back();
    colours[c].push_back(i);
    colour_of[i] = c;
  }
  return colours;
}

std::shared_ptr<Internals> make_structure(const MeshedProblem &problem) {
  const auto &mesh = *problem.mesh();
  auto internals = std::make_shared<Internals>();
  internals->free_index.assign(mesh.nodes.size(), none);
  internals->dirichlet_index.assign(mesh.nodes.size(), none);
  for (std::size_t i = 0; i < problem.free_nodes().size(); ++i)
    internals->free_index[problem.free_nodes()[i]] = i;
  for (std::size_t k = 0; k < problem.dirichlet_nodes().size(); ++k)
    internals->dirichlet_index[problem.dirichlet_nodes()[k].node] = k;

  // One loop over the triangles: every node pair (p, q), p free.
  const std::size_t n_free = problem.free_nodes().size();
  const std::size_t n_dirichlet = problem.dirichlet_nodes().size();
  std::vector<std::vector<Index>> free_rows(n_free), dirichlet_rows(n_free);
  for (const auto &triangle : mesh.triangles)
    for (const std::size_t p : triangle.nodes) {
      const std::size_t row = internals->free_index[p];
      if (row == none)
        continue;
      for (const std::size_t q : triangle.nodes) {
        if (internals->free_index[q] != none)
          free_rows[row].push_back(
              static_cast<Index>(internals->free_index[q]));
        else
          dirichlet_rows[row].push_back(
              static_cast<Index>(internals->dirichlet_index[q]));
      }
    }
  internals->free_pattern = make_pattern(n_free, n_free, free_rows);
  internals->dirichlet_pattern =
      make_pattern(n_free, n_dirichlet, dirichlet_rows);

  std::vector<std::array<std::size_t, 3>> triangles;
  for (const auto &triangle : mesh.triangles)
    triangles.push_back(triangle.nodes);
  std::vector<std::array<std::size_t, 2>> edges;
  for (const auto &edge : mesh.boundary_edges)
    edges.push_back(edge.nodes);
  internals->triangle_colours = colour(mesh.nodes.size(), triangles);
  internals->edge_colours = colour(mesh.nodes.size(), edges);
  return internals;
}

// ---------------------------------------------------------------------
// Scattering.

// Where entry (i, j) of an element block goes.
struct Slot {
  enum class Kind { skip, free, dirichlet } kind = Kind::skip;
  std::size_t slot = 0;
};

template <std::size_t N> struct Destinations {
  std::array<std::array<Slot, N>, N> entries{};
  std::array<std::size_t, N> rows{}; // free unknown per node, or none
};

template <std::size_t N>
Destinations<N> destinations(const Internals &internals,
                             const std::array<std::size_t, N> &nodes) {
  Destinations<N> result;
  for (std::size_t i = 0; i < N; ++i) {
    const std::size_t row = internals.free_index[nodes[i]];
    result.rows[i] = row;
    if (row == none)
      continue; // a Dirichlet row is skipped
    for (std::size_t j = 0; j < N; ++j) {
      const std::size_t column = internals.free_index[nodes[j]];
      if (column != none)
        result.entries[i][j] = {Slot::Kind::free,
                                *internals.free_pattern->find(row, column)};
      else
        result.entries[i][j] = {Slot::Kind::dirichlet,
                                *internals.dirichlet_pattern->find(
                                    row, internals.dirichlet_index[nodes[j]])};
    }
  }
  return result;
}

template <std::size_t N>
void scatter(const std::array<std::array<double, N>, N> &block,
             const Destinations<N> &where, OperatorValues &target) {
  for (std::size_t i = 0; i < N; ++i)
    for (std::size_t j = 0; j < N; ++j) {
      const Slot &slot = where.entries[i][j];
      if (slot.kind == Slot::Kind::free)
        target.free[slot.slot] += block[i][j];
      else if (slot.kind == Slot::Kind::dirichlet)
        target.dirichlet[slot.slot] += block[i][j];
    }
}

// The arrays one call fills: one set for the constant parts, one for the
// time-dependent parts. An operator that receives nothing has empty
// arrays in that set.
struct Accumulator {
  OperatorValues mass, damping, stiffness;
  std::vector<double> load;
};

template <std::size_t N>
void scatter(const detail::OperatorBlocks<N> &blocks,
             const Destinations<N> &where, Accumulator &target) {
  if (!target.mass.free.empty() || !target.mass.dirichlet.empty())
    scatter(blocks.mass, where, target.mass);
  if (!target.damping.free.empty() || !target.damping.dirichlet.empty())
    scatter(blocks.damping, where, target.damping);
  if (!target.stiffness.free.empty() || !target.stiffness.dirichlet.empty())
    scatter(blocks.stiffness, where, target.stiffness);
  if (!target.load.empty())
    for (std::size_t i = 0; i < N; ++i)
      if (where.rows[i] != none)
        target.load[where.rows[i]] += blocks.load[i];
}

OperatorValues zeros(const Internals &internals, bool needed) {
  if (!needed)
    return {};
  return {std::vector<double>(internals.free_pattern->nonzeros(), 0.0),
          std::vector<double>(internals.dirichlet_pattern->nonzeros(), 0.0)};
}

// Elementwise constant + time.
std::vector<double> total(const std::vector<double> &constant,
                          const std::vector<double> &time) {
  std::vector<double> result(constant.size());
  for (std::size_t i = 0; i < constant.size(); ++i)
    result[i] = constant[i] + time[i];
  return result;
}

} // namespace

int AssembledProblem::time_order() const {
  if (!mass_.is_zero)
    return 2;
  return damping_.is_zero ? 0 : 1;
}

AssembledProblem
Assembler::assemble(std::shared_ptr<const MeshedProblem> meshed_problem,
                    double t, const AssembledProblem *previous) const {
  const auto start = std::chrono::steady_clock::now();
  if (!meshed_problem)
    throw std::invalid_argument("assemble needs a MeshedProblem, not null");
  if (!std::isfinite(t))
    throw std::invalid_argument(
        std::format("The time t = {} to assemble at is not finite", t));
  if (previous && previous->meshed_problem_.get() != meshed_problem.get())
    throw std::invalid_argument(
        "The previous assembled problem was built from another MeshedProblem "
        "object; it can be reused only for the same object");
  const MeshedProblem &problem = *meshed_problem;
  const auto &mesh = *problem.mesh();
  const Plan plan = make_plan(problem);
  const bool reuse = previous != nullptr;

  // The structure: built once, then shared through `previous`.
  std::shared_ptr<Internals> fresh;
  const Internals *internals = nullptr;
  if (reuse) {
    internals = previous->internals_.get();
  } else {
    fresh = make_structure(problem);
    internals = fresh.get();
  }

  // The arrays this call fills. Constant parts only without `previous`.
  const auto active = [](const TermFacts &f) { return !f.zero; };
  const auto timed = [](const TermFacts &f) { return f.depends_on_t; };
  Accumulator constant, time;
  if (!reuse) {
    constant.mass = zeros(*internals, active(plan.mass));
    constant.damping = zeros(*internals, active(plan.damping));
    constant.stiffness = zeros(*internals, active(plan.stiffness));
    if (active(plan.load))
      constant.load.assign(problem.free_nodes().size(), 0.0);
  }
  time.mass = zeros(*internals, timed(plan.mass));
  time.damping = zeros(*internals, timed(plan.damping));
  time.stiffness = zeros(*internals, timed(plan.stiffness));
  if (timed(plan.load))
    time.load.assign(problem.free_nodes().size(), 0.0);

  // Triangles, colour by colour. Only this driver would know about
  // threads: the elements of one colour share no node.
  const TriangleTerms triangle_terms{.k = target(plan.k, reuse),
                                     .c = target(plan.c, reuse),
                                     .r = target(plan.r, reuse),
                                     .m1 = target(plan.m1, reuse),
                                     .m2 = target(plan.m2, reuse),
                                     .f = target(plan.f, reuse)};
  const bool any_triangle_term =
      triangle_terms.k != Target::skip || triangle_terms.c != Target::skip ||
      triangle_terms.r != Target::skip || triangle_terms.m1 != Target::skip ||
      triangle_terms.m2 != Target::skip || triangle_terms.f != Target::skip;
  if (any_triangle_term)
    for (const auto &colour_class : internals->triangle_colours)
      for (const std::size_t index : colour_class) {
        const auto &nodes = mesh.triangles[index].nodes;
        const auto blocks = detail::triangle_blocks(
            problem,
            {mesh.nodes[nodes[0]], mesh.nodes[nodes[1]], mesh.nodes[nodes[2]]},
            t, triangle_terms);
        const auto where = destinations(*internals, nodes);
        scatter(blocks.constant, where, constant);
        scatter(blocks.time, where, time);
      }
  // Boundary edges of natural and dynamic conditions, colour by colour.
  for (const auto &colour_class : internals->edge_colours)
    for (const std::size_t index : colour_class) {
      const auto &edge = mesh.boundary_edges[index];
      const auto &region = plan.regions[edge.region];
      const EdgeTerms edge_terms{.s = target(region.s, reuse),
                                 .t = target(region.t, reuse),
                                 .d = target(region.d, reuse),
                                 .g = target(region.g, reuse)};
      if (edge_terms.s == Target::skip && edge_terms.t == Target::skip &&
          edge_terms.d == Target::skip && edge_terms.g == Target::skip)
        continue;
      const auto blocks =
          detail::edge_blocks(problem, edge.region, edge.nodes, t, edge_terms);
      const auto where = destinations(*internals, edge.nodes);
      scatter(blocks.constant, where, constant);
      scatter(blocks.time, where, time);
    }

  // Totals: the constant part, plus the time-dependent part when there is
  // one (then the constant part is kept for the next call).
  AssembledProblem result;
  result.meshed_problem_ = std::move(meshed_problem);
  result.time_ = t;
  const auto finish = [&](Operator &op, const TermFacts &facts,
                          OperatorValues &constant_values,
                          OperatorValues &time_values,
                          OperatorValues Internals::*kept,
                          const Operator AssembledProblem::*previous_op) {
    op.free.pattern = internals->free_pattern;
    op.dirichlet.pattern = internals->dirichlet_pattern;
    op.depends_on_t = facts.depends_on_t;
    op.is_zero = facts.zero;
    if (facts.depends_on_t) {
      if (fresh)
        (*fresh).*kept = std::move(constant_values);
      const OperatorValues &base = internals->*kept;
      op.free.values = total(base.free, time_values.free);
      op.dirichlet.values = total(base.dirichlet, time_values.dirichlet);
    } else if (reuse) {
      op = (*previous).*previous_op;
    } else if (facts.zero) {
      op.free.values.assign(internals->free_pattern->nonzeros(), 0.0);
      op.dirichlet.values.assign(internals->dirichlet_pattern->nonzeros(), 0.0);
    } else {
      op.free.values = std::move(constant_values.free);
      op.dirichlet.values = std::move(constant_values.dirichlet);
    }
  };
  finish(result.mass_, plan.mass, constant.mass, time.mass, &Internals::mass,
         &AssembledProblem::mass_);
  finish(result.damping_, plan.damping, constant.damping, time.damping,
         &Internals::damping, &AssembledProblem::damping_);
  finish(result.stiffness_, plan.stiffness, constant.stiffness, time.stiffness,
         &Internals::stiffness, &AssembledProblem::stiffness_);
  if (plan.load.depends_on_t) {
    if (fresh)
      fresh->load = std::move(constant.load);
    result.load_ = total(internals->load, time.load);
  } else if (reuse) {
    result.load_ = previous->load_;
  } else if (plan.load.zero) {
    result.load_.assign(problem.free_nodes().size(), 0.0);
  } else {
    result.load_ = std::move(constant.load);
  }
  result.load_depends_on_t_ = plan.load.depends_on_t;
  result.internals_ = reuse ? previous->internals_ : std::move(fresh);

  // The Dirichlet data and the lifting.
  const std::size_t n_dirichlet = problem.dirichlet_nodes().size();
  const int order = result.time_order();
  for (std::size_t k = 0; k < n_dirichlet; ++k)
    result.values_.push_back(problem.dirichlet_value(k, t));
  if (order >= 1)
    for (std::size_t k = 0; k < n_dirichlet; ++k)
      result.rates_.push_back(problem.dirichlet_rate(k, t));
  if (order >= 2)
    for (std::size_t k = 0; k < n_dirichlet; ++k)
      result.accelerations_.push_back(problem.dirichlet_acceleration(k, t));
  result.right_hand_side_ = result.load_;
  const auto subtract = [&](const Operator &op,
                            const std::vector<double> &data) {
    if (op.is_zero || n_dirichlet == 0)
      return;
    const auto lifted = op.dirichlet.multiply(data);
    for (std::size_t i = 0; i < lifted.size(); ++i)
      result.right_hand_side_[i] -= lifted[i];
  };
  subtract(result.stiffness_, result.values_);
  if (order >= 1)
    subtract(result.damping_, result.rates_);
  if (order >= 2)
    subtract(result.mass_, result.accelerations_);
  // U_D, U_D' and U_D'' change with t only when g / alpha of a region used
  // at a Dirichlet node depends on t (otherwise U_D' and U_D'' are zero).
  bool data_depends_on_t = false;
  for (const auto &entry : problem.dirichlet_nodes())
    for (const std::size_t r : entry.regions) {
      const auto &condition = problem.conditions()[r];
      data_depends_on_t = data_depends_on_t ||
                          condition.g.facts().depends_on_t ||
                          condition.alpha.facts().depends_on_t;
    }
  const bool lifting_depends_on_t =
      n_dirichlet > 0 &&
      ((!plan.stiffness.zero &&
        (plan.stiffness.depends_on_t || data_depends_on_t)) ||
       (!plan.damping.zero && data_depends_on_t) ||
       (!plan.mass.zero && data_depends_on_t));
  result.rhs_depends_on_t_ = plan.load.depends_on_t || lifting_depends_on_t;

  // The report.
  const auto rebuilt = [&](const TermFacts &facts) {
    return !reuse || facts.depends_on_t ? 1.0 : 0.0;
  };
  auto &report = result.report_;
  report.stage = "assembly";
  report.metrics = {
      {"free_nodes", static_cast<double>(problem.free_nodes().size())},
      {"dirichlet_nodes", static_cast<double>(n_dirichlet)},
      {"free_nonzeros",
       static_cast<double>(internals->free_pattern->nonzeros())},
      {"dirichlet_nonzeros",
       static_cast<double>(internals->dirichlet_pattern->nonzeros())},
      {"triangle_colours",
       static_cast<double>(internals->triangle_colours.size())},
      {"edge_colours", static_cast<double>(internals->edge_colours.size())},
      {"used_previous", reuse ? 1.0 : 0.0},
      {"mass_rebuilt", rebuilt(plan.mass)},
      {"damping_rebuilt", rebuilt(plan.damping)},
      {"stiffness_rebuilt", rebuilt(plan.stiffness)},
      {"load_rebuilt", rebuilt(plan.load)}};
  report.time = std::chrono::steady_clock::now() - start;
  return result;
}

} // namespace pde2d::fem

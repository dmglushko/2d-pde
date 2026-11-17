#include "pde2d/fem/assembler.hpp"

#include "element_blocks.hpp"
#include "threads.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
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

// What never changes for one MeshedProblem: the numbering, the patterns
// and the colours. Built by the first assemble() and shared by every later
// result through `previous`.
struct AssemblyStructure {
  std::shared_ptr<const SparsityPattern> free_pattern, dirichlet_pattern;
  // Per mesh node: its free unknown or its Dirichlet column, else none.
  std::vector<std::size_t> free_index, dirichlet_index;
  // Element indices by colour, in index order inside a colour.
  std::vector<std::vector<std::size_t>> triangle_colours, edge_colours;
};

// What a later assemble() reuses through `previous`. Each result has its
// own (small) object; the arrays inside are immutable and shared.
struct AssemblyInternals {
  std::shared_ptr<const AssemblyStructure> structure;
  // The max_time_order this result was assembled with: an operator above
  // it was not built, so a later call cannot reuse it.
  int max_time_order = 2;
  // The constant parts of the operators and the load that have terms
  // depending on t (null otherwise: then the total is the constant part,
  // and the result's operator itself is reused).
  std::shared_ptr<const OperatorValues> mass, damping, stiffness;
  std::shared_ptr<const std::vector<double>> load;
};

} // namespace detail

namespace {

using detail::OperatorValues;
using Internals = detail::AssemblyInternals;
using Structure = detail::AssemblyStructure;

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
  // The stiffness's free part is symmetric: K, R and S are, so it is when
  // C and every T are zero (M1, M2 and D are always symmetric).
  bool stiffness_symmetric = true;
};

// The lowest max_time_order at which each operator is built: A and the
// load always, B = M1 - D from 1, M2 from 2.
constexpr int stiffness_level = 0;
constexpr int load_level = 0;
constexpr int damping_level = 1;
constexpr int mass_level = 2;

TermFacts merged(std::initializer_list<TermFacts> terms) {
  TermFacts result;
  for (const auto &term : terms) {
    result.zero = result.zero && term.zero;
    result.depends_on_t = result.depends_on_t || term.depends_on_t;
  }
  return result;
}

// The facts of every term that assembly builds up to `max_time_order`; a
// term above it is zero here (not built).
Plan make_plan(const MeshedProblem &problem, int max_time_order) {
  const auto &e = problem.equation();
  const auto &divergence = problem.divergence_of_a();
  Plan plan;
  plan.k = facts_of({&e.a_xx, &e.a_xy, &e.a_yy});
  plan.c = facts_of({&e.b_x, &e.b_y, &divergence.x, &divergence.y});
  plan.r = facts_of({&e.c});
  if (max_time_order >= damping_level)
    plan.m1 = facts_of({&e.m1});
  if (max_time_order >= mass_level)
    plan.m2 = facts_of({&e.m2});
  plan.f = facts_of({&e.f});
  plan.mass = plan.m2;
  plan.damping = plan.m1;
  plan.stiffness = merged({plan.k, plan.c, plan.r});
  plan.stiffness_symmetric = plan.c.zero;
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
      if (max_time_order >= damping_level)
        region.d = with(condition.gamma);
      region.g = with(condition.g);
      // kappa_tau - kappa_n beta_tau_hat / beta_n_hat. For an isotropic A
      // (a multiple of the identity) kappa_tau = n.A tau is a multiple of
      // n.tau = 0, so with beta_tau, beta_x and beta_y zero nothing is left
      // (docs/decisions.md, "T is recognised as zero from the facts").
      const bool t_vanishes = problem.a_is_isotropic() && beta_tau_hat.zero;
      if (!t_vanishes)
        region.t =
            product({kappa}, !beta_tau_hat.zero && (beta_tau_hat.depends_on_t ||
                                                    beta_n_hat.depends_on_t));
    }
    plan.stiffness_symmetric = plan.stiffness_symmetric && region.t.zero;
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

std::shared_ptr<const Structure> make_structure(const MeshedProblem &problem) {
  const auto &mesh = *problem.mesh();
  auto structure = std::make_shared<Structure>();
  structure->free_index.assign(mesh.nodes.size(), none);
  structure->dirichlet_index.assign(mesh.nodes.size(), none);
  for (std::size_t i = 0; i < problem.free_nodes().size(); ++i)
    structure->free_index[problem.free_nodes()[i]] = i;
  for (std::size_t k = 0; k < problem.dirichlet_nodes().size(); ++k)
    structure->dirichlet_index[problem.dirichlet_nodes()[k].node] = k;

  // One loop over the triangles: every node pair (p, q), p free.
  const std::size_t n_free = problem.free_nodes().size();
  const std::size_t n_dirichlet = problem.dirichlet_nodes().size();
  std::vector<std::vector<Index>> free_rows(n_free), dirichlet_rows(n_free);
  for (const auto &triangle : mesh.triangles)
    for (const std::size_t p : triangle.nodes) {
      const std::size_t row = structure->free_index[p];
      if (row == none)
        continue;
      for (const std::size_t q : triangle.nodes) {
        if (structure->free_index[q] != none)
          free_rows[row].push_back(
              static_cast<Index>(structure->free_index[q]));
        else
          dirichlet_rows[row].push_back(
              static_cast<Index>(structure->dirichlet_index[q]));
      }
    }
  structure->free_pattern = make_pattern(n_free, n_free, free_rows);
  structure->dirichlet_pattern =
      make_pattern(n_free, n_dirichlet, dirichlet_rows);

  std::vector<std::array<std::size_t, 3>> triangles;
  for (const auto &triangle : mesh.triangles)
    triangles.push_back(triangle.nodes);
  std::vector<std::array<std::size_t, 2>> edges;
  for (const auto &edge : mesh.boundary_edges)
    edges.push_back(edge.nodes);
  structure->triangle_colours = colour(mesh.nodes.size(), triangles);
  structure->edge_colours = colour(mesh.nodes.size(), edges);
  return structure;
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
Destinations<N> destinations(const Structure &structure,
                             const std::array<std::size_t, N> &nodes) {
  Destinations<N> result;
  for (std::size_t i = 0; i < N; ++i) {
    const std::size_t row = structure.free_index[nodes[i]];
    result.rows[i] = row;
    if (row == none)
      continue; // a Dirichlet row is skipped
    for (std::size_t j = 0; j < N; ++j) {
      const std::size_t column = structure.free_index[nodes[j]];
      if (column != none)
        result.entries[i][j] = {Slot::Kind::free,
                                *structure.free_pattern->find(row, column)};
      else
        result.entries[i][j] = {Slot::Kind::dirichlet,
                                *structure.dirichlet_pattern->find(
                                    row, structure.dirichlet_index[nodes[j]])};
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

// Calls body(element) for every element of one colour (in index order).
// With one thread it is a plain loop. With more, OpenMP shares the
// elements among the threads: they share no node, so no slot or load
// entry gets two additions in one colour, and the order of the additions
// to each one (colour by colour) is the same for every thread count. An
// exception does not leave a thread (that would terminate the program):
// each is caught, and after the colour the one of the lowest position
// (the lowest element index, so the one the plain loop would throw) is
// thrown again.
template <class Body>
void for_each_in_colour(const std::vector<std::size_t> &colour, int threads,
                        const Body &body) {
  if (threads <= 1 || colour.size() < 2) {
    for (const std::size_t index : colour)
      body(index);
    return;
  }
  std::exception_ptr error;
  std::size_t error_position = none;
  const auto count = static_cast<std::ptrdiff_t>(colour.size());
#pragma omp parallel for schedule(static) num_threads(threads)
  for (std::ptrdiff_t position = 0; position < count; ++position) {
    const auto at = static_cast<std::size_t>(position);
    try {
      body(colour[at]);
    } catch (...) {
#pragma omp critical(pde2d_assembly_error)
      if (at < error_position) {
        error_position = at;
        error = std::current_exception();
      }
    }
  }
  if (error)
    std::rethrow_exception(error);
}

OperatorValues zeros(const Structure &structure, bool needed) {
  if (!needed)
    return {};
  return {std::vector<double>(structure.free_pattern->nonzeros(), 0.0),
          std::vector<double>(structure.dirichlet_pattern->nonzeros(), 0.0)};
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

namespace {

// The time order of operators with these formula facts: 2 if the mass is
// not zero, else 1 if the damping is not zero, else 0. One rule for
// AssembledProblem::time_order and Assembler::time_order, so they agree.
int order_of(bool mass_zero, bool damping_zero) {
  if (!mass_zero)
    return 2;
  return damping_zero ? 0 : 1;
}

} // namespace

int AssembledProblem::time_order() const {
  return order_of(mass_.always_zero, damping_.always_zero);
}

Assembler::Assembler(AssemblerOptions options) : options_(options) {
  detail::check_threads(options_.threads, "assembler");
}

int Assembler::time_order(const MeshedProblem &meshed_problem) const {
  // The plan of a full assembly (max_time_order 2) holds the facts each
  // operator would get; making it reads only the coefficients' facts and
  // evaluates nothing.
  const Plan plan = make_plan(meshed_problem, /*max_time_order=*/2);
  return order_of(plan.mass.zero, plan.damping.zero);
}

AssembledProblem
Assembler::assemble(std::shared_ptr<const MeshedProblem> meshed_problem,
                    double t, const AssembledProblem *previous,
                    int max_time_order) const {
  const auto start = std::chrono::steady_clock::now();
  if (!meshed_problem)
    throw std::invalid_argument("assemble needs a MeshedProblem, not null");
  if (!std::isfinite(t))
    throw std::invalid_argument(
        std::format("The time t = {} to assemble at is not finite", t));
  if (max_time_order < 0 || max_time_order > 2)
    throw std::invalid_argument(std::format(
        "assemble: max_time_order must be 0, 1 or 2, not {}", max_time_order));
  if (previous && previous->meshed_problem_.get() != meshed_problem.get())
    throw std::invalid_argument(
        "The previous assembled problem was built from another MeshedProblem "
        "object; it can be reused only for the same object");
  const MeshedProblem &problem = *meshed_problem;
  const auto &mesh = *problem.mesh();
  const Plan plan = make_plan(problem, max_time_order);
  const Internals *before = previous ? previous->internals_.get() : nullptr;

  // An operator's constant part (and, when nothing of it depends on t, the
  // operator itself) is reused when `previous` built it: it was assembled
  // at a max_time_order that includes the operator. An operator above this
  // call's max_time_order is not built at all (its facts are zero).
  const auto reusable = [&](int level) {
    return before && level <= max_time_order && before->max_time_order >= level;
  };
  const bool reuse_mass = reusable(mass_level);
  const bool reuse_damping = reusable(damping_level);
  const bool reuse_stiffness = reusable(stiffness_level);
  const bool reuse_load = reusable(load_level);

  // The structure: built once, then shared through `previous`.
  auto internals = std::make_shared<Internals>();
  internals->structure = before ? before->structure : make_structure(problem);
  internals->max_time_order = max_time_order;
  const Structure &structure = *internals->structure;

  // The arrays this call fills. A constant part only for an operator that
  // is not reused.
  const auto active = [](const TermFacts &f) { return !f.zero; };
  const auto timed = [](const TermFacts &f) { return f.depends_on_t; };
  Accumulator constant, time;
  constant.mass = zeros(structure, !reuse_mass && active(plan.mass));
  constant.damping = zeros(structure, !reuse_damping && active(plan.damping));
  constant.stiffness =
      zeros(structure, !reuse_stiffness && active(plan.stiffness));
  if (!reuse_load && active(plan.load))
    constant.load.assign(problem.free_nodes().size(), 0.0);
  time.mass = zeros(structure, timed(plan.mass));
  time.damping = zeros(structure, timed(plan.damping));
  time.stiffness = zeros(structure, timed(plan.stiffness));
  if (timed(plan.load))
    time.load.assign(problem.free_nodes().size(), 0.0);

  // Triangles, colour by colour. Only this driver knows about threads:
  // the elements of one colour share no node (for_each_in_colour).
  const int threads = detail::resolve_threads(options_.threads);
  const TriangleTerms triangle_terms{.k = target(plan.k, reuse_stiffness),
                                     .c = target(plan.c, reuse_stiffness),
                                     .r = target(plan.r, reuse_stiffness),
                                     .m1 = target(plan.m1, reuse_damping),
                                     .m2 = target(plan.m2, reuse_mass),
                                     .f = target(plan.f, reuse_load)};
  const bool any_triangle_term =
      triangle_terms.k != Target::skip || triangle_terms.c != Target::skip ||
      triangle_terms.r != Target::skip || triangle_terms.m1 != Target::skip ||
      triangle_terms.m2 != Target::skip || triangle_terms.f != Target::skip;
  if (any_triangle_term)
    for (const auto &colour_class : structure.triangle_colours)
      for_each_in_colour(colour_class, threads, [&](std::size_t index) {
        const auto &nodes = mesh.triangles[index].nodes;
        const auto blocks = detail::triangle_blocks(
            problem,
            {mesh.nodes[nodes[0]], mesh.nodes[nodes[1]], mesh.nodes[nodes[2]]},
            t, triangle_terms);
        const auto where = destinations(structure, nodes);
        scatter(blocks.constant, where, constant);
        scatter(blocks.time, where, time);
      });
  // Boundary edges of natural and dynamic conditions, colour by colour.
  for (const auto &colour_class : structure.edge_colours)
    for_each_in_colour(colour_class, threads, [&](std::size_t index) {
      const auto &edge = mesh.boundary_edges[index];
      const auto &region = plan.regions[edge.region];
      const EdgeTerms edge_terms{.s = target(region.s, reuse_stiffness),
                                 .t = target(region.t, reuse_stiffness),
                                 .d = target(region.d, reuse_damping),
                                 .g = target(region.g, reuse_load)};
      if (edge_terms.s == Target::skip && edge_terms.t == Target::skip &&
          edge_terms.d == Target::skip && edge_terms.g == Target::skip) {
        // Nothing to compute. The condition is still solved for u_n
        // wherever A is not zero, so check that it can be at this t: always
        // in a fresh assembly (every term can be zero, for example u_n = 0
        // with an isotropic A, whose T is zero), and with `previous` when
        // the normal coefficient depends on t (it was checked before
        // otherwise). A Dirichlet region has no terms and no check.
        const auto &condition = problem.conditions()[edge.region];
        const bool solved_for_u_n =
            condition.kind != ConditionKind::dirichlet && !plan.k.zero;
        if (solved_for_u_n &&
            (!before || condition.beta_n.facts().depends_on_t ||
             condition.beta_x.facts().depends_on_t ||
             condition.beta_y.facts().depends_on_t))
          detail::edge_blocks(problem, edge.region, edge.nodes, t, edge_terms);
        return;
      }
      const auto blocks =
          detail::edge_blocks(problem, edge.region, edge.nodes, t, edge_terms);
      const auto where = destinations(structure, edge.nodes);
      scatter(blocks.constant, where, constant);
      scatter(blocks.time, where, time);
    });

  // Totals: the constant part, plus the time-dependent part when there is
  // one (then the constant part is kept for the next call).
  AssembledProblem result;
  result.meshed_problem_ = std::move(meshed_problem);
  result.time_ = t;
  const auto finish =
      [&](Operator &op, const TermFacts &facts, bool reused,
          OperatorValues &constant_values, const OperatorValues &time_values,
          std::shared_ptr<const OperatorValues> Internals::*kept,
          const Operator AssembledProblem::*previous_op) {
        if (facts.depends_on_t) {
          internals.get()->*kept = reused
                                       ? before->*kept
                                       : std::make_shared<const OperatorValues>(
                                             std::move(constant_values));
          const OperatorValues &base = *(internals.get()->*kept);
          op.free.values = total(base.free, time_values.free);
          op.dirichlet.values = total(base.dirichlet, time_values.dirichlet);
        } else if (reused) {
          op = (*previous).*previous_op;
        } else if (facts.zero) {
          op.free.values.assign(structure.free_pattern->nonzeros(), 0.0);
          op.dirichlet.values.assign(structure.dirichlet_pattern->nonzeros(),
                                     0.0);
        } else {
          op.free.values = std::move(constant_values.free);
          op.dirichlet.values = std::move(constant_values.dirichlet);
        }
        op.free.pattern = structure.free_pattern;
        op.dirichlet.pattern = structure.dirichlet_pattern;
        op.depends_on_t = facts.depends_on_t;
        op.always_zero = facts.zero;
      };
  finish(result.mass_, plan.mass, reuse_mass, constant.mass, time.mass,
         &Internals::mass, &AssembledProblem::mass_);
  finish(result.damping_, plan.damping, reuse_damping, constant.damping,
         time.damping, &Internals::damping, &AssembledProblem::damping_);
  finish(result.stiffness_, plan.stiffness, reuse_stiffness, constant.stiffness,
         time.stiffness, &Internals::stiffness, &AssembledProblem::stiffness_);
  // M2, M1 and D are symmetric (mass-like integrals of phi_i phi_j); a zero
  // operator is too.
  result.mass_.always_symmetric = true;
  result.damping_.always_symmetric = true;
  result.stiffness_.always_symmetric =
      plan.stiffness.zero || plan.stiffness_symmetric;
  // The facts from the numbers, read from the values just built (or
  // reused: the same values give the same answer, so reuse stays bit for
  // bit equal to a fresh assembly). A formula fact implies its number fact.
  for (Operator *op : {&result.mass_, &result.damping_, &result.stiffness_}) {
    op->zero_now = op->always_zero || has_only_zero_entries(op->free);
    op->symmetric_now = op->always_symmetric || is_symmetric(op->free);
  }
  if (plan.load.depends_on_t) {
    internals->load = reuse_load ? before->load
                                 : std::make_shared<const std::vector<double>>(
                                       std::move(constant.load));
    result.load_ = total(*internals->load, time.load);
  } else if (reuse_load) {
    result.load_ = previous->load_;
  } else if (plan.load.zero) {
    result.load_.assign(problem.free_nodes().size(), 0.0);
  } else {
    result.load_ = std::move(constant.load);
  }
  result.load_depends_on_t_ = plan.load.depends_on_t;
  result.internals_ = internals;

  // The Dirichlet data and the lifting. The time order is at most
  // max_time_order, so U_D' and U_D'' are evaluated only when asked for.
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
    if (op.always_zero || n_dirichlet == 0)
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

  // The report. An operator above max_time_order is not built: 0.
  const auto rebuilt = [&](const TermFacts &facts, bool reused, int level) {
    if (level > max_time_order)
      return 0.0;
    return !reused || facts.depends_on_t ? 1.0 : 0.0;
  };
  auto &report = result.report_;
  report.stage = "assembly";
  report.metrics = {
      {"free_nodes", static_cast<double>(problem.free_nodes().size())},
      {"dirichlet_nodes", static_cast<double>(n_dirichlet)},
      {"free_nonzeros",
       static_cast<double>(structure.free_pattern->nonzeros())},
      {"dirichlet_nonzeros",
       static_cast<double>(structure.dirichlet_pattern->nonzeros())},
      {"triangle_colours",
       static_cast<double>(structure.triangle_colours.size())},
      {"edge_colours", static_cast<double>(structure.edge_colours.size())},
      {"used_previous", before ? 1.0 : 0.0},
      {"max_time_order", static_cast<double>(max_time_order)},
      {"threads", static_cast<double>(threads)},
      {"mass_rebuilt", rebuilt(plan.mass, reuse_mass, mass_level)},
      {"damping_rebuilt", rebuilt(plan.damping, reuse_damping, damping_level)},
      {"stiffness_rebuilt",
       rebuilt(plan.stiffness, reuse_stiffness, stiffness_level)},
      {"load_rebuilt", rebuilt(plan.load, reuse_load, load_level)}};
  report.time = std::chrono::steady_clock::now() - start;
  return result;
}

} // namespace pde2d::fem

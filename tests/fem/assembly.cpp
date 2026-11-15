#include "assembly_support.hpp"

#include "pde2d/fem/assembler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <ranges>
#include <stdexcept>
#include <string>
#include <vector>

using namespace pde2d::fem;
using pde2d::mesh::Mesh;
using pde2d::test::check;
using pde2d::test::Dense;
using pde2d::test::dense;
using pde2d::test::expect_throws;
using pde2d::test::near;

namespace {

constexpr double sqrt2 = std::numbers::sqrt2;

Expression e(const char *text) { return Expression(text); }

BoundaryCondition neumann(const char *g = "0") {
  return {.beta_n = e("1"), .g = e(g)};
}
BoundaryCondition dirichlet(const char *g) {
  return {.alpha = e("1"), .g = e(g)};
}

std::shared_ptr<const MeshedProblem> meshed(std::shared_ptr<const Mesh> mesh,
                                            const Problem &problem) {
  return std::make_shared<const MeshedProblem>(std::move(mesh), problem);
}

// A problem on the reference triangle with the given equation and the
// conditions (Neumann u_n = 0 where none is given).
std::shared_ptr<const MeshedProblem>
on_triangle(const Equation &equation,
            std::map<std::string, BoundaryCondition> conditions = {},
            InitialConditions initial = {}) {
  for (const char *region : {"bottom", "diagonal", "left"})
    conditions.try_emplace(region, neumann());
  return meshed(pde2d::test::reference_triangle(),
                {.equation = equation,
                 .boundary_conditions = std::move(conditions),
                 .initial = std::move(initial)});
}

AssembledProblem assemble(const std::shared_ptr<const MeshedProblem> &problem,
                          double t = 0.0,
                          const AssembledProblem *previous = nullptr) {
  return Assembler().assemble(problem, t, previous);
}

void near_matrix(const Dense &actual, const Dense &expected, double tolerance,
                 const std::string &what) {
  check(actual.size() == expected.size(), what + ": rows");
  for (std::size_t i = 0; i < actual.size(); ++i) {
    check(actual[i].size() == expected[i].size(), what + ": columns");
    for (std::size_t j = 0; j < actual[i].size(); ++j)
      near(actual[i][j], expected[i][j], tolerance,
           what + " (" + std::to_string(i) + ", " + std::to_string(j) + ")");
  }
}

void near_vector(std::span<const double> actual,
                 const std::vector<double> &expected, double tolerance,
                 const std::string &what) {
  check(actual.size() == expected.size(), what + ": size");
  for (std::size_t i = 0; i < actual.size(); ++i)
    near(actual[i], expected[i], tolerance,
         what + " [" + std::to_string(i) + "]");
}

Dense scaled(const Dense &m, double factor) {
  Dense r = m;
  for (auto &row : r)
    for (auto &v : row)
      v *= factor;
  return r;
}

Dense plus(const Dense &a, const Dense &b) {
  Dense r = a;
  for (std::size_t i = 0; i < r.size(); ++i)
    for (std::size_t j = 0; j < r[i].size(); ++j)
      r[i][j] += b[i][j];
  return r;
}

// An edge block (2 x 2, rows and columns its nodes a, b) placed into a
// 3 x 3 matrix.
Dense on_edge(std::size_t a, std::size_t b,
              const std::array<std::array<double, 2>, 2> &block) {
  Dense r(3, std::vector<double>(3, 0.0));
  const std::array<std::size_t, 2> nodes{a, b};
  for (std::size_t i = 0; i < 2; ++i)
    for (std::size_t j = 0; j < 2; ++j)
      r[nodes[i]][nodes[j]] = block[i][j];
  return r;
}

// Hand-computed on the reference triangle (area 1/2, gradients of the
// barycentric coordinates (-1, -1), (1, 0), (0, 1)).
// K for A = identity: area * grad phi_i . grad phi_j.
const Dense k_identity{{1, -0.5, -0.5}, {-0.5, 0.5, 0}, {-0.5, 0, 0.5}};
// The mass matrix: area / 12 * [[2, 1, 1], [1, 2, 1], [1, 1, 2]].
const Dense mass_matrix{{2.0 / 24, 1.0 / 24, 1.0 / 24},
                        {1.0 / 24, 2.0 / 24, 1.0 / 24},
                        {1.0 / 24, 1.0 / 24, 2.0 / 24}};
// The edge mass matrix of the bottom edge (length 1): 1/6 [[2, 1], [1, 2]].
const std::array<std::array<double, 2>, 2> edge_mass{
    {{2.0 / 6, 1.0 / 6}, {1.0 / 6, 2.0 / 6}}};

constexpr double tight = 1e-14;

// u = 1 + 2x + 3y, linear, and the problem that has it as its solution on
// [0, 2] x [0, 1] with constant, non-symmetric coefficients. `left` is the
// condition on the left side; the others are Robin (bottom), oblique
// Neumann (right) and oblique Robin (top), with g from u.
Problem linear_problem(BoundaryCondition left) {
  return {
      .equation = {.a_xx = e("2"),
                   .a_xy = e("1"),
                   .a_yy = e("1.5"),
                   .b_x = e("1"),
                   .b_y = e("-2"),
                   .c = e("0.5"),
                   // b . grad u + c u
                   .f = e("-3.5 + x + 1.5*y")},
      .boundary_conditions = {
          {"left", std::move(left)},
          // n = (0, -1): u + u_n = u - 3.
          {"bottom",
           {.alpha = e("1"), .beta_n = e("1"), .g = e("1 + 2*x + 3*y - 3")}},
          // n = (1, 0), tau = (0, 1): u_n + 0.5 u_tau + 0.3 u_x
          // = 2 + 1.5 + 0.6.
          {"right",
           {.beta_n = e("1"),
            .beta_tau = e("0.5"),
            .beta_x = e("0.3"),
            .g = e("4.1")}},
          // n = (0, 1): 2 u + 2 u_n + 0.5 u_y = 2 u + 6 + 1.5.
          {"top",
           {.alpha = e("2"),
            .beta_n = e("2"),
            .beta_y = e("0.5"),
            .g = e("2*(1 + 2*x + 3*y) + 7.5")}}}};
}

double linear_u(const pde2d::geometry::Point &p) {
  return 1 + 2 * p.x + 3 * p.y;
}

std::vector<double> at_nodes(const MeshedProblem &problem,
                             const std::vector<std::size_t> &nodes,
                             double (*u)(const pde2d::geometry::Point &)) {
  std::vector<double> values;
  for (const std::size_t node : nodes)
    values.push_back(u(problem.mesh()->nodes[node]));
  return values;
}

std::vector<std::size_t> dirichlet_node_list(const MeshedProblem &problem) {
  std::vector<std::size_t> nodes;
  for (const auto &entry : problem.dirichlet_nodes())
    nodes.push_back(entry.node);
  return nodes;
}

bool same(const Operator &a, const Operator &b) {
  return a.free.values == b.free.values &&
         a.dirichlet.values == b.dirichlet.values &&
         a.depends_on_t == b.depends_on_t && a.always_zero == b.always_zero &&
         a.always_symmetric == b.always_symmetric;
}

bool same(const AssembledProblem &a, const AssembledProblem &b) {
  const auto eq = [](std::span<const double> x, std::span<const double> y) {
    return std::ranges::equal(x, y);
  };
  return same(a.mass(), b.mass()) && same(a.damping(), b.damping()) &&
         same(a.stiffness(), b.stiffness()) && eq(a.load(), b.load()) &&
         eq(a.dirichlet_values(), b.dirichlet_values()) &&
         eq(a.dirichlet_rates(), b.dirichlet_rates()) &&
         eq(a.dirichlet_accelerations(), b.dirichlet_accelerations()) &&
         eq(a.right_hand_side(), b.right_hand_side()) &&
         a.load_depends_on_t() == b.load_depends_on_t() &&
         a.right_hand_side_depends_on_t() == b.right_hand_side_depends_on_t() &&
         a.time_order() == b.time_order();
}

double metric(const AssembledProblem &a, const std::string &name) {
  return a.report().metrics.at(name);
}

// --- max_time_order -----------------------------------------------------

// Every operator has a constant term and one in t (the mass only one in
// t), the load too, and the Dirichlet data moves, so each level and each
// reuse path has something to build. Time order 2.
std::shared_ptr<const MeshedProblem> every_term_problem() {
  return meshed(pde2d::test::rectangle_mesh(2, 1, 0.4),
                {.equation = {.a_xx = e("2"),
                              .a_yy = e("2"),
                              .c = e("sin(t)"),
                              .m1 = e("1"),
                              .m2 = e("1 + t^2"),
                              .f = e("x")},
                 .boundary_conditions = {{"left", dirichlet("exp(t)*(1 + y)")},
                                         {"bottom",
                                          {.alpha = e("1"),
                                           .beta_n = e("1"),
                                           .gamma = e("1 + t"),
                                           .g = e("t*x")}},
                                         {"right", neumann("1")},
                                         {"top", neumann("0")}},
                 .initial = {.u = e("0"), .u_t = e("0")}});
}

bool all_zero(const Operator &op) {
  return std::ranges::all_of(op.free.values, [](double v) { return v == 0; }) &&
         std::ranges::all_of(op.dirichlet.values,
                             [](double v) { return v == 0; });
}

// load - A_FD U_D [- B_FD U_D'] [- M2_FD U_D''], from the full assembly's
// operators and data, up to `order`.
std::vector<double> lifted(const AssembledProblem &full, int order) {
  std::vector<double> b(full.load().begin(), full.load().end());
  const auto subtract = [&](const Operator &op, std::span<const double> data) {
    const auto product = op.dirichlet.multiply(data);
    for (std::size_t i = 0; i < b.size(); ++i)
      b[i] -= product[i];
  };
  subtract(full.stiffness(), full.dirichlet_values());
  if (order >= 1)
    subtract(full.damping(), full.dirichlet_rates());
  if (order >= 2)
    subtract(full.mass(), full.dirichlet_accelerations());
  return b;
}

// --- symmetry -----------------------------------------------------------

// max |a_ij - a_ji| / max |a_ij| over the free part, read entry by entry:
// an independent check of the `symmetric` fact.
double asymmetry(const CsrMatrix &matrix) {
  const Dense a = dense(matrix);
  double largest = 0.0, difference = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i)
    for (std::size_t j = 0; j < a.size(); ++j) {
      largest = std::max(largest, std::abs(a[i][j]));
      difference = std::max(difference, std::abs(a[i][j] - a[j][i]));
    }
  return largest > 0.0 ? difference / largest : 0.0;
}

// The assembly of `equation` on the reference triangle with the given
// condition on the bottom and Neumann conditions on the others. The
// diagonal edge is not axis-aligned: on axis-aligned edges T vanishes
// for any diagonal A (n.A tau = 0), which would hide an anisotropic A.
AssembledProblem stiffness_of(const Equation &equation,
                              BoundaryCondition bottom) {
  return assemble(on_triangle(
      equation, {{"bottom", std::move(bottom)}, {"diagonal", neumann("1")}}));
}

void symmetric_case(const Equation &equation, BoundaryCondition bottom,
                    bool expected, const std::string &what) {
  const auto a = stiffness_of(equation, std::move(bottom));
  check(a.stiffness().always_symmetric == expected, what + ": the fact");
  const double measured = asymmetry(a.stiffness().free);
  if (expected)
    check(measured < 1e-14, what + ": A = A^T to rounding");
  else
    check(measured > 1e-6, what + ": A differs from A^T");
}

} // namespace

int main() {

  return pde2d::test::run({
      // ---------------------------------------------------------------
      // Hand-computed blocks on the reference triangle (all nodes free).
      {"K for A = identity",
       [] {
         // On the diagonal edge kappa_tau = n . tau = 0, so T is zero.
         const auto a = assemble(on_triangle({.a_xx = e("1"), .a_yy = e("1")}));
         near_matrix(dense(a.stiffness().free), scaled(k_identity, -1), tight,
                     "-K");
         check(a.stiffness().dirichlet.columns() == 0, "no Dirichlet columns");
         check(a.mass().always_zero && a.damping().always_zero,
               "no time terms");
       }},
      {"K and T for an anisotropic A",
       [] {
         // A = [[2, 0.5], [0.5, 3]] (a_xy = 1), Neumann u_n = 0 everywhere.
         // K = area * grad phi_i . A grad phi_j.
         const Dense k{{3, -1.25, -1.75}, {-1.25, 1, 0.25}, {-1.75, 0.25, 1.5}};
         // T per edge: kappa_tau / 2 * [[-1, 1], [-1, 1]], kappa_tau =
         // n . A tau: -0.5 on the bottom, 0.5 on the diagonal and the left.
         const Dense t{
             {0.5, -0.25, -0.25}, {0.25, -0.5, 0.25}, {0.25, -0.25, 0}};
         const auto a = assemble(
             on_triangle({.a_xx = e("2"), .a_xy = e("1"), .a_yy = e("3")}));
         near_matrix(dense(a.stiffness().free), plus(scaled(k, -1), t), tight,
                     "-K + T");
       }},
      {"convection, reaction and the masses",
       [] {
         // A = 0, so no boundary term. C_ij = (b . grad phi_j) area / 3
         // with b = (1, 2): b . grad phi = -3, 1, 2.
         const auto c = assemble(on_triangle({.b_x = e("1"), .b_y = e("2")}));
         const std::vector<double> row{-3.0 / 6, 1.0 / 6, 2.0 / 6};
         near_matrix(dense(c.stiffness().free), {row, row, row}, tight, "C");
         const auto r = assemble(
             on_triangle({.c = e("1"), .m1 = e("2"), .m2 = e("3"), .f = e("1")},
                         {}, {.u = e("0"), .u_t = e("0")}));
         near_matrix(dense(r.stiffness().free), mass_matrix, tight, "R");
         near_matrix(dense(r.damping().free), scaled(mass_matrix, 2), tight,
                     "M1");
         near_matrix(dense(r.mass().free), scaled(mass_matrix, 3), tight, "M2");
         near_vector(r.load(), {1.0 / 6, 1.0 / 6, 1.0 / 6}, tight, "F");
       }},
      {"Robin: S",
       [] {
         // Bottom: 2 u + u_n = 0, A = identity: kappa_n alpha / beta_n = 2.
         const auto a = assemble(
             on_triangle({.a_xx = e("1"), .a_yy = e("1")},
                         {{"bottom", {.alpha = e("2"), .beta_n = e("1")}}}));
         const Dense s = on_edge(0, 1, edge_mass);
         near_matrix(dense(a.stiffness().free),
                     plus(scaled(k_identity, -1), scaled(s, -2)), tight,
                     "-K - S");
       }},
      {"Neumann data: G",
       [] {
         // u_n = 3 on every edge: G_i = 3 * (half the lengths at i).
         const auto a = assemble(on_triangle({.a_xx = e("1"), .a_yy = e("1")},
                                             {{"bottom", neumann("3")},
                                              {"diagonal", neumann("3")},
                                              {"left", neumann("3")}}));
         near_vector(a.load(),
                     {-3.0, -3 * (1 + sqrt2) / 2, -3 * (1 + sqrt2) / 2}, tight,
                     "-G");
         check(!a.load_depends_on_t(), "constant");
       }},
      {"an oblique condition: S, T and G",
       [] {
         // Bottom, n = (0, -1), tau = (1, 0): u + u_n + u_x + 0.5 u_y = 1.
         // beta_n_hat = 1 - 0.5 = 0.5, beta_tau_hat = 1, so with A =
         // identity (kappa_n = 1, kappa_tau = 0) the factor kappa_n /
         // beta_n_hat is 2: S = 2 * edge mass, T coefficient 0 - 2 * 1 =
         // -2, so T = -1 * [[-1, 1], [-1, 1]], G_i = 2 * 1 * 1/2 = 1.
         const auto a = assemble(on_triangle({.a_xx = e("1"), .a_yy = e("1")},
                                             {{"bottom",
                                               {.alpha = e("1"),
                                                .beta_n = e("1"),
                                                .beta_x = e("1"),
                                                .beta_y = e("0.5"),
                                                .g = e("1")}}}));
         const Dense s = on_edge(0, 1, edge_mass);
         const Dense t = on_edge(0, 1, {{{1, -1}, {1, -1}}});
         near_matrix(dense(a.stiffness().free),
                     plus(plus(scaled(k_identity, -1), scaled(s, -2)), t),
                     tight, "-K - S + T");
         near_vector(a.load(), {-1, -1, 0}, tight, "-G");
       }},
      {"a dynamic condition: D",
       [] {
         // Bottom: u_n + 4 u_t = 0, A = identity: D = 4 * edge mass, and
         // the damping is M1 - D = -D.
         const auto a = assemble(on_triangle(
             {.a_xx = e("1"), .a_yy = e("1")},
             {{"bottom", {.beta_n = e("1"), .gamma = e("4")}}}, {.u = e("0")}));
         near_matrix(dense(a.damping().free),
                     scaled(on_edge(0, 1, edge_mass), -4), tight, "-D");
         check(!a.damping().always_zero && a.time_order() == 1,
               "damping only from D");
       }},

      // ---------------------------------------------------------------
      // Invariants on a Mesher mesh of [0, 2] x [0, 1].
      {"invariants: K row sums and symmetry, mass and load sums",
       [] {
         const auto mesh = pde2d::test::rectangle_mesh(2, 1, 0.3);
         std::map<std::string, BoundaryCondition> all_natural;
         for (const char *side : {"bottom", "right", "top", "left"})
           all_natural[side] = neumann();
         const auto a =
             assemble(meshed(mesh, {.equation = {.a_xx = e("1"),
                                                 .a_yy = e("1"),
                                                 .m2 = e("1"),
                                                 .f = e("1")},
                                    .boundary_conditions = all_natural,
                                    .initial = {.u = e("0"), .u_t = e("0")}}));
         const Dense k = dense(a.stiffness().free);
         check(k.size() == mesh->nodes.size(), "every node is free");
         for (std::size_t i = 0; i < k.size(); ++i) {
           double sum = 0;
           for (std::size_t j = 0; j < k.size(); ++j) {
             sum += k[i][j];
             near(k[i][j], k[j][i], 1e-14, "symmetric");
           }
           near(sum, 0.0, 1e-13, "zero row sum");
         }
         double mass = 0;
         for (const double v : a.mass().free.values)
           mass += v;
         near(mass, 2.0, 1e-13, "mass sum = area");
         double load = 0;
         for (const double v : a.load())
           load += v;
         near(load, 2.0, 1e-13, "load sum = area");
       }},

      // ---------------------------------------------------------------
      // Exactness: P1 reproduces linear functions.
      {"a linear solution satisfies the assembled equations",
       [] {
         const auto mesh = pde2d::test::rectangle_mesh(2, 1, 0.25);
         for (const bool with_dirichlet : {true, false}) {
           // Left, n = (-1, 0): u_n = -2.
           const auto problem = meshed(
               mesh, linear_problem(with_dirichlet ? dirichlet("1 + 2*x + 3*y")
                                                   : neumann("-2")));
           const auto a = assemble(problem);
           check(with_dirichlet == !problem->dirichlet_nodes().empty(),
                 "Dirichlet nodes");
           near_vector(
               a.dirichlet_values(),
               at_nodes(*problem, dirichlet_node_list(*problem), linear_u),
               1e-15, "U_D");
           const auto u = at_nodes(*problem, problem->free_nodes(), linear_u);
           const auto lhs = a.stiffness().free.multiply(u);
           near_vector(lhs,
                       std::vector<double>(a.right_hand_side().begin(),
                                           a.right_hand_side().end()),
                       1e-12, "A U = b");
         }
       }},

      // ---------------------------------------------------------------
      // Elimination.
      {"the shrunk system is the full system without Dirichlet rows",
       [] {
         const auto mesh = pde2d::test::rectangle_mesh(2, 1, 0.3);
         auto with_time = [](BoundaryCondition left) {
           Problem p = linear_problem(std::move(left));
           p.equation.m1 = e("1 + x");
           p.equation.m2 = e("2 + y");
           p.initial = {.u = e("0"), .u_t = e("0")};
           return p;
         };
         const auto eliminated =
             meshed(mesh, with_time(dirichlet("(1 + 2*x + 3*y)*(1 + t^2)")));
         const auto full = meshed(mesh, with_time(neumann("0")));
         const double t = 0.7;
         const auto a = assemble(eliminated, t);
         const auto f = assemble(full, t);
         const auto &free = eliminated->free_nodes();
         const auto dirichlet_nodes = dirichlet_node_list(*eliminated);
         check(!dirichlet_nodes.empty(), "the left side is Dirichlet");
         const auto compare = [&](const Operator &shrunk, const Operator &whole,
                                  const char *what) {
           const Dense w = dense(whole.free);
           const Dense s = dense(shrunk.free);
           const Dense d = dense(shrunk.dirichlet);
           for (std::size_t i = 0; i < free.size(); ++i) {
             for (std::size_t j = 0; j < free.size(); ++j)
               check(s[i][j] == w[free[i]][free[j]], what);
             for (std::size_t k = 0; k < dirichlet_nodes.size(); ++k)
               check(d[i][k] == w[free[i]][dirichlet_nodes[k]], what);
           }
         };
         compare(a.stiffness(), f.stiffness(), "stiffness");
         compare(a.damping(), f.damping(), "damping");
         compare(a.mass(), f.mass(), "mass");
         for (std::size_t i = 0; i < free.size(); ++i)
           check(a.load()[i] == f.load()[free[i]], "load");

         // U_D = u (1 + t^2), U_D' = 2 t u, U_D'' = 2 u.
         std::vector<double> u_d, rate, acceleration;
         for (const std::size_t node : dirichlet_nodes) {
           const double u = linear_u(mesh->nodes[node]);
           u_d.push_back(u * (1 + t * t));
           rate.push_back(2 * t * u);
           acceleration.push_back(2 * u);
         }
         near_vector(a.dirichlet_values(), u_d, 1e-14, "U_D");
         near_vector(a.dirichlet_rates(), rate, 1e-14, "U_D'");
         near_vector(a.dirichlet_accelerations(), acceleration, 1e-14, "U_D''");
         const auto lift_a = pde2d::test::times(
             dense(a.stiffness().dirichlet),
             {a.dirichlet_values().begin(), a.dirichlet_values().end()});
         const auto lift_b = pde2d::test::times(
             dense(a.damping().dirichlet),
             {a.dirichlet_rates().begin(), a.dirichlet_rates().end()});
         const auto lift_m = pde2d::test::times(
             dense(a.mass().dirichlet), {a.dirichlet_accelerations().begin(),
                                         a.dirichlet_accelerations().end()});
         std::vector<double> b;
         for (std::size_t i = 0; i < free.size(); ++i)
           b.push_back(a.load()[i] - lift_a[i] - lift_b[i] - lift_m[i]);
         near_vector(a.right_hand_side(), b, 1e-12, "b");
         check(a.time_order() == 2, "time order 2");
       }},

      // ---------------------------------------------------------------
      // Time dependence and reuse.
      {"previous gives the same result bit for bit",
       [] {
         const auto mesh = pde2d::test::rectangle_mesh(2, 1, 0.3);
         // c and the bottom's alpha depend on t; m1, f, the rest do not.
         Problem p = linear_problem(dirichlet("t*x + y"));
         p.equation.c = e("0.5 + sin(t)");
         p.equation.m1 = e("1");
         p.boundary_conditions["bottom"].alpha = e("1 + t^2");
         p.initial = {.u = e("0")};
         const auto problem = meshed(mesh, p);
         const auto first = assemble(problem, 0.25);
         const auto again = assemble(problem, 1.5, &first);
         const auto fresh = assemble(problem, 1.5);
         check(same(again, fresh), "previous at t1 = fresh at t2");
         const auto third = assemble(problem, -3.0, &again);
         check(same(third, assemble(problem, -3.0)), "chained");
         check(!same(first, fresh), "the result does change with t");
         check(metric(again, "used_previous") == 1 &&
                   metric(fresh, "used_previous") == 0,
               "used_previous");
         check(metric(again, "stiffness_rebuilt") == 1, "stiffness rebuilt");
         check(metric(again, "damping_rebuilt") == 0 &&
                   metric(again, "load_rebuilt") == 0 &&
                   metric(again, "mass_rebuilt") == 0,
               "the others reused");
         check(metric(fresh, "damping_rebuilt") == 1, "fresh builds all");
         check(again.stiffness().depends_on_t && !again.damping().depends_on_t,
               "operator facts");
         check(!again.load_depends_on_t() &&
                   again.right_hand_side_depends_on_t(),
               "the Dirichlet value makes b depend on t");
       }},
      {"a t-dependent load and boundary data",
       [] {
         const auto mesh = pde2d::test::rectangle_mesh(2, 1, 0.4);
         Problem p = linear_problem(neumann("-2*t"));
         const auto problem = meshed(mesh, p);
         const auto first = assemble(problem, 1.0);
         const auto second = assemble(problem, 2.0, &first);
         check(same(second, assemble(problem, 2.0)), "bit for bit");
         check(second.load_depends_on_t() &&
                   second.right_hand_side_depends_on_t(),
               "load in t");
         check(metric(second, "stiffness_rebuilt") == 0 &&
                   metric(second, "load_rebuilt") == 1,
               "only the load rebuilt");
         check(!second.stiffness().depends_on_t, "stiffness constant");
       }},
      {"nothing in t: b is constant",
       [] {
         const auto problem = meshed(pde2d::test::rectangle_mesh(2, 1, 0.4),
                                     linear_problem(dirichlet("x")));
         const auto a = assemble(problem, 3.0);
         check(!a.load_depends_on_t() && !a.right_hand_side_depends_on_t(),
               "constant");
         check(a.dirichlet_rates().empty() &&
                   a.dirichlet_accelerations().empty(),
               "no rates for time order 0");
         const auto b = assemble(problem, 4.0, &a);
         check(metric(b, "stiffness_rebuilt") == 0 &&
                   metric(b, "load_rebuilt") == 0,
               "everything reused");
         check(same(a, b) && b.time() == 4.0, "same values");
       }},

      // ---------------------------------------------------------------
      // Facts.
      {"facts: time order, zero and t",
       [] {
         const Equation laplace{.a_xx = e("1"), .a_yy = e("1")};
         const auto a = assemble(on_triangle(laplace));
         check(a.time_order() == 0 && a.mass().always_zero &&
                   a.damping().always_zero && !a.stiffness().always_zero,
               "static");
         Equation heat = laplace;
         heat.m1 = e("1");
         check(assemble(on_triangle(heat, {}, {.u = e("0")})).time_order() == 1,
               "heat");
         Equation wave = laplace;
         wave.m2 = e("1");
         const auto w =
             assemble(on_triangle(wave, {}, {.u = e("0"), .u_t = e("0")}));
         check(w.time_order() == 2 && w.damping().always_zero, "wave");
         check(w.dirichlet_rates().empty(), "no Dirichlet nodes");
         // A dynamic condition with A = 0 has no D: the problem's time order
         // is 1, the assembled one 0.
         const auto no_a = on_triangle(
             {.c = e("1")}, {{"bottom", {.beta_n = e("1"), .gamma = e("1")}}},
             {.u = e("0")});
         check(time_order(*no_a) == 1, "problem: time order 1");
         const auto n = assemble(no_a);
         check(n.time_order() == 0 && n.damping().always_zero, "assembled: 0");
         Equation in_t = laplace;
         in_t.a_xx = e("1 + t");
         in_t.m2 = e("x");
         const auto tt =
             assemble(on_triangle(in_t, {}, {.u = e("0"), .u_t = e("0")}));
         check(tt.stiffness().depends_on_t && !tt.mass().depends_on_t,
               "depends_on_t");
         // u_n = t on the bottom: the load depends on t through G.
         const auto g =
             assemble(on_triangle(laplace, {{"bottom", neumann("t")}}));
         check(g.load_depends_on_t() && !g.stiffness().depends_on_t, "G in t");
       }},

      // ---------------------------------------------------------------
      // max_time_order.
      {"max_time_order builds exactly the terms of its level",
       [] {
         const auto problem = every_term_problem();
         const double t = 0.7;
         const auto full = assemble(problem, t);
         check(full.time_order() == 2 && !full.mass().always_zero &&
                   !full.damping().always_zero,
               "the full assembly has every term");
         check(same(Assembler().assemble(problem, t, nullptr, 2), full),
               "level 2 is the full assembly");
         check(metric(full, "max_time_order") == 2, "reported");

         const auto one = Assembler().assemble(problem, t, nullptr, 1);
         check(one.time_order() == 1, "level 1: time order 1");
         check(one.mass().always_zero && all_zero(one.mass()) &&
                   !one.mass().depends_on_t,
               "level 1: M2 left out");
         check(same(one.damping(), full.damping()) &&
                   same(one.stiffness(), full.stiffness()),
               "level 1: B and A as in the full assembly");
         check(std::ranges::equal(one.load(), full.load()) &&
                   std::ranges::equal(one.dirichlet_values(),
                                      full.dirichlet_values()) &&
                   std::ranges::equal(one.dirichlet_rates(),
                                      full.dirichlet_rates()) &&
                   one.dirichlet_accelerations().empty(),
               "level 1: load, U_D and U_D', no U_D''");
         near_vector(one.right_hand_side(), lifted(full, 1), tight,
                     "level 1: b = load - A_FD U_D - B_FD U_D'");
         check(metric(one, "mass_rebuilt") == 0, "M2 not built");

         const auto zero = Assembler().assemble(problem, t, nullptr, 0);
         check(zero.time_order() == 0, "level 0: time order 0");
         check(zero.mass().always_zero && zero.damping().always_zero &&
                   all_zero(zero.mass()) && all_zero(zero.damping()),
               "level 0: M2 and B left out");
         check(same(zero.stiffness(), full.stiffness()),
               "level 0: A as in the full assembly");
         check(std::ranges::equal(zero.load(), full.load()) &&
                   std::ranges::equal(zero.dirichlet_values(),
                                      full.dirichlet_values()) &&
                   zero.dirichlet_rates().empty() &&
                   zero.dirichlet_accelerations().empty(),
               "level 0: load and U_D only");
         near_vector(zero.right_hand_side(), lifted(full, 0), tight,
                     "level 0: b = load - A_FD U_D");
         check(zero.right_hand_side_depends_on_t(),
               "U_D and the reaction depend on t");
         check(metric(zero, "damping_rebuilt") == 0 &&
                   metric(zero, "stiffness_rebuilt") == 1,
               "B not built, A built");
       }},
      {"max_time_order: reuse across levels is bit for bit",
       [] {
         const auto problem = every_term_problem();
         for (int before = 0; before <= 2; ++before)
           for (int now = 0; now <= 2; ++now) {
             const std::string what = "from level " + std::to_string(before) +
                                      " to " + std::to_string(now);
             const auto first =
                 Assembler().assemble(problem, 0.3, nullptr, before);
             const auto again = Assembler().assemble(problem, 1.1, &first, now);
             check(
                 same(again, Assembler().assemble(problem, 1.1, nullptr, now)),
                 what + ": previous equals fresh");
             check(metric(again, "used_previous") == 1, what + ": reused");
             // A built operator is reused only when `first` built it.
             const double expected = now >= 2 ? 1.0 : 0.0; // M2 is in t
             check(metric(again, "mass_rebuilt") == expected,
                   what + ": mass_rebuilt");
             // And on again, one level further.
             for (int next = 0; next <= 2; ++next)
               check(same(Assembler().assemble(problem, -0.4, &again, next),
                          Assembler().assemble(problem, -0.4, nullptr, next)),
                     what + ", then " + std::to_string(next) + ": chained");
           }
         // A constant operator is copied only when `previous` built it.
         const auto steady =
             meshed(pde2d::test::rectangle_mesh(2, 1, 0.4),
                    {.equation = {.a_xx = e("1"), .a_yy = e("1"), .m1 = e("2")},
                     .boundary_conditions = {{"left", dirichlet("t*y")},
                                             {"bottom", neumann()},
                                             {"right", neumann("1")},
                                             {"top", neumann()}},
                     .initial = {.u = e("0")}});
         const auto low = Assembler().assemble(steady, 0.0, nullptr, 0);
         const auto high = Assembler().assemble(steady, 1.0, &low, 1);
         check(metric(high, "damping_rebuilt") == 1 &&
                   metric(high, "stiffness_rebuilt") == 0,
               "B built fresh, A reused");
         check(same(high, Assembler().assemble(steady, 1.0, nullptr, 1)),
               "bit for bit");
         check(!high.damping().always_zero, "B is there");
       }},
      {"max_time_order: other values throw",
       [] {
         const auto problem = every_term_problem();
         for (const int level : {-1, 3, 100})
           expect_throws<std::invalid_argument>(
               [&] {
                 (void)Assembler().assemble(problem, 0.0, nullptr, level);
               },
               "max_time_order must be 0, 1 or 2, not " +
                   std::to_string(level));
       }},
      {"max_time_order 0 never evaluates U_D' (issue #35)",
       [] {
         // u = sqrt(t) on the boundary: d/dt sqrt(t) is not finite at 0.
         const auto problem = meshed(
             pde2d::test::rectangle_mesh(1, 1, 0.5),
             {.equation = {.a_xx = e("-1"), .a_yy = e("-1"), .m1 = e("1")},
              .boundary_conditions = {{"left", dirichlet("sqrt(t)")},
                                      {"bottom", dirichlet("sqrt(t)")},
                                      {"right", dirichlet("sqrt(t)")},
                                      {"top", dirichlet("sqrt(t)")}},
              .initial = {.u = e("0")}});
         expect_throws<std::invalid_argument>(
             [&] { (void)assemble(problem, 0.0); }, "not finite");
         const auto a = Assembler().assemble(problem, 0.0, nullptr, 0);
         check(a.dirichlet_rates().empty(), "no rates");
         check(std::ranges::all_of(a.dirichlet_values(),
                                   [](double v) { return v == 0.0; }),
               "U_D = 0 at t = 0");
       }},

      // ---------------------------------------------------------------
      // Symmetry facts, each checked against the matrix itself.
      {"symmetry: isotropic diffusion with natural conditions",
       [] {
         const Equation laplace{.a_xx = e("-1"), .a_yy = e("-1")};
         symmetric_case(laplace, neumann("2"), true, "isotropic");
         symmetric_case(laplace, {.alpha = e("3"), .beta_n = e("1")}, true,
                        "Robin");
         // Isotropic only after expanding; constant in x and y, so div A
         // (a convection) is zero.
         symmetric_case({.a_xx = e("-(t + 2)*(1 + t)"),
                         .a_yy = e("-2 - 3*t - t^2"),
                         .c = e("1 + x")},
                        neumann(), true, "isotropic after expanding, with R");
         symmetric_case({.a_xx = e("0"), .c = e("2")}, neumann(), true,
                        "A = 0");
       }},
      {"symmetry: what makes A non-symmetric",
       [] {
         symmetric_case({.a_xx = e("-1"), .a_xy = e("0.5"), .a_yy = e("-1")},
                        neumann(), false, "a_xy");
         symmetric_case({.a_xx = e("-1"), .a_yy = e("-2")}, neumann(), false,
                        "a_xx != a_yy");
         symmetric_case({.a_xx = e("-1"), .a_yy = e("-1")},
                        {.beta_n = e("1"), .beta_tau = e("0.5")}, false,
                        "beta_tau");
         symmetric_case({.a_xx = e("-1"), .a_yy = e("-1")},
                        {.beta_n = e("1"), .beta_x = e("0.5")}, false,
                        "beta_x");
         symmetric_case({.a_xx = e("-1"), .a_yy = e("-1"), .b_x = e("1")},
                        neumann(), false, "convection");
         symmetric_case({.a_xx = e("-x"), .a_yy = e("-x")}, neumann(), false,
                        "div A (convection from a varying A)");
       }},
      {"symmetry: masses and the isotropy of A",
       [] {
         const auto problem = every_term_problem();
         const auto a = assemble(problem, 0.5);
         check(a.mass().always_symmetric && a.damping().always_symmetric,
               "M2 and B");
         check(asymmetry(a.mass().free) < 1e-14 &&
                   asymmetry(a.damping().free) < 1e-14,
               "measured");
         check(problem->a_is_isotropic(), "a_xx = a_yy = 2");
         const auto zero = Assembler().assemble(problem, 0.5, nullptr, 0);
         check(zero.mass().always_symmetric && zero.damping().always_symmetric,
               "a left-out operator is symmetric");
         check(on_triangle({.a_xx = e("1 + x"), .a_yy = e("x + 1")})
                   ->a_is_isotropic(),
               "equal as written");
         check(on_triangle({})->a_is_isotropic(), "A = 0");
         check(!on_triangle({.a_xx = e("1"), .a_yy = e("2")})->a_is_isotropic(),
               "a_xx != a_yy");
         check(!on_triangle(
                    {.a_xx = e("1"), .a_xy = e("x - x + 1"), .a_yy = e("1")})
                    ->a_is_isotropic(),
               "a_xy not zero");
       }},

      // ---------------------------------------------------------------
      // Errors.
      {"errors",
       [] {
         const Equation laplace{.a_xx = e("1"), .a_yy = e("1")};
         const auto p = on_triangle(laplace);
         const auto q = on_triangle(laplace);
         const auto a = assemble(p);
         expect_throws<std::invalid_argument>(
             [&] { (void)assemble(q, 0.0, &a); },
             "built from another MeshedProblem object");
         expect_throws<std::invalid_argument>([&] { (void)assemble(nullptr); },
                                              "not null");
         for (const double t : {std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::infinity()})
           expect_throws<std::invalid_argument>([&] { (void)assemble(p, t); },
                                                "is not finite");
         Equation huge = laplace;
         huge.c = e("exp(2000*x)");
         expect_throws<std::invalid_argument>(
             [&] { (void)assemble(on_triangle(huge)); },
             "Equation coefficient c is not finite at (");
         expect_throws<std::invalid_argument>(
             [&] {
               (void)assemble(on_triangle(
                   laplace, {{"bottom",
                              {.alpha = e("exp(2000*x)"), .beta_n = e("1")}}}));
             },
             "Region 'bottom', coefficient alpha is not finite");
         // n = (0, -1) on the bottom: beta_n + beta_y n_y = 1 - 1 = 0.
         expect_throws<std::invalid_argument>(
             [&] {
               (void)assemble(on_triangle(
                   laplace,
                   {{"bottom", {.beta_n = e("1"), .beta_y = e("1")}}}));
             },
             "Region 'bottom': beta_n + beta_x n_x + beta_y n_y is zero");
       }},

      // ---------------------------------------------------------------
      // Colouring: only its effect is visible.
      {"repeated runs give identical results",
       [] {
         const auto mesh = pde2d::test::rectangle_mesh(2, 1, 0.2);
         const auto problem =
             meshed(mesh, linear_problem(dirichlet("1 + 2*x + 3*y")));
         const auto a = assemble(problem);
         check(same(a, assemble(problem)), "same problem object");
         check(same(a, assemble(meshed(
                           mesh, linear_problem(dirichlet("1 + 2*x + 3*y"))))),
               "another object with the same input");
         // No two triangles of one colour share a node, so there are at
         // least as many colours as triangles around the busiest node.
         std::vector<int> around(mesh->nodes.size(), 0);
         for (const auto &tri : mesh->triangles)
           for (const std::size_t node : tri.nodes)
             ++around[node];
         check(metric(a, "triangle_colours") >=
                   *std::ranges::max_element(around),
               "colours");
         check(metric(a, "edge_colours") >= 2, "edge colours");
         check(metric(a, "free_nodes") ==
                   static_cast<double>(problem->free_nodes().size()),
               "free_nodes");
         check(metric(a, "free_nonzeros") ==
                   static_cast<double>(a.stiffness().free.values.size()),
               "free_nonzeros");
         check(a.report().stage == "assembly", "stage name");
       }},
      {"review: reuse still checks a normal coefficient that depends on t",
       [] {
         // (1 - t) u_n = 0 on the bottom: every term there is constant or
         // zero, but at t = 1 the condition cannot be solved for u_n.
         const Equation laplace{.a_xx = e("1"), .a_yy = e("1")};
         const auto problem = on_triangle(
             laplace, {{"bottom", {.beta_n = e("1-t"), .g = e("0")}}});
         const auto first = assemble(problem, 0.0);
         expect_throws<std::invalid_argument>(
             [&] { (void)assemble(problem, 1.0); },
             "cannot be solved for the normal derivative");
         expect_throws<std::invalid_argument>(
             [&] { (void)assemble(problem, 1.0, &first); },
             "cannot be solved for the normal derivative");
       }},
      {"review: a tiny normal coefficient does not spoil a zero term",
       [] {
         // exp(-710) is a nonzero denormal number, so 1 / beta_n overflows;
         // the condition is still u_n = 0 and must give the same matrix.
         const Equation laplace{.a_xx = e("1"), .a_yy = e("1")};
         const auto tiny = on_triangle(
             laplace, {{"bottom", {.beta_n = e("exp(-710*t)"), .g = e("0")}}});
         const auto plain = on_triangle(laplace);
         const auto a = assemble(tiny, 1.0);
         const auto b = assemble(plain, 1.0);
         check(a.stiffness().free.values == b.stiffness().free.values,
               "same stiffness as u_n = 0");
       }},
      {"review: large coefficients do not overflow before the weight",
       [] {
         // K00 of the reference triangle for A = I is 1, so the entry is
         // -1e308, which is representable.
         const auto a =
             assemble(on_triangle({.a_xx = e("1e308"), .a_yy = e("1e308")}));
         near(a.stiffness().free.entry(0, 0), -1e308, 1e294, "K00");
       }},
      {"review: a triangle outside the range of double is an error",
       [] {
         for (const double scale : {1e-200, 1e200}) {
           auto scaled =
               std::make_shared<Mesh>(*pde2d::test::reference_triangle());
           for (auto &node : scaled->nodes)
             node = {node.x * scale, node.y * scale};
           Problem problem{.equation = {.a_xx = e("1"), .a_yy = e("1")}};
           for (const char *region : {"bottom", "diagonal", "left"})
             problem.boundary_conditions.emplace(region, neumann());
           const auto combined = meshed(scaled, problem);
           expect_throws<std::invalid_argument>(
               [&] { (void)assemble(combined); },
               "too small or too large for double arithmetic");
         }
       }},
      {"review: lifting includes the dynamic boundary matrix",
       [] {
         // u = exp(t) (1 + x + 2y) solves u_t - Laplace(u) - u = 0. Left
         // edge Dirichlet; on the others u + 3 u_n + 2 u_t = g. Only node 1
         // is free, and its two edges touch Dirichlet corners, so the
         // right-hand side needs D_FD U_D'.
         const auto problem = on_triangle(
             {.a_xx = e("-1"), .a_yy = e("-1"), .c = e("-1"), .m1 = e("1")},
             {{"left", dirichlet("exp(t)*(1+x+2*y)")},
              {"bottom",
               {.alpha = e("1"),
                .beta_n = e("3"),
                .gamma = e("2"),
                .g = e("exp(t)*(3*(1+x+2*y)-6)")}},
              {"diagonal",
               {.alpha = e("1"),
                .beta_n = e("3"),
                .gamma = e("2"),
                .g = e("exp(t)*(3*(1+x+2*y)+9/sqrt(2))")}}},
             {.u = e("1+x+2*y")});
         const double t = 0.3;
         const auto a = assemble(problem, t);
         check(problem->free_nodes() == std::vector<std::size_t>{1},
               "node 1 is the free node");
         // u = u_t = 2 exp(t) at node 1 = (1, 0).
         const std::vector<double> exact{2.0 * std::exp(t)};
         const double residual = a.damping().free.multiply(exact)[0] +
                                 a.stiffness().free.multiply(exact)[0] -
                                 a.right_hand_side()[0];
         near(residual, 0.0, 1e-12, "residual at the exact solution");
         check(
             a.damping().dirichlet.values !=
                 std::vector<double>(a.damping().dirichlet.values.size(), 0.0),
             "the damping has Dirichlet columns");
       }},
  });
}

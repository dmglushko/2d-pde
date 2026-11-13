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

std::shared_ptr<const MeshedProblem>
meshed(std::shared_ptr<const Mesh> mesh, const Problem &problem) {
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
         a.depends_on_t == b.depends_on_t && a.is_zero == b.is_zero;
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

} // namespace

int main() {

  return pde2d::test::run({
      // ---------------------------------------------------------------
      // Hand-computed blocks on the reference triangle (all nodes free).
      {"K for A = identity",
       [] {
         // On the diagonal edge kappa_tau = n . tau = 0, so T is zero.
         const auto a =
             assemble(on_triangle({.a_xx = e("1"), .a_yy = e("1")}));
         near_matrix(dense(a.stiffness().free), scaled(k_identity, -1), tight,
                     "-K");
         check(a.stiffness().dirichlet.columns() == 0, "no Dirichlet columns");
         check(a.mass().is_zero && a.damping().is_zero, "no time terms");
       }},
      {"K and T for an anisotropic A",
       [] {
         // A = [[2, 0.5], [0.5, 3]] (a_xy = 1), Neumann u_n = 0 everywhere.
         // K = area * grad phi_i . A grad phi_j.
         const Dense k{{3, -1.25, -1.75}, {-1.25, 1, 0.25}, {-1.75, 0.25, 1.5}};
         // T per edge: kappa_tau / 2 * [[-1, 1], [-1, 1]], kappa_tau =
         // n . A tau: -0.5 on the bottom, 0.5 on the diagonal and the left.
         const Dense t{{0.5, -0.25, -0.25}, {0.25, -0.5, 0.25}, {0.25, -0.25, 0}};
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
         const auto r = assemble(on_triangle(
             {.c = e("1"), .m1 = e("2"), .m2 = e("3"), .f = e("1")},
             {}, {.u = e("0"), .u_t = e("0")}));
         near_matrix(dense(r.stiffness().free), mass_matrix, tight, "R");
         near_matrix(dense(r.damping().free), scaled(mass_matrix, 2), tight,
                     "M1");
         near_matrix(dense(r.mass().free), scaled(mass_matrix, 3), tight,
                     "M2");
         near_vector(r.load(), {1.0 / 6, 1.0 / 6, 1.0 / 6}, tight, "F");
       }},
      {"Robin: S",
       [] {
         // Bottom: 2 u + u_n = 0, A = identity: kappa_n alpha / beta_n = 2.
         const auto a = assemble(on_triangle(
             {.a_xx = e("1"), .a_yy = e("1")},
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
         const auto a = assemble(
             on_triangle({.a_xx = e("1"), .a_yy = e("1")},
                         {{"bottom", {.beta_n = e("1"), .gamma = e("4")}}},
                         {.u = e("0")}));
         near_matrix(dense(a.damping().free),
                     scaled(on_edge(0, 1, edge_mass), -4), tight, "-D");
         check(!a.damping().is_zero && a.time_order() == 1,
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
         const auto a = assemble(meshed(
             mesh, {.equation = {.a_xx = e("1"),
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
               mesh, linear_problem(with_dirichlet
                                        ? dirichlet("1 + 2*x + 3*y")
                                        : neumann("-2")));
           const auto a = assemble(problem);
           check(with_dirichlet == !problem->dirichlet_nodes().empty(),
                 "Dirichlet nodes");
           near_vector(a.dirichlet_values(),
                       at_nodes(*problem, dirichlet_node_list(*problem),
                                linear_u),
                       1e-15, "U_D");
           const auto u =
               at_nodes(*problem, problem->free_nodes(), linear_u);
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
         const auto eliminated = meshed(
             mesh, with_time(dirichlet("(1 + 2*x + 3*y)*(1 + t^2)")));
         const auto full = meshed(mesh, with_time(neumann("0")));
         const double t = 0.7;
         const auto a = assemble(eliminated, t);
         const auto f = assemble(full, t);
         const auto &free = eliminated->free_nodes();
         const auto dirichlet_nodes = dirichlet_node_list(*eliminated);
         check(!dirichlet_nodes.empty(), "the left side is Dirichlet");
         const auto compare = [&](const Operator &shrunk,
                                  const Operator &whole, const char *what) {
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
         near_vector(a.dirichlet_accelerations(), acceleration, 1e-14,
                     "U_D''");
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
         check(a.time_order() == 0 && a.mass().is_zero &&
                   a.damping().is_zero && !a.stiffness().is_zero,
               "static");
         Equation heat = laplace;
         heat.m1 = e("1");
         check(assemble(on_triangle(heat, {}, {.u = e("0")})).time_order() ==
                   1,
               "heat");
         Equation wave = laplace;
         wave.m2 = e("1");
         const auto w =
             assemble(on_triangle(wave, {}, {.u = e("0"), .u_t = e("0")}));
         check(w.time_order() == 2 && w.damping().is_zero, "wave");
         check(w.dirichlet_rates().empty(), "no Dirichlet nodes");
         // A dynamic condition with A = 0 has no D: the problem's time order
         // is 1, the assembled one 0.
         const auto no_a =
             on_triangle({.c = e("1")},
                         {{"bottom", {.beta_n = e("1"), .gamma = e("1")}}},
                         {.u = e("0")});
         check(time_order(*no_a) == 1, "problem: time order 1");
         const auto n = assemble(no_a);
         check(n.time_order() == 0 && n.damping().is_zero, "assembled: 0");
         Equation in_t = laplace;
         in_t.a_xx = e("1 + t");
         in_t.m2 = e("x");
         const auto tt = assemble(
             on_triangle(in_t, {}, {.u = e("0"), .u_t = e("0")}));
         check(tt.stiffness().depends_on_t && !tt.mass().depends_on_t,
               "depends_on_t");
         // u_n = t on the bottom: the load depends on t through G.
         const auto g = assemble(on_triangle(laplace, {{"bottom", neumann("t")}}));
         check(g.load_depends_on_t() && !g.stiffness().depends_on_t, "G in t");
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
         expect_throws<std::invalid_argument>(
             [&] { (void)assemble(nullptr); }, "not null");
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
                   laplace,
                   {{"bottom",
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
         check(same(a, assemble(meshed(mesh, linear_problem(dirichlet(
                                                 "1 + 2*x + 3*y"))))),
               "another object with the same input");
         // No two triangles of one colour share a node, so there are at
         // least as many colours as triangles around the busiest node.
         std::vector<int> around(mesh->nodes.size(), 0);
         for (const auto &tri : mesh->triangles)
           for (const std::size_t node : tri.nodes)
             ++around[node];
         check(metric(a, "triangle_colours") >= *std::ranges::max_element(around),
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
  });
}

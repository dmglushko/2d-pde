#include "pde2d/fem/meshed_problem.hpp"
#include "support/check.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace pde2d::fem;
using pde2d::mesh::Mesh;
using pde2d::test::check;
using pde2d::test::expect_throws;

namespace {

Expression e(const char *text) { return Expression(text); }

// The unit square as a 3 x 3 grid of nodes, numbered row by row from the
// bottom left:
//   6 7 8
//   3 4 5
//   0 1 2
// The four sides get the region names in `sides` (bottom, right, top,
// left; a name may repeat). Region ids follow the first appearance.
std::shared_ptr<const Mesh> grid(const std::array<std::string, 4> &sides) {
  Mesh mesh;
  for (int j = 0; j < 3; ++j)
    for (int i = 0; i < 3; ++i)
      mesh.nodes.push_back({0.5 * i, 0.5 * j});
  for (std::size_t j = 0; j < 2; ++j)
    for (std::size_t i = 0; i < 2; ++i) {
      const std::size_t a = i + 3 * j, b = a + 1, c = a + 4, d = a + 3;
      mesh.triangles.push_back({{a, b, c}});
      mesh.triangles.push_back({{a, c, d}});
    }
  std::array<std::size_t, 4> ids{};
  for (std::size_t s = 0; s < 4; ++s) {
    const auto found = std::ranges::find(mesh.region_names, sides[s]);
    ids[s] = static_cast<std::size_t>(found - mesh.region_names.begin());
    if (found == mesh.region_names.end())
      mesh.region_names.push_back(sides[s]);
  }
  // Counterclockwise, the domain on the left.
  const std::array<std::size_t, 9> loop{0, 1, 2, 5, 8, 7, 6, 3, 0};
  for (std::size_t k = 0; k < 8; ++k)
    mesh.boundary_edges.push_back(
        {.nodes = {loop[k], loop[k + 1]}, .region = ids[k / 2], .link = {}});
  return std::make_shared<const Mesh>(std::move(mesh));
}

std::shared_ptr<const Mesh> square() {
  return grid({"bottom", "right", "top", "left"});
}

BoundaryCondition dirichlet(const char *g, int priority = 0) {
  return {.alpha = e("1"), .g = e(g), .priority = priority};
}

BoundaryCondition neumann() { return {.beta_n = e("1")}; }

// -u_xx - u_yy = 1 with the given conditions.
Problem poisson(std::map<std::string, BoundaryCondition> conditions) {
  return {.equation = {.a_xx = e("-1"), .a_yy = e("-1"), .f = e("1")},
          .boundary_conditions = std::move(conditions)};
}

std::vector<std::size_t> dirichlet_node_list(const MeshedProblem &problem) {
  std::vector<std::size_t> nodes;
  for (const auto &entry : problem.dirichlet_nodes())
    nodes.push_back(entry.node);
  return nodes;
}

std::size_t index_of(const MeshedProblem &problem, std::size_t node) {
  for (std::size_t k = 0; k < problem.dirichlet_nodes().size(); ++k)
    if (problem.dirichlet_nodes()[k].node == node)
      return k;
  throw std::runtime_error("not a Dirichlet node");
}

double metric(const MeshedProblem &problem, const std::string &name) {
  return problem.report().metrics.at(name);
}

} // namespace

int main() {
  return pde2d::test::run({
      {"a null or invalid mesh is rejected",
       [] {
         expect_throws<std::invalid_argument>(
             [] { MeshedProblem p(nullptr, poisson({})); }, "needs a mesh");
         Mesh broken = *grid({"wall", "wall", "wall", "wall"});
         std::swap(broken.triangles[0].nodes[0], broken.triangles[0].nodes[1]);
         expect_throws<std::invalid_argument>(
             [&] {
               MeshedProblem p(std::make_shared<const Mesh>(broken),
                               poisson({{"wall", dirichlet("0")}}));
             },
             "Triangle 0 is not counterclockwise");
       }},
      {"region names are matched both ways",
       [] {
         auto conditions = std::map<std::string, BoundaryCondition>{
             {"bottom", dirichlet("0")},
             {"right", dirichlet("0")},
             {"top", dirichlet("0")},
             {"left", dirichlet("0")},
             {"inlet", dirichlet("0")}};
         expect_throws<std::invalid_argument>(
             [&] { MeshedProblem p(square(), poisson(conditions)); },
             "boundary condition for region 'inlet', but the mesh has no "
             "region of that name (mesh regions: 'bottom', 'right', 'top', "
             "'left')");
         conditions.erase("inlet");
         conditions.erase("top");
         expect_throws<std::invalid_argument>(
             [&] { MeshedProblem p(square(), poisson(conditions)); },
             "Mesh region 'top' has no boundary condition");
       }},
      {"classification errors name the region",
       [] {
         expect_throws<std::invalid_argument>(
             [] {
               MeshedProblem p(
                   grid({"wall", "wall", "wall", "lid"}),
                   poisson({{"wall", dirichlet("0")},
                            {"lid", {.alpha = e("1"), .gamma = e("1")}}}));
             },
             "Region 'lid': the boundary condition is a boundary ODE");
         expect_throws<std::invalid_argument>(
             [] {
               MeshedProblem p(grid({"wall", "wall", "wall", "lid"}),
                               poisson({{"wall", dirichlet("0")},
                                        {"lid", {.beta_tau = e("1")}}}));
             },
             "Region 'lid': the boundary condition has only a tangential");
       }},
      {"conditions are classified and kept per region id",
       [] {
         // The dynamic condition makes the problem first order in time.
         Problem problem =
             poisson({{"bottom", dirichlet("0")},
                      {"right", neumann()},
                      {"top", {.beta_n = e("1"), .beta_x = e("1")}},
                      {"left", {.beta_n = e("1"), .gamma = e("1")}}});
         problem.initial.u = e("0");
         const MeshedProblem p(square(), problem);
         const auto &c = p.conditions();
         check(c.size() == 4, "one per region");
         check(c[0].kind == ConditionKind::dirichlet && !c[0].oblique,
               "bottom");
         check(c[1].kind == ConditionKind::natural && !c[1].oblique, "right");
         check(c[2].kind == ConditionKind::natural && c[2].oblique, "top");
         check(c[3].kind == ConditionKind::dynamic, "left");
       }},
      {"all Dirichlet: one free node",
       [] {
         const MeshedProblem p(grid({"wall", "wall", "wall", "wall"}),
                               poisson({{"wall", dirichlet("0")}}));
         check(dirichlet_node_list(p) ==
                   std::vector<std::size_t>{0, 1, 2, 3, 5, 6, 7, 8},
               "boundary nodes");
         check(p.free_nodes() == std::vector<std::size_t>{4}, "free nodes");
         check(p.free_index(4) == std::size_t{0}, "free index of the centre");
         check(!p.free_index(0).has_value(), "no free index on the boundary");
         expect_throws<std::invalid_argument>([&] { (void)p.free_index(9); },
                                              "Node 9 does not exist");
         for (const auto &entry : p.dirichlet_nodes())
           check(entry.regions == std::vector<std::size_t>{0}, "one region");
         check(metric(p, "corners") == 0, "one region: no corners");
         check(metric(p, "dirichlet_nodes") == 8 &&
                   metric(p, "free_nodes") == 1,
               "counts");
         check(p.report().stage == "meshed_problem", "stage name");
       }},
      {"Dirichlet beats natural at a corner",
       [] {
         const MeshedProblem p(square(), poisson({{"bottom", dirichlet("0")},
                                                  {"right", neumann()},
                                                  {"top", neumann()},
                                                  {"left", neumann()}}));
         check(dirichlet_node_list(p) == std::vector<std::size_t>{0, 1, 2},
               "the bottom side, corners included");
         check(p.free_nodes() == std::vector<std::size_t>{3, 4, 5, 6, 7, 8},
               "free nodes");
         check(p.free_index(3) == std::size_t{0} &&
                   p.free_index(8) == std::size_t{5},
               "renumbered");
         check(metric(p, "corners") == 4, "four corners");
         check(metric(p, "dirichlet_over_natural_corners") == 2, "two");
         check(p.report().messages.empty(), "nothing to report");
       }},
      {"the higher priority wins at a corner, and the report says so",
       [] {
         const MeshedProblem p(square(),
                               poisson({{"bottom", dirichlet("0", 0)},
                                        {"right", neumann()},
                                        {"top", neumann()},
                                        {"left", dirichlet("1", 1)}}));
         check(dirichlet_node_list(p) ==
                   std::vector<std::size_t>{0, 1, 2, 3, 6},
               "bottom and left");
         const std::size_t k = index_of(p, 0);
         check(p.dirichlet_nodes()[k].regions == std::vector<std::size_t>{3},
               "left wins");
         check(p.dirichlet_value(k, 0.0) == 1.0, "value of left");
         check(metric(p, "priority_corners") == 1 &&
                   metric(p, "averaged_corners") == 0 &&
                   metric(p, "differing_corners") == 1,
               "metrics");
         check(p.report().messages.size() == 1, "one message");
         check(p.report().messages[0] ==
                   "Corner at node 0 (0, 0): region 'left' (priority 1) gives "
                   "1 and wins over region 'bottom' (priority 0), which gives "
                   "0",
               p.report().messages[0]);
         // The other way round.
         const MeshedProblem q(square(),
                               poisson({{"bottom", dirichlet("0", 2)},
                                        {"right", neumann()},
                                        {"top", neumann()},
                                        {"left", dirichlet("1", -1)}}));
         check(q.dirichlet_value(index_of(q, 0), 0.0) == 0.0, "bottom wins");
       }},
      {"equal priorities are averaged, and the report says so",
       [] {
         const MeshedProblem p(square(), poisson({{"bottom", dirichlet("0")},
                                                  {"right", neumann()},
                                                  {"top", neumann()},
                                                  {"left", dirichlet("1")}}));
         const std::size_t k = index_of(p, 0);
         check(p.dirichlet_nodes()[k].regions == std::vector<std::size_t>{0, 3},
               "both regions");
         check(p.dirichlet_value(k, 0.0) == 0.5, "average");
         check(p.dirichlet_value(index_of(p, 3), 0.0) == 1.0, "left side");
         check(metric(p, "averaged_corners") == 1, "averaged");
         check(p.report().messages.size() == 1 &&
                   p.report().messages[0] ==
                       "Corner at node 0 (0, 0): regions 'bottom' and 'left' "
                       "(both priority 0) give 0 and 1; their average 0.5 is "
                       "used",
               "message");
       }},
      {"corners whose values agree are not reported",
       [] {
         // Different expressions, the same value at the corner (0, 0).
         const MeshedProblem p(square(), poisson({{"bottom", dirichlet("x")},
                                                  {"right", neumann()},
                                                  {"top", neumann()},
                                                  {"left", dirichlet("y")}}));
         check(p.report().messages.empty(), "x = y = 0 at the corner");
         check(metric(p, "averaged_corners") == 1 &&
                   metric(p, "differing_corners") == 0,
               "metrics");
         check(p.dirichlet_value(index_of(p, 0), 0.0) == 0.0, "value");
         // All four corners between Dirichlet sides with the same g.
         const MeshedProblem all(square(), poisson({{"bottom", dirichlet("t")},
                                                    {"right", dirichlet("t")},
                                                    {"top", dirichlet("t")},
                                                    {"left", dirichlet("t")}}));
         check(all.report().messages.empty(), "same as written, even in t");
         check(metric(all, "averaged_corners") == 4, "four corners");
       }},
      {"time-dependent competing values are reported as possibly different",
       [] {
         const MeshedProblem p(square(),
                               poisson({{"bottom", dirichlet("sin(t)")},
                                        {"right", neumann()},
                                        {"top", neumann()},
                                        {"left", dirichlet("cos(t)")}}));
         check(p.report().messages.size() == 1 &&
                   p.report().messages[0].find(
                       "depend on t and may differ; their average is used") !=
                       std::string::npos,
               "message");
         const std::size_t k = index_of(p, 0);
         check(std::abs(p.dirichlet_value(k, 1.0) -
                        (std::sin(1.0) + std::cos(1.0)) / 2) < 1e-15,
               "average at t = 1");
       }},
      {"Dirichlet values are g / alpha",
       [] {
         const MeshedProblem p(
             square(),
             poisson({{"bottom", neumann()},
                      {"right", {.alpha = e("2"), .g = e("x + 3*y + t")}},
                      {"top", neumann()},
                      {"left", neumann()}}));
         check(dirichlet_node_list(p) == std::vector<std::size_t>{2, 5, 8},
               "the right side");
         // Node 5 is (1, 0.5): (1 + 1.5 + 1) / 2 at t = 1.
         check(p.dirichlet_value(index_of(p, 5), 1.0) == 1.75, "node 5");
         check(p.dirichlet_value(index_of(p, 8), 0.0) == 2.0, "node 8");
         expect_throws<std::invalid_argument>(
             [&] { (void)p.dirichlet_value(3, 0.0); },
             "Dirichlet node 3 does not exist");
         const MeshedProblem q(
             square(), poisson({{"bottom", neumann()},
                                {"right", neumann()},
                                {"top", neumann()},
                                {"left", {.alpha = e("x"), .g = e("1")}}}));
         expect_throws<std::invalid_argument>(
             [&] { (void)q.dirichlet_value(0, 0.0); },
             "The Dirichlet value g / alpha of region 'left' is not finite at "
             "node 0 (0, 0), t = 0");
       }},
      {"compiled coefficients against hand-computed values",
       [] {
         Problem problem = poisson({{"wall",
                                     {.alpha = e("h"),
                                      .beta_n = e("1"),
                                      .beta_x = e("x*y"),
                                      .g = e("h*(20 + t)")}}});
         problem.equation.a_xx = e("k*x");
         problem.equation.b_y = e("y^2 - t");
         problem.equation.f = e("x*y*t");
         problem.parameters = {{"k", 3.0}, {"h", 0.5}};
         const MeshedProblem p(grid({"wall", "wall", "wall", "wall"}), problem);
         const auto &eq = p.equation();
         check(eq.a_xx(2.0, 5.0, 0.0) == 6.0, "a_xx = k x");
         check(eq.a_yy(2.0, 5.0, 0.0) == -1.0, "a_yy");
         check(eq.a_xy(2.0, 5.0, 0.0) == 0.0 && eq.a_xy.facts().zero,
               "a_xy unset");
         check(eq.b_y(1.0, 3.0, 4.0) == 5.0, "b_y = 9 - 4");
         check(eq.f(2.0, 3.0, 4.0) == 24.0, "f = x y t");
         const auto &wall = p.conditions()[0];
         check(wall.alpha(9.0, 9.0, 9.0) == 0.5, "alpha = h");
         check(wall.beta_x(2.0, 3.0, 0.0) == 6.0, "beta_x = x y");
         check(wall.g(0.0, 0.0, 2.0) == 11.0, "g = h (20 + t)");
         check(wall.oblique && wall.kind == ConditionKind::natural, "Robin");
         check(wall.beta_tau.facts().zero && !wall.beta_x.facts().zero,
               "facts");
       }},
      {"compile errors name the coefficient",
       [] {
         Problem problem = poisson({{"wall", dirichlet("q")}});
         expect_throws<std::invalid_argument>(
             [&] {
               MeshedProblem p(grid({"wall", "wall", "wall", "wall"}), problem);
             },
             "Region 'wall', coefficient g: The expression 'q' uses 'q', "
             "which is not a parameter");
         problem = poisson({{"wall", dirichlet("0")}});
         problem.equation.c = e("k*x");
         expect_throws<std::invalid_argument>(
             [&] {
               MeshedProblem p(grid({"wall", "wall", "wall", "wall"}), problem);
             },
             "Equation coefficient c: The expression 'k*x' uses 'k'");
         problem.parameters = {{"k", 1.0}, {"pi", 3.0}};
         expect_throws<std::invalid_argument>(
             [&] {
               MeshedProblem p(grid({"wall", "wall", "wall", "wall"}), problem);
             },
             "Parameter 'pi' is not a plain symbol name");
       }},
      {"initial conditions follow the time order",
       [] {
         const auto wall = grid({"wall", "wall", "wall", "wall"});
         Problem problem = poisson({{"wall", dirichlet("0")}});
         problem.initial.u = e("x");
         expect_throws<std::invalid_argument>(
             [&] { MeshedProblem p(wall, problem); },
             "The problem has time order 0, so it takes no initial "
             "condition u, but it is set");
         problem.equation.m1 = e("1");
         problem.initial.u.reset();
         expect_throws<std::invalid_argument>(
             [&] { MeshedProblem p(wall, problem); },
             "The problem has time order 1, so it needs the initial "
             "condition u");
         problem.initial = {.u = e("x"), .u_t = e("0")};
         expect_throws<std::invalid_argument>(
             [&] { MeshedProblem p(wall, problem); },
             "takes no initial condition u_t");
         problem.equation.m2 = e("1");
         problem.initial.u_t.reset();
         expect_throws<std::invalid_argument>(
             [&] { MeshedProblem p(wall, problem); },
             "The problem has time order 2, so it needs the initial "
             "condition u_t");
         problem.initial = {.u = e("x*t"), .u_t = e("0")};
         expect_throws<std::invalid_argument>(
             [&] { MeshedProblem p(wall, problem); },
             "The initial condition u depends on t");
         problem.initial = {.u = e("1/x"), .u_t = e("0")};
         expect_throws<std::invalid_argument>(
             [&] { MeshedProblem p(wall, problem); },
             "The initial condition u is not finite at node 0 (0, 0)");
       }},
      {"initial conditions are evaluated at the nodes",
       [] {
         const auto wall = grid({"wall", "wall", "wall", "wall"});
         Problem heat = poisson({{"wall", dirichlet("0")}});
         heat.equation.m1 = e("1");
         heat.initial.u = e("x + 2*y");
         const MeshedProblem p(wall, heat);
         check(p.initial_u().has_value() && !p.initial_u_t().has_value(),
               "only u");
         const std::vector<double> expected{0, 0.5, 1, 1, 1.5, 2, 2, 2.5, 3};
         check(*p.initial_u() == expected, "x + 2 y at the nodes");
         Problem wave = heat;
         wave.equation.m2 = e("1");
         wave.initial.u_t = e("k");
         wave.parameters = {{"k", 5.0}};
         const MeshedProblem q(wall, wave);
         check(*q.initial_u_t() == std::vector<double>(9, 5.0), "u_t = k");
       }},
      {"the summaries read the compiled facts",
       [] {
         Problem problem = poisson({{"bottom", dirichlet("0")},
                                    {"right", dirichlet("t")},
                                    {"top", neumann()},
                                    {"left", neumann()}});
         const MeshedProblem p(square(), problem);
         check(rhs_depends_on_t(p) && rhs_depends_on_t(problem),
               "g makes the right-hand side depend on t");
         check(!lhs_depends_on_t(p) && time_order(p) == 0, "left-hand side");
         problem.boundary_conditions["left"] = {.beta_n = e("1"),
                                                .gamma = e("1 + t")};
         problem.initial.u = e("0");
         const MeshedProblem q(square(), problem);
         check(time_order(q) == 1 && lhs_depends_on_t(q),
               "a dynamic condition with gamma(t)");
         check(time_order(q) == time_order(problem) &&
                   lhs_depends_on_t(q) == lhs_depends_on_t(problem),
               "the same as on the problem");
       }},
      {"one region on several sides has no corner",
       [] {
         const MeshedProblem p(
             grid({"wall", "wall", "lid", "wall"}),
             poisson({{"wall", dirichlet("0")}, {"lid", dirichlet("1")}}));
         check(metric(p, "corners") == 2, "two region changes");
         check(metric(p, "averaged_corners") == 2, "both averaged");
         check(p.report().messages.size() == 2, "both differ");
         check(p.dirichlet_value(index_of(p, 7), 0.0) == 1.0, "lid");
         check(p.dirichlet_value(index_of(p, 8), 0.0) == 0.5, "corner");
         check(p.mesh()->region_names.size() == 2, "mesh kept");
         check(p.problem().boundary_conditions.size() == 2, "problem kept");
       }},
  });
}

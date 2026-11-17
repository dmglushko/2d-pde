// Assembly with several threads (docs/decisions.md, "Parallel assembly and
// iterative solvers are built now"): the result is bit for bit the same
// for every thread count, also with `previous`, and an error is the same
// exception with the same message. The tests use at most 4 threads, apart
// from one assembly with 0 (all hardware threads).

#include "assembly_support.hpp"
#include "support/l_shape_mesh.hpp"

#include "pde2d/fem/assembler.hpp"

#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

using namespace pde2d::fem;
using pde2d::test::check;
using pde2d::test::expect_throws;
using pde2d::test::same;

namespace {

Expression e(const char *text) { return Expression(text); }

std::shared_ptr<const pde2d::mesh::Mesh> l_shape() {
  static const auto mesh = pde2d::test::refined_l_shape(0.04);
  return mesh;
}

// Every term at once: an anisotropic A, convection, reaction, both masses
// and the load, all partly in t; Dirichlet (moving), Robin with a
// tangential part, a dynamic condition and an oblique Neumann condition.
// Time order 2.
Problem every_term() {
  return {.equation = {.a_xx = e("2 + t"),
                       .a_xy = e("0.5"),
                       .a_yy = e("1 + x^2"),
                       .b_x = e("1"),
                       .b_y = e("-y*t"),
                       .c = e("1 + sin(t)"),
                       .m1 = e("1 + x"),
                       .m2 = e("1 + t^2"),
                       .f = e("x*y + t")},
          .boundary_conditions =
              {{"bottom", {.alpha = e("1"), .g = e("exp(t)*(1 + x)")}},
               {"right",
                {.alpha = e("1"),
                 .beta_n = e("1"),
                 .beta_tau = e("0.5"),
                 .g = e("t*y")}},
               {"inner", {.beta_n = e("1"), .gamma = e("1 + t"), .g = e("x")}},
               {"top", {.beta_n = e("1"), .beta_x = e("0.3"), .g = e("cos(t)")}},
               {"left", {.alpha = e("1"), .g = e("y*t")}}},
          .initial = {.u = e("0"), .u_t = e("0")}};
}

std::shared_ptr<const MeshedProblem> meshed(const Problem &problem) {
  return std::make_shared<const MeshedProblem>(l_shape(), problem);
}

AssembledProblem assemble(std::size_t threads,
                          const std::shared_ptr<const MeshedProblem> &problem,
                          double t, const AssembledProblem *previous = nullptr,
                          int max_time_order = 2) {
  return Assembler({.threads = threads})
      .assemble(problem, t, previous, max_time_order);
}

double metric(const AssembledProblem &a, const std::string &name) {
  return a.report().metrics.at(name);
}

// The message of the std::invalid_argument that assembling with `threads`
// throws.
std::string error_of(std::size_t threads,
                     const std::shared_ptr<const MeshedProblem> &problem) {
  try {
    (void)assemble(threads, problem, 0.5);
  } catch (const std::invalid_argument &error) {
    return error.what();
  }
  throw pde2d::test::Failure("assembly did not throw");
}

} // namespace

int main() {
  return pde2d::test::run({
      {"the mesh has many colours",
       [] {
         const auto a = assemble(1, meshed(every_term()), 0.5);
         check(metric(a, "triangle_colours") >= 20,
               "longest-edge bisection gives nodes with many triangles");
         check(metric(a, "edge_colours") >= 2, "edge colours");
         check(!a.mass().always_zero && !a.damping().always_zero &&
                   !a.stiffness().always_symmetric &&
                   a.stiffness().depends_on_t && a.load_depends_on_t() &&
                   !a.dirichlet_accelerations().empty(),
               "every term is there");
       }},
      {"every thread count gives the same bits",
       [] {
         const auto problem = meshed(every_term());
         for (int order = 0; order <= 2; ++order) {
           const auto serial = assemble(1, problem, 0.7, nullptr, order);
           check(metric(serial, "threads") == 1, "1 thread reported");
           for (const std::size_t threads : {2, 4}) {
             const auto parallel =
                 assemble(threads, problem, 0.7, nullptr, order);
             check(same(parallel, serial),
                   std::to_string(threads) + " threads, max_time_order " +
                       std::to_string(order) + ": the same bits");
             check(metric(parallel, "threads") ==
                       static_cast<double>(threads),
                   "thread count reported");
           }
         }
         // 0: all hardware threads (at least one).
         const auto all = assemble(0, problem, 0.7);
         check(same(all, assemble(1, problem, 0.7)), "0 threads: same bits");
         check(metric(all, "threads") >= 1, "0 resolved to a count");
       }},
      {"previous is reused the same way with threads",
       [] {
         const auto problem = meshed(every_term());
         for (const std::size_t threads : {1, 2, 4}) {
           const std::string what = std::to_string(threads) + " threads";
           const auto first = assemble(threads, problem, 0.3);
           const auto again = assemble(threads, problem, 1.1, &first);
           check(metric(again, "used_previous") == 1, what + ": reused");
           check(same(again, assemble(1, problem, 1.1)),
                 what + ": previous equals a fresh serial assembly");
           // Across thread counts and levels: a result is reusable by an
           // assembler with any count.
           const auto low = assemble(1, problem, 0.3, nullptr, 0);
           check(same(assemble(threads, problem, -0.4, &low, 2),
                      assemble(1, problem, -0.4)),
                 what + ": from level 0 to 2");
         }
       }},
      {"a coefficient that is not finite: the same error for every count",
       [] {
         // log(x - 0.5) is not finite left of x = 0.5: many triangles in
         // many colours fail; the error is the first one in the serial
         // order.
         Problem triangle_error = every_term();
         triangle_error.equation.c = e("log(x - 0.5)");
         const auto on_triangles = meshed(triangle_error);
         const std::string serial = error_of(1, on_triangles);
         check(serial.find("is not finite") != std::string::npos,
               "names the problem: " + serial);
         for (const std::size_t threads : {2, 4})
           check(error_of(threads, on_triangles) == serial,
                 std::to_string(threads) + " threads: the same message");

         // On boundary edges: g of the Robin side is not finite below
         // y = 0.5.
         Problem edge_error = every_term();
         edge_error.boundary_conditions.at("right").g = e("log(y - 0.5)");
         const auto on_edges = meshed(edge_error);
         const std::string edge_serial = error_of(1, on_edges);
         check(edge_serial.find("is not finite") != std::string::npos,
               "names the problem: " + edge_serial);
         for (const std::size_t threads : {2, 4})
           check(error_of(threads, on_edges) == edge_serial,
                 std::to_string(threads) + " threads: the same edge message");
       }},
      {"the thread count is checked",
       [] {
         check(Assembler().options().threads == 1, "default 1");
         check(Assembler({.threads = 1024}).options().threads == 1024,
               "1024 is accepted");
         expect_throws<std::invalid_argument>(
             [] { (void)Assembler({.threads = 1025}); },
             "assembler: threads must be at most 1024");
         // A negative count converted to an unsigned one.
         expect_throws<std::invalid_argument>(
             [] {
               (void)Assembler(
                   {.threads = std::numeric_limits<std::size_t>::max()});
             },
             "threads must be at most 1024");
       }},
  });
}

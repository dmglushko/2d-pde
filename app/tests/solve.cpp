// The Solve step: the solve catalog matches the library, the "solve"
// section is read strictly with errors that name their path, and the
// solutions are right (problems whose exact solution P1 elements and the
// time method reproduce, and the wave example against its exact
// solution).

#include "catalog.hpp"
#include "session.hpp"

#include "support/check.hpp"

#include "pde2d/solve/bicgstab.hpp"
#include "pde2d/solve/conjugate_gradient.hpp"
#include "pde2d/solve/fixed_step.hpp"
#include "pde2d/solve/simplicial_ldlt.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numbers>
#include <string>
#include <vector>

#ifndef PDE2D_APP_EXAMPLES_DIR
#error "PDE2D_APP_EXAMPLES_DIR must name app/examples"
#endif

using namespace pde2d;
using app::Json;
using test::check;

namespace {

Json load_example(const std::string &name) {
  std::ifstream in(std::filesystem::path(PDE2D_APP_EXAMPLES_DIR) / name);
  return Json::parse(std::string(std::istreambuf_iterator<char>(in),
                                 std::istreambuf_iterator<char>()));
}

template <class List>
const app::OptionSpec &option(const List &components, const std::string &type,
                              const std::string &name) {
  for (const auto &component : components)
    if (component.type == type)
      for (const app::OptionSpec &spec : component.options)
        if (spec.name == name)
          return spec;
  throw std::logic_error("no option " + type + "." + name);
}

template <class F>
void expect_stage_error(F &&f, const std::string &stage,
                        const std::string &fragment) {
  try {
    f();
  } catch (const app::StageError &error) {
    check(error.stage() == stage,
          "stage " + error.stage() + " should be " + stage + " (" +
              error.what() + ")");
    check(std::string(error.what()).find(fragment) != std::string::npos,
          "message '" + std::string(error.what()) + "' should contain '" +
              fragment + "'");
    return;
  }
  throw test::Failure("expected a StageError of " + stage);
}

// The traits the catalog gives the component `type` of the slot list.
const Json &traits(const Json &components, const std::string &type) {
  for (const Json &component : components)
    if (component["type"] == type)
      return component["traits"];
  throw std::logic_error("no component " + type);
}

// The default case with every boundary condition Dirichlet u = g.
Json dirichlet_case(const std::string &g) {
  Json case_json = app::Session::default_case();
  for (auto &[region, condition] :
       case_json["problem"]["boundary_conditions"].items())
    condition = {{"alpha", "1"}, {"g", g}};
  return case_json;
}

// The largest |values[i] - exact(x_i, y_i)| over the mesh nodes.
template <class Exact>
double max_error(const Json &mesh_view, const Json &values, Exact exact) {
  const Json &nodes = mesh_view["nodes"];
  double error = 0.0;
  for (std::size_t i = 0; i < values.size(); ++i) {
    const double x = nodes[2 * i].get<double>();
    const double y = nodes[2 * i + 1].get<double>();
    error = std::max(error, std::abs(values[i].get<double>() - exact(x, y)));
  }
  return error;
}

Json time_section(double t0, double t1, int snapshots, double dt,
                  const std::string &method) {
  Json time = Json::object();
  time["t0"] = t0;
  time["t1"] = t1;
  time["snapshots"] = snapshots;
  time["method"] = {{"type", method}, {"options", Json::object()}};
  time["step"] = {{"type", "fixed_step"}, {"options", {{"dt", dt}}}};
  return time;
}

} // namespace

int main() {
  return test::run({
      {"every solve catalog type is the component's name()",
       [] {
         const app::SolveCatalog &catalog = app::solve_catalog();
         const auto check_names = [](const auto &components,
                                     const Json &options_json) {
           for (const auto &component : components) {
             const app::OptionReader options(options_json, "test",
                                             component.options);
             check(component.make(options)->name() == component.type,
                   "name of " + component.type);
           }
         };
         check_names(catalog.linear_solvers, Json(nullptr));
         check_names(catalog.time_methods, Json(nullptr));
         // FixedStepOptions{}.dt = 0 is refused, so give one.
         check_names(catalog.step_controllers, Json{{"dt", 0.1}});
       }},
      {"solve catalog defaults and traits are the library's",
       [] {
         const app::SolveCatalog &catalog = app::solve_catalog();
         check(option(catalog.linear_solvers, "bicgstab", "tolerance")
                       .default_value.get<double>() ==
                   solve::BiCgStabOptions{}.tolerance,
               "bicgstab tolerance");
         check(option(catalog.linear_solvers, "bicgstab", "max_iterations")
                       .default_value.get<std::size_t>() ==
                   solve::BiCgStabOptions{}.max_iterations,
               "bicgstab iterations");
         check(option(catalog.linear_solvers, "conjugate_gradient",
                      "tolerance")
                       .default_value.get<double>() ==
                   solve::ConjugateGradientOptions{}.tolerance,
               "cg tolerance");
         check(option(catalog.linear_solvers, "simplicial_ldlt",
                      "residual_tolerance")
                       .default_value.get<double>() ==
                   solve::SimplicialLdltOptions{}.residual_tolerance,
               "ldlt tolerance");
         check(option(catalog.step_controllers, "fixed_step", "dt")
                       .default_value.get<double>() ==
                   solve::FixedStepOptions{}.dt,
               "dt");

         const Json section = app::solve_catalog_json();
         const Json &solvers = section["linear_solver"]["components"];
         check(traits(solvers, "conjugate_gradient")["symmetric_matrix"] ==
                   true,
               "cg needs symmetry");
         check(traits(solvers, "sparse_lu")["symmetric_matrix"] == false,
               "lu does not");
         const Json &methods = section["modes"][1]["slots"][0]["components"];
         check(traits(methods, "implicit_euler")["time_orders"] ==
                   Json::array({0, 1, 2}),
               "euler orders");
         check(traits(methods, "implicit_midpoint")["time_orders"] ==
                   Json::array({1, 2}),
               "midpoint orders");
         check(traits(methods, "newmark")["time_orders"] == Json::array({2}),
               "newmark orders");
       }},
      {"the solve options given reach the components",
       [] {
         Json section = app::default_solve_section();
         section["mode"] = "time";
         section["linear_solver"] = {{"type", "bicgstab"},
                                     {"options", {{"tolerance", 1e-6}}}};
         section["time"]["step"]["options"]["dt"] = 0.25;
         section["time"]["t1"] = 3.0;
         section["time"]["snapshots"] = 6;
         const app::SolveSetup setup = app::build_solve_setup(section);
         const auto *bicgstab =
             dynamic_cast<const solve::BiCgStab *>(setup.linear_solver.get());
         check(bicgstab != nullptr && bicgstab->options().tolerance == 1e-6,
               "tolerance");
         check(bicgstab->options().max_iterations ==
                   solve::BiCgStabOptions{}.max_iterations,
               "the default for the rest");
         const auto *step = dynamic_cast<const solve::FixedStep *>(
             setup.step_controller.get());
         check(step != nullptr && step->options().dt == 0.25, "dt");
         check(setup.t1 == 3.0 && setup.snapshots == 6, "settings");
         check(setup.method->name() == "implicit_euler", "method");
       }},
      {"a static solve reproduces a linear solution",
       [] {
         Json case_json = dirichlet_case("x + 2*y");
         case_json["problem"]["equation"]["f"] = "0";
         case_json["solve"]["linear_solver"]["type"] = "conjugate_gradient";
         app::Session session;
         const Json mesh = session.mesh(case_json)["mesh"];
         const Json result = session.solve(case_json);
         check(result["mode"] == "static", "static");
         check(result["times"] == Json::array({0.0}), "one time");
         check(result["values"].size() == 1, "one solution");
         const double error =
             max_error(mesh, result["values"][0],
                       [](double x, double y) { return x + 2 * y; });
         check(error < 1e-8, "error " + std::to_string(error));
         check(result["reports"].back()["stage"] == "static_solver",
               "the solver's report");
       }},
      {"a time run keeps its snapshots and is exact for u = t + x",
       [] {
         // u_t - (u_xx + u_yy) = 1 with u = t + x: linear in x and in t,
         // which P1 elements and implicit Euler reproduce exactly.
         Json case_json = dirichlet_case("t + x");
         case_json["problem"]["equation"]["m1"] = "1";
         case_json["problem"]["equation"]["f"] = "1";
         case_json["problem"]["initial"] = {{"u", "x"}};
         case_json["solve"]["mode"] = "time";
         case_json["solve"]["time"] =
             time_section(0.0, 1.0, 4, 0.1, "implicit_euler");
         app::Session session;
         const Json mesh = session.mesh(case_json)["mesh"];
         const Json result = session.solve(case_json);
         check(result["mode"] == "time", "time");
         check(result["time_order"] == 1, "time order 1");
         const std::vector<double> expected{0.0, 0.25, 0.5, 0.75, 1.0};
         check(result["times"].size() == expected.size(), "five times");
         for (std::size_t k = 0; k < expected.size(); ++k) {
           const double t = result["times"][k].get<double>();
           check(std::abs(t - expected[k]) < 1e-12, "time " + std::to_string(t));
           const double error =
               max_error(mesh, result["values"][k],
                         [t](double x, double) { return t + x; });
           check(error < 1e-8, "error " + std::to_string(error) + " at t = " +
                                   std::to_string(t));
         }
         const Json metrics = result["reports"][0]["metrics"];
         check(metrics["steps"] == 12.0, "steps of 0.1 shortened to land on "
                                         "every quarter: " +
                                             metrics.dump());
       }},
      {"solve settings are read strictly, with their paths",
       [] {
         app::Session session;
         Json case_json = app::Session::default_case();
         case_json["solve"]["mode"] = "dynamic";
         expect_stage_error([&] { (void)session.solve(case_json); }, "solve",
                            "solve.mode");
         case_json = app::Session::default_case();
         case_json["solve"]["extra"] = 1;
         expect_stage_error([&] { (void)session.solve(case_json); }, "solve",
                            "extra");
         case_json = app::Session::default_case();
         case_json["solve"]["linear_solver"] = {
             {"type", "bicgstab"}, {"options", {{"tolerance", 2.0}}}};
         expect_stage_error([&] { (void)session.solve(case_json); }, "solve",
                            "solve.linear_solver.options");
         case_json = app::Session::default_case();
         case_json["solve"]["mode"] = "time";
         case_json["solve"]["time"]["snapshots"] = 0;
         expect_stage_error([&] { (void)session.solve(case_json); }, "solve",
                            "solve.time.snapshots");
         case_json["solve"]["time"]["snapshots"] = 10;
         case_json["solve"]["time"]["step"]["options"]["dt"] = 0.0;
         expect_stage_error([&] { (void)session.solve(case_json); }, "solve",
                            "solve.time.step.options");
         // A static solve does not read the time part.
         case_json["solve"]["mode"] = "static";
         check(session.solve(case_json)["mode"] == "static",
               "time part ignored");
       }},
      {"the library's refusals reach the page",
       [] {
         app::Session session;
         // The default problem has time order 0; Newmark needs 2.
         Json case_json = app::Session::default_case();
         case_json["solve"]["mode"] = "time";
         case_json["solve"]["time"]["method"]["type"] = "newmark";
         expect_stage_error([&] { (void)session.solve(case_json); }, "solve",
                            "");
         // t1 <= t0 is the library's to refuse.
         case_json["solve"]["time"]["method"]["type"] = "implicit_euler";
         case_json["solve"]["time"]["t1"] = 0.0;
         expect_stage_error([&] { (void)session.solve(case_json); }, "solve",
                            "");
       }},
      {"a case from before the solvers solves statically",
       [] {
         Json case_json = app::Session::default_case();
         case_json["solve"] = nullptr;
         app::Session session;
         check(session.solve(case_json)["mode"] == "static", "static");
       }},
      {"the solution exports as CSV",
       [] {
         Json case_json = app::Session::default_case();
         case_json["solve"]["mode"] = "time";
         case_json["solve"]["time"] = time_section(0.0, 1.0, 2, 0.5,
                                                   "implicit_euler");
         app::Session session;
         const std::size_t nodes =
             session.mesh(case_json)["stats"]["nodes"].get<std::size_t>();
         const app::ExportedFile file =
             session.export_file(case_json, "solution.csv");
         check(file.name == "solution.csv", "name");
         check(file.content.starts_with("x,y,u(t=0),u(t=0.5),u(t=1)\n"),
               "header: " + file.content.substr(0, 40));
         const auto lines = static_cast<std::size_t>(
             std::count(file.content.begin(), file.content.end(), '\n'));
         check(lines == nodes + 1, "one row per node");
       }},
      {"assembly builds only the time terms asked for",
       [] {
         Json case_json = load_example("heat_annulus.json");
         app::Session session;
         check(session.assemble(case_json)["assembly"]["time_order"] == 1,
               "order 1 with every term");
         case_json["assembly"]["max_time_order"] = 0;
         const Json assembly = session.assemble(case_json)["assembly"];
         check(assembly["time_order"] == 0, "order 0");
         check(assembly["time_order_any_t"] == 1, "the formulas say 1");
         check(assembly["operators"]["damping"]["always_zero"] == true,
               "B left out");
         check(assembly["operators"]["stiffness"]["always_symmetric"] == true,
               "A symmetric");
         case_json["assembly"]["max_time_order"] = 5;
         expect_stage_error([&] { (void)session.assemble(case_json); },
                            "assembly", "");
         case_json["assembly"]["max_time_order"] = 0.5;
         expect_stage_error([&] { (void)session.assemble(case_json); },
                            "assembly", "assembly.max_time_order");
       }},
      {"the wave example follows its exact solution",
       [] {
         const Json case_json = load_example("wave_square.json");
         app::Session session;
         const Json mesh = session.mesh(case_json)["mesh"];
         const Json result = session.solve(case_json);
         check(result["time_order"] == 2, "time order 2");
         const double pi = std::numbers::pi;
         for (std::size_t k = 0; k < result["times"].size(); k += 25) {
           const double t = result["times"][k].get<double>();
           const double error = max_error(
               mesh, result["values"][k], [t, pi](double x, double y) {
                 return std::cos(std::sqrt(2.0) * pi * t) *
                        std::sin(pi * (x + 0.5)) * std::sin(pi * (y + 0.5));
               });
           // The phase error of P1 elements and Newmark grows with t:
           // about 0.002 after one period, 0.01 after two.
           check(error < 0.03, "error " + std::to_string(error) + " at t = " +
                                   std::to_string(t));
         }
       }},
  });
}

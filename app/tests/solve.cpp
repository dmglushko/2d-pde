// The Solve step: the solve catalog matches the library, the "solve"
// section is read strictly with errors that name their path, and the
// solutions are right (problems whose exact solution P1 elements and the
// time method reproduce, and the wave example against its exact
// solution).

#include "catalog.hpp"
#include "session.hpp"

#include "app_support.hpp"

#include "support/check.hpp"

#include "pde2d/adapt/dorfler_marking.hpp"
#include "pde2d/adapt/fixed_fraction_marking.hpp"
#include "pde2d/adapt/kelly_estimator.hpp"
#include "pde2d/adapt/maximum_marking.hpp"
#include "pde2d/adapt/recovery_estimator.hpp"
#include "pde2d/adapt/residual_estimator.hpp"
#include "pde2d/solve/bicgstab.hpp"
#include "pde2d/solve/conjugate_gradient.hpp"
#include "pde2d/solve/fixed_step.hpp"
#include "pde2d/solve/simplicial_ldlt.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <sstream>
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
    check(error.stage() == stage, "stage " + error.stage() + " should be " +
                                      stage + " (" + error.what() + ")");
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
double max_error(const Json &mesh_view, const std::vector<double> &values,
                 Exact exact) {
  const Json &nodes = mesh_view["nodes"];
  double error = 0.0;
  for (std::size_t i = 0; i < values.size(); ++i) {
    const double x = nodes[2 * i].get<double>();
    const double y = nodes[2 * i + 1].get<double>();
    error = std::max(error, std::abs(values[i] - exact(x, y)));
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
         check_names(catalog.estimators, Json(nullptr));
         // EquidistributionMarkingOptions has no usable default tolerance.
         for (const auto &marking : catalog.markings) {
           const Json given = marking.type == "equidistribution"
                                  ? Json{{"tolerance", 0.5}}
                                  : Json(nullptr);
           const app::OptionReader options(given, "test", marking.options);
           check(marking.make(options)->name() == marking.type,
                 "name of " + marking.type);
         }
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
         check(option(catalog.linear_solvers, "conjugate_gradient", "tolerance")
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
         const adapt::ResidualEstimatorOptions residual{};
         check(
             option(catalog.estimators, "residual_estimator", "triangle_degree")
                         .default_value.get<int>() ==
                     residual.triangle_degree &&
                 option(catalog.estimators, "residual_estimator", "edge_degree")
                         .default_value.get<int>() == residual.edge_degree,
             "residual degrees");
         check(option(catalog.estimators, "kelly_estimator", "edge_degree")
                       .default_value.get<int>() ==
                   adapt::KellyEstimatorOptions{}.edge_degree,
               "kelly degree");
         const adapt::RecoveryEstimatorOptions recovery{};
         check(
             option(catalog.estimators, "recovery_estimator", "triangle_degree")
                         .default_value.get<int>() ==
                     recovery.triangle_degree &&
                 option(catalog.estimators, "recovery_estimator", "norm")
                         .default_value == "energy" &&
                 recovery.norm == adapt::RecoveryNorm::energy,
             "recovery");
         check(option(catalog.markings, "dorfler", "theta")
                       .default_value.get<double>() ==
                   adapt::DorflerMarkingOptions{}.theta,
               "dorfler theta");
         check(option(catalog.markings, "maximum", "theta")
                       .default_value.get<double>() ==
                   adapt::MaximumMarkingOptions{}.theta,
               "maximum theta");
         check(option(catalog.markings, "fixed_fraction", "fraction")
                       .default_value.get<double>() ==
                   adapt::FixedFractionMarkingOptions{}.fraction,
               "fraction");

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
         const Json mesh = app::test_support::mesh_view(session, case_json);
         const Json result = session.solve(case_json);
         check(result["mode"] == "static", "static");
         check(result["times"] == Json::array({0.0}), "one time");
         const auto values = session.solution_values(case_json);
         check(values.size() == 1, "one solution");
         const double error = max_error(
             mesh, values[0], [](double x, double y) { return x + 2 * y; });
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
         const Json mesh = app::test_support::mesh_view(session, case_json);
         const Json result = session.solve(case_json);
         check(result["mode"] == "time", "time");
         check(result["time_order"] == 1, "time order 1");
         const std::vector<double> expected{0.0, 0.25, 0.5, 0.75, 1.0};
         check(result["times"].size() == expected.size(), "five times");
         const auto values = session.solution_values(case_json);
         for (std::size_t k = 0; k < expected.size(); ++k) {
           const double t = result["times"][k].get<double>();
           check(std::abs(t - expected[k]) < 1e-12,
                 "time " + std::to_string(t));
           const double error = max_error(
               mesh, values[k], [t](double x, double) { return t + x; });
           check(error < 1e-8, "error " + std::to_string(error) +
                                   " at t = " + std::to_string(t));
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
         // A static solve does not build the time part: a value only the
         // library refuses (dt = 0) stays allowed.
         case_json["solve"]["mode"] = "static";
         check(session.solve(case_json)["mode"] == "static",
               "time part not built");
       }},
      {"the inactive solve part is checked for keys and types",
       [] {
         app::Session session;
         const auto refused = [&](const Json &case_json,
                                  const std::string &path) {
           try {
             (void)session.solve(case_json);
           } catch (const app::StageError &error) {
             check(error.path() == path, error.path() + " should be " + path);
             return;
           }
           throw test::Failure("expected a refusal at " + path);
         };
         Json case_json = app::Session::default_case(); // static
         case_json["solve"]["time"]["snapshots"] = "ten";
         refused(case_json, "solve.time.snapshots");
         case_json = app::Session::default_case();
         case_json["solve"]["time"]["typo"] = 1;
         refused(case_json, "solve.time");
         case_json = app::Session::default_case();
         case_json["solve"]["time"]["method"]["type"] = "runge_kutta";
         refused(case_json, "solve.time.method.type");
         case_json = app::Session::default_case();
         case_json["solve"]["time"]["step"]["options"]["dt"] = "small";
         refused(case_json, "solve.time.step.options.dt");
         case_json = app::Session::default_case();
         case_json["solve"]["time"]["step"]["options"]["size"] = 1;
         refused(case_json, "solve.time.step.options");
         // In time, the static part is checked the same way.
         case_json = app::Session::default_case();
         case_json["solve"]["mode"] = "time";
         case_json["solve"]["static"]["t"] = "now";
         refused(case_json, "solve.static.t");
         // A missing or null inactive part is fine.
         case_json = app::Session::default_case();
         case_json["solve"]["time"] = nullptr;
         check(session.solve(case_json)["state"] == "done", "no time part");
       }},
      {"the adaptive mode builds the library's components",
       [] {
         Json section = app::default_solve_section();
         check(section["adaptive"]["refiner"]["type"] ==
                   "newest_vertex_bisection",
               "NVB by default");
         section["mode"] = "adaptive";
         section["adaptive"]["marking"] = {{"type", "fixed_fraction"},
                                           {"options", {{"fraction", 0.2}}}};
         section["adaptive"]["estimator"] = {
             {"type", "recovery_estimator"},
             {"options", {{"norm", "gradient"}}}};
         section["adaptive"]["tolerance"] = {{"absolute", 0.01},
                                             {"relative", nullptr}};
         section["adaptive"]["max_cycles"] = 4;
         const app::SolveSetup setup = app::build_solve_setup(section);
         check(setup.mode == app::SolveMode::adaptive, "mode");
         const auto *marking =
             dynamic_cast<const adapt::FixedFractionMarking *>(
                 setup.marking.get());
         check(marking != nullptr && marking->options().fraction == 0.2,
               "marking");
         const auto *estimator = dynamic_cast<const adapt::RecoveryEstimator *>(
             setup.estimator.get());
         check(estimator != nullptr &&
                   estimator->options().norm == adapt::RecoveryNorm::gradient,
               "estimator");
         check(setup.refiner->name() == "newest_vertex_bisection", "refiner");
         check(setup.target.tolerance.absolute == 0.01 &&
                   !setup.target.tolerance.relative,
               "tolerance");
         check(setup.target.max_cycles == 4u &&
                   setup.target.max_nodes == 50000u,
               "budgets");
         // A missing slot is the catalog's default.
         section["adaptive"].erase("marking");
         check(app::build_solve_setup(section).marking->name() == "dorfler",
               "default marking");
         // Read strictly, with paths, also when another mode is chosen.
         section["adaptive"]["max_cycles"] = 1.5;
         test::expect_throws<std::invalid_argument>(
             [&] { (void)app::build_solve_setup(section); },
             "solve.adaptive.max_cycles");
         section["mode"] = "static";
         test::expect_throws<std::invalid_argument>(
             [&] { (void)app::build_solve_setup(section); },
             "solve.adaptive.max_cycles");
         section["adaptive"]["max_cycles"] = 4;
         section["adaptive"]["estimator"]["type"] = "guess";
         test::expect_throws<std::invalid_argument>(
             [&] { (void)app::build_solve_setup(section); },
             "solve.adaptive.estimator.type");
       }},
      {"an adaptive solve refines the L-shape at its corner and converges",
       [] {
         const Json case_json = load_example("l_shape_adaptive.json");
         app::Session session;
         const Json mesh = session.mesh(case_json);
         const Json run = session.solve(case_json);
         check(run["state"] == "done" && run["mode"] == "adaptive", "done");
         const Json &adaptive = run["adaptive"];
         check(adaptive["stop_reason"] == "tolerance_met",
               "stopped: " + adaptive["stop_reason"].dump());
         const Json &cycles = adaptive["cycles"];
         check(cycles.size() > 5, "several cycles");
         check(cycles[0]["nodes"] == mesh["mesh"]["nodes"],
               "the first cycle is the Mesh step's mesh");
         check(cycles.back()["estimate"].get<double>() <=
                   adaptive["tolerance"].get<double>(),
               "the last estimate meets the tolerance");
         for (std::size_t k = 1; k < cycles.size(); ++k)
           check(cycles[k]["nodes"].get<std::size_t>() >
                     cycles[k - 1]["nodes"].get<std::size_t>(),
                 "every cycle refines");
         check(cycles.back()["marked"].is_null() &&
                   cycles[0]["marked"].get<double>() > 0,
               "marked counts until the last cycle");
         // The run's mesh is the last one, under its own id.
         const Json &last = adaptive["mesh"];
         check(run["mesh_id"] == last["id"] &&
                   run["mesh_id"] != mesh["mesh"]["id"] &&
                   run["start_mesh_id"] == mesh["mesh"]["id"],
               "mesh ids");
         check(last["nodes"] == cycles.back()["nodes"] &&
                   run["values"]["nodes"] == last["nodes"],
               "values on the last mesh");
         Json request = Json::object();
         request["mesh"] = last["id"];
         check(session.mesh_arrays(request).size() ==
                   last["arrays"]["bytes"].get<std::size_t>(),
               "its arrays are served");
         const auto job = run["job"].get<std::uint64_t>();
         check(session.indicators_float32(job).size() ==
                   4 * last["triangles"].get<std::size_t>(),
               "one indicator per triangle");
         check(adaptive["mesh_stats"]["min_angle_degrees"].get<double>() > 20,
               "NVB keeps the angles");
         // Against the exact solution: an error per cycle, falling about
         // like N^-1 in L2 (the rate of a smooth solution on uniform
         // meshes; uniform refinement gives N^(-2/3) here).
         Json compare = Json::object();
         compare["job"] = job;
         compare["exact"] = case_json["view"]["exact_solution"];
         const Json summary = session.compare(compare);
         const Json &errors = summary["cycles"];
         check(errors.size() == cycles.size(), "an error per cycle");
         check(errors.back()["l2"] == summary["max_l2"],
               "the last cycle is the snapshot");
         const double n0 = errors[2]["nodes"].get<double>();
         const double n1 = errors.back()["nodes"].get<double>();
         const double rate = std::log(errors[2]["l2"].get<double>() /
                                      errors.back()["l2"].get<double>()) /
                             std::log(n1 / n0);
         check(rate > 0.85, "L2 rate " + std::to_string(rate));
         // The solution exports on the last mesh; the adapted mesh too.
         const std::string csv =
             session.export_file(case_json, "solution.csv").text();
         check(std::ranges::count(csv, '\n') >=
                   last["nodes"].get<std::size_t>(),
               "a CSV row per node of the last mesh");
         const std::string adapted =
             session.export_file(case_json, "solution-mesh.pde2d").text();
         check(adapted.find("NODES " + last["nodes"].dump()) !=
                   std::string::npos,
               "the adapted mesh");
       }},
      {"the library's refusal of an adaptive setting lands on its field",
       [] {
         Json case_json = load_example("l_shape_adaptive.json");
         app::Session session;
         case_json["solve"]["adaptive"]["max_cycles"] = 0;
         try {
           (void)session.solve(case_json);
           throw test::Failure("max_cycles 0 should be refused");
         } catch (const app::StageError &error) {
           check(error.path() == "solve.adaptive.max_cycles",
                 "path " + error.path());
         }
         case_json["solve"]["adaptive"]["max_cycles"] = 3;
         case_json["solve"]["adaptive"]["max_nodes"] = 5;
         try {
           (void)session.solve(case_json);
           throw test::Failure("a starting mesh above max_nodes");
         } catch (const app::StageError &error) {
           check(error.path() == "solve.adaptive.max_nodes",
                 "path " + error.path());
         }
         case_json["solve"]["adaptive"]["max_nodes"] = 100000;
         case_json["solve"]["adaptive"]["marking"] = {
             {"type", "dorfler"}, {"options", {{"theta", 2.0}}}};
         try {
           (void)session.solve(case_json);
           throw test::Failure("theta 2 should be refused");
         } catch (const app::StageError &error) {
           check(error.path() == "solve.adaptive.marking.options.theta",
                 "path " + error.path());
         }
         // Not an adaptive run: no indicators.
         const Json plain = session.solve(load_example("l_shape_corner.json"));
         expect_stage_error(
             [&] {
               (void)session.indicators_float32(
                   plain["job"].get<std::uint64_t>());
             },
             "solve", "no error indicators");
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
         case_json["solve"]["time"] =
             time_section(0.0, 1.0, 2, 0.5, "implicit_euler");
         app::Session session;
         const std::size_t nodes =
             session.mesh(case_json)["stats"]["nodes"].get<std::size_t>();
         // An export never solves: without a run of the case it refuses.
         expect_stage_error(
             [&] { (void)session.export_file(case_json, "solution.csv"); },
             "export", "no run of this case");
         (void)session.solve(case_json);
         const app::ExportedFile file =
             session.export_file(case_json, "solution.csv");
         check(file.name == "solution.csv", "name");
         check(static_cast<bool>(file.stream), "streamed");
         const std::string text = file.text();
         // The provenance in "#" lines, then the table.
         check(text.starts_with("# produced by pde2d_app "), "provenance");
         const std::size_t header = text.find("\nx,y,");
         check(header != std::string::npos &&
                   text.substr(header + 1)
                       .starts_with("x,y,u(t=0),u(t=0.5),u(t=1)\n"),
               "header");
         const auto lines = static_cast<std::size_t>(
             std::count(text.begin() + static_cast<std::ptrdiff_t>(header) + 1,
                        text.end(), '\n'));
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
      {"the solution's values come as float32 after the summary",
       [] {
         Json case_json = app::Session::default_case();
         case_json["solve"]["mode"] = "time";
         case_json["solve"]["time"] =
             time_section(0.0, 1.0, 3, 0.25, "implicit_euler");
         app::Session session;
         const std::size_t nodes =
             session.mesh(case_json)["stats"]["nodes"].get<std::size_t>();
         const Json summary = session.solve(case_json);
         const Json &layout = summary["values"];
         check(layout["format"] == "float32le" &&
                   layout["path"] == "/api/solve/values",
               "layout: " + layout.dump());
         check(layout["snapshots"] == 4 && layout["nodes"] == nodes,
               "four snapshots of every node");
         check(layout["bytes"] == 4 * 4 * nodes, "bytes");
         const auto exact = session.solution_values(case_json);
         const std::string bytes = session.solution_values_float32(
             summary["job"].get<std::uint64_t>(), 0, 4);
         check(bytes.size() == layout["bytes"].get<std::size_t>(), "size");
         double low = exact[0][0];
         double high = exact[0][0];
         for (std::size_t k = 0; k < exact.size(); ++k)
           for (std::size_t i = 0; i < nodes; ++i) {
             std::uint32_t bits = 0;
             for (std::size_t b = 0; b < 4; ++b)
               bits |= static_cast<std::uint32_t>(static_cast<unsigned char>(
                           bytes[4 * (k * nodes + i) + b]))
                       << (8 * b);
             const float value = std::bit_cast<float>(bits);
             check(value == static_cast<float>(exact[k][i]),
                   "value " + std::to_string(k) + ", " + std::to_string(i));
             low = std::min(low, exact[k][i]);
             high = std::max(high, exact[k][i]);
           }
         check(summary["min"].get<double>() == low &&
                   summary["max"].get<double>() == high,
               "min and max are the doubles' own");
       }},
      {"the assembled system exports in Matrix Market form",
       [] {
         const Json case_json = load_example("heat_annulus.json");
         app::Session session;
         const Json assembly = session.assemble(case_json);
         const std::size_t size =
             assembly["meshed_problem"]["free_nodes"].get<std::size_t>();
         for (const std::string kind : {"stiffness", "damping", "mass"}) {
           const app::ExportedFile file =
               session.export_file(case_json, kind + ".mtx");
           std::istringstream in(file.text());
           std::string line;
           std::getline(in, line);
           check(line == "%%MatrixMarket matrix coordinate real general",
                 kind + " header");
           while (in.peek() == '%')
             std::getline(in, line);
           std::size_t rows = 0, columns = 0, entries = 0;
           in >> rows >> columns >> entries;
           check(rows == size && columns == size, kind + " size");
           const Json &op = assembly["assembly"]["operators"][kind];
           check(entries == op["free_nonzeros"].get<std::size_t>(),
                 kind + " entries");
           std::size_t read = 0;
           std::size_t i = 0, j = 0;
           double value = 0.0;
           bool in_range = true;
           while (in >> i >> j >> value) {
             ++read;
             in_range = in_range && i >= 1 && i <= size && j >= 1 && j <= size;
           }
           check(read == entries && in_range, kind + " entries read");
         }
         const app::ExportedFile rhs =
             session.export_file(case_json, "rhs.mtx");
         std::istringstream in(rhs.text());
         std::string line;
         std::getline(in, line);
         check(line == "%%MatrixMarket matrix array real general",
               "rhs header");
         while (in.peek() == '%')
           std::getline(in, line);
         std::size_t rows = 0, columns = 0;
         in >> rows >> columns;
         check(rows == size && columns == 1, "rhs size");
         std::size_t read = 0;
         double value = 0.0;
         while (in >> value)
           ++read;
         check(read == size, "one value per unknown");
       }},
      {"the wave example follows its exact solution",
       [] {
         const Json case_json = load_example("wave_square.json");
         app::Session session;
         const Json mesh = app::test_support::mesh_view(session, case_json);
         const Json result = session.solve(case_json);
         check(result["time_order"] == 2, "time order 2");
         const auto values = session.solution_values(case_json);
         const double pi = std::numbers::pi;
         for (std::size_t k = 0; k < result["times"].size(); k += 25) {
           const double t = result["times"][k].get<double>();
           const double error =
               max_error(mesh, values[k], [t, pi](double x, double y) {
                 return std::cos(std::sqrt(2.0) * pi * t) *
                        std::sin(pi * (x + 0.5)) * std::sin(pi * (y + 0.5));
               });
           // The phase error of P1 elements and Newmark grows with t:
           // about 0.002 after one period, 0.01 after two.
           check(error < 0.03, "error " + std::to_string(error) +
                                   " at t = " + std::to_string(t));
         }
       }},
  });
}

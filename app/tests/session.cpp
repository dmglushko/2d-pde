// The app's core without HTTP: the catalog matches the library, cases are
// read strictly with errors that name their path, and every stage runs.

#include "catalog.hpp"
#include "domain_input.hpp"
#include "problem_input.hpp"
#include "session.hpp"

#include "support/check.hpp"

#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/delaunay_flips.hpp"
#include "pde2d/mesh/ear_clipping.hpp"
#include "pde2d/mesh/laplacian_smoothing.hpp"
#include "pde2d/mesh/longest_edge_bisection.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>

#ifndef PDE2D_APP_EXAMPLES_DIR
#error "PDE2D_APP_EXAMPLES_DIR must name app/examples"
#endif

using namespace pde2d;
using app::Json;
using test::check;
using test::expect_throws;
using test::near;

namespace {

Json load_example(const std::string &name) {
  std::ifstream in(std::filesystem::path(PDE2D_APP_EXAMPLES_DIR) / name);
  return Json::parse(std::string(std::istreambuf_iterator<char>(in),
                                 std::istreambuf_iterator<char>()));
}

// The option `name` of the component `type` in a catalog list.
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

// Expects a StageError of `stage` whose message contains `fragment`.
template <class F>
void expect_stage_error(F &&f, const std::string &stage,
                        const std::string &fragment) {
  try {
    f();
  } catch (const app::StageError &error) {
    check(error.stage() == stage,
          "stage " + error.stage() + " should be " + stage);
    check(std::string(error.what()).find(fragment) != std::string::npos,
          "message '" + std::string(error.what()) + "' should contain '" +
              fragment + "'");
    return;
  }
  throw test::Failure("expected a StageError of " + stage);
}

} // namespace

int main() {
  return test::run({
      {"every catalog type is the component's name()",
       [] {
         const app::MeshCatalog &catalog = app::mesh_catalog();
         const auto check_names = [](const auto &components) {
           for (const auto &component : components) {
             const app::OptionReader options(Json(nullptr), "test",
                                             component.options);
             check(component.make(options)->name() == component.type,
                   "name of " + component.type);
           }
         };
         check_names(catalog.discretizers);
         check_names(catalog.triangulators);
         check_names(catalog.refiners);
         check_names(catalog.improvers);
       }},
      {"catalog defaults are the library's defaults",
       [] {
         const app::MeshCatalog &catalog = app::mesh_catalog();
         check(option(catalog.discretizers, "chord_length",
                      "corner_angle_degrees")
                       .default_value.get<double>() ==
                   mesh::ChordLengthOptions{}.corner_angle_degrees,
               "corner angle");
         check(option(catalog.refiners, "ruppert", "min_angle_degrees")
                       .default_value.get<double>() ==
                   mesh::RuppertOptions{}.min_angle_degrees,
               "ruppert angle");
         check(option(catalog.refiners, "ruppert", "max_points")
                       .default_value.get<std::size_t>() ==
                   mesh::RuppertOptions{}.max_points,
               "ruppert points");
         check(option(catalog.improvers, "laplacian_smoothing", "iterations")
                       .default_value.get<int>() ==
                   mesh::LaplacianSmoothingOptions{}.iterations,
               "smoothing iterations");
         check(option(catalog.improvers, "laplacian_smoothing", "use")
                       .default_value.get<std::string>() == "final_only",
               "smoothing use");
         check(option(catalog.improvers, "delaunay_flips", "use")
                       .default_value.get<std::string>() == "local_and_final",
               "flips use");
       }},
      {"the options given reach the component",
       [] {
         Json section = app::default_mesh_section();
         section["refiner"]["options"]["min_angle_degrees"] = 28.5;
         section["discretizer"]["options"]["max_curve_deviation"]["absolute"] =
             0.01;
         const app::MeshSetup setup = app::build_mesh_setup(section);
         const auto *ruppert =
             dynamic_cast<const mesh::Ruppert *>(setup.refiner.get());
         check(ruppert != nullptr, "a Ruppert refiner");
         check(ruppert->options().min_angle_degrees == 28.5, "angle");
         const auto *chords =
             dynamic_cast<const mesh::ChordLengthDiscretizer *>(
                 setup.discretizer.get());
         check(chords != nullptr &&
                   chords->options().max_curve_deviation.absolute == 0.01,
               "deviation");
         check(setup.spec.max_edge_length.relative == 0.05, "spec");
       }},
      {"mesh section errors name their path",
       [] {
         Json section = app::default_mesh_section();
         section["refiner"]["type"] = "magic";
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_mesh_setup(section); },
             "mesh.refiner.type: unknown component 'magic'");
         section = app::default_mesh_section();
         section["refiner"]["options"]["min_angel"] = 20;
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_mesh_setup(section); },
             "mesh.refiner.options: unknown option 'min_angel'");
         section = app::default_mesh_section();
         section["refiner"]["options"]["min_angle_degrees"] = "twenty";
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_mesh_setup(section); },
             "mesh.refiner.options.min_angle_degrees: expected a number");
         // A value the library refuses: its message, with the path.
         section = app::default_mesh_section();
         section["refiner"]["options"]["min_angle_degrees"] = 50;
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_mesh_setup(section); },
             "mesh.refiner.options: ");
         section = app::default_mesh_section();
         section["spec"]["max_edge_length"] = {{"absolute", nullptr},
                                               {"relative", nullptr}};
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_mesh_setup(section); }, "mesh.spec: ");
         section = app::default_mesh_section();
         section["improvers"] =
             Json::array({{{"type", "delaunay_flips"},
                           {"options", {{"use", "sometimes"}}}}});
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_mesh_setup(section); },
             "mesh.improvers[0].options.use: 'sometimes' is not one of");
       }},
      {"a missing refiner is no refiner",
       [] {
         Json section = app::default_mesh_section();
         section["refiner"] = nullptr;
         check(app::build_mesh_setup(section).refiner == nullptr, "no refiner");
       }},
      {"domains: shapes, region edits, svg and mesh files",
       [] {
         Json section = app::default_geometry_section();
         section["region_edits"] = Json::array(
             {{{"op", "merge"}, {"source", "top"}, {"target", "bottom"}},
              {{"op", "rename"}, {"from", "bottom"}, {"to", "walls"}}});
         const app::Domain domain = app::build_domain(section);
         const auto names = domain.geometry->region_names();
         check(names.size() == 3, "three regions after a merge");
         check(std::ranges::find(names, "walls") != names.end(), "renamed");

         section["region_edits"] = Json::array(
             {{{"op", "rename"}, {"from", "nowhere"}, {"to", "x"}}});
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_domain(section); },
             "geometry.region_edits[0]: ");

         Json svg = Json::object();
         svg["source"] = "svg";
         svg["svg"] = "<svg xmlns='http://www.w3.org/2000/svg'><rect/></svg>";
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_domain(svg); }, "geometry.svg: ");

         Json shape = app::default_geometry_section();
         shape["options"]["width"] = -1;
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_domain(shape); }, "geometry.options: ");

         // A mesh file round trip: export a meshed case, import the text.
         app::Session session;
         const Json case_json = app::Session::default_case();
         const auto file = session.export_file(case_json, "mesh.pde2d");
         Json imported = Json::object();
         imported["source"] = "mesh_file";
         imported["format"] = "pde2d";
         imported["text"] = file.content;
         const app::Domain from_file = app::build_domain(imported);
         check(from_file.mesh.has_value() && !from_file.geometry, "a mesh");
         check(from_file.mesh->region_names.size() == 4, "regions kept");
       }},
      {"problem errors are collected with their paths",
       [] {
         Json problem = app::default_problem_section();
         problem["equation"]["c"] = "sin(";
         problem["equation"]["f"] = "k*x";
         problem["boundary_conditions"]["left"] = {{"gamma", "1"}};
         problem["boundary_conditions"]["right"] = {{"alpha", "1"}, {"g", "0"}};
         const Json check_result = app::check_problem(problem);
         check(!check_result["ok"].get<bool>(), "not ok");
         std::set<std::string> paths;
         for (const Json &error : check_result["errors"])
           paths.insert(error["path"].get<std::string>());
         check(paths.contains("problem.equation.c"), "parse error");
         check(paths.contains("problem.equation.f"), "unknown parameter");
         check(paths.contains("problem.boundary_conditions.left"),
               "a boundary ODE is refused");
         check(check_result["conditions"]["right"]["kind"] == "dirichlet",
               "classified");
         check(check_result["time_order"] == 1, "gamma makes order 1");

         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_problem(problem); },
             "problem.equation.c: ");
         problem = app::default_problem_section();
         problem["equation"]["q"] = "1";
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_problem(problem); }, "unknown key 'q'");
       }},
      {"the default case runs to the end",
       [] {
         app::Session session;
         const Json summary = session.run(app::Session::default_case());
         check(summary["ok"].get<bool>(), "ok: " + summary.dump());
         check(summary["mesh"]["target_met"].get<bool>(), "target met");
       }},
      {"stages reuse what did not change",
       [] {
         app::Session session;
         Json case_json = app::Session::default_case();
         const Json first = session.assemble(case_json);
         check(first["assembly"]["report"]["metrics"]["used_previous"] == 0,
               "fresh");
         case_json["assembly"]["t"] = 1.0;
         const Json second = session.assemble(case_json);
         check(second["assembly"]["report"]["metrics"]["used_previous"] == 1,
               "a later t reuses the previous assembly");
         check(second["assembly"]["t"] == 1.0, "t");
       }},
      {"fields are evaluated at the nodes",
       [] {
         app::Session session;
         Json case_json = app::Session::default_case();
         case_json["problem"]["parameters"]["k"] = 2.0;
         Json request = Json::object();
         request["case"] = case_json;
         request["field"] = {
             {"kind", "expression"}, {"text", "k*x + t"}, {"t", 0.5}};
         const Json field = session.field(request);
         const Json mesh = session.mesh(case_json)["mesh"];
         check(field["values"].size() * 2 == mesh["nodes"].size(), "size");
         for (std::size_t i = 0; i < field["values"].size(); ++i)
           near(field["values"][i].get<double>(),
                2.0 * mesh["nodes"][2 * i].get<double>() + 0.5, 1e-14, "value");

         request["field"] = {{"kind", "dirichlet"}};
         const Json dirichlet = session.field(request);
         std::size_t boundary = 0;
         for (const Json &value : dirichlet["values"])
           if (!value.is_null()) {
             ++boundary;
             check(value.get<double>() == 0.0, "u = 0");
           }
         check(boundary == session.mesh(case_json)["stats"]["boundary_edges"]
                               .get<std::size_t>(),
               "one value per boundary node");

         request["field"] = {{"kind", "initial_u"}};
         expect_stage_error([&] { (void)session.field(request); }, "field",
                            "time order 0");
       }},
      {"errors carry their stage",
       [] {
         app::Session session;
         Json case_json = app::Session::default_case();
         case_json["problem"]["boundary_conditions"].erase("left");
         expect_stage_error([&] { (void)session.assemble(case_json); },
                            "meshed_problem", "left");
         case_json = app::Session::default_case();
         case_json["mesh"]["triangulator"]["type"] = "nope";
         expect_stage_error([&] { (void)session.mesh(case_json); }, "mesh",
                            "mesh.triangulator.type");
         case_json = app::Session::default_case();
         case_json["extra"] = 1;
         expect_stage_error([&] { (void)session.mesh(case_json); }, "case",
                            "unknown key 'extra'");
         const Json summary = session.run(case_json);
         check(!summary["ok"].get<bool>() &&
                   summary["error"]["stage"] == "case",
               "run reports the failing stage");
       }},
      {"an imported mesh is refined by the remesher",
       [] {
         app::Session session;
         Json case_json = app::Session::default_case();
         const auto file = session.export_file(case_json, "mesh.pde2d");
         case_json["geometry"] = {{"source", "mesh_file"},
                                  {"format", "pde2d"},
                                  {"text", file.content}};
         case_json["mesh"]["spec"]["max_edge_length"]["relative"] = 0.02;
         const Json result = session.mesh(case_json);
         check(result["target_met"].get<bool>(), "refined to the target");
         expect_stage_error(
             [&] { (void)session.export_file(case_json, "geometry.svg"); },
             "export", "no curve geometry");
       }},
      {"the examples run",
       [] {
         for (const char *name : {"poisson_rectangle.json", "heat_annulus.json",
                                  "plate_with_hole.json"}) {
           app::Session session;
           const Json summary = session.run(load_example(name));
           check(summary["ok"].get<bool>(),
                 std::string(name) + ": " + summary.dump());
         }
       }},
      {"the catalog lists every section",
       [] {
         const Json catalog = app::Session::catalog();
         check(catalog["sections"][0]["id"] == "mesh", "mesh section");
         check(catalog["sections"][1]["available"] == false,
               "solve is not available yet");
         check(catalog["problem"]["equation_fields"].size() == 9, "fields");
         check(catalog["default_case"] == app::Session::default_case(),
               "default case");
       }},
  });
}

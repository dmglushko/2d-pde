// The app's core without HTTP: the catalog matches the library, cases are
// read strictly with errors that name their path, and every stage runs.

#include "case_format.hpp"
#include "catalog.hpp"
#include "domain_input.hpp"
#include "problem_input.hpp"
#include "session.hpp"

#include "app_support.hpp"

#include "support/check.hpp"

#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/delaunay_flips.hpp"
#include "pde2d/mesh/ear_clipping.hpp"
#include "pde2d/mesh/laplacian_smoothing.hpp"
#include "pde2d/mesh/longest_edge_bisection.hpp"
#include "pde2d/mesh/newest_vertex_bisection.hpp"
#include "pde2d/mesh/red_green_blue_refinement.hpp"
#include "pde2d/mesh/ruppert.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <vector>

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

// A small JSON Schema validator: the keywords the case schema uses (type,
// const, enum, properties, additionalProperties, required, items, oneOf,
// allOf, if/then, minimum, maximum). Returns the first violation's place,
// or "" when `value` is valid.
std::string violation(const Json &schema, const Json &value,
                      const std::string &where);

bool has_type(const Json &value, const std::string &type) {
  if (type == "null")
    return value.is_null();
  if (type == "object")
    return value.is_object();
  if (type == "array")
    return value.is_array();
  if (type == "string")
    return value.is_string();
  if (type == "boolean")
    return value.is_boolean();
  if (type == "number")
    return value.is_number();
  if (type == "integer")
    return value.is_number_integer() ||
           (value.is_number_float() &&
            value.get<double>() == std::floor(value.get<double>()));
  throw std::logic_error("unknown type " + type);
}

std::string violation(const Json &schema, const Json &value,
                      const std::string &where) {
  if (schema.contains("type")) {
    bool any = false;
    const Json &type = schema["type"];
    for (const Json &name : type.is_array() ? type : Json::array({type}))
      any = any || has_type(value, name.get<std::string>());
    if (!any)
      return where + ": not " + type.dump();
  }
  if (schema.contains("const") && schema["const"] != value)
    return where + ": not " + schema["const"].dump();
  if (schema.contains("enum") &&
      std::ranges::find(schema["enum"], value) == schema["enum"].end())
    return where + ": not one of " + schema["enum"].dump();
  if (value.is_number()) {
    if (schema.contains("minimum") &&
        value.get<double>() < schema["minimum"].get<double>())
      return where + ": below the minimum";
    if (schema.contains("maximum") &&
        value.get<double>() > schema["maximum"].get<double>())
      return where + ": above the maximum";
  }
  if (value.is_object()) {
    for (const Json &key : schema.value("required", Json::array()))
      if (!value.contains(key.get<std::string>()))
        return where + ": misses " + key.dump();
    const Json properties = schema.value("properties", Json::object());
    for (const auto &[key, item] : value.items()) {
      const std::string place = where + "." + key;
      if (properties.contains(key)) {
        if (const std::string v = violation(properties[key], item, place);
            !v.empty())
          return v;
      } else if (schema.contains("additionalProperties")) {
        const Json &extra = schema["additionalProperties"];
        if (extra == false)
          return place + ": not allowed";
        if (extra.is_object())
          if (const std::string v = violation(extra, item, place); !v.empty())
            return v;
      }
    }
  }
  if (value.is_array() && schema.contains("items"))
    for (std::size_t i = 0; i < value.size(); ++i)
      if (const std::string v = violation(
              schema["items"], value[i], where + "[" + std::to_string(i) + "]");
          !v.empty())
        return v;
  if (schema.contains("oneOf")) {
    std::size_t matches = 0;
    std::string reasons;
    for (const Json &alternative : schema["oneOf"]) {
      const std::string v = violation(alternative, value, where);
      if (v.empty())
        ++matches;
      else
        reasons += " | " + v;
    }
    if (matches != 1)
      return where + ": matches " + std::to_string(matches) +
             " alternatives of oneOf" + reasons;
  }
  for (const Json &part : schema.value("allOf", Json::array()))
    if (const std::string v = violation(part, value, where); !v.empty())
      return v;
  if (schema.contains("if") && violation(schema["if"], value, where).empty() &&
      schema.contains("then"))
    if (const std::string v = violation(schema["then"], value, where);
        !v.empty())
      return v;
  return "";
}

// The fake migrations of the mechanism's test: version 1 had "solver" at
// the top, version 2 renamed it "solve_settings", version 3 moved it into
// "solve".
Json fake_v1_to_v2(Json case_json) {
  case_json["solve_settings"] = case_json["solver"];
  case_json.erase("solver");
  return case_json;
}
Json fake_v2_to_v3(Json case_json) {
  case_json["solve"] = case_json["solve_settings"];
  case_json.erase("solve_settings");
  return case_json;
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
         check(option(catalog.refiners, "newest_vertex_bisection", "max_points")
                       .default_value.get<std::size_t>() ==
                   mesh::NewestVertexBisectionOptions{}.max_points,
               "nvb points");
         check(option(catalog.refiners, "red_green_blue", "max_points")
                       .default_value.get<std::size_t>() ==
                   mesh::RedGreenBlueRefinementOptions{}.max_points,
               "rgb points");
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
             "mesh.refiner.options.min_angle_degrees: ");
         section = app::default_mesh_section();
         section["spec"]["max_edge_length"] = {{"absolute", nullptr},
                                               {"relative", nullptr}};
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_mesh_setup(section); },
             "mesh.spec.max_edge_length: ");
         section = app::default_mesh_section();
         section["improvers"] =
             Json::array({{{"type", "delaunay_flips"},
                           {"options", {{"use", "sometimes"}}}}});
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_mesh_setup(section); },
             "mesh.improvers[0].options.use: 'sometimes' is not one of");
       }},
      {"whole numbers out of range are refused, not narrowed",
       [] {
         Json section = app::default_mesh_section();
         section["improvers"] =
             Json::array({{{"type", "laplacian_smoothing"},
                           {"options", {{"iterations", -4294967295LL}}}}});
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_mesh_setup(section); },
             "mesh.improvers[0].options.iterations: is out of range");
         section["improvers"][0]["options"]["iterations"] =
             18446744073709551615ULL;
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_mesh_setup(section); }, "out of range");
         section = app::default_mesh_section();
         section["refiner"]["options"]["max_points"] = -1;
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_mesh_setup(section); },
             "must not be negative");

         Json problem = app::default_problem_section();
         problem["boundary_conditions"]["left"] = {
             {"alpha", "1"}, {"g", "0"}, {"priority", 1e100}};
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_problem(problem); },
             "problem.boundary_conditions.left.priority: expected a whole "
             "number that fits an int");
         problem["boundary_conditions"]["left"]["priority"] = -2147483648.0;
         check(app::build_problem(problem)
                       .boundary_conditions.at("left")
                       .priority == -2147483647 - 1,
               "the smallest int is accepted");
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
             [&] { (void)app::build_domain(shape); },
             "geometry.options.width: ");

         // A mesh file round trip: export a meshed case, import the text.
         app::Session session;
         const Json case_json = app::Session::default_case();
         const auto file = session.export_file(case_json, "mesh.pde2d");
         Json imported = Json::object();
         imported["source"] = "mesh_file";
         imported["format"] = "pde2d";
         imported["text"] = file.text();
         const app::Domain from_file = app::build_domain(imported);
         check(from_file.mesh.has_value() && !from_file.geometry, "a mesh");
         check(from_file.mesh->region_names.size() == 4, "regions kept");
       }},
      {"a refused option value names the option's path",
       [] {
         // The library's own messages, from each component's constructor.
         const auto message = [](const Json &section) {
           try {
             (void)app::build_mesh_setup(section);
           } catch (const std::invalid_argument &error) {
             return std::string(error.what());
           }
           throw test::Failure("expected an error");
         };
         const auto starts = [](const std::string &text,
                                const std::string &prefix) {
           check(text.starts_with(prefix),
                 "'" + text + "' should start with '" + prefix + "'");
         };
         Json section = app::default_mesh_section();
         section["refiner"]["options"]["min_angle_degrees"] = 50;
         starts(message(section),
                "mesh.refiner.options.min_angle_degrees: "
                "RuppertOptions::min_angle_degrees must be in (0, 34]");
         section = app::default_mesh_section();
         section["refiner"]["options"]["max_points"] = 0;
         starts(message(section), "mesh.refiner.options.max_points: ");
         section = app::default_mesh_section();
         section["discretizer"]["options"]["corner_angle_degrees"] = 200;
         starts(message(section),
                "mesh.discretizer.options.corner_angle_degrees: ");
         section = app::default_mesh_section();
         section["discretizer"]["options"]["max_curve_deviation"] = {
             {"absolute", -1.0}, {"relative", nullptr}};
         starts(message(section),
                "mesh.discretizer.options.max_curve_deviation.absolute: ");
         section = app::default_mesh_section();
         section["improvers"] =
             Json::array({{{"type", "laplacian_smoothing"},
                           {"options", {{"iterations", 0}}}}});
         starts(message(section), "mesh.improvers[0].options.iterations: ");
         section = app::default_mesh_section();
         section["spec"]["max_edge_length"] = {{"absolute", -2.0},
                                               {"relative", nullptr}};
         starts(message(section), "mesh.spec.max_edge_length.absolute: ");

         // The solve section: "fixed_step: dt must be ...",
         // "bicgstab: tolerance must be ...".
         const auto solve_message = [](const Json &solve_section) {
           try {
             (void)app::build_solve_setup(solve_section);
           } catch (const std::invalid_argument &error) {
             return std::string(error.what());
           }
           throw test::Failure("expected an error");
         };
         Json solve = app::default_solve_section();
         solve["mode"] = "time";
         solve["time"]["step"]["options"]["dt"] = 0.0;
         starts(solve_message(solve), "solve.time.step.options.dt: ");
         solve = app::default_solve_section();
         solve["linear_solver"] = {{"type", "bicgstab"},
                                   {"options", {{"tolerance", 2.0}}}};
         starts(solve_message(solve),
                "solve.linear_solver.options.tolerance: ");
         solve["linear_solver"] = {{"type", "conjugate_gradient"},
                                   {"options", {{"max_iterations", 0}}}};
         starts(solve_message(solve),
                "solve.linear_solver.options.max_iterations: ");
         solve["linear_solver"] = {{"type", "simplicial_ldlt"},
                                   {"options", {{"residual_tolerance", 0.0}}}};
         starts(solve_message(solve),
                "solve.linear_solver.options.residual_tolerance: ");
       }},
      {"an imported mesh takes rename and merge region edits",
       [] {
         app::Session session;
         const Json case_json = app::Session::default_case();
         const auto file = session.export_file(case_json, "mesh.pde2d");
         Json section = {{"source", "mesh_file"},
                         {"format", "pde2d"},
                         {"text", file.text()}};
         const app::Domain plain = app::build_domain(section);
         const std::vector<std::string> before = plain.mesh->region_names;
         check(before ==
                   std::vector<std::string>{"bottom", "right", "top", "left"},
               "the rectangle's regions");
         section["region_edits"] = Json::array(
             {{{"op", "merge"}, {"source", "right"}, {"target", "left"}},
              {{"op", "rename"}, {"from", "left"}, {"to", "sides"}}});
         const app::Domain edited = app::build_domain(section);
         const mesh::Mesh &m = *edited.mesh;
         check(m.region_names ==
                   std::vector<std::string>{"bottom", "top", "sides"},
               "names after the edits");
         // Every edge keeps its place, under its new tag.
         check(m.boundary_edges.size() == plain.mesh->boundary_edges.size(),
               "edges kept");
         for (std::size_t i = 0; i < m.boundary_edges.size(); ++i) {
           const std::string old_name =
               before[plain.mesh->boundary_edges[i].region];
           const std::string new_name =
               m.region_names[m.boundary_edges[i].region];
           const std::string expected =
               old_name == "right" || old_name == "left" ? "sides" : old_name;
           check(new_name == expected,
                 "edge " + std::to_string(i) + ": " + new_name);
         }
         mesh::validate(m);
         const Json view = app::domain_json(edited);
         check(view["regions"].size() == 3, "three regions in the view");

         // The whole case runs with the new names.
         Json edited_case = case_json;
         edited_case["geometry"] = section;
         Json conditions = Json::object();
         for (const std::string name : {"bottom", "top", "sides"})
           conditions[name] = app::default_condition();
         edited_case["problem"]["boundary_conditions"] = conditions;
         check(session.run(edited_case)["ok"].get<bool>(), "runs");

         // Errors name the edit.
         section["region_edits"] = Json::array(
             {{{"op", "rename"}, {"from", "bottom"}, {"to", "floor"}},
              {{"op", "rename"}, {"from", "nowhere"}, {"to", "x"}}});
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_domain(section); },
             "geometry.region_edits[1]: Unknown region 'nowhere'");
         section["region_edits"] = Json::array(
             {{{"op", "rename"}, {"from", "bottom"}, {"to", "top"}}});
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_domain(section); },
             "geometry.region_edits[0]: rename: region 'top' already exists");
         section["region_edits"] = Json::array(
             {{{"op", "merge"}, {"source", "top"}, {"target", "top"}}});
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_domain(section); },
             "geometry.region_edits[0]: merge: region 'top' cannot be merged");
         section["region_edits"] = Json::array({{{"op", "split"},
                                                 {"region", "top"},
                                                 {"fraction", 0.5},
                                                 {"first", "a"},
                                                 {"second", "b"}}});
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_domain(section); },
             "geometry.region_edits[0].op: an imported mesh has no curves");
         section["region_edits"] =
             Json::array({{{"op", "rename"}, {"from", "top"}, {"to", "a b"}}});
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_domain(section); },
             "geometry.region_edits[0]: rename: region name 'a b' is invalid");
       }},
      {"the case version is a whole number up to the current one",
       [] {
         Json case_json = app::Session::default_case();
         for (const Json &version : {Json(0), Json(-1), Json(0.5), Json(1.5)}) {
           case_json["version"] = version;
           expect_throws<std::invalid_argument>(
               [&] { app::check_case(case_json); },
               "version: expected a whole");
         }
         case_json["version"] = 2;
         expect_throws<std::invalid_argument>(
             [&] { app::check_case(case_json); }, "newer version");
         case_json["version"] = 1.0;
         app::check_case(case_json);
       }},
      {"a field request is read strictly",
       [] {
         app::Session session;
         Json request = Json::object();
         request["case"] = app::Session::default_case();
         request["field"] = {{"kind", "expression"}, {"text", "x"}};
         check(session.field(request)["values"].size() > 0, "a valid request");
         request["extra"] = 1;
         expect_stage_error([&] { (void)session.field(request); }, "field",
                            "request: unknown key 'extra'");
         request.erase("extra");
         request["field"]["colour"] = "red";
         expect_stage_error([&] { (void)session.field(request); }, "field",
                            "field: unknown key 'colour'");
       }},
      {"problem errors are collected with their paths",
       [] {
         Json problem = app::default_problem_section();
         problem["equation"] = {
             {"a_xx", "-1"}, {"a_yy", "-1"}, {"c", "sin("}, {"f", "k*x"}};
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
      {"equations and conditions as lines, with both forms in the check",
       [] {
         Json problem = app::default_problem_section();
         problem["parameters"] = {{"k", 2.0}};
         problem["equation"] = {{"text", "u_t - div(k*grad(u)) = 1"}};
         problem["boundary_conditions"]["left"] = {{"text", "u_n + u = 0"},
                                                   {"priority", 2}};
         problem["boundary_conditions"]["right"] = {{"alpha", "1"}, {"g", "y"}};
         problem["initial"] = {{"u", "0"}};
         const Json result = app::check_problem(problem);
         check(result["ok"].get<bool>(), "ok: " + result.dump());
         const Json &equation = result["forms"]["problem.equation"];
         check(equation["line"] == true &&
                   equation["text"] == "u_t - div(k*grad(u)) = 1",
               "the line as written");
         check(
             equation["coefficients"] ==
                 Json(
                     {{"a_xx", "-k"}, {"a_yy", "-k"}, {"m1", "1"}, {"f", "1"}}),
             "the coefficients read from it: " +
                 equation["coefficients"].dump());
         check(result["facts"]["problem.equation.m1"]["zero"] == false,
               "facts per coefficient");
         const Json &left = result["forms"]["problem.boundary_conditions.left"];
         check(left["coefficients"] == Json({{"alpha", "1"}, {"beta_n", "1"}}),
               "the condition's coefficients");
         check(result["conditions"]["left"]["kind"] == "natural", "natural");
         const Json &right =
             result["forms"]["problem.boundary_conditions.right"];
         check(right["line"] == false && right["text"] == "u = y",
               "coefficients printed as a line: " + right.dump());
         check(app::build_problem(problem)
                       .boundary_conditions.at("left")
                       .priority == 2,
               "the priority beside a line");

         // Errors of a line are at its text; a line with coefficients is
         // refused at the object.
         problem["equation"] = {{"text", "u*u_x = 1"}};
         problem["boundary_conditions"]["left"] = {{"text", "u = 0"},
                                                   {"g", "1"}};
         problem["boundary_conditions"]["right"] = {{"text", "  "}};
         const Json failed = app::check_problem(problem);
         std::set<std::string> paths;
         for (const Json &error : failed["errors"])
           paths.insert(error["path"].get<std::string>());
         check(paths.contains("problem.equation.text"), "the line's error");
         check(paths.contains("problem.boundary_conditions.left"),
               "both forms");
         check(paths.contains("problem.boundary_conditions.right.text"),
               "an empty line");
         check(!failed["forms"].contains("problem.equation"),
               "no forms for a line that failed");
         // A name that is not a parameter is found by compiling, at the
         // line.
         problem = app::default_problem_section();
         problem["equation"] = {{"text", "-lap(u) = q"}};
         const Json unknown = app::check_problem(problem);
         check(unknown["errors"].size() == 1 &&
                   unknown["errors"][0]["path"] == "problem.equation.text",
               "unknown name at the line: " + unknown.dump());
         expect_throws<std::invalid_argument>(
             [&] {
               problem["equation"] = {{"text", "u^2 = 1"}};
               (void)app::build_problem(problem);
             },
             "problem.equation.text: Cannot read the equation");
         // No form while a coefficient has an error: switching would drop
         // what is being typed.
         problem = app::default_problem_section();
         problem["equation"] = {{"a_xx", "-1"}, {"a_yy", "-1"}, {"f", "sin("}};
         check(
             !app::check_problem(problem)["forms"].contains("problem.equation"),
             "no form with an error in a coefficient");
         // A parameter named u cannot be printed in a line: the reason.
         problem["equation"] = {{"a_xx", "-1"}, {"a_yy", "-1"}, {"f", "u"}};
         problem["parameters"] = {{"u", 1.0}};
         const Json reserved = app::check_problem(problem);
         check(reserved["ok"].get<bool>(), "a parameter named u is valid");
         const Json &form = reserved["forms"]["problem.equation"];
         check(form["text"].is_null() &&
                   form["reason"].get<std::string>().find(
                       "uses the parameter 'u'") != std::string::npos,
               "no line, with the reason: " + form.dump());
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
         const Json mesh = app::test_support::mesh_view(session, case_json);
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

         // The right-hand side at the time asked for, not at assembly.t.
         case_json["problem"]["equation"] = {{"text", "-lap(u) = t"}};
         request["case"] = case_json;
         request["field"] = {{"kind", "right_hand_side"}, {"t", 2.0}};
         const Json at_two = session.field(request);
         check(at_two["t"] == 2.0, "assembled at t = 2");
         Json later = case_json;
         later["assembly"]["t"] = 2.0;
         request["case"] = later;
         request["field"] = {{"kind", "right_hand_side"}};
         check(session.field(request)["values"] == at_two["values"],
               "the same as assembling the case at t = 2");
         request["case"] = case_json;
         request["field"] = {{"kind", "right_hand_side"}};
         bool zero = true;
         for (const Json &value : session.field(request)["values"])
           zero = zero && (value.is_null() || value.get<double>() == 0.0);
         check(zero, "f = t is zero at the case's t = 0");
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
                                  {"text", file.text()}};
         case_json["mesh"]["spec"]["max_edge_length"]["relative"] = 0.02;
         const Json result = session.mesh(case_json);
         check(result["target_met"].get<bool>(), "refined to the target");
         // The remesher's plan is a note, not a warning.
         const Json &pipeline = result["reports"][0];
         check(pipeline["stage"] == "pipeline", pipeline.dump());
         check(
             pipeline["notes"].size() == 1 &&
                 pipeline["notes"][0].get<std::string>().starts_with("plan: "),
             "the plan is a note: " + pipeline.dump());
         for (const Json &report : result["reports"])
           for (const Json &message : report["messages"])
             check(!message.get<std::string>().starts_with("plan: "),
                   "no plan among the warnings: " + report.dump());
         expect_stage_error(
             [&] { (void)session.export_file(case_json, "geometry.svg"); },
             "export", "no curve geometry");
       }},
      {"the examples run",
       [] {
         for (const char *name : {"poisson_rectangle.json", "heat_annulus.json",
                                  "plate_with_hole.json", "wave_square.json"}) {
           app::Session session;
           const Json summary = session.run(load_example(name));
           check(summary["ok"].get<bool>(),
                 std::string(name) + ": " + summary.dump());
           check(summary["solve"]["times"].size() >= 1,
                 std::string(name) + " solved");
         }
       }},
      {"a case from an older version is migrated, step by step",
       [] {
         Json old = app::Session::default_case();
         old["version"] = 1;
         old["solver"] = old["solve"];
         old.erase("solve");
         constexpr app::CaseMigration fakes[] = {fake_v1_to_v2, fake_v2_to_v3};
         const auto migrated = app::migrate_case(old, 3, fakes);
         check(migrated.has_value(), "migrated");
         check((*migrated)["version"] == 3, "the version follows");
         check((*migrated)["solve"] == app::Session::default_case()["solve"] &&
                   !migrated->contains("solver") &&
                   !migrated->contains("solve_settings"),
               "both steps in order");
         Json middle = old;
         middle = fake_v1_to_v2(middle);
         middle["version"] = 2;
         check(app::migrate_case(middle, 3, fakes)->at("solve") ==
                   (*migrated)["solve"],
               "from the middle of the chain");
         Json current = app::Session::default_case();
         current["version"] = 3;
         check(!app::migrate_case(current, 3, fakes).has_value(),
               "a current case is not copied");
         current["version"] = 4;
         expect_throws<std::invalid_argument>(
             [&] { (void)app::migrate_case(current, 3, fakes); },
             "version: the case was written by a newer version of the app "
             "(case version 4; this app reads versions 1 to 3)");
         constexpr app::CaseMigration short_chain[] = {fake_v1_to_v2};
         expect_throws<std::logic_error>(
             [&] { (void)app::migrate_case(old, 3, short_chain); },
             "a migration is missing");
         // This app: no migration yet, one per version.
         check(app::case_migrations().size() ==
                   static_cast<std::size_t>(app::case_version - 1),
               "one migration per version");
       }},
      {"every example validates against the generated case schema",
       [] {
         const Json schema = app::Session::case_schema();
         check(schema["$schema"].get<std::string>().find("2020-12") !=
                   std::string::npos,
               "draft 2020-12");
         std::vector<std::filesystem::path> files;
         for (const auto &entry :
              std::filesystem::directory_iterator(PDE2D_APP_EXAMPLES_DIR))
           if (entry.path().extension() == ".json")
             files.push_back(entry.path());
         check(files.size() >= 6, "the examples");
         for (const auto &file : files) {
           const Json example = load_example(file.filename().string());
           const std::string v = violation(schema, example, "case");
           check(v.empty(), file.filename().string() + ": " + v);
         }
         check(violation(schema, app::Session::default_case(), "case").empty(),
               "the default case");
         // Drawings (lines and arcs, Beziers) and a mesh file's split at a
         // node.
         {
           app::Session session;
           for (const std::string shape :
                {"rectangle", "annulus", "rectangle_with_semicircle"}) {
             Json drawn = app::Session::default_case();
             drawn["geometry"] = {{"source", "shape"},
                                  {"shape", shape},
                                  {"options", Json::object()}};
             drawn["geometry"] = session.to_drawing(drawn)["geometry"];
             const std::string v = violation(schema, drawn, "case");
             check(v.empty(), shape + " drawn: " + v);
           }
           Json drawn = app::Session::default_case();
           drawn["geometry"] = session.to_drawing(drawn)["geometry"];
           drawn["geometry"]["loops"][0]["edges"][0] = {
               {"kind", "cubic"},
               {"region", "bottom"},
               {"control1", {-0.5, -1}},
               {"control2", {0.5, -1}}};
           drawn["geometry"]["loops"][0]["edges"][1] = {
               {"kind", "quadratic"}, {"region", "right"}, {"control", {2, 0}}};
           check(violation(schema, drawn, "case").empty(), "Beziers");
           drawn["geometry"]["loops"][0]["edges"][1]["kind"] = "spline";
           check(!violation(schema, drawn, "case").empty(), "an unknown kind");
           Json imported = app::Session::default_case();
           imported["geometry"] = {
               {"source", "mesh_file"},
               {"format", "gmsh"},
               {"text", "..."},
               {"region_edits", Json::array({{{"op", "split_at_node"},
                                              {"region", "bottom"},
                                              {"node", 3},
                                              {"first", "a"},
                                              {"second", "b"}}})}};
           check(violation(schema, imported, "case").empty(),
                 "a mesh file's split at a node");
           imported["geometry"]["region_edits"][0]["fraction"] = 0.5;
           check(!violation(schema, imported, "case").empty(),
                 "a split at a node has no fraction");
         }
         // What the readers refuse, the schema refuses too.
         Json bad = app::Session::default_case();
         bad["mesh"]["refiner"]["options"]["min_angel_degrees"] = 25;
         check(!violation(schema, bad, "case").empty(), "a misspelt option");
         bad = app::Session::default_case();
         bad["solve"]["time"]["snapshots"] = "ten";
         check(!violation(schema, bad, "case").empty(), "a wrong type");
         bad = app::Session::default_case();
         bad["geometry"]["shape"] = "hexagon";
         check(!violation(schema, bad, "case").empty(), "an unknown shape");
         bad = app::Session::default_case();
         bad["extra"] = 1;
         check(!violation(schema, bad, "case").empty(), "an unknown section");
         bad = app::Session::default_case();
         bad["solve"]["mode"] = "time";
         bad["solve"]["time"].erase("method");
         check(!violation(schema, bad, "case").empty(),
               "in time the method is needed");
       }},
      {"the schema's options are the catalog's, with their kinds",
       [] {
         const Json schema = app::Session::case_schema();
         // name -> kind of every option in a choice list of the schema.
         const auto options_of = [](const Json &choice) {
           std::map<std::string, std::map<std::string, std::string>> out;
           for (const Json &alternative : choice["oneOf"]) {
             if (!alternative.contains("properties"))
               continue; // null
             const std::string type =
                 alternative["properties"]["type"]["const"].get<std::string>();
             auto &options = out[type];
             for (const auto &[name, option] :
                  alternative["properties"]["options"]["properties"].items())
               options[name] = option["x-pde2d-kind"].get<std::string>();
           }
           return out;
         };
         const auto catalog_options = [](const auto &components) {
           std::map<std::string, std::map<std::string, std::string>> out;
           for (const auto &component : components) {
             auto &options = out[component.type];
             for (const app::OptionSpec &spec : component.options)
               options[spec.name] = app::option_kind_name(spec.kind);
           }
           return out;
         };
         const Json &mesh = schema["properties"]["mesh"]["properties"];
         const app::MeshCatalog &m = app::mesh_catalog();
         check(options_of(mesh["discretizer"]) ==
                   catalog_options(m.discretizers),
               "discretizers");
         check(options_of(mesh["triangulator"]) ==
                   catalog_options(m.triangulators),
               "triangulators");
         check(options_of(mesh["refiner"]) == catalog_options(m.refiners),
               "refiners");
         check(options_of(mesh["improvers"]["items"]) ==
                   catalog_options(m.improvers),
               "improvers");
         const Json &solve = schema["properties"]["solve"]["properties"];
         const app::SolveCatalog &s = app::solve_catalog();
         check(options_of(solve["linear_solver"]) ==
                   catalog_options(s.linear_solvers),
               "linear solvers");
         check(options_of(solve["time"]["properties"]["method"]) ==
                   catalog_options(s.time_methods),
               "time methods");
         check(options_of(solve["time"]["properties"]["step"]) ==
                   catalog_options(s.step_controllers),
               "step controllers");
         std::set<std::string> spec;
         for (const auto &[name, option] : mesh["spec"]["properties"].items())
           spec.insert(name);
         check(spec.size() == m.spec.size(), "the mesh spec");
       }},
      {"the catalog lists every section",
       [] {
         const Json catalog = app::Session::catalog();
         check(catalog["sections"][0]["id"] == "mesh", "mesh section");
         check(catalog["sections"][1]["id"] == "solve" &&
                   catalog["sections"][1]["available"] == true,
               "solve section");
         check(catalog["problem"]["equation_fields"].size() == 9, "fields");
         check(catalog["default_case"] == app::Session::default_case(),
               "default case");
       }},
  });
}

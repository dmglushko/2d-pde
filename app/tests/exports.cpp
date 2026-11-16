// Exports (docs/app.md, "Exports"): they never compute a run, they carry
// their provenance, the VTK file holds the mesh and every snapshot; errors
// carry their path as a field; the problem check gives no facts for a
// text that did not parse; --set edits a case by path.

#include "case_set.hpp"
#include "domain_input.hpp"
#include "session.hpp"

#include "app_support.hpp"

#include "support/check.hpp"

#include <chrono>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace pde2d;
using app::Json;
using test::check;
using test::expect_throws;

namespace {

Json time_case(int snapshots, double dt) {
  Json case_json = app::Session::default_case();
  case_json["problem"]["equation"] = {
      {"m1", "1"}, {"a_xx", "-1"}, {"a_yy", "-1"}, {"f", "1"}};
  case_json["problem"]["initial"] = {{"u", "0"}};
  case_json["solve"]["mode"] = "time";
  case_json["solve"]["time"] = {
      {"t0", 0.0},
      {"t1", 1.0},
      {"snapshots", snapshots},
      {"method", {{"type", "implicit_euler"}, {"options", Json::object()}}},
      {"step", {{"type", "fixed_step"}, {"options", {{"dt", dt}}}}}};
  return case_json;
}

template <class F> app::StageError stage_error(F &&f) {
  try {
    f();
  } catch (const app::StageError &error) {
    return error;
  }
  throw test::Failure("expected a StageError");
}

std::vector<std::string> lines_of(const std::string &text) {
  std::vector<std::string> out;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);)
    out.push_back(line);
  return out;
}

} // namespace

int main() {
  return test::run({
      {"a solution export never computes",
       [] {
         app::Session session({.background_runs = true});
         const Json case_json = time_case(400, 0.0025);
         // No run of the case: refused, nothing started.
         const app::StageError none = stage_error(
             [&] { (void)session.export_file(case_json, "solution.csv"); });
         check(none.kind() == "not_ready", none.kind());
         check(std::string(none.what()).find("solve it first") !=
                   std::string::npos,
               none.what());
         // A run that is not done (stopped at once): refused too.
         const Json started = session.solve(case_json);
         const auto job = started["job"].get<std::uint64_t>();
         const Json stopped = session.stop_solve(job);
         if (stopped["state"] == "stopped") {
           for (const char *kind : {"solution.csv", "solution.vtk"}) {
             const app::StageError error = stage_error(
                 [&] { (void)session.export_file(case_json, kind); });
             check(error.kind() == "not_ready", error.kind());
             check(std::string(error.what()).find("stopped") !=
                       std::string::npos,
                   error.what());
           }
           check(session.solve_progress(job)["state"] == "stopped",
                 "the export did not continue the run");
         }
         // Another client's run is not this client's.
         app::Session plain;
         const Json small = time_case(2, 0.25);
         (void)plain.solve(small, "a");
         check(stage_error([&] {
                 (void)plain.export_file(small, "solution.csv",
                                         {.client = "b", .case_name = ""});
               }).kind() == "not_ready",
               "per client");
         const auto file = plain.export_file(small, "solution.csv",
                                             {.client = "a", .case_name = ""});
         check(file.text().find("x,y,u(t=0),u(t=0.5),u(t=1)") !=
                   std::string::npos,
               "the client's run");
       }},
      {"exports carry their provenance",
       [] {
         app::Session session;
         const Json case_json = time_case(2, 0.25);
         (void)session.solve(case_json);
         const app::ExportContext context{.client = "local",
                                          .case_name = "study.json"};
         const std::string csv =
             session.export_file(case_json, "solution.csv", context).text();
         const auto csv_lines = lines_of(csv);
         check(csv_lines[0].starts_with("# produced by pde2d_app "), "app");
         check(csv.find("# case: study.json\n") != std::string::npos, "case");
         check(csv.find("# run: ") != std::string::npos, "run");
         check(csv.find("# settings: {\"geometry\"") != std::string::npos,
               "settings");

         const std::string mtx =
             session.export_file(case_json, "stiffness.mtx", context).text();
         const auto mtx_lines = lines_of(mtx);
         check(mtx_lines[0] == "%%MatrixMarket matrix coordinate real general",
               "the banner stays first");
         check(mtx.find("\n% produced by pde2d_app ") != std::string::npos,
               "% comments");
         check(mtx.find("\"assembly\"") != std::string::npos,
               "the assembly settings");

         const std::string svg =
             session.export_file(case_json, "mesh.svg", context).text();
         check(svg.find("<!--\n  produced by pde2d_app") != std::string::npos,
               "an XML comment");
         // .pde2d keeps the old format exactly: no comment.
         const std::string pde2d =
             session.export_file(case_json, "mesh.pde2d", context).text();
         check(pde2d.find("pde2d_app") == std::string::npos, "unchanged");

         const Json provenance = Json::parse(
             session.export_file(case_json, "provenance.json", context)
                 .text());
         check(provenance["format"] == "pde2d-app-provenance", "format");
         check(provenance["case"] == case_json, "the full case");
         check(provenance["case_name"] == "study.json", "the name");
         check(provenance["library"]["version"].is_string(), "versions");
         check(provenance["run"]["state"] == "done" &&
                   provenance["run"]["reports"].size() == 1,
               "the run's report");
         // Without a run of the case, provenance has none.
         Json other = case_json;
         other["solve"]["time"]["snapshots"] = 4;
         check(Json::parse(session.export_file(other, "provenance.json").text())
                   ["run"]
                       .is_null(),
               "no run");
       }},
      {"the VTK file has the mesh and one array per snapshot",
       [] {
         app::Session session;
         const Json case_json = time_case(2, 0.25);
         const auto values = session.solution_values(case_json);
         const Json mesh = app::test_support::mesh_view(session, case_json);
         const std::size_t nodes = mesh["nodes"].size() / 2;
         const std::size_t triangles = mesh["triangles"].size() / 3;
         const app::ExportedFile file =
             session.export_file(case_json, "solution.vtk");
         check(static_cast<bool>(file.stream), "streamed");
         const auto lines = lines_of(file.text());
         check(lines[0] == "# vtk DataFile Version 3.0", "version");
         check(lines[1].starts_with("pde2d_app ") && lines[1].size() < 256,
               "title");
         check(lines[2] == "ASCII" && lines[3] == "DATASET UNSTRUCTURED_GRID",
               "kind");
         check(lines[4] == "POINTS " + std::to_string(nodes) + " double",
               lines[4]);
         std::size_t at = 5;
         for (std::size_t i = 0; i < nodes; ++i, ++at) {
           std::istringstream in(lines[at]);
           double x = 0, y = 0, z = 1;
           in >> x >> y >> z;
           check(x == mesh["nodes"][2 * i].get<double>() &&
                     y == mesh["nodes"][2 * i + 1].get<double>() && z == 0,
                 "point " + std::to_string(i));
         }
         check(lines[at] == "CELLS " + std::to_string(triangles) + " " +
                                std::to_string(4 * triangles),
               lines[at]);
         at += 1 + triangles;
         check(lines[at] == "CELL_TYPES " + std::to_string(triangles),
               lines[at]);
         check(lines[at + 1] == "5", "triangles");
         at += 1 + triangles;
         check(lines[at] == "POINT_DATA " + std::to_string(nodes), lines[at]);
         ++at;
         for (std::size_t k = 0; k < values.size(); ++k) {
           check(lines[at].starts_with("SCALARS u(t=") &&
                     lines[at].ends_with(" double 1"),
                 lines[at]);
           check(lines[at + 1] == "LOOKUP_TABLE default", "table");
           at += 2;
           for (std::size_t i = 0; i < nodes; ++i, ++at)
             check(std::stod(lines[at]) == values[k][i],
                   "value " + std::to_string(i) + " exactly");
         }
         check(at == lines.size(), "nothing after the last array");
       }},
      {"errors carry the path of their field",
       [] {
         app::Session session;
         Json case_json = app::Session::default_case();
         case_json["mesh"]["refiner"]["options"]["min_angle_degrees"] = 50;
         app::StageError error =
             stage_error([&] { (void)session.mesh(case_json); });
         check(error.path() == "mesh.refiner.options.min_angle_degrees",
               error.path());
         check(std::string(error.what())
                   .starts_with("mesh.refiner.options.min_angle_degrees: "),
               "the message keeps its path in front");
         check(app::error_json(error)["path"] == error.path(), "in JSON");

         // A shape's option, like a component's.
         case_json = app::Session::default_case();
         case_json["geometry"]["options"]["height"] = -1;
         error = stage_error([&] { (void)session.geometry(case_json); });
         check(error.path() == "geometry.options.height", error.path());

         // A region edit.
         case_json = app::Session::default_case();
         case_json["geometry"]["region_edits"] = {
             {{"op", "rename"}, {"from", "nowhere"}, {"to", "x"}}};
         error = stage_error([&] { (void)session.geometry(case_json); });
         check(error.path() == "geometry.region_edits[0]", error.path());

         // The solve settings.
         case_json = app::Session::default_case();
         case_json["solve"]["mode"] = "time";
         case_json["solve"]["time"]["step"]["options"]["dt"] = -1;
         error = stage_error([&] { (void)session.solve(case_json); });
         check(error.path() == "solve.time.step.options.dt", error.path());

         // A coefficient.
         case_json = app::Session::default_case();
         case_json["problem"]["equation"]["c"] = "sin(";
         error = stage_error([&] { (void)session.assemble(case_json); });
         check(error.path() == "problem.equation.c", error.path());

         // A library message that names no field has no path.
         case_json = app::Session::default_case();
         case_json["problem"]["boundary_conditions"].erase("left");
         error = stage_error([&] { (void)session.assemble(case_json); });
         check(error.stage() == "meshed_problem" && error.path().empty(),
               error.stage() + " " + error.path());
       }},
      {"a text that did not parse has no facts",
       [] {
         app::Session session;
         Json case_json = app::Session::default_case();
         case_json["problem"]["equation"]["a_yy"] = "sin(";
         const Json check_result = session.check_problem(case_json);
         check(!check_result["ok"].get<bool>(), "an error");
         check(check_result["errors"][0]["path"] == "problem.equation.a_yy",
               "its path");
         check(!check_result["facts"].contains("problem.equation.a_yy"),
               "no facts for it");
         check(check_result["facts"]["problem.equation.a_xx"]["zero"] == false,
               "the others keep theirs");
       }},
      {"--set replaces a value of the case by path",
       [] {
         Json case_json = app::Session::default_case();
         app::apply_set_argument(case_json,
                                 "mesh.spec.max_edge_length.relative=0.02");
         check(case_json["mesh"]["spec"]["max_edge_length"]["relative"] ==
                   0.02,
               "a number");
         app::apply_set_argument(
             case_json,
             R"(mesh.improvers=[{"type": "delaunay_flips", "options": {}}])");
         app::apply_set_argument(case_json,
                                 "mesh.improvers[0].options.use=final_only");
         check(case_json["mesh"]["improvers"][0]["options"]["use"] ==
                   "final_only",
               "an index; text that is not JSON is a string");
         app::apply_set_argument(case_json, "solve.mode=\"time\"");
         check(case_json["solve"]["mode"] == "time", "a JSON string");
         app::apply_set_argument(case_json, "mesh.refiner=null");
         check(case_json["mesh"]["refiner"].is_null(), "null");
         app::apply_set_argument(case_json, "problem.parameters.k=2.5");
         check(case_json["problem"]["parameters"]["k"] == 2.5,
               "a new last key");
         expect_throws<app::PathError>(
             [&] { app::apply_set_argument(case_json, "mesh.nope.x=1"); },
             "mesh.nope.x: 'nope' does not exist in mesh");
         expect_throws<app::PathError>(
             [&] {
               app::apply_set_argument(case_json, "mesh.improvers[7].type=x");
             },
             "mesh.improvers has 1 items, so there is no [7]");
         expect_throws<app::PathError>(
             [&] { app::apply_set_argument(case_json, "solve.mode.x=1"); },
             "is not an object");
         expect_throws<std::invalid_argument>(
             [&] { app::apply_set_argument(case_json, "no_equals"); },
             "PATH=VALUE");
         // The edited case runs.
         Json run_case = app::Session::default_case();
         app::apply_set_argument(run_case, "geometry.options.width=3");
         app::Session session;
         check(session.run(run_case)["ok"].get<bool>(), "runs");
       }},
  });
}

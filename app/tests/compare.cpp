// Comparisons in double against exact solutions and reference runs,
// probes, runs per client, and the mesh's binary arrays (docs/app.md,
// "Comparisons", "Probes", "Runs").

#include "case_set.hpp"
#include "field_eval.hpp"
#include "presets.hpp"
#include "session.hpp"

#include "app_support.hpp"

#include "support/check.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

using namespace pde2d;
using app::Json;
using test::check;
using test::near;

namespace {

// u = x on the default rectangle: P1 reproduces it at the nodes.
Json laplace_case() {
  const Json base = app::Session::default_case();
  app::Session session;
  Json request = Json::object();
  request["case"] = base;
  request["preset"] = "laplace";
  Json case_json = session.apply_preset(request);
  case_json["solve"]["mode"] = "static";
  return case_json;
}

// u = x + t: m1 u_t - u_xx - u_yy = 1 with u = x + t on the boundary.
// Implicit Euler is exact for a solution linear in t.
Json moving_case(int snapshots) {
  Json case_json = app::Session::default_case();
  case_json["problem"]["equation"] = {
      {"m1", "1"}, {"a_xx", "-1"}, {"a_yy", "-1"}, {"f", "1"}};
  for (auto &[region, condition] :
       case_json["problem"]["boundary_conditions"].items())
    condition = {{"alpha", "1"}, {"g", "x + t"}};
  case_json["problem"]["initial"] = {{"u", "x"}};
  case_json["solve"]["mode"] = "time";
  case_json["solve"]["time"] = {
      {"t0", 0.0},
      {"t1", 1.0},
      {"snapshots", snapshots},
      {"method", {{"type", "implicit_euler"}, {"options", Json::object()}}},
      {"step", {{"type", "fixed_step"}, {"options", {{"dt", 0.05}}}}}};
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

Json job_request(const Json &progress) {
  Json out = Json::object();
  out["job"] = progress["job"];
  return out;
}

} // namespace

int main() {
  return test::run({
      {"a comparison with the exact solution is in double",
       [] {
         app::Session session;
         const Json case_json = laplace_case();
         const Json run = session.solve(case_json);
         Json request = job_request(run);
         request["exact"] = "x";
         const Json exact = session.compare(request);
         check(exact["snapshots"].size() == 1, "one snapshot");
         const double error = exact["max_abs"].get<double>();
         // The float32 copy of u reported about 3e-8 here.
         check(error < 1e-12, "u = x to rounding: " + std::to_string(error));

         // Against x + 1 the difference is 1 at every node: max 1, RMS 1,
         // lumped L2 the square root of the area (2 x 1).
         request["exact"] = "x + 1";
         const Json shifted = session.compare(request)["snapshots"][0];
         near(shifted["max_abs"].get<double>(), 1.0, 1e-12, "max");
         near(shifted["rms"].get<double>(), 1.0, 1e-12, "rms");
         near(shifted["l2"].get<double>(), std::sqrt(2.0), 1e-12, "l2");

         // The difference of one snapshot in float32: u - (x + 1) = -1.
         request["snapshot"] = 0;
         const std::string bytes = session.compare_difference(request);
         const Json mesh = app::test_support::mesh_view(session, case_json);
         check(bytes.size() == 4 * mesh["nodes"].size() / 2, "one per node");
         for (std::size_t i = 0; i < bytes.size(); i += 4)
           near(app::test_support::read_float32(bytes, i), -1.0, 1e-6,
                "difference");
       }},
      {"a comparison in time is per snapshot, at each snapshot's time",
       [] {
         app::Session session;
         const Json run = session.solve(moving_case(4));
         Json request = job_request(run);
         request["exact"] = "x + t";
         const Json result = session.compare(request);
         check(result["snapshots"].size() == 5, "every snapshot");
         for (std::size_t k = 0; k < 5; ++k) {
           const Json &item = result["snapshots"][k];
           near(item["t"].get<double>(), 0.25 * static_cast<double>(k), 1e-15,
                "time");
           check(item["max_abs"].get<double>() < 1e-11,
                 "exact in time: " + item["max_abs"].dump());
         }
         // Against x the error is t: the comparison uses each time.
         request["exact"] = "x";
         const Json late = session.compare(request);
         for (std::size_t k = 0; k < 5; ++k)
           near(late["snapshots"][k]["max_abs"].get<double>(),
                0.25 * static_cast<double>(k), 1e-11, "error t");
         near(late["max_abs"].get<double>(), 1.0, 1e-11, "largest");
       }},
      {"a comparison refuses a bad expression with its path",
       [] {
         app::Session session;
         const Json run = session.solve(laplace_case());
         Json request = job_request(run);
         request["exact"] = "sin(";
         const app::StageError error =
             stage_error([&] { (void)session.compare(request); });
         check(error.path() == "exact", "path " + error.path());
         request["exact"] = "q * x"; // q is not a parameter
         check(stage_error([&] { (void)session.compare(request); }).path() ==
                   "exact",
               "an unknown name");
         request.erase("exact");
         check(stage_error([&] { (void)session.compare(request); }).kind() ==
                   "input",
               "exact or reference");
       }},
      {"a reference run on the same mesh compares node by node",
       [] {
         app::Session session;
         const Json first = laplace_case();
         const Json run = session.solve(first);
         const Json kept = session.keep_reference(job_request(run));
         check(kept["reference"]["job"] == run["job"], "kept");

         // Another problem on the same geometry and mesh: u = 2x.
         Json second = first;
         for (auto &[region, condition] :
              second["problem"]["boundary_conditions"].items())
           if (condition.contains("g"))
             condition["g"] = "2*x";
         const Json other = session.solve(second);
         Json request = job_request(other);
         request["reference"] = true;
         const Json result = session.compare(request);
         // u_B - u_A = x at the nodes: its largest |value| is max |x|.
         const Json mesh = app::test_support::mesh_view(session, second);
         double largest = 0.0;
         for (std::size_t i = 0; i < mesh["nodes"].size(); i += 2)
           largest =
               std::max(largest, std::abs(mesh["nodes"][i].get<double>()));
         near(result["max_abs"].get<double>(), largest, 1e-12, "max |x|");
         check(result["against"]["reference"] == run["job"], "against");

         // The reference stays when the run is replaced; another mesh is
         // refused.
         Json finer = second;
         finer["mesh"]["spec"]["max_edge_length"]["relative"] = 0.03;
         const Json moved = session.solve(finer);
         Json refused = job_request(moved);
         refused["reference"] = true;
         const app::StageError error =
             stage_error([&] { (void)session.compare(refused); });
         check(std::string(error.what()).find("another mesh") !=
                   std::string::npos,
               error.what());

         // Without a reference, a clear message.
         (void)session.clear_reference();
         check(std::string(
                   stage_error([&] { (void)session.compare(refused); }).what())
                       .find("no reference") != std::string::npos,
               "no reference");
       }},
      {"a reference must be done; a static reference serves every time",
       [] {
         app::Session session({.background_runs = true});
         Json long_run = moving_case(400);
         long_run["solve"]["time"]["step"]["options"]["dt"] = 0.0025;
         const Json started = session.solve(long_run);
         (void)session.stop_solve(started["job"].get<std::uint64_t>());
         const Json stopped =
             session.solve_progress(started["job"].get<std::uint64_t>());
         if (stopped["state"] == "stopped") {
           const app::StageError error = stage_error(
               [&] { (void)session.keep_reference(job_request(started)); });
           check(error.kind() == "not_ready", error.kind());
         }
         // The steady state u = x + 1 against the run u = x + t: the
         // difference is t - 1 at every time.
         Json steady = moving_case(4);
         steady["problem"]["equation"] = {{"a_xx", "-1"}, {"a_yy", "-1"}};
         for (auto &[region, condition] :
              steady["problem"]["boundary_conditions"].items())
           condition["g"] = "x + 1";
         steady["solve"]["mode"] = "static";
         const Json reference = session.solve(steady);
         (void)session.keep_reference(job_request(reference));
         const Json run = session.solve(moving_case(4));
         Json request = job_request(run);
         request["reference"] = true;
         Json result;
         for (int i = 0; i < 400; ++i) {
           if (session.solve_progress(
                   run["job"].get<std::uint64_t>())["state"] == "done")
             break;
           std::this_thread::sleep_for(std::chrono::milliseconds(10));
         }
         result = session.compare(request);
         for (std::size_t k = 0; k < 5; ++k)
           near(result["snapshots"][k]["max_abs"].get<double>(),
                1.0 - 0.25 * static_cast<double>(k), 1e-11, "1 - t");
       }},
      {"probes give u(t) at points, in double",
       [] {
         app::Session session;
         const Json run = session.solve(moving_case(4));
         Json request = job_request(run);
         request["points"] = {{0.3, 0.2}, {-0.7, -0.3}, {5.0, 5.0}};
         const Json probes = session.probes(request);
         check(probes["times"].size() == 5, "every snapshot");
         for (std::size_t p = 0; p < 2; ++p) {
           const Json &probe = probes["probes"][p];
           check(probe["inside"].get<bool>(), "inside");
           for (std::size_t k = 0; k < 5; ++k)
             near(probe["values"][k].get<double>(),
                  probe["x"].get<double>() + 0.25 * static_cast<double>(k),
                  1e-11, "x + t at the probe");
         }
         check(!probes["probes"][2]["inside"].get<bool>(), "outside");
         check(probes["probes"][2]["values"].empty(), "no values outside");
         request["points"] = {{1}};
         check(stage_error([&] { (void)session.probes(request); }).path() ==
                   "points[0]",
               "a bad point names itself");
       }},
      {"the point locator finds every triangle's centroid",
       [] {
         app::Session session;
         const Json mesh = app::test_support::mesh_view(
             session, app::Session::default_case());
         mesh::Mesh plain;
         for (std::size_t i = 0; i < mesh["nodes"].size(); i += 2)
           plain.nodes.push_back({mesh["nodes"][i].get<double>(),
                                  mesh["nodes"][i + 1].get<double>()});
         for (std::size_t i = 0; i < mesh["triangles"].size(); i += 3)
           plain.triangles.push_back(
               {{mesh["triangles"][i].get<std::size_t>(),
                 mesh["triangles"][i + 1].get<std::size_t>(),
                 mesh["triangles"][i + 2].get<std::size_t>()}});
         const app::PointLocator locator(plain);
         for (std::size_t t = 0; t < plain.triangles.size(); ++t) {
           const auto &n = plain.triangles[t].nodes;
           const geometry::Point c{(plain.nodes[n[0]].x + plain.nodes[n[1]].x +
                                    plain.nodes[n[2]].x) /
                                       3,
                                   (plain.nodes[n[0]].y + plain.nodes[n[1]].y +
                                    plain.nodes[n[2]].y) /
                                       3};
           const auto found = locator.locate(c);
           check(found && found->triangle == t,
                 "triangle " + std::to_string(t));
           for (const double w : found->weights)
             near(w, 1.0 / 3.0, 1e-9, "centroid weights");
         }
         check(!locator.locate({-10, -10}), "far outside");
       }},
      {"runs belong to their client",
       [] {
         app::Session session({.max_clients = 2});
         const Json a = session.solve(laplace_case(), "a");
         Json other = laplace_case();
         other["mesh"]["spec"]["max_edge_length"]["relative"] = 0.1;
         const Json b = session.solve(other, "b");
         check(a["job"] != b["job"], "two runs");
         // Each client still has its own run.
         check(session.solve_progress(a["job"].get<std::uint64_t>(),
                                      "a")["state"] == "done",
               "a's run");
         check(session.solve_progress(b["job"].get<std::uint64_t>(),
                                      "b")["state"] == "done",
               "b's run");
         // Not the other's.
         (void)stage_error([&] {
           (void)session.solve_progress(a["job"].get<std::uint64_t>(), "b");
         });
         // A third client drops the one used least recently (a was used
         // before b's progress).
         (void)session.solve(laplace_case(), "c");
         check(session.clients() == 2, "bounded");
         (void)stage_error([&] {
           (void)session.solve_progress(a["job"].get<std::uint64_t>(), "a");
         });
         check(session.solve_progress(b["job"].get<std::uint64_t>(),
                                      "b")["state"] == "done",
               "b kept");
       }},
      {"a reloaded page finds its run of the case, and only that",
       [] {
         app::Session session;
         check(session.current_run(laplace_case(), "a")["run"].is_null(),
               "no run yet");
         check(session.clients() == 0, "a lookup makes no client");
         const Json a = session.solve(laplace_case(), "a");
         const Json found = session.current_run(laplace_case(), "a");
         check(found["run"]["job"] == a["job"], "the same run");
         check(found["run"]["state"] == "done", "its progress");
         check(session.current_run(laplace_case(), "b")["run"].is_null(),
               "not another client's");
         Json other = laplace_case();
         other["mesh"]["spec"]["max_edge_length"]["relative"] = 0.1;
         check(session.current_run(other, "a")["run"].is_null(),
               "not for other inputs");
         // The view section is not an input of the run.
         Json viewed = laplace_case();
         viewed["view"] = {{"exact_solution", "x"}};
         check(session.current_run(viewed, "a")["run"]["job"] == a["job"],
               "the view does not matter");
       }},
      {"the mesh's arrays round trip in binary",
       [] {
         app::Session session;
         const Json case_json = app::Session::default_case();
         const Json result = session.mesh(case_json);
         const Json &summary = result["mesh"];
         check(summary["nodes"] == result["stats"]["nodes"], "counts");
         const Json mesh = app::test_support::mesh_view(session, case_json);
         // The nodes in float64, the same numbers as the library's (the
         // expression x at the nodes, which the field call gives in JSON).
         Json request = Json::object();
         request["case"] = case_json;
         request["field"] = {{"kind", "expression"}, {"text", "x"}};
         const Json xs = session.field(request)["values"];
         for (std::size_t i = 0; i < xs.size(); ++i)
           check(mesh["nodes"][2 * i].get<double>() == xs[i].get<double>(),
                 "node " + std::to_string(i));
         const std::size_t nodes = xs.size();
         check(mesh["triangles"].size() ==
                   3 * result["stats"]["triangles"].get<std::size_t>(),
               "triangles");
         for (const Json &index : mesh["triangles"])
           check(index.get<std::size_t>() < nodes, "an index of a node");
         check(mesh["boundary_edges"].size() ==
                   3 * result["stats"]["boundary_edges"].get<std::size_t>(),
               "edges");
         for (std::size_t e = 2; e < mesh["boundary_edges"].size(); e += 3)
           check(mesh["boundary_edges"][e].get<std::size_t>() <
                     summary["region_names"].size(),
                 "a region tag");
         Json unknown = Json::object();
         unknown["mesh"] = 999;
         check(stage_error([&] {
                 (void)session.mesh_arrays(unknown);
               }).path() == "mesh",
               "an unknown mesh");
       }},
  });
}

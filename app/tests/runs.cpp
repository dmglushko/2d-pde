// Time runs in chunks (docs/app.md, "Runs"): a run split at snapshot
// times gives the same snapshots as one integrate call, the chunks'
// reports combine by declared rules, chunk sizes keep the fixed cost a
// small share, a background run can be followed, stopped and continued
// while progress, values, probes and status never wait for a chunk; the
// memory budget drops other clients' finished runs and refuses what cannot
// fit; a run that is gone says why; and every problem preset runs on a
// rectangle and an annulus.

#include "presets.hpp"
#include "session.hpp"

#include "app_support.hpp"

#include "support/check.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace pde2d;
using app::Json;
using test::check;
using test::expect_throws;

namespace {

Json time_section(double t1, int snapshots, double dt,
                  const std::string &method) {
  Json time = Json::object();
  time["t0"] = 0.0;
  time["t1"] = t1;
  time["snapshots"] = snapshots;
  time["method"] = {{"type", method}, {"options", Json::object()}};
  time["step"] = {{"type", "fixed_step"}, {"options", {{"dt", dt}}}};
  return time;
}

// Heat with a boundary value that changes in time; dt does not divide the
// snapshot interval, so steps are shortened to land on the snapshots.
Json heat_case(const std::string &method, int snapshots = 10) {
  Json case_json = app::Session::default_case();
  case_json["problem"]["equation"] = {
      {"m1", "1"}, {"a_xx", "-0.1"}, {"a_yy", "-0.1"}, {"f", "x*y"}};
  for (auto &[region, condition] :
       case_json["problem"]["boundary_conditions"].items())
    condition = {{"alpha", "1"}, {"g", "1 - exp(-3*t)"}};
  case_json["problem"]["initial"] = {{"u", "0"}};
  case_json["solve"]["mode"] = "time";
  case_json["solve"]["time"] = time_section(1.0, snapshots, 0.03, method);
  return case_json;
}

// A membrane from a bump at rest: time order 2.
Json wave_case(const std::string &method) {
  Json case_json = app::Session::default_case();
  case_json["problem"]["equation"] = {
      {"m2", "1"}, {"a_xx", "-1"}, {"a_yy", "-1"}};
  case_json["problem"]["initial"] = {
      {"u", "exp(-((x - 1)^2 + (y - 0.5)^2) / 0.04)"}, {"u_t", "0"}};
  case_json["solve"]["mode"] = "time";
  case_json["solve"]["time"] = time_section(1.0, 10, 0.02, method);
  return case_json;
}

using Values = std::vector<std::vector<double>>;

// One interval per chunk, and the whole run in one chunk.
Values chunked(const Json &case_json) {
  app::Session session({.chunk_seconds = 0});
  return session.solution_values(case_json);
}
Values single(const Json &case_json) {
  app::Session session({.first_chunk = 1000000});
  return session.solution_values(case_json);
}

void check_identical(const Values &a, const Values &b,
                     const std::string &what) {
  check(a.size() == b.size(), what + ": as many snapshots");
  for (std::size_t k = 0; k < a.size(); ++k)
    check(a[k] == b[k], what + ": snapshot " + std::to_string(k) +
                            " is the same bit for bit");
}

double largest(const Values &values) {
  double out = 0.0;
  for (const auto &snapshot : values)
    for (const double v : snapshot)
      out = std::max(out, std::abs(v));
  return out;
}

bool supports(const std::string &method, int order) {
  for (const Json &section : app::Session::catalog()["sections"])
    if (section["id"] == "solve")
      for (const Json &mode : section["modes"])
        for (const Json &slot : mode["slots"])
          for (const Json &component : slot["components"])
            if (component["type"] == method)
              for (const Json &o : component["traits"]["time_orders"])
                if (o == order)
                  return true;
  return false;
}

const Json &metrics(const Json &progress) {
  return progress["reports"][0]["metrics"];
}

// Polls until the run is no longer running.
Json wait(app::Session &session, std::uint64_t job) {
  const auto until =
      std::chrono::steady_clock::now() + std::chrono::seconds(60);
  while (std::chrono::steady_clock::now() < until) {
    Json progress = session.solve_progress(job);
    if (progress["state"] != "running")
      return progress;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  throw test::Failure("the run did not end");
}

// A heat run of two snapshot intervals, each about a second or more: A
// depends on t, so every step assembles and prepares again.
Json long_case() {
  Json case_json = app::Session::default_case();
  case_json["mesh"]["spec"]["max_edge_length"]["relative"] = 0.03;
  case_json["problem"]["equation"] = {
      {"m1", "1"}, {"a_xx", "-(1 + t)"}, {"a_yy", "-1"}, {"f", "1"}};
  case_json["problem"]["initial"] = {{"u", "0"}};
  case_json["solve"]["mode"] = "time";
  case_json["solve"]["time"] = time_section(2.0, 2, 0.002, "implicit_euler");
  return case_json;
}

// Seconds a call takes.
template <class F> double seconds_of(F &&f) {
  const auto start = std::chrono::steady_clock::now();
  f();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

// The chunks a run of `intervals` snapshot intervals takes when a chunk
// costs fixed + per_step * steps.
std::vector<app::ChunkCost> simulate(double fixed, double per_step,
                                     double steps_per_interval,
                                     std::size_t intervals) {
  const app::SessionOptions options;
  std::vector<app::ChunkCost> history;
  std::size_t done = 0;
  while (done < intervals) {
    const std::size_t n =
        std::min(app::plan_chunk(history, options), intervals - done);
    const double steps = static_cast<double>(n) * steps_per_interval;
    history.push_back(
        {.intervals = n, .steps = steps, .seconds = fixed + per_step * steps});
    done += n;
  }
  return history;
}

app::StageError stage_error_of(const std::function<void()> &f) {
  try {
    f();
  } catch (const app::StageError &error) {
    return error;
  }
  throw test::Failure("expected a StageError");
}

Json annulus_case() {
  Json case_json = app::Session::default_case();
  case_json["geometry"] = {
      {"source", "shape"},
      {"shape", "annulus"},
      {"options", {{"outer_radius", 1.0}, {"inner_radius", 0.4}}},
      {"region_edits", Json::array()}};
  return case_json;
}

} // namespace

int main() {
  return test::run({
      {"implicit Euler and the implicit midpoint rule: chunks give the same "
       "snapshots bit for bit",
       [] {
         for (const std::string method :
              {"implicit_euler", "implicit_midpoint"}) {
           check_identical(chunked(heat_case(method)),
                           single(heat_case(method)), method + ", order 1");
           if (supports(method, 2))
             check_identical(chunked(wave_case(method)),
                             single(wave_case(method)), method + ", order 2");
         }
       }},
      {"error-controlled steps: chunks continue the step anchor, so the "
       "snapshots are the same bit for bit",
       [] {
         for (const std::string method :
              {"implicit_euler", "implicit_midpoint"}) {
           Json case_json = heat_case(method);
           case_json["solve"]["time"]["step"] = {
               {"type", "adaptive_step"},
               {"options", {{"initial_dt", 0.01}, {"rtol", 1e-3}}}};
           check_identical(chunked(case_json), single(case_json),
                           method + " with adaptive steps");
         }
         Json case_json = wave_case("newmark");
         case_json["solve"]["time"]["step"] = {
             {"type", "adaptive_step"},
             {"options", {{"initial_dt", 0.01}, {"max_dt", 0.05}}}};
         check_identical(chunked(case_json), single(case_json),
                         "newmark with adaptive steps");
       }},
      {"Newmark: chunks continue from the carried acceleration, so the "
       "snapshots are the same bit for bit",
       [] {
         check_identical(chunked(wave_case("newmark")),
                         single(wave_case("newmark")), "newmark");
       }},
      {"the chunks' reports add up to the one call's",
       [] {
         const Json case_json = heat_case("implicit_euler");
         app::Session many({.chunk_seconds = 0});
         app::Session one({.first_chunk = 1000000});
         const Json a = many.solve(case_json);
         const Json b = one.solve(case_json);
         check(a["state"] == "done" && b["state"] == "done", "done");
         check(a["times"] == b["times"], "the same times");
         check(a["min"] == b["min"] && a["max"] == b["max"], "the same range");
         check(metrics(a)["chunks"] == 10 && metrics(b)["chunks"] == 1,
               "ten chunks and one");
         for (const char *name :
              {"steps", "rejected_steps", "linear_iterations", "snapshots",
               "min_dt", "max_dt", "max_linear_iterations", "t0", "t1"})
           check(metrics(a)[name] == metrics(b)[name],
                 std::string(name) + ": " + metrics(a)[name].dump() + " vs " +
                     metrics(b)[name].dump());
         check(a["t"] == 1.0 && a["snapshots_total"] == 11, "t reached");
       }},
      {"a background run can be followed, stopped and continued",
       [] {
         const Json case_json = heat_case("implicit_euler", 400);
         const Values expected = single(case_json);
         app::Session session({.background_runs = true, .chunk_seconds = 0});
         Json progress = session.solve(case_json);
         const auto job = progress["job"].get<std::uint64_t>();
         check(progress["values"]["snapshots"].get<std::size_t>() >= 2,
               "the first chunk is ready at once");
         progress = session.stop_solve(job);
         const std::size_t ready =
             progress["values"]["snapshots"].get<std::size_t>();
         check(progress["state"] == "stopped" || progress["state"] == "done",
               "stopped: " + progress["state"].dump());
         std::this_thread::sleep_for(std::chrono::milliseconds(50));
         if (progress["state"] == "stopped")
           check(session.solve_progress(job)["values"]["snapshots"] == ready,
                 "a stopped run does not go on");
         // The same inputs continue it.
         progress = session.solve(case_json);
         check(progress["job"] == job, "the same run");
         progress = wait(session, job);
         check(progress["state"] == "done", "done");
         const std::string bytes =
             session.solution_values_float32(job, 0, expected.size());
         check(bytes.size() == 4 * expected.size() * expected[0].size(),
               "every snapshot");
         check(session.solution_values(case_json) == expected,
               "continuing gives the same snapshots");
         expect_throws<app::StageError>(
             [&] { (void)session.solution_values_float32(job, 400, 5); },
             "not ready");
         // Other inputs replace the run.
         Json other = case_json;
         other["solve"]["time"]["t1"] = 0.5;
         const auto next = session.solve(other)["job"].get<std::uint64_t>();
         check(next != job, "a new run");
         expect_throws<app::StageError>(
             [&] { (void)session.solve_progress(job); },
             "was replaced by a newer solve");
         (void)wait(session, next);
       }},
      {"chunk sizes keep the fixed cost a small share, within the time cap",
       [] {
         const app::SessionOptions options;
         // A factorization of 1.7 s and 0.05 s a step (the 110k-unknown
         // heat run): the share is capped by the time a chunk may take,
         // max(max_chunk_seconds, 2 x fixed), so at most half.
         const auto slow = simulate(1.7, 0.05, 1.0, 1000);
         check(slow.size() < 45, std::to_string(slow.size()) + " chunks");
         for (std::size_t k = 3; k + 1 < slow.size(); ++k) {
           check(slow[k].seconds <= 3.4 * 1.001,
                 "chunk " + std::to_string(k) + " takes " +
                     std::to_string(slow[k].seconds) + " s");
           check(1.7 / slow[k].seconds <= 0.5 + 1e-9, "fixed share");
         }
         // A cheap fixed cost: chunks of about chunk_seconds, the fixed
         // cost at most max_fixed_share of them.
         const auto cheap = simulate(0.05, 0.002, 4.0, 2000);
         for (std::size_t k = 4; k + 1 < cheap.size(); ++k) {
           check(0.05 / cheap[k].seconds <= options.max_fixed_share + 1e-9,
                 "share of chunk " + std::to_string(k));
           check(cheap[k].seconds <= options.max_chunk_seconds,
                 "within the cap");
           check(cheap[k].seconds >= 0.5 * options.chunk_seconds,
                 "not shorter than needed");
         }
         // Expensive steps and no fixed cost: after one doubling (to tell
         // the costs apart) one interval per chunk, none over the cap.
         const auto costly = simulate(0.0, 0.8, 1.0, 20);
         for (std::size_t k = 0; k < costly.size(); ++k) {
           check(costly[k].seconds <= options.max_chunk_seconds, "capped");
           if (k >= 2)
             check(costly[k].intervals == 1, "one interval");
         }
         // Without a fit (equal step counts so far) the next chunk doubles.
         const std::vector<app::ChunkCost> one = {
             {.intervals = 1, .steps = 1, .seconds = 1.7}};
         check(app::plan_chunk(one, options) == 2, "doubles to learn");
         check(!app::fit_chunk_cost(one).fitted, "not fitted from one");
         const std::vector<app::ChunkCost> two = {
             {.intervals = 1, .steps = 10, .seconds = 1.1},
             {.intervals = 2, .steps = 20, .seconds = 1.2}};
         const app::ChunkCostModel model = app::fit_chunk_cost(two);
         check(model.fitted && std::abs(model.fixed - 1.0) < 1e-9 &&
                   std::abs(model.per_step - 0.01) < 1e-12,
               "fixed 1 s and 0.01 s a step");
         check(app::plan_chunk({}, {.chunk_seconds = 0}) == 1,
               "chunk_seconds 0: one interval");
       }},
      {"report metrics combine by their declared rules",
       [] {
         std::set<std::string> ruled;
         for (const app::MetricRule &rule : app::integrator_metric_rules())
           ruled.insert(std::string(rule.name));
         // Every metric the integrator reports has a rule, and only those.
         app::Session session({.chunk_seconds = 0});
         const Json progress = session.solve(heat_case("implicit_euler"));
         std::set<std::string> reported;
         for (const auto &[name, value] : metrics(progress).items())
           if (name != "chunks")
             reported.insert(name);
         check(ruled == reported, "a rule for every metric");
         const auto part = [](double steps, double min_dt, double max_dt,
                              double t1) {
           mesh::StageReport report;
           report.stage = "integrator";
           report.metrics = {{"t0", t1 - 1},      {"t1", t1},
                             {"steps", steps},    {"min_dt", min_dt},
                             {"max_dt", max_dt},  {"prepares", 1},
                             {"max_residual", t1}};
           return report;
         };
         // The worst solve: the largest backward error, the smallest pivot
         // ratio above 0 (0 is "not measured").
         const auto solves = [](mesh::StageReport report, double backward,
                                double pivot) {
           report.metrics["max_backward_error"] = backward;
           report.metrics["min_pivot_ratio"] = pivot;
           return report;
         };
         std::optional<mesh::StageReport> total;
         app::combine_report(total, solves(part(2, 0.1, 0.2, 1), 2e-16, 0));
         app::combine_report(total, part(0, 0, 0, 2)); // no steps
         app::combine_report(total, solves(part(3, 0.05, 0.3, 3), 1e-16, 1e-5));
         app::combine_report(total, solves(part(1, 0.1, 0.1, 4), 0, 1e-3));
         const auto &m = total->metrics;
         check(m.at("max_backward_error") == 2e-16 &&
                   m.at("min_pivot_ratio") == 1e-5,
               "the worst solve, 0 pivot ratios ignored");
         std::optional<mesh::StageReport> unmeasured;
         app::combine_report(unmeasured, solves(part(1, 1, 1, 1), 0, 0));
         app::combine_report(unmeasured, solves(part(1, 1, 1, 2), 0, 0));
         check(unmeasured->metrics.at("min_pivot_ratio") == 0,
               "no measured pivot ratio: 0");
         // A NaN backward error (a residual that is not finite) stays,
         // whether a later chunk or an earlier one had it.
         for (const bool nan_first : {false, true}) {
           const double nan = std::numeric_limits<double>::quiet_NaN();
           std::optional<mesh::StageReport> with_nan;
           app::combine_report(
               with_nan, solves(part(1, 1, 1, 1), nan_first ? nan : 1e-16, 0));
           app::combine_report(
               with_nan, solves(part(1, 1, 1, 2), nan_first ? 1e-16 : nan, 0));
           app::combine_report(with_nan, solves(part(1, 1, 1, 3), 2e-16, 0));
           check(std::isnan(with_nan->metrics.at("max_backward_error")),
                 nan_first ? "a NaN of the first chunk stays"
                           : "a NaN of a later chunk stays");
         }
         check(m.at("steps") == 6 && m.at("prepares") == 4, "sums");
         check(m.at("min_dt") == 0.05 && m.at("max_dt") == 0.3,
               "extremes over chunks with steps");
         check(m.at("t0") == 0 && m.at("t1") == 4, "first and last");
         check(m.at("max_residual") == 4, "largest");
         mesh::StageReport unknown = part(1, 1, 1, 5);
         unknown.metrics["something_new"] = 1;
         expect_throws<std::logic_error>(
             [&] { app::combine_report(total, unknown); }, "something_new");
       }},
      {"each snapshot's min and max are in double",
       [] {
         app::Session session;
         const Json case_json = heat_case("implicit_euler");
         const Json progress = session.solve(case_json);
         const Values values = session.solution_values(case_json);
         check(progress["snapshot_min"].size() == values.size() &&
                   progress["snapshot_max"].size() == values.size(),
               "one per snapshot");
         double low = 1e300, high = -1e300;
         for (std::size_t k = 0; k < values.size(); ++k) {
           const auto [a, b] = std::ranges::minmax(values[k]);
           check(progress["snapshot_min"][k].get<double>() == a &&
                     progress["snapshot_max"][k].get<double>() == b,
                 "snapshot " + std::to_string(k));
           low = std::min(low, a);
           high = std::max(high, b);
         }
         check(progress["min"] == low && progress["max"] == high,
               "the run's range");
       }},
      {"progress, stop, values, probes and status do not wait for a chunk",
       [] {
         app::Session session({.background_runs = true});
         // The first interval runs inside solve; the second goes on in the
         // background, holding the session mutex for about a second.
         const Json progress = session.solve(long_case());
         const auto job = progress["job"].get<std::uint64_t>();
         const auto until =
             std::chrono::steady_clock::now() + std::chrono::seconds(10);
         Json status = session.status();
         while (status["busy"].is_null() &&
                std::chrono::steady_clock::now() < until)
           status = session.status();
         check(status["busy"]["task"] == "run" && status["busy"]["job"] == job,
               "busy with the run: " + status.dump());
         check(status["busy"]["yours"] == true, "the local client's");
         const double limit = 0.25;
         double slowest = 0.0;
         const auto timed = [&](const char *what, const auto &call) {
           const double seconds = seconds_of(call);
           slowest = std::max(slowest, seconds);
           check(seconds < limit,
                 std::string(what) + " took " + std::to_string(seconds) + " s");
         };
         timed("progress", [&] { (void)session.solve_progress(job); });
         timed("status", [&] { (void)session.status(); });
         timed("values",
               [&] { (void)session.solution_values_float32(job, 0, 1); });
         timed("probes", [&] {
           (void)session.probes({{"job", job}, {"points", {{1.0, 0.5}}}});
         });
         // Still the same chunk: the calls above did not wait for it.
         check(session.status()["busy"]["task"] == "run",
               "the chunk was still running");
         timed("stop", [&] { (void)session.stop_solve(job); });
         const Json done = wait(session, job);
         check(done["state"] == "stopped" || done["state"] == "done",
               done["state"].dump());
       }},
      {"the memory budget drops other clients' finished runs first",
       [] {
         const Json case_json = app::Session::default_case();
         std::size_t bytes = 0;
         {
           app::Session probe;
           const Json progress = probe.solve(case_json);
           bytes = progress["memory"]["bytes"].get<std::size_t>();
           check(bytes > 0 && progress["memory"]["budget"] ==
                                  app::SessionOptions{}.memory_budget,
                 "the estimate and the budget");
         }
         app::Session session({.memory_budget = 5 * bytes / 2});
         const auto a = session.solve(case_json, "a")["job"];
         const auto b = session.solve(case_json, "b")["job"];
         Json status = session.status("c");
         check(status["memory"]["used"] == 2 * bytes, status.dump());
         // c fits once a's run (the least recently used) is dropped.
         (void)session.solve_progress(a.get<std::uint64_t>(), "a");
         (void)session.solve_progress(b.get<std::uint64_t>(), "b"); // last
         (void)session.solve(case_json, "c");
         const app::StageError gone = stage_error_of([&] {
           (void)session.solve_progress(a.get<std::uint64_t>(), "a");
         });
         check(gone.kind() == "expired", gone.kind());
         check(std::string(gone.what()).find("memory budget") !=
                   std::string::npos,
               gone.what());
         check(session.status()["memory"]["used"] == 2 * bytes, "two kept");
         // A run that cannot fit even alone is refused, with advice.
         app::Session small({.memory_budget = bytes / 2});
         const app::StageError refused =
             stage_error_of([&] { (void)small.solve(case_json); });
         check(refused.kind() == "memory", refused.kind());
         check(std::string(refused.what()).find("coarser mesh") !=
                   std::string::npos,
               refused.what());
         Json timed = heat_case("implicit_euler");
         app::Session tiny({.memory_budget = 1000});
         const app::StageError in_time =
             stage_error_of([&] { (void)tiny.solve(timed); });
         check(in_time.path() == "solve.time.snapshots", in_time.path());
         check(std::string(in_time.what()).find("fewer snapshots") !=
                   std::string::npos,
               in_time.what());
       }},
      {"a run that is gone says whether it was replaced or expired",
       [] {
         app::Session session;
         const Json case_json = app::Session::default_case();
         const auto first =
             session.solve(case_json)["job"].get<std::uint64_t>();
         Json other = case_json;
         other["solve"]["static"]["t"] = 1.0;
         (void)session.solve(other);
         const app::StageError replaced =
             stage_error_of([&] { (void)session.solve_progress(first); });
         check(replaced.kind() == "replaced", replaced.kind());
         const app::StageError unknown =
             stage_error_of([&] { (void)session.solve_progress(12345); });
         check(unknown.kind() == "expired", unknown.kind());
         check(std::string(unknown.what()).find("restarted") !=
                   std::string::npos,
               unknown.what());
         const app::StageError stranger = stage_error_of(
             [&] { (void)session.solve_progress(first, "someone"); });
         check(stranger.kind() == "expired", "another client's: expired");
         // A client beyond the limit drops the oldest one's run.
         app::Session few({.max_clients = 1});
         const auto old = few.solve(case_json, "x")["job"].get<std::uint64_t>();
         (void)few.solve(case_json, "y");
         const app::StageError limit =
             stage_error_of([&] { (void)few.solve_progress(old, "x"); });
         check(limit.kind() == "expired", limit.kind());
       }},
      {"a static solve is a run that is done at once",
       [] {
         app::Session session({.background_runs = true});
         const Json progress = session.solve(app::Session::default_case());
         check(progress["state"] == "done" && progress["mode"] == "static",
               "done");
         check(session.solution_values_float32(
                          progress["job"].get<std::uint64_t>(), 0, 1)
                       .size() ==
                   4 * progress["values"]["nodes"].get<std::size_t>(),
               "one snapshot");
       }},
      {"every preset runs on a rectangle and an annulus",
       [] {
         const Json presets = app::preset_catalog_json();
         check(presets.size() >= 9, "nine presets");
         for (const Json &base : {app::Session::default_case(), annulus_case()})
           for (const Json &preset : presets) {
             app::Session session;
             const std::string id = preset["id"].get<std::string>();
             const std::string what =
                 id + " on " + base["geometry"]["shape"].get<std::string>();
             const Json applied =
                 session.apply_preset({{"case", base}, {"preset", id}});
             const Json regions = session.geometry(applied)["regions"];
             check(applied["problem"]["boundary_conditions"].size() ==
                       regions.size(),
                   what + ": a condition per region");
             for (const Json &region : regions)
               check(applied["problem"]["boundary_conditions"].contains(
                         region["name"].get<std::string>()),
                     what + ": " + region["name"].dump());
             const Json summary = session.run(applied);
             check(summary["ok"] == true, what + ": " + summary.dump());
             const Values values = session.solution_values(applied);
             for (const auto &snapshot : values)
               for (const double v : snapshot)
                 check(std::isfinite(v), what + ": a finite solution");
             check(largest(values) > 1e-3, what + ": not zero");
             if (id == "laplace") {
               const Json mesh =
                   app::test_support::mesh_view(session, applied)["nodes"];
               for (std::size_t i = 0; i < values[0].size(); ++i)
                 check(std::abs(values[0][i] - mesh[2 * i].get<double>()) <
                           1e-8,
                       what + ": u = x");
             }
             if (id == "heat_cooling" || id == "wave_damped" ||
                 id == "heat_bump") {
               double first = 0.0, last = 0.0;
               for (const double v : values.front())
                 first = std::max(first, std::abs(v));
               for (const double v : values.back())
                 last = std::max(last, std::abs(v));
               check(last < 0.5 * first, what + ": decays");
             }
           }
         expect_throws<app::StageError>(
             [] {
               app::Session session;
               (void)session.apply_preset(
                   {{"case", app::Session::default_case()},
                    {"preset", "nothing"}});
             },
             "unknown preset 'nothing'");
       }},
  });
}

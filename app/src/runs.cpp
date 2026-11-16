#include "runs.hpp"

#include "problem_input.hpp"
#include "views.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/static_solver.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <stdexcept>
#include <utility>

namespace pde2d::app {

namespace {

// The times a time run keeps the solution at: snapshots + 1 equal steps
// from t0 to t1, the last exactly t1. Empty for an interval that is not
// valid: the library names a bad one itself.
std::vector<double> snapshot_times(const SolveSetup &setup) {
  std::vector<double> times;
  if (!std::isfinite(setup.t0) || !std::isfinite(setup.t1) ||
      !(setup.t1 > setup.t0))
    return times;
  const double span = setup.t1 - setup.t0;
  const auto count = static_cast<double>(setup.snapshots);
  for (std::size_t k = 0; k < setup.snapshots; ++k)
    times.push_back(setup.t0 + span * (static_cast<double>(k) / count));
  times.push_back(setup.t1);
  return times;
}

void keep_range(Run &run, const solve::Solution &solution) {
  for (const double value : solution.values)
    if (std::isfinite(value)) {
      if (!run.has_range) {
        run.low = run.high = value;
        run.has_range = true;
      }
      run.low = std::min(run.low, value);
      run.high = std::max(run.high, value);
    }
}

// One report for the chunks of a run: counts add up, extremes combine,
// t1 is the last chunk's.
void combine_report(std::optional<mesh::StageReport> &total,
                    const mesh::StageReport &part) {
  if (!total) {
    total = part;
    return;
  }
  auto &m = total->metrics;
  const auto value = [&](const std::string &name) {
    const auto it = part.metrics.find(name);
    return it == part.metrics.end() ? 0.0 : it->second;
  };
  for (const char *name : {"steps", "rejected_steps", "linear_iterations",
                           "assemblies", "prepares", "snapshots"})
    m[name] += value(name);
  for (const char *name : {"max_linear_iterations", "max_residual"})
    m[name] = std::max(m[name], value(name));
  if (value("steps") > 0) {
    m["min_dt"] = m["steps"] == value("steps")
                      ? value("min_dt")
                      : std::min(m["min_dt"], value("min_dt"));
    m["max_dt"] = std::max(m["max_dt"], value("max_dt"));
  }
  m["t1"] = value("t1");
  total->time += part.time;
  for (const std::string &message : part.messages)
    if (std::ranges::find(total->messages, message) == total->messages.end())
      total->messages.push_back(message);
}

// The sections a run is made from, for provenance.
Json run_settings(const Json &case_json) {
  Json out = Json::object();
  for (const char *name : {"mesh", "problem", "solve"})
    out[name] = member(case_json, name);
  return out;
}

} // namespace

std::string_view state_name(RunState state) {
  switch (state) {
  case RunState::running:
    return "running";
  case RunState::done:
    return "done";
  case RunState::failed:
    return "failed";
  case RunState::stopped:
    return "stopped";
  }
  throw std::logic_error("unknown run state");
}

void advance(Run &run, const SessionOptions &options) {
  if (run.state != RunState::running)
    return;
  const auto started = std::chrono::steady_clock::now();
  try {
    staged("solve", [&] {
      std::vector<solve::Solution> &solutions = *run.solutions;
      const std::size_t last_index = run.times.size() - 1;
      const std::size_t ready = solutions.size();
      const std::size_t from = ready == 0 ? 0 : ready - 1;
      const std::size_t to = std::min(last_index, from + run.chunk);
      solve::IntegrationOutput output;
      output.snapshot_times.assign(
          run.times.begin() + static_cast<std::ptrdiff_t>(ready),
          run.times.begin() + static_cast<std::ptrdiff_t>(to) + 1);
      solve::IntegrationResult result =
          solve::Integrator(run.setup.method, run.setup.step_controller,
                            run.setup.linear_solver)
              .integrate(run.meshed, run.times[from], run.times[to], output,
                         run.last ? &*run.last : nullptr);
      combine_report(run.integrator, result.report);
      for (solve::Solution &solution : result.snapshots) {
        keep_range(run, solution);
        solutions.push_back(std::move(solution));
      }
      run.last = std::move(result.final);
      ++run.chunks;
      if (solutions.size() != to + 1)
        throw std::logic_error("a chunk did not keep its snapshots");
      if (to == last_index)
        run.state = RunState::done;
    });
  } catch (const StageError &error) {
    run.state = RunState::failed;
    run.error = error;
  }
  const double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
          .count();
  if (options.chunk_seconds <= 0) {
    run.chunk = 1;
    return;
  }
  const double scaled = static_cast<double>(run.chunk) * options.chunk_seconds /
                        std::max(seconds, 1e-3);
  const double low = std::max(1.0, static_cast<double>(run.chunk) / 2);
  const double high = static_cast<double>(run.chunk) * 4;
  run.chunk =
      static_cast<std::size_t>(std::round(std::clamp(scaled, low, high)));
}

std::shared_ptr<Run> make_run(StageCache &cache, const Json &case_json,
                              const SessionOptions &options, std::uint64_t id) {
  const auto problem = cache.meshed(case_json);
  auto made = std::make_shared<Run>();
  made->key = solve_key(case_json);
  made->meshed = problem;
  made->mesh_id = cache.id_of(problem->mesh().get());
  made->settings = run_settings(case_json);
  made->started = std::chrono::system_clock::now();
  made->parameters = staged("problem", [&] {
    return read_parameters(
        member(member(case_json, "problem"), "parameters"));
  });
  made->setup = staged(
      "solve", [&] { return build_solve_setup(member(case_json, "solve")); });
  made->base["time_order"] =
      staged("solve", [&] { return fem::Assembler().time_order(*problem); });
  made->base["problem_time_order"] = fem::time_order(*problem);
  made->base["free_unknowns"] = problem->free_nodes().size();
  if (made->setup.mode == SolveMode::static_solve) {
    solve::StaticResult result = staged("solve", [&] {
      return solve::StaticSolver(made->setup.linear_solver)
          .solve(problem, made->setup.t);
    });
    made->base["mode"] = "static";
    made->static_reports = reports_json(result.reports);
    keep_range(*made, result.solution);
    made->solutions->push_back(std::move(result.solution));
    made->state = RunState::done;
  } else {
    made->base["mode"] = "time";
    made->times = snapshot_times(made->setup);
    if (made->times.size() < 2) {
      // An interval the library refuses: let it say why.
      staged("solve", [&] {
        (void)solve::Integrator(made->setup.method, made->setup.step_controller,
                                made->setup.linear_solver)
            .integrate(problem, made->setup.t0, made->setup.t1);
      });
      throw StageError("solve", "internal", "the time interval is empty");
    }
    made->chunk = std::max<std::size_t>(1, options.first_chunk);
    advance(*made, options);
    if (made->state == RunState::failed)
      throw *made->error;
  }
  made->id = id;
  return made;
}

Json progress_json(const Run &run) {
  const std::vector<solve::Solution> &solutions = *run.solutions;
  Json out = run.base;
  out["job"] = run.id;
  out["state"] = state_name(run.state);
  out["mesh_id"] = run.mesh_id;
  Json times = Json::array();
  for (const solve::Solution &solution : solutions)
    times.push_back(solution.time);
  out["times"] = std::move(times);
  out["min"] = run.has_range ? Json(run.low) : Json(nullptr);
  out["max"] = run.has_range ? Json(run.high) : Json(nullptr);
  if (run.setup.mode == SolveMode::time_integration) {
    out["t0"] = run.setup.t0;
    out["t1"] = run.setup.t1;
    out["t"] = run.last ? run.last->time : run.setup.t0;
    out["snapshots_total"] = run.times.size();
    Json reports = Json::array();
    if (run.integrator) {
      mesh::StageReport report = *run.integrator;
      report.metrics["chunks"] = static_cast<double>(run.chunks);
      reports.push_back(report_json(report));
    }
    out["reports"] = std::move(reports);
  } else
    out["reports"] = run.static_reports;
  // Where the values are: POST {"job", "first", "count"} to
  // /api/solve/values.
  const std::size_t nodes = run.meshed->mesh()->nodes.size();
  Json layout = Json::object();
  layout["path"] = "/api/solve/values";
  layout["format"] = "float32le";
  layout["snapshots"] = solutions.size();
  layout["nodes"] = nodes;
  layout["bytes"] = solutions.size() * nodes * 4;
  out["values"] = std::move(layout);
  if (run.error)
    out["error"] = error_json(*run.error);
  return out;
}

RunManager::Client &RunManager::client(const std::string &id) {
  auto found = clients_.find(id);
  if (found == clients_.end()) {
    // Dropped here, under the session mutex: a run holds expressions.
    while (clients_.size() >= std::max<std::size_t>(1, max_clients_)) {
      auto oldest = std::ranges::min_element(
          clients_, {}, [](const auto &item) { return item.second.used; });
      clients_.erase(oldest);
    }
    found = clients_.emplace(id, Client{}).first;
  }
  found->second.used = ++clock_;
  return found->second;
}

RunManager::Client *RunManager::find(const std::string &id) {
  const auto found = clients_.find(id);
  if (found == clients_.end())
    return nullptr;
  found->second.used = ++clock_;
  return &found->second;
}

Run &RunManager::run_of(const std::string &client_id, std::uint64_t job) {
  Client *owner = find(client_id);
  if (owner == nullptr || !owner->run || owner->run->id != job)
    throw StageError("solve", "input",
                     std::format("the run {} is no longer the current one "
                                 "(another solve replaced it)",
                                 job));
  return *owner->run;
}

Run *RunManager::next_running() {
  // The first running run after the client advanced last, wrapping round.
  auto start = clients_.upper_bound(last_advanced_);
  for (int pass = 0; pass < 2; ++pass) {
    for (auto it = pass == 0 ? start : clients_.begin();
         it != (pass == 0 ? clients_.end() : start); ++it)
      if (it->second.run && it->second.run->state == RunState::running) {
        last_advanced_ = it->first;
        return it->second.run.get();
      }
  }
  return nullptr;
}

std::shared_ptr<const mesh::Mesh>
RunManager::mesh_of(std::uint64_t mesh_id) const {
  for (const auto &[name, entry] : clients_)
    for (const Run *run :
         {static_cast<const Run *>(entry.run.get()), entry.reference.get()})
      if (run != nullptr && run->mesh_id == mesh_id)
        return run->meshed->mesh();
  return nullptr;
}

} // namespace pde2d::app

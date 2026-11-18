#include "runs.hpp"

#include "problem_input.hpp"
#include "views.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/static_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace pde2d::app {

namespace {

constexpr double nan = std::numeric_limits<double>::quiet_NaN();

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

// Keeps a snapshot (without its velocities: only the state a chunk ends
// with needs them) with its smallest and largest finite value. The run
// lock held.
void publish_snapshot(Run::Published &out, solve::Solution solution) {
  double low = nan, high = nan;
  for (const double value : solution.values)
    if (std::isfinite(value)) {
      low = std::isnan(low) ? value : std::min(low, value);
      high = std::isnan(high) ? value : std::max(high, value);
    }
  solution.velocities = {};
  out.minima.push_back(low);
  out.maxima.push_back(high);
  out.snapshots.push_back(
      std::make_shared<const solve::Solution>(std::move(solution)));
}

constexpr std::array<MetricRule, 17> metric_rules{{
    {"t0", Combine::first},
    {"t1", Combine::last},
    {"time_order", Combine::last},
    {"steps", Combine::sum},
    {"rejected_steps", Combine::sum},
    {"min_dt", Combine::min, true},
    {"max_dt", Combine::max, true},
    {"linear_iterations", Combine::sum},
    {"max_linear_iterations", Combine::max},
    {"max_residual", Combine::max},
    {"assemblies", Combine::sum},
    {"prepares", Combine::sum},
    {"snapshots", Combine::sum},
    {"assembly_threads", Combine::last},
    {"linear_threads", Combine::max},
    {"max_backward_error", Combine::max},
    {"min_pivot_ratio", Combine::min_measured},
}};

// The run's sections, for provenance.
Json run_settings(const Json &case_json) {
  Json out = Json::object();
  for (const char *name : {"mesh", "problem", "solve"})
    out[name] = member(case_json, name);
  return out;
}

Json number_or_null(double value) {
  return std::isfinite(value) ? Json(value) : Json(nullptr);
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

// --- chunk sizes
// --------------------------------------------------------------

namespace {

// Least squares over `recent`; not fitted when they do not tell the fixed
// cost apart.
ChunkCostModel fit_over(std::span<const ChunkCost> recent) {
  ChunkCostModel model;
  if (recent.empty())
    return model;
  double mean_steps = 0.0, mean_seconds = 0.0;
  for (const ChunkCost &cost : recent) {
    mean_steps += cost.steps;
    mean_seconds += cost.seconds;
  }
  const auto n = static_cast<double>(recent.size());
  mean_steps /= n;
  mean_seconds /= n;
  double covariance = 0.0, variance = 0.0;
  double fewest = recent.front().steps, most = fewest;
  for (const ChunkCost &cost : recent) {
    covariance += (cost.steps - mean_steps) * (cost.seconds - mean_seconds);
    variance += (cost.steps - mean_steps) * (cost.steps - mean_steps);
    fewest = std::min(fewest, cost.steps);
    most = std::max(most, cost.steps);
  }
  // Two clearly different step counts at least: a step more or less in
  // many is the step controller landing on a snapshot, not a second
  // point.
  if (most - fewest < std::max(1.0, 0.1 * mean_steps))
    return model;
  model.fitted = true;
  model.per_step = covariance / variance;
  model.fixed = mean_seconds - model.per_step * mean_steps;
  if (model.per_step < 0) {
    // Noise: no measurable cost per step.
    model.per_step = 0.0;
    model.fixed = mean_seconds;
  }
  if (model.fixed < 0) {
    double steps = 0.0, seconds = 0.0;
    for (const ChunkCost &cost : recent) {
      steps += cost.steps;
      seconds += cost.seconds;
    }
    model.fixed = 0.0;
    model.per_step = steps > 0 ? seconds / steps : 0.0;
  }
  return model;
}

} // namespace

ChunkCostModel fit_chunk_cost(std::span<const ChunkCost> history) {
  // The last chunks first: a cost may drift (a time-dependent matrix that
  // becomes harder to factorize). Once the plan settles they all have the
  // same steps; then the whole history tells the costs apart.
  constexpr std::size_t window = 8;
  if (history.size() > window)
    if (const ChunkCostModel recent = fit_over(history.last(window));
        recent.fitted)
      return recent;
  return fit_over(history);
}

std::size_t plan_chunk(std::span<const ChunkCost> history,
                       const SessionOptions &options) {
  if (options.chunk_seconds <= 0)
    return 1;
  if (history.empty())
    return std::max<std::size_t>(1, options.first_chunk);
  const ChunkCost &last = history.back();
  const auto last_intervals =
      static_cast<double>(std::max<std::size_t>(1, last.intervals));
  double steps = 0.0, intervals = 0.0;
  for (const ChunkCost &cost : history) {
    steps += cost.steps;
    intervals += static_cast<double>(cost.intervals);
  }
  const double steps_per_interval =
      intervals > 0 && steps > 0 ? steps / intervals : 1.0;
  const ChunkCostModel model = fit_chunk_cost(history);
  double next = 1.0;
  if (model.fitted) {
    const double per_step = std::max(model.per_step, 1e-12);
    const double share = std::clamp(options.max_fixed_share, 0.01, 1.0);
    // F / (F + c s) <= share, and at least chunk_seconds.
    const double for_share = model.fixed * (1.0 - share) / (share * per_step);
    const double for_target = (options.chunk_seconds - model.fixed) / per_step;
    // At most max_chunk_seconds; a fixed cost beyond half of that raises
    // the cap to twice the fixed cost, so it is never more than half a
    // chunk.
    const double cap_seconds =
        std::max(options.max_chunk_seconds, 2.0 * model.fixed);
    const double for_cap = (cap_seconds - model.fixed) / per_step;
    // The lower bounds rounded up, the cap down (it wins).
    next = std::min(
        std::ceil(std::max(for_share, for_target) / steps_per_interval - 1e-9),
        std::floor(for_cap / steps_per_interval + 1e-9));
    // A fit from few points may be far off: grow by at most 8 times.
    next = std::min(next, 8.0 * last_intervals);
  } else {
    // The chunks so far all took the same steps, so the fixed cost cannot
    // be told apart: scale by time towards chunk_seconds, and at least
    // double (a larger chunk costs less when the fixed cost dominates, and
    // the next fit has two points), within max_chunk_seconds or twice the
    // last chunk's time, whichever is longer (a chunk that is all fixed
    // cost would otherwise never grow past one interval).
    const double per_interval = std::max(last.seconds, 1e-6) / last_intervals;
    next = std::max(std::round(options.chunk_seconds / per_interval),
                    2.0 * last_intervals);
    next = std::min(next, std::floor(std::max(options.max_chunk_seconds,
                                              2.0 * last.seconds) /
                                         per_interval +
                                     1e-9));
  }
  if (!std::isfinite(next) || next < 1.0)
    return 1;
  return static_cast<std::size_t>(std::min(next, 1e9));
}

// --- reports
// --------------------------------------------------------------------

std::span<const MetricRule> integrator_metric_rules() { return metric_rules; }

void combine_report(std::optional<mesh::StageReport> &total,
                    const mesh::StageReport &part) {
  const auto rule_of = [](const std::string &name) -> const MetricRule & {
    for (const MetricRule &rule : metric_rules)
      if (rule.name == name)
        return rule;
    throw std::logic_error("no rule for the integrator metric '" + name +
                           "' (runs.cpp, metric_rules)");
  };
  for (const auto &[name, value] : part.metrics)
    (void)rule_of(name);
  if (!total) {
    total = part;
    return;
  }
  auto &m = total->metrics;
  const auto steps_of = [](const mesh::StageReport &report) {
    const auto it = report.metrics.find("steps");
    return it == report.metrics.end() ? 0.0 : it->second;
  };
  // Before the steps are added up: whether the total had chunks with steps.
  const bool total_has_steps = steps_of(*total) > 0;
  const bool part_has_steps = steps_of(part) > 0;
  for (const auto &[name, value] : part.metrics) {
    const MetricRule &rule = rule_of(name);
    if (rule.needs_steps && !part_has_steps)
      continue;
    const auto found = m.find(name);
    if (found == m.end() || (rule.needs_steps && !total_has_steps)) {
      m[name] = value;
      continue;
    }
    double &into = found->second;
    switch (rule.combine) {
    case Combine::sum:
      into += value;
      break;
    case Combine::max:
      into = std::max(into, value);
      break;
    case Combine::min:
      into = std::min(into, value);
      break;
    case Combine::min_measured:
      if (value > 0.0)
        into = into > 0.0 ? std::min(into, value) : value;
      break;
    case Combine::first:
      break;
    case Combine::last:
      into = value;
      break;
    }
  }
  total->time += part.time;
  for (const std::string &message : part.messages)
    if (std::ranges::find(total->messages, message) == total->messages.end())
      total->messages.push_back(message);
}

// --- runs
// ---------------------------------------------------------------------

RunSnapshots Run::snapshots() const {
  return {.id = id, .mesh = mesh, .snapshots = published.snapshots};
}

std::size_t run_bytes(std::size_t nodes, std::size_t snapshots,
                      int time_order) {
  // A snapshot: its values and what holds them (Solution, shared_ptr).
  constexpr std::size_t per_snapshot = 128;
  const std::size_t state = nodes * 8 * (time_order == 2 ? 2 : 1);
  return snapshots * (nodes * 8 + per_snapshot) + state;
}

void advance(Run &run, const SessionOptions &options, std::mutex &run_lock) {
  {
    const std::scoped_lock lock(run_lock);
    if (run.published.state != RunState::running)
      return;
  }
  // Written only here (under the session mutex), so read without the
  // run lock.
  const std::size_t ready = run.published.snapshots.size();
  const auto started = std::chrono::steady_clock::now();
  const std::size_t last_index = run.times.size() - 1;
  const std::size_t from = ready == 0 ? 0 : ready - 1;
  const std::size_t to = std::min(last_index, from + run.chunk);
  try {
    solve::IntegrationResult result = staged("solve", [&] {
      solve::IntegrationOutput output;
      output.snapshot_times.assign(
          run.times.begin() + static_cast<std::ptrdiff_t>(ready),
          run.times.begin() + static_cast<std::ptrdiff_t>(to) + 1);
      return solve::Integrator(run.setup.method, run.setup.step_controller,
                               run.setup.linear_solver)
          .integrate(run.meshed, run.times[from], run.times[to], output,
                     run.last ? &*run.last : nullptr);
    });
    if (ready + result.snapshots.size() != to + 1)
      throw StageError("solve", "internal",
                       "a chunk did not keep its snapshots");
    const auto steps = result.report.metrics.find("steps");
    run.costs.push_back(
        {.intervals = to - from,
         .steps = steps == result.report.metrics.end() ? 0.0 : steps->second,
         .seconds = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - started)
                        .count()});
    run.last = std::move(result.final);
    const std::scoped_lock lock(run_lock);
    Run::Published &out = run.published;
    combine_report(out.integrator, result.report);
    for (solve::Solution &solution : result.snapshots)
      publish_snapshot(out, std::move(solution));
    out.t = run.last->time;
    ++out.chunks;
    if (to == last_index)
      out.state = RunState::done;
  } catch (const StageError &error) {
    const std::scoped_lock lock(run_lock);
    run.published.state = RunState::failed;
    run.published.error = error;
  }
  run.chunk = plan_chunk(run.costs, options);
}

std::shared_ptr<Run> make_run(StageCache &cache, const Json &case_json,
                              std::uint64_t id, const std::string &client) {
  const auto problem = cache.meshed(case_json);
  auto made = std::make_shared<Run>();
  made->id = id;
  made->client = client;
  made->key = solve_key(case_json);
  made->meshed = problem;
  made->mesh = problem->mesh();
  made->mesh_id = cache.id_of(problem->mesh().get());
  made->settings = run_settings(case_json);
  made->started = std::chrono::system_clock::now();
  made->parameters = staged("problem", [&] {
    return read_parameters(member(member(case_json, "problem"), "parameters"));
  });
  made->setup = staged(
      "solve", [&] { return build_solve_setup(member(case_json, "solve")); });
  const int time_order =
      staged("solve", [&] { return fem::Assembler().time_order(*problem); });
  made->base["time_order"] = time_order;
  made->base["problem_time_order"] = fem::time_order(*problem);
  made->base["free_unknowns"] = problem->free_nodes().size();
  if (made->setup.mode == SolveMode::static_solve) {
    made->base["mode"] = "static";
    made->times = {made->setup.t};
    made->published.t = made->setup.t;
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
    made->published.t = made->setup.t0;
  }
  made->bytes =
      run_bytes(made->mesh->nodes.size(), made->times.size(),
                made->setup.mode == SolveMode::static_solve ? 0 : time_order);
  return made;
}

void start_run(Run &run, const SessionOptions &options, std::mutex &run_lock) {
  if (run.setup.mode == SolveMode::static_solve) {
    try {
      solve::StaticResult result = staged("solve", [&] {
        return solve::StaticSolver(run.setup.linear_solver)
            .solve(run.meshed, run.setup.t);
      });
      const std::scoped_lock lock(run_lock);
      run.published.static_reports = reports_json(result.reports);
      publish_snapshot(run.published, std::move(result.solution));
      run.published.state = RunState::done;
    } catch (const StageError &error) {
      const std::scoped_lock lock(run_lock);
      run.published.state = RunState::failed;
      run.published.error = error;
      throw;
    }
    return;
  }
  run.chunk = plan_chunk({}, options);
  advance(run, options, run_lock);
  const std::scoped_lock lock(run_lock);
  if (run.published.state == RunState::failed)
    throw *run.published.error;
}

Json progress_json(const Run &run, std::size_t memory_budget) {
  const Run::Published &p = run.published;
  Json out = run.base;
  out["job"] = run.id;
  out["state"] = state_name(p.state);
  out["mesh_id"] = run.mesh_id;
  Json times = Json::array();
  Json minima = Json::array();
  Json maxima = Json::array();
  double low = nan, high = nan;
  for (std::size_t k = 0; k < p.snapshots.size(); ++k) {
    times.push_back(p.snapshots[k]->time);
    minima.push_back(number_or_null(p.minima[k]));
    maxima.push_back(number_or_null(p.maxima[k]));
    if (!std::isnan(p.minima[k])) {
      low = std::isnan(low) ? p.minima[k] : std::min(low, p.minima[k]);
      high = std::isnan(high) ? p.maxima[k] : std::max(high, p.maxima[k]);
    }
  }
  out["times"] = std::move(times);
  out["min"] = number_or_null(low);
  out["max"] = number_or_null(high);
  out["snapshot_min"] = std::move(minima);
  out["snapshot_max"] = std::move(maxima);
  if (run.setup.mode == SolveMode::time_integration) {
    out["t0"] = run.setup.t0;
    out["t1"] = run.setup.t1;
    out["t"] = p.t;
    out["snapshots_total"] = run.times.size();
    Json reports = Json::array();
    if (p.integrator) {
      mesh::StageReport report = *p.integrator;
      report.metrics["chunks"] = static_cast<double>(p.chunks);
      reports.push_back(report_json(report));
    }
    out["reports"] = std::move(reports);
  } else
    out["reports"] = p.static_reports;
  // Where the values are: POST {"job", "first", "count"} to
  // /api/solve/values.
  const std::size_t nodes = run.mesh->nodes.size();
  Json layout = Json::object();
  layout["path"] = "/api/solve/values";
  layout["format"] = "float32le";
  layout["snapshots"] = p.snapshots.size();
  layout["nodes"] = nodes;
  layout["bytes"] = p.snapshots.size() * nodes * 4;
  out["values"] = std::move(layout);
  Json memory = Json::object();
  memory["bytes"] = run.bytes;
  memory["budget"] = memory_budget;
  out["memory"] = std::move(memory);
  if (p.error)
    out["error"] = error_json(*p.error);
  return out;
}

// --- the clients' runs
// ------------------------------------------------------------

void RunManager::forget(const std::shared_ptr<const Run> &run,
                        GoneReason reason) {
  if (!run)
    return;
  gone_[run->id] = {.client = run->client, .reason = reason};
  while (gone_.size() > max_gone)
    gone_.erase(gone_.begin());
  dropped_.push_back(run);
}

void RunManager::touch(const Run &run) { run.published.used = ++clock_; }

RunManager::Client &RunManager::client(const std::string &id) {
  auto found = clients_.find(id);
  if (found == clients_.end()) {
    while (clients_.size() >= std::max<std::size_t>(1, max_clients_)) {
      auto oldest = std::ranges::min_element(
          clients_, {}, [](const auto &item) { return item.second.used; });
      forget(oldest->second.run, GoneReason::client_limit);
      if (oldest->second.reference != oldest->second.run)
        forget(oldest->second.reference, GoneReason::client_limit);
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
  if (owner != nullptr && owner->run && owner->run->id == job) {
    touch(*owner->run);
    return *owner->run;
  }
  const auto gone = gone_.find(job);
  if (gone != gone_.end() && gone->second.client == client_id)
    switch (gone->second.reason) {
    case GoneReason::replaced:
      throw StageError("solve", "replaced",
                       std::format("the run {} was replaced by a newer solve "
                                   "of this page",
                                   job));
    case GoneReason::memory_budget:
      throw StageError(
          "solve", "expired",
          std::format("the run {} was dropped to make room for newer runs "
                      "(the server's memory budget for runs); solve again",
                      job));
    case GoneReason::client_limit:
      throw StageError(
          "solve", "expired",
          std::format("the run {} was dropped: more pages used this server "
                      "than it keeps runs for; solve again",
                      job));
    }
  throw StageError("solve", "expired",
                   std::format("this server has no run {} (it was restarted, "
                               "or the run was dropped long ago); solve again",
                               job));
}

std::shared_ptr<const Run>
RunManager::reference_of(const std::string &client_id) {
  Client *owner = find(client_id);
  if (owner == nullptr)
    return nullptr;
  if (!owner->reference && owner->reference_gone)
    throw StageError(
        "compare", "expired",
        *owner->reference_gone == GoneReason::memory_budget
            ? "the reference run was dropped to make room for newer runs "
              "(the server's memory budget for runs); keep a reference again"
            : "the reference run was dropped; keep a reference again");
  if (owner->reference)
    touch(*owner->reference);
  return owner->reference;
}

void RunManager::set_run(Client &owner, std::shared_ptr<Run> run) {
  if (owner.run)
    forget(owner.run, GoneReason::replaced);
  owner.run = std::move(run);
  if (owner.run)
    touch(*owner.run);
}

void RunManager::set_reference(Client &owner,
                               std::shared_ptr<const Run> reference) {
  if (owner.reference)
    dropped_.push_back(owner.reference);
  owner.reference = std::move(reference);
  owner.reference_gone.reset();
}

Run *RunManager::next_running() {
  // The first running run after the client advanced last, wrapping round.
  auto start = clients_.upper_bound(last_advanced_);
  for (int pass = 0; pass < 2; ++pass) {
    for (auto it = pass == 0 ? start : clients_.begin();
         it != (pass == 0 ? clients_.end() : start); ++it)
      if (it->second.run &&
          it->second.run->published.state == RunState::running) {
        last_advanced_ = it->first;
        return it->second.run.get();
      }
  }
  return nullptr;
}

std::size_t RunManager::running() const {
  return static_cast<std::size_t>(
      std::ranges::count_if(clients_, [](const auto &item) {
        return item.second.run &&
               item.second.run->published.state == RunState::running;
      }));
}

std::shared_ptr<const mesh::Mesh>
RunManager::mesh_of(std::uint64_t mesh_id) const {
  for (const auto &[name, entry] : clients_)
    for (const Run *run :
         {static_cast<const Run *>(entry.run.get()), entry.reference.get()})
      if (run != nullptr && run->mesh_id == mesh_id)
        return run->mesh;
  return nullptr;
}

std::size_t RunManager::used_bytes() const {
  std::set<const Run *> seen;
  std::size_t total = 0;
  for (const auto &[name, entry] : clients_)
    for (const Run *run :
         {static_cast<const Run *>(entry.run.get()), entry.reference.get()})
      if (run != nullptr && seen.insert(run).second)
        total += run->bytes;
  return total;
}

namespace {

// A run that another client's solve may drop: one that is not running.
bool droppable(const Run &run) {
  return run.published.state != RunState::running;
}

} // namespace

std::size_t RunManager::room_for(const std::string &client,
                                 std::size_t budget) const {
  std::set<const Run *> seen;
  std::size_t kept = 0;
  for (const auto &[name, entry] : clients_)
    for (const Run *run :
         {static_cast<const Run *>(entry.run.get()), entry.reference.get()}) {
      if (run == nullptr || !seen.insert(run).second)
        continue;
      const bool own = name == client;
      if (own && run == entry.run.get() && run != entry.reference.get())
        continue; // a solve replaces it
      if (own || !droppable(*run))
        kept += run->bytes;
    }
  return kept >= budget ? 0 : budget - kept;
}

bool RunManager::make_room(const std::string &client, std::size_t bytes,
                           std::size_t budget) {
  if (bytes > room_for(client, budget))
    return false;
  // What is kept now, the client's current run aside (a solve replaces
  // it).
  std::size_t kept = used_bytes();
  if (const auto it = clients_.find(client);
      it != clients_.end() && it->second.run &&
      it->second.run != it->second.reference)
    kept -= it->second.run->bytes;
  // Other clients' finished runs, least recently used first.
  std::vector<const Run *> candidates;
  for (const auto &[name, entry] : clients_)
    if (name != client)
      for (const Run *run :
           {static_cast<const Run *>(entry.run.get()), entry.reference.get()})
        if (run != nullptr && droppable(*run) &&
            std::ranges::find(candidates, run) == candidates.end())
          candidates.push_back(run);
  std::ranges::sort(candidates, {}, [](const Run *run) {
    return std::pair(run->published.used, run->id);
  });
  for (const Run *victim : candidates) {
    if (kept + bytes <= budget)
      break;
    kept -= victim->bytes;
    for (auto &[name, entry] : clients_) {
      if (entry.run.get() == victim) {
        forget(entry.run, GoneReason::memory_budget);
        entry.run.reset();
      }
      if (entry.reference.get() == victim) {
        forget(entry.reference, GoneReason::memory_budget);
        entry.reference.reset();
        entry.reference_gone = GoneReason::memory_budget;
      }
    }
  }
  return kept + bytes <= budget;
}

std::vector<std::shared_ptr<const Run>> RunManager::take_dropped() {
  return std::exchange(dropped_, {});
}

} // namespace pde2d::app

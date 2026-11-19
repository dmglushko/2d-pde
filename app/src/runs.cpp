#include "runs.hpp"

#include "problem_input.hpp"
#include "views.hpp"

#include "pde2d/adapt/adaptive_integrator.hpp"
#include "pde2d/adapt/adaptive_static_solver.hpp"
#include "pde2d/adapt/mesh_refinement.hpp"
#include "pde2d/fem/assembler.hpp"
#include "pde2d/mesh/remesher.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/static_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>
#include <set>
#include <span>
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
// Empties a vector and gives its memory back (assigning {} keeps the
// capacity).
void release(std::vector<double> &values) {
  std::vector<double>().swap(values);
}

// The warnings of reports that are not published themselves (earlier
// cycles, earlier pieces), each once, prefixed with where they came from,
// added to `into`'s messages.
void gather_messages(mesh::StageReport &into,
                     std::span<const mesh::StageReport> reports,
                     const std::string &where) {
  for (const mesh::StageReport &report : reports)
    for (const std::string &message : report.messages) {
      std::string line =
          std::format("{}, {}: {}", where, report.stage, message);
      if (std::ranges::find(into.messages, line) == into.messages.end())
        into.messages.push_back(std::move(line));
    }
}

void publish_snapshot(Run::Published &out, solve::Solution solution) {
  double low = nan, high = nan;
  for (const double value : solution.values)
    if (std::isfinite(value)) {
      low = std::isnan(low) ? value : std::min(low, value);
      high = std::isnan(high) ? value : std::max(high, value);
    }
  // A snapshot keeps only u: the velocities and Newmark's accelerations
  // are the next chunk's state (Run::last), not something a reader needs.
  release(solution.velocities);
  release(solution.accelerations);
  out.minima.push_back(low);
  out.maxima.push_back(high);
  out.snapshots.push_back(
      std::make_shared<const solve::Solution>(std::move(solution)));
}

constexpr std::array<MetricRule, 18> metric_rules{{
    {"t0", Combine::first},
    {"t1", Combine::last},
    {"t_end", Combine::last},
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

// --- adaptive runs
// ------------------------------------------------------------

// The library's adaptive loop with one MeshRefinement. Its refusal of a
// setting of the target (adaptive_static_solver: max_cycles must be ...)
// is marked at that setting.
adapt::AdaptiveStaticResult
solve_adaptive(const SolveSetup &setup,
               std::shared_ptr<const fem::MeshedProblem> problem,
               const adapt::CycleObserver &on_cycle) {
  const auto refinement = std::make_shared<const adapt::MeshRefinement>(
      setup.estimator, setup.marking,
      std::make_shared<const mesh::Remesher>(setup.refiner));
  const adapt::AdaptiveStaticSolver solver(
      solve::StaticSolver(setup.linear_solver, setup.assembler), {refinement});
  try {
    return solver.solve(std::move(problem), setup.target, setup.t, on_cycle);
  } catch (const std::invalid_argument &error) {
    const std::string_view message = error.what();
    if (message.starts_with("adaptive_static_solver:"))
      if (const std::string option =
              named_option(message, solve_catalog().adaptive_settings);
          !option.empty())
        throw PathError("solve.adaptive" + option, std::string(message));
    throw;
  }
}

// The library's adaptive loop in time with one MeshRefinement, over the
// run's snapshot times. Its refusal of a setting is marked at that setting.
adapt::AdaptiveIntegrationResult
solve_adaptive_time(const Run &run, const solve::StepObserver &on_step) {
  const SolveSetup &setup = run.setup;
  const auto refinement = std::make_shared<const adapt::MeshRefinement>(
      setup.estimator, setup.marking,
      std::make_shared<const mesh::Remesher>(setup.refiner));
  const adapt::AdaptiveIntegrator integrator(
      solve::Integrator(setup.method, setup.step_controller,
                        setup.linear_solver, setup.assembler),
      {refinement}, setup.time_options);
  solve::IntegrationOutput output;
  output.snapshot_times = run.times;
  output.on_step = on_step;
  try {
    return integrator.integrate(run.meshed, setup.t0, setup.t1,
                                setup.time_target, output);
  } catch (const std::invalid_argument &error) {
    const std::string_view message = error.what();
    if (message.starts_with("adaptive_integrator:")) {
      if (message.find("damping method") != std::string_view::npos)
        throw PathError("solve.adaptive_time.damping_method",
                        std::string(message));
      if (message.find("the schedule") != std::string_view::npos)
        throw PathError("solve.adaptive_time.every", std::string(message));
      if (const std::string option =
              named_option(message, solve_catalog().adaptive_time_settings);
          !option.empty())
        throw PathError("solve.adaptive_time" + option, std::string(message));
    }
    throw;
  }
}

// The metric `name` of the report of `stage` among `reports`, or null.
Json metric_of(std::span<const mesh::StageReport> reports,
               std::string_view stage, std::string_view name) {
  for (const mesh::StageReport &report : reports)
    if (report.stage == stage) {
      const auto found = report.metrics.find(std::string(name));
      if (found != report.metrics.end())
        return number_or_null(found->second);
    }
  return nullptr;
}

// Keeps an adaptive run's result: the last cycle's solution as its one
// snapshot on the last mesh (under its own id), every cycle's solution,
// the last estimate's indicators, and the summary.
// The row of cycle `k` (from 0) of an adaptive run, as the summary has it.
Json cycle_row(std::size_t k, const adapt::AdaptiveCycle &cycle) {
  const mesh::Mesh &mesh = *cycle.solution.mesh;
  double seconds = 0.0;
  for (const mesh::StageReport &report : cycle.reports)
    seconds += report.time.count();
  Json row = Json::object();
  row["cycle"] = k + 1;
  row["nodes"] = mesh.nodes.size();
  row["triangles"] = mesh.triangles.size();
  row["estimate"] =
      cycle.estimate ? number_or_null(cycle.estimate->total) : Json(nullptr);
  row["max_indicator"] =
      cycle.estimate ? metric_of(std::span(&cycle.estimate->report, 1),
                                 cycle.estimate->report.stage, "max_indicator")
                     : Json(nullptr);
  // Not on the last cycle: the loop stopped before marking.
  row["marked"] = metric_of(cycle.reports, "mesh_refinement", "marked");
  row["residual"] = metric_of(cycle.reports, "static_solver", "residual");
  row["seconds"] = seconds;
  return row;
}

// Done, or stopped when the caller stopped it.
RunState final_state(bool stopped) {
  return stopped ? RunState::stopped : RunState::done;
}

void publish_adaptive(Run &run, adapt::AdaptiveStaticResult result,
                      std::mutex &run_lock) {
  Json cycles = Json::array();
  for (std::size_t k = 0; k < result.cycles.size(); ++k)
    cycles.push_back(cycle_row(k, result.cycles[k]));
  Json summary = Json::object();
  summary["stop_reason"] = adapt::to_string(result.stop_reason);
  summary["tolerance"] =
      metric_of(std::span(&result.report, 1), result.report.stage, "tolerance");
  summary["cycles"] = std::move(cycles);
  summary["mesh_stats"] = mesh_stats_json(*result.problem->mesh());

  // The last cycle's reports are published; the earlier cycles' warnings
  // (a refinement that made small angles, a tiny pivot) go with the loop's.
  mesh::StageReport loop = result.report;
  for (std::size_t k = 0; k + 1 < result.cycles.size(); ++k)
    gather_messages(loop, result.cycles[k].reports,
                    std::format("cycle {}", k + 1));
  Json reports = Json::array();
  reports.push_back(report_json(loop));
  for (const Json &report : reports_json(result.cycles.back().reports))
    reports.push_back(report);

  std::vector<Snapshot> kept;
  for (adapt::AdaptiveCycle &cycle : result.cycles) {
    release(cycle.solution.velocities);
    kept.push_back(
        std::make_shared<const solve::Solution>(std::move(cycle.solution)));
  }
  std::shared_ptr<const std::vector<double>> indicators;
  if (result.cycles.back().estimate)
    indicators = std::make_shared<const std::vector<double>>(
        std::move(result.cycles.back().estimate->indicators));

  const std::scoped_lock lock(run_lock);
  run.meshed = result.problem;
  run.mesh = result.problem->mesh();
  run.mesh_id = run.adapted_mesh_id;
  run.base["free_unknowns"] = result.problem->free_nodes().size();
  Run::Published &out = run.published;
  out.static_reports = std::move(reports);
  out.adaptive = std::move(summary);
  out.indicators = std::move(indicators);
  publish_snapshot(out, *kept.back());
  out.cycles = std::move(kept);
  out.state = final_state(result.stop_reason == adapt::StopReason::stopped);
}
// Keeps an adaptive run in time's result: its snapshots, each on its
// piece's mesh; every mesh with an id (the starting mesh keeps its own, a
// new one takes the next reserved id); the last problem and mesh as the
// run's; and the summary (a row per adaptation time, the mesh of each
// snapshot).
void publish_adaptive_time(Run &run, adapt::AdaptiveIntegrationResult result,
                           std::mutex &run_lock) {
  std::vector<std::pair<std::uint64_t, std::shared_ptr<const mesh::Mesh>>>
      meshes;
  std::uint64_t reserved = 0;
  const std::uint64_t reserved_count =
      run.setup.time_target.max_refinements.value_or(0);
  const auto index_of = [&](const std::shared_ptr<const mesh::Mesh> &mesh) {
    for (std::size_t i = 0; i < meshes.size(); ++i)
      if (meshes[i].second == mesh)
        return i;
    std::uint64_t id = run.mesh_id;
    if (mesh != run.mesh) {
      if (reserved == reserved_count)
        throw StageError("solve", "internal",
                         "the run made more meshes than it has ids for");
      id = run.adapted_mesh_id + reserved++;
    }
    meshes.emplace_back(id, mesh);
    return meshes.size() - 1;
  };
  for (const adapt::IntegrationPiece &piece : result.pieces)
    (void)index_of(piece.problem->mesh());

  Json adaptations = Json::array();
  for (std::size_t k = 0; k < result.pieces.size(); ++k) {
    const adapt::IntegrationPiece &piece = result.pieces[k];
    if (!piece.adaptation)
      continue;
    const adapt::AdaptationPoint &point = *piece.adaptation;
    const mesh::Mesh &before = *piece.problem->mesh();
    Json row = Json::object();
    row["time"] = point.time;
    row["steps"] = point.steps;
    row["estimate"] =
        point.estimate ? number_or_null(point.estimate->total) : Json(nullptr);
    row["outcome"] = adapt::to_string(point.outcome);
    row["nodes"] = before.nodes.size();
    row["marked"] = metric_of(point.reports, "mesh_refinement", "marked");
    // The mesh the run goes on with (the next piece's).
    const mesh::Mesh &after = k + 1 < result.pieces.size()
                                  ? *result.pieces[k + 1].problem->mesh()
                                  : before;
    row["nodes_after"] = after.nodes.size();
    row["mesh"] = index_of(k + 1 < result.pieces.size()
                               ? result.pieces[k + 1].problem->mesh()
                               : piece.problem->mesh());
    adaptations.push_back(std::move(row));
  }
  Json snapshot_mesh = Json::array();
  for (const solve::Solution &snapshot : result.snapshots)
    snapshot_mesh.push_back(index_of(snapshot.mesh));
  const auto &metrics = result.report.metrics;
  const auto metric = [&](const char *name) {
    const auto found = metrics.find(name);
    return found == metrics.end() ? Json(nullptr)
                                  : number_or_null(found->second);
  };
  Json summary = Json::object();
  summary["adaptations"] = std::move(adaptations);
  summary["snapshot_mesh"] = std::move(snapshot_mesh);
  summary["refinements"] = metric("refinements");
  summary["pieces"] = metric("pieces");
  summary["steps"] = metric("steps");
  summary["damping_steps"] = metric("damping_steps");
  summary["tolerance"] = metric("tolerance");
  summary["messages"] = result.report.messages;
  const auto &last = result.pieces.back().problem;
  summary["mesh_stats"] = mesh_stats_json(*last->mesh());

  // The last piece's report is published; the warnings of the earlier
  // pieces and of every adaptation go with the loop's.
  mesh::StageReport loop = result.report;
  for (std::size_t k = 0; k < result.pieces.size(); ++k) {
    const adapt::IntegrationPiece &piece = result.pieces[k];
    const std::string where =
        std::format("t = {} to {}", piece.t_start, piece.t_end);
    if (k + 1 < result.pieces.size())
      gather_messages(loop, std::span(&piece.report, 1), where);
    if (piece.adaptation)
      gather_messages(
          loop, piece.adaptation->reports,
          std::format("adaptation at t = {}", piece.adaptation->time));
  }
  Json reports = Json::array();
  reports.push_back(report_json(loop));
  reports.push_back(report_json(result.pieces.back().report));

  const std::scoped_lock lock(run_lock);
  Run::Published &out = run.published;
  run.meshed = last;
  run.mesh = last->mesh();
  run.mesh_id = meshes[index_of(last->mesh())].first;
  run.base["free_unknowns"] = last->free_nodes().size();
  out.meshes = std::move(meshes);
  out.static_reports = std::move(reports);
  out.adaptive = std::move(summary);
  for (solve::Solution &snapshot : result.snapshots) {
    release(snapshot.accelerations);
    publish_snapshot(out, std::move(snapshot));
  }
  out.t = result.final.time;
  out.chunks = 1;
  out.state = final_state(result.stopped);
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
      // A NaN stays, from either side (std::max would drop a later one),
      // as in the integrator's own maximum of the backward error.
      if (!std::isnan(into) && !(value <= into))
        into = value;
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
  return {.id = id,
          .mesh = mesh,
          .snapshots = published.snapshots,
          .cycles = published.cycles,
          .indicators = published.indicators};
}

std::size_t run_bytes(std::size_t nodes, std::size_t snapshots,
                      int time_order) {
  // A snapshot: its values and what holds them (Solution, shared_ptr).
  constexpr std::size_t per_snapshot = 128;
  // u, and for time order 2 u_t and (Newmark) u_tt.
  const std::size_t state = nodes * 8 * (time_order == 2 ? 3 : 1);
  return snapshots * (nodes * 8 + per_snapshot) + state;
}

std::size_t adaptive_run_bytes(std::size_t max_nodes, std::size_t max_cycles) {
  // Per node: its value (8), its point (16), about two triangles (2 x 24
  // with size_t indices), two indicators (16) and the boundary edges and
  // the solver's copies around them.
  constexpr std::size_t per_node = 96;
  constexpr std::size_t per_cycle = 512;
  return max_cycles * (max_nodes * per_node + per_cycle);
}

std::size_t adaptive_time_run_bytes(std::size_t max_nodes,
                                    std::size_t snapshots, std::size_t meshes,
                                    int time_order) {
  // A mesh as in adaptive_run_bytes (less the values), the snapshots and
  // the state as in run_bytes; for time order 2 the library holds each
  // snapshot's u_t and u_tt too until it returns.
  constexpr std::size_t per_mesh_node = 88;
  constexpr std::size_t per_mesh = 512;
  const std::size_t derivatives =
      time_order == 2 ? snapshots * max_nodes * 16 : 0;
  return meshes * (max_nodes * per_mesh_node + per_mesh) +
         run_bytes(max_nodes, snapshots, time_order) + derivatives;
}

void request_stop(Run &run) {
  Run::Published &p = run.published;
  if (p.state != RunState::running)
    return;
  if (run.setup.mode == SolveMode::adaptive ||
      run.setup.mode == SolveMode::adaptive_time)
    p.stop_requested = true;
  else
    p.state = RunState::stopped;
}

namespace {

// Whether a run goes on: running, and not asked to stop. The run lock
// held.
bool goes_on(const Run &run) {
  return run.published.state == RunState::running &&
         !run.published.stop_requested;
}

// An adaptive run: the library's whole loop (one call, under the session
// mutex), its cycles (or its time) published as they come, Stop checked
// after every cycle (or step).
void run_adaptive(Run &run, std::mutex &run_lock) {
  try {
    if (run.setup.mode == SolveMode::adaptive_time) {
      adapt::AdaptiveIntegrationResult result = staged("solve", [&] {
        return solve_adaptive_time(run, [&](const solve::StepProgress &step) {
          const std::scoped_lock lock(run_lock);
          run.published.t = step.time;
          return goes_on(run);
        });
      });
      publish_adaptive_time(run, std::move(result), run_lock);
      return;
    }
    std::size_t cycles = 0;
    adapt::AdaptiveStaticResult result = staged("solve", [&] {
      return solve_adaptive(
          run.setup, run.meshed,
          [&](const adapt::AdaptiveCycle &cycle, const fem::MeshedProblem &) {
            Json row = cycle_row(cycles++, cycle);
            const std::scoped_lock lock(run_lock);
            Json &summary = run.published.adaptive;
            if (summary.is_null())
              summary = Json::object();
            summary["cycles"].push_back(std::move(row));
            return goes_on(run);
          });
    });
    publish_adaptive(run, std::move(result), run_lock);
  } catch (const StageError &error) {
    const std::scoped_lock lock(run_lock);
    run.published.state = RunState::failed;
    run.published.error = error;
  }
}

} // namespace

void advance(Run &run, const SessionOptions &options, std::mutex &run_lock) {
  {
    const std::scoped_lock lock(run_lock);
    if (run.published.state != RunState::running)
      return;
  }
  if (run.setup.mode == SolveMode::adaptive ||
      run.setup.mode == SolveMode::adaptive_time)
    return run_adaptive(run, run_lock);
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
      // Every step moves the progress, and Stop ends the chunk at the
      // next step (what it has is published; Continue goes on from there
      // with the anchor, as if it had not stopped).
      output.on_step = [&](const solve::StepProgress &step) {
        const std::scoped_lock lock(run_lock);
        run.published.t = step.time;
        return run.published.state == RunState::running;
      };
      // A run stopped inside an interval goes on from where it stood.
      const double start = run.last ? run.last->time : run.times[from];
      return solve::Integrator(run.setup.method, run.setup.step_controller,
                               run.setup.linear_solver, run.setup.assembler)
          .integrate(run.meshed, start, run.times[to], output,
                     run.last ? &*run.last : nullptr, std::nullopt,
                     run.anchor ? &*run.anchor : nullptr);
    });
    if (!result.stopped && ready + result.snapshots.size() != to + 1)
      throw StageError("solve", "internal",
                       "a chunk did not keep its snapshots");
    // A stopped chunk's cost says nothing about a whole one.
    if (!result.stopped) {
      const auto steps = result.report.metrics.find("steps");
      run.costs.push_back(
          {.intervals = to - from,
           .steps = steps == result.report.metrics.end() ? 0.0 : steps->second,
           .seconds = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - started)
                          .count()});
    }
    run.last = std::move(result.final);
    run.anchor = std::move(result.anchor);
    const std::scoped_lock lock(run_lock);
    Run::Published &out = run.published;
    combine_report(out.integrator, result.report);
    for (solve::Solution &solution : result.snapshots)
      publish_snapshot(out, std::move(solution));
    out.t = run.last->time;
    ++out.chunks;
    if (!result.stopped && to == last_index) {
      out.state = RunState::done;
      // A done run is never continued: its steps' history goes.
      run.anchor.reset();
    }
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
  } else if (made->setup.mode == SolveMode::adaptive) {
    made->base["mode"] = "adaptive";
    made->times = {made->setup.t};
    made->published.t = made->setup.t;
    made->adapted_mesh_id = cache.new_mesh_id();
    // The Mesh step's mesh it starts from (mesh_id becomes its last one).
    made->base["start_mesh_id"] = made->mesh_id;
  } else {
    const bool adaptive = made->setup.mode == SolveMode::adaptive_time;
    made->base["mode"] = adaptive ? "adaptive_time" : "time";
    if (adaptive) {
      made->adapted_mesh_id =
          made->setup.time_target.max_refinements
              ? cache.new_mesh_ids(*made->setup.time_target.max_refinements)
              : 0;
      made->base["start_mesh_id"] = made->mesh_id;
    }
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
  if (made->setup.mode == SolveMode::adaptive) {
    // Bounded by the target (the library refuses a cycle or node count of
    // 0, and a starting mesh above max_nodes, when the run starts).
    const adapt::AdaptiveTarget &target = made->setup.target;
    made->bytes = adaptive_run_bytes(
        std::max(made->mesh->nodes.size(), target.max_nodes.value_or(0)),
        std::max<std::size_t>(1, target.max_cycles.value_or(1)));
    return made;
  }
  if (made->setup.mode == SolveMode::adaptive_time) {
    const adapt::AdaptiveTimeTarget &target = made->setup.time_target;
    made->bytes = adaptive_time_run_bytes(
        std::max(made->mesh->nodes.size(), target.max_nodes.value_or(0)),
        made->times.size(), target.max_refinements.value_or(0) + 1, time_order);
    return made;
  }
  made->bytes =
      run_bytes(made->mesh->nodes.size(), made->times.size(),
                made->setup.mode == SolveMode::static_solve ? 0 : time_order);
  // An error-controlled step keeps the method's earlier steps in the
  // anchor between chunks (at most two states of the free unknowns).
  if (made->setup.mode == SolveMode::time_integration &&
      made->setup.step_controller->controls_error())
    made->bytes += made->mesh->nodes.size() * 8 * 2;
  return made;
}

void start_run(Run &run, const SessionOptions &options, std::mutex &run_lock) {
  // An adaptive run is one library call: the worker (advance) runs it, so
  // the page follows its cycles and can stop it.
  if (run.setup.mode == SolveMode::adaptive ||
      run.setup.mode == SolveMode::adaptive_time)
    return;
  if (run.setup.mode == SolveMode::static_solve) {
    try {
      solve::StaticResult result = staged("solve", [&] {
        return solve::StaticSolver(run.setup.linear_solver, run.setup.assembler)
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
  if (run.setup.mode == SolveMode::adaptive_time) {
    out["t0"] = run.setup.t0;
    out["t1"] = run.setup.t1;
    out["t"] = p.t;
    out["snapshots_total"] = run.times.size();
    out["reports"] = p.static_reports;
    Json adaptive = p.adaptive.is_null() ? Json::object() : p.adaptive;
    // Every mesh the snapshots are on (snapshot_mesh indexes it), whose
    // arrays POST /api/mesh/arrays sends.
    Json meshes = Json::array();
    for (const auto &[mesh_id, mesh] : p.meshes)
      meshes.push_back(mesh_summary_json(*mesh, mesh_id));
    adaptive["meshes"] = std::move(meshes);
    out["adaptive_time"] = std::move(adaptive);
  } else if (run.setup.mode == SolveMode::time_integration) {
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
  if (run.setup.mode == SolveMode::adaptive) {
    Json adaptive = p.adaptive.is_null() ? Json::object() : p.adaptive;
    // Where the indicators are: POST {"job"} to /api/solve/indicators.
    adaptive["indicators"] =
        p.indicators ? Json(p.indicators->size()) : Json(nullptr);
    // The last mesh, whose arrays POST /api/mesh/arrays sends, once the
    // result is published (done, or stopped after a cycle).
    if (!p.cycles.empty())
      adaptive["mesh"] = mesh_summary_json(*run.mesh, run.mesh_id);
    out["adaptive"] = std::move(adaptive);
  }
  // Where the values are: POST {"job", "first", "count"} to
  // /api/solve/values.
  // Snapshots on several meshes (an adaptive run in time) have their own
  // counts (snapshot_nodes); `nodes` is then the largest.
  std::size_t nodes = run.mesh->nodes.size();
  std::size_t values = 0;
  bool varied = false;
  for (const Snapshot &snapshot : p.snapshots) {
    const std::size_t count = snapshot->values.size();
    varied = varied || count != run.mesh->nodes.size();
    nodes = std::max(nodes, count);
    values += count;
  }
  Json layout = Json::object();
  layout["path"] = "/api/solve/values";
  layout["format"] = "float32le";
  layout["snapshots"] = p.snapshots.size();
  layout["nodes"] = nodes;
  layout["bytes"] = values * 4;
  if (varied) {
    Json counts = Json::array();
    for (const Snapshot &snapshot : p.snapshots)
      counts.push_back(snapshot->values.size());
    layout["snapshot_nodes"] = std::move(counts);
  }
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
      if (run != nullptr) {
        if (run->mesh_id == mesh_id)
          return run->mesh;
        for (const auto &[id, mesh] : run->published.meshes)
          if (id == mesh_id)
            return mesh;
      }
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

#include "session.hpp"

#include "catalog.hpp"
#include "domain_input.hpp"
#include "presets.hpp"
#include "problem_input.hpp"
#include "views.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/fem/compiled_expression.hpp"
#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/io/pde2d_mesh.hpp"
#include "pde2d/io/svg.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/static_solver.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <format>
#include <limits>
#include <optional>
#include <span>
#include <sstream>
#include <utility>
#include <vector>

#ifndef PDE2D_VERSION
#define PDE2D_VERSION "unknown"
#endif

namespace pde2d::app {

namespace {

constexpr std::string_view case_format = "pde2d-app-case";
constexpr int case_version = 1;

// Runs one stage and turns its exception into a StageError.
template <class Step> auto staged(std::string_view stage, Step step) {
  try {
    return step();
  } catch (const StageError &) {
    throw;
  } catch (const std::invalid_argument &error) {
    throw StageError(std::string(stage), "input", error.what());
  } catch (const std::runtime_error &error) {
    throw StageError(std::string(stage), "algorithm", error.what());
  } catch (const std::exception &error) {
    throw StageError(std::string(stage), "internal", error.what());
  }
}

const Json &section(const Json &case_json, std::string_view key) {
  return member(case_json, key);
}

std::string key_of(const Json &case_json, std::string_view name) {
  return section(case_json, name).dump();
}

// The "assembly" section: the time to assemble at (0 by default) and
// which time terms to build (Assembler::assemble's max_time_order, 2 by
// default).
struct AssemblySettings {
  double t = 0.0;
  int max_time_order = 2;
};

AssemblySettings assembly_settings(const Json &case_json) {
  AssemblySettings out;
  const Json &assembly = section(case_json, "assembly");
  if (assembly.is_null())
    return out;
  require_object(assembly, "assembly");
  static constexpr std::string_view known[] = {"t", "max_time_order"};
  reject_unknown_keys(assembly, "assembly", known);
  if (const Json &t = member(assembly, "t"); !t.is_null())
    out.t = require_number(t, "assembly.t");
  if (const Json &order = member(assembly, "max_time_order");
      !order.is_null()) {
    const double value = require_number(order, "assembly.max_time_order");
    // The library checks the value; the app only refuses what an int
    // cannot hold.
    if (value != std::floor(value) || std::abs(value) > 1e9)
      throw std::invalid_argument(
          "assembly.max_time_order: expected a whole number");
    out.max_time_order = static_cast<int>(value);
  }
  return out;
}

double assembly_time(const Json &case_json) {
  return assembly_settings(case_json).t;
}

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

// A free part of an operator in Matrix Market coordinate form (real,
// general, 1-based), `size` x `size`; a zero operator without a pattern
// has no entries.
std::string matrix_market(const fem::CsrMatrix &matrix, std::size_t size,
                          std::string_view comment) {
  std::string out = "%%MatrixMarket matrix coordinate real general\n";
  out += std::format("% {}\n", comment);
  const fem::SparsityPattern *pattern = matrix.pattern.get();
  const std::size_t nonzeros = pattern ? pattern->nonzeros() : 0;
  out += std::format("{} {} {}\n", size, size, nonzeros);
  if (pattern)
    for (std::size_t row = 0; row < pattern->rows; ++row)
      for (auto slot = pattern->row_offsets[row];
           slot < pattern->row_offsets[row + 1]; ++slot) {
        const auto index = static_cast<std::size_t>(slot);
        out += std::format("{} {} {}\n", row + 1,
                           pattern->column_indices[index] + 1,
                           matrix.values[index]);
      }
  return out;
}

// A vector in Matrix Market array form (one column).
std::string matrix_market(std::span<const double> vector,
                          std::string_view comment) {
  std::string out = "%%MatrixMarket matrix array real general\n";
  out += std::format("% {}\n", comment);
  out += std::format("{} 1\n", vector.size());
  for (const double value : vector)
    out += std::format("{}\n", value);
  return out;
}

// Values as float32, little-endian, one after the other.
void append_float32(std::string &out, const std::vector<double> &values) {
  for (const double value : values) {
    const auto bits = std::bit_cast<std::uint32_t>(static_cast<float>(value));
    for (int byte = 0; byte < 4; ++byte)
      out.push_back(static_cast<char>((bits >> (8 * byte)) & 0xffU));
  }
}

Json values_json(const std::vector<double> &values) {
  Json out = Json::array();
  for (const double value : values)
    out.push_back(std::isfinite(value) ? Json(value) : Json(nullptr));
  return out;
}

Json operator_json(const fem::Operator &op) {
  const auto nonzeros = [](const fem::CsrMatrix &matrix) -> std::size_t {
    return matrix.pattern ? matrix.pattern->nonzeros() : 0;
  };
  Json out = Json::object();
  out["always_zero"] = op.always_zero;
  out["always_symmetric"] = op.always_symmetric;
  out["zero_now"] = op.zero_now;
  out["symmetric_now"] = op.symmetric_now;
  out["depends_on_t"] = op.depends_on_t;
  out["free_nonzeros"] = nonzeros(op.free);
  out["dirichlet_nonzeros"] = nonzeros(op.dirichlet);
  return out;
}

// "x,y,u(t=...),..." with one row per mesh node.
std::string solution_csv(const mesh::Mesh &mesh,
                         const std::vector<solve::Solution> &solutions) {
  std::string out = "x,y";
  for (const solve::Solution &solution : solutions)
    out += std::format(",u(t={})", solution.time);
  out += '\n';
  for (std::size_t i = 0; i < mesh.nodes.size(); ++i) {
    out += std::format("{},{}", mesh.nodes[i].x, mesh.nodes[i].y);
    for (const solve::Solution &solution : solutions)
      out += std::format(",{}", solution.values[i]);
    out += '\n';
  }
  return out;
}

} // namespace

// --- state ------------------------------------------------------------------

struct MeshOutcome {
  std::shared_ptr<const mesh::Mesh> mesh;
  std::vector<mesh::StageReport> reports;
  bool target_met = false;
  double longest_edge = 0.0;
  double max_edge_length = 0.0;
  double domain_size = 0.0;
};

// A solve: a static one is done when it is made; a time integration runs
// in chunks that end on snapshot times, each continuing from the state the
// previous one ended with (Integrator::integrate with `initial`), so the
// page sees the snapshots while the run goes on (docs/app.md, "Runs in
// chunks").
struct SolveJob {
  std::uint64_t id = 0;
  std::string key;               // the inputs it was made from
  std::string state = "running"; // running, done, failed, stopped
  std::shared_ptr<const fem::MeshedProblem> meshed;
  SolveSetup setup;
  // The snapshot times of a time run (snapshot_times); the kept solutions
  // are the first solutions.size() of them.
  std::vector<double> times;
  std::vector<solve::Solution> solutions;
  // The state at the end of the last chunk (with velocities for time
  // order 2), the start of the next one.
  std::optional<solve::Solution> last;
  std::size_t chunk = 1; // snapshot intervals in the next chunk
  std::size_t chunks = 0;
  Json base = Json::object(); // mode, time orders, free unknowns
  Json static_reports = Json::array();
  std::optional<mesh::StageReport> integrator; // the chunks' reports, combined
  double low = std::numeric_limits<double>::infinity();
  double high = -std::numeric_limits<double>::infinity();
  Json error = nullptr; // {stage, kind, message} of a failed run
};

namespace {

void keep_range(SolveJob &job, const solve::Solution &solution) {
  for (const double value : solution.values)
    if (std::isfinite(value)) {
      job.low = std::min(job.low, value);
      job.high = std::max(job.high, value);
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

// Runs the next chunk of a running time run: from the last kept snapshot
// time over `chunk` snapshot intervals. A failure ends the run with its
// error. The next chunk is scaled so that it takes about chunk_seconds.
void advance(SolveJob &job, const SessionOptions &options) {
  if (job.state != "running")
    return;
  const auto started = std::chrono::steady_clock::now();
  try {
    staged("solve", [&] {
      const std::size_t last_index = job.times.size() - 1;
      const std::size_t ready = job.solutions.size();
      const std::size_t from = ready == 0 ? 0 : ready - 1;
      const std::size_t to = std::min(last_index, from + job.chunk);
      solve::IntegrationOutput output;
      output.snapshot_times.assign(
          job.times.begin() + static_cast<std::ptrdiff_t>(ready),
          job.times.begin() + static_cast<std::ptrdiff_t>(to) + 1);
      solve::IntegrationResult result =
          solve::Integrator(job.setup.method, job.setup.step_controller,
                            job.setup.linear_solver)
              .integrate(job.meshed, job.times[from], job.times[to], output,
                         job.last ? &*job.last : nullptr);
      combine_report(job.integrator, result.report);
      for (solve::Solution &solution : result.snapshots) {
        keep_range(job, solution);
        job.solutions.push_back(std::move(solution));
      }
      job.last = std::move(result.final);
      ++job.chunks;
      if (job.solutions.size() != to + 1)
        throw std::logic_error("a chunk did not keep its snapshots");
      if (to == last_index)
        job.state = "done";
    });
  } catch (const StageError &error) {
    job.state = "failed";
    job.error = Json::object();
    job.error["stage"] = error.stage();
    job.error["kind"] = error.kind();
    job.error["message"] = error.what();
  }
  const double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
          .count();
  if (options.chunk_seconds <= 0) {
    job.chunk = 1;
    return;
  }
  const double scaled = static_cast<double>(job.chunk) * options.chunk_seconds /
                        std::max(seconds, 1e-3);
  const double low = std::max(1.0, static_cast<double>(job.chunk) / 2);
  const double high = static_cast<double>(job.chunk) * 4;
  job.chunk =
      static_cast<std::size_t>(std::round(std::clamp(scaled, low, high)));
}

Json progress_json(const SolveJob &job) {
  Json out = job.base;
  out["job"] = job.id;
  out["state"] = job.state;
  Json times = Json::array();
  for (const solve::Solution &solution : job.solutions)
    times.push_back(solution.time);
  out["times"] = std::move(times);
  out["min"] = std::isfinite(job.low) ? Json(job.low) : Json(nullptr);
  out["max"] = std::isfinite(job.high) ? Json(job.high) : Json(nullptr);
  if (job.setup.mode == SolveMode::time_integration) {
    out["t0"] = job.setup.t0;
    out["t1"] = job.setup.t1;
    out["t"] = job.last ? job.last->time : job.setup.t0;
    out["snapshots_total"] = job.times.size();
    Json reports = Json::array();
    if (job.integrator) {
      mesh::StageReport report = *job.integrator;
      report.metrics["chunks"] = static_cast<double>(job.chunks);
      reports.push_back(report_json(report));
    }
    out["reports"] = std::move(reports);
  } else
    out["reports"] = job.static_reports;
  // Where the values are: POST {"job", "first", "count"} to
  // /api/solve/values.
  const std::size_t nodes = job.meshed->mesh()->nodes.size();
  Json layout = Json::object();
  layout["path"] = "/api/solve/values";
  layout["format"] = "float32le";
  layout["snapshots"] = job.solutions.size();
  layout["nodes"] = nodes;
  layout["bytes"] = job.solutions.size() * nodes * 4;
  out["values"] = std::move(layout);
  if (!job.error.is_null())
    out["error"] = job.error;
  return out;
}

StageError job_error(const SolveJob &job) {
  return StageError(job.error["stage"].get<std::string>(),
                    job.error["kind"].get<std::string>(),
                    job.error["message"].get<std::string>());
}

} // namespace

// The last result of each stage and the key (the case sections) it was
// built from.
struct Session::State {
  std::string domain_key;
  std::shared_ptr<const Domain> domain;
  std::string mesh_key;
  std::shared_ptr<const MeshOutcome> mesh;
  std::string meshed_key;
  std::shared_ptr<const fem::MeshedProblem> meshed;
  std::string assembled_key;
  std::shared_ptr<const fem::AssembledProblem> assembled;
  // The current solve (one at a time) and the id the next one gets.
  std::unique_ptr<SolveJob> job;
  std::uint64_t next_job = 1;

  const Domain &get_domain(const Json &case_json) {
    std::string key = key_of(case_json, "geometry");
    if (!domain || key != domain_key) {
      domain.reset();
      domain = staged("geometry", [&] {
        return std::make_shared<const Domain>(
            build_domain(section(case_json, "geometry")));
      });
      domain_key = std::move(key);
    }
    return *domain;
  }

  const MeshOutcome &get_mesh(const Json &case_json) {
    const Domain &input = get_domain(case_json);
    std::string key = domain_key + "\n" + key_of(case_json, "mesh");
    if (!mesh || key != mesh_key) {
      mesh.reset();
      const MeshSetup setup = staged(
          "mesh", [&] { return build_mesh_setup(section(case_json, "mesh")); });
      mesh::MeshResult result = staged("mesh", [&] {
        return input.geometry ? setup.mesher().run(*input.geometry, setup.spec)
                              : setup.remesher().run(*input.mesh, setup.spec);
      });
      auto outcome = std::make_shared<MeshOutcome>();
      outcome->mesh =
          std::make_shared<const mesh::Mesh>(std::move(result.mesh));
      outcome->reports = std::move(result.reports);
      outcome->target_met = result.target_met;
      outcome->longest_edge = result.longest_edge;
      outcome->max_edge_length = result.max_edge_length;
      outcome->domain_size = result.domain_size;
      mesh = std::move(outcome);
      mesh_key = std::move(key);
    }
    return *mesh;
  }

  const std::shared_ptr<const fem::MeshedProblem> &
  get_meshed(const Json &case_json) {
    const MeshOutcome &outcome = get_mesh(case_json);
    std::string key = mesh_key + "\n" + key_of(case_json, "problem");
    if (!meshed || key != meshed_key) {
      assembled.reset();
      meshed.reset();
      const fem::Problem problem = staged("problem", [&] {
        return build_problem(section(case_json, "problem"));
      });
      meshed = staged("meshed_problem", [&] {
        return std::make_shared<const fem::MeshedProblem>(outcome.mesh,
                                                          problem);
      });
      meshed_key = std::move(key);
    }
    return meshed;
  }

  // The case's "assembly" section, read for the assembly stage.
  static AssemblySettings read_assembly(const Json &case_json) {
    return staged("assembly", [&] { return assembly_settings(case_json); });
  }

  // With `settings` (the case's, or another t for a field).
  const fem::AssembledProblem &get_assembled(const Json &case_json,
                                             const AssemblySettings &settings) {
    const double t = settings.t;
    const auto &problem = get_meshed(case_json);
    std::string key =
        meshed_key + "\n" + std::format("{} {}", t, settings.max_time_order);
    if (!assembled || key != assembled_key) {
      // A previous result of the same MeshedProblem lets assembly reuse
      // its constant parts; the result is the same bit for bit.
      const fem::AssembledProblem *previous =
          assembled && assembled->meshed_problem() == problem ? assembled.get()
                                                              : nullptr;
      auto next = staged("assembly", [&] {
        return std::make_shared<const fem::AssembledProblem>(
            fem::Assembler().assemble(problem, t, previous,
                                      settings.max_time_order));
      });
      assembled = std::move(next);
      assembled_key = std::move(key);
    }
    return *assembled;
  }

  // A new solve of the case; a static one is solved, a time one has run
  // its first chunk. Throws the StageError of either.
  std::unique_ptr<SolveJob> make_job(const Json &case_json,
                                     const SessionOptions &options) {
    const auto &problem = get_meshed(case_json);
    auto made = std::make_unique<SolveJob>();
    made->key = meshed_key + "\n" + key_of(case_json, "solve");
    made->meshed = problem;
    made->setup = staged("solve", [&] {
      return build_solve_setup(section(case_json, "solve"));
    });
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
      made->solutions.push_back(std::move(result.solution));
      made->state = "done";
    } else {
      made->base["mode"] = "time";
      made->times = snapshot_times(made->setup);
      if (made->times.size() < 2) {
        // An interval the library refuses: let it say why.
        staged("solve", [&] {
          (void)solve::Integrator(made->setup.method,
                                  made->setup.step_controller,
                                  made->setup.linear_solver)
              .integrate(problem, made->setup.t0, made->setup.t1);
        });
        throw StageError("solve", "internal", "the time interval is empty");
      }
      made->chunk = std::max<std::size_t>(1, options.first_chunk);
      advance(*made, options);
      if (made->state == "failed")
        throw job_error(*made);
    }
    made->id = next_job++;
    return made;
  }

  // The current job if it is `id`, else a StageError.
  SolveJob &job_of(std::uint64_t id) {
    if (!job || job->id != id)
      throw StageError("solve", "input",
                       std::format("the run {} is no longer the current one "
                                   "(another solve replaced it)",
                                   id));
    return *job;
  }
};

Session::Session(SessionOptions options)
    : options_(options), state_(std::make_unique<State>()) {
  if (options_.background_runs)
    worker_ = std::thread([this] { background_loop(); });
}

Session::~Session() {
  {
    const std::scoped_lock lock(mutex_);
    closing_ = true;
  }
  wake_.notify_all();
  if (worker_.joinable())
    worker_.join();
}

// Runs the current time run chunk by chunk. The mutex is released between
// chunks, so other requests (a problem check, a field) are served then.
void Session::background_loop() {
  std::unique_lock lock(mutex_);
  while (!closing_) {
    if (state_->job && state_->job->state == "running") {
      advance(*state_->job, options_);
      lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      lock.lock();
    } else
      wake_.wait(lock);
  }
}

// --- the catalog and the default case ---------------------------------------

Json Session::catalog() {
  Json sections = Json::array();
  sections.push_back(mesh_catalog_json());
  sections.push_back(solve_catalog_json());

  Json out = Json::object();
  out["format"] = case_format;
  out["version"] = case_version;
  out["library_version"] = PDE2D_VERSION;
  out["shapes"] = shape_catalog_json();
  out["sections"] = std::move(sections);
  out["problem"] = problem_catalog_json();
  out["problem"]["presets"] = preset_catalog_json();
  out["default_case"] = default_case();
  return out;
}

Json Session::default_case() {
  Json geometry = default_geometry_section();
  Json problem = default_problem_section();
  // One condition per region of the default shape, so the default case
  // runs to the end.
  const Domain domain = build_domain(geometry);
  for (const std::string &region : domain.geometry->region_names())
    problem["boundary_conditions"][region] = default_condition();

  Json assembly = Json::object();
  assembly["t"] = 0.0;
  assembly["max_time_order"] = 2;
  Json out = Json::object();
  out["format"] = case_format;
  out["version"] = case_version;
  out["geometry"] = std::move(geometry);
  out["mesh"] = default_mesh_section();
  out["problem"] = std::move(problem);
  out["assembly"] = std::move(assembly);
  out["solve"] = default_solve_section();
  return out;
}

void check_case(const Json &case_json) {
  require_object(case_json, "case");
  static constexpr std::string_view known[] = {"format", "version", "geometry",
                                               "mesh",   "problem", "assembly",
                                               "solve",  "view",    "notes"};
  reject_unknown_keys(case_json, "case", known);
  if (const Json &format = member(case_json, "format"); !format.is_null()) {
    if (require_string(format, "format") != case_format)
      throw std::invalid_argument("format: this is not a pde2d app case");
  }
  if (const Json &version = member(case_json, "version"); !version.is_null()) {
    const double number = require_number(version, "version");
    if (number != std::floor(number) || number < 1)
      throw std::invalid_argument("version: expected a whole number from 1");
    if (number > case_version)
      throw std::invalid_argument(
          "version: the case was written by a newer version of the app");
  }
}

// --- stages -----------------------------------------------------------------

Json Session::geometry(const Json &case_json) {
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  return staged("geometry",
                [&] { return domain_json(state_->get_domain(case_json)); });
}

Json Session::mesh(const Json &case_json) {
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  const MeshOutcome &outcome = state_->get_mesh(case_json);
  Json out = Json::object();
  out["mesh"] = mesh_json(*outcome.mesh);
  out["stats"] = mesh_stats_json(*outcome.mesh);
  out["target_met"] = outcome.target_met;
  out["longest_edge"] = outcome.longest_edge;
  out["max_edge_length"] = outcome.max_edge_length;
  out["domain_size"] = outcome.domain_size;
  out["reports"] = reports_json(outcome.reports);
  return out;
}

Json Session::check_problem(const Json &case_json) {
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  return staged("problem", [&] {
    return app::check_problem(section(case_json, "problem"));
  });
}

Json Session::assemble(const Json &case_json) {
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  const AssemblySettings settings = State::read_assembly(case_json);
  const fem::AssembledProblem &assembled =
      state_->get_assembled(case_json, settings);
  const fem::MeshedProblem &meshed = *assembled.meshed_problem();

  Json dirichlet = Json::array();
  for (const fem::DirichletNode &node : meshed.dirichlet_nodes())
    dirichlet.push_back(node.node);
  Json meshed_json = Json::object();
  meshed_json["report"] = report_json(meshed.report());
  meshed_json["dirichlet_nodes"] = std::move(dirichlet);
  meshed_json["free_nodes"] = meshed.free_nodes().size();
  meshed_json["time_order"] = fem::time_order(meshed);
  meshed_json["coefficients_depend_on_t"] =
      fem::coefficients_depend_on_t(meshed);
  meshed_json["data_depends_on_t"] = fem::data_depends_on_t(meshed);

  Json operators = Json::object();
  operators["mass"] = operator_json(assembled.mass());
  operators["damping"] = operator_json(assembled.damping());
  operators["stiffness"] = operator_json(assembled.stiffness());
  double norm = 0.0;
  for (const double value : assembled.right_hand_side())
    norm += value * value;

  Json assembly = Json::object();
  assembly["report"] = report_json(assembled.report());
  assembly["t"] = assembled.time();
  assembly["max_time_order"] = settings.max_time_order;
  assembly["time_order"] = assembled.time_order();
  // The order assembly gives at every t, from the formulas alone.
  assembly["time_order_any_t"] = fem::Assembler().time_order(meshed);
  assembly["operators"] = std::move(operators);
  assembly["load_depends_on_t"] = assembled.load_depends_on_t();
  assembly["right_hand_side_depends_on_t"] =
      assembled.right_hand_side_depends_on_t();
  assembly["right_hand_side_norm"] = std::sqrt(norm);
  if (const auto &pattern = assembled.stiffness().free.pattern)
    assembly["sparsity"] = sparsity_json(*pattern, 160);

  Json out = Json::object();
  out["meshed_problem"] = std::move(meshed_json);
  out["assembly"] = std::move(assembly);
  return out;
}

Json Session::field(const Json &request) {
  const std::scoped_lock lock(mutex_);
  staged("field", [&] {
    require_object(request, "request");
    static constexpr std::string_view known[] = {"case", "field"};
    reject_unknown_keys(request, "request", known);
  });
  const Json &case_json = member(request, "case");
  staged("case", [&] { check_case(case_json); });
  const Json &spec = staged("field", [&]() -> const Json & {
    const Json &object = require_object(member(request, "field"), "field");
    static constexpr std::string_view known[] = {"kind", "text", "t"};
    reject_unknown_keys(object, "field", known);
    return object;
  });
  const std::string kind = staged("field", [&] {
    return require_string(member(spec, "kind"), "field.kind");
  });
  const double t = staged("field", [&] {
    const Json &value = member(spec, "t");
    return value.is_null() ? assembly_time(case_json)
                           : require_number(value, "field.t");
  });

  Json out = Json::object();
  out["kind"] = kind;
  out["t"] = t;
  if (kind == "expression") {
    const MeshOutcome &outcome = state_->get_mesh(case_json);
    const auto text = staged("field", [&] {
      return require_string(member(spec, "text"), "field.text");
    });
    const fem::CompiledExpression compiled = staged("field", [&] {
      const fem::Parameters parameters =
          read_parameters(member(section(case_json, "problem"), "parameters"));
      return fem::CompiledExpression(fem::Expression(text), parameters);
    });
    std::vector<double> values;
    values.reserve(outcome.mesh->nodes.size());
    for (const geometry::Point &p : outcome.mesh->nodes)
      values.push_back(compiled(p.x, p.y, t));
    out["label"] = text;
    out["values"] = values_json(values);
    return out;
  }
  if (kind == "initial_u" || kind == "initial_u_t") {
    const auto &meshed = state_->get_meshed(case_json);
    const auto values = staged("field", [&] {
      return kind == "initial_u" ? meshed->initial_values(t)
                                 : meshed->initial_velocities(t);
    });
    if (!values)
      throw StageError("field", "input",
                       kind == "initial_u"
                           ? "the problem has time order 0, so it has no "
                             "initial values"
                           : "the problem has time order below 2, so it has "
                             "no initial velocities");
    out["label"] = kind == "initial_u" ? "u at the start" : "u_t at the start";
    out["values"] = values_json(*values);
    return out;
  }
  if (kind == "dirichlet") {
    const auto &meshed = state_->get_meshed(case_json);
    std::vector<double> values(meshed->mesh()->nodes.size(), std::nan(""));
    staged("field", [&] {
      for (std::size_t k = 0; k < meshed->dirichlet_nodes().size(); ++k)
        values[meshed->dirichlet_nodes()[k].node] =
            meshed->dirichlet_value(k, t);
    });
    out["label"] = "Dirichlet values";
    out["values"] = values_json(values);
    return out;
  }
  if (kind == "right_hand_side") {
    AssemblySettings settings = State::read_assembly(case_json);
    settings.t = t;
    const fem::AssembledProblem &assembled =
        state_->get_assembled(case_json, settings);
    const fem::MeshedProblem &meshed = *assembled.meshed_problem();
    std::vector<double> values(meshed.mesh()->nodes.size(), std::nan(""));
    const auto rhs = assembled.right_hand_side();
    for (std::size_t i = 0; i < rhs.size(); ++i)
      values[meshed.free_nodes()[i]] = rhs[i];
    out["t"] = assembled.time();
    out["label"] = "assembled right-hand side b";
    out["values"] = values_json(values);
    return out;
  }
  throw StageError("field", "input",
                   "field.kind: unknown field '" + kind +
                       "' (expression, initial_u, initial_u_t, dirichlet or "
                       "right_hand_side)");
}

Json Session::solve(const Json &case_json) {
  std::uint64_t id = 0;
  {
    const std::scoped_lock lock(mutex_);
    staged("case", [&] { check_case(case_json); });
    State &state = *state_;
    const auto &problem = state.get_meshed(case_json);
    (void)problem;
    const std::string key =
        state.meshed_key + "\n" + key_of(case_json, "solve");
    if (state.job && state.job->key == key && state.job->state != "failed") {
      if (state.job->state == "stopped") {
        state.job->state = "running";
        wake_.notify_all();
      }
    } else {
      // Dropped under the mutex: the run holds expressions.
      state.job.reset();
      state.job = state.make_job(case_json, options_);
      wake_.notify_all();
    }
    id = state.job->id;
    if (state.job->state != "running" || options_.background_runs)
      return progress_json(*state.job);
  }
  return finish_run(id);
}

Json Session::finish_run(std::uint64_t job) {
  while (advance_solve(job)) {
  }
  const std::scoped_lock lock(mutex_);
  const SolveJob &current = state_->job_of(job);
  if (current.state == "failed")
    throw job_error(current);
  return progress_json(current);
}

bool Session::advance_solve(std::uint64_t job) {
  const std::scoped_lock lock(mutex_);
  SolveJob &current = state_->job_of(job);
  advance(current, options_);
  return current.state == "running";
}

Json Session::solve_progress(std::uint64_t job) {
  const std::scoped_lock lock(mutex_);
  return progress_json(state_->job_of(job));
}

Json Session::stop_solve(std::uint64_t job) {
  const std::scoped_lock lock(mutex_);
  SolveJob &current = state_->job_of(job);
  if (current.state == "running")
    current.state = "stopped";
  return progress_json(current);
}

std::vector<std::vector<double>>
Session::solution_values(const Json &case_json) {
  const Json progress = solve(case_json);
  const auto id = progress["job"].get<std::uint64_t>();
  (void)finish_run(id);
  const std::scoped_lock lock(mutex_);
  std::vector<std::vector<double>> out;
  for (const solve::Solution &solution : state_->job_of(id).solutions)
    out.push_back(solution.values);
  return out;
}

std::string Session::solution_values_float32(std::uint64_t job,
                                             std::size_t first,
                                             std::size_t count) {
  const std::scoped_lock lock(mutex_);
  const SolveJob &current = state_->job_of(job);
  if (first > current.solutions.size() ||
      count > current.solutions.size() - first)
    throw StageError("solve", "input",
                     std::format("snapshots {} to {} are not ready ({} are)",
                                 first, first + count,
                                 current.solutions.size()));
  std::string out;
  if (count > 0)
    out.reserve(count * current.solutions[first].values.size() * 4);
  for (std::size_t k = first; k < first + count; ++k)
    append_float32(out, current.solutions[k].values);
  return out;
}

Json Session::apply_preset(const Json &request) {
  const std::scoped_lock lock(mutex_);
  staged("preset", [&] {
    require_object(request, "request");
    static constexpr std::string_view known[] = {"case", "preset"};
    reject_unknown_keys(request, "request", known);
  });
  const Json &case_json = member(request, "case");
  staged("case", [&] { check_case(case_json); });
  const std::string id = staged("preset", [&] {
    return require_string(member(request, "preset"), "preset");
  });
  const Domain &domain = state_->get_domain(case_json);
  return staged("preset", [&] {
    return app::apply_preset(case_json, id, domain_facts(domain));
  });
}

ExportedFile Session::export_file(const Json &case_json,
                                  std::string_view kind) {
  if (kind == "solution.csv") {
    // Solved to the end first, chunk by chunk (each under the mutex).
    const auto id = solve(case_json)["job"].get<std::uint64_t>();
    (void)finish_run(id);
    const std::scoped_lock lock(mutex_);
    const SolveJob &job = state_->job_of(id);
    return {.name = "solution.csv",
            .content_type = "text/csv",
            .content = solution_csv(*job.meshed->mesh(), job.solutions)};
  }
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  std::ostringstream out;
  if (kind == "geometry.svg") {
    const Domain &domain = state_->get_domain(case_json);
    if (!domain.geometry)
      throw StageError("export", "input",
                       "an imported mesh has no curve geometry to export");
    staged("export", [&] { io::export_svg(*domain.geometry, out); });
    return {.name = "geometry.svg",
            .content_type = "image/svg+xml",
            .content = out.str()};
  }
  if (kind == "stiffness.mtx" || kind == "mass.mtx" || kind == "damping.mtx" ||
      kind == "rhs.mtx") {
    const fem::AssembledProblem &assembled =
        state_->get_assembled(case_json, State::read_assembly(case_json));
    const std::size_t size = assembled.meshed_problem()->free_nodes().size();
    const std::string at = std::format("at t = {}", assembled.time());
    std::string content;
    if (kind == "rhs.mtx")
      content = matrix_market(assembled.right_hand_side(),
                              "pde2d: right-hand side b " + at);
    else {
      const fem::Operator &op = kind == "stiffness.mtx" ? assembled.stiffness()
                                : kind == "mass.mtx"    ? assembled.mass()
                                                        : assembled.damping();
      const std::string_view what = kind == "stiffness.mtx" ? "stiffness A"
                                    : kind == "mass.mtx"    ? "mass M2"
                                                            : "damping B";
      content = matrix_market(
          op.free, size, std::format("pde2d: {}, free part, {}", what, at));
    }
    return {.name = std::string(kind),
            .content_type = "text/plain",
            .content = std::move(content)};
  }
  const MeshOutcome &outcome = state_->get_mesh(case_json);
  if (kind == "mesh.pde2d") {
    staged("export", [&] { io::export_pde2d(*outcome.mesh, out); });
    return {.name = "mesh.pde2d",
            .content_type = "text/plain",
            .content = out.str()};
  }
  if (kind == "mesh.svg") {
    staged("export", [&] { io::export_mesh_svg(*outcome.mesh, out); });
    return {.name = "mesh.svg",
            .content_type = "image/svg+xml",
            .content = out.str()};
  }
  throw StageError("export", "input",
                   "unknown export '" + std::string(kind) +
                       "' (mesh.pde2d, mesh.svg, geometry.svg, "
                       "solution.csv, stiffness.mtx, mass.mtx, damping.mtx "
                       "or rhs.mtx)");
}

Json Session::run(const Json &case_json) {
  Json out = Json::object();
  out["ok"] = false;
  try {
    {
      Json domain = geometry(case_json);
      Json summary = Json::object();
      summary["kind"] = domain["kind"];
      summary["regions"] = domain["regions"];
      summary["domain_size"] = domain["domain_size"];
      out["geometry"] = std::move(summary);
    }
    {
      Json result = mesh(case_json);
      result.erase("mesh");
      out["mesh"] = std::move(result);
    }
    {
      Json check = check_problem(case_json);
      const bool ok = check["ok"].get<bool>();
      out["problem"] = std::move(check);
      if (!ok)
        throw StageError("problem", "input",
                         "the problem has errors (see problem.errors)");
    }
    {
      Json result = assemble(case_json);
      result["meshed_problem"].erase("dirichlet_nodes");
      result["assembly"].erase("sparsity");
      out["assembly"] = std::move(result);
    }
    {
      // The same chunks as the page's runs, run here to the end.
      Json solved = solve(case_json);
      if (solved["state"] == "running")
        solved = finish_run(solved["job"].get<std::uint64_t>());
      out["solve"] = std::move(solved);
    }
    out["ok"] = true;
  } catch (const StageError &error) {
    Json item = Json::object();
    item["stage"] = error.stage();
    item["kind"] = error.kind();
    item["message"] = error.what();
    out["error"] = std::move(item);
  }
  return out;
}

} // namespace pde2d::app

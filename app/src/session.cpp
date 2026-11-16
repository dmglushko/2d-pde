#include "session.hpp"

#include "catalog.hpp"
#include "domain_input.hpp"
#include "field_eval.hpp"
#include "presets.hpp"
#include "problem_input.hpp"
#include "stage_cache.hpp"
#include "views.hpp"
#include "binary.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/fem/compiled_expression.hpp"
#include "pde2d/fem/meshed_problem.hpp"

#include <chrono>
#include <cmath>
#include <format>
#include <utility>
#include <vector>

#ifndef PDE2D_VERSION
#define PDE2D_VERSION "unknown"
#endif

namespace pde2d::app {

namespace {

constexpr std::string_view case_format = "pde2d-app-case";
constexpr int case_version = 1;

const std::string &client_name(const std::string &client) {
  static const std::string local(local_client);
  return client.empty() ? local : client;
}

// The case's "assembly" section, read for the assembly stage.
AssemblySettings read_assembly(const Json &case_json) {
  return staged("assembly", [&] { return assembly_settings(case_json); });
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

Json values_json(const std::vector<double> &values) {
  Json out = Json::array();
  for (const double value : values)
    out.push_back(std::isfinite(value) ? Json(value) : Json(nullptr));
  return out;
}

// Reads a request of the solve stage, as a StageError when it refuses it.
template <class Read> auto solve_request(Read read) {
  return staged("solve", read);
}

} // namespace

// The stage cache and the clients' runs.
struct Session::State {
  StageCache cache;
  RunManager runs;
  explicit State(const SessionOptions &options) : runs(options.max_clients) {}
};

Session::Session(SessionOptions options)
    : options_(options), state_(std::make_unique<State>(options_)) {
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

// Runs every running run chunk by chunk, taking the clients in turn. The
// mutex is released between chunks, so other requests (a problem check, a
// field) are served then.
void Session::background_loop() {
  std::unique_lock lock(mutex_);
  while (!closing_) {
    if (Run *run = state_->runs.next_running()) {
      advance(*run, options_);
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
                [&] { return domain_json(state_->cache.domain(case_json)); });
}

Json Session::mesh(const Json &case_json) {
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  const auto outcome = state_->cache.mesh(case_json);
  Json out = Json::object();
  out["mesh"] = mesh_summary_json(*outcome->mesh, outcome->id);
  out["stats"] = mesh_stats_json(*outcome->mesh);
  out["target_met"] = outcome->target_met;
  out["longest_edge"] = outcome->longest_edge;
  out["max_edge_length"] = outcome->max_edge_length;
  out["domain_size"] = outcome->domain_size;
  out["reports"] = reports_json(outcome->reports);
  return out;
}

std::string Session::mesh_arrays(const Json &request) {
  const std::scoped_lock lock(mutex_);
  const double id = staged("mesh", [&] {
    require_object(request, "request");
    static constexpr std::string_view known[] = {"mesh"};
    reject_unknown_keys(request, "request", known);
    const double value = require_number(member(request, "mesh"), "mesh");
    if (value != std::floor(value) || value < 1 || value > 9e15)
      throw PathError("mesh", "expected a mesh's id");
    return value;
  });
  const auto mesh_id = static_cast<std::uint64_t>(id);
  std::shared_ptr<const mesh::Mesh> mesh;
  if (const auto outcome = state_->cache.find_mesh(mesh_id))
    mesh = outcome->mesh;
  else
    mesh = state_->runs.mesh_of(mesh_id);
  if (!mesh)
    throw StageError("mesh", "input",
                     std::format("mesh: the mesh {} is no longer kept (mesh "
                                 "again)",
                                 mesh_id),
                     "mesh");
  return staged("mesh", [&] { return mesh_arrays_binary(*mesh); });
}

Json Session::check_problem(const Json &case_json) {
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  return staged("problem", [&] {
    return app::check_problem(member(case_json, "problem"));
  });
}

Json Session::assemble(const Json &case_json) {
  const std::scoped_lock lock(mutex_);
  staged("case", [&] { check_case(case_json); });
  const AssemblySettings settings = read_assembly(case_json);
  const fem::AssembledProblem &assembled =
      state_->cache.assembled(case_json, settings);
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
    return value.is_null() ? assembly_settings(case_json).t
                           : require_number(value, "field.t");
  });

  Json out = Json::object();
  out["kind"] = kind;
  out["t"] = t;
  if (kind == "expression") {
    const auto outcome = state_->cache.mesh(case_json);
    const auto text = staged("field", [&] {
      return require_string(member(spec, "text"), "field.text");
    });
    const fem::CompiledExpression compiled = staged("field", [&] {
      const fem::Parameters parameters =
          read_parameters(member(member(case_json, "problem"), "parameters"));
      return at_path("field.text", [&] {
        return fem::CompiledExpression(fem::Expression(text), parameters);
      });
    });
    const std::vector<double> values =
        values_at_nodes(*outcome->mesh, compiled, t);
    out["label"] = text;
    out["values"] = values_json(values);
    return out;
  }
  if (kind == "initial_u" || kind == "initial_u_t") {
    const auto &meshed = state_->cache.meshed(case_json);
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
    const auto &meshed = state_->cache.meshed(case_json);
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
    AssemblySettings settings = read_assembly(case_json);
    settings.t = t;
    const fem::AssembledProblem &assembled =
        state_->cache.assembled(case_json, settings);
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
                       "right_hand_side)",
                   "field.kind");
}

// --- runs -------------------------------------------------------------------

Json Session::solve(const Json &case_json, const std::string &client) {
  const std::string &who = client_name(client);
  std::uint64_t id = 0;
  {
    const std::scoped_lock lock(mutex_);
    staged("case", [&] { check_case(case_json); });
    State &state = *state_;
    // Built (or found in the cache) first, so a refusal of an earlier
    // stage is thrown as that stage's.
    (void)state.cache.meshed(case_json);
    const std::string key = solve_key(case_json);
    RunManager::Client &owner = state.runs.client(who);
    if (owner.run && owner.run->key == key &&
        owner.run->state != RunState::failed) {
      if (owner.run->state == RunState::stopped) {
        owner.run->state = RunState::running;
        wake_.notify_all();
      }
    } else {
      // Dropped under the mutex: the run holds expressions.
      owner.run.reset();
      owner.run =
          make_run(state.cache, case_json, options_, state.runs.next_id());
      wake_.notify_all();
    }
    id = owner.run->id;
    if (owner.run->state != RunState::running || options_.background_runs)
      return progress_json(*owner.run);
  }
  return finish_run(id, who);
}

Json Session::finish_run(std::uint64_t job, const std::string &client) {
  while (advance_solve(job, client)) {
  }
  const std::scoped_lock lock(mutex_);
  const Run &current = state_->runs.run_of(client_name(client), job);
  if (current.state == RunState::failed)
    throw *current.error;
  return progress_json(current);
}

bool Session::advance_solve(std::uint64_t job, const std::string &client) {
  const std::scoped_lock lock(mutex_);
  Run &current = state_->runs.run_of(client_name(client), job);
  advance(current, options_);
  return current.state == RunState::running;
}

Json Session::solve_progress(std::uint64_t job, const std::string &client) {
  const std::scoped_lock lock(mutex_);
  return progress_json(state_->runs.run_of(client_name(client), job));
}

Json Session::stop_solve(std::uint64_t job, const std::string &client) {
  const std::scoped_lock lock(mutex_);
  Run &current = state_->runs.run_of(client_name(client), job);
  if (current.state == RunState::running)
    current.state = RunState::stopped;
  return progress_json(current);
}

std::vector<std::vector<double>>
Session::solution_values(const Json &case_json, const std::string &client) {
  const Json progress = solve(case_json, client);
  const auto id = progress["job"].get<std::uint64_t>();
  (void)finish_run(id, client_name(client));
  const std::scoped_lock lock(mutex_);
  std::vector<std::vector<double>> out;
  for (const solve::Solution &solution :
       *state_->runs.run_of(client_name(client), id).solutions)
    out.push_back(solution.values);
  return out;
}

std::string Session::solution_values_float32(std::uint64_t job,
                                             std::size_t first,
                                             std::size_t count,
                                             const std::string &client) {
  const std::scoped_lock lock(mutex_);
  const Run &current = state_->runs.run_of(client_name(client), job);
  const auto &solutions = *current.solutions;
  if (first > solutions.size() || count > solutions.size() - first)
    throw StageError("solve", "input",
                     std::format("snapshots {} to {} are not ready ({} are)",
                                 first, first + count, solutions.size()));
  std::string out;
  if (count > 0)
    out.reserve(count * solutions[first].values.size() * 4);
  for (std::size_t k = first; k < first + count; ++k)
    for (const double value : solutions[k].values)
      append_float32(out, value);
  return out;
}

Json Session::keep_reference(const Json &request, const std::string &client) {
  const std::scoped_lock lock(mutex_);
  static constexpr std::string_view known[] = {"job"};
  const auto job = solve_request([&] { return request_job(request, known); });
  const Run &run = state_->runs.run_of(client_name(client), job);
  RunManager::Client &owner = *state_->runs.find(client_name(client));
  if (run.state != RunState::done)
    throw StageError("solve", "not_ready",
                     std::format("the run {} is {}; only a run that is done "
                                 "can be the reference",
                                 job, state_name(run.state)));
  owner.reference = owner.run;
  Json reference = Json::object();
  reference["job"] = run.id;
  reference["mode"] = run.base["mode"];
  reference["mesh_id"] = run.mesh_id;
  Json times = Json::array();
  for (const solve::Solution &solution : *run.solutions)
    times.push_back(solution.time);
  reference["times"] = std::move(times);
  Json out = Json::object();
  out["reference"] = std::move(reference);
  return out;
}

Json Session::clear_reference(const std::string &client) {
  const std::scoped_lock lock(mutex_);
  // Dropped under the mutex: the run holds expressions.
  if (RunManager::Client *owner = state_->runs.find(client_name(client)))
    owner->reference.reset();
  Json out = Json::object();
  out["reference"] = nullptr;
  return out;
}

namespace {

constexpr std::string_view compare_keys[] = {"job", "exact", "reference",
                                             "snapshot"};

} // namespace

Json Session::compare(const Json &request, const std::string &client) {
  const std::scoped_lock lock(mutex_);
  const auto job = solve_request([&] {
    static constexpr std::string_view known[] = {"job", "exact", "reference"};
    return request_job(request, known);
  });
  const Run &run = state_->runs.run_of(client_name(client), job);
  auto reference = state_->runs.find(client_name(client))->reference;
  return staged("compare", [&] {
    return Comparison(run, request, std::move(reference)).summary();
  });
}

std::string Session::compare_difference(const Json &request,
                                        const std::string &client) {
  const std::scoped_lock lock(mutex_);
  const auto [job, snapshot] = solve_request([&] {
    return std::pair(request_job(request, compare_keys),
                     request_count(request, "snapshot"));
  });
  const Run &run = state_->runs.run_of(client_name(client), job);
  auto reference = state_->runs.find(client_name(client))->reference;
  return staged("compare", [&] {
    return Comparison(run, request, std::move(reference))
        .difference_float32(snapshot);
  });
}

Json Session::probes(const Json &request, const std::string &client) {
  const std::scoped_lock lock(mutex_);
  const auto job = solve_request([&] {
    static constexpr std::string_view known[] = {"job", "points"};
    return request_job(request, known);
  });
  const Run &run = state_->runs.run_of(client_name(client), job);
  return staged("probes",
                [&] { return probe_series(run, member(request, "points")); });
}

std::size_t Session::clients() {
  const std::scoped_lock lock(mutex_);
  return state_->runs.clients();
}

// --- presets and exports ----------------------------------------------------

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
  const Domain &domain = state_->cache.domain(case_json);
  return staged("preset", [&] {
    return app::apply_preset(case_json, id, domain_facts(domain));
  });
}

ExportedFile Session::export_file(const Json &case_json, std::string_view kind,
                                  const ExportContext &context) {
  const std::scoped_lock lock(mutex_);
  const ExportInfo *info = find_export(kind);
  if (info == nullptr)
    throw StageError("export", "input",
                     "unknown export '" + std::string(kind) + "' (" +
                         export_names() + ")");
  staged("case", [&] { check_case(case_json); });
  ExportInputs in;
  in.case_json = &case_json;
  // A run of this case, found by its inputs' text alone: an export never
  // starts or continues one.
  std::shared_ptr<const Run> run;
  if (info->needs == ExportNeeds::done_run ||
      info->needs == ExportNeeds::case_only) {
    const RunManager::Client *owner =
        state_->runs.find(client_name(context.client));
    if (owner != nullptr && owner->run &&
        owner->run->key == solve_key(case_json))
      run = owner->run;
    if (info->needs == ExportNeeds::done_run) {
      if (!run)
        throw StageError("export", "not_ready",
                         std::string(info->name) +
                             ": no run of this case; solve it first");
      if (run->state != RunState::done)
        throw StageError("export", "not_ready",
                         std::format("{}: the run {} is {}; a solution is "
                                     "exported once its run is done",
                                     info->name, run->id,
                                     state_name(run->state)));
    } else if (run && run->state != RunState::done)
      run.reset();
  }
  switch (info->needs) {
  case ExportNeeds::domain:
    in.domain = &state_->cache.domain(case_json);
    break;
  case ExportNeeds::mesh:
    in.mesh = state_->cache.mesh(case_json);
    break;
  case ExportNeeds::assembly:
    in.assembled =
        &state_->cache.assembled(case_json, read_assembly(case_json));
    break;
  case ExportNeeds::done_run:
  case ExportNeeds::case_only:
    break;
  }
  in.run = run;
  in.provenance =
      provenance_json(*info, case_json, context.case_name, run.get());
  return staged("export", [&] { return info->write(*info, in); });
}

// --- the command line --------------------------------------------------------

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
        solved = finish_run(solved["job"].get<std::uint64_t>(),
                            std::string(local_client));
      const Json job = solved["job"];
      out["solve"] = std::move(solved);
      // With an exact solution in the case (view.exact_solution), the
      // error of the run against it, for convergence studies.
      const Json &exact = member(member(case_json, "view"), "exact_solution");
      if (exact.is_string() && !exact.get<std::string>().empty()) {
        Json request = Json::object();
        request["job"] = job;
        request["exact"] = exact;
        Json comparison = compare(request);
        comparison.erase("snapshots");
        comparison["nodes"] = out["mesh"]["stats"]["nodes"];
        comparison["longest_edge"] = out["mesh"]["longest_edge"];
        out["comparison"] = std::move(comparison);
      }
    }
    out["ok"] = true;
  } catch (const StageError &error) {
    out["error"] = error_json(error);
  }
  return out;
}

} // namespace pde2d::app

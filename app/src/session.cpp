#include "session.hpp"

#include "binary.hpp"
#include "case_schema.hpp"
#include "catalog.hpp"
#include "domain_input.hpp"
#include "drawing.hpp"
#include "locate.hpp"
#include "field_eval.hpp"
#include "presets.hpp"
#include "problem_input.hpp"
#include "stage_cache.hpp"
#include "views.hpp"

#include "pde2d/fem/assembler.hpp"
#include "pde2d/fem/compiled_expression.hpp"
#include "pde2d/fem/meshed_problem.hpp"

#include <array>
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

const std::string &client_name(const std::string &client) {
  static const std::string local(local_client);
  return client.empty() ? local : client;
}

// The case's "assembly" section, read for the assembly stage.
AssemblySettings read_assembly(const Json &case_json) {
  return staged("assembly", [&] { return assembly_settings(case_json); });
}

// The case, checked and brought to the current version.
CaseInput read_case(const Json &case_json) {
  return staged("case", [&] { return CaseInput(case_json); });
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

// "27.4 MB" for messages.
std::string bytes_text(std::size_t bytes) {
  const auto value = static_cast<double>(bytes);
  if (value >= 1e9)
    return std::format("{:.2f} GB", value / 1e9);
  if (value >= 1e6)
    return std::format("{:.1f} MB", value / 1e6);
  return std::format("{:.0f} kB", value / 1e3);
}

// --- field kinds
// ---------------------------------------------------------------

// What a field request gives: the values at the nodes, their label, and
// the time when it is not the one asked for.
struct FieldValues {
  std::string label;
  std::vector<double> values;
  std::optional<double> t;
};

using FieldMaker = FieldValues (*)(StageCache &, const Json &case_json,
                                   const Json &spec, double t);

FieldValues expression_field(StageCache &cache, const Json &case_json,
                             const Json &spec, double t) {
  const auto outcome = cache.mesh(case_json);
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
  return {.label = text,
          .values = values_at_nodes(*outcome->mesh, compiled, t),
          .t = std::nullopt};
}

FieldValues initial_u_field(StageCache &cache, const Json &case_json,
                            const Json &, double t) {
  const auto meshed = cache.meshed(case_json);
  auto values = staged("field", [&] { return meshed->initial_values(t); });
  if (!values)
    throw StageError("field", "input",
                     "the problem has time order 0, so it has no initial "
                     "values");
  return {.label = "u at the start", .values = std::move(*values), .t = {}};
}

FieldValues initial_u_t_field(StageCache &cache, const Json &case_json,
                              const Json &, double t) {
  const auto meshed = cache.meshed(case_json);
  auto values = staged("field", [&] { return meshed->initial_velocities(t); });
  if (!values)
    throw StageError("field", "input",
                     "the problem has time order below 2, so it has no "
                     "initial velocities");
  return {.label = "u_t at the start", .values = std::move(*values), .t = {}};
}

FieldValues dirichlet_field(StageCache &cache, const Json &case_json,
                            const Json &, double t) {
  const auto meshed = cache.meshed(case_json);
  std::vector<double> values(meshed->mesh()->nodes.size(), std::nan(""));
  staged("field", [&] {
    for (std::size_t k = 0; k < meshed->dirichlet_nodes().size(); ++k)
      values[meshed->dirichlet_nodes()[k].node] = meshed->dirichlet_value(k, t);
  });
  return {.label = "Dirichlet values", .values = std::move(values), .t = {}};
}

FieldValues right_hand_side_field(StageCache &cache, const Json &case_json,
                                  const Json &, double t) {
  AssemblySettings settings = read_assembly(case_json);
  settings.t = t;
  const fem::AssembledProblem &assembled = cache.assembled(case_json, settings);
  const fem::MeshedProblem &meshed = *assembled.meshed_problem();
  std::vector<double> values(meshed.mesh()->nodes.size(), std::nan(""));
  const auto rhs = assembled.right_hand_side();
  for (std::size_t i = 0; i < rhs.size(); ++i)
    values[meshed.free_nodes()[i]] = rhs[i];
  return {.label = "assembled right-hand side b",
          .values = std::move(values),
          .t = assembled.time()};
}

// Every field the page can show, by its "kind".
constexpr std::array<std::pair<std::string_view, FieldMaker>, 5> field_kinds{{
    {"expression", expression_field},
    {"initial_u", initial_u_field},
    {"initial_u_t", initial_u_t_field},
    {"dirichlet", dirichlet_field},
    {"right_hand_side", right_hand_side_field},
}};

std::string field_kind_names() {
  std::string out;
  for (const auto &[name, maker] : field_kinds)
    out += (out.empty() ? "" : ", ") + std::string(name);
  return out;
}

} // namespace

// The stage cache and the clients' runs.
struct Session::State {
  StageCache cache;
  RunManager runs; // the run lock
  explicit State(const SessionOptions &options) : runs(options.max_clients) {}
};

// Holds the session mutex for one call and says what it is doing
// (status); on release it destroys the runs dropped meanwhile, after the
// run lock and before the session mutex is released (they hold
// expressions).
class Session::Work {
public:
  Work(Session &session, std::string_view task, const std::string &client,
       std::uint64_t job = 0)
      : session_(session), lock_(session.mutex_) {
    const std::scoped_lock book(session_.run_lock_);
    session_.activity_ = Activity{.task = std::string(task),
                                  .client = client_name(client),
                                  .job = job,
                                  .since = std::chrono::system_clock::now(),
                                  .started = std::chrono::steady_clock::now()};
  }
  ~Work() {
    std::vector<std::shared_ptr<const Run>> dropped;
    {
      const std::scoped_lock book(session_.run_lock_);
      session_.activity_.reset();
      dropped = session_.state_->runs.take_dropped();
    }
  }
  Work(const Work &) = delete;
  Work &operator=(const Work &) = delete;

private:
  Session &session_;
  std::unique_lock<std::mutex> lock_;
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
    Run *run = nullptr;
    {
      const std::scoped_lock book(run_lock_);
      run = state_->runs.next_running();
      if (run != nullptr)
        activity_ = Activity{.task = "run",
                             .client = run->client,
                             .job = run->id,
                             .since = std::chrono::system_clock::now(),
                             .started = std::chrono::steady_clock::now()};
    }
    if (run == nullptr) {
      wake_.wait(lock);
      continue;
    }
    advance(*run, options_, run_lock_);
    {
      std::vector<std::shared_ptr<const Run>> dropped;
      {
        const std::scoped_lock book(run_lock_);
        activity_.reset();
        dropped = state_->runs.take_dropped();
      }
    }
    lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    lock.lock();
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

Json Session::case_schema() { return app::case_schema(); }

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

// --- stages -----------------------------------------------------------------

Json Session::geometry(const Json &input, const std::string &client) {
  const Work work(*this, "geometry", client);
  const CaseInput read = read_case(input);
  const Json &case_json = read.json();
  return staged("geometry",
                [&] { return domain_json(state_->cache.domain(case_json)); });
}

Json Session::locate(const Json &request, const std::string &client) {
  const Work work(*this, "geometry", client);
  const geometry::Point point = staged("geometry", [&] {
    require_object(request, "request");
    static constexpr std::string_view known[] = {"case", "point", "snap"};
    reject_unknown_keys(request, "request", known);
    const Json &at = member(request, "point");
    if (!at.is_array() || at.size() != 2)
      throw PathError("point", "expected a point [x, y]");
    return geometry::Point{require_number(at[0], "point[0]"),
                           require_number(at[1], "point[1]")};
  });
  const double snap = staged("geometry", [&] {
    const Json &value = member(request, "snap");
    const double out = value.is_null() ? 0.0 : require_number(value, "snap");
    if (out < 0)
      throw PathError("snap", "must not be negative");
    return out;
  });
  const CaseInput read = read_case(member(request, "case"));
  const Json &case_json = read.json();
  const Domain &domain = state_->cache.domain(case_json);
  return staged("geometry",
                [&] { return locate_boundary(domain, point, snap); });
}

Json Session::to_drawing(const Json &input, const std::string &client) {
  const Work work(*this, "geometry", client);
  const CaseInput read = read_case(input);
  const Json &case_json = read.json();
  const Domain &domain = state_->cache.domain(case_json);
  return staged("geometry", [&] {
    if (!domain.geometry)
      throw PathError("geometry.source",
                      "an imported mesh has no curves to edit as a drawing");
    Json out = Json::object();
    out["geometry"] = drawing_section(*domain.geometry);
    return out;
  });
}

Json Session::mesh(const Json &input, const std::string &client) {
  const Work work(*this, "mesh", client);
  const CaseInput read = read_case(input);
  const Json &case_json = read.json();
  const auto outcome = state_->cache.mesh(case_json);
  Json out = Json::object();
  out["mesh"] = mesh_summary_json(*outcome->mesh, outcome->id);
  out["stats"] = mesh_stats_json(*outcome->mesh);
  out["target_met"] = outcome->target_met;
  out["longest_edge"] = outcome->longest_edge;
  out["max_edge_length"] = outcome->max_edge_length;
  out["domain_size"] = outcome->domain_size;
  out["reports"] = reports_json(outcome->reports);
  // What a run on this mesh keeps per snapshot, for a warning before a
  // solve: about snapshot_bytes * (snapshots + 1) of the budget.
  Json memory = Json::object();
  memory["snapshot_bytes"] = run_bytes(outcome->mesh->nodes.size(), 1, 0) -
                             run_bytes(outcome->mesh->nodes.size(), 0, 0);
  memory["budget"] = options_.memory_budget;
  out["memory"] = std::move(memory);
  return out;
}

std::string Session::mesh_arrays(const Json &request,
                                 const std::string &client) {
  const Work work(*this, "mesh arrays", client);
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
  else {
    const std::scoped_lock book(run_lock_);
    mesh = state_->runs.mesh_of(mesh_id);
  }
  if (!mesh)
    throw StageError("mesh", "input",
                     std::format("mesh: the mesh {} is no longer kept (mesh "
                                 "again)",
                                 mesh_id),
                     "mesh");
  return staged("mesh", [&] { return mesh_arrays_binary(*mesh); });
}

Json Session::check_problem(const Json &input, const std::string &client) {
  const Work work(*this, "problem check", client);
  const CaseInput read = read_case(input);
  const Json &case_json = read.json();
  return staged("problem", [&] {
    return app::check_problem(member(case_json, "problem"));
  });
}

Json Session::assemble(const Json &input, const std::string &client) {
  const Work work(*this, "assembly", client);
  const CaseInput read = read_case(input);
  const Json &case_json = read.json();
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

Json Session::field(const Json &request, const std::string &client) {
  const Work work(*this, "field", client);
  staged("field", [&] {
    require_object(request, "request");
    static constexpr std::string_view known[] = {"case", "field"};
    reject_unknown_keys(request, "request", known);
  });
  const CaseInput read = read_case(member(request, "case"));
  const Json &case_json = read.json();
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
  const auto found = std::ranges::find(
      field_kinds, kind, [](const auto &item) { return item.first; });
  if (found == field_kinds.end())
    throw StageError("field", "input",
                     "field.kind: unknown field '" + kind + "' (" +
                         field_kind_names() + ")",
                     "field.kind");
  FieldValues field = found->second(state_->cache, case_json, spec, t);
  Json out = Json::object();
  out["kind"] = kind;
  out["t"] = field.t.value_or(t);
  out["label"] = std::move(field.label);
  out["values"] = values_json(field.values);
  return out;
}

// --- runs -------------------------------------------------------------------

Json Session::solve(const Json &input, const std::string &client) {
  const std::string &who = client_name(client);
  std::uint64_t id = 0;
  {
    const Work work(*this, "solve", who);
    const CaseInput read = read_case(input);
    const Json &case_json = read.json();
    State &state = *state_;
    // Built (or found in the cache) first, so a refusal of an earlier
    // stage is thrown as that stage's.
    (void)state.cache.meshed(case_json);
    const std::string key = solve_key(case_json);
    {
      const std::scoped_lock book(run_lock_);
      RunManager::Client &owner = state.runs.client(who);
      if (owner.run && owner.run->key == key &&
          owner.run->published.state != RunState::failed) {
        Run &current = *owner.run;
        if (current.published.state == RunState::stopped) {
          current.published.state = RunState::running;
          wake_.notify_all();
        }
        if (current.published.state != RunState::running ||
            options_.background_runs)
          return progress_json(current, options_.memory_budget);
        id = current.id;
      }
    }
    if (id == 0) {
      std::uint64_t next = 0;
      {
        const std::scoped_lock book(run_lock_);
        next = state.runs.next_id();
      }
      const std::shared_ptr<Run> made =
          make_run(state.cache, case_json, next, who);
      {
        const std::scoped_lock book(run_lock_);
        if (!state.runs.make_room(who, made->bytes, options_.memory_budget)) {
          const std::size_t room =
              state.runs.room_for(who, options_.memory_budget);
          const RunManager::Client *owner = state.runs.find(who);
          const bool has_reference = owner != nullptr && owner->reference;
          const bool in_time = made->setup.mode == SolveMode::time_integration;
          const char *budget = made->setup.mode == SolveMode::adaptive
                                   ? "solve.adaptive.max_nodes"
                               : made->setup.mode == SolveMode::adaptive_time
                                   ? "solve.adaptive_time.max_nodes"
                                   : "";
          throw StageError(
              "solve", "memory",
              std::format(
                  "{}this run would keep {} snapshot{} of {} nodes ({}), but "
                  "only {} of the server's memory budget for runs ({}) is "
                  "free for it{}. {}use a coarser mesh{}, or start the "
                  "server with a larger --memory-budget",
                  in_time ? "solve.time.snapshots: " : "", made->times.size(),
                  made->times.size() == 1 ? "" : "s", made->mesh->nodes.size(),
                  bytes_text(made->bytes), bytes_text(room),
                  bytes_text(options_.memory_budget),
                  has_reference ? " (your reference run and other pages' "
                                  "running runs hold the rest)"
                                : " (other pages' running runs hold the rest)",
                  in_time ? "Keep fewer snapshots, " : "",
                  has_reference ? ", forget the reference run" : ""),
              in_time ? "solve.time.snapshots" : budget);
        }
        state.runs.set_run(state.runs.client(who), made);
      }
      start_run(*made, options_, run_lock_);
      wake_.notify_all();
      const std::scoped_lock book(run_lock_);
      if (made->published.state != RunState::running ||
          options_.background_runs)
        return progress_json(*made, options_.memory_budget);
      id = made->id;
    }
  }
  return finish_run(id, who);
}

Json Session::finish_run(std::uint64_t job, const std::string &client) {
  while (advance_solve(job, client)) {
  }
  const std::scoped_lock book(run_lock_);
  const Run &current = state_->runs.run_of(client_name(client), job);
  if (current.published.state == RunState::failed)
    throw *current.published.error;
  return progress_json(current, options_.memory_budget);
}

bool Session::advance_solve(std::uint64_t job, const std::string &client) {
  const Work work(*this, "run", client, job);
  Run *current = nullptr;
  {
    const std::scoped_lock book(run_lock_);
    current = &state_->runs.run_of(client_name(client), job);
  }
  // Not removed meanwhile: only a holder of the session mutex drops runs.
  advance(*current, options_, run_lock_);
  const std::scoped_lock book(run_lock_);
  return current->published.state == RunState::running;
}

Json Session::solve_progress(std::uint64_t job, const std::string &client) {
  const std::scoped_lock book(run_lock_);
  return progress_json(state_->runs.run_of(client_name(client), job),
                       options_.memory_budget);
}

Json Session::current_run(const Json &input, const std::string &client) {
  // Reading the case is JSON only (the keys and migrations): no library.
  const CaseInput read = read_case(input);
  const std::string key = solve_key(read.json());
  Json out = Json::object();
  out["run"] = nullptr;
  const std::scoped_lock book(run_lock_);
  const RunManager::Client *owner = state_->runs.find(client_name(client));
  if (owner != nullptr && owner->run && owner->run->key == key)
    out["run"] = progress_json(*owner->run, options_.memory_budget);
  return out;
}

Json Session::stop_solve(std::uint64_t job, const std::string &client) {
  const std::scoped_lock book(run_lock_);
  Run &current = state_->runs.run_of(client_name(client), job);
  if (current.published.state == RunState::running)
    current.published.state = RunState::stopped;
  return progress_json(current, options_.memory_budget);
}

RunSnapshots Session::snapshots_of(std::uint64_t job,
                                   const std::string &client) {
  const std::scoped_lock book(run_lock_);
  return state_->runs.run_of(client_name(client), job).snapshots();
}

std::vector<std::vector<double>>
Session::solution_values(const Json &case_json, const std::string &client) {
  const Json progress = solve(case_json, client);
  const auto id = progress["job"].get<std::uint64_t>();
  (void)finish_run(id, client_name(client));
  std::vector<std::vector<double>> out;
  for (const Snapshot &solution : snapshots_of(id, client).snapshots)
    out.push_back(solution->values);
  return out;
}

std::string Session::solution_values_float32(std::uint64_t job,
                                             std::size_t first,
                                             std::size_t count,
                                             const std::string &client) {
  const RunSnapshots run = snapshots_of(job, client);
  const auto &snapshots = run.snapshots;
  if (first > snapshots.size() || count > snapshots.size() - first)
    throw StageError("solve", "input",
                     std::format("snapshots {} to {} are not ready ({} are)",
                                 first, first + count, snapshots.size()));
  std::string out;
  if (count > 0)
    out.reserve(count * snapshots[first]->values.size() * 4);
  for (std::size_t k = first; k < first + count; ++k)
    for (const double value : snapshots[k]->values)
      append_float32(out, value);
  return out;
}

std::string Session::indicators_float32(std::uint64_t job,
                                        const std::string &client) {
  const RunSnapshots run = snapshots_of(job, client);
  if (!run.indicators)
    throw StageError("solve", "not_ready",
                     std::format("the run {} has no error indicators (only "
                                 "a finished adaptive run has them)",
                                 job));
  std::string out;
  out.reserve(run.indicators->size() * 4);
  for (const double value : *run.indicators)
    append_float32(out, value);
  return out;
}

Json Session::keep_reference(const Json &request, const std::string &client) {
  const Work work(*this, "reference", client);
  static constexpr std::string_view known[] = {"job"};
  const auto job = solve_request([&] { return request_job(request, known); });
  const std::scoped_lock book(run_lock_);
  const Run &run = state_->runs.run_of(client_name(client), job);
  RunManager::Client &owner = *state_->runs.find(client_name(client));
  if (run.published.state != RunState::done)
    throw StageError("solve", "not_ready",
                     std::format("the run {} is {}; only a run that is done "
                                 "can be the reference",
                                 job, state_name(run.published.state)));
  state_->runs.set_reference(owner, owner.run);
  Json reference = Json::object();
  reference["job"] = run.id;
  reference["mode"] = run.base["mode"];
  reference["mesh_id"] = run.mesh_id;
  Json times = Json::array();
  for (const Snapshot &solution : run.published.snapshots)
    times.push_back(solution->time);
  reference["times"] = std::move(times);
  Json out = Json::object();
  out["reference"] = std::move(reference);
  return out;
}

Json Session::clear_reference(const std::string &client) {
  const Work work(*this, "reference", client);
  {
    const std::scoped_lock book(run_lock_);
    if (RunManager::Client *owner = state_->runs.find(client_name(client)))
      state_->runs.set_reference(*owner, nullptr);
  }
  Json out = Json::object();
  out["reference"] = nullptr;
  return out;
}

namespace {

constexpr std::string_view compare_keys[] = {"job", "exact", "reference",
                                             "snapshot"};

} // namespace

Session::ComparedRuns Session::compared_runs(const Json &request,
                                             std::uint64_t job,
                                             const std::string &client) {
  const std::scoped_lock book(run_lock_);
  const Run &current = state_->runs.run_of(client_name(client), job);
  ComparedRuns out{.run = current.snapshots(),
                   .reference = std::nullopt,
                   .parameters = current.parameters};
  if (!member(request, "reference").is_null())
    if (const auto kept = state_->runs.reference_of(client_name(client)))
      out.reference = kept->snapshots();
  return out;
}

Json Session::compare(const Json &request, const std::string &client) {
  const Work work(*this, "comparison", client);
  const auto job = solve_request([&] {
    static constexpr std::string_view known[] = {"job", "exact", "reference"};
    return request_job(request, known);
  });
  ComparedRuns runs = compared_runs(request, job, client);
  return staged("compare", [&] {
    return Comparison(std::move(runs.run), runs.parameters, request,
                      std::move(runs.reference))
        .summary();
  });
}

std::string Session::compare_difference(const Json &request,
                                        const std::string &client) {
  const Work work(*this, "comparison", client);
  const auto [job, snapshot] = solve_request([&] {
    return std::pair(request_job(request, compare_keys),
                     request_count(request, "snapshot"));
  });
  ComparedRuns runs = compared_runs(request, job, client);
  return staged("compare", [&] {
    return Comparison(std::move(runs.run), runs.parameters, request,
                      std::move(runs.reference))
        .difference_float32(snapshot);
  });
}

Json Session::probes(const Json &request, const std::string &client) {
  const auto job = solve_request([&] {
    static constexpr std::string_view known[] = {"job", "points"};
    return request_job(request, known);
  });
  // The mesh and the values only: no lock while interpolating.
  const RunSnapshots run = snapshots_of(job, client);
  return staged("probes",
                [&] { return probe_series(run, member(request, "points")); });
}

Json Session::status(const std::string &client) {
  const std::string &who = client_name(client);
  const std::scoped_lock book(run_lock_);
  Json out = Json::object();
  if (activity_) {
    Json busy = Json::object();
    busy["task"] = activity_->task;
    busy["client"] = activity_->client;
    busy["yours"] = activity_->client == who;
    busy["job"] = activity_->job == 0 ? Json(nullptr) : Json(activity_->job);
    busy["since"] = utc_time(activity_->since);
    busy["seconds"] = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - activity_->started)
                          .count();
    out["busy"] = std::move(busy);
  } else
    out["busy"] = nullptr;
  out["running"] = state_->runs.running();
  out["clients"] = state_->runs.clients();
  Json memory = Json::object();
  memory["budget"] = options_.memory_budget;
  memory["used"] = state_->runs.used_bytes();
  memory["room"] = state_->runs.room_for(who, options_.memory_budget);
  out["memory"] = std::move(memory);
  return out;
}

std::size_t Session::clients() {
  const std::scoped_lock book(run_lock_);
  return state_->runs.clients();
}

// --- presets and exports ----------------------------------------------------

Json Session::apply_preset(const Json &request, const std::string &client) {
  const Work work(*this, "preset", client);
  staged("preset", [&] {
    require_object(request, "request");
    static constexpr std::string_view known[] = {"case", "preset"};
    reject_unknown_keys(request, "request", known);
  });
  const CaseInput read = read_case(member(request, "case"));
  const Json &case_json = read.json();
  const std::string id = staged("preset", [&] {
    return require_string(member(request, "preset"), "preset");
  });
  const Domain &domain = state_->cache.domain(case_json);
  return staged("preset", [&] {
    return app::apply_preset(case_json, id, domain_facts(domain));
  });
}

ExportedFile Session::export_file(const Json &input, std::string_view kind,
                                  const ExportContext &context) {
  const Work work(*this, "export", context.client);
  const ExportInfo *info = find_export(kind);
  if (info == nullptr)
    throw StageError("export", "input",
                     "unknown export '" + std::string(kind) + "' (" +
                         export_names() + ")");
  const CaseInput read = read_case(input);
  const Json &case_json = read.json();
  ExportInputs in;
  in.case_json = &case_json;
  // A run of this case, found by its inputs' text alone: an export never
  // starts or continues one.
  Json run_provenance = nullptr;
  if (info->needs == ExportNeeds::done_run ||
      info->needs == ExportNeeds::case_only) {
    const std::scoped_lock book(run_lock_);
    const RunManager::Client *owner =
        state_->runs.find(client_name(context.client));
    const Run *run = owner != nullptr && owner->run &&
                             owner->run->key == solve_key(case_json)
                         ? owner->run.get()
                         : nullptr;
    const RunState state =
        run != nullptr ? run->published.state : RunState::failed;
    if (info->needs == ExportNeeds::done_run) {
      if (run == nullptr)
        throw StageError("export", "not_ready",
                         std::string(info->name) +
                             ": no run of this case; solve it first");
      if (state != RunState::done)
        throw StageError("export", "not_ready",
                         std::format("{}: the run {} is {}; a solution is "
                                     "exported once its run is done",
                                     info->name, run->id, state_name(state)));
    }
    if (run != nullptr && state == RunState::done) {
      in.run = run->snapshots();
      in.run_summary = progress_json(*run, options_.memory_budget);
      in.run_summary.erase("values");
      in.run_summary["started"] = utc_time(run->started);
      run_provenance = Json::object();
      run_provenance["id"] = run->id;
      run_provenance["state"] = state_name(state);
      run_provenance["started"] = utc_time(run->started);
    }
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
  in.provenance =
      provenance_json(*info, case_json, context.case_name, run_provenance);
  return staged("export", [&] { return info->write(*info, in); });
}

// --- the command line --------------------------------------------------------

Json Session::run(const Json &input) {
  Json out = Json::object();
  out["ok"] = false;
  try {
    const CaseInput read = read_case(input);
    const Json &case_json = read.json();
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
        // The error at the last snapshot (t1 of a run in time), besides
        // the largest over the snapshots.
        if (!comparison["snapshots"].empty())
          comparison["final"] = comparison["snapshots"].back();
        comparison.erase("snapshots");
        // An adaptive run's mesh is its last one (in time: the last
        // snapshot's): the comparison's own node count, and no longest
        // edge of the Mesh stage.
        const Json &mode = out["solve"]["mode"];
        if (mode != "adaptive" && mode != "adaptive_time") {
          comparison["nodes"] = out["mesh"]["stats"]["nodes"];
          comparison["longest_edge"] = out["mesh"]["longest_edge"];
        }
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

#pragma once

#include "app_error.hpp"
#include "case_format.hpp"
#include "exports.hpp"
#include "options.hpp"
#include "runs.hpp"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

// The app's work, independent of HTTP: each call takes a case (the whole
// setup as JSON, see docs/app.md) and returns JSON for the page. The
// server and the command line (--run) both use it.
//
// A Session keeps the results of the last call of each stage (the
// StageCache), keyed by the sections they were built from, so meshing
// again after a change to the problem reuses the mesh, and the runs of
// each client (the RunManager): one page is one client, with its current
// run and its reference run.
//
// Two locks (docs/app.md, "Locks"). The session mutex serializes every
// call that uses the library: the library's symbolic work must not run in
// two threads at once (SymEngine's reference counts are not thread-safe in
// this build), and a run holds expressions. The run lock is short: it
// guards the runs' bookkeeping (states, kept snapshots, clients) and what
// the session is busy with, so progress, stop, values, probes and status
// never wait for a chunk, a mesh or a solve.
namespace pde2d::app {

// The client of calls that name none (the command line, tests).
inline constexpr std::string_view local_client = "local";

// What an export is for: whose run, and the case's file name (for the
// provenance; may be empty).
struct ExportContext {
  std::string client{local_client};
  std::string case_name;
};

class Session {
public:
  explicit Session(SessionOptions options = {});
  ~Session();
  Session(const Session &) = delete;
  Session &operator=(const Session &) = delete;

  // Everything the page builds its forms from (docs/app.md, "The
  // catalog").
  static Json catalog();
  // A complete case to start from.
  static Json default_case();
  // A JSON Schema of the case, generated from the catalog and the section
  // readers' keys (case_schema.hpp).
  static Json case_schema();

  // Each throws StageError. Every call reads only the sections it needs.
  // `client` only says whose call it is (status).
  Json geometry(const Json &case_json, const std::string &client = {});
  // Where on the boundary a point is, for splitting a region there
  // (locate.hpp, locate_boundary). request: {"case", "point": [x, y],
  // "snap": distance (0 by default)}.
  Json locate(const Json &request, const std::string &client = {});
  // The case's curve geometry (after its region edits) as a drawing
  // section, curve for curve (drawing.hpp, drawing_section):
  // {"geometry": section}. Refused for a mesh file.
  Json to_drawing(const Json &case_json, const std::string &client = {});
  // The mesh's summary (views.hpp, mesh_summary_json, under "mesh"), its
  // stats and reports; the arrays come from mesh_arrays.
  Json mesh(const Json &case_json, const std::string &client = {});
  // The arrays of a mesh this session made (views.hpp,
  // mesh_arrays_binary). request: {"mesh": id}.
  std::string mesh_arrays(const Json &request, const std::string &client = {});
  Json check_problem(const Json &case_json, const std::string &client = {});
  Json assemble(const Json &case_json, const std::string &client = {});
  // request: {"case": ..., "field": {"kind": ..., ...}}; see docs/app.md.
  Json field(const Json &request, const std::string &client = {});

  // A static solve or a time integration of the client's, as the case's
  // "solve" section says; see docs/app.md. Every solve is a run with an id
  // ("job"); a static one is done at once. A time run with the same
  // inputs as the client's current run reuses it (a stopped one
  // continues, a failed one starts again); other inputs replace it. The
  // first chunk runs before this returns, so the library's refusals are
  // thrown here. With background_runs the run then continues in the
  // background; without, it runs to the end (and throws when a later chunk
  // fails). Returns the progress (solve_progress).
  Json solve(const Json &case_json, const std::string &client = {});
  // The client's current run's summary so far (runs.hpp, progress_json).
  // Throws StageError for a run that is not the client's current one
  // (kind "replaced" or "expired", RunManager::run_of). The run lock only.
  Json solve_progress(std::uint64_t job, const std::string &client = {});
  // The client's current run of the case (the same geometry, mesh,
  // problem and solve sections), as {"run": progress} or {"run": null}
  // when the client has none (or a run of other inputs). Never starts or
  // continues a run and never makes a client: a page that was reloaded
  // finds its run again with it. The run lock only.
  Json current_run(const Json &case_json, const std::string &client = {});
  // Stops the run after the chunk that is running; solve() with the same
  // inputs continues it. Returns its progress. The run lock only.
  Json stop_solve(std::uint64_t job, const std::string &client = {});
  // Runs one chunk of the run; false once it is no longer running. For
  // the command line and tests (the background thread does the same).
  bool advance_solve(std::uint64_t job, const std::string &client = {});
  // Snapshots [first, first + count) of the run as float32, little-endian,
  // snapshot after snapshot (what POST /api/solve/values sends). The run
  // lock only.
  std::string solution_values_float32(std::uint64_t job, std::size_t first,
                                      std::size_t count,
                                      const std::string &client = {});
  // The error indicators of an adaptive run's last cycle as float32, one
  // per triangle of its mesh (what POST /api/solve/indicators sends). The
  // run lock only.
  std::string indicators_float32(std::uint64_t job,
                                 const std::string &client = {});
  // The solution of an adaptive run's cycle `cycle` (from 1) as float32,
  // one per node of that cycle's mesh (what POST /api/solve/cycle sends).
  // The run lock only.
  std::string cycle_values_float32(std::uint64_t job, std::size_t cycle,
                                   const std::string &client = {});
  // Solves the case to the end and returns every kept solution's values,
  // one vector per time (one value per mesh node), as computed.
  std::vector<std::vector<double>>
  solution_values(const Json &case_json, const std::string &client = {});

  // Keeps the client's run {"job"} (done) as its reference run; returns
  // {"reference": {"job", "mode", "times", "mesh_id"}}.
  Json keep_reference(const Json &request, const std::string &client = {});
  // Forgets the client's reference run; {"reference": null}.
  Json clear_reference(const std::string &client = {});
  // The run {"job"} against {"exact": text} or {"reference": true}, in
  // double: per snapshot max |u - other|, nodal RMS and lumped L2
  // (field_eval.hpp, Comparison::summary).
  Json compare(const Json &request, const std::string &client = {});
  // u - other at {"snapshot": k} of the same request, float32.
  std::string compare_difference(const Json &request,
                                 const std::string &client = {});
  // u(t) of the run {"job"} at {"points": [[x, y], ...]}. The run lock
  // only.
  Json probes(const Json &request, const std::string &client = {});

  // What the session is busy with, for the page ("server busy: meshing
  // (12 s)"): {"busy": {"task", "client", "yours", "job", "since",
  // "seconds"} or null, "running" (runs), "clients", "memory": {"budget",
  // "used", "room" (what a new run of `client` may keep)}}. The run lock
  // only.
  Json status(const std::string &client = {});

  // The case with a complete problem of the preset `preset` for its
  // geometry (problem and solve sections replaced; see presets.hpp).
  // request: {"case": ..., "preset": id}.
  Json apply_preset(const Json &request, const std::string &client = {});
  // A file of the kind `kind` (exports.hpp, export_kinds). A solution
  // export needs the client's run of this case to be done: it never
  // computes one (StageError of kind "not_ready" otherwise).
  ExportedFile export_file(const Json &case_json, std::string_view kind,
                           const ExportContext &context = {});

  // Runs every stage the case allows, for the command line: geometry,
  // mesh, problem check, assembly, solve, and with view.exact_solution in
  // the case, the comparison with it ("comparison": max_abs, max_rms,
  // max_l2 over the snapshots, nodes, longest_edge). Stops at the first
  // failing stage and reports it under "error". "ok" is true when every
  // stage ran.
  Json run(const Json &case_json);

  // How many clients the session keeps now.
  std::size_t clients();
  const SessionOptions &options() const { return options_; }

private:
  struct State;
  class Work;
  // What holds the session mutex now (under the run lock).
  struct Activity {
    std::string task;
    std::string client;
    std::uint64_t job = 0;
    std::chrono::system_clock::time_point since;
    std::chrono::steady_clock::time_point started;
  };
  // Runs the client's run to its end, chunk by chunk (each under the
  // mutex); throws its error when it failed. Returns its progress.
  Json finish_run(std::uint64_t job, const std::string &client);
  void background_loop();
  // The run's kept snapshots (the run lock taken here).
  RunSnapshots snapshots_of(std::uint64_t job, const std::string &client);
  // What a comparison of the run `job` reads: its snapshots, the
  // reference's when the request asks for it, and its parameters.
  struct ComparedRuns {
    RunSnapshots run;
    std::optional<RunSnapshots> reference;
    fem::Parameters parameters;
  };
  ComparedRuns compared_runs(const Json &request, std::uint64_t job,
                             const std::string &client);

  SessionOptions options_;
  std::mutex mutex_;    // the session mutex: library work
  std::mutex run_lock_; // the run lock: bookkeeping
  std::unique_ptr<State> state_;
  std::optional<Activity> activity_;
  // The background runner (with background_runs): wakes when a run starts.
  std::condition_variable wake_;
  bool closing_ = false;
  std::thread worker_;
};

} // namespace pde2d::app

#pragma once

#include "options.hpp"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

// The app's work, independent of HTTP: each call takes a case (the whole
// setup as JSON, see docs/app.md) and returns JSON for the page. The
// server and the command line (--run) both use it.
//
// A Session keeps the results of the last call of each stage, keyed by the
// sections they were built from, so meshing again after a change to the
// problem reuses the mesh. Calls are serialized by one mutex: the library's
// symbolic work must not run in two threads at once (SymEngine's
// reference counts are not thread-safe in this build), and a local app has
// one user.
namespace pde2d::app {

// A stage failed. `kind` is "input" for std::invalid_argument (the
// library refused the input or the settings), "algorithm" for
// std::runtime_error (an algorithm could not finish) and "internal" for
// anything else.
class StageError : public std::runtime_error {
public:
  StageError(std::string stage, std::string kind, const std::string &message)
      : std::runtime_error(message), stage_(std::move(stage)),
        kind_(std::move(kind)) {}
  const std::string &stage() const { return stage_; }
  const std::string &kind() const { return kind_; }

private:
  std::string stage_;
  std::string kind_;
};

struct ExportedFile {
  std::string name;
  std::string content_type;
  std::string content;
};

// How a Session runs time integrations (docs/app.md, "Runs in chunks").
struct SessionOptions {
  // Run time integrations in a background thread: solve() returns after
  // the first chunk and the run continues, one chunk at a time under the
  // session mutex. Off (the command line, tests): solve() runs every chunk
  // before it returns.
  bool background_runs = false;
  // What a chunk should take; the next chunk's number of snapshot
  // intervals is scaled towards it. 0 keeps every chunk one interval.
  double chunk_seconds = 0.4;
  // Snapshot intervals in the first chunk.
  std::size_t first_chunk = 1;
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

  // Each throws StageError. Every call reads only the sections it needs.
  Json geometry(const Json &case_json);
  Json mesh(const Json &case_json);
  Json check_problem(const Json &case_json);
  Json assemble(const Json &case_json);
  // request: {"case": ..., "field": {"kind": ..., ...}}; see docs/app.md.
  Json field(const Json &request);
  // A static solve or a time integration, as the case's "solve" section
  // says; see docs/app.md. Every solve is a run with an id ("job"); a
  // static one is done at once. A time run with the same inputs as the
  // current run reuses it (a stopped one continues, a failed one starts
  // again); other inputs replace it. The first chunk runs before this
  // returns, so the library's refusals are thrown here. With
  // background_runs the run then continues in the background; without, it
  // runs to the end (and throws when a later chunk fails). Returns the
  // progress (solve_progress).
  Json solve(const Json &case_json);
  // The current run's summary so far: "job", "state" (running, done,
  // failed or stopped), the times and range of the snapshots ready, the
  // combined reports, "t" reached, "error" for a failed run, and "values"
  // (where to get them, how many snapshots are ready). Throws StageError
  // for a run that is not the current one.
  Json solve_progress(std::uint64_t job);
  // Stops the run after the chunk that is running; solve() with the same
  // inputs continues it. Returns its progress.
  Json stop_solve(std::uint64_t job);
  // Runs one chunk of the run; false once it is no longer running. For
  // the command line and tests (the background thread does the same).
  bool advance_solve(std::uint64_t job);
  // Snapshots [first, first + count) of the run as float32, little-endian,
  // snapshot after snapshot (what POST /api/solve/values sends).
  std::string solution_values_float32(std::uint64_t job, std::size_t first,
                                      std::size_t count);
  // Solves the case to the end and returns every kept solution's values,
  // one vector per time (one value per mesh node), as computed.
  std::vector<std::vector<double>> solution_values(const Json &case_json);
  // The case with a complete problem of the preset `preset` for its
  // geometry (problem and solve sections replaced; see presets.hpp).
  // request: {"case": ..., "preset": id}.
  Json apply_preset(const Json &request);
  // kind: "mesh.pde2d", "mesh.svg", "geometry.svg", "solution.csv", or the
  // assembled system at assembly.t in Matrix Market form:
  // "stiffness.mtx", "mass.mtx", "damping.mtx" (free parts) and "rhs.mtx".
  ExportedFile export_file(const Json &case_json, std::string_view kind);

  // Runs every stage the case allows, for the command line: geometry,
  // mesh, problem check, assembly, solve. Stops at the first failing stage and
  // reports it under "error". "ok" is true when every stage ran.
  Json run(const Json &case_json);

private:
  struct State;
  // Runs the current run to its end, chunk by chunk (each under the
  // mutex); throws its error when it failed. Returns its progress.
  Json finish_run(std::uint64_t job);
  void background_loop();

  SessionOptions options_;
  std::mutex mutex_;
  std::unique_ptr<State> state_;
  // The background runner (with background_runs): wakes when a run starts.
  std::condition_variable wake_;
  bool closing_ = false;
  std::thread worker_;
};

// Throws std::invalid_argument for a case that is not an object, has a
// key that is not a section, or names another format or a newer version.
void check_case(const Json &case_json);

} // namespace pde2d::app

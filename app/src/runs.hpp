#pragma once

#include "app_error.hpp"
#include "catalog.hpp"
#include "options.hpp"
#include "stage_cache.hpp"

#include "pde2d/fem/expression.hpp"
#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/solve/solution.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// Solves as runs: a static one is done when it is made; a time
// integration runs in chunks that end on snapshot times, each continuing
// from the state the previous one ended with (Integrator::integrate with
// `initial`), so the page sees the snapshots while the run goes on
// (docs/app.md, "Runs"). Runs belong to clients (one page each). Not
// thread-safe: the Session calls everything here under its mutex.
namespace pde2d::app {

// How a Session runs time integrations.
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
  // The most clients kept; a new one drops the least recently used (and
  // stops its run).
  std::size_t max_clients = 8;
};

enum class RunState { running, done, failed, stopped };
std::string_view state_name(RunState state);

struct Run {
  std::uint64_t id = 0;
  std::string key; // solve_key of the case it was made from
  RunState state = RunState::running;
  std::shared_ptr<const fem::MeshedProblem> meshed;
  std::uint64_t mesh_id = 0;  // the StageCache's id of its mesh
  fem::Parameters parameters; // the problem's, for comparisons
  Json settings;              // the sections it was made from
  std::chrono::system_clock::time_point started;
  SolveSetup setup;
  // The snapshot times of a time run; the kept solutions are the first
  // solutions->size() of them. Shared so an export can stream a done
  // run's values without the session mutex (Solution holds no
  // expressions).
  std::vector<double> times;
  std::shared_ptr<std::vector<solve::Solution>> solutions =
      std::make_shared<std::vector<solve::Solution>>();
  // The state at the end of the last chunk (with velocities for time
  // order 2), the start of the next one.
  std::optional<solve::Solution> last;
  std::size_t chunk = 1; // snapshot intervals in the next chunk
  std::size_t chunks = 0;
  Json base = Json::object(); // mode, time orders, free unknowns
  Json static_reports = Json::array();
  std::optional<mesh::StageReport> integrator; // the chunks', combined
  double low = 0.0;
  double high = 0.0;
  bool has_range = false;
  std::optional<StageError> error; // of a failed run
};

// A new run of the case: a static one is solved, a time one has run its
// first chunk. Throws the StageError of either.
std::shared_ptr<Run> make_run(StageCache &cache, const Json &case_json,
                              const SessionOptions &options, std::uint64_t id);

// Runs the next chunk of a running time run. A failure ends the run with
// its error. The next chunk is scaled so that it takes about
// chunk_seconds.
void advance(Run &run, const SessionOptions &options);

// The run's summary so far (docs/app.md, "The API": POST /api/solve).
Json progress_json(const Run &run);

// The runs of every client: its current run and its reference run.
class RunManager {
public:
  explicit RunManager(std::size_t max_clients) : max_clients_(max_clients) {}

  struct Client {
    std::shared_ptr<Run> run;
    std::shared_ptr<const Run> reference;
    std::uint64_t used = 0;
  };

  // The client `id`, made when it is new (dropping the least recently
  // used client beyond max_clients); marks it used. Only a solve makes a
  // client.
  Client &client(const std::string &id);
  // The client `id` if it is kept (marked used), or null.
  Client *find(const std::string &id);
  // The client's current run if it is `job`, else a StageError.
  Run &run_of(const std::string &client, std::uint64_t job);
  // A running run after the one advanced last (round robin over clients),
  // or null.
  Run *next_running();
  // A run of any client that holds `mesh_id`'s mesh, or null.
  std::shared_ptr<const mesh::Mesh> mesh_of(std::uint64_t mesh_id) const;
  std::size_t clients() const { return clients_.size(); }
  std::uint64_t next_id() { return next_run_++; }

private:
  std::size_t max_clients_;
  std::map<std::string, Client, std::less<>> clients_;
  std::uint64_t clock_ = 0;
  std::uint64_t next_run_ = 1;
  std::string last_advanced_;
};

} // namespace pde2d::app

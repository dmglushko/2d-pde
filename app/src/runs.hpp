#pragma once

#include "app_error.hpp"
#include "catalog.hpp"
#include "options.hpp"
#include "stage_cache.hpp"

#include "pde2d/fem/expression.hpp"
#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/mesh/report.hpp"
#include "pde2d/solve/integrator.hpp"
#include "pde2d/solve/solution.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Solves as runs: a static one is done when it is started, and so is an
// adaptive one (the library's AdaptiveStaticSolver, one call; its run's
// mesh is the last adapted mesh, under an id of its own) and an adaptive
// one in time (the library's AdaptiveIntegrator, one call; each snapshot is
// on its piece's mesh, Solution::mesh, every mesh under an id); a time
// integration runs in chunks that end on snapshot times, each continuing
// from the state the previous one ended with (Integrator::integrate with
// `initial`), so the page sees the snapshots while the run goes on
// (docs/app.md, "Runs"). Runs belong to clients (one page each).
//
// Two locks guard a run (docs/app.md, "Locks"). The session mutex guards
// everything that touches the library: building a run, running a chunk,
// dropping a run (it holds expressions). The run lock (RunLock, the
// Session's short lock) guards what readers see: the RunManager's clients
// and every Run's `published` part. A chunk computes without the run lock
// and takes it only to publish what it made. Lock order: the session
// mutex first, then the run lock; never the other way round.
namespace pde2d::app {

// How a Session runs time integrations and how much it keeps.
struct SessionOptions {
  // Run time integrations in a background thread: solve() returns after
  // the first chunk and the run continues, one chunk at a time under the
  // session mutex. Off (the command line, tests): solve() runs every chunk
  // before it returns.
  bool background_runs = false;
  // Chunk planning (plan_chunk): a chunk should take at least about
  // chunk_seconds (fewer, longer chunks cost less), its fixed cost (the
  // preparation of the linear solver and the assembly at its start) at
  // most max_fixed_share of it, and it at most about max_chunk_seconds
  // (progress and Stop answer between chunks). chunk_seconds 0 keeps every
  // chunk one interval.
  double chunk_seconds = 0.4;
  double max_chunk_seconds = 3.0;
  double max_fixed_share = 0.2;
  // Snapshot intervals in the first chunk.
  std::size_t first_chunk = 1;
  // The most clients kept; a new one drops the least recently used (and
  // stops its run).
  std::size_t max_clients = 8;
  // The bytes every kept run together may hold (their snapshots); a run
  // that does not fit even after dropping other clients' finished runs is
  // refused. 0 (the default): no budget, every run is kept.
  std::size_t memory_budget = 0;
};

enum class RunState { running, done, failed, stopped };
std::string_view state_name(RunState state);

using Snapshot = std::shared_ptr<const solve::Solution>;

// A run's kept snapshots at one moment, for readers outside the session
// mutex: the mesh and the values hold no expressions, and a snapshot never
// changes once it is kept.
struct RunSnapshots {
  std::uint64_t id = 0;
  std::shared_ptr<const mesh::Mesh> mesh;
  std::vector<Snapshot> snapshots;
  // An adaptive run's solution of every cycle, each on its own mesh (the
  // last is snapshots[0]); empty for other runs.
  std::vector<Snapshot> cycles;
  // An adaptive run's last indicators (one per triangle of `mesh`), or
  // null.
  std::shared_ptr<const std::vector<double>> indicators;
};

// What one chunk cost: its snapshot intervals, its steps and its seconds
// (wall clock, the whole integrate call).
struct ChunkCost {
  std::size_t intervals = 0;
  double steps = 0.0;
  double seconds = 0.0;
};

// A chunk's cost as fixed + per_step * steps, fitted by least squares to
// the chunks so far. `fitted` is false while they do not tell the two
// apart (fewer than two different step counts).
struct ChunkCostModel {
  double fixed = 0.0;
  double per_step = 0.0;
  bool fitted = false;
};
ChunkCostModel fit_chunk_cost(std::span<const ChunkCost> history);

// The snapshot intervals of the next chunk (at least 1), from the chunks
// so far (docs/app.md, "Chunk sizes").
std::size_t plan_chunk(std::span<const ChunkCost> history,
                       const SessionOptions &options);

// How one metric of the integrator's report combines over chunks.
// min_measured: the smallest value above 0, 0 when no chunk has one (0
// means "not measured", as for the integrator's min_pivot_ratio).
// max keeps a NaN (from any chunk); min_measured ignores 0 ("not
// measured").
enum class Combine { sum, max, min, min_measured, first, last };
struct MetricRule {
  std::string_view name;
  Combine combine;
  // Only chunks that took steps count (min_dt, max_dt).
  bool needs_steps = false;
};
// Every metric the integrator reports, with its rule.
std::span<const MetricRule> integrator_metric_rules();
// Adds one chunk's report to the run's total (`total` empty: the first).
// Throws std::logic_error for a metric without a rule.
void combine_report(std::optional<mesh::StageReport> &total,
                    const mesh::StageReport &part);

struct Run {
  // Set when the run is made; read without a lock.
  std::uint64_t id = 0;
  std::string client;
  std::string key; // solve_key of the case it was made from
  // An adaptive run replaces meshed, mesh, mesh_id and base's
  // free_unknowns once, under the run lock, when it is solved (its last
  // problem, its last mesh and adapted_mesh_id); readers take the run lock.
  std::shared_ptr<const fem::MeshedProblem> meshed; // session mutex only
  std::shared_ptr<const mesh::Mesh> mesh;           // meshed's mesh
  std::uint64_t mesh_id = 0; // the StageCache's id of its mesh
  // An adaptive run in time: the first of max_refinements ids reserved for
  // its new meshes (an adaptive run takes its cycles' ids when it is
  // published).
  std::uint64_t adapted_mesh_id = 0;
  fem::Parameters parameters; // the problem's, for comparisons
  Json settings;              // the sections it was made from
  std::chrono::system_clock::time_point started;
  SolveSetup setup;
  // The snapshot times of a time run (one time for a static solve).
  std::vector<double> times;
  Json base = Json::object(); // mode, time orders, free unknowns
  // The bytes its snapshots will hold when it is done (run_bytes).
  std::size_t bytes = 0;

  // The computing side: the session mutex only.
  // The state at the end of the last chunk (with velocities for time
  // order 2), the start of the next one.
  std::optional<solve::Solution> last;
  // Where its steps stand (the integrator's anchor), so that the next
  // chunk continues the stretch: an error-controlled step goes on with
  // its proposed dt and the method's earlier steps.
  std::optional<solve::StepAnchor> anchor;
  std::size_t chunk = 1; // snapshot intervals in the next chunk
  std::vector<ChunkCost> costs;

  // What readers see: the run lock.
  struct Published {
    RunState state = RunState::running;
    std::vector<Snapshot> snapshots;
    // Per snapshot, the smallest and largest finite value (NaN when it has
    // none).
    std::vector<double> minima;
    std::vector<double> maxima;
    double t = 0.0; // reached
    std::size_t chunks = 0;
    Json static_reports = Json::array();
    std::optional<mesh::StageReport> integrator; // the chunks', combined
    std::optional<StageError> error;             // of a failed run
    // An adaptive run: every cycle's solution, the indicators of the last
    // cycle's estimate (one per triangle of the last mesh, empty without
    // an estimate), and the summary sent as "adaptive" (stop reason,
    // tolerance, a row per cycle).
    std::vector<Snapshot> cycles;
    std::shared_ptr<const std::vector<double>> indicators;
    Json adaptive; // also an adaptive run in time's summary
    // An adaptive run in time: every mesh its snapshots are on, with its
    // id, in the order of use (the first is the starting mesh). An
    // adaptive run: each cycle's mesh, with its id, one per cycle (the
    // first is the starting mesh, the last the run's mesh).
    std::vector<std::pair<std::uint64_t, std::shared_ptr<const mesh::Mesh>>>
        meshes;
    // RunManager's clock at its last use (a lookup marks it).
    mutable std::uint64_t used = 0;
    // Whether the worker computes it now (a chunk, or an adaptive loop).
    bool computing = false;
    // A run asked to stop while it is computed (request_stop): it stays
    // running until the chunk or the loop has stopped (at its next step or
    // after its current cycle) and published what it has, then it is
    // stopped; so a reader that sees "stopped" sees all of it.
    bool stop_requested = false;
  } published;

  // The run lock held.
  RunSnapshots snapshots() const;
};

// The bytes a run of `snapshots` snapshots on `nodes` nodes keeps: the
// values of each (velocities are not kept) and the state the next chunk
// starts from.
std::size_t run_bytes(std::size_t nodes, std::size_t snapshots, int time_order);

// The most an adaptive run keeps: at most `max_cycles` solutions and
// meshes of at most `max_nodes` nodes each (values, nodes, about two
// triangles a node, boundary edges and the indicators).
std::size_t adaptive_run_bytes(std::size_t max_nodes, std::size_t max_cycles);

// The most an adaptive run in time keeps: `snapshots` snapshots and
// `meshes` meshes of at most `max_nodes` nodes each, and its state.
std::size_t adaptive_time_run_bytes(std::size_t max_nodes,
                                    std::size_t snapshots, std::size_t meshes,
                                    int time_order);

// A new run of the case, ready to start: its setup, times and bytes are
// known, nothing is solved. Throws the StageError of a stage it builds.
std::shared_ptr<Run> make_run(StageCache &cache, const Json &case_json,
                              std::uint64_t id, const std::string &client);

// Solves a static or adaptive run, or runs a time run's first chunk. Throws the
// StageError of either (a failed first chunk is also recorded in the
// run). Under the session mutex; takes `run_lock` to publish.
// An adaptive run takes ids for its cycles' meshes from `cache`.
void start_run(Run &run, const SessionOptions &options, std::mutex &run_lock,
               StageCache &cache);

// Runs the next chunk of a running time run, or the whole loop of an
// adaptive run (whose cycles, or time, are published as they come). A
// failure ends the run with its error. Under the session mutex; takes
// `run_lock` to publish; an adaptive run takes ids for its cycles' meshes
// from `cache`.
void advance(Run &run, const SessionOptions &options, std::mutex &run_lock,
             StageCache &cache);

// Asks a running run to stop: at once when it is not being computed, else
// once its chunk (at its next step) or its adaptive loop (after its
// current cycle or step) has stopped and published what it has. The run
// lock held.
void request_stop(Run &run);

// The run's summary so far (docs/app.md, "The API": POST /api/solve). The
// run lock held.
Json progress_json(const Run &run, std::size_t memory_budget);

// A memory budget as the answers send it: its bytes, or null for none (0).
Json budget_json(std::size_t memory_budget);

// Why a run is no longer a client's current run.
enum class GoneReason { replaced, memory_budget, client_limit };

// The runs of every client: its current run and its reference run. Its
// methods need the run lock; those that drop runs need the session mutex
// too, and keep the dropped runs until take_dropped (to be destroyed
// after the run lock is released, still under the session mutex).
class RunManager {
public:
  explicit RunManager(std::size_t max_clients) : max_clients_(max_clients) {}

  struct Client {
    std::shared_ptr<Run> run;
    std::shared_ptr<const Run> reference;
    // Why the reference run went, when it was dropped (not forgotten).
    std::optional<GoneReason> reference_gone;
    std::uint64_t used = 0;
  };

  // The client `id`, made when it is new (dropping the least recently
  // used client beyond max_clients); marks it used. Only a solve makes a
  // client.
  Client &client(const std::string &id);
  // The client `id` if it is kept (marked used), or null.
  Client *find(const std::string &id);
  // The client's current run if it is `job`, else a StageError: kind
  // "replaced" for a run a newer solve of the client replaced, "expired"
  // for one that was dropped (memory budget, client limit) or that this
  // server never had (it was restarted).
  Run &run_of(const std::string &client, std::uint64_t job);
  // The client's reference run, or null; a StageError ("expired") when it
  // was dropped.
  std::shared_ptr<const Run> reference_of(const std::string &client);
  // Replaces the client's current run (records why the old one went).
  void set_run(Client &owner, std::shared_ptr<Run> run);
  void set_reference(Client &owner, std::shared_ptr<const Run> reference);
  // A running run after the one advanced last (round robin over clients),
  // or null.
  Run *next_running();
  std::size_t running() const;
  // A run of any client that holds `mesh_id`'s mesh (its own, or one of
  // an adaptive run in time's), or null.
  std::shared_ptr<const mesh::Mesh> mesh_of(std::uint64_t mesh_id) const;
  std::size_t clients() const { return clients_.size(); }
  std::uint64_t next_id() { return next_run_++; }

  // The bytes every kept run reserves (each run counted once).
  std::size_t used_bytes() const;
  // The bytes a new run of `client` may have: the budget less what the
  // runs that cannot be dropped for it reserve (running runs, its
  // reference run, other clients' references and current runs that are
  // running). Its own current run is not counted (a solve replaces it)
  // unless it is also its reference.
  std::size_t room_for(const std::string &client, std::size_t budget) const;
  // Drops other clients' finished runs, least recently used first, until
  // `bytes` fit beside what stays; false (nothing dropped) when they
  // cannot fit even then.
  bool make_room(const std::string &client, std::size_t bytes,
                 std::size_t budget);

  // The runs dropped since the last call.
  std::vector<std::shared_ptr<const Run>> take_dropped();

private:
  void forget(const std::shared_ptr<const Run> &run, GoneReason reason);
  void touch(const Run &run);

  struct Gone {
    std::string client;
    GoneReason reason;
  };
  std::size_t max_clients_;
  std::map<std::string, Client, std::less<>> clients_;
  // Runs that are gone, by id (the last max_gone).
  std::map<std::uint64_t, Gone> gone_;
  static constexpr std::size_t max_gone = 4096;
  std::vector<std::shared_ptr<const Run>> dropped_;
  std::uint64_t clock_ = 0;
  std::uint64_t next_run_ = 1;
  std::string last_advanced_;
};

} // namespace pde2d::app

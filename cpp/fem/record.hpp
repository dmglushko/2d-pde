#pragma once
// Reproducibility record of one top-level result (StationarySolution,
// Evolution): the options that ran, on which inputs, with which library and
// how much work. Structured fields only, so two runs can be compared field
// by field; write_report (report.hpp) prints it and write_json
// (record_json.hpp) serializes it (versioned schema, diff_records compares
// two records).
//
//   auto s = solve_stationary(mesh, problem);
//   s.record.solver;                 // the SolverOptions that ran
//   s.record.work.solver_iterations; // work of the solve
//   attach_mesh_record(s.record, mesh_result);  // where the mesh came from
//   write_json(out, s.record);       // record_json.hpp
//
// Inputs are identified, not stored: the mesh by mesh_fingerprint, the
// problem by problem_fingerprint, initial fields and guesses by
// field_fingerprint (fingerprint.hpp). The options are the ones the run
// used, so a record replays a run with built-in strategies:
//
//   solve_stationary(mesh, problem, record.assembly, record.solver)
//
// A user linear solver backend (SolverOptions::backend) or time integrator
// (TimeOptions::integrator) is a factory and cannot be rebuilt from a
// record: it is recorded by its name() with user_supplied set, and
// replayable() is false.
//
// Not recorded: per-step timings (in the timing structs; only totals are
// here, under `timings`, the one field that differs between identical
// runs) and actual OpenMP team sizes (only the selected policy is known).
#include "discretization/mesh_types.hpp"
#include "execution.hpp"
#include "fem/assembly.hpp"
#include "fem/linear_solver_types.hpp"
#include "fem/time_integrator.hpp"
#include "provenance.hpp"
#include <Eigen/Core>
#include <symengine/symengine_config.h>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>

namespace pde2d::fem {
// A strategy that ran: its report name and whether the user supplied it as
// a factory (then the record names it but cannot rebuild it).
struct StrategyRecord {
  std::string name{};
  bool user_supplied = false;
  bool operator==(const StrategyRecord &) const = default;
};

struct WorkloadRecord {
  std::size_t nodes = 0, elements = 0, boundary_edges = 0;
};
// Work counters of the run. Exact counts are performance contracts; see
// EvolutionTimings and StationaryStatistics for their meaning.
struct WorkRecord {
  std::size_t assembly_calls = 0, linear_solves = 0, solver_preparations = 0,
              solver_reuses = 0, time_steps = 0;
  // Iterations of the linear solves, summed (SolverReport::iterations;
  // direct backends report their own count, SparseLU 1).
  std::size_t solver_iterations = 0;
};

// The library (pde2d::library_version) and the numerical dependencies it
// was compiled against.
struct VersionRecord {
  pde2d::LibraryVersion library = pde2d::library_version();
  std::string eigen = std::to_string(EIGEN_WORLD_VERSION) + '.' +
                      std::to_string(EIGEN_MAJOR_VERSION) + '.' +
                      std::to_string(EIGEN_MINOR_VERSION);
  std::string symengine = SYMENGINE_VERSION;
};

// A fingerprinted field (field_kind, field_fingerprint).
struct FieldRecord {
  std::string kind{}, fingerprint{};
};
struct InitialRecord {
  FieldRecord value{};
  std::optional<FieldRecord> velocity{};
};
// A stationary solve: the time its coefficients were evaluated at, the
// initial guess it was given (field_fingerprint of the nodal vector as
// passed, before its Dirichlet rows are set) and whether the backend used
// it, and how the linear solve ended (SolverReport).
struct StationaryRecord {
  double time = 0;
  InitialGuess initial_guess = InitialGuess::None;
  std::optional<FieldRecord> guess{};
  int iterations = 0;
  bool converged = false;
  double residual_norm = 0, relative_residual = 0;
};
// The time settings of an evolution (TimeOptions without its callables).
struct TimeRecord {
  // TimeOptions::method; ignored when a user integrator ran.
  TimeMethod method = TimeMethod::BackwardEuler;
  // The integrator that ran (TimeIntegrator::name()); user_supplied when it
  // came from TimeOptions::integrator.
  StrategyRecord integrator{};
  double start = 0, end = 0, step = 0;
  std::size_t frame_stride = 1;
  bool store_reports = true;
  // An observer was set (it cannot be recorded).
  bool observer = false;
};

struct TimingRecord {
  // Wall-clock seconds of the solve or evolution and of its assembly and
  // solver phases. Never reproducible; diff_records ignores them by default.
  double total_seconds = 0, assembly_seconds = 0, solver_seconds = 0;
};

struct RunRecord {
  // The options that ran (assembly.execution is the selected execution
  // policy, not observed OpenMP team sizes).
  AssemblyOptions assembly{};
  SolverOptions solver{};
  // linear_solver_name(solver); user_supplied for SolverOptions::backend.
  StrategyRecord linear_solver{};
  // Evolutions only.
  std::optional<TimeRecord> time{};
  // The library was compiled with OpenMP (pde2d::openmp_available()).
  bool openmp_available = false;
  WorkloadRecord workload{};
  WorkRecord work{};
  // Stationary solves only.
  std::optional<StationaryRecord> stationary{};
  // Evolutions only.
  std::optional<InitialRecord> initial{};
  std::string mesh_fingerprint{};    // discretization::mesh_fingerprint
  std::string problem_fingerprint{}; // fingerprint.hpp
  // The meshing call that produced the mesh, when attached
  // (attach_mesh_record).
  std::optional<discretization::MeshRunRecord> mesh_run{};
  VersionRecord versions{};
  TimingRecord timings{};

  // Every strategy is built in: the record's options alone rerun it.
  bool replayable() const {
    return !linear_solver.user_supplied &&
           !(time && time->integrator.user_supplied);
  }
};

// Attach the record of the meshing call that produced the mesh of `record`.
// Throws std::invalid_argument ("Mesh run record describes another mesh")
// when the fingerprints differ, for example for a hand-built MeshResult or a
// mesh edited after meshing.
inline void attach_mesh_record(RunRecord &record,
                               const discretization::MeshRunRecord &mesh) {
  if (mesh.fingerprint != record.mesh_fingerprint)
    throw std::invalid_argument("Mesh run record describes another mesh");
  record.mesh_run = mesh;
}
inline void attach_mesh_record(RunRecord &record,
                               const discretization::MeshResult &result) {
  attach_mesh_record(record, result.record);
}

namespace record_detail {
inline StrategyRecord linear_solver(const SolverOptions &options) {
  return {linear_solver_name(options), static_cast<bool>(options.backend)};
}
inline WorkloadRecord workload(const discretization::TriangleMesh &mesh) {
  return {mesh.nodes.size(), mesh.elements.size(), mesh.boundary_edges.size()};
}
inline std::size_t iterations(const SolverReport &report) {
  return report.iterations > 0 ? static_cast<std::size_t>(report.iterations)
                               : 0;
}
} // namespace record_detail
} // namespace pde2d::fem

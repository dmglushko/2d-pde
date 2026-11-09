#pragma once
// Reproducibility record of one top-level result (StationarySolution,
// Evolution): what ran, on which inputs, with which library, with how much
// work, and how well the choices were supported. Structured fields only, so
// two runs can be compared field by field; write_report (report.hpp) prints
// it and write_json (record_json.hpp) serializes it (versioned schema,
// diff_records compares two records).
//
// Inputs are identified, not stored: the mesh by mesh_fingerprint (plus
// its MeshOptions and MeshReport when the run was given a MeshResult), the
// problem by problem_fingerprint, initial fields by field_fingerprint
// (fingerprint.hpp). Strategy factories (std::function objects and backend
// factories) cannot be replayed from a record: `strategies` names what ran
// and says per strategy whether the options alone rebuild it ("built-in"),
// a descriptor describes it ("described") or nothing does ("not
// replayable").
//
// Not recorded: per-step timings (in the timing structs; only the total
// wall-clock time is here, under `timings`, the one field that differs
// between identical runs) and actual OpenMP team sizes (only the selected
// policy is known).
#include "discretization/mesh_fingerprint.hpp"
#include "discretization/mesh_types.hpp"
#include "execution.hpp"
#include "fem/decision.hpp"
#include "fem/evidence.hpp"
#include "fem/plan.hpp"
#include "provenance.hpp"
#include <Eigen/Core>
#include <symengine/symengine_config.h>
#include <algorithm>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
// Counts of the evidence behind a list of decisions.
struct EvidenceSummary {
  std::size_t proven = 0, sampled = 0, unknown = 0;
  // Weakest evidence of any decision (Proven when there are none).
  Evidence weakest = Evidence::Proven;
};
inline EvidenceSummary summarize_evidence(const std::vector<Decision> &list) {
  EvidenceSummary summary;
  for (const auto &d : list) {
    switch (d.evidence) {
    case Evidence::Proven:
      ++summary.proven;
      break;
    case Evidence::Sampled:
      ++summary.sampled;
      break;
    case Evidence::Unknown:
      ++summary.unknown;
      break;
    }
    if (static_cast<int>(d.evidence) > static_cast<int>(summary.weakest))
      summary.weakest = d.evidence;
  }
  return summary;
}

struct ExecutionRecord {
  // Selected policy (SolvePlan::assembly.execution), not observed team sizes.
  pde2d::ExecutionOptions selected{};
  // The library was compiled with OpenMP (pde2d::openmp_available()).
  bool openmp_available = false;
};
struct WorkloadRecord {
  std::size_t nodes = 0, elements = 0, boundary_edges = 0;
};
// Work counters of the run. Exact counts are performance contracts; see
// EvolutionTimings and StationaryStatistics for their meaning.
struct WorkRecord {
  std::size_t assembly_calls = 0, linear_solves = 0, solver_preparations = 0,
              solver_reuses = 0, time_steps = 0, rejected_steps = 0;
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

// Where a mesh came from, when the run was given a MeshResult.
struct MeshProvenance {
  discretization::MeshOptions options{};
  discretization::MeshReport report{};
  // The result kept its boundary context (refinable along the curves).
  bool boundary_context = false;
};
inline MeshProvenance mesh_provenance(const discretization::MeshResult &r) {
  return {r.options, r.report, r.boundary_context.has_value()};
}
struct MeshRecord {
  std::string fingerprint{}; // discretization::mesh_fingerprint
  std::optional<MeshProvenance> provenance{};
};

// A fingerprinted initial field (field_kind, field_fingerprint).
struct FieldRecord {
  std::string kind{}, fingerprint{};
};
struct InitialRecord {
  FieldRecord value{};
  std::optional<FieldRecord> velocity{};
};

// Whether a record rebuilds a strategy: from the options alone (built-in),
// from a descriptor only a reader can turn back into an object (described),
// or not at all (not replayable: a factory without a descriptor).
enum class StrategyReplay { BuiltIn, Described, NotReplayable };
inline const char *strategy_replay_name(StrategyReplay replay) {
  switch (replay) {
  case StrategyReplay::BuiltIn:
    return "built-in";
  case StrategyReplay::Described:
    return "described";
  case StrategyReplay::NotReplayable:
    return "not replayable";
  }
  return "invalid";
}
// One strategy that ran: the decision subject it serves (time integrator,
// time steps, linear solver, size field), its name() and how to rebuild it.
struct StrategyRecord {
  DecisionSubject subject{};
  std::string name{};
  StrategyReplay replay = StrategyReplay::BuiltIn;
  std::optional<pde2d::StrategyDescriptor> descriptor{};
};
inline StrategyRecord
strategy_record(DecisionSubject subject, std::string name, bool builtin,
                std::optional<pde2d::StrategyDescriptor> descriptor) {
  const auto replay = builtin      ? StrategyReplay::BuiltIn
                      : descriptor ? StrategyReplay::Described
                                   : StrategyReplay::NotReplayable;
  return {subject, std::move(name), replay, std::move(descriptor)};
}

struct TimingRecord {
  // Wall-clock seconds of the solve or evolution. Never reproducible;
  // diff_records ignores it by default.
  double total_seconds = 0;
};

struct RunRecord {
  // Settings that ran; plan.decisions explain them, plan.mesh and
  // plan.analysis say what they were based on. Strategy factories in the
  // plan are not comparable; `strategies` names what they created.
  SolvePlan plan{};
  ExecutionRecord execution{};
  WorkloadRecord workload{};
  WorkRecord work{};
  EvidenceSummary evidence{};
  VersionRecord versions{};
  MeshRecord mesh{};
  std::string problem_fingerprint{}; // fingerprint.hpp
  // The time a stationary solve evaluated its coefficients at.
  std::optional<double> stationary_time{};
  // Initial fields of an evolution.
  std::optional<InitialRecord> initial{};
  // In DecisionSubject order.
  std::vector<StrategyRecord> strategies{};
  TimingRecord timings{};

  // Every strategy is built in: the record's options alone rerun it.
  bool replayable() const {
    return std::ranges::all_of(strategies, [](const StrategyRecord &s) {
      return s.replay == StrategyReplay::BuiltIn;
    });
  }
};

// Insert or replace the strategy of `strategy.subject`, in subject order.
inline void set_strategy(RunRecord &record, StrategyRecord strategy) {
  auto &list = record.strategies;
  const auto at =
      std::ranges::find_if(list, [&](const StrategyRecord &existing) {
        return static_cast<int>(existing.subject) >=
               static_cast<int>(strategy.subject);
      });
  if (at != list.end() && at->subject == strategy.subject)
    *at = std::move(strategy);
  else
    list.insert(at, std::move(strategy));
}
// The size field of these mesh options, if one is set.
inline void set_size_field_strategy(RunRecord &record,
                                    const discretization::MeshOptions &o) {
  if (o.refinement.size_field)
    set_strategy(record, strategy_record(DecisionSubject::SizeField,
                                         size_field_choice(o.refinement), false,
                                         o.refinement.size_field_descriptor));
}
// Attach where the mesh came from (and its size field strategy).
inline void set_mesh_provenance(RunRecord &record, MeshProvenance provenance) {
  set_size_field_strategy(record, provenance.options);
  record.mesh.provenance = std::move(provenance);
}
inline void set_mesh_provenance(RunRecord &record,
                                const discretization::MeshResult &result) {
  set_mesh_provenance(record, mesh_provenance(result));
}

namespace record_detail {
inline RunRecord make(SolvePlan plan, const WorkloadRecord &workload,
                      std::string mesh_fingerprint, const WorkRecord &work) {
  RunRecord record;
  record.execution = {plan.assembly.execution, pde2d::openmp_available()};
  record.workload = workload;
  record.work = work;
  record.evidence = summarize_evidence(plan.decisions);
  record.mesh.fingerprint = std::move(mesh_fingerprint);
  const auto &solver = plan.solver;
  set_strategy(record,
               strategy_record(DecisionSubject::LinearSolver,
                               linear_solver_name(solver), !solver.backend,
                               solver.backend ? solver.backend->descriptor()
                                              : std::nullopt));
  if (plan.mesh)
    set_size_field_strategy(record, plan.mesh->options);
  record.plan = std::move(plan);
  return record;
}
inline WorkloadRecord workload(const discretization::TriangleMesh &mesh) {
  return {mesh.nodes.size(), mesh.elements.size(), mesh.boundary_edges.size()};
}
} // namespace record_detail

// The record of a run of `plan` on `mesh`: execution, workload, work,
// evidence, versions, the mesh fingerprint and the plan's strategies. The
// solve entry points add the problem and initial-field fingerprints, mesh
// provenance, the stationary time, time strategies and timings.
inline RunRecord make_run_record(SolvePlan plan,
                                 const discretization::TriangleMesh &mesh,
                                 const WorkRecord &work) {
  return record_detail::make(std::move(plan), record_detail::workload(mesh),
                             discretization::mesh_fingerprint(mesh), work);
}
} // namespace pde2d::fem

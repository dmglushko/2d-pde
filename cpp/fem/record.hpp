#pragma once
// Reproducibility record of one top-level result (StationarySolution,
// Evolution): what ran, on what, with how much work, and how well the
// choices were supported. Structured fields only, so two runs can be
// compared field by field; write_report (report.hpp) prints it.
//
// Not recorded: wall-clock timings (they are in the timing structs and are
// never reproducible), the problem and mesh themselves (the caller has
// them; the workload identifies the mesh size), and actual OpenMP team
// sizes (only the selected policy is known).
#include "execution.hpp"
#include "fem/decision.hpp"
#include "fem/evidence.hpp"
#include "fem/plan.hpp"
#include <cstddef>
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

struct RunRecord {
  // Settings that ran; plan.decisions explain them. Strategy factories in
  // the plan are not comparable; the decisions name what they created.
  SolvePlan plan{};
  ExecutionRecord execution{};
  WorkloadRecord workload{};
  WorkRecord work{};
  EvidenceSummary evidence{};
};

inline RunRecord make_run_record(SolvePlan plan,
                                 const discretization::TriangleMesh &mesh,
                                 const WorkRecord &work) {
  RunRecord record;
  record.execution = {plan.assembly.execution, pde2d::openmp_available()};
  record.workload = {mesh.nodes.size(), mesh.elements.size(),
                     mesh.boundary_edges.size()};
  record.work = work;
  record.evidence = summarize_evidence(plan.decisions);
  record.plan = std::move(plan);
  return record;
}
} // namespace pde2d::fem

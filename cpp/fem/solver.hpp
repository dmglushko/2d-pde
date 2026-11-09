#pragma once
// Stationary P1 solve. The element-independent linear algebra lives in
// constraints.hpp (Dirichlet lifting) and prepared_linear_solver.hpp
// (PreparedLinearSolver, backends); linear_solver.hpp includes both.
//
//   PreparedStationary s(mesh, problem, plan);  // symbolic preparation once
//   auto a = s.solve();                         // assemble, lift, solve
//   auto b = s.solve(0, guess);                 // reuses the factorization
//
// PreparedStationary keeps one PreparedAssembly and the last prepared linear
// solver, reused while the constrained matrix is coefficient-for-coefficient
// unchanged, so repeated solves (parameter loops, an adaptive loop's
// re-solves on one mesh, a time-dependent source sampled at several times)
// pay for symbolic preparation and factorization once. solve_stationary()
// is the one-shot wrapper.
//
// The plan's decisions are validated on construction (validate_decisions,
// plan.hpp): a stale decision throws before any work, and so does an
// analysis (plan.analysis) made for another problem or mesh ("Stale
// analysis", require_current_analysis). Constructed from a
// discretization::MeshResult, the solve also checks the plan's mesh plan
// against the result's options and records the result's MeshOptions and
// MeshReport in every RunRecord.
#include "discretization/mesh_fingerprint.hpp"
#include "fem/assembly.hpp"
#include "fem/fingerprint.hpp"
#include "fem/constraints.hpp"
#include "fem/prepared_linear_solver.hpp"
#include "fem/mesh_acceptance.hpp"
#include "fem/plan.hpp"
#include "fem/record.hpp"
#include "fem/timing.hpp"
#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::fem {
// A stationary solution with the record needed to reproduce it.
struct StationarySolution : LinearSolution {
  RunRecord record{};
};

// Cumulative work of one PreparedStationary. Performance contract: with an
// unchanged constrained matrix, solver.preparations == 1 and
// solver_reuses == solves - 1.
struct StationaryStatistics {
  std::size_t solves = 0, assembly_calls = 0, solver_reuses = 0;
  // Summed solver work (preparations, factorizations, rhs_solves, seconds).
  SolverTimings solver{};
};

namespace solver_detail {
// The solve plan's decisions, its mesh plan against the mesh's options and
// its analysis against the problem and mesh fingerprints. A MeshResult must
// meet its size target unless the caller accepted it (accept_unsatisfied,
// mesh_acceptance.hpp).
inline const SolvePlan &checked_plan(const SolvePlan &plan,
                                     const discretization::MeshResult *mesh,
                                     bool accepted,
                                     const std::string &problem_fingerprint,
                                     const std::string &mesh_fingerprint) {
  if (mesh && !accepted)
    require_size_satisfied(*mesh);
  validate_decisions(plan);
  if (plan.mesh && mesh) {
    validate_decisions(*plan.mesh, mesh->options);
    if (const auto subject =
            mesh_options_difference(plan.mesh->options, mesh->options))
      throw std::invalid_argument(
          "Mesh plan does not match the mesh's options: " +
          std::string(decision_subject_name(*subject)));
  }
  if (plan.analysis)
    require_current_analysis(*plan.analysis, problem_fingerprint,
                             mesh_fingerprint);
  return plan;
}
} // namespace solver_detail

class PreparedStationary {
  // Inputs of every record: computed once, the mesh and problem are copied
  // into the assembly and do not change.
  std::string mesh_fingerprint_, problem_fingerprint_;
  SolvePlan plan_;
  PreparedAssembly assembly_;
  std::unique_ptr<PreparedLinearSolver> prepared_;
  StationaryStatistics statistics_;
  std::optional<MeshProvenance> provenance_;

  StationarySolution solve_from(double time, const Vector *guess) {
    const auto started = timing::Clock::now();
    const auto system = assembly_.assemble(time);
    ++statistics_.assembly_calls;
    auto [a, b] = constrain(system.stiffness, system.load, system.dirichlet);
    WorkRecord work{.assembly_calls = 1, .linear_solves = 1};
    SolverTimings preparation;
    if (!prepared_ || !prepared_->matches(a, plan_.solver)) {
      prepared_ = std::make_unique<PreparedLinearSolver>(a, plan_.solver);
      preparation = prepared_->preparation_timings();
      work.solver_preparations = 1;
    } else {
      ++statistics_.solver_reuses;
      work.solver_reuses = 1;
    }
    StationarySolution result;
    static_cast<LinearSolution &>(result) = [&] {
      if (!guess)
        return prepared_->solve(b);
      // Constrained rows are identities: start them at their values.
      Vector start = *guess;
      if (start.size() == b.size())
        for (const auto &d : system.dirichlet)
          start[d.node] = d.value;
      return prepared_->solve(b, start);
    }();
    ++statistics_.solves;
    result.report.timings += preparation;
    result.report.timings.total_seconds = timing::seconds_since(started);
    statistics_.solver += result.report.timings;
    result.report.decisions = plan_.decisions;
    if (!result.report.converged)
      throw std::runtime_error("Stationary solve: " + result.report.message);
    result.record =
        record_detail::make(plan_, record_detail::workload(assembly_.mesh()),
                            mesh_fingerprint_, work);
    if (provenance_)
      set_mesh_provenance(result.record, *provenance_);
    result.record.problem_fingerprint = problem_fingerprint_;
    result.record.stationary_time = time;
    result.record.work.solver_iterations =
        static_cast<std::size_t>(std::max(0, result.report.iterations));
    const auto &r = result.report;
    StationarySolveRecord solve{r.initial_guess, std::nullopt,
                                r.iterations,    r.converged,
                                r.residual_norm, r.relative_residual};
    if (guess) {
      const Field given(*guess);
      solve.guess = FieldRecord{field_kind(given), field_fingerprint(given)};
    }
    result.record.stationary_solve = std::move(solve);
    result.record.timings.total_seconds = result.report.timings.total_seconds;
    return result;
  }

  PreparedStationary(const discretization::TriangleMesh &mesh,
                     const Problem &problem, const SolvePlan &plan,
                     const discretization::MeshResult *result, bool accepted)
      : mesh_fingerprint_(discretization::mesh_fingerprint(mesh)),
        problem_fingerprint_(fem::problem_fingerprint(problem)),
        plan_(solver_detail::checked_plan(
            plan, result, accepted, problem_fingerprint_, mesh_fingerprint_)),
        assembly_(mesh, problem, plan_.assembly) {
    if (result)
      provenance_ = mesh_provenance(*result);
  }

public:
  PreparedStationary(const discretization::TriangleMesh &mesh,
                     const Problem &problem, const SolvePlan &plan)
      : PreparedStationary(mesh, problem, plan, nullptr, false) {}
  PreparedStationary(const discretization::TriangleMesh &mesh,
                     const Problem &problem,
                     const AssemblyOptions &assembly = {},
                     const SolverOptions &solver = {})
      : PreparedStationary(mesh, problem, plan_from_user(assembly, solver)) {}
  // A mesh with provenance: records carry its MeshOptions and MeshReport,
  // and plan.mesh (if set) must match result.options. The result must meet
  // its size target (report.size_satisfied), otherwise this throws; pass
  // accept_unsatisfied(result) to use it anyway.
  PreparedStationary(const discretization::MeshResult &result,
                     const Problem &problem, const SolvePlan &plan)
      : PreparedStationary(result.mesh, problem, plan, &result, false) {}
  PreparedStationary(const discretization::MeshResult &result,
                     const Problem &problem,
                     const AssemblyOptions &assembly = {},
                     const SolverOptions &solver = {})
      : PreparedStationary(result, problem, plan_from_user(assembly, solver)) {}
  PreparedStationary(AcceptedMesh accepted, const Problem &problem,
                     const SolvePlan &plan)
      : PreparedStationary(accepted.result().mesh, problem, plan,
                           &accepted.result(), true) {}
  PreparedStationary(AcceptedMesh accepted, const Problem &problem,
                     const AssemblyOptions &assembly = {},
                     const SolverOptions &solver = {})
      : PreparedStationary(accepted, problem,
                           plan_from_user(assembly, solver)) {}

  // Assemble at `time`, lift Dirichlet data and solve. Throws
  // std::runtime_error ("Stationary solve: ...") when the solve fails.
  StationarySolution solve(double time = 0) {
    return solve_from(time, nullptr);
  }
  // Start an iterative backend from `guess` (one finite entry per node, for
  // example transfer_field output); direct backends ignore it and the
  // report says InitialGuess::Ignored.
  StationarySolution solve(double time, const Vector &guess) {
    return solve_from(time, &guess);
  }

  const PreparedAssembly &assembly() const noexcept { return assembly_; }
  const SolvePlan &plan() const noexcept { return plan_; }
  const StationaryStatistics &statistics() const noexcept {
    return statistics_;
  }
};

inline StationarySolution
solve_stationary(const discretization::TriangleMesh &mesh,
                 const Problem &problem, const AssemblyOptions &assembly = {},
                 const SolverOptions &solver = {}) {
  return PreparedStationary(mesh, problem, assembly, solver).solve();
}
inline StationarySolution
solve_stationary(const discretization::TriangleMesh &mesh,
                 const Problem &problem, const AssemblyOptions &assembly,
                 const SolverOptions &solver, const Vector &guess) {
  return PreparedStationary(mesh, problem, assembly, solver).solve(0, guess);
}
inline StationarySolution
solve_stationary(const discretization::TriangleMesh &mesh,
                 const Problem &problem, const SolvePlan &plan) {
  return PreparedStationary(mesh, problem, plan).solve();
}
inline StationarySolution
solve_stationary(const discretization::TriangleMesh &mesh,
                 const Problem &problem, const SolvePlan &plan,
                 const Vector &guess) {
  return PreparedStationary(mesh, problem, plan).solve(0, guess);
}
// From a MeshResult: the record also carries its options and report. The
// result must meet its size target; wrap it in accept_unsatisfied() to
// solve on it anyway.
inline StationarySolution
solve_stationary(const discretization::MeshResult &result,
                 const Problem &problem, const AssemblyOptions &assembly = {},
                 const SolverOptions &solver = {}) {
  return PreparedStationary(result, problem, assembly, solver).solve();
}
inline StationarySolution
solve_stationary(const discretization::MeshResult &result,
                 const Problem &problem, const SolvePlan &plan) {
  return PreparedStationary(result, problem, plan).solve();
}
inline StationarySolution
solve_stationary(const discretization::MeshResult &result,
                 const Problem &problem, const SolvePlan &plan,
                 const Vector &guess) {
  return PreparedStationary(result, problem, plan).solve(0, guess);
}
inline StationarySolution solve_stationary(AcceptedMesh result,
                                           const Problem &problem,
                                           const AssemblyOptions &assembly = {},
                                           const SolverOptions &solver = {}) {
  return PreparedStationary(result, problem, assembly, solver).solve();
}
inline StationarySolution solve_stationary(AcceptedMesh result,
                                           const Problem &problem,
                                           const SolvePlan &plan) {
  return PreparedStationary(result, problem, plan).solve();
}
inline StationarySolution solve_stationary(AcceptedMesh result,
                                           const Problem &problem,
                                           const SolvePlan &plan,
                                           const Vector &guess) {
  return PreparedStationary(result, problem, plan).solve(0, guess);
}
} // namespace pde2d::fem

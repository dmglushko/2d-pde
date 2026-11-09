#pragma once
// Stationary P1 solve. The element-independent linear algebra lives in
// constraints.hpp (Dirichlet lifting) and prepared_linear_solver.hpp
// (PreparedLinearSolver, backends); linear_solver.hpp includes both.
//
//   PreparedStationary s(mesh, problem, assembly, solver); // preparation once
//   auto a = s.solve();                  // assemble, lift, solve
//   auto b = s.solve(0, guess);          // reuses the factorization
//
// PreparedStationary keeps one PreparedAssembly and the last prepared linear
// solver, reused while the constrained matrix is coefficient-for-coefficient
// unchanged, so repeated solves (parameter loops, a time-dependent source
// sampled at several times) pay for symbolic preparation and factorization
// once. solve_stationary() is the one-shot wrapper.
//
// Every solution carries a RunRecord (record.hpp): the options, workload,
// work of this solve, the time, guess and solver outcome, the mesh and
// problem fingerprints (computed once per PreparedStationary) and versions.
#include "discretization/mesh_fingerprint.hpp"
#include "fem/assembly.hpp"
#include "fem/constraints.hpp"
#include "fem/fingerprint.hpp"
#include "fem/prepared_linear_solver.hpp"
#include "fem/record.hpp"
#include "fem/timing.hpp"
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace pde2d::fem {
// The nodal values of a stationary solve, its solver report and the record
// needed to reproduce it.
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

class PreparedStationary {
  SolverOptions solver_;
  PreparedAssembly assembly_;
  std::unique_ptr<PreparedLinearSolver> prepared_;
  StationaryStatistics statistics_;
  // Inputs of every record, computed once.
  std::string mesh_fingerprint_, problem_fingerprint_;

  StationarySolution solve_from(double time, const Vector *guess) {
    const auto started = timing::Clock::now();
    const auto system = assembly_.assemble(time);
    ++statistics_.assembly_calls;
    const double assembly_seconds = timing::seconds_since(started);
    const auto solver_started = timing::Clock::now();
    auto [a, b] = constrain(system.stiffness, system.load, system.dirichlet);
    SolverTimings preparation;
    bool reused = false;
    if (!prepared_ || !prepared_->matches(a, solver_)) {
      prepared_ = std::make_unique<PreparedLinearSolver>(a, solver_);
      preparation = prepared_->preparation_timings();
    } else {
      ++statistics_.solver_reuses;
      reused = true;
    }
    StationarySolution result{[&] {
      if (!guess)
        return prepared_->solve(b);
      // Constrained rows are identities: start them at their values.
      Vector start = *guess;
      if (start.size() == b.size())
        for (const auto &d : system.dirichlet)
          start[d.node] = d.value;
      return prepared_->solve(b, start);
    }()};
    const double solver_seconds = timing::seconds_since(solver_started);
    ++statistics_.solves;
    result.report.timings += preparation;
    result.report.timings.total_seconds = timing::seconds_since(started);
    statistics_.solver += result.report.timings;
    if (!result.report.converged)
      throw std::runtime_error("Stationary solve: " + result.report.message);
    record(result, time, guess, reused, assembly_seconds, solver_seconds);
    return result;
  }
  void record(StationarySolution &result, double time, const Vector *guess,
              bool reused, double assembly_seconds,
              double solver_seconds) const {
    const auto &report = result.report;
    auto &r = result.record;
    r.assembly = assembly_.options();
    r.solver = solver_;
    r.linear_solver = record_detail::linear_solver(solver_);
    r.openmp_available = pde2d::openmp_available();
    r.workload = record_detail::workload(assembly_.mesh());
    r.work = {.assembly_calls = 1,
              .linear_solves = 1,
              .solver_preparations = report.timings.preparations,
              .solver_reuses = reused ? 1u : 0u,
              .time_steps = 0,
              .solver_iterations = record_detail::iterations(report)};
    StationaryRecord stationary{time,
                                report.initial_guess,
                                std::nullopt,
                                report.iterations,
                                report.converged,
                                report.residual_norm,
                                report.relative_residual};
    if (guess) {
      const Field given{*guess};
      stationary.guess =
          FieldRecord{field_kind(given), field_fingerprint(given)};
    }
    r.stationary = std::move(stationary);
    r.mesh_fingerprint = mesh_fingerprint_;
    r.problem_fingerprint = problem_fingerprint_;
    r.timings = {report.timings.total_seconds, assembly_seconds,
                 solver_seconds};
  }

public:
  PreparedStationary(const discretization::TriangleMesh &mesh,
                     const Problem &problem,
                     const AssemblyOptions &assembly = {},
                     const SolverOptions &solver = {})
      : solver_(solver), assembly_(mesh, problem, assembly),
        mesh_fingerprint_(discretization::mesh_fingerprint(mesh)),
        problem_fingerprint_(fem::problem_fingerprint(problem)) {}

  // Assemble at `time`, lift Dirichlet data and solve. Throws
  // std::runtime_error ("Stationary solve: ...") when the solve fails.
  StationarySolution solve(double time = 0) {
    return solve_from(time, nullptr);
  }
  // Start an iterative backend from `guess` (one finite entry per node);
  // direct backends ignore it and the report says InitialGuess::Ignored.
  StationarySolution solve(double time, const Vector &guess) {
    return solve_from(time, &guess);
  }

  const PreparedAssembly &assembly() const noexcept { return assembly_; }
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
} // namespace pde2d::fem

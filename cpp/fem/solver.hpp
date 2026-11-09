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
#include "fem/assembly.hpp"
#include "fem/constraints.hpp"
#include "fem/prepared_linear_solver.hpp"
#include "fem/timing.hpp"
#include <memory>
#include <stdexcept>

namespace pde2d::fem {
// The nodal values of a stationary solve and its solver report.
using StationarySolution = LinearSolution;

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

  StationarySolution solve_from(double time, const Vector *guess) {
    const auto started = timing::Clock::now();
    const auto system = assembly_.assemble(time);
    ++statistics_.assembly_calls;
    auto [a, b] = constrain(system.stiffness, system.load, system.dirichlet);
    SolverTimings preparation;
    if (!prepared_ || !prepared_->matches(a, solver_)) {
      prepared_ = std::make_unique<PreparedLinearSolver>(a, solver_);
      preparation = prepared_->preparation_timings();
    } else
      ++statistics_.solver_reuses;
    StationarySolution result = [&] {
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
    if (!result.report.converged)
      throw std::runtime_error("Stationary solve: " + result.report.message);
    return result;
  }

public:
  PreparedStationary(const discretization::TriangleMesh &mesh,
                     const Problem &problem,
                     const AssemblyOptions &assembly = {},
                     const SolverOptions &solver = {})
      : solver_(solver), assembly_(mesh, problem, assembly) {}

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

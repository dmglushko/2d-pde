#pragma once
#include <chrono>
#include <cstddef>

namespace pde2d::fem {
// Wall-clock seconds from steady_clock; parent totals include their subphases.
struct SolverTimings {
  double validation_seconds = 0, rank_check_seconds = 0,
         factorization_seconds = 0;
  double solve_seconds = 0, residual_seconds = 0, total_seconds = 0;
  // Factorizations counts matrix-dependent preparation attempts (including
  // SPD/preconditioner setup); rhs_solves counts calls, successful or failed.
  std::size_t preparations = 0, rank_checks = 0, factorizations = 0,
              rhs_solves = 0;
  SolverTimings &operator+=(const SolverTimings &other) {
    validation_seconds += other.validation_seconds;
    rank_check_seconds += other.rank_check_seconds;
    factorization_seconds += other.factorization_seconds;
    solve_seconds += other.solve_seconds;
    residual_seconds += other.residual_seconds;
    total_seconds += other.total_seconds;
    preparations += other.preparations;
    rank_checks += other.rank_checks;
    factorizations += other.factorizations;
    rhs_solves += other.rhs_solves;
    return *this;
  }
};
struct EvolutionTimings {
  double assembly_seconds = 0, constraints_seconds = 0, solver_seconds = 0;
  // Disjoint evolution phases; solver below contains nested solver subphases.
  double observer_seconds = 0, other_seconds = 0, total_seconds = 0;
  std::size_t assembly_calls = 0, linear_solves = 0, time_steps = 0,
              solver_reuses = 0;
  SolverTimings solver;
  // Trial steps a StepController rejected. time_steps counts accepted steps
  // only; the work of rejected trials is in the other counters and seconds.
  std::size_t rejected_steps = 0;
  // Iterations of every linear solve, summed (SolverReport::iterations).
  std::size_t solver_iterations = 0;
};
namespace timing {
using Clock = std::chrono::steady_clock;
inline double seconds_since(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}
// Adds the lifetime of the scope to `target` (also when leaving by
// exception), e.g.  { timing::Accumulate _(t.assembly_seconds); ... }
class Accumulate {
  double &target_;
  Clock::time_point start_ = Clock::now();

public:
  explicit Accumulate(double &target) : target_(target) {}
  Accumulate(const Accumulate &) = delete;
  Accumulate &operator=(const Accumulate &) = delete;
  ~Accumulate() { target_ += seconds_since(start_); }
};
} // namespace timing
} // namespace pde2d::fem

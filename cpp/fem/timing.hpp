#pragma once
#include <chrono>
#include <cstddef>

namespace fem {
// Wall-clock seconds from steady_clock; parent totals include their subphases.
struct SolverTimings {
  double validation_seconds=0,rank_check_seconds=0,factorization_seconds=0;
  double solve_seconds=0,residual_seconds=0,total_seconds=0;
  // Factorizations counts matrix-dependent preparation attempts (including
  // SPD/preconditioner setup); rhs_solves counts calls, successful or failed.
  std::size_t preparations=0,rank_checks=0,factorizations=0,rhs_solves=0;
  SolverTimings& operator+=(const SolverTimings& other) {
    validation_seconds+=other.validation_seconds;rank_check_seconds+=other.rank_check_seconds;
    factorization_seconds+=other.factorization_seconds;solve_seconds+=other.solve_seconds;
    residual_seconds+=other.residual_seconds;total_seconds+=other.total_seconds;
    preparations+=other.preparations;rank_checks+=other.rank_checks;
    factorizations+=other.factorizations;rhs_solves+=other.rhs_solves;
    return *this;
  }
};
struct EvolutionTimings {
  double assembly_seconds=0,constraints_seconds=0,solver_seconds=0;
  // Disjoint evolution phases; solver below contains nested solver subphases.
  double observer_seconds=0,other_seconds=0,total_seconds=0;
  std::size_t assembly_calls=0,linear_solves=0,time_steps=0,solver_reuses=0;
  SolverTimings solver;
};
namespace timing_detail {
using Clock=std::chrono::steady_clock;
inline double seconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now()-start).count();
}
}
} // namespace fem

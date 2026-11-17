#pragma once

// Private: checking and resolving the `threads` option of a solver (the
// iterative solvers and the direct solvers that run in parallel).
// Includes <omp.h>: only the solver .cpp files and private headers include
// this header.

#include <omp.h>

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace pde2d::solve::detail {

// The largest thread count an option may ask for (as in the Assembler):
// it catches a negative count converted to an unsigned one, with which
// OpenMP would try to start billions of threads and abort.
inline constexpr std::size_t max_threads = 1024;

// Throws std::invalid_argument, naming `solver`, for more than max_threads
// threads.
inline void check_threads(std::size_t threads, const std::string &solver) {
  if (threads > max_threads)
    throw std::invalid_argument(solver + ": threads must be at most " +
                                std::to_string(max_threads) +
                                " (0 means all "
                                "hardware threads), not " +
                                std::to_string(threads));
}

// The number of threads `threads` asks for: itself, or for 0 every
// processor available to this process (omp_get_num_procs, as in the
// Assembler).
inline int resolve_threads(std::size_t threads) {
  if (threads == 0)
    return std::min(omp_get_num_procs(), static_cast<int>(max_threads));
  return static_cast<int>(threads);
}

} // namespace pde2d::solve::detail

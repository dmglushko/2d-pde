#pragma once

// Private: the thread count of AssemblerOptions, checked and resolved for
// OpenMP. Includes <omp.h>: only .cpp files of pde2d_fem include this
// header, never a public one.

#include <omp.h>

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace pde2d::fem::detail {

// The largest thread count an option may ask for. Far above any machine
// this library targets; it catches a negative count converted to an
// unsigned one, with which OpenMP would try to start billions of threads
// and abort the program.
inline constexpr std::size_t max_threads = 1024;

// Throws std::invalid_argument, naming `component`, for more than
// max_threads threads.
inline void check_threads(std::size_t threads, const std::string &component) {
  if (threads > max_threads)
    throw std::invalid_argument(component + ": threads must be at most " +
                                std::to_string(max_threads) +
                                " (0 means all "
                                "hardware threads), not " +
                                std::to_string(threads));
}

// The number of threads `threads` asks for: itself, or for 0 every
// processor available to this process (omp_get_num_procs: it honours the
// process's CPU affinity, and not OMP_NUM_THREADS, which is a setting of
// the environment, not of the component).
inline int resolve_threads(std::size_t threads) {
  if (threads == 0)
    return std::min(omp_get_num_procs(), static_cast<int>(max_threads));
  return static_cast<int>(threads);
}

} // namespace pde2d::fem::detail

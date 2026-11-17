#pragma once

// Private: how many threads Eigen's products may use. Includes <omp.h> and
// Eigen: only the solver .cpp files include this header.
//
// pde2d_solve is compiled with OpenMP, so Eigen runs some products in
// parallel: the row-major sparse matrix times dense vector product (rows
// shared among the threads, each row summed by one thread, so the result
// is the same for every count; only for more than 20000 nonzeros) and
// large dense matrix products (inside SparseLU's supernodes). Eigen reads
// the count from Eigen::nbThreads(), which is omp_get_max_threads() unless
// Eigen::setNbThreads was called; omp_get_max_threads() reads the calling
// thread's own OpenMP setting (omp_set_num_threads). So every Eigen
// computation in pde2d_solve runs inside an EigenThreads scope, which sets
// that per-thread setting for its lifetime: two solves at once in
// different threads keep their own counts, and nothing global is written.
// If the program calls Eigen::setNbThreads itself, that count wins; the
// results of the parallel sparse product are the same anyway.

#include <Eigen/Core>
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

// While it lives, Eigen's products in the calling thread may use
// `threads` threads; the thread's previous OpenMP setting comes back
// afterwards.
class EigenThreads {
public:
  explicit EigenThreads(int threads) : previous_(omp_get_max_threads()) {
    omp_set_num_threads(threads);
  }
  ~EigenThreads() { omp_set_num_threads(previous_); }
  EigenThreads(const EigenThreads &) = delete;
  EigenThreads &operator=(const EigenThreads &) = delete;

  // The count Eigen's products use now (Eigen::nbThreads()).
  static int in_use() { return Eigen::nbThreads(); }

private:
  int previous_;
};

} // namespace pde2d::solve::detail

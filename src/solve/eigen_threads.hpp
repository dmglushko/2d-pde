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

#include "threads.hpp"

#include <Eigen/Core>
#include <omp.h>

namespace pde2d::solve::detail {

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

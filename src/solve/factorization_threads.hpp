#pragma once

// Private: the threads of the parallel direct solvers (SupernodalCholesky,
// MultifrontalLu). No heavy dependency here; the OpenBLAS and OpenMP calls
// are in factorization_threads.cpp.
//
// These solvers get their threads from two places: OpenMP (inside the
// factorization library, which reads the calling thread's OpenMP count)
// and the BLAS (OpenBLAS, which runs its own pool of threads). OpenMP's
// count is set per calling thread, but OpenBLAS has only one count for the
// whole process (openblas_set_num_threads). So a FactorizationThreads
// scope also holds a process-wide lock: the direct solves of this library
// that would run at once in different threads run one after another, and
// each sees its own BLAS count for its whole duration (with `.threads = 1`
// that makes it reproducible run to run). Code outside this library that
// calls OpenBLAS while a scope lives sees the scope's count.

#include <mutex>

namespace pde2d::solve::detail {

// While it lives: no other FactorizationThreads scope lives (it waits for
// them), OpenBLAS uses `blas` threads and the calling thread's OpenMP
// count is `openmp`. Both counts come back afterwards.
class FactorizationThreads {
public:
  FactorizationThreads(int openmp, int blas);
  ~FactorizationThreads();
  FactorizationThreads(const FactorizationThreads &) = delete;
  FactorizationThreads &operator=(const FactorizationThreads &) = delete;

private:
  std::unique_lock<std::mutex> lock_;
  int previous_openmp_;
  int previous_blas_;
};

} // namespace pde2d::solve::detail

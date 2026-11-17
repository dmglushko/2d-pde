#include "factorization_threads.hpp"

#include <omp.h>

// OpenBLAS's thread control, declared here rather than through its
// cblas.h (which would also declare the whole CBLAS interface). Both
// functions exist in every OpenBLAS build (pthreads or OpenMP). The
// library is linked directly (src/solve/CMakeLists.txt); it is the same
// shared object the system BLAS (libblas.so.3) of SuiteSparse resolves to
// when OpenBLAS is the system BLAS.
extern "C" {
void openblas_set_num_threads(int threads);
int openblas_get_num_threads(void);
}

namespace pde2d::solve::detail {

namespace {

std::mutex &factorization_mutex() {
  static std::mutex mutex;
  return mutex;
}

} // namespace

FactorizationThreads::FactorizationThreads(int openmp, int blas)
    : lock_(factorization_mutex()), previous_openmp_(omp_get_max_threads()),
      previous_blas_(openblas_get_num_threads()) {
  omp_set_num_threads(openmp);
  openblas_set_num_threads(blas);
}

FactorizationThreads::~FactorizationThreads() {
  openblas_set_num_threads(previous_blas_);
  omp_set_num_threads(previous_openmp_);
}

} // namespace pde2d::solve::detail

#pragma once
#include <cstddef>
#include <exception>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace pde2d {
enum class ExecutionBackend { Serial, OpenMP };
struct ExecutionOptions {
  ExecutionBackend backend = ExecutionBackend::Serial;
  int threads = 0;
  std::size_t min_parallel_size = 64;
};
inline bool openmp_available() {
#ifdef _OPENMP
  return true;
#else
  return false;
#endif
}
inline void validate_execution(const ExecutionOptions &options) {
  if (options.threads < 0)
    throw std::invalid_argument("Execution thread count must be nonnegative");
  switch (options.backend) {
  case ExecutionBackend::Serial:
    break;
  case ExecutionBackend::OpenMP:
    if (!openmp_available())
      throw std::invalid_argument(
          "OpenMP execution requested in a serial-only build");
#ifdef _OPENMP
    if (options.threads > omp_get_thread_limit())
      throw std::invalid_argument(
          "Execution thread count exceeds OpenMP thread limit");
#endif
    break;
  default:
    throw std::invalid_argument("Unknown execution backend");
  }
}
// Each work item owns its output. Symbolic preparation and final reductions
// belong outside this loop. Workers finish before the lowest-index error is
// rethrown on the caller thread; no exceptions escape an OpenMP region.
template <class Function>
void parallel_for(std::size_t count, const ExecutionOptions &options,
                  Function &&function) {
  validate_execution(options);
  if (count >
      static_cast<std::size_t>(std::numeric_limits<std::ptrdiff_t>::max()))
    throw std::invalid_argument("Parallel work range exceeds index capacity");
  bool serial = options.backend == ExecutionBackend::Serial ||
                count < options.min_parallel_size || options.threads == 1;
#ifdef _OPENMP
  serial = serial || omp_in_parallel();
#endif
  if (serial) {
    for (std::size_t i = 0; i < count; ++i)
      function(i);
    return;
  }
#ifdef _OPENMP
  const int threads = options.threads ? options.threads : omp_get_max_threads();
  std::mutex error_mutex;
  std::exception_ptr error;
  std::size_t first_error = count;
#pragma omp parallel for schedule(static) num_threads(threads)
  for (std::ptrdiff_t i = 0; i < static_cast<std::ptrdiff_t>(count); ++i) {
    try {
      function(static_cast<std::size_t>(i));
    } catch (...) {
      std::lock_guard lock(error_mutex);
      if (static_cast<std::size_t>(i) < first_error) {
        first_error = static_cast<std::size_t>(i);
        error = std::current_exception();
      }
    }
  }
  if (error)
    std::rethrow_exception(error);
#else
  // validate_execution already rejected OpenMP; this handles an empty range.
  for (std::size_t i = 0; i < count; ++i)
    function(i);
#endif
}
} // namespace pde2d

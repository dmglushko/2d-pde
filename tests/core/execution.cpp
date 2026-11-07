#include "execution.hpp"
#include "support/check.hpp"
#include <iostream>
#include <string>
#include <vector>
using pde2d_test::check;
using pde2d_test::expect_throws;
void execution_contract() {
  using namespace pde2d;
#ifdef _OPENMP
  // Observe actual teams rather than inferring concurrency from policy values.
  omp_set_dynamic(0);
#endif
  expect_throws<std::invalid_argument>(
      [] { parallel_for(0, {ExecutionBackend::Serial, -1, 0}, [](auto) {}); },
      "nonnegative");
  expect_throws<std::invalid_argument>(
      [] {
        parallel_for(0, {static_cast<ExecutionBackend>(42), 0, 0}, [](auto) {});
      },
      "Unknown");
  for (auto backend : {ExecutionBackend::Serial, ExecutionBackend::OpenMP}) {
    if (backend == ExecutionBackend::OpenMP && !openmp_available()) {
      expect_throws<std::invalid_argument>(
          [&] { parallel_for(0, {backend, 2, 10000}, [](auto) {}); },
          "serial-only");
      continue;
    }
    for (int workers : {1, 2, 4})
      for (std::size_t threshold : {0, 10000}) {
        ExecutionOptions o{backend, workers, threshold};
        std::vector<int> a(128), teams(128, 1);
        parallel_for(a.size(), o, [&](auto i) {
          a[i] = i * i;
#ifdef _OPENMP
          teams[i] = omp_get_num_threads();
#endif
        });
        const int expected_team = backend == ExecutionBackend::OpenMP &&
                                          workers > 1 && a.size() >= threshold
                                      ? workers
                                      : 1;
        for (std::size_t i = 0; i < a.size(); ++i) {
          check(a[i] == int(i * i), "parallel_for result mismatch");
          check(teams[i] == expected_team, "unexpected team size");
        }
        try {
          parallel_for(a.size(), o, [](auto i) {
            if (i == 3 || i == 99)
              throw std::runtime_error(std::to_string(i));
          });
          check(false, "expected parallel_for exception");
        } catch (const std::runtime_error &e) {
          check(std::string(e.what()) == "3", "wrong exception propagated");
        }
      }
  }
  std::cout << "OpenMP=" << openmp_available() << '\n';
}
int main() {
  return pde2d_test::run({{"execution_contract", execution_contract}});
}

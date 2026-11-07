#include "../tests/support/error_norms_reference.hpp"
#include "discretization/square_mesh.hpp"
using pde2d::discretization::square_mesh;
#include <chrono>
#include <iostream>
int main() {
  using namespace pde2d::fem;
  auto mesh = square_mesh(20);
  Vector v = Vector::Zero(mesh.nodes.size());
  const Expr exact = sin(2 * x) * cos(y) + t * x * y;
  PreparedErrorNorms prepared(mesh, exact);
  auto measure = [&](auto f) {
    const auto start = std::chrono::steady_clock::now();
    double checksum = 0;
    for (int i = 0; i < 40; i++)
      checksum += f(i * .01).l2;
    std::cout << std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - start)
                     .count()
              << " s checksum=" << checksum << '\n';
  };
  std::cout << "800 triangles, degree 8, 40 frames, original: ";
  measure([&](double t) { return reference(mesh, v, exact, t); });
  std::cout << "prepared serial: ";
  measure([&](double t) { return prepared.evaluate(v, t); });
  if (pde2d::openmp_available()) {
    std::cout << "prepared OpenMP 2: ";
    measure([&](double t) {
      return prepared.evaluate(v, t, {pde2d::ExecutionBackend::OpenMP, 2, 0});
    });
  }
}

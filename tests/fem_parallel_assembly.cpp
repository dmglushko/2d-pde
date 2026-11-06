#include "../examples/fem/square_mesh.hpp"
#include "fem/prepared_assembly.hpp"
#include <iostream>
#include <stdexcept>
using namespace fem;
namespace {
void check(bool v) {
  if (!v)
    throw std::runtime_error("Parallel assembly mismatch");
}
std::string error(auto &&f) {
  try {
    f();
  } catch (const std::exception &e) {
    return e.what();
  }
  throw std::runtime_error("Expected assembly rejection");
}
void same(const AssembledSystem &a, const AssembledSystem &b) {
  check((a.stiffness - b.stiffness).norm() == 0);
  check((a.first_mass - b.first_mass).norm() == 0);
  check((a.second_mass - b.second_mass).norm() == 0);
  check((a.load - b.load).norm() == 0);
  check(a.dirichlet.size() == b.dirichlet.size());
  for (std::size_t k = 0; k < a.dirichlet.size(); ++k)
    check(a.dirichlet[k].node == b.dirichlet[k].node &&
          a.dirichlet[k].value == b.dirichlet[k].value);
}
} // namespace
int main() {
  try {
    auto mesh = square_mesh(8);
    Problem p;
    p.equation.a_xx = -(2 + x + t);
    p.equation.a_xy = x * y;
    p.equation.a_yy = -(3 + y + t);
    p.equation.m1 = 1 + x + t;
    p.equation.m2 = 2 + t * t;
    p.equation.c = x + t;
    p.equation.source = sin(x + y + t);
    p.assign(dynamic_flux_robin(1 + t, 2 + x, 3 + t, y + t), {"top", "right"});
    p.assign(dirichlet(x + y + t * t), {"bottom", "left"});
    AssemblyOptions options{6};
    PreparedAssembly serial(mesh, p, options);
    for (int threads : {2, 4}) {
      options.execution = {pde2d::ExecutionBackend::OpenMP, threads, 0};
      if (!pde2d::openmp_available()) {
        check(error([&] {
                PreparedAssembly a(mesh, p, options);
              }).find("serial-only") != std::string::npos);
        continue;
      }
      PreparedAssembly parallel(mesh, p, options);
      auto fallback_options = options;
      fallback_options.execution.min_parallel_size = 100000;
      same(serial.assemble(.4),
           PreparedAssembly(mesh, p, fallback_options).assemble(.4));
      fallback_options.execution.min_parallel_size = 0;
      fallback_options.execution.threads = 1;
      same(serial.assemble(.4),
           PreparedAssembly(mesh, p, fallback_options).assemble(.4));
      for (double time : {0., .3, .8})
        for (int order = 0; order < 3; ++order)
          same(serial.assemble(time, order), parallel.assemble(time, order));
      // Errors are thrown by numeric workers after serial symbolic preparation.
      p.equation.source = log(x - t);
      PreparedAssembly bad_serial(mesh, p);
      PreparedAssembly bad_parallel(mesh, p, options);
      check(error([&] { bad_serial.assemble(2); }) ==
            error([&] { bad_parallel.assemble(2); }));
      p.equation.source = 0;
      p.boundaries.clear();
      p.assign(flux_robin(1, t - x, 1), {"bottom", "top", "left", "right"});
      PreparedAssembly edge_serial(mesh, p), edge_parallel(mesh, p, options);
      check(error([&] { edge_serial.assemble(.5); }) ==
            error([&] { edge_parallel.assemble(.5); }));
      p.boundaries.clear();
      p.assign(dirichlet(x + y + t * t), {"bottom", "left"});
      p.assign(dynamic_flux_robin(1 + t, 2 + x, 3 + t, y + t),
               {"top", "right"});
      p.equation.source = sin(x + y + t);
    }
    options.execution = {pde2d::ExecutionBackend::Serial, -1, 0};
    check(error([&] {
            PreparedAssembly a(mesh, p, options);
          }).find("nonnegative") != std::string::npos);
    options.execution = {static_cast<pde2d::ExecutionBackend>(42), 0, 0};
    check(error([&] {
            PreparedAssembly a(mesh, p, options);
          }).find("Unknown execution backend") != std::string::npos);
    std::cout << "Parallel assembly tests passed (OpenMP="
              << pde2d::openmp_available() << ")\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

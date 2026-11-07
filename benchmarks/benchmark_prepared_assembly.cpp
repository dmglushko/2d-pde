// Standalone build command and recorded settings:
// docs/performance/assembly-2026-11-06.md
#include "../examples/fem/square_mesh.hpp"
#include "fem/prepared_assembly.hpp"
#include <chrono>
#include <iostream>
int main(int argc, char **argv) {
  using namespace fem;
  const int threads = argc > 1 ? std::stoi(argv[1]) : 0;
  const bool variable = argc > 2;
  auto mesh = square_mesh(20);
  Problem p;
  p.equation.m2 = 1;
  if (variable) {
    p.equation.c = x + t;
    p.equation.m1 = 1 + t;
    p.equation.m2 = 1 + t * t;
    p.equation.source = sin(x + y + t);
  }
  p.assign(flux_neumann(0), {"bottom", "top", "left", "right"});
  AssemblyOptions options;
  if (threads)
    options.execution = {pde2d::ExecutionBackend::OpenMP, threads, 0};
  auto start = std::chrono::steady_clock::now();
  PreparedAssembly a(mesh, p, options);
  auto prep = std::chrono::steady_clock::now();
  double sum = 0;
  Eigen::Index nnz = 0;
  for (int i = 0; i < 100; ++i) {
    auto s = a.assemble(i * .05);
    sum += s.stiffness.norm();
    nnz = s.stiffness.nonZeros() + s.first_mass.nonZeros() +
          s.second_mass.nonZeros();
  }
  auto stop = std::chrono::steady_clock::now();
  const auto &stats = a.statistics();
  std::cout << "nodes=" << mesh.nodes.size()
            << " elements=" << mesh.elements.size()
            << " calls=100 threads=" << threads << " variable=" << variable
            << " preparation_s="
            << std::chrono::duration<double>(prep - start).count()
            << " assembly_s="
            << std::chrono::duration<double>(stop - prep).count()
            << " checksum=" << sum
            << " evaluators_prepared=" << stats.evaluators
            << " physical_points=" << stats.physical_quadrature_points
            << " constant_interior_evaluations="
            << stats.interior_constant_evaluations
            << " interior_evaluations_per_assembly="
            << stats.interior_evaluations_per_assembly
            << " final_matrix_nnz=" << nnz << '\n';
}

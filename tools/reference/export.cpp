// Opt-in data producer, deliberately outside tests/ discovery.
#include "discretization/square_mesh.hpp"
#include "fem/evolution.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace fem;
void vector(std::ostream &out, const Vector &v) {
  out << '[';
  for (int i = 0; i < v.size(); ++i) {
    if (i)
      out << ',';
    out << v[i];
  }
  out << ']';
}
void matrix(std::ostream &out, const SparseMatrix &a) {
  out << '[';
  for (int i = 0; i < a.rows(); ++i) {
    if (i)
      out << ',';
    vector(out, Vector(a.row(i)));
  }
  out << ']';
}
int main(int argc, char **argv) try {
  if (argc != 2 && argc != 3)
    return 2;
  AssemblyOptions assembly{8};
  if (argc == 3)
    assembly.execution = {pde2d::ExecutionBackend::OpenMP, std::stoi(argv[2]),
                          0};
  std::filesystem::create_directories(argv[1]);
  auto mesh = square_mesh(4);
  std::ofstream out;
  out.exceptions(std::ios::failbit | std::ios::badbit);
  out.open(std::string(argv[1]) + "/reference.json");
  out << std::setprecision(17);
  out << "{\"openmp\":" << (pde2d::openmp_available() ? "true" : "false")
      << ",\"nodes\":[";
  for (size_t i = 0; i < mesh.nodes.size(); ++i) {
    if (i)
      out << ',';
    out << '[' << mesh.nodes[i].x << ',' << mesh.nodes[i].y << ']';
  }
  out << "],\"triangles\":[";
  for (size_t i = 0; i < mesh.elements.size(); ++i) {
    if (i)
      out << ',';
    auto a = mesh.elements[i];
    out << '[' << a[0] << ',' << a[1] << ',' << a[2] << ']';
  }
  out << ']';
  Problem p;
  p.equation.a_xx = -(2 + x);
  p.equation.a_xy = -.4;
  p.equation.a_yy = -(3 + y);
  p.equation.b_x = .3;
  p.equation.b_y = -.2;
  p.equation.c = 1;
  p.equation.source = 1 + x + 2 * y;
  p.equation.m1 = 1 + x;
  p.assign(dirichlet(1 + y), {"left", "right"});
  p.assign(flux_neumann(.5 + x), {"bottom"});
  p.assign(dynamic_flux_robin(.4, 1, 1, 2 + x), {"top"});
  auto a = assemble(mesh, p, 0, assembly);
  out << ",\"K\":";
  matrix(out, a.stiffness);
  out << ",\"C\":";
  matrix(out, a.first_mass);
  out << ",\"F\":";
  vector(out, a.load);
  out << ",\"stationary\":";
  vector(out, solve_stationary(mesh, p, assembly).values);
  out << ",\"runs\":[";
  bool comma = false;
  for (auto method : {TimeMethod::BackwardEuler, TimeMethod::CrankNicolson,
                      TimeMethod::Newmark})
    for (double dt : {.04, .02, .01}) {
      p = Problem{};
      bool wave = method == TimeMethod::Newmark;
      p.equation.m1 = wave ? .2 : 1;
      p.equation.m2 = wave ? 1 : 0;
      p.equation.source = (1 + x + y) * sin(t);
      p.assign(dynamic_flux_robin(.4, 1, 1, (1 + x + y) * (1 + t)),
               {"bottom", "right", "top", "left"});
      TimeOptions o;
      o.method = method;
      o.end = .2;
      o.step = dt;
      o.assembly = assembly;
      auto e = evolve(mesh, p, 1 + x + y, -(1 + x + y), o);
      if (comma)
        out << ',';
      comma = true;
      out << "{\"method\":" << int(method) << ",\"dt\":" << dt << ",\"u\":";
      vector(out, e.frames.back().values);
      out << ",\"assembly_calls\":" << e.timings.assembly_calls
          << ",\"linear_solves\":" << e.timings.linear_solves
          << ",\"solver_reuses\":" << e.timings.solver_reuses;
      out << '}';
    }
  out << "]}\n";
  // Destruction alone cannot report a buffered write or close failure.
  out.close();
} catch (const std::exception &error) {
  std::cerr << "Reference export failed: " << error.what() << '\n';
  return 1;
}

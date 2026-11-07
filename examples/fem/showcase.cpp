#include "fem/evolution.hpp"
#include "discretization/geometry.hpp"
#include "discretization/triangulate.hpp"
#include "square_mesh.hpp"
#include "error_norms.hpp"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>

using namespace fem;

struct Example {
  std::string id, name, equation, boundary, description;
  Problem problem;
  Expr exact;
  bool transient = false;
  Expr initial_velocity = 0;
  TimeOptions time;
};

void write_vector(std::ostream &out, const Vector &values) {
  out << '[';
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    if (i)
      out << ',';
    out << values[i];
  }
  out << ']';
}
// Strings below are developer-written labels, not arbitrary user input.
void quote(std::ostream &out, const std::string &text) {
  out << '"';
  for (char c : text) {
    if (c == '"' || c == '\\')
      out << '\\';
    if (c == '\n')
      out << "\\n";
    else
      out << c;
  }
  out << '"';
}

int main(int argc, char **argv) {
  try {
    const std::filesystem::path destination =
        argc > 1 ? argv[1] : "build/fem-showcase.json";
    if (destination.has_parent_path())
      std::filesystem::create_directories(destination.parent_path());
    std::ofstream out(destination);
    if (!out)
      throw std::runtime_error("Cannot open showcase output");
    out << std::setprecision(17);
    const double pi = std::numbers::pi;
    const Expr shape = sin(pi * x) * sin(pi * y);
    std::vector<Example> examples;
    Example poisson;
    poisson.id = "poisson";
    poisson.name = "Poisson: steady membrane";
    poisson.equation = "-u_xx - u_yy = 2 pi² sin(pi x) sin(pi y)";
    poisson.boundary = "u = 0 on every edge";
    poisson.description =
        "A static manufactured solution. Mesh refinement checks L2 and "
        "gradient errors against sin(pi x) sin(pi y).";
    poisson.exact = shape;
    poisson.problem.equation.source = 2 * pi * pi * shape;
    poisson.problem.assign(dirichlet(0), {"bottom", "right", "top", "left"});
    examples.push_back(poisson);

    Example mixed;
    mixed.id = "mixed";
    mixed.name = "Variable anisotropy + mixed boundaries";
    mixed.equation = "-(1+x²)u_xx - 0.4(1+xy)u_xy - (2+y²)u_yy + (0.3+y)u_x + "
                     "(x-0.2)u_y + u = f";
    mixed.boundary =
        "Dirichlet left/right; flux Neumann bottom; flux Robin top";
    mixed.description =
        "The forcing and boundary data are derived symbolically from u = "
        "1+x+2y+xy. This tests coefficient-derivative corrections and the "
        "mixed-term convention.";
    mixed.exact = 1 + x + 2 * y + x * y;
    auto &eq = mixed.problem.equation;
    eq.a_xx = -(1 + x * x);
    eq.a_xy = -.4 * (1 + x * y);
    eq.a_yy = -(2 + y * y);
    eq.b_x = .3 + y;
    eq.b_y = x - .2;
    eq.c = 1;
    eq.source = eq.a_xx * diff(diff(mixed.exact, x), x) +
                eq.a_xy * diff(diff(mixed.exact, x), y) +
                eq.a_yy * diff(diff(mixed.exact, y), y) +
                eq.b_x * diff(mixed.exact, x) + eq.b_y * diff(mixed.exact, y) +
                eq.c * mixed.exact;
    const Expr flux = -nx * (eq.a_xx * diff(mixed.exact, x) +
                             .5 * eq.a_xy * diff(mixed.exact, y)) -
                      ny * (.5 * eq.a_xy * diff(mixed.exact, x) +
                            eq.a_yy * diff(mixed.exact, y));
    mixed.problem.assign(dirichlet(mixed.exact), {"left", "right"});
    mixed.problem.assign(flux_neumann(flux), {"bottom"});
    mixed.problem.assign(flux_robin(2, .7, 2 * mixed.exact + .7 * flux),
                         {"top"});
    examples.push_back(mixed);

    Example heat;
    heat.id = "heat";
    heat.name = "Heat: diffusion of a hot spot";
    heat.equation = "u_t - u_xx - u_yy = 0";
    heat.boundary = "u = 0 on every edge";
    heat.description = "Crank–Nicolson time integration. The exact temperature "
                       "is exp(-2 pi² t) sin(pi x) sin(pi y).";
    heat.transient = true;
    heat.exact = exp(-2 * pi * pi * t) * shape;
    heat.problem.equation.m1 = 1;
    heat.problem.assign(dirichlet(0), {"bottom", "right", "top", "left"});
    heat.time.method = TimeMethod::CrankNicolson;
    heat.time.end = .18;
    heat.time.step = .003;
    examples.push_back(heat);

    Example wave;
    wave.id = "wave";
    wave.name = "Wave: vibrating membrane";
    wave.equation = "u_tt - u_xx - u_yy = 0";
    wave.boundary = "u = 0 on every edge; initial velocity = 0";
    wave.description = "Newmark average acceleration. Two exact membrane modes "
                       "oscillate at different frequencies; the surface passes "
                       "through zero and changes sign.";
    wave.transient = true;
    wave.exact =
        shape * cos(pi * std::sqrt(2.) * t) +
        .35 * sin(2 * pi * x) * sin(pi * y) * cos(pi * std::sqrt(5.) * t);
    wave.problem.equation.m2 = 1;
    wave.problem.assign(dirichlet(0), {"bottom", "right", "top", "left"});
    wave.time.method = TimeMethod::Newmark;
    wave.time.end = 2.;
    wave.time.step = .0125;
    examples.push_back(wave);

    Example damped = wave;
    damped.id = "damped";
    damped.name = "Damped wave: losing energy";
    damped.equation = "u_tt + 0.5 u_t - u_xx - u_yy = 0";
    damped.boundary =
        "u = 0 on every edge; initial velocity = -0.25 sin(pi x) sin(pi y)";
    damped.description =
        "Both time derivatives are present. The exact mode has an "
        "exponentially decaying envelope; no source maintains the oscillation.";
    damped.exact =
        exp(-.25 * t) * shape * cos(std::sqrt(2 * pi * pi - .0625) * t);
    damped.initial_velocity = -.25 * shape;
    damped.problem.equation.m1 = .5;
    examples.push_back(damped);

    Example boundary;
    boundary.id = "boundary-storage";
    boundary.name = "Wave with a dynamic Robin boundary";
    boundary.equation = "u_tt + 0.2 u_t - u_xx - u_yy = 0.8 exp(-t)(1+x+y)";
    boundary.boundary = "0.4 u_t + u + du/dn = exp(-t)[0.6(1+x+y) + nx+ny]";
    boundary.description =
        "A manufactured solution isolates the boundary u_t term. Boundary "
        "storage contributes to the first-derivative matrix while the interior "
        "retains u_tt.";
    boundary.transient = true;
    boundary.exact = exp(-t) * (1 + x + y);
    boundary.initial_velocity = -(1 + x + y);
    boundary.problem.equation.m2 = 1;
    boundary.problem.equation.m1 = .2;
    boundary.problem.equation.source = .8 * boundary.exact;
    boundary.problem.assign(
        dynamic_robin(.4, 1, 1, .6 * boundary.exact + exp(-t) * (nx + ny)),
        {"bottom", "right", "top", "left"});
    boundary.time.method = TimeMethod::Newmark;
    boundary.time.end = 1;
    boundary.time.step = .0125;
    examples.push_back(boundary);

    discretization::RectangleGeometry geometry(1, 1);
    geometry.setCenter(.5, .5);
    discretization::MeshOptions mesh_options;
    mesh_options.boundary.max_edge_length = .1;
    mesh_options.boundary.preserve_corners = true;
    mesh_options.refinement.max_edge_length = .12;
    const auto meshed = discretization::generate_mesh(geometry, mesh_options);
    if (!meshed.report.size_satisfied)
      throw std::runtime_error(meshed.report.message);
    const auto &mesh = meshed.mesh;
    out << "{\"nodes\":[";
    for (std::size_t i = 0; i < mesh.nodes.size(); ++i) {
      if (i)
        out << ',';
      out << '[' << mesh.nodes[i].x << ',' << mesh.nodes[i].y << ']';
    }
    out << "],\"triangles\":[";
    for (std::size_t i = 0; i < mesh.elements.size(); ++i) {
      if (i)
        out << ',';
      const auto &tri = mesh.elements[i];
      out << '[' << tri[0] << ',' << tri[1] << ',' << tri[2] << ']';
    }
    out << "],\"examples\":[";
    bool first = true;
    for (auto &ex : examples) {
      if (!first)
        out << ',';
      first = false;
      ex.time.assembly.quadrature_degree = 6;
      std::vector<Frame> frames;
      double residual = 0;
      if (ex.transient) {
        const Expr initial =
            ex.exact.subs({{t.get_basic(), Expr(0).get_basic()}});
        auto solution =
            evolve(mesh, ex.problem, initial, ex.initial_velocity, ex.time);
        frames = std::move(solution.frames);
        for (const auto &report : solution.reports)
          residual = std::max(residual, report.relative_residual);
      } else {
        auto solution = solve_stationary(mesh, ex.problem, AssemblyOptions{6});
        residual = solution.report.relative_residual;
        frames.push_back({0, solution.values, Vector{}});
      }
      out << '{';
      for (const auto &[key, value] :
           std::vector<std::pair<std::string, std::string>>{
               {"id", ex.id},
               {"name", ex.name},
               {"equation", ex.equation},
               {"boundary", ex.boundary},
               {"description", ex.description}}) {
        quote(out, key);
        out << ':';
        quote(out, value);
        out << ',';
      }
      out << "\"residual\":" << residual << ",\"frames\":[";
      const PreparedErrorNorms errors(mesh, ex.exact);
      const auto &evaluator = errors.exact_evaluator();
      for (std::size_t k = 0; k < frames.size(); ++k) {
        if (k)
          out << ',';
        const auto &frame = frames[k];
        const auto error = errors.evaluate(frame.values, frame.time);
        out << "{\"time\":" << frame.time << ",\"l2\":" << error.l2
            << ",\"h1\":" << error.h1_seminorm << ",\"values\":";
        write_vector(out, frame.values);
        Vector truth(mesh.nodes.size());
        for (std::size_t i = 0; i < mesh.nodes.size(); ++i)
          truth[i] = evaluator({mesh.nodes[i].x, mesh.nodes[i].y, frame.time});
        out << ",\"exact\":";
        write_vector(out, truth);
        out << '}';
      }
      out << "]}";
      std::cout << ex.name << ": " << frames.size()
                << " frames, final L2 error "
                << errors.evaluate(frames.back().values, frames.back().time).l2
                << '\n';
    }
    out << "],\"convergence\":[";
    for (int i = 0; i < 3; ++i) {
      const int n = 4 * (1 << i);
      const auto fine = square_mesh(n);
      auto s = solve_stationary(fine, poisson.problem, AssemblyOptions{6});
      const auto e = error_norms(fine, s.values, poisson.exact);
      if (i)
        out << ',';
      out << "{\"n\":" << n << ",\"nodes\":" << fine.nodes.size()
          << ",\"l2\":" << e.l2 << ",\"h1\":" << e.h1_seminorm << '}';
    }
    out << "]}\n";
    out.close();
    if (!out)
      throw std::runtime_error("Showcase output write failed");
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

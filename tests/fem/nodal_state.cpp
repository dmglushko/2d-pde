// Nodal fields as initial conditions (InitialConditions with Vector values).
// Oracles: restarting from the last frame of an evolution continues it (to
// rounding for a stateless scheme with time-independent data, and to solver
// tolerance for Newmark, whose restart recomputes the acceleration from the
// equation); invalid nodal input is rejected.
#include "fem/evolution.hpp"
#include "discretization/square_mesh.hpp"
#include "support/check.hpp"
#include "support/check_eigen.hpp"
#include <cmath>

using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;
const Expr pi = acos(-1.);
const std::initializer_list<std::string> sides{"bottom", "right", "top",
                                               "left"};

void restart_continues_the_evolution() {
  const auto mesh = square_mesh(4);
  Problem p;
  p.equation.m1 = 1;
  p.equation.source = x * (1 - x);
  p.assign(dirichlet(0), sides);
  const InitialConditions initial{sin(pi * x) * sin(pi * y)};
  TimeOptions o;
  o.end = .1;
  o.step = .01;
  const auto whole = evolve(mesh, p, initial, o);
  o.end = .05;
  const auto first = evolve(mesh, p, initial, o);
  o.start = .05;
  o.end = .1;
  const auto second =
      evolve(mesh, p, InitialConditions{first.frames.back().values}, o);
  check(second.frames.front().values == first.frames.back().values,
        "A nodal initial value is used as given");
  // Same matrices and step sizes: only the stamps' rounding differs.
  pde2d_test::near_vector(second.frames.back().values,
                          whole.frames.back().values, 1e-13);

  // Newmark from nodal displacement and velocity.
  Problem wave = p;
  wave.equation.m1 = 0;
  wave.equation.m2 = 1;
  TimeOptions w;
  w.method = TimeMethod::Newmark;
  w.end = .1;
  w.step = .01;
  const InitialConditions still{sin(pi * x) * sin(pi * y), Expr(0)};
  const auto wave_whole = evolve(mesh, wave, still, w);
  w.end = .05;
  const auto wave_first = evolve(mesh, wave, still, w);
  w.start = .05;
  w.end = .1;
  const auto &last = wave_first.frames.back();
  const auto wave_second =
      evolve(mesh, wave, InitialConditions{last.values, last.velocity}, w);
  pde2d_test::near_vector(wave_second.frames.back().values,
                          wave_whole.frames.back().values, 1e-9);
  pde2d_test::near_vector(wave_second.frames.back().velocity,
                          wave_whole.frames.back().velocity, 1e-9);
}

void invalid_nodal_input() {
  const auto mesh = square_mesh(2);
  Problem p;
  p.equation.m1 = 1;
  p.assign(dirichlet(0), sides);
  Vector wrong = Vector::Zero(3);
  expect_throws<std::invalid_argument>(
      [&] { evolve(mesh, p, {wrong}); },
      "Initial value must have one finite entry per mesh node");
  Vector nonfinite = Vector::Zero(mesh.nodes.size());
  nonfinite[4] = std::nan("");
  expect_throws<std::invalid_argument>(
      [&] { evolve(mesh, p, {nonfinite}); },
      "Initial value must have one finite entry per mesh node");
  Vector zero = Vector::Zero(mesh.nodes.size());
  expect_throws<std::invalid_argument>(
      [&] { evolve(mesh, p, {zero, wrong}); },
      "Initial velocity must have one finite entry per mesh node");
  // Dirichlet compatibility is enforced as for expressions.
  Vector ones = Vector::Ones(mesh.nodes.size());
  expect_throws<std::invalid_argument>([&] { evolve(mesh, p, {ones}); },
                                       "Incompatible initial prescribed "
                                       "displacement");
  Problem wave = p;
  wave.equation.m1 = 0;
  wave.equation.m2 = 1;
  TimeOptions w;
  w.method = TimeMethod::Newmark;
  expect_throws<std::invalid_argument>([&] { evolve(mesh, wave, {zero}, w); },
                                       "requires an initial velocity");
  expect_throws<std::invalid_argument>(
      [&] { evolve(mesh, wave, {zero, ones}, w); },
      "Incompatible initial prescribed velocity");
  // Expressions keep their symbol checks.
  expect_throws<std::invalid_argument>([&] { evolve(mesh, p, {nx}); },
                                       "Initial value cannot depend on");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"restart_continues_the_evolution", restart_continues_the_evolution},
       {"invalid_nodal_input", invalid_nodal_input}});
}

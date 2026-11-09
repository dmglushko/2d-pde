// Local P1 kernels (p1_element.hpp) against hand-integrated element and edge
// matrices.
//
// Oracles: the reference-triangle stiffness 1/2 [[2,-1,-1],[-1,1,0],[-1,0,1]],
// the P1 mass |T|/12 (1 + delta_ij), the cotangent formula for the Laplace
// stiffness of a general triangle, the integral of phi_i (|T|/3), the edge
// mass L/6 [[2,1],[1,2]] and the edge load L/2.
#include "fem/p1_element.hpp"
#include "fem/quadrature.hpp"
#include "support/check.hpp"
#include <array>
#include <cmath>

using namespace pde2d::fem;
using pde2d::discretization::Point2;
using pde2d_test::check;
using pde2d_test::near;
namespace {
using Matrix3 = std::array<double, 9>;

// Sum the kernel over the degree-2 rule (exact for P1 mass and constants).
p1::LocalBlock integrate(const p1::TriangleGeometry &g,
                         const WeakCoefficients<double> &v) {
  WeakFlags enabled{};
  for (std::size_t k = 0; k < weak::Count; ++k)
    enabled[k] = v[k] != 0;
  p1::LocalBlock out;
  for (const auto &q : triangle_quadrature(2))
    p1::accumulate_element_point(out, v, q.barycentric, g,
                                 g.twice_area / 2 * q.weight,
                                 p1::active_blocks(enabled));
  return out;
}
void near_matrix(const Matrix3 &actual, const Matrix3 &expected) {
  for (std::size_t k = 0; k < actual.size(); ++k)
    near(actual[k], expected[k], 1e-14);
}

void reference_triangle_geometry() {
  const auto g = p1::triangle_geometry({0, 0}, {1, 0}, {0, 1});
  check(g.twice_area == 1, "Reference triangle area");
  check(g.gx == std::array<double, 3>{-1, 1, 0} &&
            g.gy == std::array<double, 3>{-1, 0, 1},
        "Reference basis gradients");
  pde2d_test::expect_throws<std::invalid_argument>(
      [] { p1::triangle_geometry({0, 0}, {0, 1}, {1, 0}); },
      "Unrepresentable triangle area");
}

void reference_stiffness_and_mass() {
  const auto g = p1::triangle_geometry({0, 0}, {1, 0}, {0, 1});
  WeakCoefficients<double> laplace{};
  laplace[weak::DiffusionXX] = laplace[weak::DiffusionYY] = -1;
  const auto k = integrate(g, laplace);
  near_matrix(k.stiffness, {1, -.5, -.5, -.5, .5, 0, -.5, 0, .5});
  check(k.first == Matrix3{} && k.second == Matrix3{} &&
            k.load == std::array<double, 3>{},
        "Laplace touches only the stiffness");

  WeakCoefficients<double> masses{};
  masses[weak::FirstMass] = 1;
  masses[weak::SecondMass] = 3;
  masses[weak::Reaction] = 2;
  const auto m = integrate(g, masses);
  Matrix3 mass;
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j)
      mass[3 * i + j] = (i == j ? 2. : 1.) / 24;
  near_matrix(m.first, mass);
  Matrix3 scaled;
  for (std::size_t k = 0; k < 9; ++k)
    scaled[k] = 3 * mass[k];
  near_matrix(m.second, scaled);
  for (std::size_t k = 0; k < 9; ++k)
    scaled[k] = 2 * mass[k];
  near_matrix(m.stiffness, scaled); // reaction c M
}

void drift_and_load() {
  // A general triangle: int phi_i = |T|/3 for constant data.
  const Point2 a{.2, .1}, b{1.3, .4}, c{.5, 1.1};
  const auto g = p1::triangle_geometry(a, b, c);
  const double area = g.twice_area / 2;
  WeakCoefficients<double> v{};
  v[weak::DriftX] = 2;
  v[weak::DriftY] = -3;
  v[weak::Source] = 6;
  const auto out = integrate(g, v);
  for (std::size_t i = 0; i < 3; ++i) {
    near(out.load[i], area / 3 * 6, 1e-14);
    for (std::size_t j = 0; j < 3; ++j)
      near(out.stiffness[3 * i + j], area / 3 * (2 * g.gx[j] - 3 * g.gy[j]),
           1e-14);
  }
}

void cotangent_formula() {
  // -Laplace on a general triangle: K_ij = -cot(theta_k)/2 for i != j, where
  // theta_k is the angle at the third vertex; rows sum to zero.
  const std::array<Point2, 3> p{Point2{.2, .1}, Point2{1.3, .4},
                                Point2{.5, 1.1}};
  const auto g = p1::triangle_geometry(p[0], p[1], p[2]);
  WeakCoefficients<double> laplace{};
  laplace[weak::DiffusionXX] = laplace[weak::DiffusionYY] = -1;
  const auto k = integrate(g, laplace);
  const auto cot = [&](std::size_t at) {
    const auto &o = p[at], &u = p[(at + 1) % 3], &w = p[(at + 2) % 3];
    const double ux = u.x - o.x, uy = u.y - o.y, wx = w.x - o.x, wy = w.y - o.y;
    return (ux * wx + uy * wy) / std::abs(ux * wy - uy * wx);
  };
  for (std::size_t i = 0; i < 3; ++i) {
    double row = 0;
    for (std::size_t j = 0; j < 3; ++j) {
      row += k.stiffness[3 * i + j];
      if (i != j)
        near(k.stiffness[3 * i + j], -cot(3 - i - j) / 2, 1e-13);
    }
    near(row, 0, 1e-13);
  }
  // Anisotropic A = [[2, .5], [.5, 1]] (a_xy = 1 enters as A_xy = 1/2):
  // K_ij = |T| grad phi_i . A grad phi_j for the operator -div(A grad u).
  WeakCoefficients<double> anisotropic{};
  anisotropic[weak::DiffusionXX] = -2;
  anisotropic[weak::DiffusionXY] = -.5;
  anisotropic[weak::DiffusionYY] = -1;
  const auto ka = integrate(g, anisotropic);
  const double area = g.twice_area / 2;
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j)
      near(ka.stiffness[3 * i + j],
           area * (2 * g.gx[i] * g.gx[j] +
                   .5 * (g.gx[i] * g.gy[j] + g.gy[i] * g.gx[j]) +
                   g.gy[i] * g.gy[j]),
           1e-13);
}

void edge_kernel() {
  const auto frame = p1::edge_frame({0, 0}, {2, 0}, {1, 1});
  check(frame.length == 2 && frame.normal_x == 0 && frame.normal_y == -1,
        "Edge frame: length and outward normal");
  const auto flipped = p1::edge_frame({0, 0}, {2, 0}, {1, -1});
  check(flipped.normal_y == 1, "Normal points away from the inside vertex");
  p1::LocalBlock out;
  const double alpha = 3, gamma = 5, value = 7;
  for (const auto &q : edge_quadrature(2))
    p1::accumulate_edge_point(out, q.coordinate, frame.length * q.weight, alpha,
                              gamma, value);
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) {
      const double mass =
          i < 2 && j < 2 ? frame.length / 6 * (i == j ? 2 : 1) : 0;
      near(out.stiffness[3 * i + j], alpha * mass, 1e-14);
      near(out.first[3 * i + j], gamma * mass, 1e-14);
      check(out.second[3 * i + j] == 0, "Edges have no second mass");
    }
  near(out.load[0], value * frame.length / 2, 1e-14);
  near(out.load[1], value * frame.length / 2, 1e-14);
  check(out.load[2] == 0, "Only the leading 2x2 block is used");
}

} // namespace

int main() {
  return pde2d_test::run(
      {{"reference_triangle_geometry", reference_triangle_geometry},
       {"reference_stiffness_and_mass", reference_stiffness_and_mass},
       {"drift_and_load", drift_and_load},
       {"cotangent_formula", cotangent_formula},
       {"edge_kernel", edge_kernel}});
}

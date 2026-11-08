// Local P1 kernels (p1_element.hpp) against hand-integrated element and edge
// matrices, and the region accumulation of analysis
// (analyze_boundary_region) on synthetic classified edges.
//
// Oracles: the reference-triangle stiffness 1/2 [[2,-1,-1],[-1,1,0],[-1,0,1]],
// the P1 mass |T|/12 (1 + delta_ij), the cotangent formula for the Laplace
// stiffness of a general triangle, the integral of phi_i (|T|/3), the edge
// mass L/6 [[2,1],[1,2]] and the edge load L/2. Region properties are known
// by construction of the synthetic edges.
#include "fem/analysis.hpp"
#include "fem/boundary_assignment.hpp"
#include "fem/p1_element.hpp"
#include "fem/quadrature.hpp"
#include "discretization/square_mesh.hpp"
#include "support/check.hpp"
#include <array>
#include <cmath>
#include <string>
#include <vector>

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

// Synthetic edges on the unit segment of the x axis, outward normal (0,-1).
const p1::EdgeFrame unit_edge = p1::edge_frame({0, 0}, {1, 0}, {0, 1});
ClassifiedEdge dirichlet_edge(Expr alpha, Expr value) {
  return {EdgeCondition{DirichletEdge{alpha, value, value / alpha}}, unit_edge};
}
ClassifiedEdge natural_edge(Expr alpha, Expr gamma, Expr value, Expr beta) {
  return {EdgeCondition{NaturalEdge{alpha, gamma, value, beta}}, unit_edge};
}

void region_accumulation() {
  // Restricted expressions use x as the edge coordinate s in [0, 1].
  {
    const std::vector<ClassifiedEdge> edges{dirichlet_edge(1, x),
                                            dirichlet_edge(2, 0)};
    const auto r = analyze_boundary_region("wall", edges);
    check(r.region == "wall" && r.edges == 2 && r.dirichlet_edges == 2 &&
              r.natural_edges == 0,
          "Dirichlet region counts");
    check(r.form.value == BoundaryForm::Dirichlet &&
              r.form.evidence == Evidence::Proven,
          "Dirichlet region form");
    check(r.divisor_keeps_sign.value &&
              r.divisor_keeps_sign.evidence == Evidence::Proven,
          "Constant divisors are proven");
    check(!r.time_dependent.value &&
              r.time_dependent.evidence == Evidence::Proven,
          "No time dependence");
  }
  {
    // beta = 1 + x is positive but not constant: sampled at every edge sample
    // of both edges.
    const std::vector<ClassifiedEdge> edges{dirichlet_edge(1, 0),
                                            natural_edge(0, 0, sin(t), 1 + x)};
    const auto r = analyze_boundary_region("mixed", edges);
    check(r.form.value == BoundaryForm::Mixed && r.dirichlet_edges == 1 &&
              r.natural_edges == 1,
          "Mixed region");
    check(r.divisor_keeps_sign.value &&
              r.divisor_keeps_sign.evidence == Evidence::Sampled &&
              r.divisor_keeps_sign.reason ==
                  "divisors sampled at " +
                      std::to_string(edge_divisor_samples(2).size() * 2) +
                      " points",
          "Sampled divisor sign");
    check(r.time_dependent.value, "t occurs in the natural value");
  }
  {
    // beta = x - 1/2 changes sign on the edge; the first failure is kept.
    AnalysisOptions options;
    options.sample_times = {0, 1};
    const std::vector<ClassifiedEdge> edges{natural_edge(1, 0, 0, x - .5)};
    const auto r = analyze_boundary_region("flux", edges, options);
    check(r.form.value == BoundaryForm::Natural, "Natural region");
    check(!r.divisor_keeps_sign.value &&
              r.divisor_keeps_sign.evidence == Evidence::Sampled &&
              r.divisor_keeps_sign.reason ==
                  "Natural flux multiplier vanishes or changes sign on "
                  "boundary at t=0",
          "Sign change found at the first sample time");
  }
  {
    const std::vector<ClassifiedEdge> edges{
        dirichlet_edge(1, 0),
        {std::string("rejected for a reason"), unit_edge}};
    const auto r = analyze_boundary_region("bad", edges);
    check(r.edges == 2 && r.form.value == BoundaryForm::Rejected &&
              r.form.evidence == Evidence::Proven &&
              r.form.reason == "rejected for a reason" &&
              r.divisor_keeps_sign.evidence == Evidence::Unknown,
          "Rejected region");
  }
}

void region_accumulation_matches_analyze() {
  // analyze(problem, mesh) forms each region from the same binding and
  // classification; feeding those edges directly gives the same result.
  const auto mesh = pde2d::discretization::square_mesh(3);
  Problem p;
  p.assign(dirichlet(x * y), {"bottom", "left"});
  p.assign(flux_robin(1 + x, 2 + y * y, t), {"top"});
  p.assign(neumann(1), {"right"});
  const auto properties = analyze(p, mesh);
  const auto binding = bind_boundary(mesh, p);
  check(binding.regions ==
                std::vector<std::string>{"bottom", "left", "top", "right"} &&
            binding.edges.size() == mesh.boundary_edges.size(),
        "Binding regions and edges");
  for (std::size_t r = 0; r < binding.regions.size(); ++r) {
    std::vector<ClassifiedEdge> edges;
    for (const auto &e : binding.edges)
      if (e.region == r)
        edges.push_back(
            {classify_edge_condition(p.equation, *e.condition, e.frame),
             e.frame});
    const auto direct = analyze_boundary_region(binding.regions[r], edges);
    const auto &expected = properties.boundaries[r];
    check(direct.region == expected.region && direct.edges == expected.edges &&
              direct.dirichlet_edges == expected.dirichlet_edges &&
              direct.natural_edges == expected.natural_edges &&
              direct.form.value == expected.form.value &&
              direct.form.reason == expected.form.reason &&
              direct.divisor_keeps_sign.value ==
                  expected.divisor_keeps_sign.value &&
              direct.divisor_keeps_sign.evidence ==
                  expected.divisor_keeps_sign.evidence &&
              direct.divisor_keeps_sign.reason ==
                  expected.divisor_keeps_sign.reason &&
              direct.time_dependent.value == expected.time_dependent.value,
          "Direct region analysis differs from analyze()");
  }
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"reference_triangle_geometry", reference_triangle_geometry},
       {"reference_stiffness_and_mass", reference_stiffness_and_mass},
       {"drift_and_load", drift_and_load},
       {"cotangent_formula", cotangent_formula},
       {"edge_kernel", edge_kernel},
       {"region_accumulation", region_accumulation},
       {"region_accumulation_matches_analyze",
        region_accumulation_matches_analyze}});
}

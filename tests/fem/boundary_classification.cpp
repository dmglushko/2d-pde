// classify_edge_condition is pure: these cases need no mesh or assembly.
#include "fem/boundary_classification.hpp"
#include "support/check.hpp"
#include <algorithm>
#include <string>
#include <variant>

using namespace pde2d::fem;
using pde2d_test::check;
using pde2d_test::expect_throws;
using pde2d_test::near;

namespace {
// Bottom edge of the unit square: outward normal (0,-1).
const p1::EdgeFrame bottom = p1::edge_frame({0, 0}, {1, 0}, {0, 1});
// Right edge: outward normal (1,0).
const p1::EdgeFrame right = p1::edge_frame({1, 0}, {1, 1}, {0, 0});

double at(const Expr &restricted, double s, double time = 0) {
  return Evaluator(restricted)({s, 0, time});
}
// The typed alternatives; std::get also checks the classification.
const DirichletEdge &dirichlet_form(const EdgeCondition &c) {
  return std::get<DirichletEdge>(c.form);
}
const NaturalEdge &natural_form(const EdgeCondition &c) {
  return std::get<NaturalEdge>(c.form);
}

void divisor_sign_sampling() {
  // 0, 1/2, 1 and the edge rule, sorted.
  const auto samples = edge_divisor_samples(2);
  check(samples.size() == 5 && samples.front() == 0 && samples.back() == 1 &&
            std::is_sorted(samples.begin(), samples.end()),
        "Divisor samples");
  check(divisor_keeps_strict_sign([](double s) { return 1 + s; }, samples),
        "Positive divisor rejected");
  check(divisor_keeps_strict_sign([](double s) { return -2 - s; }, samples),
        "Negative divisor rejected");
  check(!divisor_keeps_strict_sign([](double s) { return s - .3; }, samples),
        "Sign change accepted");
  check(!divisor_keeps_strict_sign([](double s) { return s; }, samples),
        "Zero at an endpoint accepted");
  // Sampled, not proven: a root strictly between samples is not seen.
  check(divisor_keeps_strict_sign([](double s) { return (s - .3) * (s - .4); },
                                  samples),
        "Expected the documented sampling limitation");
  check(std::string(edge_divisor_name<DirichletEdge>()) == "Dirichlet alpha" &&
            std::string(edge_divisor_name<NaturalEdge>()) ==
                "Natural flux multiplier",
        "Divisor names are part of assembly messages");
}

void edge_frames() {
  near(bottom.length, 1, 1e-15);
  near(bottom.normal_x, 0, 1e-15);
  near(bottom.normal_y, -1, 1e-15);
  near(right.normal_x, 1, 1e-15);
  expect_throws([] { p1::edge_frame({0, 0}, {0, 0}, {1, 1}); }, "Zero");
}

void dirichlet_restricts_to_edge_parameter() {
  const auto c = classify_edge_condition({}, dirichlet(2 * (1 + x + y)), right);
  check(c.is_dirichlet(), "Dirichlet condition misclassified");
  // On the right edge x = 1 and y = s.
  near(at(dirichlet_form(c).prescribed, 0), 4, 1e-14);
  near(at(dirichlet_form(c).prescribed, .5), 5, 1e-14);
  // A scaled Dirichlet operator still prescribes value/alpha.
  const auto scaled = classify_edge_condition({}, robin(4, 0, 8), bottom);
  check(scaled.is_dirichlet(), "Zero-beta Robin should be Dirichlet");
  near(at(dirichlet_form(scaled).prescribed, .3), 2, 1e-14);
  near(at(dirichlet_form(scaled).alpha, .3), 4, 1e-14);
  check(std::string(edge_form_name(scaled.form)) == "Dirichlet",
        "Dirichlet form name");
}

void neumann_and_robin_are_natural() {
  // Default equation is -Laplace(u), so q_n = du/dn and beta reduces to 1.
  const auto n = classify_edge_condition({}, neumann(3), bottom);
  check(!n.is_dirichlet(), "Neumann misclassified");
  near(at(natural_form(n).beta, .5), 1, 1e-14);
  near(at(natural_form(n).value, .5), 3, 1e-14);
  check(std::string(edge_form_name(n.form)) == "Natural", "Natural form name");
  const auto r = classify_edge_condition({}, robin(2, 5, 7 * t), bottom);
  check(!r.is_dirichlet(), "Robin misclassified");
  near(at(natural_form(r).alpha, .2), 2, 1e-14);
  near(at(natural_form(r).beta, .2), 5, 1e-14);
  near(at(natural_form(r).value, .2, 2), 14, 1e-14);
  // Flux form: beta multiplies q_n directly.
  const auto f = classify_edge_condition({}, flux_robin(1, 4, 0), right);
  near(at(natural_form(f).beta, .5), 4, 1e-14);
}

void diffusion_scales_normal_derivative() {
  // -3 Laplace(u): a_xx = a_yy = -3, so A = -3 I and q_n = -n.A grad u =
  // 3 du/dn. A Neumann condition du/dn = g therefore becomes q_n / 3 = g.
  Equation eq;
  eq.a_xx = -3;
  eq.a_yy = -3;
  near(at(natural_form(classify_edge_condition(eq, neumann(1), bottom)).beta,
          .5),
       1. / 3, 1e-14);
  // The sign follows the operator: +3 Laplace(u) flips the natural flux.
  eq.a_xx = 3;
  eq.a_yy = 3;
  near(at(natural_form(classify_edge_condition(eq, neumann(1), bottom)).beta,
          .5),
       -1. / 3, 1e-14);
}

void normals_are_substituted() {
  const auto c = classify_edge_condition({}, neumann(nx + 2 * ny), right);
  near(at(natural_form(c).value, .5), 1, 1e-14);
  const auto d = classify_edge_condition({}, neumann(nx + 2 * ny), bottom);
  near(at(natural_form(d).value, .5), -2, 1e-14);
}

void proportional_partial_derivatives_are_accepted() {
  // u_x on the right edge is the outward normal derivative.
  BoundaryCondition bc;
  bc.beta_x = 1;
  bc.value = 1;
  const auto c = classify_edge_condition({}, bc, right);
  check(!c.is_dirichlet(), "u_x on a vertical edge should be natural");
  near(at(natural_form(c).beta, .5), 1, 1e-14);
}

void oblique_derivatives_are_rejected() {
  BoundaryCondition bc;
  bc.beta_x = 1; // tangential on the bottom edge
  expect_throws<std::invalid_argument>(
      [&] { classify_edge_condition({}, bc, bottom); }, "oblique");
}

const ObliqueEdge &oblique_form(const EdgeCondition &c) {
  return std::get<ObliqueEdge>(c.form);
}

// du/dn + r du/ds with s = (-ny, nx), the counterclockwise tangent.
BoundaryCondition normal_plus_tangential(Expr r) {
  BoundaryCondition bc;
  bc.beta_normal = 1;
  bc.beta_x = -r * ny;
  bc.beta_y = r * nx;
  return bc;
}

void oblique_derivatives_are_decomposed() {
  // -Laplace u: q_n = du/dn. On the bottom edge s = (1, 0), so
  // du/dn + 0.5 du/ds = q_n + 0.5 du/ds: beta = 1, tau = 0.5.
  const auto c =
      classify_edge_condition({}, normal_plus_tangential(.5), bottom);
  check(!c.is_dirichlet() && std::string(edge_form_name(c.form)) == "Oblique",
        "Tangential data should give the oblique form");
  near(at(oblique_form(c).beta, .3), 1, 1e-14);
  near(at(oblique_form(c).tau, .3), .5, 1e-14);
  near(at(oblique_form(c).conormal, .3), 1, 1e-14);
  check(std::string(edge_divisor_name<ObliqueEdge>()) ==
                "Oblique flux multiplier" &&
            &edge_divisor(oblique_form(c)) == &oblique_form(c).beta,
        "Oblique divisor name is part of assembly messages");
  // A variable tangential coefficient is restricted like any other: on the
  // right edge x = 1, y = s and s = (0, 1).
  const auto v =
      classify_edge_condition({}, normal_plus_tangential(1 + y), right);
  near(at(oblique_form(v).tau, .5), 1.5, 1e-14);
}

void anisotropic_normal_derivative_is_oblique() {
  // A = diag(-2, -3), so c = -A n = (2 nx, 3 ny). On the axis-aligned edges
  // n is an eigenvector: du/dn is proportional to q_n (natural form).
  Equation eq;
  eq.a_xx = -2;
  eq.a_yy = -3;
  near(
      at(natural_form(classify_edge_condition(eq, neumann(1), right)).beta, .5),
      .5, 1e-14);
  near(at(natural_form(classify_edge_condition(eq, neumann(1), bottom)).beta,
          .5),
       1. / 3, 1e-14);
  // On the diagonal from (1,0) to (0,1), n = (1,1)/sqrt 2 and
  // n.c = 5/2: D = n = beta c + tau s with beta = 1/(5/2) = 0.4 and
  // tau = det(c, n)/(n.c) = -1/2 / (5/2) = -0.2 (s = (-1,1)/sqrt 2).
  const p1::EdgeFrame diagonal = p1::edge_frame({1, 0}, {0, 1}, {0, 0});
  const auto c = classify_edge_condition(eq, neumann(1), diagonal);
  near(at(oblique_form(c).beta, .5), .4, 1e-14);
  near(at(oblique_form(c).tau, .5), -.2, 1e-14);
  near(at(oblique_form(c).conormal, .5), 2.5, 1e-14);
  near(at(oblique_form(c).value, .5), 1, 1e-14);
  // s follows the outward normal, not the order of the edge's nodes.
  const p1::EdgeFrame reversed = p1::edge_frame({0, 1}, {1, 0}, {0, 0});
  const auto r = classify_edge_condition(eq, neumann(1), reversed);
  near(at(oblique_form(r).tau, .5), -.2, 1e-14);
  // A flux term adds to beta only: beta_flux q_n + du/dn.
  auto bc = neumann(1);
  bc.beta_flux = 2;
  const auto f = classify_edge_condition(eq, bc, diagonal);
  near(at(oblique_form(f).beta, .5), 2.4, 1e-14);
  near(at(oblique_form(f).tau, .5), -.2, 1e-14);
}

void degenerate_oblique_edges_are_rejected() {
  // a_xx = 0, a_xy = 1, a_yy = -1: on the right edge -A n = (0, -1/2) is
  // tangential, so q_n cannot be isolated.
  Equation eq;
  eq.a_xx = 0;
  eq.a_xy = 1;
  expect_throws<std::invalid_argument>(
      [&] { classify_edge_condition(eq, neumann(0), right); }, "conormal");
  // A tangential derivative with a flux term that cancels its normal part.
  auto bc = normal_plus_tangential(1);
  bc.beta_flux = -1;
  expect_throws<std::invalid_argument>(
      [&] { classify_edge_condition({}, bc, bottom); },
      "tangential to the edge");
}

void cancelled_derivatives_reduce_to_dirichlet() {
  BoundaryCondition bc;
  bc.alpha = 2;
  bc.beta_flux = 1;
  bc.beta_normal = -1; // q_n - du/dn = 0 for -Laplace(u)
  bc.value = 6;
  const auto c = classify_edge_condition({}, bc, bottom);
  check(c.is_dirichlet(), "Cancelled derivative terms should leave Dirichlet");
  near(at(dirichlet_form(c).prescribed, .5), 3, 1e-14);
  bc.alpha = 0;
  expect_throws<std::invalid_argument>(
      [&] { classify_edge_condition({}, bc, bottom); }, "no nonzero operator");
}

void dynamic_dirichlet_is_rejected() {
  BoundaryCondition bc;
  bc.alpha = 1;
  bc.gamma = 1;
  expect_throws<std::invalid_argument>(
      [&] { classify_edge_condition({}, bc, bottom); }, "Derivative-free");
  // With a flux term the same gamma is a dynamic natural condition.
  const auto c = classify_edge_condition({}, dynamic_robin(1, 1, 1, 0), bottom);
  check(!c.is_dirichlet(), "Dynamic Robin misclassified");
  near(at(natural_form(c).gamma, .5), 1, 1e-14);
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"edge_frames", edge_frames},
       {"dirichlet_restricts_to_edge_parameter",
        dirichlet_restricts_to_edge_parameter},
       {"neumann_and_robin_are_natural", neumann_and_robin_are_natural},
       {"diffusion_scales_normal_derivative",
        diffusion_scales_normal_derivative},
       {"normals_are_substituted", normals_are_substituted},
       {"proportional_partial_derivatives_are_accepted",
        proportional_partial_derivatives_are_accepted},
       {"oblique_derivatives_are_rejected", oblique_derivatives_are_rejected},
       {"oblique_derivatives_are_decomposed",
        oblique_derivatives_are_decomposed},
       {"anisotropic_normal_derivative_is_oblique",
        anisotropic_normal_derivative_is_oblique},
       {"degenerate_oblique_edges_are_rejected",
        degenerate_oblique_edges_are_rejected},
       {"cancelled_derivatives_reduce_to_dirichlet",
        cancelled_derivatives_reduce_to_dirichlet},
       {"dynamic_dirichlet_is_rejected", dynamic_dirichlet_is_rejected},
       {"divisor_sign_sampling", divisor_sign_sampling}});
}

#pragma once
// Reference problems: complete problems with exact or manufactured
// solutions, the yardstick of docs/vision.md ("'Better' is measured":
// automatic choices are judged against simple defaults on reference
// problems with known solutions, in accuracy for the work spent).
//
//   for (const auto &reference : reference_problems()) { ... }
//   auto heat = find_reference_problem("heat_moving_peak");
//   auto comparison = compare_plans(heat, mesh_plan, {plan_a, plan_b});
//                                                   // reference/compare.hpp
//
// A ReferenceProblem is data: geometry (named boundary regions), the
// Problem, initial conditions and a time window for transient problems, the
// exact solution u(x, y, t) (a stationary one does not depend on t), notes
// on what makes it interesting, and suggested mesh options. Each exact
// solution satisfies its problem's strong form and boundary conditions;
// tests/fem/reference_problems.cpp checks the residuals independently at
// sample points. Names are stable: comparison documents refer to them.
//
// The library is deliberately small and covers the situations automatic
// choices must handle: a corner singularity, a boundary layer, variable
// anisotropic diffusion with flux conditions, oblique conditions, a
// multiply connected domain with mixed conditions, a heat problem with a
// moving peak and a standing wave.
#include "discretization/curve_geometry.hpp"
#include "discretization/geometry.hpp"
#include "discretization/mesh_types.hpp"
#include "fem/evolution.hpp"
#include "fem/expression.hpp"
#include "fem/problem.hpp"
#include <cmath>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pde2d::fem {
// The initial state and time window of a transient reference problem.
struct ReferenceTransient {
  InitialConditions initial{};
  double start = 0, end = 1;
};

struct ReferenceProblem {
  std::string name{};
  discretization::CurveGeometry geometry{};
  Problem problem{};
  // Set for transient problems: evolve from `initial` over [start, end].
  std::optional<ReferenceTransient> transient{};
  // u(x, y, t); compare a stationary solution at t = 0 and an evolution at
  // the end of its window.
  Expr exact = Expr(0);
  std::string notes{};
  // Coarse mesh options that resolve the problem reasonably; comparisons
  // take their own MeshPlan.
  discretization::MeshOptions mesh{};

  bool time_dependent() const { return transient.has_value(); }
  // The exact solution depends on t (true for every transient problem
  // here; a stationary exact solution never does).
  bool exact_time_dependent() const { return depends_on(exact, t); }
};

namespace reference_detail {
namespace disc = pde2d::discretization;
inline disc::CurveGeometry unit_square() {
  using disc::LineCurve;
  return disc::CurveGeometry({{"bottom", {LineCurve{{0, 0}, {1, 0}}}},
                              {"right", {LineCurve{{1, 0}, {1, 1}}}},
                              {"top", {LineCurve{{1, 1}, {0, 1}}}},
                              {"left", {LineCurve{{0, 1}, {0, 0}}}}});
}
inline disc::MeshOptions mesh_options(double h, bool curved = false) {
  disc::MeshOptions o;
  o.boundary.max_edge_length = h;
  o.refinement.max_edge_length = h;
  if (curved)
    o.boundary_treatment = disc::BoundaryTreatment::RefineOnCurves;
  return o;
}
// The source of `exact` under the strong form of `e` (problem.hpp).
inline Expr strong_source(const Equation &e, const Expr &u) {
  const Expr ux = diff(u, x), uy = diff(u, y);
  return e.m2 * diff(diff(u, t), t) + e.m1 * diff(u, t) + e.a_xx * diff(ux, x) +
         e.a_xy * diff(ux, y) + e.a_yy * diff(uy, y) + e.b_x * ux + e.b_y * uy +
         e.c * u;
}
// The natural flux q_n = -n.A grad(u) of `exact` (problem.hpp).
inline Expr natural_flux(const Equation &e, const Expr &u) {
  const Expr ux = diff(u, x), uy = diff(u, y);
  return -nx * (e.a_xx * ux + e.a_xy / 2 * uy) -
         ny * (e.a_xy / 2 * ux + e.a_yy * uy);
}
} // namespace reference_detail

// Laplace equation on the L-shaped domain (-1, 1)^2 without [0, 1] x
// [-1, 0], u = r^(2/3) sin(2 theta / 3) with theta measured from the edge
// along the positive x axis through the domain (0 .. 3 pi / 2). The
// gradient is singular at the reentrant corner: uniform meshes converge at
// rate 2/3 in H1, so graded or adaptive meshes are the point.
inline ReferenceProblem reference_l_shape_corner() {
  using discretization::LineCurve;
  ReferenceProblem r;
  r.name = "l_shape_corner";
  r.geometry = discretization::CurveGeometry(
      {{"boundary",
        {LineCurve{{0, 0}, {1, 0}}, LineCurve{{1, 0}, {1, 1}},
         LineCurve{{1, 1}, {-1, 1}}, LineCurve{{-1, 1}, {-1, -1}},
         LineCurve{{-1, -1}, {0, -1}}, LineCurve{{0, -1}, {0, 0}}}}});
  // theta = arg(z e^{-i a}) + a with a = 0.7 pi: the branch cut of atan2
  // (direction a - pi = -0.3 pi) lies inside the excluded quadrant. The
  // floating coefficients are deliberate: restricted to an edge, symmetric
  // arguments such as atan2(-s, -s) (a = 3 pi / 4) are rewritten by SymEngine
  // to a constant on the wrong branch.
  const double a = .7 * std::numbers::pi, ca = std::cos(a), sa = std::sin(a);
  const Expr re = Expr(ca) * x + Expr(sa) * y,
             im = Expr(-sa) * x + Expr(ca) * y;
  const Expr theta =
      Expr(SymEngine::atan2(im.get_basic(), re.get_basic())) + Expr(a);
  r.exact = pow(x * x + y * y, Expr(1) / 3) * sin(Expr(2) * theta / 3);
  r.problem.assign(dirichlet(r.exact), {"boundary"});
  r.notes = "corner singularity r^(2/3) sin(2 theta/3) at the reentrant "
            "corner (0, 0); H1 rate 2/3 on uniform meshes";
  r.mesh = reference_detail::mesh_options(.25);
  return r;
}

// -eps Laplace(u) + u_x + u_y = g(x) + g(y) on the unit square with
// u = g(x) g(y), g(s) = s - (e^{(s-1)/eps} - e^{-1/eps}) / (1 - e^{-1/eps}),
// eps = 0.05: boundary layers of width eps along x = 1 and y = 1, zero
// Dirichlet data. Nonsymmetric (convection).
inline ReferenceProblem reference_boundary_layer(double eps = .05) {
  if (!(eps > 0))
    throw std::invalid_argument("Boundary layer width must be positive");
  ReferenceProblem r;
  r.name = "boundary_layer";
  r.geometry = reference_detail::unit_square();
  const Expr e(eps), tail = exp(-1 / e);
  const auto g = [&](const Expr &s) {
    return s - (exp((s - 1) / e) - tail) / (1 - tail);
  };
  r.exact = g(x) * g(y);
  auto &eq = r.problem.equation;
  eq.a_xx = -e;
  eq.a_yy = -e;
  eq.b_x = 1;
  eq.b_y = 1;
  eq.source = g(x) + g(y);
  r.problem.assign(dirichlet(0), {"bottom", "right", "top", "left"});
  r.notes = "convection-diffusion boundary layers of width eps = " +
            std::to_string(eps) + " at x = 1 and y = 1";
  r.mesh = reference_detail::mesh_options(.1);
  return r;
}

// -div(K grad u) in nondivergence form with variable, anisotropic K
// (a_xx = -(1 + x^2), a_xy = -0.4 (1 + y), a_yy = -(2 + y)), Dirichlet data
// on the left and bottom and the natural flux q_n = -n.A grad(u) on the
// right and top, on the unit square.
inline ReferenceProblem reference_variable_anisotropic() {
  ReferenceProblem r;
  r.name = "variable_anisotropic_diffusion";
  r.geometry = reference_detail::unit_square();
  r.exact = sin(Expr(1.5) * x + Expr(.5)) * cos(y) + x * y;
  auto &eq = r.problem.equation;
  eq.a_xx = -(1 + x * x);
  eq.a_xy = -Expr(.4) * (1 + y);
  eq.a_yy = -(2 + y);
  eq.source = reference_detail::strong_source(eq, r.exact);
  r.problem.assign(dirichlet(r.exact), {"left", "bottom"});
  r.problem.assign(flux_neumann(reference_detail::natural_flux(eq, r.exact)),
                   {"right", "top"});
  r.notes = "variable anisotropic diffusion with a mixed term; conormal "
            "(natural flux) conditions on two sides";
  r.mesh = reference_detail::mesh_options(.15);
  return r;
}

// -Laplace(u) + u = f on the unit square with the oblique condition
// u + du/dn + r du/ds = value (s = (-ny, nx), r = 0.5 (1 + x y)) on the
// bottom, right and top, and Dirichlet data on the left.
inline ReferenceProblem reference_oblique_square() {
  ReferenceProblem r;
  r.name = "oblique_square";
  r.geometry = reference_detail::unit_square();
  r.exact = sin(Expr(1.3) * x + Expr(.4)) * exp(Expr(.7) * y) + x * y / 2;
  auto &eq = r.problem.equation;
  eq.c = 1;
  eq.source = reference_detail::strong_source(eq, r.exact);
  const Expr tangential = Expr(.5) * (1 + x * y);
  BoundaryCondition bc;
  bc.alpha = 1;
  bc.beta_normal = 1;
  bc.beta_x = -tangential * ny;
  bc.beta_y = tangential * nx;
  bc.value = r.exact + (nx + bc.beta_x) * diff(r.exact, x) +
             (ny + bc.beta_y) * diff(r.exact, y);
  r.problem.assign(bc, {"bottom", "right", "top"});
  r.problem.assign(dirichlet(r.exact), {"left"});
  r.notes = "oblique-derivative condition whose direction is not the "
            "normal; Dirichlet data on one side";
  r.mesh = reference_detail::mesh_options(.15);
  return r;
}

// Poisson equation on the annulus 0.4 < r < 1 (regions "outer", "inner"),
// Dirichlet data outside and the Robin condition 2u + du/dn = g on the hole
// (n points into the hole). Symmetric positive definite.
inline ReferenceProblem reference_annulus_mixed() {
  ReferenceProblem r;
  r.name = "annulus_mixed";
  r.geometry = discretization::AnnulusGeometry(1.0, .4);
  r.exact = exp(Expr(.5) * x) * sin(Expr(1.2) * y) + x * y;
  auto &eq = r.problem.equation;
  eq.source = reference_detail::strong_source(eq, r.exact);
  const Expr du_dn = nx * diff(r.exact, x) + ny * diff(r.exact, y);
  r.problem.assign(dirichlet(r.exact), {"outer"});
  r.problem.assign(robin(2, 1, 2 * r.exact + du_dn), {"inner"});
  r.notes = "multiply connected curved domain; Dirichlet outside, Robin on "
            "the hole";
  r.mesh = reference_detail::mesh_options(.2, true);
  return r;
}

// u_t - Laplace(u) = f on the unit square over [0, 0.6] with the moving
// Gaussian u = exp(-30 ((x - 0.3 - 0.5 t)^2 + (y - 0.5)^2)) and its
// time-dependent Dirichlet data.
inline ReferenceProblem reference_heat_moving_peak() {
  ReferenceProblem r;
  r.name = "heat_moving_peak";
  r.geometry = reference_detail::unit_square();
  const Expr cx = Expr(.3) + Expr(.5) * t, dx = x - cx, dy = y - Expr(.5);
  r.exact = exp(Expr(-30) * (dx * dx + dy * dy));
  auto &eq = r.problem.equation;
  eq.m1 = 1;
  eq.source = reference_detail::strong_source(eq, r.exact);
  r.problem.assign(dirichlet(r.exact), {"bottom", "right", "top", "left"});
  r.transient = ReferenceTransient{{r.exact}, 0, .6};
  r.notes = "heat equation with a Gaussian peak moving from x = 0.3 to 0.6; "
            "time-dependent Dirichlet data";
  r.mesh = reference_detail::mesh_options(.1);
  return r;
}

// u_tt - Laplace(u) = 0 on the unit square over [0, 0.5], the standing
// wave u = cos(sqrt(2) pi t) sin(pi x) sin(pi y), zero Dirichlet data,
// initial velocity 0. Second order in time (needs Newmark).
inline ReferenceProblem reference_wave_standing() {
  ReferenceProblem r;
  r.name = "wave_standing";
  r.geometry = reference_detail::unit_square();
  const Expr pi(SymEngine::pi);
  r.exact = cos(sqrt(Expr(2)) * pi * t) * sin(pi * x) * sin(pi * y);
  r.problem.equation.m2 = 1;
  r.problem.assign(dirichlet(0), {"bottom", "right", "top", "left"});
  r.transient =
      ReferenceTransient{{sin(pi * x) * sin(pi * y), Field(Expr(0))}, 0, .5};
  r.notes = "standing wave; second order in time with an initial velocity";
  r.mesh = reference_detail::mesh_options(.1);
  return r;
}

// The whole library, in this order.
inline std::vector<ReferenceProblem> reference_problems() {
  return {reference_l_shape_corner(),       reference_boundary_layer(),
          reference_variable_anisotropic(), reference_oblique_square(),
          reference_annulus_mixed(),        reference_heat_moving_peak(),
          reference_wave_standing()};
}
// The reference problem called `name`. Throws std::invalid_argument
// ("Unknown reference problem: ...").
inline ReferenceProblem find_reference_problem(std::string_view name) {
  for (auto &r : reference_problems())
    if (r.name == name)
      return std::move(r);
  throw std::invalid_argument("Unknown reference problem: " +
                              std::string(name));
}
} // namespace pde2d::fem

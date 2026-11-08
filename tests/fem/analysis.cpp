// Problem analysis (analysis.hpp). Oracles: problems whose properties are
// known by construction, and the consequences the solve itself draws from
// the same property (a singular stationary solve, the ConjugateGradient
// symmetry check, the assembly's divisor check and caching counters).
#include "fem/analysis.hpp"
#include "fem/assembly.hpp"
#include "fem/report.hpp"
#include "fem/solver.hpp"
#include "discretization/square_mesh.hpp"
#include "support/check.hpp"
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;

const std::initializer_list<std::string> sides{"bottom", "right", "top",
                                               "left"};

Problem poisson() {
  Problem p;
  p.equation.source = 1;
  p.assign(dirichlet(0), sides);
  return p;
}

template <class T>
void expect(const Property<T> &p, const T &value, Evidence evidence,
            const char *what) {
  check(p.value == value && p.evidence == evidence, what);
  check(!p.reason.empty(), "Every property explains itself");
}

void poisson_is_proven_symmetric_elliptic() {
  const auto mesh = square_mesh(3);
  const auto a = analyze(poisson(), mesh);
  expect(a.time_order, 0, Evidence::Proven, "Stationary time order");
  expect(a.symmetric, true, Evidence::Proven, "Laplacian symmetry");
  expect(a.diffusion, Definiteness::NegativeDefinite, Evidence::Proven,
         "-Laplace is elliptic with A = -I");
  expect(a.reaction, Sign::Zero, Evidence::Proven, "No reaction");
  expect(a.has_dirichlet, true, Evidence::Proven, "Dirichlet present");
  expect(a.constant_nullspace, false, Evidence::Proven, "Fixed constant");
  check(a.boundaries.size() == 4, "One entry per region");
  std::size_t edges = 0;
  for (const auto &r : a.boundaries) {
    expect(r.form, BoundaryForm::Dirichlet, Evidence::Proven, "Region form");
    expect(r.divisor_keeps_sign, true, Evidence::Proven, "Constant alpha");
    expect(r.time_dependent, false, Evidence::Proven, "Static condition");
    check(r.dirichlet_edges == r.edges && r.natural_edges == 0, "Counts");
    edges += r.edges;
  }
  check(edges == mesh.boundary_edges.size(), "Every edge is counted once");
  // The same judgement on the assembled matrix, as the CG backend makes it.
  const auto system = analyze(assemble(mesh, poisson()));
  expect(system.symmetric_stiffness, true, Evidence::Sampled,
         "Assembled Laplacian symmetric");
  expect(system.time_order, 0, Evidence::Sampled, "Assembled time order");
  std::ostringstream text;
  write_report(text, a);
  for (const char *token :
       {"FEM problem analysis:", "time_order=0 [Proven]", "symmetric=true",
        "diffusion=NegativeDefinite", "region bottom:", "form=Dirichlet"})
    check(text.str().find(token) != std::string::npos,
          "Analysis report missing field");
}

void drift_breaks_symmetry_as_the_solver_sees_it() {
  const auto mesh = square_mesh(3);
  auto p = poisson();
  p.equation.b_x = 1;
  const auto a = analyze(p, mesh);
  expect(a.symmetric, false, Evidence::Proven, "Constant drift");
  const auto s = assemble(mesh, p);
  expect(analyze(s).symmetric_stiffness, false, Evidence::Sampled,
         "Assembled drift matrix");
  SolverOptions cg;
  cg.method = LinearSolver::ConjugateGradient;
  expect_throws<std::runtime_error>([&] { solve_stationary(mesh, p, {}, cg); },
                                    "symmetric positive");

  // Variable diffusion whose divergence cancels the drift: b - div A = 0.
  Problem q = poisson();
  q.equation.a_xx = -(1 + x);
  q.equation.a_yy = -(1 + x);
  q.equation.b_x = -1;
  const auto b = analyze(q, mesh);
  expect(b.symmetric, true, Evidence::Proven, "Divergence-form operator");
  expect(b.diffusion, Definiteness::NegativeDefinite, Evidence::Sampled,
         "Variable A is sampled");
  expect(analyze(assemble(mesh, q)).symmetric_stiffness, true,
         Evidence::Sampled, "Divergence-form matrix");
  check(solve_stationary(mesh, q, {}, cg).report.converged,
        "CG accepts what analysis calls symmetric");
}

void pure_neumann_nullspace_is_singular() {
  const auto mesh = square_mesh(3);
  Problem p;
  p.assign(flux_neumann(0), sides);
  const auto a = analyze(p, mesh);
  expect(a.has_dirichlet, false, Evidence::Proven, "Pure Neumann");
  expect(a.constant_nullspace, true, Evidence::Proven, "Constant nullspace");
  expect_throws<std::runtime_error>([&] { solve_stationary(mesh, p); },
                                    "Singular");
  p.equation.c = 1;
  expect(analyze(p, mesh).constant_nullspace, false, Evidence::Proven,
         "Reaction removes the nullspace");
  check(solve_stationary(mesh, p).report.converged, "Reaction is solvable");
  p.equation.c = 0;
  p.boundaries.clear();
  p.assign(flux_robin(1, 1, 0), sides);
  expect(analyze(p, mesh).constant_nullspace, false, Evidence::Proven,
         "Robin term removes the nullspace");
  // Mesh-free: flux conditions are recognised from their structure.
  Problem neumann_only;
  neumann_only.assign(flux_neumann(0), sides);
  expect(analyze(neumann_only).constant_nullspace, true, Evidence::Proven,
         "Mesh-free pure flux Neumann");
}

void sampled_and_unknown_evidence() {
  const auto mesh = square_mesh(3);
  Problem p = poisson();
  p.equation.c = x - .5;
  expect(analyze(p, mesh).reaction, Sign::Indefinite, Evidence::Sampled,
         "Sign-changing reaction");
  expect(analyze(p).reaction, Sign::Indefinite, Evidence::Unknown,
         "Mesh-free variable reaction");
  p.equation.c = 1 + x * x;
  expect(analyze(p, mesh).reaction, Sign::Positive, Evidence::Sampled,
         "Positive variable reaction");
  p.equation.c = -2;
  expect(analyze(p).reaction, Sign::Negative, Evidence::Proven,
         "Constant reaction needs no mesh");
  p.equation.m2 = 1 + x;
  expect(analyze(p, mesh).time_order, 2, Evidence::Sampled, "Variable m2");
  expect(analyze(p).time_order, 2, Evidence::Unknown, "Mesh-free m2");
  p.equation.m2 = 0;
  p.equation.m1 = 3;
  expect(analyze(p).time_order, 1, Evidence::Proven, "Constant m1");
  // Coefficients that cannot be evaluated at a sample stay Unknown.
  p.equation.c = 1 / t;
  expect(analyze(p, mesh).reaction, Sign::Indefinite, Evidence::Unknown,
         "Unevaluable sample");
  AnalysisOptions later;
  later.sample_times = {1, 2};
  expect(analyze(p, mesh, later).reaction, Sign::Positive, Evidence::Sampled,
         "Sample times are honoured");
  later.sample_times = {std::numeric_limits<double>::quiet_NaN()};
  expect_throws<std::invalid_argument>([&] { analyze(p, mesh, later); },
                                       "Invalid analysis sample time");
}

void diffusion_definiteness() {
  Problem p = poisson();
  p.equation.a_xx = 1;
  p.equation.a_yy = 1;
  expect(analyze(p).diffusion, Definiteness::PositiveDefinite, Evidence::Proven,
         "+Laplace");
  p.equation.a_yy = -1;
  expect(analyze(p).diffusion, Definiteness::Indefinite, Evidence::Proven,
         "Wave-type operator");
  // A = [[-1, 1], [1, -1]] (a_xy = 2 carries no implicit factor of two).
  p.equation.a_xx = -1;
  p.equation.a_xy = 2;
  expect(analyze(p).diffusion, Definiteness::NegativeSemidefinite,
         Evidence::Proven, "Degenerate diffusion");
  p.equation.a_xx = p.equation.a_xy = p.equation.a_yy = 0;
  expect(analyze(p).diffusion, Definiteness::Zero, Evidence::Proven,
         "First-order operator");
}

void time_dependence_matches_assembly_caching() {
  const auto mesh = square_mesh(3);
  Problem p = poisson();
  p.equation.m1 = 1;
  p.equation.source = sin(t) * x;
  p.equation.c = 1 + t;
  const auto a = analyze(p, mesh);
  check(a.time_dependent.evidence == Evidence::Proven, "Symbolic test");
  std::size_t dynamic = 0;
  for (std::size_t k = 0; k < weak::Count; ++k) {
    const bool expected = k == weak::Source || k == weak::Reaction;
    check(a.time_dependent.value[k] == expected, "Time-dependent terms");
    dynamic += a.time_dependent.value[k];
  }
  // Assembly re-evaluates exactly these terms at every quadrature point.
  const auto stats = PreparedAssembly(mesh, p).statistics();
  check(stats.interior_evaluations_per_assembly ==
            dynamic * stats.physical_quadrature_points,
        "Analysis and assembly disagree about time dependence");
}

void boundary_divisor_matches_assembly() {
  const auto mesh = square_mesh(2);
  for (const auto &[beta, keeps] :
       {std::pair{Expr(2 + x), true}, std::pair{Expr(2 * x - 1), false}}) {
    Problem p;
    p.assign(flux_robin(1, beta, 1), {"bottom"});
    p.assign(dirichlet(0), {"right", "top", "left"});
    const auto a = analyze(p, mesh);
    const auto &bottom = a.boundaries.front();
    check(bottom.region == "bottom", "Region order follows the problem");
    expect(bottom.form, BoundaryForm::Natural, Evidence::Proven, "Robin form");
    expect(bottom.divisor_keeps_sign, keeps, Evidence::Sampled,
           "Sampled divisor sign");
    bool assembled = true;
    try {
      assemble(mesh, p);
    } catch (const std::invalid_argument &e) {
      assembled = false;
      check(std::string(e.what()).find("vanishes or changes sign") !=
                std::string::npos,
            "Unexpected assembly error");
    }
    check(assembled == keeps, "Analysis and assembly disagree on the divisor");
  }
  // A divisor that loses its sign only later in time.
  Problem p;
  p.assign(flux_robin(1, 1 + x - 4 * t, 1), {"bottom"});
  p.assign(dirichlet(0), {"right", "top", "left"});
  AnalysisOptions o;
  o.sample_times = {0, .25};
  const auto late = analyze(p, mesh, o);
  const auto &bottom = late.boundaries.front();
  expect(bottom.divisor_keeps_sign, false, Evidence::Sampled, "Late root");
  expect(bottom.time_dependent, true, Evidence::Proven, "Moving condition");
  check(bottom.divisor_keeps_sign.reason.find("t=0.25") != std::string::npos,
        "Reason names the failing time");
  PreparedAssembly prepared(mesh, p);
  (void)prepared.assemble(0);
  expect_throws<std::invalid_argument>([&] { prepared.assemble(.25); },
                                       "Natural flux multiplier vanishes");
}

void mixed_and_rejected_regions() {
  // One region over two edges: y u_x restricts to zero on the bottom
  // (Dirichlet) but to a flux multiplier -y on the left (natural, vanishing
  // at the corner).
  const pde2d::discretization::TriangleMesh mesh{
      {{0, 0}, {1, 0}, {0, 1}},
      {{0, 1, 2}},
      {{1, 0}, {2, 1}, {0, 2}},
      {"outer", "diagonal", "outer"}};
  Problem p;
  BoundaryCondition bc;
  bc.alpha = 1;
  bc.beta_x = y;
  p.assign(bc, {"outer"});
  p.assign(flux_neumann(0), {"diagonal"});
  const auto a = analyze(p, mesh);
  const auto &outer = a.boundaries.front();
  expect(outer.form, BoundaryForm::Mixed, Evidence::Proven, "Mixed region");
  check(outer.edges == 2 && outer.dirichlet_edges == 1 &&
            outer.natural_edges == 1,
        "Mixed counts");
  expect(outer.divisor_keeps_sign, false, Evidence::Sampled,
         "Vanishing flux multiplier");
  expect(a.has_dirichlet, true, Evidence::Proven, "One Dirichlet edge");
  expect_throws<std::invalid_argument>([&] { assemble(mesh, p); },
                                       "Natural flux multiplier vanishes");

  // Tangential derivatives are rejected by classification, and analysis
  // reports the same message instead of throwing.
  const auto square = square_mesh(2);
  Problem oblique;
  BoundaryCondition tangential;
  tangential.beta_x = 1;
  oblique.assign(tangential, {"bottom"});
  oblique.assign(dirichlet(0), {"right", "top", "left"});
  const auto r = analyze(oblique, square).boundaries.front();
  check(r.form.value == BoundaryForm::Rejected &&
            r.form.reason.find("oblique") != std::string::npos,
        "Rejected region");
  expect_throws<std::invalid_argument>([&] { assemble(square, oblique); },
                                       "oblique");
  // Structural errors are the assembly's errors.
  Problem missing;
  missing.assign(dirichlet(0), {"bottom"});
  expect_throws<std::invalid_argument>([&] { analyze(missing, square); },
                                       "Missing boundary assignment");
}

void oblique_regions_are_reported() {
  // du/dn + 0.5 du/ds on the bottom (s = (-ny, nx)), plain Neumann under
  // anisotropic diffusion on the right (oblique only where n is not an
  // eigenvector of A: never on an axis-aligned edge), Dirichlet elsewhere.
  const auto mesh = square_mesh(3);
  Problem p;
  p.equation.a_xx = -2;
  p.equation.a_yy = -1;
  p.equation.c = 1;
  BoundaryCondition bc = neumann(0);
  bc.beta_x = -.5 * ny;
  bc.beta_y = .5 * nx;
  p.assign(bc, {"bottom"});
  p.assign(neumann(0), {"right"});
  p.assign(dirichlet(0), {"top", "left"});
  const auto a = analyze(p, mesh);
  const auto &bottom = a.boundaries[0];
  expect(bottom.form, BoundaryForm::Oblique, Evidence::Proven,
         "Oblique region form");
  check(bottom.oblique_edges == bottom.edges && bottom.edges == 3 &&
            bottom.natural_edges == 0,
        "Oblique edge counts");
  check(bottom.form.reason.find("well-posedness") != std::string::npos,
        "The region says well-posedness is not analyzed");
  expect(bottom.divisor_keeps_sign, true, Evidence::Proven,
         "Constant oblique divisors");
  expect(a.boundaries[1].form, BoundaryForm::Natural, Evidence::Proven,
         "Axis-aligned anisotropic Neumann stays natural");
  // The interior is self-adjoint, the oblique boundary term is not.
  expect(a.symmetric, false, Evidence::Unknown, "Oblique edges and symmetry");
  check(!numerically_symmetric(assemble(mesh, p).stiffness),
        "Assembled oblique stiffness is nonsymmetric");
  std::ostringstream text;
  write_report(text, a);
  for (const char *token : {"form=Oblique", "oblique_edges=3"})
    check(text.str().find(token) != std::string::npos,
          "Analysis report missing oblique field");
  // Mesh-free: the tangential data may be oblique, the plain Neumann data
  // under isotropic diffusion never are.
  expect(analyze(p).symmetric, false, Evidence::Unknown,
         "Mesh-free oblique symmetry");
  Problem isotropic;
  isotropic.assign(neumann(0), {"bottom"});
  isotropic.assign(dirichlet(0), {"right", "top", "left"});
  expect(analyze(isotropic).symmetric, true, Evidence::Proven,
         "Isotropic Neumann is never oblique");
}

void mesh_free_boundary_forms() {
  Problem p;
  p.assign(dirichlet(x), {"bottom"});
  p.assign(flux_robin(1, 2, 0), {"right"});
  p.assign(neumann(0), {"top", "left"});
  const auto a = analyze(p);
  check(a.boundaries.size() == 4, "One entry per named region");
  expect(a.boundaries[0].form, BoundaryForm::Dirichlet, Evidence::Proven,
         "No derivatives");
  expect(a.boundaries[1].form, BoundaryForm::Natural, Evidence::Proven,
         "Constant flux multiplier");
  // du/dn = q_n / a only after restriction: needs the edges.
  check(a.boundaries[2].form.evidence == Evidence::Unknown,
        "Normal derivative needs a mesh");
  expect(a.has_dirichlet, true, Evidence::Proven, "Mesh-free Dirichlet");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"poisson_is_proven_symmetric_elliptic",
        poisson_is_proven_symmetric_elliptic},
       {"drift_breaks_symmetry_as_the_solver_sees_it",
        drift_breaks_symmetry_as_the_solver_sees_it},
       {"pure_neumann_nullspace_is_singular",
        pure_neumann_nullspace_is_singular},
       {"sampled_and_unknown_evidence", sampled_and_unknown_evidence},
       {"diffusion_definiteness", diffusion_definiteness},
       {"time_dependence_matches_assembly_caching",
        time_dependence_matches_assembly_caching},
       {"boundary_divisor_matches_assembly", boundary_divisor_matches_assembly},
       {"mixed_and_rejected_regions", mixed_and_rejected_regions},
       {"oblique_regions_are_reported", oblique_regions_are_reported},
       {"mesh_free_boundary_forms", mesh_free_boundary_forms}});
}

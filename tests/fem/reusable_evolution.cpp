#include "fem/evolution.hpp"
#include "discretization/square_mesh.hpp"
using pde2d::discretization::square_mesh;
#include "support/check_eigen.hpp"
#include <iostream>
#include <thread>

using namespace pde2d::fem;
using pde2d_test::check;
namespace {
void near(const Vector &a, const Vector &b) {
  pde2d_test::near_vector(a, b, 2e-9);
}
// Uncached reference driver: the same integrators as evolve(), but every
// assembly is a fresh one-shot assemble() and every solve prepares a new
// factorization. Agreement isolates evolve()'s caching (prepared assembly,
// factorization reuse) from the schemes themselves, whose accuracy is
// checked against exact solutions in transient.cpp and moving_convergence.cpp.
std::vector<Frame> uncached(const pde2d::discretization::TriangleMesh &mesh,
                            const Problem &p, const Expr &field,
                            const Expr &velocity, const TimeOptions &o) {
  auto integrator = make_time_integrator(o.method);
  IntegratorServices services;
  services.boundary_tolerance = o.assembly.boundary_tolerance;
  services.solve = [&](const SparseMatrix &a, const Vector &b,
                       const std::vector<DirichletValue> &d, bool) {
    auto [c, r] = constrain(a, b, d);
    auto s = solve_linear(c, r, o.solver);
    check(s.report.converged, "Uncached reference failed");
    return s.values;
  };
  services.start_dirichlet = [&](DirichletDerivative derivative) {
    return PreparedAssembly(mesh, p, o.assembly)
        .assemble(o.start, derivative)
        .dirichlet;
  };
  auto old = assemble(mesh, p, o.start, o.assembly);
  IntegratorState state{interpolate(mesh, field, o.start),
                        interpolate(mesh, velocity, o.start)};
  impose_compatible(state.u, old.dirichlet, o.assembly.boundary_tolerance,
                    "displacement");
  integrator->start(old, state, services);
  std::vector<Frame> out{{o.start, state.u, state.v}};
  TimeGrid grid(o.start, o.end, o.step);
  while (const auto step = grid.next()) {
    auto cur = assemble(mesh, p, step->time, o.assembly);
    auto next = integrator->advance(old, cur, step->h, state, services);
    for (const auto &d : cur.dirichlet)
      next.u[d.node] = d.value;
    state = std::move(next);
    old = std::move(cur);
    out.push_back({step->time, state.u, state.v});
  }
  return out;
}
void repeated_rhs() {
  SparseMatrix a(3, 3);
  std::vector<Eigen::Triplet<double>> entries{
      {0, 0, 4}, {1, 1, 5}, {2, 2, 6}, {0, 1, -1}, {1, 0, -1}};
  a.setFromTriplets(entries.begin(), entries.end());
  for (auto method : {LinearSolver::SparseLU, LinearSolver::ConjugateGradient,
                      LinearSolver::GMRES})
    for (auto validation :
         {SolverValidation::Checked, SolverValidation::AssumeValid}) {
      SolverOptions o;
      o.method = method;
      o.validation = validation;
      PreparedLinearSolver prepared(a, o);
      auto stats = prepared.preparation_timings();
      check(stats.preparations == 1 &&
                stats.rank_checks ==
                    (validation == SolverValidation::Checked ? 1u : 0u) &&
                stats.factorizations == 1,
            "Preparation counters");
      for (double scale : {1., 0., -2., 1e-180}) {
        Vector rhs = Vector::Constant(3, scale);
        auto cached = prepared.solve(rhs), fresh = solve_linear(a, rhs, o);
        check(cached.report.converged == fresh.report.converged,
              "Cached convergence mismatch");
        near(cached.values, fresh.values);
        check(cached.report.timings.preparations == 0 &&
                  cached.report.timings.rank_checks == 0 &&
                  cached.report.timings.factorizations == 0 &&
                  cached.report.timings.rhs_solves == 1,
              "Repeated preparation");
      }
      SparseMatrix changed = a;
      changed.coeffRef(0, 0) += 1;
      check(!prepared.matches(changed, o), "Value change reused factorization");
      check(prepared.matches(a, o), "Equal matrix missed cache");
      SolverOptions other = o;
      other.restart++;
      check(!prepared.matches(a, other), "Option change reused preparation");
      a.coeffRef(0, 0) += 2;
      check(!prepared.matches(a, o), "Caller mutation changed snapshot");
      a.coeffRef(0, 0) -= 2;
      Vector rhs = Vector::Ones(3);
      near(prepared.solve(rhs).values, solve_linear(a, rhs, o).values);
    }
  SparseMatrix singular(3, 3);
  PreparedLinearSolver bad(singular);
  check(!bad.solve(Vector::Zero(3)).report.converged,
        "Prepared singular zero RHS accepted");
}
void evolution() {
  auto mesh = square_mesh(3);
  const Expr phi = 1 + x + 2 * y;
  for (auto solver : {LinearSolver::SparseLU, LinearSolver::ConjugateGradient,
                      LinearSolver::GMRES})
    for (auto validation :
         {SolverValidation::Checked, SolverValidation::AssumeValid})
      for (auto method : {TimeMethod::BackwardEuler, TimeMethod::CrankNicolson,
                          TimeMethod::Newmark})
        for (bool variable : {false, true}) {
          Problem p;
          p.equation.m1 = 1 + (variable ? t * t : Expr(0));
          p.equation.m2 =
              method == TimeMethod::Newmark ? Expr(1) + x / 4 : Expr(0);
          p.equation.c = 1 + x + (variable ? sin(t) : Expr(0));
          p.equation.source = phi * (1 + sin(t) + t * t * t);
          p.assign(dirichlet(phi * (1 + t + t * t)), {"bottom"});
          p.assign(dynamic_flux_robin(.3, 1, 1,
                                      (nx + 2 * ny) * (1 + t + t * t) +
                                          phi * (1 + t + t * t) +
                                          .3 * phi * (1 + 2 * t)),
                   {"right", "top", "left"});
          TimeOptions o;
          o.method = method;
          o.solver.method = solver;
          o.solver.validation = validation;
          o.step = .03;
          o.end = .1;
          auto e = evolve(mesh, p, {phi, phi}, o);
          auto ref = uncached(mesh, p, phi, phi, o);
          check(e.frames.size() == ref.size(), "Frame count mismatch");
          for (std::size_t i = 0; i < ref.size(); ++i) {
            near(e.frames[i].values, ref[i].values);
            near(e.frames[i].velocity, ref[i].velocity);
          }
          const std::size_t prep =
              (variable ? 4 : 2) + (method == TimeMethod::Newmark ? 1 : 0);
          check(e.timings.solver.preparations == prep,
                "Effective matrix invalidation count");
          check(e.timings.time_steps == 4 && e.reports.size() == 4,
                "Step/report counts");
          check(e.timings.solver.rank_checks ==
                        (validation == SolverValidation::Checked ? prep : 0) &&
                    e.timings.solver.factorizations == prep,
                "Rank/factor count");
          check(e.timings.solver_reuses + prep == e.timings.linear_solves,
                "Reuse accounting");
        }
  Problem p;
  p.equation.m1 = 1;
  p.equation.source = sin(t) + t * t * t;
  p.assign(flux_neumann(0), {"bottom", "right", "top", "left"});
  TimeOptions o;
  o.end = .3;
  o.step = .01;
  auto full = evolve(mesh, p, {0, 0}, o);
  // Performance contract: a time-independent matrix is factorized once, even
  // though stamps are computed from the origin and the source varies in t.
  check(full.timings.time_steps == 30 &&
            full.timings.solver.preparations == 1 &&
            full.timings.solver_reuses == full.timings.time_steps - 1,
        "Timestamp ULPs or source variation prevented reuse");
  o.frame_stride = 7;
  o.store_reports = false;
  std::size_t observed = 0;
  const auto caller = std::this_thread::get_id();
  o.observer = [&](const Frame &f) {
    check(std::this_thread::get_id() == caller, "Observer moved threads");
    near(f.values, full.frames.at(observed).values);
    ++observed;
  };
  auto bounded = evolve(mesh, p, {0, 0}, o);
  check(observed == 31 && bounded.frames.size() == 6 && bounded.reports.empty(),
        "Observation/storage semantics");
  check(bounded.frames.front().time == 0 && bounded.frames.back().time == .3,
        "Retention endpoints");
  observed = 0;
  o.observer = [&](const Frame &) {
    if (++observed == 3)
      throw std::runtime_error("observer failure");
  };
  bool threw = false;
  try {
    evolve(mesh, p, {0, 0}, o);
  } catch (const std::runtime_error &e) {
    threw = std::string(e.what()) == "observer failure";
  }
  check(threw, "Observer exception swallowed");
  o.observer = {};
  p.equation.m2 = 1;
  o.method = TimeMethod::Newmark;
  auto wave = evolve(mesh, p, {0, 0}, o);
  // Performance contract: one factorization for the start-up mass solve and
  // one for the constant step matrix, reused for every later step.
  const auto steps = wave.timings.time_steps;
  check(steps == 30 && wave.timings.solver.preparations == 2 &&
            wave.timings.solver_reuses == steps - 1 &&
            wave.timings.linear_solves == steps + 1,
        "Constant wave reuse count");
  check(wave.timings.solver.rhs_solves == wave.timings.linear_solves,
        "Initial solve omitted from timings");
  p.equation.m2 = 0;
  o.method = TimeMethod::BackwardEuler;
  o.end = 0;
  check(evolve(mesh, p, {0, 0}, o).frames.size() == 1, "Zero duration");
  o.frame_stride = 0;
  threw = false;
  try {
    evolve(mesh, p, {0, 0}, o);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "Zero stride accepted");
  if (pde2d::openmp_available()) {
    pde2d::ExecutionOptions parallel;
    parallel.backend = pde2d::ExecutionBackend::OpenMP;
    parallel.threads = 2;
    parallel.min_parallel_size = 0;
    near(interpolate(mesh, exp(x + y), 0, parallel),
         interpolate(mesh, exp(x + y), 0));
    o.frame_stride = 1;
    o.end = .04;
    o.assembly.execution = parallel;
    observed = 0;
    o.observer = [&](const Frame &) {
      check(std::this_thread::get_id() == caller, "Parallel observer thread");
      ++observed;
    };
    auto par = evolve(mesh, p, {0, 0}, o);
    o.assembly.execution = {};
    o.observer = {};
    near(par.frames.back().values,
         evolve(mesh, p, {0, 0}, o).frames.back().values);
    check(observed == 5, "Parallel observer count");
    threw = false;
    try {
      interpolate(mesh, log(x - 2), 0, parallel);
    } catch (const std::exception &) {
      threw = true;
    }
    check(threw, "Parallel initial evaluation failure swallowed");
  }
}
} // namespace
int main() {
  return pde2d_test::run(
      {{"repeated_rhs", repeated_rhs}, {"evolution", evolution}});
}

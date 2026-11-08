// User time integrators run through evolve() via TimeOptions::integrator.
// Oracles: the built-in schemes they reproduce (theta = 1 is backward
// Euler; a delegating wrapper is Newmark) and the second-order temporal rate
// of theta = 1/2 on an exact semidiscrete eigenmode.
#include "fem/evolution.hpp"
#include "fem/report.hpp"
#include "discretization/square_mesh.hpp"
#include "support/check.hpp"
#include "support/check_eigen.hpp"
#include <cmath>
#include <memory>
#include <sstream>

using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;

// The theta method with endpoint mass M1(t+h):
// (M1 + theta h K1) u1 = M1 u0 - (1-theta) h K0 u0 + h (theta F1 + (1-theta)
// F0).
class Theta final : public TimeIntegrator {
  double theta_;

public:
  explicit Theta(double theta) : theta_(theta) {}
  std::string name() const override { return "Theta"; }
  int order() const override { return 1; }
  void check(const AssembledSystem &s) const override {
    if (s.first_mass.norm() == 0 || s.second_mass.norm() != 0)
      throw std::invalid_argument("Theta needs a first-order system");
  }
  IntegratorState advance(const AssembledSystem &old,
                          const AssembledSystem &cur, double h,
                          const IntegratorState &s,
                          const IntegratorServices &services) override {
    SparseMatrix a = cur.first_mass + (theta_ * h) * cur.stiffness;
    Vector rhs = cur.first_mass * s.u -
                 ((1 - theta_) * h) * (old.stiffness * s.u) +
                 h * (theta_ * cur.load + (1 - theta_) * old.load);
    Vector u = services.solve(a, rhs, cur.dirichlet, true);
    Vector v = (u - s.u) / h;
    return {std::move(u), std::move(v)};
  }
};

// A wrapper around a built-in scheme that counts its calls.
class CountingNewmark final : public TimeIntegrator {
  NewmarkIntegrator inner_;
  std::shared_ptr<int> steps_;

public:
  explicit CountingNewmark(std::shared_ptr<int> steps)
      : steps_(std::move(steps)) {}
  int order() const override { return inner_.order(); }
  void check(const AssembledSystem &s) const override { inner_.check(s); }
  void start(const AssembledSystem &s, IntegratorState &state,
             const IntegratorServices &services) override {
    inner_.start(s, state, services);
  }
  IntegratorState advance(const AssembledSystem &old,
                          const AssembledSystem &cur, double h,
                          const IntegratorState &s,
                          const IntegratorServices &services) override {
    ++*steps_;
    return inner_.advance(old, cur, h, s, services);
  }
};

void theta_method() {
  const auto mesh = square_mesh(4);
  Problem p;
  p.equation.m1 = 1 + x;
  p.equation.source = sin(x + t);
  p.assign(dirichlet(x * y * cos(t)), {"bottom", "left"});
  p.assign(neumann(t), {"top", "right"});
  TimeOptions o;
  o.end = .1;
  o.step = .02;
  const auto expected = evolve(mesh, p, {x * y}, o);
  int created = 0;
  o.integrator = [&created] {
    ++created;
    return std::make_unique<Theta>(1.);
  };
  const auto custom = evolve(mesh, p, {x * y}, o);
  check(created == 1, "Integrator factory must run once per evolution");
  check(custom.frames.size() == expected.frames.size(), "Frame count changed");
  for (std::size_t k = 0; k < custom.frames.size(); ++k)
    pde2d_test::near_vector(custom.frames[k].values, expected.frames[k].values,
                            1e-12);
  check(custom.metadata.method == TimeMethod::Custom &&
            custom.metadata.method_name == "Theta",
        "Custom integrator metadata");
  std::ostringstream text;
  write_report(text, custom);
  check(text.str().find("method=Theta") != std::string::npos,
        "Report lost the integrator name");
  check(expected.metadata.method_name == "BackwardEuler",
        "Built-in integrator name");
}

void theta_rate() {
  // One free node gives an exact semidiscrete eigenmode, removing spatial
  // error from the temporal order test.
  const auto mesh = square_mesh(2);
  Problem p;
  p.equation.m1 = 1;
  p.assign(dirichlet(0), {"bottom", "right", "top", "left"});
  const auto a = assemble(mesh, p);
  const double lambda = a.stiffness.coeff(4, 4) / a.first_mass.coeff(4, 4);
  const Expr pi = acos(-1.);
  const Expr initial = sin(pi * x) * sin(pi * y);
  auto error = [&](double h) {
    TimeOptions o;
    o.end = .1;
    o.step = h;
    o.integrator = [] { return std::make_unique<Theta>(.5); };
    const auto e = evolve(mesh, p, {initial}, o);
    // Constant matrix and step: one preparation, then reuse every step.
    check(e.timings.solver_reuses == e.timings.time_steps - 1,
          "Custom integrator lost factorization reuse");
    return std::abs(e.frames.back().values[4] - std::exp(-lambda * o.end));
  };
  check(error(.01) / error(.005) > 3.8, "Theta=1/2 temporal order");
}

void wrapped_newmark() {
  const auto mesh = square_mesh(3);
  Problem p;
  p.equation.m2 = 1;
  p.assign(dirichlet(sin(t) * x), {"bottom", "top"});
  p.assign(neumann(0), {"left", "right"});
  TimeOptions o;
  o.method = TimeMethod::Newmark;
  o.end = .2;
  o.step = .05;
  const InitialConditions initial{x * y * (1 - y), x};
  const auto expected = evolve(mesh, p, initial, o);
  auto steps = std::make_shared<int>(0);
  o.method = TimeMethod::BackwardEuler; // ignored when integrator is set
  o.integrator = [steps] { return std::make_unique<CountingNewmark>(steps); };
  const auto custom = evolve(mesh, p, initial, o);
  check(*steps == static_cast<int>(custom.timings.time_steps),
        "Wrapper not driven by evolve()");
  for (std::size_t k = 0; k < custom.frames.size(); ++k)
    check(custom.frames[k].values == expected.frames[k].values &&
              custom.frames[k].velocity == expected.frames[k].velocity,
          "Wrapped Newmark differs from the built-in scheme");
  expect_throws<std::invalid_argument>(
      [&] { evolve(mesh, p, {x * y * (1 - y)}, o); }, "initial velocity");
}

void invalid_integrators() {
  const auto mesh = square_mesh(2);
  Problem p;
  p.equation.m1 = 1;
  p.assign(dirichlet(0), {"bottom", "right", "top", "left"});
  TimeOptions o;
  o.end = .1;
  o.method = TimeMethod::Custom;
  expect_throws<std::invalid_argument>([&] { evolve(mesh, p, {0}, o); },
                                       "requires TimeOptions::integrator");
  o.integrator = [] { return std::unique_ptr<TimeIntegrator>(); };
  expect_throws<std::invalid_argument>([&] { evolve(mesh, p, {0}, o); },
                                       "returned no integrator");
}
} // namespace

int main() {
  return pde2d_test::run({{"theta_method", theta_method},
                          {"theta_rate", theta_rate},
                          {"wrapped_newmark", wrapped_newmark},
                          {"invalid_integrators", invalid_integrators}});
}

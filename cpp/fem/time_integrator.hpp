#pragma once
// The time integrator interface (TimeIntegrator) for the semi-discrete
// system
//
//   M2 u'' + M1 u' + K u = F(t),   u = g(t) on Dirichlet nodes,
//
// where AssembledSystem supplies K = stiffness, M1 = first_mass,
// M2 = second_mass and F = load at a time stamp. Integrators only form linear
// systems and delegate solving (with Dirichlet lifting and factorization
// reuse) to the driver through IntegratorServices, so the same integrator runs
// unchanged under evolve() and under a test's uncached reference driver.
//
// Steps are transactional: evolve() calls trial() for a candidate step and
// then accept() or reject() (a StepController may reject it, step_control.hpp).
// A trial must not commit internal state; Newmark keeps its new acceleration
// provisional until accept(). advance() is trial() followed by accept(), for
// drivers that never reject.
//
// The built-in schemes are units in detail/time_schemes/ (one per
// TimeMethod); time_integration.hpp includes them and is the header to use.
// A user scheme implements TimeIntegrator and is passed as a factory in
// TimeOptions::integrator (evolution.hpp); it needs no library change.
#include "fem/system.hpp"
#include "provenance.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
// The built-in schemes, one unit each (detail/time_schemes/schemes.hpp
// checks that every value before Custom has its unit). Custom marks
// integrators supplied through TimeOptions::integrator. New values go
// before Custom.
enum class TimeMethod { BackwardEuler, CrankNicolson, Newmark, Custom };

// Displacement and velocity at one time stamp.
struct IntegratorState {
  Vector u, v;
};

struct IntegratorServices {
  // Solve matrix*x = rhs subject to x[node] = value for every constraint.
  // time_step=false marks auxiliary solves (e.g. Newmark start-up), which are
  // excluded from per-step solver reports.
  std::function<Vector(const SparseMatrix &, const Vector &,
                       const std::vector<DirichletValue> &, bool time_step)>
      solve;
  // Dirichlet data of the first or second time derivative at the start time.
  std::function<std::vector<DirichletValue>(DirichletDerivative)>
      start_dirichlet{};
  double boundary_tolerance = 1e-10;
  // The system (Dirichlet values, not derivatives) at any finite time, e.g.
  // a stage of a multistage scheme. evolve() serves it from the same cached
  // PreparedAssembly as the step stamps and counts it in assembly_calls and
  // assembly_seconds. Empty in drivers that do not provide it.
  std::function<AssembledSystem(double time)> assemble{};
};

// Check that constrained entries of v agree with the prescribed data (relative
// to max(1,|v|,|g|)) and then overwrite them exactly.
inline void impose_compatible(Vector &v,
                              const std::vector<DirichletValue> &data,
                              double tolerance, const char *description) {
  for (const auto &d : data) {
    if (std::abs(v[d.node] - d.value) >
        tolerance * std::max({1., std::abs(v[d.node]), std::abs(d.value)}))
      throw std::invalid_argument(
          std::string("Incompatible initial prescribed ") + description);
    v[d.node] = d.value;
  }
}

// Integrators may keep internal state (Newmark keeps the acceleration), so
// use one instance per evolution.
class TimeIntegrator {
public:
  virtual ~TimeIntegrator() = default;
  // Built-in schemes return their TimeMethod; user schemes may keep Custom.
  virtual TimeMethod method() const { return TimeMethod::Custom; }
  // Name used in reports, decisions and the run record's strategies.
  // Built-in schemes return their unit's name; override it in a user scheme.
  virtual std::string name() const { return "Custom"; }
  // Stable description (name + parameters) for run records, when the scheme
  // comes from a factory (TimeOptions::integrator). Without one the record
  // says the integrator is "not replayable". Built-in schemes return theirs.
  virtual std::optional<pde2d::StrategyDescriptor> descriptor() const {
    if (method() == TimeMethod::Custom)
      return std::nullopt;
    return pde2d::StrategyDescriptor{name()};
  }
  // 1: needs u at the start; 2: also needs u_t.
  virtual int order() const = 0;
  // Reject systems this scheme cannot integrate.
  virtual void check(const AssembledSystem &system) const = 0;
  // Called once with the start-time system, after the initial displacement
  // was made compatible with its Dirichlet data.
  virtual void start(const AssembledSystem &, IntegratorState &,
                     const IntegratorServices &) {}
  // Advance from `previous` (stamp t) to `current` (stamp t+h) and commit
  // any internal state. The driver re-imposes Dirichlet displacement on the
  // returned u afterwards.
  virtual IntegratorState advance(const AssembledSystem &previous,
                                  const AssembledSystem &current, double h,
                                  const IntegratorState &state,
                                  const IntegratorServices &services) = 0;

  // Transactional stepping. True when trial() leaves internal state
  // untouched until accept(), so a rejected trial can be discarded. The
  // default (false) suits schemes written before trials existed: evolve()
  // then refuses a controller's rejection instead of continuing from a
  // corrupted state. Stateless schemes may simply return true.
  virtual bool transactional() const { return false; }
  // The candidate state of one step, same arguments as advance(). The
  // default calls advance(), which is correct for stateless schemes.
  virtual IntegratorState trial(const AssembledSystem &previous,
                                const AssembledSystem &current, double h,
                                const IntegratorState &state,
                                const IntegratorServices &services) {
    return advance(previous, current, h, state, services);
  }
  // Commit or discard the internal state of the last trial().
  virtual void accept() {}
  virtual void reject() {}
};

// Creates the integrator of one evolution (integrators may keep state).
using TimeIntegratorFactory = std::function<std::unique_ptr<TimeIntegrator>()>;
} // namespace pde2d::fem

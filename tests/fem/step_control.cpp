// Pluggable step sequences (step_control.hpp). Oracles: the hand-computed
// one-step recurrences of backward Euler and Crank-Nicolson on an exact
// semidiscrete eigenmode (one free node), the default TimeGrid path, and
// the driver's contract checks.
#include "fem/evolution.hpp"
#include "fem/report.hpp"
#include "discretization/square_mesh.hpp"
#include "support/check.hpp"
#include "support/check_eigen.hpp"
#include <cmath>
#include <memory>
#include <sstream>
#include <utility>
#include <vector>

using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;

// What the driver reported back; outlives the controller, which evolve()
// owns and destroys.
struct Accepted {
  std::vector<double> times, previous, current;
};

// Steps from a list of (stamp, h).
class ListController final : public StepController {
  std::vector<TimeStep> steps_;
  std::size_t next_ = 0;
  std::shared_ptr<Accepted> accepted_;

public:
  explicit ListController(std::vector<TimeStep> steps,
                          std::shared_ptr<Accepted> accepted = nullptr)
      : steps_(std::move(steps)), accepted_(std::move(accepted)) {}
  std::string name() const override { return "List"; }
  std::optional<TimeStep> next() override {
    if (next_ == steps_.size())
      return std::nullopt;
    return steps_[next_++];
  }
  void accepted(const StepOutcome &outcome) override {
    if (!accepted_)
      return;
    accepted_->times.push_back(outcome.step.time);
    accepted_->previous.push_back(outcome.previous.u[4]);
    accepted_->current.push_back(outcome.current.u[4]);
  }
};

// stamps .01, .03, .05, .1 with h = .01, .02, .02, .05.
std::vector<TimeStep> nonuniform() {
  return {{1, .01, .01}, {2, .03, .02}, {3, .05, .02}, {4, .1, .05}};
}

struct Mode {
  pde2d::discretization::TriangleMesh mesh = square_mesh(2);
  Problem problem;
  double lambda;
  Expr initial;
  Mode() {
    problem.equation.m1 = 1;
    problem.assign(dirichlet(0), {"bottom", "right", "top", "left"});
    // u' = -lambda u at the single free node (index 4).
    const auto a = assemble(mesh, problem);
    lambda = a.stiffness.coeff(4, 4) / a.first_mass.coeff(4, 4);
    const Expr pi = acos(-1.);
    initial = sin(pi * x) * sin(pi * y);
  }
};

void nonuniform_steps_follow_the_recurrence() {
  const Mode mode;
  for (auto method : {TimeMethod::BackwardEuler, TimeMethod::CrankNicolson}) {
    TimeOptions o;
    o.method = method;
    o.end = .1;
    auto accepted = std::make_shared<Accepted>();
    o.step_controller = [accepted] {
      return std::make_unique<ListController>(nonuniform(), accepted);
    };
    std::vector<double> observed;
    o.observer = [&](const Frame &f) { observed.push_back(f.time); };
    const auto e = evolve(mode.mesh, mode.problem, {mode.initial}, o);
    // Hand-computed amplification factors of each scheme for u' = -lambda u.
    double expected = e.frames.front().values[4];
    std::vector<double> after;
    for (const auto &step : nonuniform()) {
      const double z = step.h * mode.lambda;
      expected *= method == TimeMethod::BackwardEuler
                      ? 1 / (1 + z)
                      : (1 - z / 2) / (1 + z / 2);
      after.push_back(expected);
    }
    check(e.frames.size() == 5 && e.timings.time_steps == 4 &&
              accepted->times.size() == 4,
          "Step count");
    for (std::size_t k = 0; k < after.size(); ++k) {
      pde2d_test::near_rel(e.frames[k + 1].values[4], after[k], 1e-13);
      check(e.frames[k + 1].time == nonuniform()[k].time, "Frame stamps");
      check(accepted->times[k] == nonuniform()[k].time &&
                accepted->current[k] == e.frames[k + 1].values[4] &&
                accepted->previous[k] == e.frames[k].values[4],
            "accepted() must see the accepted states");
    }
    check(observed == std::vector<double>{0, .01, .03, .05, .1},
          "Observer stamps");
    // Performance contract: the matrix depends only on h here, so it is
    // factorized once per run of equal steps (.01 | .02 .02 | .05).
    check(e.timings.solver.preparations == 3 &&
              e.timings.solver_reuses == e.timings.time_steps - 3,
          "Factorization reuse across equal steps");
    const auto &d = e.record.plan.decisions;
    check(d.size() == 5 && d[1].subject == DecisionSubject::TimeSteps &&
              d[1].choice == "List" &&
              d[1].basis == "user-selected (TimeOptions::step_controller)",
          "Step controller decision");
  }
}

void default_controller_is_the_time_grid() {
  const Mode mode;
  Problem p = mode.problem;
  p.equation.source = sin(t) * x * y;
  TimeOptions o;
  o.method = TimeMethod::CrankNicolson;
  o.end = .1;
  o.step = .03; // last step shortened by TimeGrid
  const auto implicit = evolve(mode.mesh, p, {mode.initial}, o);
  o.step_controller = [&] {
    return std::make_unique<UniformStepController>(o.start, o.end, o.step);
  };
  const auto explicit_default = evolve(mode.mesh, p, {mode.initial}, o);
  check(implicit.frames.size() == explicit_default.frames.size(),
        "Frame count");
  for (std::size_t k = 0; k < implicit.frames.size(); ++k)
    check(implicit.frames[k].time == explicit_default.frames[k].time &&
              implicit.frames[k].values == explicit_default.frames[k].values,
          "UniformStepController must be the default path, bit for bit");
  check(implicit.record.plan.decisions[1].choice == "Uniform(step=0.03)" &&
            implicit.record.plan.decisions[1].basis ==
                "user-selected (TimeOptions::step)",
        "Default step decision");
}

void controller_contract() {
  const Mode mode;
  TimeOptions o;
  o.end = .1;
  auto with = [&](std::vector<TimeStep> steps) {
    o.step_controller = [steps] {
      return std::make_unique<ListController>(steps);
    };
    return [&] { evolve(mode.mesh, mode.problem, {mode.initial}, o); };
  };
  expect_throws<std::invalid_argument>(with({{1, .05, .05}}),
                                       "stopped before the end time");
  expect_throws<std::invalid_argument>(with({{1, .2, .2}}), "invalid step");
  expect_throws<std::invalid_argument>(with({{1, .05, .05}, {2, .05, .05}}),
                                       "invalid step");
  expect_throws<std::invalid_argument>(with({{1, .1, 0}}), "invalid step");
  expect_throws<std::invalid_argument>(with({{2, .1, .1}}), "non-consecutive");
  o.step_controller = [] { return std::unique_ptr<StepController>(); };
  expect_throws<std::invalid_argument>(
      [&] { evolve(mode.mesh, mode.problem, {mode.initial}, o); },
      "returned no controller");
  // A user sequence still needs a valid interval; `step` is not used.
  o.step = -1;
  o.end = -1;
  expect_throws<std::invalid_argument>(with({}), "Invalid time interval");
  o.end = 0;
  with({});
  check(evolve(mode.mesh, mode.problem, {mode.initial}, o).frames.size() == 1,
        "Zero duration with an empty step sequence");
}
} // namespace

int main() {
  return pde2d_test::run({{"nonuniform_steps_follow_the_recurrence",
                           nonuniform_steps_follow_the_recurrence},
                          {"default_controller_is_the_time_grid",
                           default_controller_is_the_time_grid},
                          {"controller_contract", controller_contract}});
}

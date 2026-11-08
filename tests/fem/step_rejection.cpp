// Trial, accept and reject of time steps (step_control.hpp,
// time_integration.hpp). Oracles: a controller that rejects every other
// trial and retries it with the same step reproduces the plain run bit for
// bit; a controller that proposes a doubled step, rejects it and retries
// with the plain step reproduces the plain run with that step. Both are run
// with Newmark, whose acceleration must stay provisional until a trial is
// accepted. The rounding bound between h and the stamps is checked against
// TimeGrid, the default step sequence.
#include "fem/evolution.hpp"
#include "discretization/square_mesh.hpp"
#include "support/check.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using pde2d::discretization::square_mesh;
using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
using namespace pde2d::fem;

struct Counts {
  std::size_t trials = 0, rejected = 0, accepted = 0;
};

// The TimeGrid steps; every odd-numbered trial is rejected and retried with
// the same step.
class RetrySame final : public StepController {
  TimeGrid grid_;
  std::optional<TimeStep> last_;
  bool retry_ = false;
  std::shared_ptr<Counts> counts_;

public:
  RetrySame(double start, double end, double step,
            std::shared_ptr<Counts> counts)
      : grid_(start, end, step), counts_(std::move(counts)) {}
  std::string name() const override { return "RetrySame"; }
  std::optional<TimeStep> next() override {
    if (!retry_)
      last_ = grid_.next();
    return last_;
  }
  StepVerdict judge(const StepOutcome &) override {
    retry_ = ++counts_->trials % 2 == 1;
    counts_->rejected += retry_;
    return retry_ ? StepVerdict::Reject : StepVerdict::Accept;
  }
  void accepted(const StepOutcome &) override { ++counts_->accepted; }
};

// Proposes twice the fine step from the last accepted stamp, rejects that
// trial and retries with the fine TimeGrid step.
class Halving final : public StepController {
  TimeGrid fine_;
  double reached_;
  std::size_t index_ = 1;
  bool coarse_ = true;
  std::shared_ptr<Counts> counts_;

public:
  Halving(double start, double end, double step, std::shared_ptr<Counts> counts)
      : fine_(start, end, step), reached_(start), counts_(std::move(counts)) {}
  std::string name() const override { return "Halving"; }
  std::optional<TimeStep> next() override {
    if (!coarse_)
      return fine_.next();
    if (!(reached_ < fine_.end()))
      return std::nullopt;
    const double time = std::min(reached_ + 2 * fine_.step(), fine_.end());
    return TimeStep{index_, time, time - reached_};
  }
  StepVerdict judge(const StepOutcome &) override {
    ++counts_->trials;
    if (!coarse_)
      return StepVerdict::Accept;
    coarse_ = false;
    ++counts_->rejected;
    return StepVerdict::Reject;
  }
  void accepted(const StepOutcome &outcome) override {
    reached_ = outcome.step.time;
    ++index_;
    coarse_ = true;
    ++counts_->accepted;
  }
};

struct Setup {
  pde2d::discretization::TriangleMesh mesh = square_mesh(4);
  Problem problem;
  InitialConditions initial;
  TimeOptions options;
  Setup(TimeMethod method) {
    const Expr pi = acos(-1.);
    if (method == TimeMethod::Newmark)
      problem.equation.m2 = 1;
    else
      problem.equation.m1 = 1;
    problem.equation.source = sin(3 * t) * x * (1 - y);
    problem.assign(dirichlet(t * t * x * y),
                   {"bottom", "right", "top", "left"});
    initial = {sin(pi * x) * sin(pi * y), Expr(0)};
    options.method = method;
    options.end = .1;
    options.step = .01;
  }
};

void same_frames(const Evolution &a, const Evolution &b) {
  check(a.frames.size() == b.frames.size(), "Frame count differs");
  for (std::size_t k = 0; k < a.frames.size(); ++k)
    check(a.frames[k].time == b.frames[k].time &&
              a.frames[k].values == b.frames[k].values &&
              a.frames[k].velocity == b.frames[k].velocity,
          "Rejected trials changed the accepted states");
  check(a.reports.size() == b.reports.size(),
        "Reports of rejected trials were kept");
  for (std::size_t k = 0; k < a.reports.size(); ++k)
    check(a.reports[k].residual_norm == b.reports[k].residual_norm,
          "Step reports differ");
}

void retry_with_same_step_is_bit_identical() {
  for (auto method : {TimeMethod::BackwardEuler, TimeMethod::CrankNicolson,
                      TimeMethod::Newmark}) {
    Setup s(method);
    std::size_t plain_calls = 0;
    s.options.observer = [&](const Frame &) { ++plain_calls; };
    const auto plain = evolve(s.mesh, s.problem, s.initial, s.options);
    auto counts = std::make_shared<Counts>();
    std::vector<double> observed;
    s.options.observer = [&](const Frame &f) { observed.push_back(f.time); };
    s.options.step_controller = [&] {
      return std::make_unique<RetrySame>(s.options.start, s.options.end,
                                         s.options.step, counts);
    };
    const auto retried = evolve(s.mesh, s.problem, s.initial, s.options);
    same_frames(retried, plain);
    const auto &t = retried.timings;
    // Every step was tried twice; only accepted ones count as steps, publish
    // frames or call the observer.
    check(t.time_steps == plain.timings.time_steps &&
              t.rejected_steps == t.time_steps &&
              counts->rejected == t.rejected_steps &&
              counts->accepted == t.time_steps &&
              counts->trials == 2 * t.time_steps,
          "Trial accounting");
    check(observed.size() == plain_calls, "Observer saw a rejected trial");
    // Work contract: one extra assembly and one extra solve per rejected
    // trial; the step matrix never changes, so it is factorized as often as
    // in the plain run.
    check(t.assembly_calls == plain.timings.assembly_calls + t.rejected_steps &&
              t.linear_solves ==
                  plain.timings.linear_solves + t.rejected_steps &&
              t.solver.preparations == plain.timings.solver.preparations,
          "Work of rejected trials");
    check(retried.record.work.rejected_steps == t.rejected_steps &&
              plain.timings.rejected_steps == 0,
          "Rejections in the run record");
  }
}

void rejected_doubled_step_matches_the_fine_run() {
  for (auto method : {TimeMethod::CrankNicolson, TimeMethod::Newmark}) {
    Setup s(method);
    const auto plain = evolve(s.mesh, s.problem, s.initial, s.options);
    auto counts = std::make_shared<Counts>();
    s.options.step_controller = [&] {
      return std::make_unique<Halving>(s.options.start, s.options.end,
                                       s.options.step, counts);
    };
    const auto halved = evolve(s.mesh, s.problem, s.initial, s.options);
    same_frames(halved, plain);
    check(halved.timings.rejected_steps == halved.timings.time_steps &&
              counts->rejected == halved.timings.rejected_steps,
          "Every doubled step was rejected once");
  }
}

// Old-style scheme that does not declare transactional(): rejections must
// be refused rather than continued from a possibly committed state.
class Legacy final : public TimeIntegrator {
  BackwardEulerIntegrator inner_;

public:
  std::string name() const override { return "Legacy"; }
  int order() const override { return 1; }
  void check(const AssembledSystem &s) const override { inner_.check(s); }
  IntegratorState advance(const AssembledSystem &a, const AssembledSystem &b,
                          double h, const IntegratorState &s,
                          const IntegratorServices &services) override {
    return inner_.advance(a, b, h, s, services);
  }
};

void rejection_needs_a_transactional_integrator() {
  Setup s(TimeMethod::BackwardEuler);
  auto counts = std::make_shared<Counts>();
  s.options.integrator = [] { return std::make_unique<Legacy>(); };
  // Without rejections a legacy scheme runs unchanged.
  const auto plain = evolve(s.mesh, s.problem, s.initial, s.options);
  s.options.integrator = {};
  const auto builtin = evolve(s.mesh, s.problem, s.initial, s.options);
  same_frames(plain, builtin);
  s.options.integrator = [] { return std::make_unique<Legacy>(); };
  s.options.step_controller = [&] {
    return std::make_unique<RetrySame>(s.options.start, s.options.end,
                                       s.options.step, counts);
  };
  expect_throws<std::invalid_argument>(
      [&] { evolve(s.mesh, s.problem, s.initial, s.options); },
      "cannot discard one");
}

// Steps from a fixed list.
class ListController final : public StepController {
  std::vector<TimeStep> steps_;
  std::size_t next_ = 0;

public:
  explicit ListController(std::vector<TimeStep> steps)
      : steps_(std::move(steps)) {}
  std::string name() const override { return "List"; }
  std::optional<TimeStep> next() override {
    if (next_ == steps_.size())
      return std::nullopt;
    return steps_[next_++];
  }
};

void step_size_must_agree_with_stamps() {
  const double eps = std::numeric_limits<double>::epsilon();
  // TimeGrid, including snapped and shortened final steps and large
  // origins, stays inside the documented bound.
  struct Grid {
    double start, end, step;
  };
  for (const auto &g :
       {Grid{0, 1, .1}, Grid{0, 1, .3}, Grid{1e6, 1e6 + .05, .01},
        Grid{-1e6, 1e6, 2e6 / 3}, Grid{0, 1e6, 1e6 / 3},
        Grid{1e-9, 2e-9, 1e-10}, Grid{0, .1, .01}, Grid{.05, .1, .01}}) {
    TimeGrid grid(g.start, g.end, g.step);
    double previous = g.start;
    while (const auto step = grid.next()) {
      check(step_size_consistent(*step, previous, g.start, g.end),
            "TimeGrid step outside the documented rounding bound");
      previous = step->time;
    }
  }
  check(step_size_consistent({1, .1, .1 + 4 * eps * .1}, 0, 0, .1),
        "Rounding-level difference accepted");
  check(!step_size_consistent({1, .1, .1 * (1 + 1e-9)}, 0, 0, .1),
        "A relative difference of 1e-9 is not rounding");

  Setup s(TimeMethod::BackwardEuler);
  auto with = [&](std::vector<TimeStep> steps) {
    s.options.step_controller = [steps] {
      return std::make_unique<ListController>(steps);
    };
    return [&] { return evolve(s.mesh, s.problem, s.initial, s.options); };
  };
  expect_throws<std::invalid_argument>(with({{1, .05, .04}, {2, .1, .05}}),
                                       "disagrees with its time stamps");
  expect_throws<std::invalid_argument>(with({{1, .05, .05}, {2, .1, .06}}),
                                       "disagrees with its time stamps");
  // Stamps from decimal literals differ from h by rounding only.
  const auto ok = with({{1, .03, .03}, {2, .07, .04}, {3, .1, .03}})();
  check(ok.timings.time_steps == 3, "Consistent list accepted");
}
// Rejects every trial: without the retry cap evolve() would never return.
class AlwaysReject final : public StepController {
  TimeGrid grid_;
  std::optional<TimeStep> step_;
  std::shared_ptr<Counts> counts_;

public:
  AlwaysReject(double start, double end, double step,
               std::shared_ptr<Counts> counts)
      : grid_(start, end, step), counts_(std::move(counts)) {}
  std::string name() const override { return "AlwaysReject"; }
  std::optional<TimeStep> next() override {
    if (!step_)
      step_ = grid_.next();
    return step_;
  }
  StepVerdict judge(const StepOutcome &) override {
    ++counts_->trials;
    ++counts_->rejected;
    return StepVerdict::Reject;
  }
};

void consecutive_rejections_are_capped() {
  Setup s(TimeMethod::BackwardEuler);
  check(s.options.max_consecutive_rejections ==
                default_max_consecutive_rejections &&
            plan_from_user(s.options).time->max_consecutive_rejections ==
                default_max_consecutive_rejections,
        "Default cap in the options and the plan");
  auto counts = std::make_shared<Counts>();
  s.options.step_controller = [&] {
    return std::make_unique<AlwaysReject>(s.options.start, s.options.end,
                                          s.options.step, counts);
  };
  expect_throws<std::runtime_error>(
      [&] { evolve(s.mesh, s.problem, s.initial, s.options); },
      "AlwaysReject rejected 101 consecutive trial steps at step 1");
  // The cap is exceeded by exactly one trial: the default allows 100.
  check(counts->trials == default_max_consecutive_rejections + 1 &&
            counts->accepted == 0,
        "Trials before the cap");
  // A configured cap; 0 forbids every rejection.
  for (std::size_t cap : {std::size_t{0}, std::size_t{3}}) {
    *counts = {};
    s.options.max_consecutive_rejections = cap;
    expect_throws<std::runtime_error>(
        [&] { evolve(s.mesh, s.problem, s.initial, s.options); },
        "max_consecutive_rejections = " + std::to_string(cap));
    check(counts->trials == cap + 1, "Configured cap");
  }
  // The count restarts after each accepted step: RetrySame rejects every
  // step once, which a cap of 1 allows and a cap of 0 does not.
  s.options.max_consecutive_rejections = 1;
  s.options.step_controller = [&] {
    return std::make_unique<RetrySame>(s.options.start, s.options.end,
                                       s.options.step, counts);
  };
  *counts = {};
  const auto retried = evolve(s.mesh, s.problem, s.initial, s.options);
  check(retried.timings.rejected_steps == retried.timings.time_steps &&
            retried.timings.time_steps == 10,
        "One rejection per step is within a cap of 1");
  s.options.max_consecutive_rejections = 0;
  expect_throws<std::runtime_error>(
      [&] { evolve(s.mesh, s.problem, s.initial, s.options); },
      "consecutive trial steps");
}
} // namespace

int main() {
  return pde2d_test::run(
      {{"retry_with_same_step_is_bit_identical",
        retry_with_same_step_is_bit_identical},
       {"rejected_doubled_step_matches_the_fine_run",
        rejected_doubled_step_matches_the_fine_run},
       {"rejection_needs_a_transactional_integrator",
        rejection_needs_a_transactional_integrator},
       {"step_size_must_agree_with_stamps", step_size_must_agree_with_stamps},
       {"consecutive_rejections_are_capped",
        consecutive_rejections_are_capped}});
}

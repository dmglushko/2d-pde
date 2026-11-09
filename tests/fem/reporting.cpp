#include "fem/fem.hpp"
#include "discretization/square_mesh.hpp"
using pde2d::discretization::square_mesh;
#include "support/check.hpp"
#include <iostream>
#include <sstream>

using namespace pde2d::fem;
using pde2d_test::check;
namespace {
struct CommaDecimal : std::numpunct<char> {
  char do_decimal_point() const override { return ','; }
};
struct ThrowingBuffer : std::streambuf {
  std::streamsize xsputn(const char *, std::streamsize) override {
    throw std::runtime_error("output sentinel");
  }
};
void accounting(const Evolution &e) {
  const auto &t = e.timings;
  for (double value :
       {t.assembly_seconds, t.constraints_seconds, t.solver_seconds,
        t.observer_seconds, t.other_seconds, t.total_seconds,
        t.solver.validation_seconds, t.solver.rank_check_seconds,
        t.solver.factorization_seconds, t.solver.solve_seconds,
        t.solver.residual_seconds, t.solver.total_seconds})
    check(std::isfinite(value) && value >= 0, "Invalid native timing");
  const double sum = t.assembly_seconds + t.constraints_seconds +
                     t.solver_seconds + t.observer_seconds + t.other_seconds;
  check(std::abs(sum - t.total_seconds) <=
            16 * std::numeric_limits<double>::epsilon() *
                std::max(1., t.total_seconds),
        "Evolution phases overlap or omit time");
  check(t.solver.total_seconds <= t.solver_seconds,
        "Nested solver exceeds enclosing phase");
}
template <class Report> std::string formatted(const Report &report) {
  std::ostringstream out;
  out.imbue(std::locale(std::locale::classic(), new CommaDecimal));
  out << std::hex << std::fixed << std::noboolalpha << std::setprecision(2)
      << std::setfill('@');
  out.width(17);
  const auto flags = out.flags();
  const auto locale = out.getloc();
  check(&write_report(out, report) == &out,
        "Formatter did not return caller stream");
  check(out.flags() == flags && out.precision() == 2 && out.fill() == '@' &&
            out.width() == 17 && out.getloc() == locale,
        "Formatter changed caller stream state");
  return out.str();
}
void reports() {
  auto mesh = square_mesh(2);
  Problem p;
  p.equation.m1 = 1;
  p.equation.source = 1;
  p.assign(flux_neumann(0), {"bottom", "right", "top", "left"});
  TimeOptions options;
  options.end = .035;
  options.step = .01;
  options.frame_stride = 3;
  options.store_reports = false;
  std::size_t calls = 0;
  double inside_callback = 0;
  std::vector<double> stamps;
  options.observer = [&](const Frame &f) {
    const auto started = timing::Clock::now();
    ++calls;
    stamps.push_back(f.time);
    check(f.values.allFinite(), "Invalid observed values");
    inside_callback += timing::seconds_since(started);
  };
  const auto evolution = evolve(mesh, p, {0, 0}, options);
  accounting(evolution);
  check(calls == 5 && stamps.front() == 0 && stamps.back() == options.end,
        "Observer frequency changed");
  check(evolution.frames.size() == 3 && evolution.frames[1].time == .03 &&
            evolution.reports.empty(),
        "Stride/report retention changed");
  check(evolution.timings.observer_seconds >= inside_callback,
        "Callback not included in observer timing");
  // First-order accounting: one assembly per stamp, one solve per step.
  const auto &w = evolution.timings;
  check(w.time_steps == 4 && w.assembly_calls == w.time_steps + 1 &&
            w.linear_solves == w.time_steps &&
            w.solver.rhs_solves == w.linear_solves &&
            w.solver.preparations + w.solver_reuses == w.linear_solves,
        "Aggregates lost with store_reports=false");
  const auto &m = evolution.metadata;
  check(m.nodes == mesh.nodes.size() && m.elements == mesh.elements.size() &&
            m.boundary_edges == mesh.boundary_edges.size() &&
            m.quadrature_degree == 2 && m.start == 0 && m.end == options.end &&
            m.step == options.step && m.method == options.method &&
            m.integrator == time_method_name(options.method) &&
            m.solver.method == options.solver.method &&
            m.solver.validation == options.solver.validation,
        "Evolution workload metadata mismatch");
  check(m.execution.backend == pde2d::ExecutionBackend::Serial &&
            m.execution.threads == 0 &&
            m.openmp_available == pde2d::openmp_available(),
        "Default execution metadata mismatch");
  const auto text = formatted(evolution);
  for (const char *token :
       {"FEM evolution:", "selected_backend=Serial", "requested_threads=0",
        "actual team size not measured", "time_steps=4", "stored_reports=0",
        "observer=", "other=", "solver seconds (nested)", "validation=Checked"})
    check(text.find(token) != std::string::npos,
          "Evolution summary missing field");
  options.observer = {};
  options.store_reports = true;
  const auto stored = evolve(mesh, p, {0, 0}, options);
  accounting(stored);
  check(stored.timings.observer_seconds == 0 && stored.reports.size() == 4 &&
            stored.timings.solver.rhs_solves ==
                evolution.timings.solver.rhs_solves &&
            stored.timings.solver.preparations ==
                evolution.timings.solver.preparations,
        "Report retention affected aggregate work");
  for (std::size_t i = 0; i < stored.frames.size(); ++i)
    check((stored.frames[i].values - evolution.frames[i].values).norm() == 0,
          "Observer changed numerical output");
  calls = 0;
  options.observer = [&](const Frame &) {
    if (++calls == 3)
      throw std::runtime_error("observer sentinel");
  };
  bool threw = false;
  try {
    evolve(mesh, p, {0, 0}, options);
  } catch (const std::runtime_error &e) {
    threw = std::string(e.what()) == "observer sentinel";
  }
  check(threw && calls == 3, "Observer exception changed");
  options.end = 0;
  calls = 0;
  auto initial = evolve(mesh, p, {0, 0}, options);
  accounting(initial);
  check(calls == 1 && initial.timings.time_steps == 0 &&
            initial.timings.linear_solves == 0,
        "Initial-only observer/report changed");
  options.observer = {};
  options.end = .02;
  options.method = TimeMethod::Newmark;
  p.equation.m2 = 1;
  auto wave = evolve(mesh, p, {0, 0}, options);
  accounting(wave);
  // Newmark adds one start-up acceleration solve that has no step report.
  check(wave.timings.linear_solves == wave.timings.time_steps + 1 &&
            wave.timings.solver.rhs_solves == wave.timings.linear_solves &&
            wave.reports.size() == wave.timings.time_steps,
        "Newmark initial solve missing from aggregate report");
  if (pde2d::openmp_available()) {
    options.assembly.execution.backend = pde2d::ExecutionBackend::OpenMP;
    options.assembly.execution.threads = 2;
    // Force serial loops despite selected OpenMP; metadata must record policy.
    options.assembly.execution.min_parallel_size = 1000000;
    auto parallel = evolve(mesh, p, {0, 0}, options);
    accounting(parallel);
    const auto &selected = parallel.metadata.execution;
    check(selected.threads == 2 && selected.min_parallel_size == 1000000 &&
              selected.backend == pde2d::ExecutionBackend::OpenMP,
          "Selected policy lost");
    check(formatted(parallel).find("requested_threads=2") != std::string::npos,
          "Requested workers mislabeled");
  }
  SparseMatrix a(2, 2);
  a.setIdentity();
  auto solution = solve_linear(a, Vector::Ones(2));
  const auto linear = formatted(solution);
  check(linear == formatted(solution.report) &&
            linear.find("converged=true") != std::string::npos &&
            linear.find("rhs_solves=1") != std::string::npos,
        "Linear report overloads mismatch");
  check(linear.find("0.000000e+00") != std::string::npos,
        "Caller locale leaked into report");
  ThrowingBuffer buffer;
  std::ostream broken(&buffer);
  broken << std::fixed << std::setprecision(3) << std::setfill('#');
  broken.width(9);
  broken.exceptions(std::ios::badbit);
  const auto flags = broken.flags();
  threw = false;
  try {
    write_report(broken, solution);
  } catch (...) {
    threw = true;
  }
  check(threw && broken.flags() == flags && broken.precision() == 3 &&
            broken.fill() == '#' && broken.width() == 9,
        "Output exception changed formatting state");
  SolverReport failed;
  failed.message = "failure sentinel";
  check(formatted(failed).find("converged=false") != std::string::npos &&
            formatted(failed).find("failure sentinel") != std::string::npos,
        "Failed report missing status");
}
} // namespace
int main() { return pde2d_test::run({{"reports", reports}}); }

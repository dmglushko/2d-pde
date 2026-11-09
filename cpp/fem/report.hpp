#pragma once
// Text summaries of solves and evolutions. Nothing in the library prints
// unless one of these is called.
#include "fem/evolution.hpp"
#include "fem/solver.hpp"
#include <iomanip>
#include <locale>
#include <ostream>
#include <sstream>

namespace pde2d::fem {
namespace report_detail {
inline void solver_timings(std::ostream &out, const SolverTimings &t) {
  out << "  solver work: preparations=" << t.preparations
      << " rank_checks=" << t.rank_checks
      << " factorizations=" << t.factorizations
      << " rhs_solves=" << t.rhs_solves << '\n'
      << "  solver seconds (nested): validation=" << t.validation_seconds
      << " rank_check=" << t.rank_check_seconds
      << " factorization=" << t.factorization_seconds
      << " solve=" << t.solve_seconds << " residual=" << t.residual_seconds
      << " total=" << t.total_seconds << '\n';
}
inline const char *backend_name(pde2d::ExecutionBackend backend) {
  return backend == pde2d::ExecutionBackend::OpenMP ? "OpenMP" : "Serial";
}
// Format into a private stream, then use unformatted output so even width and
// locale on the caller's stream survive, including when output throws.
template <class Format>
inline std::ostream &write(std::ostream &out, Format &&format) {
  std::ostringstream buffer;
  buffer.imbue(std::locale::classic());
  buffer << std::scientific << std::setprecision(6) << std::boolalpha;
  format(buffer);
  const auto text = buffer.str();
  return out.write(text.data(), static_cast<std::streamsize>(text.size()));
}
} // namespace report_detail
// Explicit summaries only: solving/evolving never prints automatically.
inline std::ostream &write_report(std::ostream &out,
                                  const SolverReport &report) {
  return report_detail::write(out, [&](std::ostream &s) {
    s << "FEM linear solver: converged=" << report.converged
      << " iterations=" << report.iterations
      << " residual=" << report.residual_norm
      << " relative_residual=" << report.relative_residual
      << " message=" << report.message
      << " initial_guess=" << initial_guess_name(report.initial_guess) << '\n';
    report_detail::solver_timings(s, report.timings);
  });
}
inline std::ostream &write_report(std::ostream &out,
                                  const LinearSolution &solution) {
  return write_report(out, solution.report);
}
inline std::ostream &write_report(std::ostream &out,
                                  const Evolution &evolution) {
  return report_detail::write(out, [&](std::ostream &s) {
    const auto &m = evolution.metadata;
    const auto &t = evolution.timings;
    s << "FEM evolution: method=" << m.integrator
      << " solver=" << linear_solver_name(m.solver)
      << " preconditioner=" << preconditioner_name(m.solver.preconditioner)
      << " validation=" << solver_validation_name(m.solver.validation) << '\n'
      << "  execution: selected_backend="
      << report_detail::backend_name(m.execution.backend)
      << " requested_threads=" << m.execution.threads
      << " (0=runtime default; actual team size not measured)"
      << " min_parallel_size=" << m.execution.min_parallel_size
      << " openmp_available=" << m.openmp_available << '\n'
      << "  workload: nodes=" << m.nodes << " elements=" << m.elements
      << " boundary_edges=" << m.boundary_edges
      << " quadrature_degree=" << m.quadrature_degree << " start=" << m.start
      << " end=" << m.end << " step=" << m.step << '\n'
      << "  work: time_steps=" << t.time_steps
      << " assembly_calls=" << t.assembly_calls
      << " linear_solves=" << t.linear_solves
      << " solver_reuses=" << t.solver_reuses
      << " retained_frames=" << evolution.frames.size()
      << " stored_reports=" << evolution.reports.size() << '\n'
      << "  seconds: assembly=" << t.assembly_seconds
      << " constraints=" << t.constraints_seconds
      << " solver=" << t.solver_seconds << " observer=" << t.observer_seconds
      << " other=" << t.other_seconds << " total=" << t.total_seconds << '\n';
    report_detail::solver_timings(s, t.solver);
  });
}
} // namespace pde2d::fem

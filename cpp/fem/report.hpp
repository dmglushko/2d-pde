#pragma once
// Text summaries of solves and evolutions. Nothing in the library prints
// unless one of these is called.
#include "fem/evolution.hpp"
#include "fem/record.hpp"
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
    const auto &r = evolution.record;
    const auto time = r.time.value_or(TimeRecord{});
    const auto &execution = r.assembly.execution;
    const auto &t = evolution.timings;
    s << "FEM evolution: method=" << time.integrator.name
      << " solver=" << linear_solver_name(r.solver)
      << " preconditioner=" << preconditioner_name(r.solver.preconditioner)
      << " validation=" << solver_validation_name(r.solver.validation) << '\n'
      << "  execution: selected_backend="
      << report_detail::backend_name(execution.backend)
      << " requested_threads=" << execution.threads
      << " (0=runtime default; actual team size not measured)"
      << " min_parallel_size=" << execution.min_parallel_size
      << " openmp_available=" << r.openmp_available << '\n'
      << "  workload: nodes=" << r.workload.nodes
      << " elements=" << r.workload.elements
      << " boundary_edges=" << r.workload.boundary_edges
      << " quadrature_degree=" << r.assembly.quadrature_degree
      << " start=" << time.start << " end=" << time.end << " step=" << time.step
      << '\n'
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
// What a record identifies: kind, replayability, strategies, fingerprints,
// versions and work (record_json.hpp writes everything).
inline std::ostream &write_report(std::ostream &out, const RunRecord &r) {
  return report_detail::write(out, [&](std::ostream &s) {
    s << "FEM run record: kind=" << (r.time ? "evolution" : "stationary")
      << " replayable=" << r.replayable() << '\n'
      << "  strategies: linear_solver=" << r.linear_solver.name
      << " user_supplied=" << r.linear_solver.user_supplied;
    if (r.time)
      s << " time_integrator=" << r.time->integrator.name
        << " user_supplied=" << r.time->integrator.user_supplied;
    s << '\n'
      << "  inputs: mesh=" << r.mesh_fingerprint
      << " problem=" << r.problem_fingerprint
      << " mesh_run=" << (r.mesh_run ? "attached" : "none") << '\n'
      << "  versions: pde2d=" << r.versions.library.version
      << " git_sha=" << r.versions.library.git_sha
      << " eigen=" << r.versions.eigen << " symengine=" << r.versions.symengine
      << '\n'
      << "  work: assembly_calls=" << r.work.assembly_calls
      << " linear_solves=" << r.work.linear_solves
      << " solver_preparations=" << r.work.solver_preparations
      << " solver_reuses=" << r.work.solver_reuses
      << " time_steps=" << r.work.time_steps
      << " solver_iterations=" << r.work.solver_iterations
      << " total_seconds=" << r.timings.total_seconds << '\n';
  });
}
} // namespace pde2d::fem

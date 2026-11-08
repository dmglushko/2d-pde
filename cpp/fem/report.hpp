#pragma once
// Text summaries of solves, evolutions, decisions, run records and analyses.
// Nothing in the library prints unless one of these is called.
#include "fem/analysis.hpp"
#include "fem/decision.hpp"
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
inline void decision(std::ostream &out, const Decision &d) {
  out << "  decision: " << d.subject << '=' << d.choice << " basis=\""
      << d.basis << "\" evidence=" << evidence_name(d.evidence)
      << " source=" << decision_source_name(d.source) << '\n';
}
inline void decisions(std::ostream &out, const std::vector<Decision> &list) {
  for (const auto &d : list)
    decision(out, d);
}
inline const char *backend_name(pde2d::ExecutionBackend backend) {
  return backend == pde2d::ExecutionBackend::OpenMP ? "OpenMP" : "Serial";
}
inline void solver_options(std::ostream &out, const SolverOptions &o) {
  out << "solver=" << linear_solver_name(o)
      << " preconditioner=" << preconditioner_name(o.preconditioner)
      << " validation=" << solver_validation_name(o.validation)
      << " relative_tolerance=" << o.relative_tolerance
      << " absolute_tolerance=" << o.absolute_tolerance
      << " max_iterations=" << o.max_iterations << " restart=" << o.restart;
}
inline void record(std::ostream &out, const RunRecord &r) {
  const auto &e = r.execution;
  const auto &a = r.plan.assembly;
  out << "  record execution: selected_backend="
      << backend_name(e.selected.backend)
      << " requested_threads=" << e.selected.threads
      << " min_parallel_size=" << e.selected.min_parallel_size
      << " openmp_available=" << e.openmp_available << '\n'
      << "  record workload: nodes=" << r.workload.nodes
      << " elements=" << r.workload.elements
      << " boundary_edges=" << r.workload.boundary_edges << '\n'
      << "  record plan: quadrature_degree=" << a.quadrature_degree
      << " boundary_tolerance=" << a.boundary_tolerance << ' ';
  solver_options(out, r.plan.solver);
  out << '\n';
  if (const auto &t = r.plan.time)
    out << "  record time: method="
        << (t->integrator ? "user" : time_method_name(t->method))
        << " start=" << t->start << " end=" << t->end << " step=" << t->step
        << " step_controller=" << (t->step_controller ? "user" : "uniform")
        << '\n';
  const auto &w = r.work;
  out << "  record work: assembly_calls=" << w.assembly_calls
      << " linear_solves=" << w.linear_solves
      << " solver_preparations=" << w.solver_preparations
      << " solver_reuses=" << w.solver_reuses << " time_steps=" << w.time_steps
      << " rejected_steps=" << w.rejected_steps << '\n'
      << "  record evidence: proven=" << r.evidence.proven
      << " sampled=" << r.evidence.sampled << " unknown=" << r.evidence.unknown
      << " weakest=" << evidence_name(r.evidence.weakest) << '\n';
}
template <class T, class Name>
void property(std::ostream &out, const char *label, const Property<T> &p,
              Name &&name) {
  out << "  " << label << '=' << name(p.value) << " ["
      << evidence_name(p.evidence) << "] " << p.reason << '\n';
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
    report_detail::decisions(s, report.decisions);
  });
}
inline std::ostream &write_report(std::ostream &out,
                                  const LinearSolution &solution) {
  return write_report(out, solution.report);
}
// The solve, then its run record (plan, execution, workload, work, evidence).
inline std::ostream &write_report(std::ostream &out,
                                  const StationarySolution &solution) {
  write_report(out, solution.report);
  return report_detail::write(
      out, [&](std::ostream &s) { report_detail::record(s, solution.record); });
}
inline std::ostream &write_report(std::ostream &out, const RunRecord &record) {
  return report_detail::write(out, [&](std::ostream &s) {
    s << "FEM run record:\n";
    report_detail::record(s, record);
    report_detail::decisions(s, record.plan.decisions);
  });
}
inline std::ostream &write_report(std::ostream &out,
                                  const Evolution &evolution) {
  return report_detail::write(out, [&](std::ostream &s) {
    const auto &m = evolution.metadata;
    const auto &t = evolution.timings;
    s << "FEM evolution: method="
      << (m.method_name.empty() ? time_method_name(m.method) : m.method_name)
      << " solver=" << linear_solver_name(m.solver)
      << " preconditioner=" << preconditioner_name(m.solver.preconditioner)
      << " validation=" << solver_validation_name(m.solver.validation) << '\n'
      << "  execution: selected_backend="
      << (m.execution.backend == pde2d::ExecutionBackend::OpenMP ? "OpenMP"
                                                                 : "Serial")
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
      << " rejected_steps=" << t.rejected_steps
      << " retained_frames=" << evolution.frames.size()
      << " stored_reports=" << evolution.reports.size() << '\n'
      << "  seconds: assembly=" << t.assembly_seconds
      << " constraints=" << t.constraints_seconds
      << " solver=" << t.solver_seconds << " observer=" << t.observer_seconds
      << " other=" << t.other_seconds << " total=" << t.total_seconds << '\n';
    report_detail::solver_timings(s, t.solver);
    report_detail::decisions(s, m.decisions);
    report_detail::record(s, evolution.record);
  });
}
inline std::ostream &write_report(std::ostream &out, const Decision &decision) {
  return report_detail::write(
      out, [&](std::ostream &s) { report_detail::decision(s, decision); });
}
inline std::ostream &write_report(std::ostream &out,
                                  const ProblemProperties &properties) {
  return report_detail::write(out, [&](std::ostream &s) {
    using report_detail::property;
    const auto &p = properties;
    const auto integer = [](int v) { return v; };
    const auto boolean = [](bool v) { return v; };
    const auto sign = [](Sign v) { return sign_name(v); };
    s << "FEM problem analysis:\n";
    property(s, "time_order", p.time_order, integer);
    property(s, "first_mass", p.first_mass, sign);
    property(s, "second_mass", p.second_mass, sign);
    property(s, "time_dependent", p.time_dependent, [](const WeakFlags &flags) {
      std::string terms;
      for (bool f : flags)
        terms += f ? '1' : '0';
      return terms;
    });
    property(s, "symmetric", p.symmetric, boolean);
    property(s, "diffusion", p.diffusion,
             [](Definiteness v) { return definiteness_name(v); });
    property(s, "reaction", p.reaction, sign);
    property(s, "has_dirichlet", p.has_dirichlet, boolean);
    property(s, "constant_nullspace", p.constant_nullspace, boolean);
    for (const auto &r : p.boundaries) {
      s << "  region " << r.region << ": edges=" << r.edges
        << " dirichlet_edges=" << r.dirichlet_edges
        << " natural_edges=" << r.natural_edges
        << " oblique_edges=" << r.oblique_edges << '\n';
      property(s, "  form", r.form,
               [](BoundaryForm v) { return boundary_form_name(v); });
      property(s, "  divisor_keeps_sign", r.divisor_keeps_sign, boolean);
      property(s, "  time_dependent", r.time_dependent, boolean);
    }
  });
}
} // namespace pde2d::fem

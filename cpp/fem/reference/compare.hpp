#pragma once
// Comparing named configurations on a reference problem
// (reference/problems.hpp): mesh the reference geometry once, run each
// configuration on that mesh, measure its error against the exact solution
// and keep its run record, so a choice can be judged in accuracy for the
// work spent (docs/vision.md).
//
//   auto reference = find_reference_problem("annulus_mixed");
//   SolverOptions cg;
//   cg.method = LinearSolver::ConjugateGradient;
//   auto c = compare(reference, reference.mesh,
//                    {{"LU", {}, {}}, {"CG", {}, cg}});
//   c.entries[k].errors.l2;          // L2 / H1-seminorm error (error_norms)
//   c.entries[k].record.work;        // solves, iterations, ... (RunRecord)
//   c.entries[k].record.mesh_run;    // the meshing call, attached
//   write_json(out, c);              // schema "pde2d.comparison"
//
// Stationary problems are solved at t = 0 (solve_stationary). Transient
// ones are evolved over the reference window with the configuration's
// TimeOptions, whose start and end must equal the window; its assembly and
// solver options are replaced by the configuration's (one source per
// setting), only the final frame is kept and no reports are stored. The
// error is measured at the end of the window. Nothing here changes a
// numerical result: every entry is the plain public solve of its inputs.
#include "../../json.hpp"
#include "discretization/mesh_record_json.hpp"
#include "discretization/triangulate.hpp"
#include "fem/error_norms.hpp"
#include "fem/evolution.hpp"
#include "fem/record.hpp"
#include "fem/record_json.hpp"
#include "fem/reference/problems.hpp"
#include "fem/solver.hpp"
#include <cstddef>
#include <format>
#include <istream>
#include <limits>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pde2d::fem {
// One named way to solve: the options of a stationary solve, plus time
// settings for a transient reference problem.
struct ComparisonConfiguration {
  std::string name{};
  AssemblyOptions assembly{};
  SolverOptions solver{};
  // Required for transient problems, rejected for stationary ones.
  std::optional<TimeOptions> time{};
};
struct ComparisonOptions {
  // Quadrature exactness degree of the error norms (error_norms.hpp).
  int error_degree = 8;
  // Solve on a mesh that misses its size target; otherwise compare throws.
  bool accept_unsatisfied = false;
};
// The error of one run against the exact solution at `time`.
struct ComparisonErrors {
  double l2 = 0, h1_seminorm = 0;
  double time = 0;
};
struct ComparisonEntry {
  std::string name{};
  // What ran, its work and timings, with the mesh run record attached.
  RunRecord record{};
  ComparisonErrors errors{};
};
struct Comparison {
  std::string reference{}; // ReferenceProblem::name
  std::string notes{};
  std::vector<ComparisonEntry> entries{};
};

namespace comparison_detail {
inline ComparisonEntry run(const ReferenceProblem &reference,
                           const discretization::MeshResult &mesh,
                           const ComparisonConfiguration &configuration,
                           const ComparisonOptions &options) {
  ComparisonEntry entry;
  entry.name = configuration.name;
  Vector values;
  if (const auto &window = reference.transient) {
    if (!configuration.time)
      throw std::invalid_argument("Reference problem " + reference.name +
                                  " is transient; configuration " +
                                  configuration.name + " has no time options");
    auto time = *configuration.time;
    if (time.start != window->start || time.end != window->end)
      throw std::invalid_argument(std::format(
          "Configuration {} time window [{}, {}] does not match reference "
          "problem {} window [{}, {}]",
          configuration.name, time.start, time.end, reference.name,
          window->start, window->end));
    time.assembly = configuration.assembly;
    time.solver = configuration.solver;
    time.frame_stride = std::numeric_limits<std::size_t>::max();
    time.store_reports = false;
    auto evolution =
        evolve(mesh.mesh, reference.problem, window->initial, time);
    values = std::move(evolution.frames.back().values);
    entry.errors.time = evolution.frames.back().time;
    entry.record = std::move(evolution.record);
  } else {
    if (configuration.time)
      throw std::invalid_argument("Reference problem " + reference.name +
                                  " is stationary; configuration " +
                                  configuration.name + " has time options");
    auto solution =
        solve_stationary(mesh.mesh, reference.problem, configuration.assembly,
                         configuration.solver);
    values = std::move(solution.values);
    entry.record = std::move(solution.record);
  }
  attach_mesh_record(entry.record, mesh);
  const auto norms =
      PreparedErrorNorms(mesh.mesh, reference.exact, options.error_degree)
          .evaluate(values, entry.errors.time,
                    configuration.assembly.execution);
  entry.errors.l2 = norms.l2;
  entry.errors.h1_seminorm = norms.h1_seminorm;
  return entry;
}
} // namespace comparison_detail

// Mesh the reference geometry with `mesh_options` (generate_mesh) and run
// every configuration on it, in order. Throws std::runtime_error
// ("Reference mesh misses its size target: ...") unless
// options.accept_unsatisfied, std::invalid_argument for time options that
// do not fit the problem (none for a transient problem, any for a
// stationary one, another window), and what the solves throw.
inline Comparison
compare(const ReferenceProblem &reference,
        const discretization::MeshOptions &mesh_options,
        const std::vector<ComparisonConfiguration> &configurations,
        const ComparisonOptions &options = {}) {
  const auto mesh =
      discretization::generate_mesh(reference.geometry, mesh_options);
  if (!mesh.report.size_satisfied && !options.accept_unsatisfied)
    throw std::runtime_error("Reference mesh misses its size target: " +
                             mesh.report.message);
  Comparison comparison{reference.name, reference.notes, {}};
  for (const auto &configuration : configurations)
    comparison.entries.push_back(
        comparison_detail::run(reference, mesh, configuration, options));
  return comparison;
}

inline constexpr const char *comparison_schema = "pde2d.comparison";
inline constexpr int comparison_schema_version = 1;

// The JSON document of a comparison (conventions of record_json.hpp; each
// entry embeds its run record under "record").
inline JsonValue to_json(const Comparison &c) {
  auto v = JsonValue::object();
  v.set("schema", json_string(comparison_schema));
  v.set("schema_version", json_number(comparison_schema_version));
  v.set("reference", json_string(c.reference));
  v.set("notes", json_string(c.notes));
  auto entries = JsonValue::array();
  for (const auto &e : c.entries) {
    auto entry = JsonValue::object();
    entry.set("name", json_string(e.name));
    auto errors = JsonValue::object();
    errors.set("l2", json_number(e.errors.l2));
    errors.set("h1_seminorm", json_number(e.errors.h1_seminorm));
    errors.set("time", json_number(e.errors.time));
    entry.set("errors", std::move(errors));
    entry.set("record", to_json(e.record));
    entries.push(std::move(entry));
  }
  v.set("entries", std::move(entries));
  return v;
}
inline std::ostream &write_json(std::ostream &out, const Comparison &c) {
  return pde2d::write_json(out, to_json(c));
}
// Parse a comparison document and check its schema. Throws
// std::invalid_argument ("Invalid JSON: ...", "Not a comparison",
// "Unsupported comparison schema version: N").
inline JsonValue read_comparison_json(std::istream &in) {
  return json_detail::read_schema(in, comparison_schema,
                                  comparison_schema_version, "comparison");
}
} // namespace pde2d::fem

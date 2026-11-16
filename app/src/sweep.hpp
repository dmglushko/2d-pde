#pragma once

#include "options.hpp"

#include <span>
#include <string>
#include <string_view>
#include <vector>

// --sweep PATH=v1,v2,... of the command line: a case run once per
// combination of values, in one session so stages a combination does not
// change are reused (docs/app.md, "The command line").
namespace pde2d::app {

class Session;

struct SweepAxis {
  std::string path;
  std::vector<std::string> values; // each read like a --set VALUE
};

// "PATH=v1,v2,..." (values separated by commas, at least one). Throws
// std::invalid_argument for a malformed argument.
SweepAxis parse_sweep_argument(std::string_view argument);

// Runs `case_json` once per combination (the first axis varies slowest)
// and returns {"axes": [paths], "rows": [...]}: per run "values" (path ->
// value), "ok", "nodes", "longest_edge", "solve_seconds" (the solve
// stage's reports), with view.exact_solution in the case "max_abs",
// "max_rms", "max_l2" (over the snapshots) and, from the second row on
// where the mesh changed, "rate_l2" and "rate_max": the observed order
// log(e_prev / e) / log(h_prev / h) with h the longest edge; and "error"
// for a run that failed. Throws PathError for a path that does not exist.
Json run_sweep(Session &session, const Json &case_json,
               std::span<const SweepAxis> axes);

// The sweep as a text table (one row per run) and as CSV.
std::string sweep_table(const Json &sweep);
std::string sweep_csv(const Json &sweep);

} // namespace pde2d::app

#pragma once

#include "domain_input.hpp"
#include "options.hpp"
#include "runs.hpp"
#include "stage_cache.hpp"

#include "pde2d/fem/assembled_problem.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>

// The files the app exports (docs/app.md, "Exports"): one table of kinds,
// each with what it needs and its writer. Every file says what produced
// it (provenance), in the comment form its format has.
namespace pde2d::app {

// A file to save. Small files carry their `content`; large ones (a
// solution) a `stream` that writes it piece by piece, so the server can
// send it without building one big string.
//
// An archive (solution-vtk.zip) also has `members`: its files one by one,
// so the command line can write them into a directory instead of the
// archive.
struct ExportedFile {
  using Sink = std::function<void(std::string_view)>;
  using MemberSink =
      std::function<void(std::string_view name, std::string_view content)>;
  std::string name;
  std::string content_type;
  std::string content;
  std::function<void(const Sink &)> stream;        // empty: use content
  std::function<void(const MemberSink &)> members; // an archive's files
  // The whole file (runs the stream).
  std::string text() const;
};

enum class ExportKind {
  mesh_pde2d,
  mesh_svg,
  geometry_svg,
  stiffness_mtx,
  damping_mtx,
  mass_mtx,
  rhs_mtx,
  solution_csv,
  solution_vtk,
  solution_vtk_zip,
  solution_mesh_pde2d,
  provenance_json,
};

// What a kind needs before it can be written.
enum class ExportNeeds {
  domain,    // the geometry stage
  mesh,      // the mesh stage
  assembly,  // the assembled system at assembly.t
  done_run,  // a run of the case that is done (never computed here)
  case_only, // the case, and the run when one of the case is done
};

// What the writers get; only what the kind needs is set.
struct ExportInputs {
  const Json *case_json = nullptr;
  Json provenance; // provenance_json
  const Domain *domain = nullptr;
  std::shared_ptr<const MeshOutcome> mesh;
  const fem::AssembledProblem *assembled = nullptr;
  // A done run of the case (its mesh and values, no expressions) and its
  // summary (progress_json without the values' layout, with "started"),
  // or nothing.
  std::optional<RunSnapshots> run;
  Json run_summary;
};

struct ExportInfo {
  ExportKind kind;
  std::string_view name; // the file name and the URL's last part
  std::string_view content_type;
  ExportNeeds needs;
  ExportedFile (*write)(const ExportInfo &, const ExportInputs &);
};

std::span<const ExportInfo> export_kinds();
// The kind called `name`, or null.
const ExportInfo *find_export(std::string_view name);
// "mesh.pde2d, mesh.svg, ..." for messages.
std::string export_names();

// A time as "2026-11-16T12:00:00Z".
std::string utc_time(std::chrono::system_clock::time_point time);

// The app's and the library's versions.
std::string_view app_version();
std::string_view library_version();

// The case's content hash: SHA-256 of its canonical JSON (keys sorted, no
// spaces), as 64 hex digits.
std::string case_hash(const Json &case_json);

// What produced an export: {"app" ({"name", "version", "commit",
// "dirty"}), "library", "case_name" (or null), "case_sha256", "exported"
// (UTC), "file", "settings" (the case sections the kind is built from; an
// SVG or mesh text is replaced by its length), "run" ({"id", "state",
// "started"} or null)}. `run` is that object or null.
Json provenance_json(const ExportInfo &info, const Json &case_json,
                     std::string_view case_name, const Json &run);

} // namespace pde2d::app

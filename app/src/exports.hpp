#pragma once

#include "domain_input.hpp"
#include "options.hpp"
#include "runs.hpp"
#include "stage_cache.hpp"

#include "pde2d/fem/assembled_problem.hpp"

#include <functional>
#include <memory>
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
struct ExportedFile {
  using Sink = std::function<void(std::string_view)>;
  std::string name;
  std::string content_type;
  std::string content;
  std::function<void(const Sink &)> stream; // empty: use content
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
  std::shared_ptr<const Run> run;
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

// The app's and the library's versions.
std::string_view app_version();
std::string_view library_version();

// What produced an export: {"app", "library", "case_name" (or null),
// "exported" (UTC), "stage", "settings" (the case sections the kind is
// built from; an SVG or mesh text is replaced by its length), "run" ({"id",
// "state", "started"} or null)}.
Json provenance_json(const ExportInfo &info, const Json &case_json,
                     std::string_view case_name, const Run *run);

} // namespace pde2d::app

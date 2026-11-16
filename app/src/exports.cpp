#include "exports.hpp"

#include "app_error.hpp"
#include "build_info.hpp"
#include "sha256.hpp"
#include "views.hpp"
#include "zip.hpp"

#include "pde2d/io/pde2d_mesh.hpp"
#include "pde2d/io/svg.hpp"

#include <array>
#include <chrono>
#include <format>
#include <sstream>
#include <utility>

#ifndef PDE2D_VERSION
#define PDE2D_VERSION "unknown"
#endif

namespace pde2d::app {

std::string utc_time(std::chrono::system_clock::time_point time) {
  return std::format("{:%FT%TZ}",
                     std::chrono::floor<std::chrono::seconds>(time));
}

namespace {

// The geometry section without its document: an SVG or mesh text is
// replaced by its length, so the settings stay one readable line.
Json geometry_settings(const Json &geometry) {
  Json out = geometry;
  if (out.is_object())
    for (const char *key : {"svg", "text"})
      if (out.contains(key) && out[key].is_string())
        out[key] = std::format("<{} characters>",
                               out[key].get_ref<const std::string &>().size());
  return out;
}

// "commit 0123abc..., uncommitted changes" for the provenance lines.
std::string build_text() {
  const BuildInfo build = build_info();
  if (build.commit == "unknown")
    return "commit unknown";
  std::string out = "commit " + std::string(build.commit);
  if (build.dirty == "true")
    out += ", with uncommitted changes";
  else if (build.dirty == "unknown")
    out += ", changes unknown";
  return out;
}

// The provenance as comment lines (without the comment mark).
std::vector<std::string> provenance_lines(const Json &provenance) {
  std::vector<std::string> lines;
  lines.push_back(
      std::format("produced by pde2d_app {} ({}) (library pde2d {})",
                  app_version(), build_text(), library_version()));
  if (provenance["case_name"].is_string())
    lines.push_back("case: " + provenance["case_name"].get<std::string>());
  lines.push_back("case sha256: " +
                  provenance["case_sha256"].get<std::string>());
  lines.push_back("exported: " + provenance["exported"].get<std::string>());
  if (const Json &run = provenance["run"]; run.is_object())
    lines.push_back(std::format(
        "run: {} ({}, started {})", run["id"].get<std::uint64_t>(),
        run["state"].get<std::string>(), run["started"].get<std::string>()));
  lines.push_back("settings: " + provenance["settings"].dump());
  return lines;
}

std::string comment_block(const Json &provenance, std::string_view mark) {
  std::string out;
  for (const std::string &line : provenance_lines(provenance))
    out += std::format("{} {}\n", mark, line);
  return out;
}

ExportedFile text_file(const ExportInfo &info, std::string content) {
  return {.name = std::string(info.name),
          .content_type = std::string(info.content_type),
          .content = std::move(content),
          .stream = {},
          .members = {}};
}

// An SVG with the provenance as an XML comment after the declaration.
std::string with_svg_comment(std::string svg, const Json &provenance) {
  std::string comment = "<!--\n";
  for (std::string line : provenance_lines(provenance)) {
    // "--" may not appear inside an XML comment.
    for (std::size_t at = line.find("--"); at != std::string::npos;
         at = line.find("--", at))
      line.replace(at, 2, "- -");
    comment += "  " + line + "\n";
  }
  comment += "-->\n";
  std::size_t at = 0;
  if (svg.starts_with("<?xml"))
    at = svg.find('\n') == std::string::npos ? svg.size() : svg.find('\n') + 1;
  svg.insert(at, comment);
  return svg;
}

ExportedFile write_mesh_pde2d(const ExportInfo &info, const ExportInputs &in) {
  // The .pde2d format stays exactly the old one (the viewer reads it), so
  // it has no place for provenance; provenance.json carries it.
  std::ostringstream out;
  staged("export", [&] { io::export_pde2d(*in.mesh->mesh, out); });
  return text_file(info, out.str());
}

ExportedFile write_mesh_svg(const ExportInfo &info, const ExportInputs &in) {
  std::ostringstream out;
  staged("export", [&] { io::export_mesh_svg(*in.mesh->mesh, out); });
  return text_file(info, with_svg_comment(out.str(), in.provenance));
}

ExportedFile write_geometry_svg(const ExportInfo &info,
                                const ExportInputs &in) {
  if (!in.domain->geometry)
    throw StageError("export", "input",
                     "an imported mesh has no curve geometry to export");
  std::ostringstream out;
  staged("export", [&] { io::export_svg(*in.domain->geometry, out); });
  return text_file(info, with_svg_comment(out.str(), in.provenance));
}

// A free part of an operator in Matrix Market coordinate form (real,
// general, 1-based), `size` x `size`; a zero operator without a pattern
// has no entries.
std::string matrix_market(const fem::CsrMatrix &matrix, std::size_t size,
                          std::string_view what, const Json &provenance) {
  std::string out = "%%MatrixMarket matrix coordinate real general\n";
  out += std::format("% pde2d: {}\n", what);
  out += comment_block(provenance, "%");
  const fem::SparsityPattern *pattern = matrix.pattern.get();
  const std::size_t nonzeros = pattern ? pattern->nonzeros() : 0;
  out += std::format("{} {} {}\n", size, size, nonzeros);
  if (pattern)
    for (std::size_t row = 0; row < pattern->rows; ++row)
      for (auto slot = pattern->row_offsets[row];
           slot < pattern->row_offsets[row + 1]; ++slot) {
        const auto index = static_cast<std::size_t>(slot);
        out += std::format("{} {} {}\n", row + 1,
                           pattern->column_indices[index] + 1,
                           matrix.values[index]);
      }
  return out;
}

ExportedFile write_operator(const ExportInfo &info, const ExportInputs &in) {
  const fem::AssembledProblem &assembled = *in.assembled;
  const std::size_t size = assembled.meshed_problem()->free_nodes().size();
  const std::string at = std::format("at t = {}", assembled.time());
  if (info.kind == ExportKind::rhs_mtx) {
    std::string out = "%%MatrixMarket matrix array real general\n";
    out += std::format("% pde2d: right-hand side b {}\n", at);
    out += comment_block(in.provenance, "%");
    const auto rhs = assembled.right_hand_side();
    out += std::format("{} 1\n", rhs.size());
    for (const double value : rhs)
      out += std::format("{}\n", value);
    return text_file(info, std::move(out));
  }
  const auto [op,
              what] = [&]() -> std::pair<const fem::Operator *, const char *> {
    switch (info.kind) {
    case ExportKind::stiffness_mtx:
      return {&assembled.stiffness(), "stiffness A"};
    case ExportKind::mass_mtx:
      return {&assembled.mass(), "mass M2"};
    case ExportKind::damping_mtx:
      return {&assembled.damping(), "damping B"};
    default:
      throw std::logic_error("not an operator export");
    }
  }();
  return text_file(info,
                   matrix_market(op->free, size,
                                 std::format("{}, free part, {}", what, at),
                                 in.provenance));
}

// "x,y,u(t=...),..." with one row per mesh node, after "#" comment lines,
// written row by row.
ExportedFile write_solution_csv(const ExportInfo &info,
                                const ExportInputs &in) {
  const RunSnapshots run = *in.run;
  std::string header = comment_block(in.provenance, "#");
  header += "x,y";
  for (const Snapshot &solution : run.snapshots)
    header += std::format(",u(t={})", solution->time);
  header += '\n';
  ExportedFile file = text_file(info, {});
  // The stream holds the mesh and the values (no expressions), so it may
  // run after the session's mutex is released; a kept snapshot does not
  // change.
  file.stream = [run, header](const ExportedFile::Sink &sink) {
    sink(header);
    std::string rows;
    const mesh::Mesh &mesh = *run.mesh;
    for (std::size_t i = 0; i < mesh.nodes.size(); ++i) {
      rows += std::format("{},{}", mesh.nodes[i].x, mesh.nodes[i].y);
      for (const Snapshot &solution : run.snapshots)
        rows += std::format(",{}", solution->values[i]);
      rows += '\n';
      if (rows.size() > 1 << 16) {
        sink(rows);
        rows.clear();
      }
    }
    sink(rows);
  };
  return file;
}

// The VTK title line: what produced the file, at most 255 characters on
// one line (legacy VTK has no comments; provenance.json has the rest).
std::string vtk_title(const ExportInputs &in) {
  const BuildInfo build = build_info();
  std::string title = std::format(
      "pde2d_app {} ({}{}) library {}, run {}, case sha256 {}, exported {}",
      app_version(), build.commit.substr(0, 12),
      build.dirty == "true" ? "+changes" : "", library_version(), in.run->id,
      in.provenance["case_sha256"].get<std::string>().substr(0, 16),
      in.provenance["exported"].get<std::string>());
  if (in.provenance["case_name"].is_string())
    title += ", case " + in.provenance["case_name"].get<std::string>();
  for (char &c : title)
    if (c == '\n' || c == '\r')
      c = ' ';
  if (title.size() > 255)
    title.resize(255);
  return title;
}

// A VTK legacy unstructured grid: the mesh (z = 0), triangles, and one
// point-data array per given snapshot, named `name(k)`.
template <class Name>
void write_vtk(const ExportedFile::Sink &sink, const mesh::Mesh &mesh,
               const std::string &title, std::span<const Snapshot> arrays,
               Name name) {
  std::string out = "# vtk DataFile Version 3.0\n" + title +
                    "\nASCII\nDATASET UNSTRUCTURED_GRID\n";
  const auto flush = [&] {
    if (out.size() > 1 << 16) {
      sink(out);
      out.clear();
    }
  };
  out += std::format("POINTS {} double\n", mesh.nodes.size());
  for (const geometry::Point &p : mesh.nodes) {
    out += std::format("{} {} 0\n", p.x, p.y);
    flush();
  }
  const std::size_t triangles = mesh.triangles.size();
  out += std::format("CELLS {} {}\n", triangles, 4 * triangles);
  for (const mesh::Triangle &t : mesh.triangles) {
    out += std::format("3 {} {} {}\n", t.nodes[0], t.nodes[1], t.nodes[2]);
    flush();
  }
  out += std::format("CELL_TYPES {}\n", triangles);
  for (std::size_t k = 0; k < triangles; ++k) {
    out += "5\n"; // VTK_TRIANGLE
    flush();
  }
  out += std::format("POINT_DATA {}\n", mesh.nodes.size());
  for (std::size_t k = 0; k < arrays.size(); ++k) {
    out += std::format("SCALARS {} double 1\nLOOKUP_TABLE default\n",
                       name(*arrays[k]));
    for (const double value : arrays[k]->values) {
      out += std::format("{}\n", value);
      flush();
    }
  }
  sink(out);
}

// One file: the mesh and an array per kept snapshot, named u(t=...).
ExportedFile write_solution_vtk(const ExportInfo &info,
                                const ExportInputs &in) {
  const RunSnapshots run = *in.run;
  ExportedFile file = text_file(info, {});
  file.stream = [run, title = vtk_title(in)](const ExportedFile::Sink &sink) {
    write_vtk(sink, *run.mesh, title, run.snapshots,
              [](const solve::Solution &solution) {
                return std::format("u(t={})", solution.time);
              });
  };
  return file;
}

// provenance.json: the provenance without its settings line, the case as
// sent and the run's summary.
Json provenance_document(const ExportInputs &in) {
  Json out = Json::object();
  out["format"] = "pde2d-app-provenance";
  out["version"] = 1;
  for (const auto &[key, value] : in.provenance.items())
    if (key != "settings")
      out[key] = value;
  out["case"] = *in.case_json;
  if (in.run)
    out["run"] = in.run_summary;
  return out;
}

// The name of snapshot k's file in the series: solution_0000.vtk, ...
std::string series_member(std::size_t k) {
  return std::format("solution_{:04}.vtk", k);
}

// A time series for ParaView: one VTK file per snapshot (the mesh and u),
// solution.vtk.series (ParaView's JSON manifest: each file with its time)
// and solution-vtk.provenance.json, as members of a stored zip.
ExportedFile write_solution_vtk_zip(const ExportInfo &info,
                                    const ExportInputs &in) {
  const RunSnapshots run = *in.run;
  const std::string title = vtk_title(in);
  Json provenance = provenance_document(in);
  ExportedFile file = text_file(info, {});
  file.members = [run, title, provenance = provenance.dump(2) + "\n"](
                     const ExportedFile::MemberSink &sink) {
    Json files = Json::array();
    for (std::size_t k = 0; k < run.snapshots.size(); ++k) {
      std::string text;
      write_vtk([&](std::string_view piece) { text += piece; }, *run.mesh,
                std::format("{}, t = {}", title, run.snapshots[k]->time)
                    .substr(0, 255),
                std::span(run.snapshots).subspan(k, 1),
                [](const solve::Solution &) { return std::string("u"); });
      sink(series_member(k), text);
      Json item = Json::object();
      item["name"] = series_member(k);
      item["time"] = run.snapshots[k]->time;
      files.push_back(std::move(item));
    }
    Json series = Json::object();
    series["file-series-version"] = "1.0";
    series["files"] = std::move(files);
    sink("solution.vtk.series", series.dump(2) + "\n");
    sink("solution-vtk.provenance.json", provenance);
  };
  file.stream = [members = file.members](const ExportedFile::Sink &sink) {
    ZipWriter zip(sink);
    members([&](std::string_view name, std::string_view content) {
      zip.add(name, content);
    });
    zip.finish();
  };
  return file;
}

ExportedFile write_provenance(const ExportInfo &info, const ExportInputs &in) {
  return text_file(info, provenance_document(in).dump(2) + "\n");
}

constexpr std::array<ExportInfo, 11> kinds{{
    {ExportKind::mesh_pde2d, "mesh.pde2d", "text/plain", ExportNeeds::mesh,
     write_mesh_pde2d},
    {ExportKind::mesh_svg, "mesh.svg", "image/svg+xml", ExportNeeds::mesh,
     write_mesh_svg},
    {ExportKind::geometry_svg, "geometry.svg", "image/svg+xml",
     ExportNeeds::domain, write_geometry_svg},
    {ExportKind::stiffness_mtx, "stiffness.mtx", "text/plain",
     ExportNeeds::assembly, write_operator},
    {ExportKind::damping_mtx, "damping.mtx", "text/plain",
     ExportNeeds::assembly, write_operator},
    {ExportKind::mass_mtx, "mass.mtx", "text/plain", ExportNeeds::assembly,
     write_operator},
    {ExportKind::rhs_mtx, "rhs.mtx", "text/plain", ExportNeeds::assembly,
     write_operator},
    {ExportKind::solution_csv, "solution.csv", "text/csv",
     ExportNeeds::done_run, write_solution_csv},
    {ExportKind::solution_vtk, "solution.vtk", "text/plain",
     ExportNeeds::done_run, write_solution_vtk},
    {ExportKind::solution_vtk_zip, "solution-vtk.zip", "application/zip",
     ExportNeeds::done_run, write_solution_vtk_zip},
    {ExportKind::provenance_json, "provenance.json", "application/json",
     ExportNeeds::case_only, write_provenance},
}};

} // namespace

std::string ExportedFile::text() const {
  if (!stream)
    return content;
  std::string out;
  stream([&](std::string_view piece) { out += piece; });
  return out;
}

std::span<const ExportInfo> export_kinds() { return kinds; }

const ExportInfo *find_export(std::string_view name) {
  for (const ExportInfo &info : kinds)
    if (info.name == name)
      return &info;
  return nullptr;
}

std::string export_names() {
  std::string out;
  for (const ExportInfo &info : kinds)
    out += (out.empty() ? "" : ", ") + std::string(info.name);
  return out;
}

std::string_view app_version() { return PDE2D_VERSION; }
std::string_view library_version() { return PDE2D_VERSION; }

std::string case_hash(const Json &case_json) {
  // nlohmann::json keeps its keys sorted, ordered_json as written.
  return sha256_hex(nlohmann::json(case_json).dump());
}

Json provenance_json(const ExportInfo &info, const Json &case_json,
                     std::string_view case_name, const Json &run) {
  const BuildInfo build = build_info();
  Json app = Json::object();
  app["name"] = "pde2d_app";
  app["version"] = app_version();
  app["commit"] = build.commit;
  app["dirty"] =
      build.dirty == "unknown" ? Json(nullptr) : Json(build.dirty == "true");
  Json library = Json::object();
  library["name"] = "pde2d";
  library["version"] = library_version();

  Json settings = Json::object();
  settings["geometry"] = geometry_settings(member(case_json, "geometry"));
  const auto take = [&](const char *name) {
    settings[name] = member(case_json, name);
  };
  switch (info.needs) {
  case ExportNeeds::domain:
    break;
  case ExportNeeds::mesh:
    take("mesh");
    break;
  case ExportNeeds::assembly:
    take("mesh");
    take("problem");
    take("assembly");
    break;
  case ExportNeeds::done_run:
  case ExportNeeds::case_only:
    take("mesh");
    take("problem");
    take("assembly");
    take("solve");
    break;
  }

  Json out = Json::object();
  out["app"] = std::move(app);
  out["library"] = std::move(library);
  out["case_name"] = case_name.empty() ? Json(nullptr) : Json(case_name);
  out["case_sha256"] = case_hash(case_json);
  out["exported"] = utc_time(std::chrono::system_clock::now());
  out["file"] = info.name;
  out["settings"] = std::move(settings);
  out["run"] = run;
  return out;
}

} // namespace pde2d::app

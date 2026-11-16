#include "exports.hpp"

#include "app_error.hpp"
#include "views.hpp"

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

namespace {

std::string utc(std::chrono::system_clock::time_point time) {
  return std::format("{:%FT%TZ}",
                     std::chrono::floor<std::chrono::seconds>(time));
}

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

// The provenance as comment lines (without the comment mark).
std::vector<std::string> provenance_lines(const Json &provenance) {
  std::vector<std::string> lines;
  lines.push_back(std::format("produced by pde2d_app {} (library pde2d {})",
                              app_version(), library_version()));
  if (provenance["case_name"].is_string())
    lines.push_back("case: " + provenance["case_name"].get<std::string>());
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
          .stream = {}};
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
  auto mesh = in.run->meshed->mesh();
  auto solutions = in.run->solutions;
  std::string header = comment_block(in.provenance, "#");
  header += "x,y";
  for (const solve::Solution &solution : *solutions)
    header += std::format(",u(t={})", solution.time);
  header += '\n';
  ExportedFile file = text_file(info, {});
  // The stream holds the mesh and the values (no expressions), so it may
  // run after the session's mutex is released; a done run's values do not
  // change.
  file.stream = [mesh, solutions, header](const ExportedFile::Sink &sink) {
    sink(header);
    std::string rows;
    for (std::size_t i = 0; i < mesh->nodes.size(); ++i) {
      rows += std::format("{},{}", mesh->nodes[i].x, mesh->nodes[i].y);
      for (const solve::Solution &solution : *solutions)
        rows += std::format(",{}", solution.values[i]);
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

// A VTK legacy unstructured grid: the mesh (z = 0), triangles, and one
// point-data array per kept snapshot, named u(t=...). The format has no
// comments; its title line (at most 256 characters) names what produced
// it, and provenance.json has the rest.
ExportedFile write_solution_vtk(const ExportInfo &info,
                                const ExportInputs &in) {
  auto mesh = in.run->meshed->mesh();
  auto solutions = in.run->solutions;
  std::string title =
      std::format("pde2d_app {} (library {}), run {}, exported {}",
                  app_version(), library_version(), in.run->id,
                  in.provenance["exported"].get<std::string>());
  if (in.provenance["case_name"].is_string())
    title += ", case " + in.provenance["case_name"].get<std::string>();
  for (char &c : title)
    if (c == '\n' || c == '\r')
      c = ' ';
  if (title.size() > 255)
    title.resize(255);
  ExportedFile file = text_file(info, {});
  file.stream = [mesh, solutions, title](const ExportedFile::Sink &sink) {
    std::string out = "# vtk DataFile Version 3.0\n" + title +
                      "\nASCII\nDATASET UNSTRUCTURED_GRID\n";
    const auto flush = [&] {
      if (out.size() > 1 << 16) {
        sink(out);
        out.clear();
      }
    };
    out += std::format("POINTS {} double\n", mesh->nodes.size());
    for (const geometry::Point &p : mesh->nodes) {
      out += std::format("{} {} 0\n", p.x, p.y);
      flush();
    }
    const std::size_t triangles = mesh->triangles.size();
    out += std::format("CELLS {} {}\n", triangles, 4 * triangles);
    for (const mesh::Triangle &t : mesh->triangles) {
      out += std::format("3 {} {} {}\n", t.nodes[0], t.nodes[1], t.nodes[2]);
      flush();
    }
    out += std::format("CELL_TYPES {}\n", triangles);
    for (std::size_t k = 0; k < triangles; ++k) {
      out += "5\n"; // VTK_TRIANGLE
      flush();
    }
    out += std::format("POINT_DATA {}\n", mesh->nodes.size());
    for (const solve::Solution &solution : *solutions) {
      out += std::format("SCALARS u(t={}) double 1\nLOOKUP_TABLE default\n",
                         solution.time);
      for (const double value : solution.values) {
        out += std::format("{}\n", value);
        flush();
      }
    }
    sink(out);
  };
  return file;
}

ExportedFile write_provenance(const ExportInfo &info, const ExportInputs &in) {
  Json out = Json::object();
  out["format"] = "pde2d-app-provenance";
  out["version"] = 1;
  for (const auto &[key, value] : in.provenance.items())
    if (key != "settings")
      out[key] = value;
  out["case"] = *in.case_json;
  if (in.run) {
    Json run = progress_json(*in.run);
    run.erase("values");
    run["started"] = utc(in.run->started);
    out["run"] = std::move(run);
  }
  return text_file(info, out.dump(2) + "\n");
}

constexpr std::array<ExportInfo, 10> kinds{{
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

Json provenance_json(const ExportInfo &info, const Json &case_json,
                     std::string_view case_name, const Run *run) {
  Json app = Json::object();
  app["name"] = "pde2d_app";
  app["version"] = app_version();
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
  out["exported"] = utc(std::chrono::system_clock::now());
  out["file"] = info.name;
  out["settings"] = std::move(settings);
  if (run != nullptr) {
    Json item = Json::object();
    item["id"] = run->id;
    item["state"] = state_name(run->state);
    item["started"] = utc(run->started);
    out["run"] = std::move(item);
  } else
    out["run"] = nullptr;
  return out;
}

} // namespace pde2d::app

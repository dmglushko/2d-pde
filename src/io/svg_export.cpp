#include "pde2d/io/svg.hpp"

#include "text.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <set>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace pde2d::io {

using detail::shortest;
using geometry::Point;

namespace {

// " x y"
std::string point_text(Point p) {
  return " " + shortest(p.x) + " " + shortest(p.y);
}

// The path command(s) that continue from the curve's start to its end.
std::string commands(const geometry::Curve &curve) {
  if (const auto *c = std::get_if<geometry::Line>(&curve))
    return " L" + point_text(c->end);
  if (const auto *c = std::get_if<geometry::QuadraticBezier>(&curve))
    return " Q" + point_text(c->control) + point_text(c->end);
  if (const auto *c = std::get_if<geometry::CubicBezier>(&curve))
    return " C" + point_text(c->control1) + point_text(c->control2) +
           point_text(c->end);
  const auto &arc = std::get<geometry::EllipticalArc>(curve);
  // Pieces of at most a quarter turn: an SVG arc cannot be a full turn
  // (its ends would coincide) and the large-arc flag stays 0.
  const int pieces = std::max(
      1, static_cast<int>(std::ceil(std::abs(arc.sweep_angle) /
                                    (std::numbers::pi / 2) * (1 - 1e-12))));
  const std::string head = " A " + shortest(arc.radius_x) + " " +
                           shortest(arc.radius_y) + " " +
                           shortest(arc.rotation * 180 / std::numbers::pi) +
                           " 0 " + (arc.sweep_angle > 0 ? "1" : "0");
  std::string text;
  for (int i = 1; i <= pieces; ++i)
    text += head + point_text(geometry::point_at(curve, double(i) / pieces));
  return text;
}

// A valid XML id: starts with a letter or '_' (region names contain only
// letters, digits, '_', '.' and '-').
bool valid_id(const std::string &name) {
  unsigned char c = static_cast<unsigned char>(name.front());
  return std::isalpha(c) || c == '_';
}

struct Bounds {
  double xmin = INFINITY, xmax = -INFINITY, ymin = INFINITY, ymax = -INFINITY;
  void add(Point p) {
    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);
  }
  // "x y width height" with a margin of 2% of the larger side.
  std::string view_box() const {
    double size = std::max({xmax - xmin, ymax - ymin, 1e-300});
    double m = 0.02 * size;
    return shortest(xmin - m) + " " + shortest(ymin - m) + " " +
           shortest(xmax - xmin + 2 * m) + " " + shortest(ymax - ymin + 2 * m);
  }
};

} // namespace

void export_svg(const geometry::CurveGeometry &geometry, std::ostream &out) {
  struct Run {
    std::string region;
    std::string d;
  };
  std::vector<Run> runs;
  Bounds bounds;
  for (const auto &loop : geometry.loops()) {
    const auto &curves = loop.curves;
    for (std::size_t i = 0; i < curves.size();) {
      Run run{curves[i].region,
              "M" + point_text(geometry::start_point(curves[i].curve))};
      for (; i < curves.size() && curves[i].region == run.region; ++i) {
        run.d += commands(curves[i].curve);
        for (int k = 0; k <= 16; ++k)
          bounds.add(geometry::point_at(curves[i].curve, k / 16.0));
      }
      runs.push_back(std::move(run));
    }
  }
  std::map<std::string, std::size_t> uses;
  for (const auto &r : runs)
    ++uses[r.region];
  // Ids: the region name where it is unique and a valid id.
  std::set<std::string> taken;
  for (const auto &r : runs)
    if (uses[r.region] == 1 && valid_id(r.region))
      taken.insert(r.region);
  std::size_t serial = 0;
  out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" "
      << "viewBox=\"" << bounds.view_box() << "\">\n";
  for (const auto &r : runs) {
    out << "  <path";
    if (uses[r.region] == 1 && valid_id(r.region)) {
      out << " id=\"" << r.region << "\"";
    } else {
      std::string id;
      do
        id = "pde2d_run_" + std::to_string(++serial);
      while (taken.count(id));
      taken.insert(id);
      out << " id=\"" << id << "\" data-region=\"" << r.region << "\"";
    }
    out << " fill=\"none\" stroke=\"black\" vector-effect=\"non-scaling-"
           "stroke\" d=\""
        << r.d << "\"/>\n";
  }
  out << "</svg>\n";
  detail::check_written(out);
}

void export_svg(const geometry::CurveGeometry &geometry,
                const std::filesystem::path &path) {
  detail::write_file(path,
                     [&](std::ostream &out) { export_svg(geometry, out); });
}

void export_mesh_svg(const mesh::Mesh &mesh, std::ostream &out) {
  if (mesh.nodes.empty())
    throw std::invalid_argument("Mesh SVG export: the mesh has no nodes");
  const std::size_t n = mesh.nodes.size();
  Bounds bounds;
  for (Point p : mesh.nodes) {
    if (!geometry::is_finite(p))
      throw std::invalid_argument("Mesh SVG export: a node is not finite");
    bounds.add(p);
  }
  for (const auto &t : mesh.triangles)
    for (auto v : t.nodes)
      if (v >= n)
        throw std::invalid_argument(
            "Mesh SVG export: a triangle uses a node index out of range");
  for (const auto &e : mesh.boundary_edges)
    if (e.nodes[0] >= n || e.nodes[1] >= n ||
        e.region >= mesh.region_names.size())
      throw std::invalid_argument(
          "Mesh SVG export: a boundary edge uses an index out of range");
  for (const auto &name : mesh.region_names)
    if (!geometry::is_valid_region_name(name))
      throw std::invalid_argument("Mesh SVG export: invalid region name '" +
                                  name + "'");
  // Colours that stay apart from each other and from the grey triangles.
  static const char *const palette[] = {"#1f77b4", "#d62728", "#2ca02c",
                                        "#ff7f0e", "#9467bd", "#8c564b",
                                        "#e377c2", "#17becf", "#bcbd22"};
  const double width = 800;
  const double aspect = (bounds.ymax - bounds.ymin + 1e-300) /
                        (bounds.xmax - bounds.xmin + 1e-300);
  out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      << "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\""
      << shortest(width) << "\" height=\""
      << shortest(std::clamp(width * aspect, 50.0, 4000.0)) << "\" viewBox=\""
      << bounds.view_box() << "\">\n";
  out << "  <path class=\"triangles\" fill=\"#eef2f8\" stroke=\"#8a8f99\" "
         "stroke-width=\"0.6\" stroke-linejoin=\"round\" "
         "vector-effect=\"non-scaling-stroke\" d=\"";
  for (const auto &t : mesh.triangles)
    out << "M" << point_text(mesh.nodes[t.nodes[0]]) << " L"
        << point_text(mesh.nodes[t.nodes[1]]) << " L"
        << point_text(mesh.nodes[t.nodes[2]]) << " Z ";
  out << "\"/>\n";
  for (std::size_t r = 0; r < mesh.region_names.size(); ++r) {
    std::string d;
    for (const auto &e : mesh.boundary_edges)
      if (e.region == r)
        d += "M" + point_text(mesh.nodes[e.nodes[0]]) + " L" +
             point_text(mesh.nodes[e.nodes[1]]) + " ";
    if (d.empty())
      continue;
    out << "  <path class=\"boundary\" data-region=\"" << mesh.region_names[r]
        << "\" fill=\"none\" stroke=\"" << palette[r % std::size(palette)]
        << "\" stroke-width=\"2.5\" stroke-linecap=\"round\" "
           "vector-effect=\"non-scaling-stroke\" d=\""
        << d << "\"><title>" << mesh.region_names[r] << "</title></path>\n";
  }
  out << "</svg>\n";
  detail::check_written(out);
}

void export_mesh_svg(const mesh::Mesh &mesh,
                     const std::filesystem::path &path) {
  detail::write_file(path,
                     [&](std::ostream &out) { export_mesh_svg(mesh, out); });
}

} // namespace pde2d::io

#include "domain_input.hpp"

#include "pde2d/geometry/shapes.hpp"
#include "pde2d/io/gmsh.hpp"
#include "pde2d/io/pde2d_mesh.hpp"
#include "pde2d/io/svg.hpp"
#include "pde2d/mesh/domain_size.hpp"

#include <functional>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace pde2d::app {

namespace {

// --- shapes -----------------------------------------------------------------

OptionSpec length_option(std::string name, std::string label,
                         std::string description, double default_value) {
  return {.name = std::move(name),
          .label = std::move(label),
          .description = std::move(description),
          .kind = OptionKind::number,
          .default_value = default_value,
          .choices = {},
          .unit = {}};
}

struct Shape {
  std::string type;
  std::string label;
  std::string regions; // which regions it has, for the form
  std::vector<OptionSpec> options;
  std::function<geometry::CurveGeometry(const OptionReader &)> make;
};

std::vector<Shape> make_shapes() {
  std::vector<Shape> list;
  list.push_back({.type = "rectangle",
                  .label = "Rectangle",
                  .regions = "bottom, right, top, left",
                  .options = {length_option("width", "Width", "", 2.0),
                              length_option("height", "Height", "", 1.0)},
                  .make = [](const OptionReader &o) {
                    return geometry::rectangle(o.number("width"),
                                               o.number("height"));
                  }});
  list.push_back({.type = "circle",
                  .label = "Circle",
                  .regions = "boundary",
                  .options = {length_option("radius", "Radius", "", 1.0)},
                  .make = [](const OptionReader &o) {
                    return geometry::circle(o.number("radius"));
                  }});
  list.push_back(
      {.type = "ellipse",
       .label = "Ellipse",
       .regions = "boundary",
       .options = {length_option("a", "Semi-axis a", "Along x.", 2.0),
                   length_option("b", "Semi-axis b", "Along y.", 1.0)},
       .make = [](const OptionReader &o) {
         return geometry::ellipse(o.number("a"), o.number("b"));
       }});
  list.push_back(
      {.type = "annulus",
       .label = "Annulus",
       .regions = "outer, inner",
       .options = {length_option("outer_radius", "Outer radius", "", 1.0),
                   length_option("inner_radius", "Inner radius",
                                 "The hole; smaller than the outer radius.",
                                 0.4)},
       .make = [](const OptionReader &o) {
         return geometry::annulus(o.number("outer_radius"),
                                  o.number("inner_radius"));
       }});
  list.push_back(
      {.type = "rectangle_with_semicircle",
       .label = "Rectangle with a round end",
       .regions = "bottom, top, cap, left",
       .options = {length_option("width", "Width",
                                 "The total length is width + height / 2.",
                                 2.0),
                   length_option("height", "Height", "", 1.0)},
       .make = [](const OptionReader &o) {
         return geometry::rectangle_with_semicircle(o.number("width"),
                                                    o.number("height"));
       }});
  list.push_back(
      {.type = "regular_polygon",
       .label = "Regular polygon",
       .regions = "boundary",
       .options = {{.name = "sides",
                    .label = "Sides",
                    .description = "At least 3.",
                    .kind = OptionKind::integer,
                    .default_value = 6,
                    .choices = {},
                    .unit = {}},
                   length_option("side_length", "Side length", "", 1.0)},
       .make = [](const OptionReader &o) {
         return geometry::regular_polygon(o.count("sides"),
                                          o.number("side_length"));
       }});
  return list;
}

const std::vector<Shape> &shapes() {
  static const std::vector<Shape> list = make_shapes();
  return list;
}

// Runs `step` and puts `where` in front of an std::invalid_argument it
// throws, unless the message already starts with it.
template <class Step> auto with_path(const std::string &where, Step step) {
  try {
    return step();
  } catch (const std::invalid_argument &error) {
    const std::string_view message = error.what();
    if (message.starts_with(where))
      throw;
    throw std::invalid_argument(where + ": " + std::string(message));
  }
}

geometry::CurveGeometry build_shape(const Json &section) {
  const std::string type =
      require_string(member(section, "shape"), "geometry.shape");
  for (const Shape &shape : shapes())
    if (shape.type == type) {
      const OptionReader options(member(section, "options"), "geometry.options",
                                 shape.options);
      return with_path("geometry.options", [&] { return shape.make(options); });
    }
  throw std::invalid_argument("geometry.shape: unknown shape '" + type + "'");
}

void apply_edit(geometry::CurveGeometry &geometry, const Json &edit,
                const std::string &where) {
  require_object(edit, where);
  const std::string op = require_string(member(edit, "op"), where + ".op");
  const auto text = [&](std::string_view key) {
    return require_string(member(edit, key), where + "." + std::string(key));
  };
  with_path(where, [&] {
    if (op == "rename") {
      static constexpr std::string_view known[] = {"op", "from", "to"};
      reject_unknown_keys(edit, where, known);
      geometry.rename_region(text("from"), text("to"));
    } else if (op == "merge") {
      static constexpr std::string_view known[] = {"op", "source", "target"};
      reject_unknown_keys(edit, where, known);
      geometry.merge_region_into(text("source"), text("target"));
    } else if (op == "split") {
      static constexpr std::string_view known[] = {"op", "region", "fraction",
                                                   "first", "second"};
      reject_unknown_keys(edit, where, known);
      geometry.split_region(
          text("region"),
          require_number(member(edit, "fraction"), where + ".fraction"),
          text("first"), text("second"));
    } else {
      throw std::invalid_argument(where + ".op: unknown edit '" + op +
                                  "' (rename, merge or split)");
    }
  });
}

// --- views ------------------------------------------------------------------

std::string_view kind_name(const geometry::Curve &curve) {
  return std::visit(
      geometry::overloaded{
          [](const geometry::Line &) { return std::string_view("line"); },
          [](const geometry::QuadraticBezier &) {
            return std::string_view("quadratic_bezier");
          },
          [](const geometry::CubicBezier &) {
            return std::string_view("cubic_bezier");
          },
          [](const geometry::EllipticalArc &) {
            return std::string_view("elliptical_arc");
          }},
      curve.kind());
}

// The curve as a polyline for drawing: its end points for a line, enough
// points for a smooth look otherwise.
Json sampled_points(const geometry::Curve &curve) {
  const int pieces = geometry::is_line(curve) ? 1 : 48;
  Json out = Json::array();
  for (int i = 0; i <= pieces; ++i) {
    const geometry::Point p =
        curve.point_at(static_cast<double>(i) / static_cast<double>(pieces));
    out.push_back(p.x);
    out.push_back(p.y);
  }
  return out;
}

Json box_json(const geometry::BoundingBox &box) {
  Json out = Json::object();
  out["min"] = {box.min.x, box.min.y};
  out["max"] = {box.max.x, box.max.y};
  return out;
}

Json geometry_view(const geometry::CurveGeometry &geometry) {
  Json loops = Json::array();
  for (std::size_t i = 0; i < geometry.loops().size(); ++i) {
    Json curves = Json::array();
    for (const geometry::BoundaryCurve &curve : geometry.loops()[i].curves) {
      Json item = Json::object();
      item["region"] = curve.region;
      item["kind"] = kind_name(curve.curve);
      item["points"] = sampled_points(curve.curve);
      curves.push_back(std::move(item));
    }
    Json loop = Json::object();
    loop["depth"] = geometry.depth(i);
    const auto parent = geometry.parent(i);
    loop["parent"] = parent ? Json(*parent) : Json(nullptr);
    loop["curves"] = std::move(curves);
    loops.push_back(std::move(loop));
  }
  Json regions = Json::array();
  for (const std::string &name : geometry.region_names()) {
    Json item = Json::object();
    item["name"] = name;
    item["length"] = geometry.region_length(name);
    regions.push_back(std::move(item));
  }
  Json out = Json::object();
  out["kind"] = "curves";
  out["loops"] = std::move(loops);
  out["regions"] = std::move(regions);
  out["bounding_box"] = box_json(geometry.bounding_box());
  out["domain_size"] = mesh::domain_size(geometry);
  return out;
}

Json mesh_file_view(const mesh::Mesh &mesh) {
  Json edges = Json::array();
  std::vector<double> lengths(mesh.region_names.size(), 0.0);
  std::vector<geometry::Point> points;
  for (const mesh::BoundaryEdge &edge : mesh.boundary_edges) {
    const geometry::Point a = mesh.nodes[edge.nodes[0]];
    const geometry::Point b = mesh.nodes[edge.nodes[1]];
    edges.push_back({a.x, a.y, b.x, b.y, edge.region});
    lengths[edge.region] += geometry::distance(a, b);
    points.push_back(a);
  }
  Json regions = Json::array();
  for (std::size_t i = 0; i < mesh.region_names.size(); ++i) {
    Json item = Json::object();
    item["name"] = mesh.region_names[i];
    item["length"] = lengths[i];
    regions.push_back(std::move(item));
  }
  Json out = Json::object();
  out["kind"] = "mesh_file";
  out["boundary_edges"] = std::move(edges);
  out["regions"] = std::move(regions);
  out["bounding_box"] = box_json(geometry::bounding_box(points));
  out["domain_size"] = mesh::domain_size(mesh);
  out["nodes"] = mesh.nodes.size();
  out["triangles"] = mesh.triangles.size();
  return out;
}

} // namespace

Domain build_domain(const Json &section) {
  require_object(section, "geometry");
  const std::string source =
      require_string(member(section, "source"), "geometry.source");
  if (source == "svg" || source == "shape") {
    static constexpr std::string_view svg_keys[] = {"source", "name", "svg",
                                                    "region_edits"};
    static constexpr std::string_view shape_keys[] = {
        "source", "shape", "options", "region_edits"};
    reject_unknown_keys(section, "geometry",
                        source == "svg" ? std::span(svg_keys)
                                        : std::span(shape_keys));
    geometry::CurveGeometry geometry =
        source == "svg"
            ? with_path("geometry.svg",
                        [&] {
                          std::istringstream in(require_string(
                              member(section, "svg"), "geometry.svg"));
                          return io::import_svg(in);
                        })
            : build_shape(section);
    if (const Json &edits = member(section, "region_edits"); !edits.is_null()) {
      if (!edits.is_array())
        throw std::invalid_argument("geometry.region_edits: expected a list");
      for (std::size_t i = 0; i < edits.size(); ++i)
        apply_edit(geometry, edits[i],
                   "geometry.region_edits[" + std::to_string(i) + "]");
    }
    return {.geometry = std::move(geometry), .mesh = std::nullopt};
  }
  if (source == "mesh_file") {
    static constexpr std::string_view known[] = {"source", "format", "name",
                                                 "text"};
    reject_unknown_keys(section, "geometry", known);
    const std::string format =
        require_string(member(section, "format"), "geometry.format");
    std::istringstream in(
        require_string(member(section, "text"), "geometry.text"));
    mesh::Mesh mesh = with_path("geometry.text", [&] {
      if (format == "gmsh")
        return io::import_gmsh(in);
      if (format == "pde2d")
        return io::import_pde2d(in);
      throw std::invalid_argument("unknown mesh format '" + format +
                                  "' (gmsh or pde2d)");
    });
    return {.geometry = std::nullopt, .mesh = std::move(mesh)};
  }
  throw std::invalid_argument("geometry.source: unknown source '" + source +
                              "' (svg, shape or mesh_file)");
}

Json domain_json(const Domain &domain) {
  if (domain.geometry)
    return geometry_view(*domain.geometry);
  return mesh_file_view(*domain.mesh);
}

Json shape_catalog_json() {
  Json out = Json::array();
  for (const Shape &shape : shapes()) {
    Json item = Json::object();
    item["type"] = shape.type;
    item["label"] = shape.label;
    item["description"] = "Regions: " + shape.regions + ".";
    item["options"] = to_json(shape.options);
    out.push_back(std::move(item));
  }
  return out;
}

Json default_geometry_section() {
  Json out = Json::object();
  out["source"] = "shape";
  out["shape"] = shapes().at(0).type;
  out["options"] = defaults(shapes().at(0).options);
  out["region_edits"] = Json::array();
  return out;
}

} // namespace pde2d::app

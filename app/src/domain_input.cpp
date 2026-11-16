#include "domain_input.hpp"

#include "app_error.hpp"

#include "pde2d/geometry/polygon.hpp"
#include "pde2d/geometry/shapes.hpp"
#include "pde2d/io/gmsh.hpp"
#include "pde2d/io/pde2d_mesh.hpp"
#include "pde2d/io/svg.hpp"
#include "pde2d/mesh/domain_size.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
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

geometry::CurveGeometry build_shape(const Json &section) {
  const std::string type =
      require_string(member(section, "shape"), "geometry.shape");
  for (const Shape &shape : shapes())
    if (shape.type == type) {
      const OptionReader options(member(section, "options"), "geometry.options",
                                 shape.options);
      try {
        return shape.make(options);
      } catch (const PathError &) {
        throw; // the reader's own errors carry their path
      } catch (const std::invalid_argument &error) {
        // The option the shape's message names, like a component's.
        throw PathError("geometry.options" +
                            named_option(error.what(), shape.options),
                        error.what());
      }
    }
  throw PathError("geometry.shape", "unknown shape '" + type + "'");
}

void apply_edit(geometry::CurveGeometry &geometry, const Json &edit,
                const std::string &where) {
  require_object(edit, where);
  const std::string op = require_string(member(edit, "op"), where + ".op");
  const auto text = [&](std::string_view key) {
    return require_string(member(edit, key), where + "." + std::string(key));
  };
  at_path(where, [&] {
    if (op == "rename") {
      reject_unknown_keys(edit, where, rename_edit_keys);
      geometry.rename_region(text("from"), text("to"));
    } else if (op == "merge") {
      reject_unknown_keys(edit, where, merge_edit_keys);
      geometry.merge_region_into(text("source"), text("target"));
    } else if (op == "split") {
      reject_unknown_keys(edit, where, split_edit_keys);
      geometry.split_region(
          text("region"),
          require_number(member(edit, "fraction"), where + ".fraction"),
          text("first"), text("second"));
    } else {
      throw PathError(where + ".op",
                      "unknown edit '" + op + "' (rename, merge or split)");
    }
  });
}

// Region edits of an imported mesh: rename and merge, applied to the
// mesh's region names and the region tags of its boundary edges. (A split
// needs the curves, which a mesh file does not have.) An imported mesh has
// no curve links, so only the tags change.
std::size_t mesh_region(const mesh::Mesh &mesh, const std::string &name) {
  const auto found = std::ranges::find(mesh.region_names, name);
  if (found == mesh.region_names.end())
    throw std::invalid_argument("Unknown region '" + name + "'");
  return static_cast<std::size_t>(found - mesh.region_names.begin());
}

void rename_mesh_region(mesh::Mesh &mesh, const std::string &from,
                        const std::string &to) {
  const std::size_t index = mesh_region(mesh, from);
  if (!geometry::is_valid_region_name(to))
    throw std::invalid_argument(
        "rename: region name '" + to +
        "' is invalid; use ASCII letters, digits, '_', '.' or '-' (a lone "
        "'-' is reserved)");
  if (from == to)
    return;
  if (std::ranges::find(mesh.region_names, to) != mesh.region_names.end())
    throw std::invalid_argument("rename: region '" + to +
                                "' already exists; use merge");
  mesh.region_names[index] = to;
}

void merge_mesh_region(mesh::Mesh &mesh, const std::string &source,
                       const std::string &target) {
  const std::size_t from = mesh_region(mesh, source);
  const std::size_t into = mesh_region(mesh, target);
  if (from == into)
    throw std::invalid_argument("merge: region '" + source +
                                "' cannot be merged into itself");
  // The source's edges go to the target; the tags above the source's
  // move down by one, as its name is dropped.
  for (mesh::BoundaryEdge &edge : mesh.boundary_edges) {
    std::size_t region = edge.region == from ? into : edge.region;
    if (region > from)
      --region;
    edge.region = region;
  }
  mesh.region_names.erase(mesh.region_names.begin() +
                          static_cast<std::ptrdiff_t>(from));
}

void apply_mesh_edit(mesh::Mesh &mesh, const Json &edit,
                     const std::string &where) {
  require_object(edit, where);
  const std::string op = require_string(member(edit, "op"), where + ".op");
  const auto text = [&](std::string_view key) {
    return require_string(member(edit, key), where + "." + std::string(key));
  };
  at_path(where, [&] {
    if (op == "rename") {
      reject_unknown_keys(edit, where, rename_edit_keys);
      rename_mesh_region(mesh, text("from"), text("to"));
    } else if (op == "merge") {
      reject_unknown_keys(edit, where, merge_edit_keys);
      merge_mesh_region(mesh, text("source"), text("target"));
    } else if (op == "split") {
      throw PathError(where + ".op",
                      "an imported mesh has no curves to split; only rename "
                      "and merge apply to it");
    } else {
      throw PathError(where + ".op", "unknown edit '" + op +
                                         "' (rename or merge for a mesh file)");
    }
  });
}

// Applies the "region_edits" list of a section with `apply` per edit.
template <class Target, class Apply>
void apply_edits(const Json &section, Target &target, Apply apply) {
  const Json &edits = member(section, "region_edits");
  if (edits.is_null())
    return;
  if (!edits.is_array())
    throw PathError("geometry.region_edits", "expected a list");
  for (std::size_t i = 0; i < edits.size(); ++i)
    apply(target, edits[i], "geometry.region_edits[" + std::to_string(i) + "]");
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
    reject_unknown_keys(section, "geometry",
                        source == "svg" ? std::span(svg_section_keys)
                                        : std::span(shape_section_keys));
    geometry::CurveGeometry geometry =
        source == "svg"
            ? at_path("geometry.svg",
                      [&] {
                        std::istringstream in(require_string(
                            member(section, "svg"), "geometry.svg"));
                        return io::import_svg(in);
                      })
            : build_shape(section);
    apply_edits(section, geometry, apply_edit);
    return {.geometry = std::move(geometry), .mesh = std::nullopt};
  }
  if (source == "mesh_file") {
    reject_unknown_keys(section, "geometry", mesh_file_section_keys);
    const std::string format =
        require_string(member(section, "format"), "geometry.format");
    std::istringstream in(
        require_string(member(section, "text"), "geometry.text"));
    mesh::Mesh mesh = at_path("geometry.text", [&] {
      if (format == "gmsh")
        return io::import_gmsh(in);
      if (format == "pde2d")
        return io::import_pde2d(in);
      throw PathError("geometry.format",
                      "unknown mesh format '" + format + "' (gmsh or pde2d)");
    });
    if (!member(section, "region_edits").is_null()) {
      apply_edits(section, mesh, apply_mesh_edit);
      // The edits keep the tags consistent; the library confirms it.
      at_path("geometry.region_edits", [&] { mesh::validate(mesh); });
    }
    return {.geometry = std::nullopt, .mesh = std::move(mesh)};
  }
  throw PathError("geometry.source",
                  "unknown source '" + source + "' (svg, shape or mesh_file)");
}

Json domain_json(const Domain &domain) {
  if (domain.geometry)
    return geometry_view(*domain.geometry);
  return mesh_file_view(*domain.mesh);
}

namespace {

double segment_distance(geometry::Point p, geometry::Point a,
                        geometry::Point b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length = dx * dx + dy * dy;
  const double s =
      length == 0.0 ? 0.0
                    : std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / length,
                                 0.0, 1.0);
  return std::hypot(p.x - (a.x + s * dx), p.y - (a.y + s * dy));
}

// The candidate with the largest distance to the segments (ties: the
// first), among those `inside` accepts.
template <class Inside>
void farthest(
    DomainFacts &facts, const std::vector<geometry::Point> &candidates,
    const std::vector<std::pair<geometry::Point, geometry::Point>> &segments,
    Inside inside) {
  facts.clearance = -1.0;
  for (const geometry::Point &p : candidates) {
    if (!inside(p))
      continue;
    double nearest = std::numeric_limits<double>::infinity();
    for (const auto &[a, b] : segments)
      nearest = std::min(nearest, segment_distance(p, a, b));
    if (nearest > facts.clearance) {
      facts.clearance = nearest;
      facts.inner = p;
    }
  }
  if (facts.clearance < 0)
    facts.clearance = 0.0;
}

} // namespace

DomainFacts domain_facts(const Domain &domain) {
  DomainFacts facts;
  std::vector<std::pair<geometry::Point, geometry::Point>> segments;
  if (domain.geometry) {
    const geometry::CurveGeometry &geometry = *domain.geometry;
    facts.regions = geometry.region_names();
    facts.domain_size = mesh::domain_size(geometry);
    std::vector<std::vector<geometry::Point>> polygons;
    for (const geometry::Loop &loop : geometry.loops()) {
      std::vector<geometry::Point> polygon;
      for (const geometry::BoundaryCurve &curve : loop.curves) {
        const int pieces = geometry::is_line(curve.curve) ? 1 : 48;
        for (int i = 0; i < pieces; ++i)
          polygon.push_back(curve.curve.point_at(static_cast<double>(i) /
                                                 static_cast<double>(pieces)));
      }
      for (std::size_t i = 0; i < polygon.size(); ++i)
        segments.emplace_back(polygon[i], polygon[(i + 1) % polygon.size()]);
      polygons.push_back(std::move(polygon));
    }
    const geometry::BoundingBox box = geometry.bounding_box();
    constexpr int grid = 40;
    std::vector<geometry::Point> candidates;
    for (int j = 0; j < grid; ++j)
      for (int i = 0; i < grid; ++i)
        candidates.push_back(
            {box.min.x + (box.max.x - box.min.x) * (i + 0.5) / grid,
             box.min.y + (box.max.y - box.min.y) * (j + 0.5) / grid});
    // Inside the domain: inside an odd number of loops.
    farthest(facts, candidates, segments, [&](geometry::Point p) {
      int count = 0;
      for (const auto &polygon : polygons)
        if (polygon.size() >= 3 && geometry::locate_in_polygon(polygon, p) > 0)
          ++count;
      return count % 2 == 1;
    });
    return facts;
  }
  const mesh::Mesh &mesh = *domain.mesh;
  facts.regions = mesh.region_names;
  facts.domain_size = mesh::domain_size(mesh);
  for (const mesh::BoundaryEdge &edge : mesh.boundary_edges)
    segments.emplace_back(mesh.nodes[edge.nodes[0]], mesh.nodes[edge.nodes[1]]);
  std::vector<geometry::Point> candidates;
  const std::size_t stride =
      std::max<std::size_t>(1, mesh.triangles.size() / 2000);
  for (std::size_t k = 0; k < mesh.triangles.size(); k += stride) {
    const auto &t = mesh.triangles[k].nodes;
    const geometry::Point a = mesh.nodes[t[0]];
    const geometry::Point b = mesh.nodes[t[1]];
    const geometry::Point c = mesh.nodes[t[2]];
    candidates.push_back({(a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3});
  }
  farthest(facts, candidates, segments, [](geometry::Point) { return true; });
  return facts;
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

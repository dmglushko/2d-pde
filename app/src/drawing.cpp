#include "drawing.hpp"

#include "app_error.hpp"

#include "pde2d/geometry/bounding_box.hpp"

#include <cstddef>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace pde2d::app {

namespace {

std::string index_path(const std::string &where, std::string_view key,
                       std::size_t i) {
  return where + "." + std::string(key) + "[" + std::to_string(i) + "]";
}

geometry::Point require_point(const Json &value, const std::string &where) {
  if (!value.is_array() || value.size() != 2)
    throw PathError(where, "expected a point [x, y]");
  return {require_number(value[0], where + "[0]"),
          require_number(value[1], where + "[1]")};
}

void require_list(const Json &value, const std::string &where) {
  if (!value.is_array())
    throw PathError(where, "expected a list");
}

Json point_json(geometry::Point p) { return Json::array({p.x, p.y}); }

// An arc has its own end points; a line or a Bezier takes them from the
// vertices.
bool is_arc(const geometry::Curve &curve) {
  return std::visit(
      geometry::overloaded{
          [](const geometry::Line &) { return false; },
          [](const geometry::QuadraticBezier &) { return false; },
          [](const geometry::CubicBezier &) { return false; },
          [](const geometry::EllipticalArc &) { return true; }},
      curve.kind());
}

// One edge from `a` to `b` as written at `where`.
geometry::BoundaryCurve read_edge(const Json &edge, const std::string &where,
                                  geometry::Point a, geometry::Point b) {
  require_object(edge, where);
  const std::string kind =
      require_string(member(edge, "kind"), where + ".kind");
  const std::string region =
      require_string(member(edge, "region"), where + ".region");
  const auto number = [&](std::string_view key) {
    return require_number(member(edge, key), where + "." + std::string(key));
  };
  const auto point = [&](std::string_view key) {
    return require_point(member(edge, key), where + "." + std::string(key));
  };
  if (kind == "line") {
    reject_unknown_keys(edge, where, line_edge_keys);
    return {geometry::Line{a, b}, region};
  }
  if (kind == "quadratic") {
    reject_unknown_keys(edge, where, quadratic_edge_keys);
    return {geometry::QuadraticBezier{a, point("control"), b}, region};
  }
  if (kind == "cubic") {
    reject_unknown_keys(edge, where, cubic_edge_keys);
    return {geometry::CubicBezier{a, point("control1"), point("control2"), b},
            region};
  }
  if (kind == "arc") {
    reject_unknown_keys(edge, where, arc_edge_keys);
    const Json &rotation = member(edge, "rotation");
    return {geometry::EllipticalArc{
                .center = point("center"),
                .radius_x = number("radius_x"),
                .radius_y = number("radius_y"),
                .rotation = rotation.is_null()
                                ? 0.0
                                : require_number(rotation, where + ".rotation"),
                .start_angle = number("start_angle"),
                .sweep_angle = number("sweep_angle")},
            region};
  }
  throw PathError(where + ".kind", "unknown edge kind '" + kind +
                                       "' (line, arc, quadratic or cubic)");
}

std::string format_point(geometry::Point p) {
  std::ostringstream out;
  out << '(' << p.x << ", " << p.y << ')';
  return out.str();
}

// An arc's ends must be its vertices: lines and Beziers take their ends
// from the vertices, an arc has its own (the library checks the joins
// between curves, but not where a vertex between two arcs is).
void check_arc_ends(const std::vector<geometry::Loop> &loops,
                    const std::vector<std::vector<geometry::Point>> &vertices) {
  std::optional<geometry::BoundingBox> box;
  for (const geometry::Loop &loop : loops)
    for (const geometry::BoundaryCurve &curve : loop.curves) {
      const geometry::BoundingBox b = curve.curve.bounding_box();
      box = box ? geometry::merged(*box, b) : b;
    }
  if (!box)
    return;
  const double tolerance = 1e-9 * geometry::diagonal(*box);
  for (std::size_t l = 0; l < loops.size(); ++l) {
    const std::size_t n = vertices[l].size();
    for (std::size_t e = 0; e < n; ++e) {
      const geometry::Curve &curve = loops[l].curves[e].curve;
      if (!is_arc(curve))
        continue;
      const std::string where =
          "geometry.loops[" + std::to_string(l) + "].edges[" +
          std::to_string(e) + "]";
      const auto check = [&](geometry::Point end, std::size_t vertex,
                             const char *which) {
        const geometry::Point at = vertices[l][vertex];
        if (geometry::distance(end, at) > tolerance)
          throw PathError(where, std::string("the arc ") + which + " at " +
                                     format_point(end) + ", not at its vertex " +
                                     std::to_string(vertex) + " " +
                                     format_point(at));
      };
      check(geometry::start_point(curve), e, "starts");
      check(geometry::end_point(curve), (e + 1) % n, "ends");
    }
  }
}

} // namespace

std::string drawing_error_path(std::string_view message) {
  const std::string text(message);
  static const std::regex curve_named(R"(Loop (\d+), curve (\d+))");
  static const std::regex not_closed(R"(Loop (\d+) is not closed: curve (\d+))");
  static const std::regex itself(
      R"(Loop (\d+) intersects or touches itself near curves (\d+))");
  static const std::regex loop_named(R"(Loops? (\d+))");
  const auto edge = [](const std::smatch &m) {
    return "geometry.loops[" + m[1].str() + "].edges[" + m[2].str() + "]";
  };
  std::smatch match;
  std::optional<std::string> last;
  for (auto it = std::sregex_iterator(text.begin(), text.end(), curve_named);
       it != std::sregex_iterator(); ++it)
    last = edge(*it);
  if (last)
    return *last;
  if (std::regex_search(text, match, not_closed) ||
      std::regex_search(text, match, itself))
    return edge(match);
  if (std::regex_search(text, match, loop_named))
    return "geometry.loops[" + match[1].str() + "]";
  return "geometry.loops";
}

geometry::CurveGeometry build_drawing(const Json &section) {
  const Json &loops_json = member(section, "loops");
  require_list(loops_json, "geometry.loops");
  std::vector<geometry::Loop> loops;
  std::vector<std::vector<geometry::Point>> vertices;
  for (std::size_t l = 0; l < loops_json.size(); ++l) {
    const std::string where = index_path("geometry", "loops", l);
    const Json &loop = loops_json[l];
    require_object(loop, where);
    reject_unknown_keys(loop, where, drawing_loop_keys);
    const Json &points = member(loop, "vertices");
    require_list(points, where + ".vertices");
    const Json &edges = member(loop, "edges");
    require_list(edges, where + ".edges");
    if (points.empty())
      throw PathError(where + ".vertices", "a loop needs at least one vertex");
    if (edges.size() != points.size())
      throw PathError(where + ".edges",
                      "a loop has one edge per vertex (" +
                          std::to_string(points.size()) + " vertices, " +
                          std::to_string(edges.size()) + " edges)");
    std::vector<geometry::Point> at;
    for (std::size_t v = 0; v < points.size(); ++v)
      at.push_back(require_point(points[v], index_path(where, "vertices", v)));
    geometry::Loop out;
    for (std::size_t e = 0; e < edges.size(); ++e) {
      const std::string edge_where = index_path(where, "edges", e);
      geometry::BoundaryCurve curve =
          read_edge(edges[e], edge_where, at[e], at[(e + 1) % at.size()]);
      // A curve the library refuses on its own (a zero radius) is refused
      // here, before its ends are compared with the vertices.
      at_path(edge_where, [&] { curve.curve.validate(); });
      out.curves.push_back(std::move(curve));
    }
    loops.push_back(std::move(out));
    vertices.push_back(std::move(at));
  }
  if (loops.empty())
    throw PathError("geometry.loops", "a drawing needs at least one loop");
  check_arc_ends(loops, vertices);
  try {
    return geometry::CurveGeometry(std::move(loops));
  } catch (const std::invalid_argument &error) {
    throw PathError(drawing_error_path(error.what()), error.what());
  }
}

namespace {

Json edge_json(const geometry::BoundaryCurve &curve) {
  Json out = Json::object();
  std::visit(geometry::overloaded{
                 [&](const geometry::Line &) { out["kind"] = "line"; },
                 [&](const geometry::QuadraticBezier &c) {
                   out["kind"] = "quadratic";
                   out["region"] = curve.region;
                   out["control"] = point_json(c.control);
                 },
                 [&](const geometry::CubicBezier &c) {
                   out["kind"] = "cubic";
                   out["region"] = curve.region;
                   out["control1"] = point_json(c.control1);
                   out["control2"] = point_json(c.control2);
                 },
                 [&](const geometry::EllipticalArc &c) {
                   out["kind"] = "arc";
                   out["region"] = curve.region;
                   out["center"] = point_json(c.center);
                   out["radius_x"] = c.radius_x;
                   out["radius_y"] = c.radius_y;
                   out["rotation"] = c.rotation;
                   out["start_angle"] = c.start_angle;
                   out["sweep_angle"] = c.sweep_angle;
                 }},
             curve.curve.kind());
  out["region"] = curve.region;
  return out;
}

} // namespace

Json drawing_section(const geometry::CurveGeometry &geometry) {
  Json loops = Json::array();
  for (const geometry::Loop &loop : geometry.loops()) {
    const auto &curves = loop.curves;
    const std::size_t n = curves.size();
    Json vertices = Json::array();
    Json edges = Json::array();
    for (std::size_t i = 0; i < n; ++i) {
      // Vertex i is where curve i - 1 ends and curve i starts: the exact
      // end point of the one that takes its ends from the vertices (a line
      // or a Bezier), or the arc's own start between two arcs.
      const geometry::Curve &before = curves[(i + n - 1) % n].curve;
      const geometry::Curve &after = curves[i].curve;
      const geometry::Point vertex =
          !is_arc(after)    ? geometry::start_point(after)
          : !is_arc(before) ? geometry::end_point(before)
                            : geometry::start_point(after);
      vertices.push_back(point_json(vertex));
      edges.push_back(edge_json(curves[i]));
    }
    Json item = Json::object();
    item["vertices"] = std::move(vertices);
    item["edges"] = std::move(edges);
    loops.push_back(std::move(item));
  }
  Json out = Json::object();
  out["source"] = "drawing";
  out["loops"] = std::move(loops);
  return out;
}

} // namespace pde2d::app

// The geometry editor's server side: the drawing source (reading, errors
// with their paths, the exact round trip of shapes and SVG examples), the
// boundary locator (fractions exact at corners) and the split of an
// imported mesh's region at a node.

#include "domain_input.hpp"
#include "drawing.hpp"
#include "locate.hpp"
#include "problem_input.hpp"
#include "session.hpp"

#include "support/check.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numbers>
#include <string>
#include <variant>
#include <vector>

#ifndef PDE2D_APP_EXAMPLES_DIR
#error "PDE2D_APP_EXAMPLES_DIR must name app/examples"
#endif

using namespace pde2d;
using app::Json;
using test::check;
using test::expect_throws;
using test::near;

namespace {

Json load_example(const std::filesystem::path &file) {
  std::ifstream in(file);
  return Json::parse(std::string(std::istreambuf_iterator<char>(in),
                                 std::istreambuf_iterator<char>()));
}

Json point(double x, double y) { return Json::array({x, y}); }

Json line(const std::string &region) {
  return {{"kind", "line"}, {"region", region}};
}

// A loop of lines through `points`, every edge in `region`.
Json polygon(const std::vector<std::pair<double, double>> &points,
             const std::string &region) {
  Json vertices = Json::array();
  Json edges = Json::array();
  for (const auto &[x, y] : points) {
    vertices.push_back(point(x, y));
    edges.push_back(line(region));
  }
  return {{"vertices", vertices}, {"edges", edges}};
}

// A full circle: one vertex, one arc.
Json circle(double cx, double cy, double r, const std::string &region) {
  return {{"vertices", Json::array({point(cx + r, cy)})},
          {"edges", Json::array({{{"kind", "arc"},
                                  {"region", region},
                                  {"center", point(cx, cy)},
                                  {"radius_x", r},
                                  {"radius_y", r},
                                  {"rotation", 0.0},
                                  {"start_angle", 0.0},
                                  {"sweep_angle", 2 * std::numbers::pi}}})}};
}

Json drawing(const Json &loops) {
  return {{"source", "drawing"}, {"loops", loops}};
}

// The square [0, 2] x [0, 2] with its four sides as regions.
Json square_loop() {
  Json loop = polygon({{0, 0}, {2, 0}, {2, 2}, {0, 2}}, "bottom");
  loop["edges"][1]["region"] = "right";
  loop["edges"][2]["region"] = "top";
  loop["edges"][3]["region"] = "left";
  return loop;
}

// The geometry stage's error for a geometry section: its path and text.
app::StageError geometry_error(const Json &section) {
  app::Session session;
  Json case_json = app::Session::default_case();
  case_json["geometry"] = section;
  try {
    (void)session.geometry(case_json);
  } catch (const app::StageError &error) {
    return error;
  }
  throw test::Failure("the geometry was accepted");
}

void expect_path(const Json &section, const std::string &path,
                 const std::string &fragment) {
  const app::StageError error = geometry_error(section);
  // A path ending in '[' is a prefix (the library picks the curve).
  const bool same = path.ends_with('[') ? error.path().starts_with(path)
                                        : error.path() == path;
  check(same, "path " + error.path() + " should be " + path + " (" +
                  error.what() + ")");
  check(std::string(error.what()).find(fragment) != std::string::npos,
        "message '" + std::string(error.what()) + "' should contain '" +
            fragment + "'");
}

// Two curves are the same exact curve: the same kind and bit for bit the
// same data.
bool same_curve(const geometry::Curve &a, const geometry::Curve &b) {
  const auto pt = [](geometry::Point p, geometry::Point q) { return p == q; };
  return std::visit(
      geometry::overloaded{
          [&](const geometry::Line &x) {
            const auto *y = std::get_if<geometry::Line>(&b.kind());
            return y && pt(x.start, y->start) && pt(x.end, y->end);
          },
          [&](const geometry::QuadraticBezier &x) {
            const auto *y = std::get_if<geometry::QuadraticBezier>(&b.kind());
            return y && pt(x.start, y->start) && pt(x.control, y->control) &&
                   pt(x.end, y->end);
          },
          [&](const geometry::CubicBezier &x) {
            const auto *y = std::get_if<geometry::CubicBezier>(&b.kind());
            return y && pt(x.start, y->start) && pt(x.control1, y->control1) &&
                   pt(x.control2, y->control2) && pt(x.end, y->end);
          },
          [&](const geometry::EllipticalArc &x) {
            const auto *y = std::get_if<geometry::EllipticalArc>(&b.kind());
            return y && pt(x.center, y->center) && x.radius_x == y->radius_x &&
                   x.radius_y == y->radius_y && x.rotation == y->rotation &&
                   x.start_angle == y->start_angle &&
                   x.sweep_angle == y->sweep_angle;
          }},
      a.kind());
}

// The drawing of `geometry`, written as text and read back, has the same
// loops, curves, regions, depths and parents, bit for bit.
void check_round_trip(const geometry::CurveGeometry &geometry,
                      const std::string &name) {
  const Json section = app::drawing_section(geometry);
  // Through text, as a saved case.
  const Json reread = Json::parse(section.dump());
  const app::Domain back = app::build_domain(reread);
  const auto &a = geometry.loops();
  const auto &b = back.geometry->loops();
  check(a.size() == b.size(), name + ": the number of loops");
  for (std::size_t l = 0; l < a.size(); ++l) {
    check(a[l].curves.size() == b[l].curves.size(),
          name + ": the curves of loop " + std::to_string(l));
    check(geometry.depth(l) == back.geometry->depth(l),
          name + ": depth of loop " + std::to_string(l));
    check(geometry.parent(l) == back.geometry->parent(l),
          name + ": parent of loop " + std::to_string(l));
    for (std::size_t c = 0; c < a[l].curves.size(); ++c) {
      const std::string where =
          name + ": loop " + std::to_string(l) + ", curve " + std::to_string(c);
      check(a[l].curves[c].region == b[l].curves[c].region, where + ": region");
      check(same_curve(a[l].curves[c].curve, b[l].curves[c].curve),
            where + ": the curve differs");
    }
  }
  check(geometry.region_names() == back.geometry->region_names(),
        name + ": the region names");
}

// A rectangle's mesh, exported as a .pde2d file.
Json rectangle_mesh_section() {
  app::Session session;
  Json case_json = app::Session::default_case();
  case_json["mesh"]["spec"]["max_edge_length"] = {{"absolute", nullptr},
                                                  {"relative", 0.1}};
  const auto file = session.export_file(case_json, "mesh.pde2d");
  return {{"source", "mesh_file"}, {"format", "pde2d"}, {"text", file.text()}};
}

std::size_t edges_of(const mesh::Mesh &m, const std::string &name) {
  std::size_t count = 0;
  for (const mesh::BoundaryEdge &edge : m.boundary_edges)
    if (m.region_names[edge.region] == name)
      ++count;
  return count;
}

} // namespace

int main() {
  return test::run({
      {"a drawing builds its loops: a square with a round hole",
       [] {
         const Json section =
             drawing(Json::array({square_loop(), circle(1, 1, 0.4, "hole")}));
         const app::Domain domain = app::build_domain(section);
         check(domain.geometry.has_value(), "curves");
         check(domain.geometry->loops().size() == 2, "two loops");
         check(domain.geometry->depth(1) == 1, "the circle is a hole");
         check(domain.geometry->region_names() ==
                   std::vector<std::string>{"bottom", "right", "top", "left",
                                            "hole"},
               "regions");
         near(domain.geometry->region_length("hole"),
              2 * std::numbers::pi * 0.4, 1e-12, "the hole's length");
         // A clockwise outer loop is turned (the library normalizes).
         Json turned = square_loop();
         Json vertices = Json::array();
         for (const auto &[x, y] : std::vector<std::pair<double, double>>{
                  {0, 0}, {0, 2}, {2, 2}, {2, 0}})
           vertices.push_back(point(x, y));
         turned["vertices"] = vertices;
         const app::Domain cw =
             app::build_domain(drawing(Json::array({turned})));
         check(cw.geometry->depth(0) == 0, "an outer loop either way");
         // Beziers and an arc between lines, the view draws them.
         Json loop = polygon({{0, 0}, {3, 0}, {3, 1}, {0, 1}}, "wall");
         loop["edges"][0] = {{"kind", "quadratic"},
                             {"region", "floor"},
                             {"control", point(1.5, -0.5)}};
         loop["edges"][2] = {{"kind", "cubic"},
                             {"region", "roof"},
                             {"control1", point(2, 1.5)},
                             {"control2", point(1, 1.5)}};
         loop["edges"][1] = {{"kind", "arc"},
                             {"region", "cap"},
                             {"center", point(3, 0.5)},
                             {"radius_x", 0.5},
                             {"radius_y", 0.5},
                             {"start_angle", -std::numbers::pi / 2},
                             {"sweep_angle", std::numbers::pi}};
         const app::Domain mixed =
             app::build_domain(drawing(Json::array({loop})));
         const Json view = app::domain_json(mixed);
         check(view["kind"] == "curves", "drawn as curves");
         check(view["loops"][0]["curves"][1]["kind"] == "elliptical_arc",
               "the arc");
       }},
      {"a drawing's errors name the loop, the vertex or the edge",
       [] {
         Json section = drawing(Json::array({square_loop()}));
         section["loops"][0]["vertices"][1] = "two";
         expect_path(section, "geometry.loops[0].vertices[1]",
                     "expected a point");
         section = drawing(Json::array({square_loop()}));
         section["loops"][0]["edges"].erase(3);
         expect_path(section, "geometry.loops[0].edges", "one edge per vertex");
         section = drawing(Json::array({square_loop()}));
         section["loops"][0]["edges"][2]["kind"] = "spline";
         expect_path(section, "geometry.loops[0].edges[2].kind",
                     "unknown edge kind 'spline'");
         section = drawing(Json::array({square_loop()}));
         section["loops"][0]["edges"][2]["control"] = point(1, 3);
         expect_path(section, "geometry.loops[0].edges[2]",
                     "unknown key 'control'");
         section = drawing(Json::array({square_loop()}));
         section["loops"][0]["edges"][3]["region"] = "a b";
         expect_path(section, "geometry.loops[0].edges[3]",
                     "region name 'a b' is invalid");
         section = drawing(Json::array({square_loop()}));
         section["region_edits"] = Json::array();
         expect_path(section, "geometry", "unknown key 'region_edits'");
         section = drawing(Json::array());
         expect_path(section, "geometry.loops", "at least one loop");
         // A loop that crosses itself (a bow tie).
         section = drawing(
             Json::array({polygon({{0, 0}, {2, 2}, {2, 0}, {0, 2}}, "x")}));
         expect_path(section, "geometry.loops[0].edges[",
                     "intersects or touches itself");
         // Two loops that cross: the later one is named.
         section = drawing(Json::array(
             {square_loop(), polygon({{1, 1}, {3, 1}, {3, 3}, {1, 3}}, "b")}));
         const app::StageError crossing = geometry_error(section);
         check(crossing.path().starts_with("geometry.loops[1].edges["),
               crossing.path());
         // An arc whose end is not at its vertex.
         Json loop = square_loop();
         loop["edges"][1] = {{"kind", "arc"},
                             {"region", "right"},
                             {"center", point(2, 1)},
                             {"radius_x", 1.0},
                             {"radius_y", 1.0},
                             {"start_angle", -std::numbers::pi / 2},
                             {"sweep_angle", std::numbers::pi / 2}};
         expect_path(drawing(Json::array({loop})), "geometry.loops[0].edges[1]",
                     "the arc ends at");
         // A zero radius (the library's own refusal of the curve).
         loop["edges"][1]["radius_x"] = 0.0;
         expect_path(drawing(Json::array({loop})), "geometry.loops[0].edges[1]",
                     "elliptical arc");
         // A line from a vertex to itself has no length.
         section = drawing(Json::array({polygon({{0, 0}}, "dot")}));
         expect_path(section, "geometry.loops[0].edges[0]", "zero length");
         // The message's own loop and curve.
         check(app::drawing_error_path(
                   "Loops 0 and 2 intersect or touch (Loop 0, curve 1 and "
                   "Loop 2, curve 4)") == "geometry.loops[2].edges[4]",
               "two loops");
         check(app::drawing_error_path("Loop 3 has no curves") ==
                   "geometry.loops[3]",
               "a loop");
         check(app::drawing_error_path("Geometry has no extent") ==
                   "geometry.loops",
               "none");
       }},
      {"every shape is a drawing, curve for curve",
       [] {
         for (const Json &shape : app::Session::catalog()["shapes"]) {
           Json section = {{"source", "shape"},
                           {"shape", shape["type"]},
                           {"options", Json::object()}};
           const app::Domain domain = app::build_domain(section);
           check_round_trip(*domain.geometry, shape["type"].get<std::string>());
         }
         // After region edits too: the drawing has the edited regions.
         Json section = {{"source", "shape"},
                         {"shape", "rectangle_with_semicircle"},
                         {"options", Json::object()},
                         {"region_edits", Json::array({{{"op", "split"},
                                                        {"region", "cap"},
                                                        {"fraction", 0.25},
                                                        {"first", "cap"},
                                                        {"second", "cap_2"}},
                                                       {{"op", "merge"},
                                                        {"source", "top"},
                                                        {"target", "left"}}})}};
         check_round_trip(*app::build_domain(section).geometry,
                          "an edited shape");
       }},
      {"every example's curves are a drawing, curve for curve",
       [] {
         app::Session session;
         std::size_t count = 0;
         for (const auto &entry :
              std::filesystem::directory_iterator(PDE2D_APP_EXAMPLES_DIR)) {
           if (entry.path().extension() != ".json")
             continue;
           const Json example = load_example(entry.path());
           const app::Domain domain = app::build_domain(example["geometry"]);
           if (!domain.geometry)
             continue;
           ++count;
           check_round_trip(*domain.geometry, entry.path().filename().string());
           // The session's call gives the same section.
           const Json answer = session.to_drawing(example);
           check(answer["geometry"] == app::drawing_section(*domain.geometry),
                 "the session's drawing");
           // The case with its drawing runs its geometry the same way.
           Json drawn = example;
           drawn["geometry"] = answer["geometry"];
           const Json a = session.geometry(example);
           const Json b = session.geometry(drawn);
           check(a["regions"] == b["regions"], "the same regions");
         }
         check(count >= 5, "the examples with curves");
       }},
      {"the example SVG file is a drawing, curve for curve",
       [] {
         std::ifstream in(std::filesystem::path(PDE2D_APP_EXAMPLES_DIR) /
                          "plate_with_hole.svg");
         const std::string svg((std::istreambuf_iterator<char>(in)),
                               std::istreambuf_iterator<char>());
         const app::Domain domain =
             app::build_domain({{"source", "svg"}, {"svg", svg}});
         check_round_trip(*domain.geometry, "plate_with_hole.svg");
       }},
      {"a drawing takes every preset and exports its geometry",
       [] {
         app::Session session;
         Json case_json =
             load_example(std::filesystem::path(PDE2D_APP_EXAMPLES_DIR) /
                          "cooling_fin.json");
         for (const Json &preset :
              app::Session::catalog()["problem"]["presets"]) {
           const Json preset_case = session.apply_preset(
               {{"case", case_json}, {"preset", preset["id"]}});
           check(preset_case["geometry"] == case_json["geometry"],
                 "the drawing stays");
           check(preset_case["problem"]["boundary_conditions"].size() == 5,
                 "a condition per region");
           const Json summary = session.run(preset_case);
           check(summary["ok"].get<bool>(),
                 preset["id"].get<std::string>() + ": " + summary.dump());
         }
         const auto file = session.export_file(case_json, "geometry.svg");
         check(file.text().find("<path") != std::string::npos, "an SVG");
         // The exported SVG reads back as the same regions.
         const app::Domain back =
             app::build_domain({{"source", "svg"}, {"svg", file.text()}});
         check(
             back.geometry->region_names() ==
                 std::vector<std::string>{"base", "tip", "top", "root", "hole"},
             "the regions back");
       }},
      {"a mesh file is not a drawing",
       [] {
         app::Session session;
         Json case_json = app::Session::default_case();
         case_json["geometry"] = rectangle_mesh_section();
         try {
           (void)session.to_drawing(case_json);
           throw test::Failure("a mesh file became a drawing");
         } catch (const app::StageError &error) {
           check(error.path() == "geometry.source", error.path());
         }
       }},
      {"locate: the nearest boundary point and its fraction",
       [] {
         // The default rectangle: 2 x 1, corners at (+-1, +-0.5).
         const app::Domain domain = app::build_domain(
             {{"source", "shape"}, {"shape", "rectangle"}, {"options", {}}});
         Json at = app::locate_boundary(domain, {0.5, -0.49}, 0.01);
         check(at["region"] == "bottom", at.dump());
         check(!at["corner"].get<bool>(), "not a corner");
         near(at["fraction"].get<double>(), 0.75, 1e-12, "the fraction");
         near(at["point"][0].get<double>(), 0.5, 1e-12, "x");
         check(at["point"][1].get<double>() == -0.5, "y on the line");
         check(at["splittable"].get<bool>(), "splittable");
         // At a corner the region ends: the library refuses.
         at = app::locate_boundary(domain, {0.995, -0.495}, 0.02);
         check(at["corner"].get<bool>(), "the corner");
         check(at["point"] == point(1, -0.5), "exactly the corner");
         check(!at["splittable"].get<bool>(), "a region's end");
         check(at["reason"].get<std::string>().find("end of the region") !=
                   std::string::npos,
               at.dump());
       }},
      {"locate: a corner inside a region splits exactly there",
       [] {
         // A hexagon: one region over six lines, the whole loop.
         Json section = {{"source", "shape"},
                         {"shape", "regular_polygon"},
                         {"options", Json::object()}};
         const app::Domain domain = app::build_domain(section);
         const auto &curves = domain.geometry->loops()[0].curves;
         const geometry::Point corner = geometry::start_point(curves[3].curve);
         // A point a little off the corner, within the snap.
         const Json at = app::locate_boundary(
             domain, {corner.x + 0.003, corner.y - 0.002}, 0.01);
         check(at["corner"].get<bool>(), "snapped");
         check(at["point"] == point(corner.x, corner.y), "the exact corner");
         check(at["splittable"].get<bool>(), at.dump());
         // The split there: no curve is cut, three curves each side.
         section["region_edits"] = Json::array({{{"op", "split"},
                                                 {"region", "boundary"},
                                                 {"fraction", at["fraction"]},
                                                 {"first", "boundary"},
                                                 {"second", "boundary_2"}}});
         const app::Domain split = app::build_domain(section);
         const auto &after = split.geometry->loops()[0].curves;
         check(after.size() == 6,
               "no curve was cut: " + std::to_string(after.size()));
         std::size_t first = 0;
         for (const auto &curve : after)
           first += curve.region == "boundary" ? 1 : 0;
         check(first == 3, "three curves keep the name");
         // A corner inside a merged region (bottom and right).
         Json rectangle = {
             {"source", "shape"},
             {"shape", "rectangle"},
             {"options", Json::object()},
             {"region_edits", Json::array({{{"op", "merge"},
                                            {"source", "right"},
                                            {"target", "bottom"}}})}};
         const app::Domain merged = app::build_domain(rectangle);
         const Json c = app::locate_boundary(merged, {1.004, -0.503}, 0.01);
         check(c["corner"].get<bool>() && c["point"] == point(1, -0.5),
               c.dump());
         check(c["fraction"].get<double>() * 3 == 2 ||
                   std::abs(c["fraction"].get<double>() - 2.0 / 3) < 1e-15,
               "two thirds of bottom + right");
         rectangle["region_edits"].push_back({{"op", "split"},
                                              {"region", "bottom"},
                                              {"fraction", c["fraction"]},
                                              {"first", "floor"},
                                              {"second", "wall"}});
         const app::Domain parts = app::build_domain(rectangle);
         const auto &loop = parts.geometry->loops()[0].curves;
         check(loop.size() == 4, "four lines still");
         check(loop[0].region == "floor" && loop[1].region == "wall",
               loop[0].region + ", " + loop[1].region);
         check(geometry::end_point(loop[0].curve) == geometry::Point{1, -0.5},
               "the floor ends at the corner exactly");
       }},
      {"locate: inside a curve the split lands on the located point",
       [] {
         Json section = {{"source", "shape"},
                         {"shape", "circle"},
                         {"options", Json::object()}};
         const app::Domain domain = app::build_domain(section);
         const Json at = app::locate_boundary(domain, {0.0, 1.2}, 0.01);
         check(at["region"] == "boundary" && !at["corner"].get<bool>(),
               at.dump());
         near(at["point"][0].get<double>(), 0.0, 1e-9, "x");
         near(at["point"][1].get<double>(), 1.0, 1e-9, "y");
         near(at["fraction"].get<double>(), 0.25, 1e-9, "a quarter");
         section["region_edits"] = Json::array({{{"op", "split"},
                                                 {"region", "boundary"},
                                                 {"fraction", at["fraction"]},
                                                 {"first", "a"},
                                                 {"second", "b"}}});
         const app::Domain split = app::build_domain(section);
         const geometry::Point end =
             geometry::end_point(split.geometry->loops()[0].curves[0].curve);
         near(end.x, at["point"][0].get<double>(), 1e-10, "split x");
         near(end.y, at["point"][1].get<double>(), 1e-10, "split y");
         // A region in two pieces cannot be split: the library says why.
         Json annulus = {
             {"source", "shape"},
             {"shape", "annulus"},
             {"options", Json::object()},
             {"region_edits", Json::array({{{"op", "merge"},
                                            {"source", "inner"},
                                            {"target", "outer"}}})}};
         const Json two =
             app::locate_boundary(app::build_domain(annulus), {1.0, 0.0}, 0.0);
         check(!two["splittable"].get<bool>() && two["fraction"].is_null(),
               two.dump());
         check(two["reason"].get<std::string>().find("several loops") !=
                   std::string::npos,
               two.dump());
       }},
      {"locate: a region that wraps around the loop's start",
       [] {
         // The rectangle's loop is bottom, right, top, left: bottom merged
         // into left is one region from the left side's start, across the
         // loop's start, to the bottom's end.
         Json section = {{"source", "shape"},
                         {"shape", "rectangle"},
                         {"options", Json::object()},
                         {"region_edits", Json::array({{{"op", "merge"},
                                                        {"source", "bottom"},
                                                        {"target", "left"}}})}};
         const app::Domain domain = app::build_domain(section);
         // The middle of the bottom: the left side (1) and half the bottom
         // (1) of the region's 3.
         const Json middle = app::locate_boundary(domain, {0.0, -0.52}, 0.01);
         check(middle["region"] == "left" && !middle["corner"].get<bool>(),
               middle.dump());
         near(middle["fraction"].get<double>(), 2.0 / 3, 1e-12,
              "bottom's middle");
         // The loop's start, a corner inside the region: exactly a third.
         const Json corner =
             app::locate_boundary(domain, {-0.996, -0.497}, 0.01);
         check(corner["corner"].get<bool>() &&
                   corner["point"] == point(-1, -0.5) &&
                   corner["splittable"].get<bool>(),
               corner.dump());
         check(std::abs(corner["fraction"].get<double>() - 1.0 / 3) < 1e-15,
               corner.dump());
         section["region_edits"].push_back({{"op", "split"},
                                            {"region", "left"},
                                            {"fraction", corner["fraction"]},
                                            {"first", "side"},
                                            {"second", "floor"}});
         const app::Domain split = app::build_domain(section);
         const auto &curves = split.geometry->loops()[0].curves;
         check(curves.size() == 4, "no curve was cut");
         check(curves[0].region == "floor" && curves[3].region == "side",
               curves[0].region + ", " + curves[3].region);
         // The middle of the bottom splits there.
         section["region_edits"][1]["fraction"] = middle["fraction"];
         const app::Domain cut = app::build_domain(section);
         const auto &pieces = cut.geometry->loops()[0].curves;
         check(pieces.size() == 5, "the bottom is cut");
         const geometry::Point end = geometry::end_point(pieces[0].curve);
         near(end.x, 0.0, 1e-12, "at the located point");
         check(end.y == -0.5, "on the bottom");
       }},
      {"locate: a point on a hole (a loop the library turned)",
       [] {
         // The annulus' inner circle runs clockwise after normalization.
         Json annulus = {{"source", "shape"},
                         {"shape", "annulus"},
                         {"options", Json::object()}};
         const app::Domain ring = app::build_domain(annulus);
         const Json at = app::locate_boundary(ring, {0.0, 0.45}, 0.01);
         check(at["region"] == "inner" && at["loop"] == 1, at.dump());
         near(at["point"][1].get<double>(), 0.4, 1e-12, "on the hole");
         annulus["region_edits"] = Json::array({{{"op", "split"},
                                                 {"region", "inner"},
                                                 {"fraction", at["fraction"]},
                                                 {"first", "inner"},
                                                 {"second", "lid"}}});
         const app::Domain split = app::build_domain(annulus);
         const geometry::Point end =
             geometry::end_point(split.geometry->loops()[1].curves[0].curve);
         near(end.x, 0.0, 1e-10, "the split lands on the point (x)");
         near(end.y, 0.4, 1e-10, "the split lands on the point (y)");
         // A square hole drawn counterclockwise, one region over its four
         // lines: the library turns it; a corner of it splits exactly.
         Json loop =
             polygon({{0.5, 0.5}, {1.5, 0.5}, {1.5, 1.5}, {0.5, 1.5}}, "hole");
         Json section = drawing(Json::array({square_loop(), loop}));
         const app::Domain drawn = app::build_domain(section);
         check(drawn.geometry->depth(1) == 1, "a hole");
         double area = 0;
         for (const auto &curve : drawn.geometry->loops()[1].curves) {
           const geometry::Point p = geometry::start_point(curve.curve);
           const geometry::Point q = geometry::end_point(curve.curve);
           area += p.x * q.y - q.x * p.y;
         }
         check(area < 0, "the library turned it clockwise");
         const Json corner = app::locate_boundary(drawn, {1.503, 1.497}, 0.01);
         check(corner["region"] == "hole" && corner["corner"].get<bool>() &&
                   corner["point"] == point(1.5, 1.5) &&
                   corner["splittable"].get<bool>(),
               corner.dump());
         // The drawing's split is the page's; the fraction is the
         // library's, checked here on the same curves as a shape would be.
         geometry::CurveGeometry geometry = *drawn.geometry;
         geometry.split_region("hole", corner["fraction"].get<double>(), "hole",
                               "hole_2");
         const auto &hole = geometry.loops()[1].curves;
         check(hole.size() == 4, "no curve of the hole was cut");
         std::size_t first = 0;
         for (const auto &curve : hole)
           first += curve.region == "hole" ? 1 : 0;
         check(first > 0 && first < 4, "both parts have lines");
         // The whole loop starts at its first curve: the parts change at
         // the corner and at that start, and nowhere else.
         const geometry::Point start = geometry::start_point(hole[0].curve);
         for (std::size_t c = 0; c < hole.size(); ++c)
           if (hole[c].region != hole[(c + 1) % hole.size()].region) {
             const geometry::Point change = geometry::end_point(hole[c].curve);
             check(change == geometry::Point{1.5, 1.5} || change == start,
                   "a change at the corner or the loop's start");
           }
         check(hole[0].region == "hole" && hole[3].region == "hole_2",
               "from the start to the corner, then to the end");
       }},
      {"locate through the session",
       [] {
         app::Session session;
         const Json case_json = app::Session::default_case();
         const Json at = session.locate(
             {{"case", case_json}, {"point", point(0, 0.6)}, {"snap", 0.01}});
         check(at["region"] == "top", at.dump());
         try {
           (void)session.locate({{"case", case_json}, {"point", "here"}});
           throw test::Failure("a point that is not one");
         } catch (const app::StageError &error) {
           check(error.path() == "point", error.path());
         }
         try {
           (void)session.locate(
               {{"case", case_json}, {"point", point(0, 0)}, {"snap", -1}});
           throw test::Failure("a negative snap");
         } catch (const app::StageError &error) {
           check(error.path() == "snap", error.path());
         }
       }},
      {"an imported mesh's region splits at a boundary node",
       [] {
         Json section = rectangle_mesh_section();
         const app::Domain plain = app::build_domain(section);
         const mesh::Mesh &m = *plain.mesh;
         const std::size_t bottom_edges = edges_of(m, "bottom");
         check(bottom_edges >= 4, "the bottom has edges");
         // The node the locator finds in the bottom's middle.
         const Json at = app::locate_boundary(plain, {0.01, -0.52}, 0.0);
         check(at["region"] == "bottom" && at["splittable"].get<bool>(),
               at.dump());
         const std::size_t node = at["node"].get<std::size_t>();
         section["region_edits"] = Json::array({{{"op", "split_at_node"},
                                                 {"region", "bottom"},
                                                 {"node", node},
                                                 {"first", "bottom"},
                                                 {"second", "outlet"}}});
         const app::Domain split = app::build_domain(section);
         const mesh::Mesh &s = *split.mesh;
         check(s.region_names == std::vector<std::string>{"bottom", "right",
                                                          "top", "left",
                                                          "outlet"},
               "a new region at the end");
         check(edges_of(s, "bottom") + edges_of(s, "outlet") == bottom_edges,
               "the bottom's edges are shared");
         check(edges_of(s, "bottom") > 0 && edges_of(s, "outlet") > 0,
               "both have edges");
         // The bottom runs left to right (domain on the left): its first
         // part is left of the node, the outlet right of it.
         const double split_x = s.nodes[node].x;
         for (const mesh::BoundaryEdge &edge : s.boundary_edges) {
           const std::string &name = s.region_names[edge.region];
           const double mid =
               (s.nodes[edge.nodes[0]].x + s.nodes[edge.nodes[1]].x) / 2;
           if (name == "bottom")
             check(mid < split_x, "bottom left of the node");
           if (name == "outlet")
             check(mid > split_x, "outlet right of the node");
         }
         mesh::validate(s);
         // The whole case runs with the new region.
         app::Session session;
         Json case_json = app::Session::default_case();
         case_json["geometry"] = section;
         case_json["problem"]["boundary_conditions"]["outlet"] =
             app::default_condition();
         check(session.run(case_json)["ok"].get<bool>(), "runs");

         // Refusals name the edit.
         const auto refused = [&](Json edit, const std::string &fragment) {
           Json bad = rectangle_mesh_section();
           bad["region_edits"] = Json::array({edit});
           expect_throws<std::invalid_argument>(
               [&] { (void)app::build_domain(bad); }, fragment);
         };
         const std::size_t corner = [&] {
           for (const mesh::BoundaryEdge &edge : m.boundary_edges)
             if (m.region_names[edge.region] == "bottom" &&
                 m.nodes[edge.nodes[0]].x == -1)
               return edge.nodes[0];
           throw test::Failure("no corner");
         }();
         refused({{"op", "split_at_node"},
                  {"region", "bottom"},
                  {"node", corner},
                  {"first", "a"},
                  {"second", "b"}},
                 "geometry.region_edits[0]: split: node " +
                     std::to_string(corner) + " is an end of the region");
         refused({{"op", "split_at_node"},
                  {"region", "top"},
                  {"node", node},
                  {"first", "a"},
                  {"second", "b"}},
                 "is not on the region 'top'");
         refused({{"op", "split_at_node"},
                  {"region", "bottom"},
                  {"node", node},
                  {"first", "a"},
                  {"second", "top"}},
                 "split: region 'top' already exists");
         refused({{"op", "split_at_node"},
                  {"region", "bottom"},
                  {"node", -1},
                  {"first", "a"},
                  {"second", "b"}},
                 "geometry.region_edits[0].node: expected a node number");
         refused({{"op", "split_at_node"},
                  {"region", "bottom"},
                  {"node", node},
                  {"fraction", 0.5},
                  {"first", "a"},
                  {"second", "b"}},
                 "unknown key 'fraction'");
         // Two pieces after a merge of the opposite sides.
         Json two = rectangle_mesh_section();
         two["region_edits"] = Json::array(
             {{{"op", "merge"}, {"source", "top"}, {"target", "bottom"}},
              {{"op", "split_at_node"},
               {"region", "bottom"},
               {"node", node},
               {"first", "a"},
               {"second", "b"}}});
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_domain(two); },
             "geometry.region_edits[1]: split: region 'bottom' has several "
             "separate pieces");
         // A curve split is not for a mesh file, and the other way round.
         refused({{"op", "split"},
                  {"region", "bottom"},
                  {"fraction", 0.5},
                  {"first", "a"},
                  {"second", "b"}},
                 "split it at a boundary node");
         Json shape = {{"source", "shape"},
                       {"shape", "rectangle"},
                       {"options", Json::object()},
                       {"region_edits", Json::array({{{"op", "split_at_node"},
                                                      {"region", "bottom"},
                                                      {"node", 1},
                                                      {"first", "a"},
                                                      {"second", "b"}}})}};
         expect_throws<std::invalid_argument>(
             [&] { (void)app::build_domain(shape); },
             "geometry.region_edits[0].op: split_at_node is for a mesh file");
       }},
      {"a whole closed region of a mesh splits from its first edge",
       [] {
         Json section = rectangle_mesh_section();
         // One region around the whole boundary.
         section["region_edits"] = Json::array(
             {{{"op", "merge"}, {"source", "right"}, {"target", "bottom"}},
              {{"op", "merge"}, {"source", "top"}, {"target", "bottom"}},
              {{"op", "merge"}, {"source", "left"}, {"target", "bottom"}}});
         const app::Domain whole = app::build_domain(section);
         const Json at = app::locate_boundary(whole, {0.02, 0.52}, 0.0);
         check(at["splittable"].get<bool>(), at.dump());
         section["region_edits"].push_back({{"op", "split_at_node"},
                                            {"region", "bottom"},
                                            {"node", at["node"]},
                                            {"first", "rest"},
                                            {"second", "lid"}});
         const app::Domain split = app::build_domain(section);
         check(split.mesh->region_names ==
                   std::vector<std::string>{"rest", "lid"},
               "two regions");
         check(edges_of(*split.mesh, "lid") > 0 &&
                   edges_of(*split.mesh, "rest") > 0,
               "both have edges");
       }},
  });
}

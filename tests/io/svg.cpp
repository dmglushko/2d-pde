#include "support/check.hpp"
#include "support/mesh_checks.hpp"
#include "support/shapes.hpp"

#include "pde2d/geometry/polygon.hpp"
#include "pde2d/geometry/shapes.hpp"
#include "pde2d/io/svg.hpp"
#include "pde2d/mesh/cdt.hpp"
#include "pde2d/mesh/chord_length_discretizer.hpp"
#include "pde2d/mesh/mesher.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numbers>
#include <sstream>
#include <string>

using namespace pde2d;
using namespace pde2d::test;
using geometry::Curve;
using geometry::Point;

namespace {

constexpr double pi = std::numbers::pi;

geometry::CurveGeometry from_text(const std::string &text) {
  std::istringstream in(text);
  return io::import_svg(in);
}

std::string document(const std::string &body) {
  return "<?xml version=\"1.0\"?>\n<svg xmlns=\"http://www.w3.org/2000/svg\" "
         "viewBox=\"0 0 10 10\" width=\"10cm\">\n" +
         body + "\n</svg>\n";
}

std::string path(const std::string &attributes, const std::string &d) {
  return "<path " + attributes + " d=\"" + d + "\"/>";
}

std::string to_text(const geometry::CurveGeometry &g) {
  std::ostringstream out;
  io::export_svg(g, out);
  return out.str();
}

bool same_point(Point a, Point b, double tolerance) {
  return distance(a, b) <= tolerance;
}

// The curve kinds in order: L line, Q quadratic, C cubic, A arc.
std::string kinds(const geometry::Loop &loop) {
  std::string s;
  for (const auto &c : loop.curves)
    s += "LQCA"[c.curve.index()];
  return s;
}

// a and b describe the same boundary: same loops, depths and regions, and
// curve by curve the same points. With `exported`, each arc of `a` comes
// back as several arcs in `b` (the exporter writes arcs in pieces of at
// most a quarter turn).
void check_same_geometry(const geometry::CurveGeometry &a,
                         const geometry::CurveGeometry &b, double tolerance,
                         bool exported = true) {
  check(a.loops().size() == b.loops().size(), "same number of loops");
  for (std::size_t l = 0; l < a.loops().size(); ++l) {
    check(a.depth(l) == b.depth(l), "same depth");
    const auto &ca = a.loops()[l].curves;
    const auto &cb = b.loops()[l].curves;
    std::size_t j = 0;
    for (const auto &c : ca) {
      int pieces = 1;
      const auto *arc = std::get_if<geometry::EllipticalArc>(&c.curve);
      if (arc && exported)
        pieces = static_cast<int>(
            std::ceil(std::abs(arc->sweep_angle) / (pi / 2) * (1 - 1e-12)));
      for (int k = 0; k < pieces; ++k, ++j) {
        check(j < cb.size(), "enough curves");
        check(cb[j].region == c.region, "same region");
        check(cb[j].curve.index() == c.curve.index(), "same kind");
        for (double u : {0.0, 0.25, 0.5, 0.75, 1.0})
          check(same_point(geometry::point_at(cb[j].curve, u),
                           geometry::point_at(c.curve, (k + u) / pieces),
                           tolerance),
                "same points");
      }
    }
    check(j == cb.size(), "no extra curves");
  }
}

// Every curve kind. The SVG y axis points down and import flips it, so
// the path is written with y negated (and the arc's sweep flag 0): it is
// counterclockwise once imported and the geometry keeps its direction.
const std::string all_commands =
    "M 0 0 H 4 V -1 Q 4.5 -2 3.5 -2 T 2 -2.2 C 1.5 -2.6 1 -2.6 0.5 -2.2 "
    "S 0 -1.4 0 -1 A 0.5 0.5 0 0 0 0 0 Z";
const std::string all_commands_relative =
    "m 0 0 h 4 v -1 q 0.5 -1 -0.5 -1 t -1.5 -0.2 c -0.5 -0.4 -1 -0.4 -1.5 0 "
    "s -0.5 0.8 -0.5 1.2 a 0.5 0.5 0 0 0 0 1 z";

} // namespace

int main() {
  return run({
      {"path_commands_map_to_exact_curves",
       [] {
         auto g = from_text(document(path("id=\"wall\"", all_commands)));
         check(g.loops().size() == 1, "one loop");
         const auto &loop = g.loops()[0];
         check(kinds(loop) == "LLQQCCA", "kinds: " + kinds(loop));
         const auto &c = loop.curves;
         auto line = std::get<geometry::Line>(c[0].curve);
         check(line.start == Point{0, 0} && line.end == Point{4, 0}, "H");
         check(std::get<geometry::Line>(c[1].curve).end == Point{4, 1}, "V");
         auto t = std::get<geometry::QuadraticBezier>(c[3].curve);
         check(t.control == Point{2.5, 2}, "T reflects the Q control");
         auto s = std::get<geometry::CubicBezier>(c[5].curve);
         near(s.control1.x, 0, 1e-15, "S reflects the C control");
         near(s.control1.y, 1.8, 1e-15, "S reflects the C control");
         auto arc = std::get<geometry::EllipticalArc>(c[6].curve);
         near(arc.center.x, 0, 1e-15, "arc center x");
         near(arc.center.y, 0.5, 1e-15, "arc center y");
         near(arc.radius_x, 0.5, 1e-15, "arc radius");
         near(arc.sweep_angle, pi, 1e-12, "half a turn, positive");
         near(geometry::point_at(arc, 0.5).x, -0.5, 1e-12, "bulges left");
         for (const auto &bc : c)
           check(bc.region == "wall", "region from the id");
         // Relative commands give the same curves.
         auto r =
             from_text(document(path("id=\"wall\"", all_commands_relative)));
         check_same_geometry(g, r, 1e-12, false);
       }},
      {"path_data_syntax",
       [] {
         // Compact numbers, commas, implicit repetition, exponents and
         // compact arc flags.
         auto g = from_text(
             document(path("id=\"a\"", "M0,0L2,0,2,1e0,.5,1.5-.5.5Z")));
         check(kinds(g.loops()[0]) == "LLLLL", "five lines");
         auto arc =
             from_text(document(path("id=\"a\"", "M0 0h2a1 1 0 001-1v-1H0z")));
         check(kinds(arc.loops()[0]) == "LALLL", "compact arc flags");
         // After Z, a command other than M starts at the subpath's start.
         auto two = from_text(
             document(path("id=\"a\"", "M0 0h3v3h-3z m1 1 v1 h1 v-1 z")));
         check(two.loops().size() == 2 && two.depth(1) == 1,
               "two subpaths of one path: a square with a hole");
         // Zero-length segments are dropped; Z adds the closing line.
         auto drop =
             from_text(document(path("id=\"a\"", "M0 0 L0 0 H1 V0 V1 Z")));
         check(kinds(drop.loops()[0]) == "LLL", "zero-length lines dropped");
         // A zero radius makes the arc a line.
         auto flat =
             from_text(document(path("id=\"a\"", "M0 0 A 0 1 0 0 1 1 0 V1 Z")));
         check(kinds(flat.loops()[0]) == "LLL", "A with rx = 0 is a line");
         // Radii too small are scaled up: a half circle of radius 1.
         auto scaled = from_text(
             document(path("id=\"a\"", "M0 0 A 0.1 0.1 0 0 0 2 0 Z")));
         auto half = std::get<geometry::EllipticalArc>(
             scaled.loops()[0].curves[0].curve);
         near(half.radius_x, 1, 1e-12, "scaled radius");
       }},
      {"arcs_at_extreme_scales",
       [] {
         // A quarter-circle sector of radius r: the radii's fourth powers
         // overflow or underflow, so the conversion must not form them.
         // (Written with y negated: (0, -r) in the file is (0, r).)
         for (double r : {1e-100, 1e-300, 1e100, 1e300}) {
           std::ostringstream text;
           text.precision(17);
           text << "M0 0 L" << r << " 0 A" << r << " " << r << " 0 0 0 0 " << -r
                << " Z";
           auto g = from_text(document(path("id=\"a\"", text.str())));
           const auto &curves = g.loops()[0].curves;
           check(kinds(g.loops()[0]) == "LAL", "line, arc, line");
           auto arc = std::get<geometry::EllipticalArc>(curves[1].curve);
           near(arc.center.x / r, 0, 1e-15, "center x");
           near(arc.center.y / r, 0, 1e-15, "center y");
           near(arc.radius_x / r, 1, 1e-15, "radius");
           Point end = geometry::end_point(arc);
           near(end.x / r, 0, 1e-15, "arc ends at (0, r): x");
           near(end.y / r, 1, 1e-15, "arc ends at (0, r): y");
         }
       }},
      {"nearly_flat_arcs_end_exactly",
       [] {
         // A radius far larger than the chord: an arc through its far-away
         // center would miss (1, 0) by about 1e8 * 2^-53, more than the
         // loop's closure tolerance.
         for (double r : {1e8, 1e12, 1e20}) {
           std::ostringstream text;
           text.precision(17);
           text << "M0 0 A" << r << " " << r << " 0 0 0 1 0 L1 -1 L0 -1 Z";
           auto g = from_text(document(path("id=\"a\"", text.str())));
           const auto &c = g.loops()[0].curves;
           check(kinds(g.loops()[0]) == "CLLL", "a flat arc is a cubic");
           check(geometry::start_point(c[0].curve) == Point{0, 0} &&
                     geometry::end_point(c[0].curve) == Point{1, 0},
                 "the flat arc ends exactly at its end points");
           // The sagitta of a chord of length 1: 1 / (8 r).
           Point mid = geometry::point_at(c[0].curve, 0.5);
           near(mid.x, 0.5, 1e-15, "middle x");
           near(std::abs(mid.y), 1 / (8 * r), 1e-6 / (8 * r), "sagitta");
         }
         // An ellipse: radii 1e9 and 1e7, rotated 30 degrees.
         auto g = from_text(document(
             path("id=\"a\"", "M0 0 A1e9 1e7 -30 0 1 1 0 L1 -1 L0 -1 Z")));
         check(kinds(g.loops()[0]) == "CLLL", "flat elliptical arc");
       }},
      {"y_axis_points_up",
       [] {
         // (0, 0) -> (1, 0) -> (1, 1) -> (0, 1) in the file is clockwise on
         // the screen; with y flipped it runs through (1, -1), clockwise in
         // y-up terms, and CurveGeometry makes it counterclockwise.
         auto g = from_text(document(path("id=\"a\"", "M0 0 H1 V1 H0 Z")));
         const auto &c = g.loops()[0].curves;
         check(kinds(g.loops()[0]) == "LLLL", "four lines");
         std::vector<Point> corners;
         for (const auto &bc : c)
           corners.push_back(geometry::start_point(bc.curve));
         for (Point p : corners)
           check(p.y <= 0, "y flipped: the square is below the x axis");
         check(std::find(corners.begin(), corners.end(), Point{1, -1}) !=
                   corners.end(),
               "(1, 1) in the file is (1, -1)");
         check(geometry::polygon_orientation(corners) == 1,
               "normalized counterclockwise");
         // What is at the top of the drawing (small SVG y) has the largest
         // y after import.
         auto top = from_text(document(path("id=\"top\"", "M0 0 H2") +
                                       path("id=\"rest\"", "M2 0 V1 H0 V0")));
         for (const auto &bc : top.loops()[0].curves)
           if (bc.region == "top") {
             check(geometry::start_point(bc.curve).y == 0 &&
                       geometry::end_point(bc.curve).y == 0,
                   "the top edge stays at y = 0");
           } else {
             check(geometry::start_point(bc.curve).y <= 0 &&
                       geometry::end_point(bc.curve).y <= 0,
                   "the rest lies below it");
           }
         // Holes: both loops drawn the same way in the file; the outer one
         // is made counterclockwise, the hole clockwise.
         auto holed = from_text(document(path("id=\"o\"", "M0 0 H3 V3 H0 Z") +
                                         path("id=\"h\"", "M1 1 H2 V2 H1 Z")));
         check(holed.depth(0) == 0 && holed.depth(1) == 1, "depths");
         for (std::size_t l = 0; l < 2; ++l) {
           std::vector<Point> p;
           for (const auto &bc : holed.loops()[l].curves)
             p.push_back(geometry::start_point(bc.curve));
           check(geometry::polygon_orientation(p) == (l == 0 ? 1 : -1),
                 "even depth counterclockwise, odd depth clockwise");
         }
         // An arc: from (1, 0) to (0, -1) in the file, sweep flag 0, is the
         // quarter circle through (cos 45, sin 45) once flipped.
         auto quarter = from_text(
             document(path("id=\"q\"", "M1 0 A1 1 0 0 0 0 -1 L0 0 Z")));
         for (const auto &bc : quarter.loops()[0].curves)
           if (const auto *arc =
                   std::get_if<geometry::EllipticalArc>(&bc.curve)) {
             Point mid = geometry::point_at(*arc, 0.5);
             near(mid.x, std::sqrt(0.5), 1e-15, "arc middle x");
             near(mid.y, std::sqrt(0.5), 1e-15, "arc middle y");
           }
         // Export writes y negated, and import(export(g)) gives the same
         // numbers back.
         auto r = geometry::rectangle(2, 1);
         std::string text = to_text(r);
         check(text.find(" -0.5") != std::string::npos, "y negated on export");
         auto back = from_text(text);
         for (std::size_t i = 0; i < r.loops()[0].curves.size(); ++i) {
           auto a = std::get<geometry::Line>(r.loops()[0].curves[i].curve);
           auto b = std::get<geometry::Line>(back.loops()[0].curves[i].curve);
           check(a.start == b.start && a.end == b.end, "exact round trip");
         }
       }},
      {"region_names",
       [] {
         auto g = from_text(
             document(path("id=\"p1\" data-region=\"inlet\"", "M0 0 L0 1") +
                      path("id=\"wall\"", "M0 1 L1 1 L1 0 L0 0")));
         check(g.loops().size() == 1, "two paths make one loop");
         check(g.region_names() == std::vector<std::string>{"inlet", "wall"} ||
                   g.region_names() ==
                       std::vector<std::string>{"wall", "inlet"},
               "data-region before id");
         expect_throws<std::invalid_argument>(
             [] { from_text(document(path("class=\"x\"", "M0 0 H1 V1 Z"))); },
             "SVG path 1 has no id or data-region attribute");
         expect_throws<std::invalid_argument>(
             [] { from_text(document(path("id=\"a b\"", "M0 0 H1 V1 Z"))); },
             "region name 'a b' is invalid");
         // Several paths may share a region through data-region.
         auto shared = from_text(
             document(path("id=\"s1\" data-region=\"w\"", "M0 0 H1 V1") +
                      path("id=\"s2\" data-region=\"w\"", "M1 1 H0 V0")));
         check(shared.region_names() == std::vector<std::string>{"w"},
               "one region");
       }},
      {"loops_from_runs_and_holes_from_depth",
       [] {
         // A square drawn as two open paths, the second one backwards, and
         // a hole drawn in the same direction as the outer loop.
         auto g = from_text(document(path("id=\"bottom\"", "M0 0 H4 V4") +
                                     path("id=\"rest\"", "M0 0 V4 H4") +
                                     path("id=\"hole\"", "M1 1 H3 V3 H1 Z")));
         check(g.loops().size() == 2, "two loops");
         check(g.depth(0) == 0 && g.depth(1) == 1, "the inner loop is a hole");
         check(g.loops()[0].curves.size() == 4, "four sides");
         // Groups are entered; metadata and other namespaces are ignored.
         auto grouped = from_text(
             "<svg xmlns=\"http://www.w3.org/2000/svg\" "
             "xmlns:x=\"urn:example\"><title>t</title><defs><path d=\"M9 "
             "9\"/></defs><x:thing/><g id=\"layer\"><g><path id=\"a\" "
             "d=\"M0 0 H1 V1 Z\" fill=\"red\" style=\"stroke:none\"/></g></g>"
             "<!-- comment --></svg>");
         check(grouped.loops().size() == 1, "path inside groups");
         auto plain =
             from_text("<svg><path id=\"a\" d=\"M0 0 H1 V1 Z\"/></svg>");
         check(plain.loops().size() == 1, "no namespace is fine too");
       }},
      {"unsupported_input_is_rejected",
       [] {
         auto rejects = [](const std::string &text, const char *fragment) {
           expect_throws<std::invalid_argument>([&] { from_text(text); },
                                                fragment);
         };
         rejects(document("<g transform=\"scale(2)\">" +
                          path("id=\"a\"", "M0 0 H1 V1 Z") + "</g>"),
                 "<g> has a transform attribute, which is not supported");
         rejects(
             document(path("id=\"a\" transform=\"rotate(3)\"", "M0 0 H1 V1 Z")),
             "<path> has a transform attribute");
         rejects(document("<rect id=\"r\" width=\"1\" height=\"1\"/>"),
                 "SVG <rect> is not supported; convert it to a <path>");
         rejects(document("<text>hi</text>"), "SVG element <text> is not "
                                              "supported");
         rejects(document(path("id=\"a\"", "M0 0 H1 V1")),
                 "SVG boundary is open: nothing continues it at (1, 1)");
         rejects(document(path("id=\"a\"", "M0 0 H1 V1 Z") +
                          path("id=\"b\"", "M1 1 H2") +
                          path("id=\"c\"", "M1 1 V2")),
                 "SVG boundary is open");
         rejects(document(path("id=\"a\"", "M0 0 H2 V1") +
                          path("id=\"b\"", "M2 1 H3") +
                          path("id=\"c\"", "M2 1 V2")),
                 "more than two path ends meet at (2, 1)");
         rejects(document(path("id=\"a\"", "M0 0 X 1 1")),
                 "unsupported path command 'X'");
         rejects(document(path("id=\"a\"", "M0 0 L 1")), "expected a number");
         rejects(document(path("id=\"a\"", "M0 0 L 1e999 0")),
                 "is not a finite double");
         rejects(document(path("id=\"a\"", "L0 0 H1 V1 Z")),
                 "must start with M or m");
         rejects(document(path("id=\"a\"", "M0 0 A 1 1 0 2 1 1 1")),
                 "expected an arc flag 0 or 1");
         rejects(document(path("id=\"a\"", "")), "the path data is empty");
         rejects(document(""), "SVG document has no path with curves");
         rejects("<svg><path id=\"a\" d=\"M0 0\"", "not well-formed XML");
         rejects("<html/>", "The root element is not <svg>");
         rejects("<!DOCTYPE svg [<!ENTITY x \"M0 0 H1 V1 Z\">]><svg><path "
                 "id=\"a\" d=\"&x;\"/></svg>",
                 "with a DTD are not supported");
         // A self-crossing loop is an invalid geometry.
         rejects(document(path("id=\"a\"", "M0 0 L1 1 L1 0 L0 1 Z")),
                 "SVG geometry is invalid");
         expect_throws<std::runtime_error>(
             [] {
               (void)io::import_svg(std::filesystem::path("/no/such.svg"));
             },
             "Cannot open '/no/such.svg' for reading");
       }},
      {"round_trip",
       [] {
         using namespace pde2d::geometry;
         auto every_kind =
             from_text(document(path("id=\"wall\"", all_commands)));
         std::vector<CurveGeometry> cases{
             rectangle(2, 1), annulus(1, 0.4),
             ellipse(2, 1).transformed({.rotation = 0.3}),
             rectangle_with_semicircle(3, 1), holes_and_island(), every_kind,
             // Regions that repeat inside a loop, and a name that is not a
             // valid XML id.
             CurveGeometry({Loop{{{Line{{0, 0}, {1, 0}}, "a"},
                                  {Line{{1, 0}, {1, 1}}, "9b"},
                                  {Line{{1, 1}, {0, 1}}, "a"},
                                  {Line{{0, 1}, {0, 0}}, "a"}}}})};
         for (const auto &g : cases) {
           std::string text = to_text(g);
           auto back = from_text(text);
           check_same_geometry(g, back, 1e-12);
           check(back.region_names() == g.region_names(), "region names");
         }
         check(to_text(from_text(to_text(cases[0]))) == to_text(cases[0]),
               "lines: exporting again gives the same text");
         std::string ids = to_text(cases.back());
         check(ids.find("id=\"pde2d_run_1\" data-region=\"a\"") !=
                   std::string::npos,
               "a repeated region goes to data-region");
         check(ids.find("data-region=\"9b\"") != std::string::npos,
               "an invalid id goes to data-region");
         check(to_text(rectangle(2, 1)).find("id=\"bottom\"") !=
                   std::string::npos,
               "a unique region is the id");
         // Shortest exact numbers.
         check(to_text(CurveGeometry(
                           {polygon_loop({{0, 0}, {0.1, 0}, {0.1, 0.3}}, "t")}))
                       .find("M 0 0 L 0.1 0 L 0.1 -0.3") != std::string::npos,
               "shortest numbers, y negated");
       }},
      {"mesh_svg",
       [] {
         auto m = mesh::Mesher(std::make_shared<mesh::ChordLengthDiscretizer>(),
                               std::make_shared<mesh::Cdt>())
                      .run(holes_and_island(), {1.0})
                      .mesh;
         std::ostringstream out;
         io::export_mesh_svg(m, out);
         std::string svg = out.str();
         std::size_t triangles = 0, at = svg.find("class=\"triangles\"");
         std::string tri = svg.substr(at, svg.find("/>", at) - at);
         for (char c : tri)
           triangles += c == 'Z';
         check(triangles == m.triangles.size(), "one subpath per triangle");
         for (const auto &name : m.region_names) {
           check(svg.find("data-region=\"" + name + "\"") != std::string::npos,
                 "a path per region");
           check(svg.find("<title>" + name + "</title>") != std::string::npos,
                 "with a title");
         }
         // y points down in the picture: the mesh in [0, 6]^2 is drawn in
         // [0, 6] x [-6, 0], so it looks like the drawing.
         check(svg.find("viewBox=\"-0.12 -6.12 6.24 6.24\"") !=
                   std::string::npos,
               "bounding box [0, 6] x [-6, 0] with a 2% margin");
         mesh::Mesh empty;
         expect_throws<std::invalid_argument>(
             [&] { io::export_mesh_svg(empty, out); }, "has no nodes");
       }},
  });
}

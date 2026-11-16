#pragma once

#include "options.hpp"

#include "pde2d/geometry/curve_geometry.hpp"

#include <string>
#include <string_view>
#include <vector>

// The "drawing" source of the geometry section: loops of exact curves
// with shared vertices, edited in the page (docs/app.md, "The drawing").
//
//   {"source": "drawing",
//    "loops": [{"vertices": [[x, y], ...],
//               "edges": [{"kind": "line", "region": "bottom"},
//                         {"kind": "arc", "region": "cap",
//                          "center": [x, y], "radius_x": r, "radius_y": r,
//                          "rotation": 0, "start_angle": a,
//                          "sweep_angle": s},
//                         {"kind": "quadratic", "region": "r",
//                          "control": [x, y]},
//                         {"kind": "cubic", "region": "r",
//                          "control1": [x, y], "control2": [x, y]}]}]}
//
// Edge i of a loop runs from vertex i to vertex i + 1 (the last to vertex
// 0), so a loop has as many edges as vertices. A line or a Bezier takes
// its ends from the vertices; an arc is stored exactly in the library's
// centre form (geometry::EllipticalArc), and its ends must lie on its
// vertices (within 1e-9 of the drawing's size, the library's closure
// tolerance). A full circle is a loop of one vertex and one arc of sweep
// 2 pi. The library's CurveGeometry decides holes by containment and
// normalizes the direction; a drawing has no region edits (its edges carry
// the final regions).
namespace pde2d::app {

inline constexpr std::string_view drawing_section_keys[] = {"source", "loops"};
inline constexpr std::string_view drawing_loop_keys[] = {"vertices", "edges"};
inline constexpr std::string_view drawing_edge_kinds[] = {"line", "arc",
                                                          "quadratic", "cubic"};
inline constexpr std::string_view line_edge_keys[] = {"kind", "region"};
inline constexpr std::string_view arc_edge_keys[] = {
    "kind",     "region",   "center",      "radius_x",
    "radius_y", "rotation", "start_angle", "sweep_angle"};
inline constexpr std::string_view quadratic_edge_keys[] = {"kind", "region",
                                                           "control"};
inline constexpr std::string_view cubic_edge_keys[] = {"kind", "region",
                                                       "control1", "control2"};

// The geometry of a drawing section. Throws PathError at the place that is
// wrong: the JSON (geometry.loops[1].vertices[2]), an arc whose end is not
// on its vertex, and the library's own refusals (a curve it refuses, loops
// that cross, a loop that is not closed), whose loop and curve indices are
// the drawing's (geometry.loops[1].edges[3], or geometry.loops[1]).
geometry::CurveGeometry build_drawing(const Json &section);

// The path in a drawing of the library's CurveGeometry message ("Loop 1,
// curve 3: ..." is geometry.loops[1].edges[3]; "Loops 0 and 2 intersect
// or touch (Loop 0, curve 1 and Loop 2, curve 0)" the last curve named);
// "geometry.loops" when it names none.
std::string drawing_error_path(std::string_view message);

// A drawing section with exactly the curves of `geometry` (its normalized
// loops, curve for curve, with their regions): reading it back with
// build_drawing gives the same curves bit for bit, as long as consecutive
// lines and Beziers of a loop share their end points exactly (an arc keeps
// its own data; its vertex is the neighbour's end point).
Json drawing_section(const geometry::CurveGeometry &geometry);

} // namespace pde2d::app

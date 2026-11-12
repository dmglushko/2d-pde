#pragma once

// Private to the io library: the SVG path data parser.

#include "pde2d/geometry/curve.hpp"

#include <string_view>
#include <vector>

namespace pde2d::io::detail {

// One subpath: the curves from one M to the next M (or the end).
struct SvgSubpath {
  std::vector<geometry::Curve> curves;
  // Ended by Z: the importer adds a closing line if the subpath does not
  // end where it starts (within the drawing's tolerance).
  bool closed = false;
  geometry::Point start{}, end{};
};

// Parses the d attribute of a <path> (SVG 1.1 path grammar, all
// commands). Zero-length segments are dropped, and so are subpaths left
// without any curve. Throws std::invalid_argument naming the problem.
std::vector<SvgSubpath> parse_path_data(std::string_view data);

// The SVG endpoint parameterization of an elliptical arc converted to an
// exact curve (SVG 1.1, appendix F.6): a Line if a radius is zero, else
// an EllipticalArc. Radii too small to reach the end point are scaled up.
// The conversion is done on the unit circle, so radii of any size work.
// A nearly flat arc (sweep below 0.01 radians, for example a huge radius
// over a short chord) becomes a CubicBezier through exactly the given end
// points: an EllipticalArc would miss them by its center's rounding, and
// the Bezier deviates from the true arc by less than that. The end points
// must differ.
geometry::Curve svg_arc(geometry::Point from, geometry::Point to, double rx,
                        double ry, double rotation_degrees, bool large_arc,
                        bool sweep);

// SVG's y axis points down, the library's up: import and export mirror
// every point, (x, y) -> (x, -y). Mirroring twice gives the same numbers
// back (0 - y is exact, and gives +0 rather than -0 for y = 0). A
// mirrored curve runs the opposite way round (CurveGeometry normalizes
// loop directions afterwards); an arc's rotation, start angle and sweep
// change sign.
geometry::Point mirrored_y(geometry::Point p);
geometry::Curve mirrored_y(const geometry::Curve &curve);

} // namespace pde2d::io::detail

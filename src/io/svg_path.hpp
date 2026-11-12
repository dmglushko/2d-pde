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
// The end points must differ.
geometry::Curve svg_arc(geometry::Point from, geometry::Point to, double rx,
                        double ry, double rotation_degrees, bool large_arc,
                        bool sweep);

} // namespace pde2d::io::detail

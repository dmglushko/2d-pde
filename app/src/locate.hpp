#pragma once

#include "domain_input.hpp"
#include "options.hpp"

#include "pde2d/geometry/point.hpp"

// Where on the boundary a point of the picture is, for splitting a region
// there (POST /api/geometry/locate; docs/app.md, "Splitting a region").
namespace pde2d::app {

// The boundary point nearest to `point`, and how to split its region
// there.
//
// For curves (a shape, an SVG, a drawing): the nearest point of the exact
// curves (sampled, then refined on the curve's parameter). Within `snap`
// of a curve's end point (a corner, also every region's end) it is that
// end point exactly. The answer:
//   {"kind": "curves", "region", "loop", "curve", "t", "point": [x, y],
//    "corner": bool, "fraction": number or null, "splittable": bool,
//    "reason": text or null, "distance"}
// `fraction` is CurveGeometry::split_region's: the arc length from the
// region's start (along the normalized loop) to the point over the
// region's length, summed curve by curve as the library sums it, so a
// corner gives exactly the library's junction. Whether the split works is
// asked of the library (the split is tried on a copy); `reason` is its
// message when it does not ("the split point coincides with an end of the
// region", a region in several pieces).
//
// For a mesh file: the boundary node nearest to `point` on the nearest
// boundary edge ("node", "point", "region", "corner": true, "splittable",
// "reason"), for the split_at_node edit; tried on a copy as well.
//
// Indices (loop, curve) are the geometry's (normalized) ones; a drawing's
// edges are split in the page.
Json locate_boundary(const Domain &domain, geometry::Point point, double snap);

} // namespace pde2d::app

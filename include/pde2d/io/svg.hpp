#pragma once

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/mesh.hpp"

#include <filesystem>
#include <istream>
#include <ostream>

// SVG import and export of geometries, and SVG export of meshes for
// looking at them. Coordinates are SVG user units, used as they are: the
// SVG y axis points down and is not flipped, so a geometry exported and
// viewed in a browser looks like the drawing it was imported from.
namespace pde2d::io {

// Reads the <path> elements of an SVG document into a CurveGeometry.
//
// - Each <path> gives exact curves: M/m L/l H/h V/v give lines, C/c S/s
//   cubic Bezier curves, Q/q T/t quadratic Bezier curves, A/a elliptical
//   arcs (a zero radius gives a line), Z/z a closing line where needed.
//   Segments of zero length are dropped.
// - Region name: the path's data-region attribute if it has one, else its
//   id. A path without either is an error. Region names follow
//   geometry::is_valid_region_name.
// - Each subpath (each M) is a run of curves. Runs that end where they
//   start are loops; other runs are joined end to start into loops (a run
//   may be reversed to fit), so one loop can be drawn as several paths
//   with different regions. Points match within 1e-9 of the drawing's
//   size. A run that cannot be closed, or a point where more than two run
//   ends meet, is an error.
// - Holes and islands come from nesting (CurveGeometry computes depth);
//   fill rules and path directions are ignored.
// - <g> groups are entered; <title>, <desc>, <metadata>, <defs>, <style>
//   and elements of other XML namespaces are ignored. A transform
//   attribute, nested <svg>, basic shapes (<rect>, <circle>, ...) and any
//   other element are errors. width, height and viewBox are ignored:
//   coordinates stay in user units.
// - A document with a DTD is rejected (no entity expansion, no network).
//
// Throws std::invalid_argument for a malformed or unsupported document or
// an invalid geometry, std::runtime_error if the file cannot be read.
geometry::CurveGeometry import_svg(std::istream &in);
geometry::CurveGeometry import_svg(const std::filesystem::path &path);

// Writes the geometry as an SVG document that import_svg reads back to
// the same geometry (up to rounding; elliptical arcs are written in
// pieces of at most a quarter turn, so they come back as several arcs).
// Each run of consecutive curves of one region in a loop is one <path>.
// Its id is the region name when that name is used by one path only and
// is a valid XML id; otherwise the id is "pde2d_run_<k>" and the name is
// in data-region. Numbers are written in the shortest form that reads
// back exactly. Throws std::runtime_error if the output fails.
void export_svg(const geometry::CurveGeometry &geometry, std::ostream &out);
void export_svg(const geometry::CurveGeometry &geometry,
                const std::filesystem::path &path);

// Writes a picture of the mesh, for inspection: the triangles (light fill,
// thin grey edges) and, on top, the boundary edges, one <path> per region
// with its own colour, a data-region attribute and a <title> with the
// region name (shown as a tooltip). Nodes and curve links are not drawn.
// The viewBox is the mesh's bounding box with a small margin. Throws
// std::invalid_argument for a mesh without nodes or with indices out of
// range, std::runtime_error if the output fails.
void export_mesh_svg(const mesh::Mesh &mesh, std::ostream &out);
void export_mesh_svg(const mesh::Mesh &mesh, const std::filesystem::path &path);

} // namespace pde2d::io

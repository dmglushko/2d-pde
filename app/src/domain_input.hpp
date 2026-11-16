#pragma once

#include "options.hpp"

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/mesh/mesh.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

// The "geometry" section of a case: where the domain comes from.
//
//   {"source": "svg", "name": "plate.svg", "svg": "<svg ...>...</svg>",
//    "region_edits": [...]}
//   {"source": "shape", "shape": "rectangle",
//    "options": {"width": 2, "height": 1}, "region_edits": [...]}
//   {"source": "mesh_file", "format": "gmsh" or "pde2d",
//    "name": "plate.msh", "text": "...", "region_edits": [...]}
//
// An SVG or a shape gives a CurveGeometry, which the Mesher meshes. A mesh
// file gives a Mesh directly; the mesh section's refiner and improvers
// then run on it (the Remesher), and its discretizer and triangulator are
// not used. The SVG and mesh texts are kept in the case, so a saved case
// is complete on its own.
//
// Region edits apply in order to a CurveGeometry, each with the
// library's CurveGeometry methods:
//   {"op": "rename", "from": "a", "to": "b"}
//   {"op": "merge", "source": "a", "target": "b"}
//   {"op": "split", "region": "a", "fraction": 0.5,
//    "first": "a1", "second": "a2"}
// A mesh file takes rename and merge (not split: it has no curves); they
// change its region names and the region tags of its boundary edges.
namespace pde2d::app {

// The keys the reader accepts, per source and per region edit (the case
// schema is generated from them).
inline constexpr std::string_view geometry_sources[] = {"shape", "svg",
                                                        "mesh_file"};
inline constexpr std::string_view shape_section_keys[] = {
    "source", "shape", "options", "region_edits"};
inline constexpr std::string_view svg_section_keys[] = {"source", "name", "svg",
                                                        "region_edits"};
inline constexpr std::string_view mesh_file_section_keys[] = {
    "source", "format", "name", "text", "region_edits"};
inline constexpr std::string_view mesh_file_formats[] = {"gmsh", "pde2d"};
inline constexpr std::string_view rename_edit_keys[] = {"op", "from", "to"};
inline constexpr std::string_view merge_edit_keys[] = {"op", "source",
                                                       "target"};
inline constexpr std::string_view split_edit_keys[] = {
    "op", "region", "fraction", "first", "second"};

// Exactly one of the two is set.
struct Domain {
  std::optional<geometry::CurveGeometry> geometry;
  std::optional<mesh::Mesh> mesh;
};

// Throws std::invalid_argument naming the path in the case file
// ("geometry.svg: ...") with the library's message for a document, shape
// or edit it refuses; std::runtime_error from the library passes through.
Domain build_domain(const Json &geometry_section);

// What the page draws: for a CurveGeometry its loops (depth, parent and
// curves, each curve sampled as a polyline with its region and kind),
// regions with their lengths, the bounding box and the domain size; for a
// mesh file its boundary edges by region.
Json domain_json(const Domain &domain);

// What a preset needs to know of a domain: its region names (in the
// order of domain_json), its domain size, and a point well inside it with
// its distance to the boundary (the inside candidate farthest from the
// boundary: a grid over the bounding box for curves, measured on the
// curves sampled as polylines; the triangle centroids for a mesh file).
struct DomainFacts {
  std::vector<std::string> regions;
  double domain_size = 0.0;
  geometry::Point inner{};
  double clearance = 0.0;
};
DomainFacts domain_facts(const Domain &domain);

// The shapes of geometry/shapes.hpp with their parameters.
Json shape_catalog_json();

// A 2 x 1 rectangle.
Json default_geometry_section();

} // namespace pde2d::app

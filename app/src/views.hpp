#pragma once

#include "options.hpp"

#include "pde2d/fem/csr_matrix.hpp"
#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/report.hpp"

#include <cstdint>
#include <span>
#include <string>

// Library results as the JSON the page draws.
namespace pde2d::app {

// {"stage", "time_ms", "messages", "metrics"}.
Json report_json(const mesh::StageReport &report);
Json reports_json(std::span<const mesh::StageReport> reports);

// What the page needs to know of a mesh before its arrays: "id" (the
// session's mesh id), the counts "nodes", "triangles", "boundary_edges",
// "region_names", "has_links", and "arrays": where and how the arrays
// travel ({"path", "format": "little-endian", "bytes", "parts": [{"name",
// "type", "count", "offset"}]}, see mesh_arrays_binary).
Json mesh_summary_json(const mesh::Mesh &mesh, std::uint64_t id);

// The mesh's arrays, one after the other, little-endian: "nodes" (float64
// x0, y0, x1, y1, ...), "triangles" (uint32 a0, b0, c0, ...),
// "boundary_edges" (uint32 a0, b0, a1, b1, ...) and "boundary_regions"
// (uint32, one region tag per boundary edge). Throws std::runtime_error
// for an index that does not fit 32 bits.
std::string mesh_arrays_binary(const mesh::Mesh &mesh);

// Counts and shape measures: nodes, triangles, boundary_edges, area,
// shortest_edge, longest_edge, min_angle_degrees (the smallest corner of
// any triangle) and mean_min_angle_degrees.
Json mesh_stats_json(const mesh::Mesh &mesh);

// A picture of where a matrix has entries: the matrix is cut into
// bins x bins blocks (bins = min(rows, max_bins)) and "counts" holds the
// number of entries of each block, row by row.
Json sparsity_json(const fem::SparsityPattern &pattern, std::size_t max_bins);

} // namespace pde2d::app

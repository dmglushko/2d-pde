#pragma once

#include "options.hpp"

#include "pde2d/fem/csr_matrix.hpp"
#include "pde2d/mesh/mesh.hpp"
#include "pde2d/mesh/report.hpp"

#include <span>

// Library results as the JSON the page draws.
namespace pde2d::app {

// {"stage", "time_ms", "messages", "metrics"}.
Json report_json(const mesh::StageReport &report);
Json reports_json(std::span<const mesh::StageReport> reports);

// Flat arrays, so a large mesh stays small: "nodes" [x0, y0, x1, y1, ...],
// "triangles" [a0, b0, c0, ...], "boundary_edges" [a0, b0, region0, ...],
// "region_names", "has_links".
Json mesh_json(const mesh::Mesh &mesh);

// Counts and shape measures: nodes, triangles, boundary_edges, area,
// shortest_edge, longest_edge, min_angle_degrees (the smallest corner of
// any triangle) and mean_min_angle_degrees.
Json mesh_stats_json(const mesh::Mesh &mesh);

// A picture of where a matrix has entries: the matrix is cut into
// bins x bins blocks (bins = min(rows, max_bins)) and "counts" holds the
// number of entries of each block, row by row.
Json sparsity_json(const fem::SparsityPattern &pattern, std::size_t max_bins);

} // namespace pde2d::app

#pragma once

// Private: what the estimators need of a P1 solution on a mesh, computed
// once per call: each triangle's area, diameter and the constant gradient
// of u_h, and the edges with the triangles on their sides.

#include "pde2d/fem/compiled_expression.hpp"
#include "pde2d/geometry/point.hpp"
#include "pde2d/mesh/mesh.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace pde2d::adapt::detail {

struct TriangleData {
  std::array<geometry::Point, 3> nodes{};
  double area = 0.0;
  double diameter = 0.0;      // the longest edge
  geometry::Point gradient{}; // grad u_h, constant on the triangle
};

// One TriangleData per triangle of the mesh, with u_h from `values` (one
// per node). Throws std::invalid_argument when a triangle's area or
// gradients are not finite positive numbers (as assembly does).
std::vector<TriangleData> triangle_data(const mesh::Mesh &mesh,
                                        std::span<const double> values,
                                        const std::string &who);

// An edge shared by two triangles: `first` is the triangle in which the
// edge goes from a to b counterclockwise (so the outward normal of
// `first` is (tau.y, -tau.x)); `second` is the other one.
struct InteriorEdge {
  std::size_t a = 0, b = 0;
  std::size_t first = 0, second = 0;
};

// The interior edges, by increasing (smaller node, larger node), and for
// each boundary edge of the mesh (indexed like Mesh::boundary_edges) the
// triangle it belongs to.
struct EdgeTopology {
  std::vector<InteriorEdge> interior;
  std::vector<std::size_t> boundary_triangle;
};

EdgeTopology edge_topology(const mesh::Mesh &mesh);

// A coefficient at a point: 0 without a call when it is zero as written;
// a value that is not finite throws std::invalid_argument, naming `who`
// and `label`.
double coefficient_value(const fem::CompiledExpression &coefficient,
                         const std::string &who, const char *label,
                         const geometry::Point &p, double t);

} // namespace pde2d::adapt::detail

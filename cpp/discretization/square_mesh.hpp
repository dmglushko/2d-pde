#pragma once
#include "discretization/geometry_base.hpp"
#include <stdexcept>

// A deterministic unit-square mesh for convergence experiments. Production
// problems may instead use generate_mesh(...).mesh from discretization.
inline discretization::TriangleMesh square_mesh(int subdivisions) {
  if (subdivisions < 1 || subdivisions > 1000)
    throw std::invalid_argument("Square subdivisions must be in [1,1000]");
  discretization::TriangleMesh mesh;
  const auto n = static_cast<std::size_t>(subdivisions);
  const auto id = [n](std::size_t i, std::size_t j) { return j * (n + 1) + i; };
  for (std::size_t j = 0; j <= n; ++j)
    for (std::size_t i = 0; i <= n; ++i)
      mesh.nodes.push_back(
          {static_cast<double>(i) / n, static_cast<double>(j) / n});
  for (std::size_t j = 0; j < n; ++j)
    for (std::size_t i = 0; i < n; ++i) {
      mesh.elements.push_back({id(i, j), id(i + 1, j), id(i + 1, j + 1)});
      mesh.elements.push_back({id(i, j), id(i + 1, j + 1), id(i, j + 1)});
    }
  for (std::size_t i = 0; i < n; ++i) {
    mesh.boundary_edges.push_back({id(i, 0), id(i + 1, 0)});
    mesh.boundary_tags.push_back("bottom");
    mesh.boundary_edges.push_back({id(n, i), id(n, i + 1)});
    mesh.boundary_tags.push_back("right");
    mesh.boundary_edges.push_back({id(n - i, n), id(n - i - 1, n)});
    mesh.boundary_tags.push_back("top");
    mesh.boundary_edges.push_back({id(0, n - i), id(0, n - i - 1)});
    mesh.boundary_tags.push_back("left");
  }
  return mesh;
}

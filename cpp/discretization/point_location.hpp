#pragma once
// Point location in a TriangleMesh: the containing triangle and barycentric
// coordinates, built on the conservative box index of spatial_index.hpp.
//
// Rules:
// * Containment uses exact orientation predicates, so there is no inside
//   tolerance. A point on a shared edge or vertex belongs to the
//   lowest-index triangle that contains it.
// * Barycentric coordinates are computed in floating point relative to
//   mesh.elements[element] in its vertex order; inside a triangle they may
//   leave [0, 1] by rounding only.
// * A point outside the closed mesh is accepted only within
//   PointLocationOptions::max_outside_distance (absolute, Euclidean) of the
//   boundary polygon. It is then assigned to the triangle owning the nearest
//   boundary edge (ties: lowest boundary-edge index) with that triangle's
//   unclamped barycentric coordinates, i.e. linear extrapolation, so affine
//   fields stay exact. Typical use: nodes on a curved boundary that lie just
//   outside the chords of a coarser mesh. Beyond the distance, locate()
//   returns std::nullopt. The default distance 0 accepts only the closed
//   mesh.
#include "mesh_validation.hpp"
#include "spatial_index.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <vector>

namespace pde2d::discretization {
struct PointLocationOptions {
  double max_outside_distance = 0.0;
};
struct PointLocation {
  std::size_t element = 0;
  std::array<double, 3> barycentric{};
  // False when the point lies outside the mesh and was extrapolated from
  // the triangle owning the nearest boundary edge.
  bool inside = true;
  // Distance to the nearest boundary edge for outside points, 0 inside.
  double distance = 0.0;
};

namespace location_detail {
inline double segment_distance(Point2 p, Point2 a, Point2 b) {
  const double dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
  double u = len2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0;
  u = std::clamp(u, 0.0, 1.0);
  return std::hypot(p.x - (a.x + u * dx), p.y - (a.y + u * dy));
}
inline double down(double v) {
  return std::nextafter(v, -std::numeric_limits<double>::infinity());
}
inline double up(double v) {
  return std::nextafter(v, std::numeric_limits<double>::infinity());
}
} // namespace location_detail

class PointLocator {
public:
  explicit PointLocator(TriangleMesh mesh, PointLocationOptions options = {})
      : mesh_(std::move(mesh)), options_(options),
        elements_(element_boxes(mesh_)), boundary_(boundary_boxes(mesh_)) {
    if (!std::isfinite(options_.max_outside_distance) ||
        options_.max_outside_distance < 0)
      throw std::invalid_argument(
          "Point location outside distance must be finite and nonnegative");
    std::map<mesh_detail::Edge, std::size_t> owner;
    for (std::size_t i = 0; i < mesh_.elements.size(); ++i)
      for (int e = 0; e < 3; ++e)
        owner.emplace(mesh_detail::edge_key(mesh_.elements[i][e],
                                            mesh_.elements[i][(e + 1) % 3]),
                      i);
    for (auto [a, b] : mesh_.boundary_edges)
      boundary_owner_.push_back(owner.at(mesh_detail::edge_key(a, b)));
  }
  const TriangleMesh &mesh() const { return mesh_; }
  const PointLocationOptions &options() const { return options_; }

  // Barycentric coordinates of p relative to an element (unclamped).
  std::array<double, 3> barycentric(std::size_t element, Point2 p) const {
    const auto &t = mesh_.elements.at(element);
    const auto a = mesh_.nodes[t[0]], b = mesh_.nodes[t[1]],
               c = mesh_.nodes[t[2]];
    const double det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    const double s = ((p.x - a.x) * (c.y - a.y) - (c.x - a.x) * (p.y - a.y)) /
                     det,
                 r = ((b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y)) /
                     det;
    return {1 - s - r, s, r};
  }

  std::optional<PointLocation> locate(Point2 p) const {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
      throw std::invalid_argument("Point location needs a finite point");
    for (auto i : elements_.candidates(mesh_detail::Box{p.x, p.x, p.y, p.y})) {
      const auto &t = mesh_.elements[i];
      const auto a = mesh_.nodes[t[0]], b = mesh_.nodes[t[1]],
                 c = mesh_.nodes[t[2]];
      if (predicates::orient(a, b, p) >= 0 &&
          predicates::orient(b, c, p) >= 0 && predicates::orient(c, a, p) >= 0)
        return PointLocation{i, barycentric(i, p), true, 0.0};
    }
    const double tol = options_.max_outside_distance;
    if (!(tol > 0))
      return std::nullopt;
    using location_detail::down, location_detail::up;
    mesh_detail::Box near{down(p.x - tol), up(p.x + tol), down(p.y - tol),
                          up(p.y + tol)};
    std::optional<std::size_t> best;
    double best_distance = 0;
    for (auto j : boundary_.candidates(near)) {
      auto [a, b] = mesh_.boundary_edges[j];
      double d =
          location_detail::segment_distance(p, mesh_.nodes[a], mesh_.nodes[b]);
      if (!best || d < best_distance) {
        best = j;
        best_distance = d;
      }
    }
    if (!best || best_distance > tol)
      return std::nullopt;
    auto element = boundary_owner_[*best];
    return PointLocation{element, barycentric(element, p), false,
                         best_distance};
  }

private:
  static std::vector<mesh_detail::Box> element_boxes(const TriangleMesh &m) {
    validate_mesh_geometry(m);
    std::vector<mesh_detail::Box> boxes;
    boxes.reserve(m.elements.size());
    for (const auto &t : m.elements) {
      auto box = mesh_detail::Box::segment(m.nodes[t[0]], m.nodes[t[1]]);
      auto c = m.nodes[t[2]];
      boxes.push_back({std::min(box.xmin, c.x), std::max(box.xmax, c.x),
                       std::min(box.ymin, c.y), std::max(box.ymax, c.y)});
    }
    return boxes;
  }
  static std::vector<mesh_detail::Box> boundary_boxes(const TriangleMesh &m) {
    std::vector<mesh_detail::Box> boxes;
    for (auto [a, b] : m.boundary_edges)
      boxes.push_back(mesh_detail::Box::segment(m.nodes[a], m.nodes[b]));
    return boxes;
  }
  TriangleMesh mesh_;
  PointLocationOptions options_;
  mesh_detail::BoxIndex elements_, boundary_;
  std::vector<std::size_t> boundary_owner_;
};

inline std::optional<PointLocation>
locate_point(const TriangleMesh &mesh, Point2 p,
             PointLocationOptions options = {}) {
  return PointLocator(mesh, options).locate(p);
}
} // namespace pde2d::discretization

#include "point_location.hpp"

#include "pde2d/geometry/predicates.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pde2d::adapt::detail {

namespace {

using geometry::Point;

// Twice the signed area of (a, b, p), in floating point (the sign is
// decided exactly by orient; this gives the size).
double area2(Point a, Point b, Point p) {
  return (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
}

// The weights of p on the segment u-v (the point of the segment nearest
// to p), and the squared distance to it.
struct SegmentPoint {
  NodeWeights weights;
  double distance2 = 0.0;
};

SegmentPoint on_segment(const mesh::Mesh &mesh, std::size_t u, std::size_t v,
                        Point p) {
  const Point a = mesh.nodes[u], b = mesh.nodes[v];
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double s = ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy);
  SegmentPoint out;
  Point q;
  if (!(s > 0.0)) {
    out.weights = {.nodes = {u, 0, 0}, .weights = {1.0, 0.0, 0.0}};
    q = a;
  } else if (!(s < 1.0)) {
    out.weights = {.nodes = {v, 0, 0}, .weights = {1.0, 0.0, 0.0}};
    q = b;
  } else {
    out.weights = {.nodes = {u, v, 0}, .weights = {1.0 - s, s, 0.0}};
    q = {a.x + s * dx, a.y + s * dy};
  }
  out.distance2 = (p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y);
  return out;
}

} // namespace

std::vector<std::size_t> boundary_loops(const mesh::Mesh &mesh) {
  // Union-find over the nodes: the two ends of a boundary edge are in one
  // loop.
  std::vector<std::size_t> parent(mesh.nodes.size());
  for (std::size_t v = 0; v < parent.size(); ++v)
    parent[v] = v;
  const auto root = [&](std::size_t v) {
    while (parent[v] != v)
      v = parent[v] = parent[parent[v]];
    return v;
  };
  for (const auto &edge : mesh.boundary_edges) {
    const std::size_t a = root(edge.nodes[0]), b = root(edge.nodes[1]);
    if (a != b)
      parent[std::max(a, b)] = std::min(a, b);
  }
  constexpr auto none = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> number(mesh.nodes.size(), none);
  std::vector<std::size_t> loops(mesh.boundary_edges.size());
  std::size_t count = 0;
  for (std::size_t e = 0; e < loops.size(); ++e) {
    std::size_t &n = number[root(mesh.boundary_edges[e].nodes[0])];
    if (n == none)
      n = count++;
    loops[e] = n;
  }
  return loops;
}

PointLocator::PointLocator(const mesh::Mesh &mesh) : mesh_(mesh) {
  edge_loops_ = boundary_loops(mesh);
  for (const std::size_t loop : edge_loops_)
    loops_ = std::max(loops_, loop + 1);
  box_ = geometry::bounding_box(mesh.nodes);
  const double w = geometry::width(box_), h = geometry::height(box_);
  const std::size_t n = std::max<std::size_t>(mesh.triangles.size(), 1);
  // About one triangle per cell; a thin domain gets at most n cells along
  // its long side.
  const double side = w > 0.0 && h > 0.0
                          ? std::sqrt(w * h / static_cast<double>(n))
                          : std::max(w, h) / static_cast<double>(n);
  const auto cells = [&](double length) {
    if (!(side > 0.0) || !(length > 0.0))
      return std::size_t{1};
    const double k = std::ceil(length / side);
    return static_cast<std::size_t>(std::clamp(k, 1.0, static_cast<double>(n)));
  };
  nx_ = cells(w);
  ny_ = cells(h);
  cell_width_ = w > 0.0 ? w / static_cast<double>(nx_) : 1.0;
  cell_height_ = h > 0.0 ? h / static_cast<double>(ny_) : 1.0;
  triangles_.resize(nx_ * ny_);
  edges_.resize(nx_ * ny_);

  const auto insert = [&](std::vector<std::vector<std::size_t>> &buckets,
                          std::size_t item, Point lo, Point hi) {
    const auto first = cell_of(lo), last = cell_of(hi);
    for (std::size_t j = first[1]; j <= last[1]; ++j)
      for (std::size_t i = first[0]; i <= last[0]; ++i)
        buckets[index(i, j)].push_back(item);
  };
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    const auto &v = mesh.triangles[t].nodes;
    const Point a = mesh.nodes[v[0]], b = mesh.nodes[v[1]],
                c = mesh.nodes[v[2]];
    insert(triangles_, t,
           {std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y})},
           {std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y})});
  }
  for (std::size_t e = 0; e < mesh.boundary_edges.size(); ++e) {
    const auto &v = mesh.boundary_edges[e].nodes;
    const Point a = mesh.nodes[v[0]], b = mesh.nodes[v[1]];
    insert(edges_, e, {std::min(a.x, b.x), std::min(a.y, b.y)},
           {std::max(a.x, b.x), std::max(a.y, b.y)});
  }
}

std::array<std::size_t, 2> PointLocator::cell_of(Point p) const {
  // Monotone in each coordinate, so a point of a box lies in a cell
  // between the cells of the box's corners. Points outside the grid go to
  // its nearest cell.
  const auto clamp_cell = [](double position, std::size_t count) {
    const double k = std::floor(position);
    return static_cast<std::size_t>(std::clamp(std::isfinite(k) ? k : 0.0, 0.0,
                                               static_cast<double>(count - 1)));
  };
  return {clamp_cell((p.x - box_.min.x) / cell_width_, nx_),
          clamp_cell((p.y - box_.min.y) / cell_height_, ny_)};
}

std::optional<NodeWeights> PointLocator::in_triangle(Point p) const {
  const auto cell = cell_of(p);
  for (const std::size_t t : triangles_[index(cell[0], cell[1])]) {
    const auto &v = mesh_.triangles[t].nodes;
    const Point x[3] = {mesh_.nodes[v[0]], mesh_.nodes[v[1]],
                        mesh_.nodes[v[2]]};
    // side[i]: p against the edge opposite vertex i (counterclockwise
    // triangles: inside is positive).
    int side[3];
    bool inside = true;
    for (std::size_t i = 0; i < 3 && inside; ++i) {
      side[i] = geometry::orient(x[(i + 1) % 3], x[(i + 2) % 3], p);
      inside = side[i] >= 0;
    }
    if (!inside)
      continue;
    for (std::size_t i = 0; i < 3; ++i)
      if (x[i] == p)
        return NodeWeights{.nodes = {v[i], 0, 0}, .weights = {1.0, 0.0, 0.0}};
    for (std::size_t i = 0; i < 3; ++i)
      if (side[i] == 0) // on the edge opposite vertex i
        return on_segment(mesh_, v[(i + 1) % 3], v[(i + 2) % 3], p).weights;
    // Strictly inside: barycentric weights from the sub-areas; rounding may
    // make a tiny one negative, which is 0.
    double lambda[3];
    double sum = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
      lambda[i] = std::max(0.0, area2(x[(i + 1) % 3], x[(i + 2) % 3], p));
      sum += lambda[i];
    }
    return NodeWeights{
        .nodes = v,
        .weights = {lambda[0] / sum, lambda[1] / sum, lambda[2] / sum}};
  }
  return std::nullopt;
}

BoundaryPoint
PointLocator::nearest_on_boundary(Point p,
                                  std::optional<std::size_t> loop) const {
  const auto cell = cell_of(p);
  const auto ci = static_cast<std::ptrdiff_t>(cell[0]),
             cj = static_cast<std::ptrdiff_t>(cell[1]);
  const auto nx = static_cast<std::ptrdiff_t>(nx_),
             ny = static_cast<std::ptrdiff_t>(ny_);
  const double min_side = std::min(cell_width_, cell_height_);
  double best = std::numeric_limits<double>::infinity();
  std::size_t best_edge = 0;
  NodeWeights best_weights{};
  const auto visit = [&](std::ptrdiff_t i, std::ptrdiff_t j) {
    if (i < 0 || j < 0 || i >= nx || j >= ny)
      return;
    for (const std::size_t e : edges_[index(static_cast<std::size_t>(i),
                                            static_cast<std::size_t>(j))]) {
      if (loop && edge_loops_[e] != *loop)
        continue;
      const auto &v = mesh_.boundary_edges[e].nodes;
      const SegmentPoint q = on_segment(mesh_, v[0], v[1], p);
      if (q.distance2 < best || (q.distance2 == best && e < best_edge)) {
        best = q.distance2;
        best_edge = e;
        best_weights = q.weights;
      }
    }
  };
  const std::ptrdiff_t rings = std::max(nx, ny);
  for (std::ptrdiff_t r = 0; r <= rings; ++r) {
    // The cells at Chebyshev distance r from p's cell.
    if (r == 0) {
      visit(ci, cj);
    } else {
      for (std::ptrdiff_t i = ci - r; i <= ci + r; ++i) {
        visit(i, cj - r);
        visit(i, cj + r);
      }
      for (std::ptrdiff_t j = cj - r + 1; j <= cj + r - 1; ++j) {
        visit(ci - r, j);
        visit(ci + r, j);
      }
    }
    // Every cell further out is at least r cells away from p's cell, so
    // its points are at least r * min_side from p: strictly farther than
    // the best found, which no later edge can then beat or tie.
    const double reach = static_cast<double>(r) * min_side;
    if (best < reach * reach)
      break;
  }
  return {.weights = best_weights, .edge = best_edge, .distance2 = best};
}

} // namespace pde2d::adapt::detail

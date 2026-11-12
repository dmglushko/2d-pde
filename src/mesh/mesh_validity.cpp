#include "mesh_validity.hpp"

#include "pde2d/geometry/predicates.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace pde2d::mesh::detail {

using geometry::orient;
using geometry::Point;

namespace {

std::string text(std::size_t i) { return std::to_string(i); }

int sign(double x) { return (x > 0) - (x < 0); }

// q, a, r collinear, a and r both different from q: true if they lie on
// the same side of q. Exact (coordinate comparisons only).
bool same_direction(Point q, Point a, Point r) {
  return sign(a.x - q.x) == sign(r.x - q.x) &&
         sign(a.y - q.y) == sign(r.y - q.y);
}

// The triangle (q, a, b) is counterclockwise, so its angle at q is the
// sector from the direction of a counterclockwise to that of b, less than
// half a turn. True if the direction of r lies in it, the direction of a
// included and that of b excluded: a closed fan of sectors then contains
// every direction equally often.
bool sector_contains(Point q, Point a, Point b, Point r) {
  int side = orient(q, a, r);
  if (side == 0)
    return same_direction(q, a, r);
  return side > 0 && orient(q, r, b) > 0;
}

// 1. No folds. The triangles around a node turn around it k times; k is
// the number of sectors that contain one direction. An interior node
// (closed fans) needs k = 1. A boundary node's fan starts at its outgoing
// boundary edge; taking that edge's direction, k = 1 if the fan turns
// less than once and no other triangles wind around the node.
void check_folds(const std::vector<Point> &nodes,
                 const std::vector<std::array<std::size_t, 3>> &triangles,
                 const std::vector<WorkingBoundaryEdge> &boundary_edges,
                 const std::vector<std::size_t> &outgoing_boundary) {
  const std::size_t n = nodes.size();
  std::vector<std::size_t> reference(n, no_index);
  for (std::size_t v = 0; v < n; ++v)
    if (outgoing_boundary[v] != no_index)
      reference[v] = boundary_edges[outgoing_boundary[v]].nodes[1];
  for (const auto &t : triangles)
    for (std::size_t i = 0; i < 3; ++i)
      if (reference[t[i]] == no_index)
        reference[t[i]] = t[(i + 1) % 3];
  std::vector<std::size_t> turns(n, 0);
  for (const auto &t : triangles)
    for (std::size_t i = 0; i < 3; ++i) {
      std::size_t q = t[i];
      if (sector_contains(nodes[q], nodes[t[(i + 1) % 3]],
                          nodes[t[(i + 2) % 3]], nodes[reference[q]]))
        ++turns[q];
    }
  // The reference node is a vertex of a strictly counterclockwise
  // triangle with v, so it is never at the same place as v.
  for (std::size_t v = 0; v < n; ++v) {
    if (turns[v] == 1)
      continue;
    const bool boundary = outgoing_boundary[v] != no_index;
    throw std::invalid_argument(
        std::string("The triangles around ") +
        (boundary ? "boundary node " : "node ") + text(v) +
        " overlap: they turn around it " +
        (boundary ? "more than once" : text(turns[v]) + " times") +
        ", so the mesh folds over itself there");
  }
}

// A uniform grid over the nodes' bounding box, about one cell per
// triangle. Items (triangles, boundary edges) are listed in every cell
// their bounding box touches, in index order.
class Grid {
public:
  Grid(const std::vector<Point> &nodes, std::size_t cells) {
    xmin_ = xmax_ = nodes[0].x;
    ymin_ = ymax_ = nodes[0].y;
    for (Point p : nodes) {
      xmin_ = std::min(xmin_, p.x);
      xmax_ = std::max(xmax_, p.x);
      ymin_ = std::min(ymin_, p.y);
      ymax_ = std::max(ymax_, p.y);
    }
    // Halves, so the extent cannot overflow for coordinates near the
    // largest double. They are positive (there is a nondegenerate
    // triangle) unless halving rounds subnormal coordinates together; one
    // cell then holds everything, which is slow but still correct.
    half_width_ = xmax_ / 2 - xmin_ / 2;
    half_height_ = ymax_ / 2 - ymin_ / 2;
    if (!(half_width_ > 0) || !(half_height_ > 0))
      return;
    const double c = static_cast<double>(std::max<std::size_t>(cells, 1));
    const double ratio = std::clamp(half_width_ / half_height_, 1 / c, c);
    nx_ = static_cast<std::size_t>(std::clamp(std::ceil(std::sqrt(c * ratio)),
                                              1.0, c));
    ny_ = static_cast<std::size_t>(
        std::clamp(std::ceil(c / static_cast<double>(nx_)), 1.0, c));
  }

  // The cell column / row of a coordinate. Monotone in x (every step is
  // a monotone rounded operation), so a point inside a bounding box falls
  // in a cell of the box's range.
  std::size_t column(double x) const {
    return index(x / 2 - xmin_ / 2, half_width_, nx_);
  }
  std::size_t row(double y) const {
    return index(y / 2 - ymin_ / 2, half_height_, ny_);
  }
  std::size_t cell(Point p) const { return row(p.y) * nx_ + column(p.x); }

  // Lists item k (k = 0, 1, ...) in the cells of its box; `box(k, lo, hi)`
  // gives the box's corners. Builds compressed lists (offsets_, items_).
  template <class Box> void fill(std::size_t count, Box box) {
    offsets_.assign(nx_ * ny_ + 1, 0);
    auto each_cell = [&](std::size_t k, auto f) {
      Point lo, hi;
      box(k, lo, hi);
      for (std::size_t r = row(lo.y); r <= row(hi.y); ++r)
        for (std::size_t c = column(lo.x); c <= column(hi.x); ++c)
          f(r * nx_ + c);
    };
    for (std::size_t k = 0; k < count; ++k)
      each_cell(k, [&](std::size_t c) { ++offsets_[c + 1]; });
    for (std::size_t c = 0; c < nx_ * ny_; ++c)
      offsets_[c + 1] += offsets_[c];
    items_.assign(offsets_.back(), 0);
    std::vector<std::size_t> at(offsets_.begin(), offsets_.end() - 1);
    for (std::size_t k = 0; k < count; ++k)
      each_cell(k, [&](std::size_t c) { items_[at[c]++] = k; });
  }

  std::size_t cells() const { return nx_ * ny_; }
  // The items listed in cell c, in index order.
  const std::size_t *begin(std::size_t c) const {
    return items_.data() + offsets_[c];
  }
  const std::size_t *end(std::size_t c) const {
    return items_.data() + offsets_[c + 1];
  }

private:
  static std::size_t index(double offset, double extent, std::size_t n) {
    if (n == 1)
      return 0;
    double t = std::floor(offset / extent * static_cast<double>(n));
    if (!(t > 0)) // also NaN
      return 0;
    return static_cast<std::size_t>(std::min(t, static_cast<double>(n - 1)));
  }

  double xmin_, xmax_, ymin_, ymax_, half_width_ = 0, half_height_ = 0;
  std::size_t nx_ = 1, ny_ = 1;
  std::vector<std::size_t> offsets_, items_;
};

std::string edge_text(const std::vector<WorkingBoundaryEdge> &edges,
                      std::size_t k) {
  return text(k) + " (" + text(edges[k].nodes[0]) + " -> " +
         text(edges[k].nodes[1]) + ")";
}

// 2. Boundary edges that cross or touch. Each node has one incoming and one
// outgoing boundary edge, so two edges sharing a node are consecutive in
// their loop; they may only meet at that node.
void check_boundary_edges(const std::vector<Point> &nodes,
                          const std::vector<WorkingBoundaryEdge> &edges,
                          Grid &grid) {
  grid.fill(edges.size(), [&](std::size_t k, Point &lo, Point &hi) {
    Point a = nodes[edges[k].nodes[0]], b = nodes[edges[k].nodes[1]];
    lo = {std::min(a.x, b.x), std::min(a.y, b.y)};
    hi = {std::max(a.x, b.x), std::max(a.y, b.y)};
  });
  for (std::size_t c = 0; c < grid.cells(); ++c)
    for (const std::size_t *i = grid.begin(c); i != grid.end(c); ++i)
      for (const std::size_t *j = i + 1; j != grid.end(c); ++j) {
        auto [a, b] = edges[*i].nodes;
        auto [p, q] = edges[*j].nodes;
        Point A = nodes[a], B = nodes[b], P = nodes[p], Q = nodes[q];
        bool meet;
        if (b == p) // a -> b -> q
          meet = geometry::on_segment(A, B, Q) || geometry::on_segment(P, Q, A);
        else if (q == a) // p -> a -> b
          meet = geometry::on_segment(A, B, P) || geometry::on_segment(P, Q, B);
        else
          meet = geometry::segments_intersect(A, B, P, Q);
        if (meet)
          throw std::invalid_argument(
              "Boundary edges " + edge_text(edges, *i) + " and " +
              edge_text(edges, *j) +
              " cross or touch, so the mesh overlaps itself or its "
              "boundary loops touch");
      }
}

// 3. A node in a triangle that does not use it: two pieces overlap (one
// lies inside the other, or they cross).
void check_nodes_outside_other_triangles(
    const std::vector<Point> &nodes,
    const std::vector<std::array<std::size_t, 3>> &triangles, Grid &grid) {
  grid.fill(triangles.size(), [&](std::size_t k, Point &lo, Point &hi) {
    const auto &t = triangles[k];
    Point a = nodes[t[0]], b = nodes[t[1]], c = nodes[t[2]];
    lo = {std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y})};
    hi = {std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y})};
  });
  for (std::size_t v = 0; v < nodes.size(); ++v) {
    const Point p = nodes[v];
    const std::size_t c = grid.cell(p);
    for (const std::size_t *k = grid.begin(c); k != grid.end(c); ++k) {
      const auto &t = triangles[*k];
      if (t[0] == v || t[1] == v || t[2] == v)
        continue;
      Point a = nodes[t[0]], b = nodes[t[1]], d = nodes[t[2]];
      if (orient(a, b, p) >= 0 && orient(b, d, p) >= 0 && orient(d, a, p) >= 0)
        throw std::invalid_argument(
            "Node " + text(v) + " lies in triangle " + text(*k) +
            " but is not one of its nodes, so the mesh overlaps itself (for "
            "example one piece lies inside another)");
    }
  }
}

} // namespace

void check_geometric_validity(
    const std::vector<Point> &nodes,
    const std::vector<std::array<std::size_t, 3>> &triangles,
    const std::vector<WorkingBoundaryEdge> &boundary_edges,
    const std::vector<std::size_t> &outgoing_boundary) {
  check_folds(nodes, triangles, boundary_edges, outgoing_boundary);
  Grid grid(nodes, triangles.size());
  check_boundary_edges(nodes, boundary_edges, grid);
  check_nodes_outside_other_triangles(nodes, triangles, grid);
}

} // namespace pde2d::mesh::detail

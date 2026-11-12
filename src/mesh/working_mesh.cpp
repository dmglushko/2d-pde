#include "pde2d/mesh/working_mesh.hpp"

#include "boundary_split_policy.hpp"

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/geometry/predicates.hpp"

#include <algorithm>
#include <stdexcept>
#include <tuple>

namespace pde2d::mesh {

using geometry::orient;
using geometry::Point;

namespace {

std::size_t next(std::size_t i) { return (i + 1) % 3; }
std::size_t prev(std::size_t i) { return (i + 2) % 3; }

std::string edge_text(std::size_t a, std::size_t b) {
  return "(" + std::to_string(a) + ", " + std::to_string(b) + ")";
}

} // namespace

WorkingMesh::WorkingMesh(std::vector<Point> nodes,
                         std::vector<TriangleNodes> triangles,
                         std::vector<WorkingBoundaryEdge> boundary_edges,
                         std::vector<std::string> region_names,
                         std::vector<CurveChain> chains)
    : nodes_(std::move(nodes)), triangles_(std::move(triangles)),
      boundary_edges_(std::move(boundary_edges)),
      region_names_(std::move(region_names)), chains_(std::move(chains)) {
  const std::size_t n = nodes_.size();
  if (triangles_.empty())
    throw std::invalid_argument("Mesh has no triangles");
  for (std::size_t i = 0; i < n; ++i)
    if (!geometry::is_finite(nodes_[i]))
      throw std::invalid_argument("Node " + std::to_string(i) +
                                  " is not finite");
  for (const auto &name : region_names_)
    if (!geometry::is_valid_region_name(name))
      throw std::invalid_argument("Invalid region name '" + name + "'");

  // Triangles: indices and orientation.
  std::vector<bool> used(n, false);
  for (std::size_t t = 0; t < triangles_.size(); ++t) {
    const auto &v = triangles_[t];
    for (auto i : v)
      if (i >= n)
        throw std::invalid_argument("Triangle " + std::to_string(t) +
                                    " uses a node index out of range");
    if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2])
      throw std::invalid_argument("Triangle " + std::to_string(t) +
                                  " repeats a node");
    if (orient(nodes_[v[0]], nodes_[v[1]], nodes_[v[2]]) <= 0)
      throw std::invalid_argument("Triangle " + std::to_string(t) +
                                  " is not counterclockwise (or degenerate)");
    for (auto i : v)
      used[i] = true;
  }
  for (std::size_t i = 0; i < n; ++i)
    if (!used[i])
      throw std::invalid_argument("Node " + std::to_string(i) +
                                  " is not used by any triangle");

  // Pair half-edges: sort by undirected key.
  struct HalfEdge {
    std::size_t lo, hi, from, triangle;
    int edge;
  };
  std::vector<HalfEdge> half;
  half.reserve(3 * triangles_.size());
  for (std::size_t t = 0; t < triangles_.size(); ++t)
    for (std::size_t i = 0; i < 3; ++i) {
      std::size_t a = triangles_[t][next(i)], b = triangles_[t][prev(i)];
      half.push_back(
          {std::min(a, b), std::max(a, b), a, t, static_cast<int>(i)});
    }
  std::sort(half.begin(), half.end(), [](const HalfEdge &x, const HalfEdge &y) {
    return std::tie(x.lo, x.hi, x.triangle, x.edge) <
           std::tie(y.lo, y.hi, y.triangle, y.edge);
  });
  neighbours_.assign(triangles_.size(), {no_index, no_index, no_index});
  // Directed boundary half-edges (from, to) -> (triangle, edge).
  std::vector<std::tuple<std::size_t, std::size_t, std::size_t, int>> open;
  for (std::size_t k = 0; k < half.size();) {
    std::size_t m = k;
    while (m < half.size() && half[m].lo == half[k].lo &&
           half[m].hi == half[k].hi)
      ++m;
    const auto &e = half[k];
    if (m - k > 2)
      throw std::invalid_argument("Edge " + edge_text(e.lo, e.hi) +
                                  " is shared by more than two triangles");
    if (m - k == 2) {
      const auto &f = half[k + 1];
      if (e.from == f.from)
        throw std::invalid_argument("Triangles " + std::to_string(e.triangle) +
                                    " and " + std::to_string(f.triangle) +
                                    " have inconsistent orientation");
      neighbours_[e.triangle][static_cast<std::size_t>(e.edge)] = f.triangle;
      neighbours_[f.triangle][static_cast<std::size_t>(f.edge)] = e.triangle;
    } else {
      std::size_t to = e.from == e.lo ? e.hi : e.lo;
      open.emplace_back(e.from, to, e.triangle, e.edge);
    }
    k = m;
  }

  // Boundary edges must be exactly the open half-edges.
  std::sort(open.begin(), open.end());
  std::vector<bool> matched(open.size(), false);
  outgoing_boundary_.assign(n, no_index);
  for (std::size_t k = 0; k < boundary_edges_.size(); ++k) {
    const auto &be = boundary_edges_[k];
    auto [a, b] = be.nodes;
    if (a >= n || b >= n)
      throw std::invalid_argument("Boundary edge " + std::to_string(k) +
                                  " uses a node index out of range");
    if (be.region >= region_names_.size())
      throw std::invalid_argument("Boundary edge " + std::to_string(k) +
                                  " has a region id out of range");
    if (be.link && !is_valid_link(*be.link, chains_))
      throw std::invalid_argument("Boundary edge " + std::to_string(k) +
                                  " has an invalid curve link");
    auto it = std::lower_bound(open.begin(), open.end(),
                               std::make_tuple(a, b, std::size_t{0}, -1));
    if (it == open.end() || std::get<0>(*it) != a || std::get<1>(*it) != b)
      throw std::invalid_argument(
          "Boundary edge " + std::to_string(k) + " " + edge_text(a, b) +
          " is not a triangle edge with the domain on its left");
    auto index = static_cast<std::size_t>(it - open.begin());
    if (matched[index])
      throw std::invalid_argument("Boundary edge " + edge_text(a, b) +
                                  " is listed twice");
    matched[index] = true;
    if (outgoing_boundary_[a] != no_index)
      throw std::invalid_argument("Boundary loops touch at node " +
                                  std::to_string(a));
    outgoing_boundary_[a] = k;
  }
  for (std::size_t i = 0; i < open.size(); ++i)
    if (!matched[i])
      throw std::invalid_argument(
          "Triangle edge " +
          edge_text(std::get<0>(open[i]), std::get<1>(open[i])) +
          " has no neighbour but is not a boundary edge");

  node_triangle_.assign(n, no_index);
  for (std::size_t t = 0; t < triangles_.size(); ++t)
    set_node_triangles(t);
  unsplittable_.assign(boundary_edges_.size(), false);
}

WorkingMesh WorkingMesh::from_mesh(const Mesh &mesh) {
  std::vector<TriangleNodes> triangles;
  triangles.reserve(mesh.triangles.size());
  for (const auto &t : mesh.triangles)
    triangles.push_back(t.nodes);
  std::vector<WorkingBoundaryEdge> edges;
  edges.reserve(mesh.boundary_edges.size());
  for (const auto &e : mesh.boundary_edges)
    edges.push_back({e.nodes, e.region, e.link});
  return WorkingMesh(mesh.nodes, std::move(triangles), std::move(edges),
                     mesh.region_names, mesh.chains);
}

void WorkingMesh::set_node_triangles(std::size_t t) {
  for (auto v : triangles_[t])
    node_triangle_[v] = t;
}

void WorkingMesh::replace_neighbour(std::size_t triangle,
                                    std::size_t old_neighbour,
                                    std::size_t new_neighbour) {
  if (triangle == no_index)
    return;
  for (auto &n : neighbours_[triangle])
    if (n == old_neighbour) {
      n = new_neighbour;
      return;
    }
}

int WorkingMesh::local_index(std::size_t triangle, std::size_t node) const {
  for (int i = 0; i < 3; ++i)
    if (triangles_[triangle][static_cast<std::size_t>(i)] == node)
      return i;
  return -1;
}

std::size_t WorkingMesh::boundary_edge_index(std::size_t triangle,
                                             int edge) const {
  auto i = static_cast<std::size_t>(edge);
  if (neighbours_[triangle][i] != no_index)
    return no_index;
  return outgoing_boundary_[triangles_[triangle][next(i)]];
}

std::pair<std::size_t, int>
WorkingMesh::boundary_edge_location(std::size_t k) const {
  auto [a, b] = boundary_edges_.at(k).nodes;
  return find_edge(a, b);
}

std::pair<std::size_t, int> WorkingMesh::find_edge(std::size_t a,
                                                   std::size_t b) const {
  // Rotate around a in both directions from a known triangle.
  const std::size_t start = node_triangle_.at(a);
  if (start == no_index) // a deleted node
    return {no_index, -1};
  for (int direction = 0; direction < 2; ++direction) {
    std::size_t t = start;
    do {
      auto k = static_cast<std::size_t>(local_index(t, a));
      if (triangles_[t][next(k)] == b)
        return {t, static_cast<int>(prev(k))};
      if (triangles_[t][prev(k)] == b) {
        // t has b -> a; the triangle with a -> b is across that edge.
        std::size_t u = neighbours_[t][next(k)];
        if (u == no_index)
          return {no_index, -1};
        auto ka = static_cast<std::size_t>(local_index(u, a));
        return {u, static_cast<int>(prev(ka))};
      }
      // direction 0: cross edge (a, next) clockwise; 1: edge (prev, a).
      t = direction == 0 ? neighbours_[t][prev(k)] : neighbours_[t][next(k)];
    } while (t != no_index && t != start);
    if (t == start)
      break;
  }
  return {no_index, -1};
}

std::vector<std::size_t> WorkingMesh::triangles_around(std::size_t node) const {
  std::vector<std::size_t> result;
  const std::size_t start = node_triangle_.at(node);
  if (start == no_index) // a deleted node
    return result;
  std::size_t t = start;
  do {
    result.push_back(t);
    auto k = static_cast<std::size_t>(local_index(t, node));
    t = neighbours_[t][prev(k)];
  } while (t != no_index && t != start);
  if (t == start)
    return result;
  t = start;
  while (true) {
    auto k = static_cast<std::size_t>(local_index(t, node));
    t = neighbours_[t][next(k)];
    if (t == no_index)
      break;
    result.push_back(t);
  }
  return result;
}

Location WorkingMesh::locate(Point p, std::size_t start) const {
  std::size_t t = start, from = no_index;
  const std::size_t limit = 4 * triangles_.size() + 16;
  for (std::size_t step = 0; step < limit; ++step) {
    const auto &v = triangles_[t];
    int sides[3];
    for (std::size_t i = 0; i < 3; ++i)
      sides[i] = orient(nodes_[v[next(i)]], nodes_[v[prev(i)]], p);
    // Among the edges that p is beyond, prefer an interior edge that does
    // not lead back, then any interior edge; only if every such edge is a
    // boundary edge is the walk blocked.
    int forward = -1, back = -1, boundary = -1;
    for (int i = 0; i < 3; ++i) {
      if (sides[i] >= 0)
        continue;
      std::size_t n = neighbours_[t][static_cast<std::size_t>(i)];
      if (n == no_index) {
        if (boundary < 0)
          boundary = i;
      } else if (n != from) {
        if (forward < 0)
          forward = i;
      } else if (back < 0) {
        back = i;
      }
    }
    int chosen = forward >= 0 ? forward : back;
    if (chosen >= 0) {
      from = t;
      t = neighbours_[t][static_cast<std::size_t>(chosen)];
      continue;
    }
    if (boundary >= 0)
      return {Location::Kind::blocked, t, boundary};
    for (std::size_t i = 0; i < 3; ++i)
      if (nodes_[v[i]] == p)
        return {Location::Kind::on_node, t, static_cast<int>(i)};
    for (std::size_t i = 0; i < 3; ++i)
      if (sides[i] == 0)
        return {Location::Kind::on_edge, t, static_cast<int>(i)};
    return {Location::Kind::inside, t, -1};
  }
  throw std::logic_error("Point location walk did not terminate");
}

bool WorkingMesh::is_flippable(std::size_t t, int edge) const {
  auto i = static_cast<std::size_t>(edge);
  std::size_t u = neighbours_[t][i];
  if (u == no_index)
    return false;
  std::size_t a = triangles_[t][i], b = triangles_[t][next(i)],
              c = triangles_[t][prev(i)];
  // u = (d, c, b): d follows b.
  std::size_t d =
      triangles_[u][next(static_cast<std::size_t>(local_index(u, b)))];
  return orient(nodes_[a], nodes_[b], nodes_[d]) > 0 &&
         orient(nodes_[a], nodes_[d], nodes_[c]) > 0;
}

std::array<std::size_t, 2> WorkingMesh::flip(std::size_t t, int edge) {
  if (!is_flippable(t, edge))
    throw std::invalid_argument("Edge is not flippable");
  auto i = static_cast<std::size_t>(edge);
  std::size_t u = neighbours_[t][i];
  // t = (a, b, c) with the edge (b, c); u = (d, c, b).
  std::size_t a = triangles_[t][i], b = triangles_[t][next(i)],
              c = triangles_[t][prev(i)];
  auto jb = static_cast<std::size_t>(local_index(u, b));
  std::size_t j = next(jb); // the vertex d of u
  std::size_t d = triangles_[u][j];
  std::size_t n_ab = neighbours_[t][prev(i)], n_ca = neighbours_[t][next(i)];
  // In u = (d, c, b): edge opposite c is (b, d), opposite b is (d, c).
  std::size_t n_bd = neighbours_[u][next(j)], n_dc = neighbours_[u][jb];
  triangles_[t] = {a, b, d};
  neighbours_[t] = {n_bd, u, n_ab};
  triangles_[u] = {a, d, c};
  neighbours_[u] = {n_dc, n_ca, t};
  replace_neighbour(n_bd, u, t);
  replace_neighbour(n_ca, t, u);
  set_node_triangles(t);
  set_node_triangles(u);
  return {t, u};
}

std::array<std::size_t, 3> WorkingMesh::insert_in_triangle(std::size_t t,
                                                           Point p) {
  auto [a, b, c] = triangles_[t];
  if (orient(nodes_[a], nodes_[b], p) <= 0 ||
      orient(nodes_[b], nodes_[c], p) <= 0 ||
      orient(nodes_[c], nodes_[a], p) <= 0)
    throw std::invalid_argument("Point is not strictly inside the triangle");
  std::size_t p_index = nodes_.size();
  std::size_t n1 = triangles_.size(), n2 = n1 + 1;
  auto old = neighbours_[t];
  std::size_t n_bc = old[0], n_ca = old[1], n_ab = old[2];
  nodes_.push_back(p);
  node_triangle_.push_back(t);
  outgoing_boundary_.push_back(no_index);
  triangles_[t] = {a, b, p_index};
  neighbours_[t] = {n1, n2, n_ab};
  triangles_.push_back({b, c, p_index});
  neighbours_.push_back({n2, t, n_bc});
  triangles_.push_back({c, a, p_index});
  neighbours_.push_back({t, n1, n_ca});
  replace_neighbour(n_bc, t, n1);
  replace_neighbour(n_ca, t, n2);
  set_node_triangles(t);
  set_node_triangles(n1);
  set_node_triangles(n2);
  return {t, n1, n2};
}

std::array<std::size_t, 4> WorkingMesh::split_interior_edge(std::size_t t,
                                                            int edge, Point p) {
  auto i = static_cast<std::size_t>(edge);
  std::size_t u = neighbours_[t][i];
  if (u == no_index)
    throw std::invalid_argument("split_interior_edge needs an interior edge");
  std::size_t a = triangles_[t][i], b = triangles_[t][next(i)],
              c = triangles_[t][prev(i)];
  auto jb = static_cast<std::size_t>(local_index(u, b));
  std::size_t j = next(jb);
  std::size_t d = triangles_[u][j];
  const auto &P = nodes_;
  if (orient(P[a], P[b], p) <= 0 || orient(P[a], p, P[c]) <= 0 ||
      orient(P[d], P[c], p) <= 0 || orient(P[d], p, P[b]) <= 0)
    throw std::invalid_argument("Split point would invert a triangle");
  std::size_t n_ab = neighbours_[t][prev(i)], n_ca = neighbours_[t][next(i)];
  std::size_t n_bd = neighbours_[u][next(j)], n_dc = neighbours_[u][jb];
  std::size_t q = nodes_.size();
  std::size_t n1 = triangles_.size(), n2 = n1 + 1;
  nodes_.push_back(p);
  node_triangle_.push_back(t);
  outgoing_boundary_.push_back(no_index);
  triangles_[t] = {a, b, q};
  neighbours_[t] = {n2, n1, n_ab};
  triangles_[u] = {d, c, q};
  neighbours_[u] = {n1, n2, n_dc};
  triangles_.push_back({a, q, c}); // n1
  neighbours_.push_back({u, n_ca, t});
  triangles_.push_back({d, q, b}); // n2
  neighbours_.push_back({t, n_bd, u});
  replace_neighbour(n_ca, t, n1);
  replace_neighbour(n_bd, u, n2);
  for (auto s : {t, u, n1, n2})
    set_node_triangles(s);
  return {t, u, n1, n2};
}

Point WorkingMesh::boundary_split_point(std::size_t k) const {
  const auto &e = boundary_edges_.at(k);
  Point chord_midpoint =
      geometry::midpoint(nodes_[e.nodes[0]], nodes_[e.nodes[1]]);
  if (!e.link)
    return chord_midpoint;
  const CurveChain &chain = chains_[e.link->chain];
  auto [curve, t] = chain.position(0.5 * (e.link->s0 + e.link->s1));
  // On a straight curve the chord midpoint is the curve point, and it keeps
  // exactly the chord's rounding.
  if (geometry::is_line(chain.curves()[curve]) &&
      e.link->s0 >= chain.offset(curve) &&
      e.link->s1 <= chain.offset(curve + 1))
    return chord_midpoint;
  return geometry::point_at(chain.curves()[curve], t);
}

BoundarySplitResult WorkingMesh::split_boundary_edge(std::size_t k) {
  if (unsplittable_.at(k)) {
    BoundarySplitResult r;
    r.failure = "an earlier split of this edge failed";
    return r;
  }
  BoundarySplitResult r = detail::boundary_split_policy().split(*this, k);
  if (!r.split)
    unsplittable_[k] = true;
  return r;
}

bool WorkingMesh::try_move_node(std::size_t node, Point p) {
  if (is_node_deleted(node))
    throw std::invalid_argument("A deleted node cannot be moved");
  if (is_boundary_node(node))
    throw std::invalid_argument("Boundary nodes cannot be moved");
  if (!geometry::is_finite(p))
    return false;
  auto around = triangles_around(node);
  for (auto t : around) {
    Point q[3];
    for (std::size_t i = 0; i < 3; ++i)
      q[i] = triangles_[t][i] == node ? p : nodes_[triangles_[t][i]];
    if (orient(q[0], q[1], q[2]) <= 0)
      return false;
  }
  nodes_[node] = p;
  return true;
}

void WorkingMesh::compact() {
  if (deleted_triangles_ == 0)
    return;
  std::vector<std::size_t> node_map(nodes_.size(), no_index),
      triangle_map(triangles_.size(), no_index);
  std::size_t n = 0, m = 0;
  for (std::size_t v = 0; v < nodes_.size(); ++v)
    if (!is_node_deleted(v)) {
      node_map[v] = n;
      nodes_[n] = nodes_[v];
      outgoing_boundary_[n] = outgoing_boundary_[v];
      node_triangle_[n] = node_triangle_[v];
      ++n;
    }
  for (std::size_t t = 0; t < triangles_.size(); ++t)
    if (!is_triangle_deleted(t)) {
      triangle_map[t] = m;
      triangles_[m] = triangles_[t];
      neighbours_[m] = neighbours_[t];
      ++m;
    }
  nodes_.resize(n);
  outgoing_boundary_.resize(n);
  node_triangle_.resize(n);
  triangles_.resize(m);
  neighbours_.resize(m);
  for (auto &v : node_triangle_)
    v = triangle_map[v];
  for (std::size_t t = 0; t < m; ++t)
    for (std::size_t i = 0; i < 3; ++i) {
      triangles_[t][i] = node_map[triangles_[t][i]];
      if (neighbours_[t][i] != no_index)
        neighbours_[t][i] = triangle_map[neighbours_[t][i]];
    }
  for (auto &e : boundary_edges_)
    for (auto &v : e.nodes)
      v = node_map[v];
  deleted_triangles_ = 0;
}

Mesh WorkingMesh::to_mesh() const {
  if (deleted_triangles_ > 0) {
    WorkingMesh copy = *this;
    copy.compact();
    return copy.to_mesh();
  }
  Mesh mesh;
  mesh.nodes = nodes_;
  mesh.triangles.reserve(triangles_.size());
  for (const auto &t : triangles_)
    mesh.triangles.push_back({t});
  mesh.region_names = region_names_;
  mesh.chains = chains_;
  std::vector<bool> visited(boundary_edges_.size(), false);
  mesh.boundary_edges.reserve(boundary_edges_.size());
  for (std::size_t k = 0; k < boundary_edges_.size(); ++k) {
    std::size_t e = k;
    while (!visited[e]) {
      visited[e] = true;
      const auto &be = boundary_edges_[e];
      mesh.boundary_edges.push_back({be.nodes, be.region, be.link});
      e = outgoing_boundary_[boundary_edges_[e].nodes[1]];
    }
  }
  return mesh;
}

} // namespace pde2d::mesh

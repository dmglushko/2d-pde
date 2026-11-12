#include "edge_recovery.hpp"

#include "pde2d/geometry/predicates.hpp"

#include <deque>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace pde2d::mesh::detail {

using geometry::orient;
using geometry::Point;

namespace {

bool properly_cross(Point a, Point b, Point c, Point d) {
  return orient(a, b, c) * orient(a, b, d) < 0 &&
         orient(c, d, a) * orient(c, d, b) < 0;
}

// The edges of `mesh` that cross the open segment a -> b, as node pairs.
std::vector<std::pair<std::size_t, std::size_t>>
crossed_edges(const WorkingMesh &mesh, std::size_t a, std::size_t b) {
  const auto &P = mesh.nodes();
  const Point A = P[a], B = P[b];
  std::vector<std::pair<std::size_t, std::size_t>> result;
  // The triangle around a through which the segment leaves.
  std::size_t right = no_index, left = no_index;
  for (auto t : mesh.triangles_around(a)) {
    const auto &v = mesh.triangles()[t];
    std::size_t k = 0;
    while (v[k] != a)
      ++k;
    std::size_t u = v[(k + 1) % 3], w = v[(k + 2) % 3];
    if (orient(A, B, P[u]) < 0 && orient(A, B, P[w]) > 0) {
      right = u;
      left = w;
      break;
    }
    if (orient(A, B, P[u]) == 0 && orient(A, P[u], B) == 0 &&
        geometry::on_segment(A, B, P[u]))
      throw std::invalid_argument(
          "Edge recovery: a node lies on the edge to recover");
  }
  if (right == no_index)
    throw std::logic_error("Edge recovery: no triangle around the edge start");
  while (true) {
    result.emplace_back(right, left);
    // Across the edge right -> left is the triangle with left -> right.
    auto [t, edge] = mesh.find_edge(left, right);
    if (t == no_index)
      throw std::logic_error(
          "Edge recovery: the walk along the edge left the mesh");
    std::size_t w = mesh.triangles()[t][static_cast<std::size_t>(edge)];
    if (w == b)
      return result;
    int side = orient(A, B, P[w]);
    if (side == 0)
      throw std::invalid_argument(
          "Edge recovery: a node lies on the edge to recover");
    if (side < 0)
      right = w;
    else
      left = w;
  }
}

} // namespace

std::size_t recover_edge(WorkingMesh &mesh, std::size_t a, std::size_t b,
                         std::vector<std::size_t> *touched) {
  if (mesh.find_edge(a, b).first != no_index ||
      mesh.find_edge(b, a).first != no_index)
    return 0;
  const auto &P = mesh.nodes();
  std::deque<std::pair<std::size_t, std::size_t>> queue;
  for (auto e : crossed_edges(mesh, a, b))
    queue.push_back(e);
  std::size_t flips = 0, guard = 0;
  const std::size_t limit = 1000 + 50 * queue.size() * queue.size();
  while (!queue.empty()) {
    if (++guard > limit)
      throw std::logic_error("Edge recovery did not terminate");
    auto [u, v] = queue.front();
    queue.pop_front();
    auto [t, edge] = mesh.find_edge(u, v);
    if (t == no_index)
      std::tie(t, edge) = mesh.find_edge(v, u);
    if (t == no_index)
      continue;
    if (!mesh.is_flippable(t, edge)) {
      queue.emplace_back(u, v);
      continue;
    }
    std::size_t c = mesh.triangles()[t][static_cast<std::size_t>(edge)];
    std::size_t n = mesh.neighbour(t, edge);
    std::size_t d = 0;
    for (auto x : mesh.triangles()[n])
      if (x != u && x != v)
        d = x;
    auto pair = mesh.flip(t, edge);
    if (touched)
      touched->insert(touched->end(), pair.begin(), pair.end());
    ++flips;
    if (c != a && c != b && d != a && d != b &&
        properly_cross(P[a], P[b], P[c], P[d]))
      queue.emplace_back(c, d);
  }
  if (mesh.find_edge(a, b).first == no_index &&
      mesh.find_edge(b, a).first == no_index)
    throw std::logic_error("Edge recovery failed");
  return flips;
}

} // namespace pde2d::mesh::detail

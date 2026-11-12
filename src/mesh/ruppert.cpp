#include "pde2d/mesh/ruppert.hpp"

#include "pde2d/mesh/domain_size.hpp"

#include "delaunay_internal.hpp"
#include "local_improvers.hpp"

#include "pde2d/geometry/predicates.hpp"
#include "pde2d/mesh/delaunay.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace pde2d::mesh {

using geometry::Point;

namespace {

double angle_at(Point at, Point p, Point q) {
  Point u = p - at, v = q - at;
  return std::atan2(std::abs(geometry::cross(u, v)), geometry::dot(u, v));
}

Point circumcenter(Point a, Point b, Point c) {
  Point u = b - a, v = c - a;
  double d = 2 * geometry::cross(u, v);
  double uu = geometry::dot(u, u), vv = geometry::dot(v, v);
  return a + Point{(v.y * uu - u.y * vv) / d, (u.x * vv - v.x * uu) / d};
}

// p lies strictly inside the circle with diameter ab.
bool encroaches(Point a, Point b, Point p) {
  return geometry::dot(a - p, b - p) < 0;
}

} // namespace

Ruppert::Ruppert(RuppertOptions options) : options_(options) {
  if (!(options_.min_angle_degrees > 0 && options_.min_angle_degrees <= 34))
    throw std::invalid_argument(
        "RuppertOptions::min_angle_degrees must be in (0, 34]");
  if (options_.max_points == 0)
    throw std::invalid_argument("RuppertOptions::max_points must be positive");
}

void Ruppert::run(WorkingMesh &mesh, const MeshSpec &spec,
                  std::span<const LocalImprover *const> local_improvers,
                  StageReport &report) const {
  validate(spec);
  const double size = domain_size(mesh);
  const double h = resolve(spec.max_edge_length, size);
  report.metrics["domain_size"] = size;
  report.metrics["max_edge_length"] = h;
  const double alpha = options_.min_angle_degrees * std::numbers::pi / 180;
  const auto &P = mesh.nodes();
  const auto &T = mesh.triangles();

  report.metrics["initial_flips"] = static_cast<double>(flip_to_delaunay(mesh));

  std::deque<std::size_t> segments, bad;
  for (std::size_t k = 0; k < mesh.boundary_edges().size(); ++k)
    segments.push_back(k);
  for (std::size_t t = 0; t < T.size(); ++t)
    bad.push_back(t);

  std::size_t inserted = 0, flips = 0, boundary_splits = 0, circumcenters = 0;
  std::size_t retriangulated = 0, deleted_nodes = 0, failed_splits = 0,
              abandoned = 0;
  detail::LocalImproverCalls local(local_improvers);
  auto count_insertion = [&] {
    if (++inserted > options_.max_points)
      throw std::runtime_error(
          "ruppert: inserted max_points = " +
          std::to_string(options_.max_points) +
          " points without reaching the angle bound; the boundary may have "
          "corners sharper than about 60 degrees, or features closer "
          "together than the edge length");
  };
  // After an insertion: restore Delaunay and queue what may need work.
  auto after = [&](std::span<const std::size_t> created) {
    std::vector<std::size_t> touched;
    flips += detail::lawson(mesh, created, &touched);
    local.after_change(mesh, touched);
    for (auto t : touched) {
      bad.push_back(t);
      for (int i = 0; i < 3; ++i) {
        std::size_t k = mesh.boundary_edge_index(t, i);
        if (k != no_index)
          segments.push_back(k);
      }
    }
  };
  auto segment_needs_split = [&](std::size_t k) {
    if (mesh.is_unsplittable(k))
      return false;
    auto [a, b] = mesh.boundary_edges()[k].nodes;
    if (geometry::distance(P[a], P[b]) > h)
      return true;
    auto [t, edge] = mesh.boundary_edge_location(k);
    return encroaches(P[a], P[b], P[T[t][static_cast<std::size_t>(edge)]]);
  };
  // Splits boundary edge k; false if it cannot be split (it is then
  // marked, never retried, and the report says so).
  auto split_segment = [&](std::size_t k) {
    if (mesh.is_unsplittable(k))
      return false;
    auto [a, b] = mesh.boundary_edges()[k].nodes;
    auto r = mesh.split_boundary_edge(k);
    if (!r.split) {
      ++failed_splits;
      report.messages.push_back(
          "boundary edge (" + std::to_string(a) + ", " + std::to_string(b) +
          ") of region '" +
          mesh.region_names()[mesh.boundary_edges()[k].region] +
          "' cannot be split: " + r.failure +
          "; it stays as it is, so the targets may not be met near it");
      return false;
    }
    count_insertion();
    ++boundary_splits;
    retriangulated += r.retriangulated;
    deleted_nodes += r.deleted_nodes;
    segments.push_back(k);
    segments.push_back(mesh.boundary_edges().size() - 1);
    after(r.touched);
    return true;
  };
  auto is_bad = [&](std::size_t t) {
    Point a = P[T[t][0]], b = P[T[t][1]], c = P[T[t][2]];
    if (std::max({geometry::distance(a, b), geometry::distance(b, c),
                  geometry::distance(c, a)}) > h)
      return true;
    return std::min({angle_at(a, b, c), angle_at(b, c, a), angle_at(c, a, b)}) <
           alpha;
  };
  // Boundary edges of the Delaunay cavity of p (the triangles whose
  // circumcircle contains p) that p encroaches.
  auto encroached_by = [&](std::size_t start, Point p) {
    std::vector<std::size_t> result, stack{start};
    std::vector<std::size_t> seen{start};
    while (!stack.empty()) {
      std::size_t t = stack.back();
      stack.pop_back();
      for (int i = 0; i < 3; ++i) {
        std::size_t u = mesh.neighbour(t, i);
        if (u == no_index) {
          std::size_t k = mesh.boundary_edge_index(t, i);
          auto [a, b] = mesh.boundary_edges()[k].nodes;
          if (encroaches(P[a], P[b], p) &&
              std::find(result.begin(), result.end(), k) == result.end())
            result.push_back(k);
        } else if (std::find(seen.begin(), seen.end(), u) == seen.end()) {
          seen.push_back(u);
          if (geometry::incircle(P[T[u][0]], P[T[u][1]], P[T[u][2]], p) > 0)
            stack.push_back(u);
        }
      }
    }
    return result;
  };

  while (true) {
    while (!segments.empty()) {
      std::size_t k = segments.front();
      segments.pop_front();
      if (segment_needs_split(k))
        split_segment(k);
    }
    if (bad.empty())
      break;
    std::size_t t = bad.front();
    bad.pop_front();
    if (mesh.is_triangle_deleted(t) || !is_bad(t))
      continue;
    Point c = circumcenter(P[T[t][0]], P[T[t][1]], P[T[t][2]]);
    Location loc = mesh.locate(c, t);
    // A triangle whose fix needs an unsplittable boundary edge is left
    // as it is.
    if (loc.kind == Location::Kind::blocked) {
      if (split_segment(mesh.boundary_edge_index(loc.triangle, loc.index)))
        bad.push_back(t);
      else
        ++abandoned;
      continue;
    }
    if (loc.kind == Location::Kind::on_node)
      throw std::logic_error("ruppert: a circumcenter coincides with a node");
    auto encroached = encroached_by(loc.triangle, c);
    if (!encroached.empty()) {
      bool any = false;
      for (auto k : encroached)
        any = split_segment(k) || any;
      if (any)
        bad.push_back(t);
      else
        ++abandoned;
      continue;
    }
    count_insertion();
    ++circumcenters;
    if (loc.kind == Location::Kind::inside) {
      auto created = mesh.insert_in_triangle(loc.triangle, c);
      after(created);
    } else {
      // On an interior edge (a point on a boundary edge encroaches it and
      // was handled above).
      auto created = mesh.split_interior_edge(loc.triangle, loc.index, c);
      after(created);
    }
  }
  report.metrics["flips"] = static_cast<double>(flips);
  report.metrics["boundary_splits"] = static_cast<double>(boundary_splits);
  report.metrics["circumcenters"] = static_cast<double>(circumcenters);
  report.metrics["retriangulated_splits"] = static_cast<double>(retriangulated);
  report.metrics["deleted_nodes"] = static_cast<double>(deleted_nodes);
  report.metrics["unsplittable_edges"] = static_cast<double>(failed_splits);
  report.metrics["abandoned_triangles"] = static_cast<double>(abandoned);
  if (abandoned > 0)
    report.messages.push_back(
        std::to_string(abandoned) +
        " bad triangles were left because fixing them needs a boundary edge "
        "that cannot be split; the angle bound may not hold there");
  local.record(report);
}

} // namespace pde2d::mesh

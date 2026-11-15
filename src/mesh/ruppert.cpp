#include "pde2d/mesh/ruppert.hpp"

#include "pde2d/mesh/domain_size.hpp"

#include "delaunay_internal.hpp"
#include "edge_bisection.hpp"
#include "local_improvers.hpp"

#include "pde2d/geometry/predicates.hpp"
#include "pde2d/mesh/delaunay.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
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

namespace {

// The state Ruppert's refinement works on, shared by run() and refine():
// the mesh, the queues of boundary edges and triangles to look at, and
// the counts for the report.
class Work {
public:
  Work(WorkingMesh &mesh, const RuppertOptions &options,
       std::span<const LocalImprover *const> local_improvers,
       StageReport &report)
      : mesh_(mesh), options_(options), report_(report),
        local_(local_improvers) {}

  // Makes the input (constrained) Delaunay; returns the number of flips.
  std::size_t make_delaunay() { return flip_to_delaunay(mesh_); }

  // A boundary edge needs a split when a node encroaches it, or when it is
  // longer than h (h = infinity: no length goal).
  bool segment_needs_split(std::size_t k, double h) const {
    if (mesh_.is_unsplittable(k))
      return false;
    const auto &P = mesh_.nodes();
    auto [a, b] = mesh_.boundary_edges()[k].nodes;
    if (geometry::distance(P[a], P[b]) > h)
      return true;
    auto [t, edge] = mesh_.boundary_edge_location(k);
    return encroaches(P[a], P[b],
                      P[mesh_.triangles()[t][static_cast<std::size_t>(edge)]]);
  }

  // Splits every queued boundary edge that needs it, until none does.
  void split_queued_segments(double h) {
    while (!segments_.empty()) {
      std::size_t k = segments_.front();
      segments_.pop_front();
      if (segment_needs_split(k, h))
        split_segment(k);
    }
  }

  // Splits boundary edge k; false if it cannot be split (it is then
  // marked, never retried, and the report says so).
  bool split_segment(std::size_t k) {
    if (mesh_.is_unsplittable(k))
      return false;
    auto [a, b] = mesh_.boundary_edges()[k].nodes;
    auto r = mesh_.split_boundary_edge(k);
    if (!r.split) {
      ++failed_splits_;
      report_.messages.push_back(
          "boundary edge (" + std::to_string(a) + ", " + std::to_string(b) +
          ") of region '" +
          mesh_.region_names()[mesh_.boundary_edges()[k].region] +
          "' cannot be split: " + r.failure +
          "; it stays as it is, so the targets may not be met near it");
      return false;
    }
    count_insertion();
    ++boundary_splits_;
    retriangulated_ += r.retriangulated;
    deleted_nodes_ += r.deleted_nodes;
    segments_.push_back(k);
    segments_.push_back(mesh_.boundary_edges().size() - 1);
    after(r.touched);
    return true;
  }

  // What became of an attempt to insert a circumcenter.
  enum class Insertion {
    inserted,
    segments_split, // it encroached boundary edges (or lay beyond one),
                    // which were split instead
    blocked,        // ... but none of them could be split
    on_node         // a node is already there
  };

  // Inserts point c (the circumcenter of a triangle near `start`), unless
  // it would encroach boundary edges or lies beyond one, in which case
  // those boundary edges are split instead.
  Insertion insert_circumcenter(Point c, std::size_t start) {
    Location loc = mesh_.locate(c, start);
    if (loc.kind == Location::Kind::blocked)
      return split_segment(mesh_.boundary_edge_index(loc.triangle, loc.index))
                 ? Insertion::segments_split
                 : Insertion::blocked;
    if (loc.kind == Location::Kind::on_node)
      return Insertion::on_node;
    auto encroached = encroached_by(loc.triangle, c);
    if (!encroached.empty()) {
      bool any = false;
      for (auto k : encroached)
        any = split_segment(k) || any;
      return any ? Insertion::segments_split : Insertion::blocked;
    }
    count_insertion();
    ++circumcenters_;
    if (loc.kind == Location::Kind::inside) {
      auto created = mesh_.insert_in_triangle(loc.triangle, c);
      after(created);
    } else {
      // On an interior edge (a point on a boundary edge encroaches it and
      // was handled above).
      auto created = mesh_.split_interior_edge(loc.triangle, loc.index, c);
      after(created);
    }
    return Insertion::inserted;
  }

  void queue_all_segments() {
    for (std::size_t k = 0; k < mesh_.boundary_edges().size(); ++k)
      segments_.push_back(k);
  }
  void queue_all_triangles() {
    for (std::size_t t = 0; t < mesh_.triangles().size(); ++t)
      bad_.push_back(t);
  }
  std::deque<std::size_t> &bad() { return bad_; }

  // The counts every report has.
  void record() const {
    report_.metrics["flips"] = static_cast<double>(flips_);
    report_.metrics["boundary_splits"] = static_cast<double>(boundary_splits_);
    report_.metrics["circumcenters"] = static_cast<double>(circumcenters_);
    report_.metrics["retriangulated_splits"] =
        static_cast<double>(retriangulated_);
    report_.metrics["deleted_nodes"] = static_cast<double>(deleted_nodes_);
    report_.metrics["unsplittable_edges"] = static_cast<double>(failed_splits_);
    local_.record(report_);
  }

private:
  void count_insertion() {
    if (++inserted_ > options_.max_points)
      throw std::runtime_error(
          "ruppert: inserted max_points = " +
          std::to_string(options_.max_points) +
          " points without reaching the angle bound; the boundary may have "
          "corners sharper than about 60 degrees, or features closer "
          "together than the edge length");
  }

  // After an insertion: restore Delaunay and queue what may need work.
  void after(std::span<const std::size_t> created) {
    std::vector<std::size_t> touched;
    flips_ += detail::lawson(mesh_, created, &touched);
    local_.after_change(mesh_, touched);
    for (auto t : touched) {
      bad_.push_back(t);
      for (int i = 0; i < 3; ++i) {
        std::size_t k = mesh_.boundary_edge_index(t, i);
        if (k != no_index)
          segments_.push_back(k);
      }
    }
  }

  // Boundary edges of the Delaunay cavity of p (the triangles whose
  // circumcircle contains p) that p encroaches.
  std::vector<std::size_t> encroached_by(std::size_t start, Point p) const {
    const auto &P = mesh_.nodes();
    const auto &T = mesh_.triangles();
    std::vector<std::size_t> result, stack{start};
    std::vector<std::size_t> seen{start};
    while (!stack.empty()) {
      std::size_t t = stack.back();
      stack.pop_back();
      for (int i = 0; i < 3; ++i) {
        std::size_t u = mesh_.neighbour(t, i);
        if (u == no_index) {
          std::size_t k = mesh_.boundary_edge_index(t, i);
          auto [a, b] = mesh_.boundary_edges()[k].nodes;
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
  }

  WorkingMesh &mesh_;
  const RuppertOptions &options_;
  StageReport &report_;
  detail::LocalImproverCalls local_;
  std::deque<std::size_t> segments_, bad_;
  std::size_t inserted_ = 0, flips_ = 0, boundary_splits_ = 0,
              circumcenters_ = 0, retriangulated_ = 0, deleted_nodes_ = 0,
              failed_splits_ = 0;
};

} // namespace

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

  Work work(mesh, options_, local_improvers, report);
  report.metrics["initial_flips"] = static_cast<double>(work.make_delaunay());
  work.queue_all_segments();
  work.queue_all_triangles();

  auto is_bad = [&](std::size_t t) {
    Point a = P[T[t][0]], b = P[T[t][1]], c = P[T[t][2]];
    if (std::max({geometry::distance(a, b), geometry::distance(b, c),
                  geometry::distance(c, a)}) > h)
      return true;
    return std::min({angle_at(a, b, c), angle_at(b, c, a), angle_at(c, a, b)}) <
           alpha;
  };

  std::size_t abandoned = 0;
  auto &bad = work.bad();
  while (true) {
    work.split_queued_segments(h);
    if (bad.empty())
      break;
    std::size_t t = bad.front();
    bad.pop_front();
    if (mesh.is_triangle_deleted(t) || !is_bad(t))
      continue;
    Point c = circumcenter(P[T[t][0]], P[T[t][1]], P[T[t][2]]);
    switch (work.insert_circumcenter(c, t)) {
    case Work::Insertion::inserted:
      break;
    case Work::Insertion::segments_split:
      bad.push_back(t); // look at it again once the segments are split
      break;
    case Work::Insertion::blocked:
      // A triangle whose fix needs an unsplittable boundary edge is left
      // as it is.
      ++abandoned;
      break;
    case Work::Insertion::on_node:
      throw std::logic_error("ruppert: a circumcenter coincides with a node");
    }
  }
  work.record();
  report.metrics["abandoned_triangles"] = static_cast<double>(abandoned);
  if (abandoned > 0)
    report.messages.push_back(
        std::to_string(abandoned) +
        " bad triangles were left because fixing them needs a boundary edge "
        "that cannot be split; the angle bound may not hold there");
}

std::optional<Parents>
Ruppert::refine(WorkingMesh &mesh, std::span<const std::size_t> marked,
                std::span<const LocalImprover *const> local_improvers,
                StageReport &report) const {
  detail::check_marked(mesh, marked, name());
  std::vector<std::size_t> sorted(marked.begin(), marked.end());
  std::sort(sorted.begin(), sorted.end());
  sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
  report.metrics["marked"] = static_cast<double>(sorted.size());
  if (sorted.empty())
    report.messages.push_back(
        "nothing was marked; the mesh is returned unchanged");

  // The circumcenters of the marked triangles as they are now, before
  // anything changes, with a node of each triangle to start the search.
  const auto &P = mesh.nodes();
  const auto &T = mesh.triangles();
  struct Target {
    Point center;
    std::size_t triangle, node;
  };
  std::vector<Target> targets;
  for (auto t : sorted)
    targets.push_back(
        {circumcenter(P[T[t][0]], P[T[t][1]], P[T[t][2]]), t, T[t][0]});

  Work work(mesh, options_, local_improvers, report);
  std::size_t initial_flips = 0, skipped = 0, blocked = 0;
  if (!sorted.empty())
    initial_flips = work.make_delaunay();
  const double no_length_goal = std::numeric_limits<double>::infinity();
  for (const auto &target : targets) {
    // Start the walk near the triangle: at its index if it is still
    // there (indices are reused locally), else at its first node.
    std::size_t start = target.triangle;
    if (mesh.is_triangle_deleted(start)) {
      if (mesh.is_node_deleted(target.node)) {
        ++skipped; // cut off with the region outside the true domain
        continue;
      }
      start = mesh.triangles_around(target.node).front();
    }
    switch (work.insert_circumcenter(target.center, start)) {
    case Work::Insertion::inserted:
    case Work::Insertion::segments_split:
      break;
    case Work::Insertion::blocked:
      ++blocked;
      break;
    case Work::Insertion::on_node:
      ++skipped; // another marked triangle had the same circumcenter
      break;
    }
    // Boundary edges the new points encroach are split, as in run().
    work.split_queued_segments(no_length_goal);
  }
  work.record();
  report.metrics["initial_flips"] = static_cast<double>(initial_flips);
  report.metrics["skipped_circumcenters"] = static_cast<double>(skipped);
  report.metrics["blocked_circumcenters"] = static_cast<double>(blocked);
  if (blocked > 0)
    report.messages.push_back(
        std::to_string(blocked) +
        " marked triangles were not refined because their circumcenter "
        "needs a boundary edge that cannot be split");
  // Ruppert flips edges, so a new triangle need not lie in one input
  // triangle.
  report.messages.push_back("parents dropped: ruppert flips edges");
  return std::nullopt;
}

} // namespace pde2d::mesh

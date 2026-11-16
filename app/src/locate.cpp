#include "locate.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::app {

namespace {

constexpr int samples = 64;

double squared(geometry::Point a, geometry::Point b) {
  const geometry::Point d = a - b;
  return geometry::dot(d, d);
}

// The parameter of the point of `curve` nearest to `p`: the best of
// `samples` + 1 points, refined by golden-section search between its
// neighbours (the distance is unimodal there for these curves), then Newton's method.
double nearest_parameter(const geometry::Curve &curve, geometry::Point p) {
  int best = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i <= samples; ++i) {
    const double d =
        squared(curve.point_at(static_cast<double>(i) / samples), p);
    if (d < best_distance) {
      best_distance = d;
      best = i;
    }
  }
  double lo = std::max(0, best - 1) / static_cast<double>(samples);
  double hi = std::min(samples, best + 1) / static_cast<double>(samples);
  const double ratio = (std::sqrt(5.0) - 1) / 2;
  double a = hi - ratio * (hi - lo);
  double b = lo + ratio * (hi - lo);
  double fa = squared(curve.point_at(a), p);
  double fb = squared(curve.point_at(b), p);
  for (int k = 0; k < 80 && hi - lo > 1e-15; ++k) {
    if (fa < fb) {
      hi = b;
      b = a;
      fb = fa;
      a = hi - ratio * (hi - lo);
      fa = squared(curve.point_at(a), p);
    } else {
      lo = a;
      a = b;
      fa = fb;
      b = lo + ratio * (hi - lo);
      fb = squared(curve.point_at(b), p);
    }
  }
  double t = (lo + hi) / 2;
  // Newton on (c(t) - p) . c'(t) = 0 makes the last digits exact (the
  // distance is too flat at its minimum for the search alone).
  for (int k = 0; k < 4; ++k) {
    const geometry::Point d = curve.point_at(t) - p;
    const geometry::Point d1 = curve.derivative_at(t);
    const double slope = geometry::dot(d1, d1) +
                         geometry::dot(d, curve.second_derivative_at(t));
    if (!(slope > 0))
      break;
    t = std::clamp(t - geometry::dot(d, d1) / slope, 0.0, 1.0);
  }
  // The ends themselves, which the search may not reach.
  for (const double end : {0.0, 1.0})
    if (squared(curve.point_at(end), p) <= squared(curve.point_at(t), p))
      t = end;
  return t;
}

Json point_json(geometry::Point p) { return Json::array({p.x, p.y}); }

// A name for the second half of a trial split that no region has.
std::string unused_name(const std::vector<std::string> &names,
                        const std::string &base) {
  for (int k = 2;; ++k) {
    const std::string name = base + "_" + std::to_string(k);
    if (std::ranges::find(names, name) == names.end())
      return name;
  }
}

// CurveGeometry::split_region's fraction of the point at parameter t of
// curve `c` of loop `l`, or empty when the region is not one run of
// curves on that loop (the library then says why).
std::optional<double> region_fraction(const geometry::CurveGeometry &geometry,
                                      std::size_t l, std::size_t c, double t) {
  const auto &loops = geometry.loops();
  const std::string &region = loops[l].curves[c].region;
  std::size_t total_count = 0;
  for (std::size_t k = 0; k < loops.size(); ++k)
    for (const geometry::BoundaryCurve &curve : loops[k].curves)
      if (curve.region == region) {
        if (k != l)
          return std::nullopt;
        ++total_count;
      }
  const auto &curves = loops[l].curves;
  const std::size_t n = curves.size();
  std::size_t start = 0;
  if (total_count < n)
    for (std::size_t i = 0; i < n; ++i)
      if (curves[i].region == region && curves[(i + n - 1) % n].region != region) {
        start = i;
        break;
      }
  std::size_t run = 0;
  while (run < n && curves[(start + run) % n].region == region)
    ++run;
  if (run != total_count)
    return std::nullopt;
  // The lengths in the library's order, summed as it sums them.
  double total = 0;
  double before = 0;
  std::optional<double> at;
  for (std::size_t i = 0; i < run; ++i) {
    const std::size_t index = (start + i) % n;
    const double length = geometry::curve_length(curves[index].curve);
    if (index == c) {
      at = t <= 0   ? before
           : t >= 1 ? before + length
                    : before + geometry::curve_length(curves[index].curve, t);
    }
    before += length;
    total += length;
  }
  return *at / total;
}

Json locate_on_curves(const geometry::CurveGeometry &geometry,
                      geometry::Point p, double snap) {
  const auto &loops = geometry.loops();
  // The nearest point of every curve.
  std::size_t best_loop = 0;
  std::size_t best_curve = 0;
  double best_t = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t l = 0; l < loops.size(); ++l)
    for (std::size_t c = 0; c < loops[l].curves.size(); ++c) {
      const geometry::Curve &curve = loops[l].curves[c].curve;
      const double t = nearest_parameter(curve, p);
      const double d = squared(curve.point_at(t), p);
      if (d < best) {
        best = d;
        best_loop = l;
        best_curve = c;
        best_t = t;
      }
    }
  // A corner within `snap`: the nearest curve's end if it is there, else
  // the curve that starts there.
  bool corner = false;
  double corner_distance = snap;
  for (std::size_t l = 0; l < loops.size(); ++l)
    for (std::size_t c = 0; c < loops[l].curves.size(); ++c) {
      const double d =
          geometry::distance(geometry::start_point(loops[l].curves[c].curve), p);
      if (d > corner_distance)
        continue;
      corner = true;
      corner_distance = d;
      const std::size_t n = loops[l].curves.size();
      if (l == best_loop && c == best_curve)
        best_t = 0;
      else if (l == best_loop && (c + n - 1) % n == best_curve)
        best_t = 1;
      else {
        best_loop = l;
        best_curve = c;
        best_t = 0;
      }
    }
  const geometry::Curve &curve = loops[best_loop].curves[best_curve].curve;
  const geometry::Point point = best_t <= 0   ? geometry::start_point(curve)
                                : best_t >= 1 ? geometry::end_point(curve)
                                              : curve.point_at(best_t);
  const std::string &region = loops[best_loop].curves[best_curve].region;
  const std::optional<double> fraction =
      region_fraction(geometry, best_loop, best_curve, best_t);

  // Whether the library splits there: tried on a copy.
  std::optional<std::string> reason;
  if (fraction && !(*fraction > 0 && *fraction < 1))
    reason = "the split point coincides with an end of the region";
  else
    try {
      geometry::CurveGeometry trial = geometry;
      trial.split_region(region, fraction.value_or(0.5), region,
                         unused_name(geometry.region_names(), region));
    } catch (const std::invalid_argument &error) {
      reason = error.what();
    }

  Json out = Json::object();
  out["kind"] = "curves";
  out["region"] = region;
  out["loop"] = best_loop;
  out["curve"] = best_curve;
  out["t"] = best_t;
  out["point"] = point_json(point);
  out["corner"] = corner;
  out["fraction"] = fraction ? Json(*fraction) : Json(nullptr);
  out["splittable"] = fraction.has_value() && !reason;
  out["reason"] = reason ? Json(*reason) : Json(nullptr);
  out["distance"] = geometry::distance(point, p);
  return out;
}

double segment_distance(geometry::Point p, geometry::Point a,
                        geometry::Point b) {
  const geometry::Point d = b - a;
  const double length = geometry::dot(d, d);
  const double s =
      length == 0 ? 0 : std::clamp(geometry::dot(p - a, d) / length, 0.0, 1.0);
  return geometry::distance(p, a + s * d);
}

Json locate_on_mesh(const mesh::Mesh &mesh, geometry::Point p) {
  if (mesh.boundary_edges.empty())
    throw std::invalid_argument("the mesh has no boundary edges");
  std::size_t best = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t e = 0; e < mesh.boundary_edges.size(); ++e) {
    const auto &nodes = mesh.boundary_edges[e].nodes;
    const double d =
        segment_distance(p, mesh.nodes[nodes[0]], mesh.nodes[nodes[1]]);
    if (d < best_distance) {
      best_distance = d;
      best = e;
    }
  }
  const mesh::BoundaryEdge &edge = mesh.boundary_edges[best];
  const std::size_t node =
      geometry::distance(mesh.nodes[edge.nodes[0]], p) <=
              geometry::distance(mesh.nodes[edge.nodes[1]], p)
          ? edge.nodes[0]
          : edge.nodes[1];
  const std::string &region = mesh.region_names[edge.region];
  std::optional<std::string> reason;
  try {
    mesh::Mesh trial = mesh;
    split_mesh_region_at_node(trial, region, node, region,
                              unused_name(mesh.region_names, region));
  } catch (const std::invalid_argument &error) {
    reason = error.what();
  }
  Json out = Json::object();
  out["kind"] = "mesh_file";
  out["region"] = region;
  out["node"] = node;
  out["point"] = point_json(mesh.nodes[node]);
  out["corner"] = true;
  out["splittable"] = !reason;
  out["reason"] = reason ? Json(*reason) : Json(nullptr);
  out["distance"] = geometry::distance(mesh.nodes[node], p);
  return out;
}

} // namespace

Json locate_boundary(const Domain &domain, geometry::Point point, double snap) {
  if (domain.geometry)
    return locate_on_curves(*domain.geometry, point, snap);
  return locate_on_mesh(*domain.mesh, point);
}

} // namespace pde2d::app

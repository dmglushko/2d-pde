#include "pde2d/mesh/remesher.hpp"

#include "loop_chains.hpp"
#include "stage.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>

namespace pde2d::mesh {

using geometry::Point;

namespace {

double longest_edge(const Mesh &mesh) {
  double longest = 0;
  for (const auto &t : mesh.triangles)
    for (std::size_t i = 0; i < 3; ++i)
      longest = std::max(longest,
                         geometry::distance(mesh.nodes[t.nodes[i]],
                                            mesh.nodes[t.nodes[(i + 1) % 3]]));
  return longest;
}

// A curve with a padded bounding box of its points.
struct CandidateCurve {
  geometry::Curve curve;
  std::string region;
  double xmin, xmax, ymin, ymax;
};

constexpr int samples = 256;

CandidateCurve candidate(const geometry::Curve &curve,
                         const std::string &region, double tolerance) {
  CandidateCurve c{curve, region, INFINITY, -INFINITY, INFINITY, -INFINITY};
  for (int k = 0; k <= samples; ++k) {
    Point p = geometry::point_at(curve, double(k) / samples);
    c.xmin = std::min(c.xmin, p.x);
    c.xmax = std::max(c.xmax, p.x);
    c.ymin = std::min(c.ymin, p.y);
    c.ymax = std::max(c.ymax, p.y);
  }
  // Between samples the curve leaves the sample polygon by at most
  // M h^2 / 8 (M bounds |c''|, h = 1 / samples).
  double pad =
      geometry::derivative_bounds(curve).second / (8.0 * samples * samples) +
      tolerance;
  c.xmin -= pad;
  c.xmax += pad;
  c.ymin -= pad;
  c.ymax += pad;
  return c;
}

// Parameter of the curve point closest to p, and the distance.
std::pair<double, double> project(const geometry::Curve &curve, Point p) {
  if (const auto *line = std::get_if<geometry::Line>(&curve)) {
    Point d = line->end - line->start;
    double t = std::clamp(
        geometry::dot(p - line->start, d) / geometry::dot(d, d), 0.0, 1.0);
    return {t, geometry::distance(geometry::point_at(curve, t), p)};
  }
  auto dist2 = [&](double t) {
    Point d = geometry::point_at(curve, t) - p;
    return geometry::dot(d, d);
  };
  int best = 0;
  for (int k = 1; k <= samples; ++k)
    if (dist2(double(k) / samples) < dist2(double(best) / samples))
      best = k;
  // Golden-section search around the best sample.
  double lo = std::max(0.0, double(best - 1) / samples);
  double hi = std::min(1.0, double(best + 1) / samples);
  const double g = (std::sqrt(5.0) - 1) / 2;
  double x1 = hi - g * (hi - lo), x2 = lo + g * (hi - lo);
  double f1 = dist2(x1), f2 = dist2(x2);
  for (int i = 0; i < 80; ++i) {
    if (f1 < f2) {
      hi = x2;
      x2 = x1;
      f2 = f1;
      x1 = hi - g * (hi - lo);
      f1 = dist2(x1);
    } else {
      lo = x1;
      x1 = x2;
      f1 = f2;
      x2 = lo + g * (hi - lo);
      f2 = dist2(x2);
    }
  }
  double t = 0.5 * (lo + hi);
  // The ends of the interval may be closer (end points of the curve).
  for (double e : {0.0, 1.0})
    if (dist2(e) < dist2(t))
      t = e;
  return {t, std::sqrt(dist2(t))};
}

// A chain of the geometry, as the discretizer builds it.
struct GeometryChain {
  CurveChain chain;
  std::string region;
  bool closed; // the whole loop, from the start of its first curve
};

WorkingMesh link_to_geometry(const Mesh &mesh,
                             const geometry::CurveGeometry &geometry,
                             double corner_angle_degrees, StageReport &report) {
  auto geometry_regions = geometry.region_names();
  for (const auto &name : mesh.region_names)
    if (std::find(geometry_regions.begin(), geometry_regions.end(), name) ==
        geometry_regions.end())
      throw std::invalid_argument("Remesher: mesh region '" + name +
                                  "' does not exist in the geometry");
  double xmin = INFINITY, xmax = -INFINITY, ymin = INFINITY, ymax = -INFINITY;
  for (auto p : mesh.nodes) {
    xmin = std::min(xmin, p.x);
    xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y);
    ymax = std::max(ymax, p.y);
  }
  const double tolerance = 1e-9 * std::hypot(xmax - xmin, ymax - ymin);
  // The chains, and for every geometry curve its chain and position in it.
  std::vector<GeometryChain> chains;
  std::vector<CandidateCurve> curves;
  std::vector<std::pair<std::size_t, std::size_t>> curve_chain; // chain, k
  for (const auto &loop : geometry.loops()) {
    auto loop_chains = detail::loop_chains(loop, corner_angle_degrees);
    bool closed = loop_chains.size() == 1 &&
                  loop_chains[0].count == loop.curves.size() &&
                  loop_chains[0].first == 0;
    for (auto &c : loop_chains) {
      for (std::size_t k = 0; k < c.count; ++k) {
        const auto &bc = loop.curves[(c.first + k) % loop.curves.size()];
        curves.push_back(candidate(bc.curve, bc.region, tolerance));
        curve_chain.emplace_back(chains.size(), k);
      }
      chains.push_back(
          {std::move(c.chain), loop.curves[c.first].region, closed});
    }
  }

  // For a node: its arc length on each chain it lies on.
  auto on_chains = [&](Point p, const std::string &region) {
    std::vector<std::pair<std::size_t, double>> result;
    for (std::size_t j = 0; j < curves.size(); ++j) {
      const auto &c = curves[j];
      if (c.region != region || p.x < c.xmin || p.x > c.xmax || p.y < c.ymin ||
          p.y > c.ymax)
        continue;
      auto [t, d] = project(c.curve, p);
      if (d > tolerance)
        continue;
      auto [index, k] = curve_chain[j];
      const CurveChain &chain = chains[index].chain;
      // t = 1 maps to exactly the start of the next curve.
      double s = t == 1.0
                     ? chain.offset(k + 1)
                     : chain.offset(k) + geometry::curve_length(c.curve, t);
      result.emplace_back(index, std::min(s, chain.length()));
    }
    return result;
  };

  std::vector<WorkingBoundaryEdge> edges;
  std::size_t linked = 0, unlinked = 0;
  for (std::size_t e = 0; e < mesh.boundary_edges.size(); ++e) {
    const auto &be = mesh.boundary_edges[e];
    if (be.region >= mesh.region_names.size())
      throw std::invalid_argument("Remesher: boundary edge " +
                                  std::to_string(e) +
                                  " has a region id out of range");
    if (be.nodes[0] >= mesh.nodes.size() || be.nodes[1] >= mesh.nodes.size())
      throw std::invalid_argument("Remesher: boundary edge " +
                                  std::to_string(e) +
                                  " uses a node index out of range");
    const auto &region = mesh.region_names[be.region];
    Point a = mesh.nodes[be.nodes[0]], b = mesh.nodes[be.nodes[1]];
    auto on_a = on_chains(a, region), on_b = on_chains(b, region);
    if (on_a.empty() || on_b.empty())
      throw std::invalid_argument("Remesher: boundary edge " +
                                  std::to_string(e) + " of region '" + region +
                                  "' does not lie on a curve of that region");
    std::optional<CurveLink> link;
    for (auto [ca, sa] : on_a)
      for (auto [cb, sb] : on_b) {
        if (ca != cb || link)
          continue;
        const auto &chain = chains[ca];
        const double length = chain.chain.length();
        // On a closed chain the start of the loop is also its end.
        if (chain.closed && sa == length)
          sa = 0.0;
        if (chain.closed && sb == 0.0)
          sb = length;
        if (!(sa < sb))
          continue;
        // The short way: the chain's middle point must be near the chord.
        Point mid = chain.chain.point_at(0.5 * (sa + sb));
        if (geometry::distance(mid, geometry::midpoint(a, b)) >
            geometry::distance(a, b))
          continue;
        link = CurveLink{ca, sa, sb};
      }
    if (link)
      ++linked;
    else
      ++unlinked;
    edges.push_back({be.nodes, be.region, link});
  }
  report.metrics["chains"] = static_cast<double>(chains.size());
  report.metrics["linked_edges"] = static_cast<double>(linked);
  report.metrics["unlinked_edges"] = static_cast<double>(unlinked);
  if (unlinked > 0)
    report.messages.push_back(
        std::to_string(unlinked) +
        " boundary edges span two chains (a corner, a region change or the "
        "start of a closed loop) and are split at their chord midpoint");
  std::vector<WorkingMesh::TriangleNodes> triangles;
  for (const auto &t : mesh.triangles)
    triangles.push_back(t.nodes);
  std::vector<CurveChain> chain_list;
  for (auto &c : chains)
    chain_list.push_back(std::move(c.chain));
  return WorkingMesh(mesh.nodes, std::move(triangles), std::move(edges),
                     mesh.region_names, std::move(chain_list));
}

// The plan in words: "plan: refiner ruppert; local: none; final:
// delaunay_flips, laplacian_smoothing".
std::string describe(const PipelinePlan &plan, const Refiner *refiner,
                     const std::vector<std::shared_ptr<const Improver>> &list) {
  auto names = [&](const std::vector<std::size_t> &indices) {
    if (indices.empty())
      return std::string("none");
    std::string text;
    for (auto i : indices)
      text += (text.empty() ? "" : ", ") + list[i]->name();
    return text;
  };
  return "plan: " +
         (refiner ? "refiner " + refiner->name() : std::string("no refiner")) +
         "; local: " + names(plan.local_improvers) +
         "; final: " + names(plan.final_passes);
}

} // namespace

Remesher::Remesher(std::shared_ptr<const Refiner> refiner,
                   std::vector<std::shared_ptr<const Improver>> improvers)
    : refiner_(std::move(refiner)), improvers_(std::move(improvers)) {
  detail::timed_stage(pipeline_report_, "pipeline", [&](StageReport &r) {
    plan_ = plan_pipeline(refiner_.get(), improvers_);
    for (auto i : plan_.local_improvers)
      local_improvers_.push_back(improvers_[i]->as_local());
    for (auto i : plan_.final_passes)
      final_improvers_.push_back(improvers_[i]->as_final());
    r.messages.push_back(describe(plan_, refiner_.get(), improvers_));
    r.messages.insert(r.messages.end(), plan_.warnings.begin(),
                      plan_.warnings.end());
    r.metrics["local_improvers"] =
        static_cast<double>(plan_.local_improvers.size());
    r.metrics["final_passes"] = static_cast<double>(plan_.final_passes.size());
  });
}

MeshResult Remesher::run(const Mesh &mesh, const MeshSpec &spec) const {
  validate(spec);
  return run(WorkingMesh::from_mesh(mesh), spec);
}

MeshResult Remesher::run(const Mesh &mesh,
                         const geometry::CurveGeometry &geometry,
                         const MeshSpec &spec,
                         double corner_angle_degrees) const {
  validate(spec);
  detail::validate_corner_angle(corner_angle_degrees,
                                "Remesher: corner_angle_degrees");
  StageReport report;
  WorkingMesh working =
      detail::timed_stage(report, "curve_links", [&](auto &r) {
        return link_to_geometry(mesh, geometry, corner_angle_degrees, r);
      });
  MeshResult result = run(std::move(working), spec);
  // After the pipeline report.
  result.reports.insert(result.reports.begin() + 1, std::move(report));
  return result;
}

MeshResult Remesher::run(WorkingMesh mesh, const MeshSpec &spec) const {
  validate(spec);
  MeshResult result;
  result.reports.push_back(pipeline_report_);
  if (refiner_) {
    StageReport report;
    detail::timed_stage(report, refiner_->name(), [&](auto &r) {
      refiner_->run(mesh, spec, local_improvers_, r);
    });
    // Final passes see a mesh without deleted triangles or nodes.
    mesh.compact();
    result.reports.push_back(std::move(report));
  }
  for (std::size_t k = 0; k < plan_.final_passes.size(); ++k) {
    StageReport report;
    detail::timed_stage(
        report, improvers_[plan_.final_passes[k]]->name(),
        [&](auto &r) { final_improvers_[k]->run(mesh, spec, r); });
    result.reports.push_back(std::move(report));
  }
  result.mesh = mesh.to_mesh();
  result.longest_edge = longest_edge(result.mesh);
  result.target_met = result.longest_edge <= spec.max_edge_length;
  return result;
}

} // namespace pde2d::mesh

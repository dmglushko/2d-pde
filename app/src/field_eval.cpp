#include "field_eval.hpp"

#include "app_error.hpp"
#include "binary.hpp"

#include "pde2d/fem/expression.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>

namespace pde2d::app {

namespace {

const std::vector<double> &values_of(const RunSnapshots &run, std::size_t k) {
  return run.snapshots[k]->values;
}

Json number_or_null(double value) {
  return std::isfinite(value) ? Json(value) : Json(nullptr);
}

// Two snapshot times are the same when they differ by rounding only.
bool same_time(double a, double b) {
  return std::abs(a - b) <= 1e-12 * std::max({1.0, std::abs(a), std::abs(b)});
}

} // namespace

std::vector<double> values_at_nodes(const mesh::Mesh &mesh,
                                    const fem::CompiledExpression &expression,
                                    double t) {
  std::vector<double> values;
  values.reserve(mesh.nodes.size());
  for (const geometry::Point &p : mesh.nodes)
    values.push_back(expression(p.x, p.y, t));
  return values;
}

std::vector<double> lumped_weights(const mesh::Mesh &mesh) {
  std::vector<double> weights(mesh.nodes.size(), 0.0);
  for (const mesh::Triangle &triangle : mesh.triangles) {
    const geometry::Point a = mesh.nodes[triangle.nodes[0]];
    const geometry::Point b = mesh.nodes[triangle.nodes[1]];
    const geometry::Point c = mesh.nodes[triangle.nodes[2]];
    const double area =
        0.5 * std::abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
    for (const std::size_t node : triangle.nodes)
      weights[node] += area / 3.0;
  }
  return weights;
}

DifferenceStats difference_stats(std::span<const double> u,
                                 std::span<const double> other,
                                 std::span<const double> weights) {
  DifferenceStats out;
  double squares = 0.0;
  double weighted = 0.0;
  for (std::size_t i = 0; i < u.size(); ++i) {
    const double d = u[i] - other[i];
    if (!std::isfinite(d)) {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      return {.max_abs = nan, .rms = nan, .l2 = nan};
    }
    out.max_abs = std::max(out.max_abs, std::abs(d));
    squares += d * d;
    weighted += weights[i] * d * d;
  }
  out.rms =
      u.empty() ? 0.0 : std::sqrt(squares / static_cast<double>(u.size()));
  out.l2 = std::sqrt(weighted);
  return out;
}

Comparison::Comparison(RunSnapshots run, const fem::Parameters &parameters,
                       const Json &request,
                       std::optional<RunSnapshots> reference)
    : run_(std::move(run)) {
  const Json &exact = member(request, "exact");
  const Json &against_reference = member(request, "reference");
  if (exact.is_null() == against_reference.is_null())
    throw PathError("request",
                    "give either \"exact\" (an expression) or \"reference\": "
                    "true");
  if (!exact.is_null()) {
    text_ = require_string(exact, "exact");
    exact_.emplace(at_path("exact", [&] {
      return fem::CompiledExpression(fem::Expression(text_), parameters);
    }));
    return;
  }
  if (against_reference != Json(true))
    throw PathError("reference", "expected true");
  if (!reference)
    throw PathError("reference", "no reference run is kept (keep one with "
                                 "/api/solve/reference)");
  // The same mesh object, so node i is the same point in both (every
  // snapshot of each: an adaptive run in time has several meshes).
  const auto on_one_mesh = [](const RunSnapshots &snapshots) {
    return std::ranges::all_of(snapshots.snapshots, [&](const Snapshot &s) {
      return s->mesh == snapshots.mesh;
    });
  };
  if (!on_one_mesh(run_) || !on_one_mesh(*reference))
    throw PathError(
        "reference",
        std::format("the run {} or the reference run {} has snapshots on "
                    "several meshes (an adaptive run in time); a comparison "
                    "node by node needs one mesh",
                    run_.id, reference->id));
  if (reference->mesh != run_.mesh)
    throw PathError(
        "reference",
        std::format("the reference run {} is on another mesh than run {}; "
                    "a comparison node by node needs the same mesh (solve "
                    "both without changing the geometry or the mesh)",
                    reference->id, run_.id));
  reference_ = std::move(reference);
  const auto &theirs = reference_->snapshots;
  for (const Snapshot &mine : run_.snapshots) {
    std::optional<std::size_t> match;
    if (theirs.size() == 1)
      match = 0; // a static reference stands for every time
    else
      for (std::size_t j = 0; j < theirs.size(); ++j)
        if (same_time(theirs[j]->time, mine->time)) {
          match = j;
          break;
        }
    matched_.push_back(match);
  }
}

std::optional<std::vector<double>> Comparison::other(std::size_t k) const {
  if (exact_)
    return values_at_nodes(*run_.snapshots[k]->mesh, *exact_,
                           run_.snapshots[k]->time);
  if (k >= matched_.size() || !matched_[k])
    return std::nullopt;
  return reference_->snapshots[*matched_[k]]->values;
}

Json Comparison::summary() const {
  // The lumped weights of each mesh the snapshots are on (one, except for
  // an adaptive run in time).
  std::vector<std::pair<const mesh::Mesh *, std::vector<double>>> weights;
  const auto weights_of =
      [&](const mesh::Mesh &mesh) -> const std::vector<double> & {
    for (const auto &[known, values] : weights)
      if (known == &mesh)
        return values;
    weights.emplace_back(&mesh, lumped_weights(mesh));
    return weights.back().second;
  };
  Json snapshots = Json::array();
  double max_abs = 0.0, max_rms = 0.0, max_l2 = 0.0;
  bool any = false, finite = true;
  for (std::size_t k = 0; k < run_.snapshots.size(); ++k) {
    const auto values = other(k);
    if (!values) {
      snapshots.push_back(nullptr);
      continue;
    }
    const DifferenceStats stats = difference_stats(
        values_of(run_, k), *values, weights_of(*run_.snapshots[k]->mesh));
    Json item = Json::object();
    item["t"] = run_.snapshots[k]->time;
    item["max_abs"] = number_or_null(stats.max_abs);
    item["rms"] = number_or_null(stats.rms);
    item["l2"] = number_or_null(stats.l2);
    snapshots.push_back(std::move(item));
    any = true;
    if (!std::isfinite(stats.max_abs))
      finite = false;
    else {
      max_abs = std::max(max_abs, stats.max_abs);
      max_rms = std::max(max_rms, stats.rms);
      max_l2 = std::max(max_l2, stats.l2);
    }
  }
  Json against = Json::object();
  if (exact_)
    against["exact"] = text_;
  else
    against["reference"] = reference_->id;
  Json out = Json::object();
  out["job"] = run_.id;
  out["against"] = std::move(against);
  out["nodes"] = run_.mesh->nodes.size();
  out["snapshots"] = std::move(snapshots);
  const auto overall = [&](double value) {
    return any && finite ? Json(value) : Json(nullptr);
  };
  out["max_abs"] = overall(max_abs);
  out["max_rms"] = overall(max_rms);
  out["max_l2"] = overall(max_l2);
  // An adaptive run against an exact solution: the error of every cycle,
  // each on its own mesh (the last is the snapshot above).
  if (exact_ && !run_.cycles.empty()) {
    Json cycles = Json::array();
    for (std::size_t k = 0; k < run_.cycles.size(); ++k) {
      const solve::Solution &cycle = *run_.cycles[k];
      const DifferenceStats stats = difference_stats(
          cycle.values, values_at_nodes(*cycle.mesh, *exact_, cycle.time),
          lumped_weights(*cycle.mesh));
      Json item = Json::object();
      item["cycle"] = k + 1;
      item["nodes"] = cycle.mesh->nodes.size();
      item["max_abs"] = number_or_null(stats.max_abs);
      item["rms"] = number_or_null(stats.rms);
      item["l2"] = number_or_null(stats.l2);
      cycles.push_back(std::move(item));
    }
    out["cycles"] = std::move(cycles);
  }
  return out;
}

std::string Comparison::difference_float32(std::size_t k) const {
  if (k >= run_.snapshots.size())
    throw PathError("snapshot", std::format("snapshot {} is not ready ({} are)",
                                            k, run_.snapshots.size()));
  const auto values = other(k);
  if (!values)
    throw PathError("snapshot",
                    std::format("the reference has no snapshot at t = {}",
                                run_.snapshots[k]->time));
  const std::vector<double> &u = values_of(run_, k);
  std::string out;
  out.reserve(4 * u.size());
  for (std::size_t i = 0; i < u.size(); ++i)
    append_float32(out, u[i] - (*values)[i]);
  return out;
}

PointLocator::PointLocator(const mesh::Mesh &mesh) : mesh_(mesh) {
  if (mesh.nodes.empty() || mesh.triangles.empty())
    return;
  double x1 = mesh.nodes[0].x, y1 = mesh.nodes[0].y;
  x0_ = x1;
  y0_ = y1;
  for (const geometry::Point &p : mesh.nodes) {
    x0_ = std::min(x0_, p.x);
    y0_ = std::min(y0_, p.y);
    x1 = std::max(x1, p.x);
    y1 = std::max(y1, p.y);
  }
  // About one triangle per cell, at most 1024 cells a side.
  const double width = x1 - x0_;
  const double height = y1 - y0_;
  const double side = std::sqrt(std::max(width * height, 0.0) /
                                static_cast<double>(mesh.triangles.size()));
  const auto cells_along = [&](double length) -> std::size_t {
    if (!(side > 0) || !(length > 0))
      return 1;
    return std::clamp<std::size_t>(
        static_cast<std::size_t>(std::ceil(length / side)), 1, 1024);
  };
  columns_ = cells_along(width);
  rows_ = cells_along(height);
  dx_ = width > 0 ? width / static_cast<double>(columns_) : 1.0;
  dy_ = height > 0 ? height / static_cast<double>(rows_) : 1.0;
  x1_ = x1;
  y1_ = y1;
  cells_.assign(columns_ * rows_, {});
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    const auto &nodes = mesh.triangles[t].nodes;
    double lx = mesh.nodes[nodes[0]].x, hx = lx;
    double ly = mesh.nodes[nodes[0]].y, hy = ly;
    for (const std::size_t node : nodes) {
      lx = std::min(lx, mesh.nodes[node].x);
      hx = std::max(hx, mesh.nodes[node].x);
      ly = std::min(ly, mesh.nodes[node].y);
      hy = std::max(hy, mesh.nodes[node].y);
    }
    for (std::size_t j = row(ly); j <= row(hy); ++j)
      for (std::size_t i = column(lx); i <= column(hx); ++i)
        cells_[j * columns_ + i].push_back(t);
  }
}

std::size_t PointLocator::column(double x) const {
  const double cell = std::floor((x - x0_) / dx_);
  return static_cast<std::size_t>(
      std::clamp(cell, 0.0, static_cast<double>(columns_ - 1)));
}

std::size_t PointLocator::row(double y) const {
  const double cell = std::floor((y - y0_) / dy_);
  return static_cast<std::size_t>(
      std::clamp(cell, 0.0, static_cast<double>(rows_ - 1)));
}

std::optional<PointLocator::Found>
PointLocator::locate(geometry::Point p) const {
  if (cells_.empty() || !std::isfinite(p.x) || !std::isfinite(p.y))
    return std::nullopt;
  // Outside the bounding box (up to rounding): outside the mesh.
  const double slack = 1e-12 * std::max({1.0, std::abs(x0_), std::abs(x1_),
                                         std::abs(y0_), std::abs(y1_)});
  if (p.x < x0_ - slack || p.x > x1_ + slack || p.y < y0_ - slack ||
      p.y > y1_ + slack)
    return std::nullopt;
  std::optional<Found> best;
  double best_min = -std::numeric_limits<double>::infinity();
  for (const std::size_t t : cells_[row(p.y) * columns_ + column(p.x)]) {
    const auto &nodes = mesh_.triangles[t].nodes;
    const geometry::Point a = mesh_.nodes[nodes[0]];
    const geometry::Point b = mesh_.nodes[nodes[1]];
    const geometry::Point c = mesh_.nodes[nodes[2]];
    const double area = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    if (area == 0.0)
      continue;
    const double wa =
        ((b.x - p.x) * (c.y - p.y) - (c.x - p.x) * (b.y - p.y)) / area;
    const double wb =
        ((c.x - p.x) * (a.y - p.y) - (a.x - p.x) * (c.y - p.y)) / area;
    const double wc = 1.0 - wa - wb;
    const double smallest = std::min({wa, wb, wc});
    if (smallest > best_min) {
      best_min = smallest;
      best = Found{.triangle = t, .nodes = nodes, .weights = {wa, wb, wc}};
    }
  }
  // Inside, up to rounding on an edge.
  if (!best || best_min < -1e-10)
    return std::nullopt;
  return best;
}

Json probe_series(const RunSnapshots &run, const Json &points) {
  if (!points.is_array())
    throw PathError("points", "expected a list of [x, y]");
  if (points.size() > max_probes)
    throw PathError("points",
                    std::format("at most {} points at once", max_probes));
  // A locator per mesh the snapshots are on (one, except for an adaptive
  // run in time), each in the order of first use.
  std::vector<const mesh::Mesh *> meshes;
  std::vector<std::size_t> mesh_of; // per snapshot
  for (const Snapshot &solution : run.snapshots) {
    const auto found = std::ranges::find(meshes, solution->mesh.get());
    mesh_of.push_back(static_cast<std::size_t>(found - meshes.begin()));
    if (found == meshes.end())
      meshes.push_back(solution->mesh.get());
  }
  if (meshes.empty())
    meshes.push_back(run.mesh.get());
  std::vector<PointLocator> locators;
  locators.reserve(meshes.size());
  for (const mesh::Mesh *mesh : meshes)
    locators.emplace_back(*mesh);
  Json probes = Json::array();
  for (std::size_t k = 0; k < points.size(); ++k) {
    const std::string where = std::format("points[{}]", k);
    const Json &point = points[k];
    if (!point.is_array() || point.size() != 2)
      throw PathError(where, "expected [x, y]");
    const geometry::Point p{require_number(point[0], where + "[0]"),
                            require_number(point[1], where + "[1]")};
    Json item = Json::object();
    item["x"] = p.x;
    item["y"] = p.y;
    std::vector<std::optional<PointLocator::Found>> found;
    for (const PointLocator &locator : locators)
      found.push_back(locator.locate(p));
    // Inside the last mesh's triangle (meshes only grow by refinement, so
    // a point is inside all of them or none, up to rounding).
    const auto &last = found[mesh_of.empty() ? 0 : mesh_of.back()];
    item["inside"] = last.has_value();
    item["triangle"] = last ? Json(last->triangle) : Json(nullptr);
    Json values = Json::array();
    if (last)
      for (std::size_t k = 0; k < run.snapshots.size(); ++k) {
        const auto &at = found[mesh_of[k]];
        if (!at) {
          values.push_back(nullptr);
          continue;
        }
        double value = 0.0;
        for (int v = 0; v < 3; ++v)
          value +=
              at->weights[static_cast<std::size_t>(v)] *
              run.snapshots[k]->values[at->nodes[static_cast<std::size_t>(v)]];
        values.push_back(number_or_null(value));
      }
    item["values"] = std::move(values);
    probes.push_back(std::move(item));
  }
  Json times = Json::array();
  for (const Snapshot &solution : run.snapshots)
    times.push_back(solution->time);
  Json out = Json::object();
  out["job"] = run.id;
  out["times"] = std::move(times);
  out["probes"] = std::move(probes);
  return out;
}

} // namespace pde2d::app

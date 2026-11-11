#include "pde2d/geometry/curve_geometry.hpp"

#include "pde2d/geometry/polygon.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>

namespace pde2d::geometry {

bool is_valid_region_name(std::string_view name) {
  return !name.empty() && name != "-" &&
         std::all_of(name.begin(), name.end(), [](unsigned char c) {
           return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '.' || c == '-';
         });
}

namespace {

std::string where(std::size_t loop, std::size_t curve) {
  return "Loop " + std::to_string(loop) + ", curve " + std::to_string(curve);
}

std::string format_point(Point p) {
  std::ostringstream s;
  s.precision(17);
  s << '(' << p.x << ", " << p.y << ')';
  return s.str();
}

void check_region_name(const std::string &name, const std::string &context) {
  if (!is_valid_region_name(name))
    throw std::invalid_argument(
        context + ": region name '" + name +
        "' is invalid; use ASCII letters, digits, '_', '.' or '-' (a lone "
        "'-' is reserved)");
}

// Size of the geometry, from the bounding box of sample points.
double geometry_scale(const std::vector<Loop> &loops) {
  double xmin = INFINITY, xmax = -INFINITY, ymin = INFINITY, ymax = -INFINITY;
  for (const auto &loop : loops)
    for (const auto &c : loop.curves)
      for (int k = 0; k <= 64; ++k) {
        Point p = point_at(c.curve, k / 64.0);
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
      }
  double scale = std::hypot(xmax - xmin, ymax - ymin);
  if (!std::isfinite(scale) || !(scale > 0))
    throw std::invalid_argument("Geometry has no extent or is too large");
  return scale;
}

// A polyline through points of the curve whose chords deviate from the
// curve by at most `tolerance` (from the second-derivative bound). The end
// point is not included: the next curve starts there.
void flatten(const Curve &curve, double tolerance, std::vector<Point> &out) {
  auto emit = [&](auto &&self, double a, double b, int depth) -> void {
    Curve piece = subcurve(curve, a, b);
    double deviation = derivative_bounds(piece).second / 8;
    bool split = depth < 2 || (!is_line(curve) && deviation > tolerance);
    if (split) {
      if (depth >= 24)
        throw std::invalid_argument(
            "Curve is too detailed to validate at this geometry scale");
      double m = (a + b) / 2;
      self(self, a, m, depth + 1);
      self(self, m, b, depth + 1);
    } else {
      out.push_back(point_at(curve, a));
    }
  };
  emit(emit, 0.0, 1.0, 0);
}

struct Analysis {
  std::vector<std::size_t> depth;
  std::vector<std::optional<std::size_t>> parent;
  std::vector<bool> counterclockwise;
};

Analysis analyze(const std::vector<Loop> &loops) {
  if (loops.empty())
    throw std::invalid_argument("CurveGeometry needs at least one loop");
  for (std::size_t l = 0; l < loops.size(); ++l) {
    if (loops[l].curves.empty())
      throw std::invalid_argument("Loop " + std::to_string(l) +
                                  " has no curves");
    for (std::size_t c = 0; c < loops[l].curves.size(); ++c) {
      const auto &bc = loops[l].curves[c];
      try {
        validate(bc.curve);
      } catch (const std::invalid_argument &e) {
        throw std::invalid_argument(where(l, c) + ": " + e.what());
      }
      if (derivative_bounds(bc.curve).first == 0)
        throw std::invalid_argument(where(l, c) + " has zero length");
      check_region_name(bc.region, where(l, c));
    }
  }
  const double scale = geometry_scale(loops);
  const double closure_tolerance = 1e-9 * scale;
  for (std::size_t l = 0; l < loops.size(); ++l) {
    const auto &curves = loops[l].curves;
    for (std::size_t c = 0; c < curves.size(); ++c) {
      std::size_t next = (c + 1) % curves.size();
      Point end = end_point(curves[c].curve);
      Point start = start_point(curves[next].curve);
      if (distance(end, start) > closure_tolerance)
        throw std::invalid_argument(
            "Loop " + std::to_string(l) + " is not closed: curve " +
            std::to_string(c) + " ends at " + format_point(end) +
            " but curve " + std::to_string(next) + " starts at " +
            format_point(start));
    }
  }
  // Polylines close to the curves decide intersections, containment and
  // direction with exact predicates.
  const double flat_tolerance = 1e-6 * scale;
  std::vector<std::vector<Point>> polylines(loops.size());
  std::vector<std::vector<std::size_t>> curve_of(loops.size());
  for (std::size_t l = 0; l < loops.size(); ++l)
    for (std::size_t c = 0; c < loops[l].curves.size(); ++c) {
      flatten(loops[l].curves[c].curve, flat_tolerance, polylines[l]);
      curve_of[l].resize(polylines[l].size(), c);
    }
  if (auto crossing = find_crossing(polylines)) {
    auto a = crossing->first, b = crossing->second;
    if (b.loop < a.loop)
      std::swap(a, b);
    std::size_t ca = curve_of[a.loop][a.segment];
    std::size_t cb = curve_of[b.loop][b.segment];
    if (a.loop == b.loop)
      throw std::invalid_argument("Loop " + std::to_string(a.loop) +
                                  " intersects or touches itself near curves " +
                                  std::to_string(ca) + " and " +
                                  std::to_string(cb));
    throw std::invalid_argument("Loops " + std::to_string(a.loop) + " and " +
                                std::to_string(b.loop) +
                                " intersect or touch (" + where(a.loop, ca) +
                                " and " + where(b.loop, cb) + ")");
  }
  Analysis result;
  const std::size_t n = loops.size();
  std::vector<std::vector<bool>> inside(n, std::vector<bool>(n, false));
  result.depth.assign(n, 0);
  for (std::size_t i = 0; i < n; ++i)
    for (std::size_t j = 0; j < n; ++j)
      if (i != j && locate_in_polygon(polylines[j], polylines[i].front()) > 0) {
        inside[i][j] = true;
        ++result.depth[i];
      }
  result.parent.assign(n, std::nullopt);
  for (std::size_t i = 0; i < n; ++i)
    for (std::size_t j = 0; j < n; ++j)
      if (inside[i][j] && result.depth[j] + 1 == result.depth[i])
        result.parent[i] = j;
  for (std::size_t i = 0; i < n; ++i)
    result.counterclockwise.push_back(signed_area(polylines[i]) > 0);
  return result;
}

Loop reversed_loop(const Loop &loop) {
  Loop r;
  for (auto it = loop.curves.rbegin(); it != loop.curves.rend(); ++it)
    r.curves.push_back({reversed(it->curve), it->region});
  return r;
}

bool has_region(const std::vector<Loop> &loops, const std::string &name) {
  for (const auto &loop : loops)
    for (const auto &c : loop.curves)
      if (c.region == name)
        return true;
  return false;
}

void require_region(const std::vector<Loop> &loops, const std::string &name) {
  if (!has_region(loops, name))
    throw std::invalid_argument("Unknown region '" + name + "'");
}

} // namespace

CurveGeometry::CurveGeometry(std::vector<Loop> loops) {
  Analysis a = analyze(loops);
  for (std::size_t l = 0; l < loops.size(); ++l) {
    bool want_ccw = a.depth[l] % 2 == 0;
    if (a.counterclockwise[l] != want_ccw)
      loops[l] = reversed_loop(loops[l]);
  }
  loops_ = std::move(loops);
  depth_ = std::move(a.depth);
  parent_ = std::move(a.parent);
}

std::size_t CurveGeometry::depth(std::size_t loop) const {
  return depth_.at(loop);
}

std::optional<std::size_t> CurveGeometry::parent(std::size_t loop) const {
  return parent_.at(loop);
}

std::vector<std::string> CurveGeometry::region_names() const {
  std::vector<std::string> names;
  std::set<std::string> seen;
  for (const auto &loop : loops_)
    for (const auto &c : loop.curves)
      if (seen.insert(c.region).second)
        names.push_back(c.region);
  return names;
}

double CurveGeometry::region_length(const std::string &region) const {
  require_region(loops_, region);
  double total = 0;
  for (const auto &loop : loops_)
    for (const auto &c : loop.curves)
      if (c.region == region)
        total += curve_length(c.curve);
  return total;
}

void CurveGeometry::rename_region(const std::string &from,
                                  const std::string &to) {
  require_region(loops_, from);
  check_region_name(to, "rename_region");
  if (from == to)
    return;
  if (has_region(loops_, to))
    throw std::invalid_argument("rename_region: region '" + to +
                                "' already exists; use merge_region_into");
  for (auto &loop : loops_)
    for (auto &c : loop.curves)
      if (c.region == from)
        c.region = to;
}

void CurveGeometry::merge_region_into(const std::string &source,
                                      const std::string &target) {
  require_region(loops_, source);
  require_region(loops_, target);
  if (source == target)
    throw std::invalid_argument("merge_region_into: region '" + source +
                                "' cannot be merged into itself");
  for (auto &loop : loops_)
    for (auto &c : loop.curves)
      if (c.region == source)
        c.region = target;
}

void CurveGeometry::split_region(const std::string &region, double fraction,
                                 const std::string &first,
                                 const std::string &second) {
  require_region(loops_, region);
  if (!(fraction > 0 && fraction < 1))
    throw std::invalid_argument(
        "split_region: fraction must be strictly between 0 and 1");
  check_region_name(first, "split_region");
  check_region_name(second, "split_region");
  if (first == second)
    throw std::invalid_argument("split_region: the two names must differ");
  for (const auto &name : {first, second})
    if (name != region && has_region(loops_, name))
      throw std::invalid_argument("split_region: region '" + name +
                                  "' already exists");

  // Find the one loop holding the region and the start of its run.
  std::optional<std::size_t> loop_index;
  std::size_t total_count = 0;
  for (std::size_t l = 0; l < loops_.size(); ++l)
    for (const auto &c : loops_[l].curves)
      if (c.region == region) {
        ++total_count;
        if (loop_index && *loop_index != l)
          throw std::invalid_argument(
              "split_region: region '" + region +
              "' lies on several loops; split needs one connected piece");
        loop_index = l;
      }
  Loop loop = loops_[*loop_index];
  const std::size_t n = loop.curves.size();
  std::size_t start = 0;
  if (total_count < n)
    for (std::size_t i = 0; i < n; ++i)
      if (loop.curves[i].region == region &&
          loop.curves[(i + n - 1) % n].region != region) {
        start = i;
        break;
      }
  std::size_t run = 0;
  while (run < n && loop.curves[(start + run) % n].region == region)
    ++run;
  if (run != total_count)
    throw std::invalid_argument(
        "split_region: region '" + region +
        "' has several separate pieces; split needs one connected piece");

  // Rotate the loop so the run starts at index 0 (rotation keeps the loop
  // closed and its direction), then split by length.
  std::rotate(loop.curves.begin(),
              loop.curves.begin() + static_cast<std::ptrdiff_t>(start),
              loop.curves.end());
  std::vector<double> lengths;
  double total = 0;
  for (std::size_t i = 0; i < run; ++i) {
    lengths.push_back(curve_length(loop.curves[i].curve));
    total += lengths.back();
  }
  const double target = fraction * total;
  const double junction = 1e-9 * total;
  double before = 0;
  std::size_t first_count = run; // curves [0, first_count) get `first`
  std::optional<std::size_t> inserted_at;
  for (std::size_t i = 0; i < run; ++i) {
    double local = target - before;
    if (local <= junction) {
      first_count = i;
      break;
    }
    if (local < lengths[i] - junction) {
      double t = parameter_at_length_fraction(loop.curves[i].curve,
                                              local / lengths[i]);
      auto [a, b] = split(loop.curves[i].curve, t);
      loop.curves[i].curve = a;
      loop.curves.insert(loop.curves.begin() +
                             static_cast<std::ptrdiff_t>(i + 1),
                         BoundaryCurve{b, region});
      inserted_at = i + 1;
      ++run;
      first_count = i + 1;
      break;
    }
    before += lengths[i];
  }
  if (first_count == 0 || first_count >= run)
    throw std::invalid_argument(
        "split_region: the split point coincides with an end of the region");
  for (std::size_t i = 0; i < run; ++i)
    loop.curves[i].region = i < first_count ? first : second;
  // Undo the rotation so the loop's first curve (or its first half, if it
  // was split) stays first.
  std::size_t first_curve = (n - start) % n;
  if (inserted_at && *inserted_at <= first_curve)
    ++first_curve;
  std::rotate(loop.curves.begin(),
              loop.curves.begin() + static_cast<std::ptrdiff_t>(first_curve),
              loop.curves.end());
  loops_[*loop_index] = std::move(loop);
}

CurveGeometry
CurveGeometry::transformed(const CurveTransform &transform) const {
  std::vector<Loop> loops = loops_;
  for (auto &loop : loops)
    for (auto &c : loop.curves)
      c.curve = geometry::transformed(c.curve, transform);
  return CurveGeometry(std::move(loops));
}

} // namespace pde2d::geometry

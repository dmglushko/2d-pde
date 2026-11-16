#pragma once

#include "options.hpp"
#include "runs.hpp"

#include "pde2d/fem/compiled_expression.hpp"
#include "pde2d/geometry/point.hpp"
#include "pde2d/mesh/mesh.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <vector>

// Values on a mesh computed in the app from the library's results: an
// expression at the nodes (evaluated by fem::CompiledExpression, the only
// evaluator), differences between a run and an exact solution or a
// reference run, and the solution at points (docs/app.md, "Comparisons"
// and "Probes"). They read a run's RunSnapshots (the mesh and the values),
// so probes need no lock; a comparison with an exact solution compiles an
// expression, which needs the session mutex.
namespace pde2d::app {

// The compiled expression at every node of `mesh` at time t.
std::vector<double> values_at_nodes(const mesh::Mesh &mesh,
                                    const fem::CompiledExpression &expression,
                                    double t);

// The lumped mass of each node: a third of the area of every triangle
// around it. sqrt(sum w d^2) is the L2 norm of the P1 function d with the
// lumped (nodal) quadrature.
std::vector<double> lumped_weights(const mesh::Mesh &mesh);

// The difference u - other of one snapshot: its largest |value|, its
// nodal root mean square and its lumped L2 norm (lumped_weights). A node
// where either value is not finite makes all three NaN.
struct DifferenceStats {
  double max_abs = 0.0;
  double rms = 0.0;
  double l2 = 0.0;
};
DifferenceStats difference_stats(std::span<const double> u,
                                 std::span<const double> other,
                                 std::span<const double> weights);

// What a run is compared with: an exact solution (an expression in x, y,
// t and the run's parameters) or another run on the same mesh object.
class Comparison {
public:
  // Reads {"exact": text} or {"reference": true} from `request` (the keys
  // besides "job", "snapshot"); `parameters` are the run's; `reference` is
  // the client's reference run (may be null). Throws PathError for a
  // request it cannot follow.
  Comparison(RunSnapshots run, const fem::Parameters &parameters,
             const Json &request, std::optional<RunSnapshots> reference);

  // The other values for snapshot k of the run, or nothing when the
  // reference has no snapshot at that time.
  std::optional<std::vector<double>> other(std::size_t k) const;
  // {"against", "nodes", "snapshots": [{"t", "max_abs", "rms", "l2"} or
  // null], "max_abs", "max_rms", "max_l2"} over every ready snapshot.
  Json summary() const;
  // u - other at snapshot k, as float32 little-endian.
  std::string difference_float32(std::size_t k) const;

private:
  RunSnapshots run_;
  std::optional<fem::CompiledExpression> exact_;
  std::string text_;
  std::optional<RunSnapshots> reference_;
  // For each run snapshot, the reference snapshot at the same time.
  std::vector<std::optional<std::size_t>> matched_;
};

// Finds the triangle of a point: a uniform grid over the mesh's bounding
// box, each cell listing the triangles whose boxes overlap it.
class PointLocator {
public:
  explicit PointLocator(const mesh::Mesh &mesh);
  struct Found {
    std::size_t triangle = 0;
    std::array<std::size_t, 3> nodes{};
    std::array<double, 3> weights{}; // barycentric, sum 1
  };
  // The triangle that holds p (on an edge: the one with the largest
  // smallest weight), or nothing outside the mesh.
  std::optional<Found> locate(geometry::Point p) const;

private:
  std::size_t column(double x) const;
  std::size_t row(double y) const;

  const mesh::Mesh &mesh_;
  double x0_ = 0.0, y0_ = 0.0, x1_ = 0.0, y1_ = 0.0, dx_ = 1.0, dy_ = 1.0;
  std::size_t columns_ = 1, rows_ = 1;
  std::vector<std::vector<std::size_t>> cells_;
};

// u(t) of the run at each point of {"points": [[x, y], ...]} (at most
// max_probes), interpolated with the P1 basis: {"times", "probes": [{"x",
// "y", "inside", "triangle", "values"}]}. Throws PathError for a bad
// request.
Json probe_series(const RunSnapshots &run, const Json &points);
inline constexpr std::size_t max_probes = 64;

} // namespace pde2d::app

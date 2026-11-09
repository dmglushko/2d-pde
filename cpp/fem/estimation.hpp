#pragma once
// A posteriori error estimation seam for adaptivity: ErrorEstimator returns
// one indicator per element plus a global estimate; mark_bulk() turns the
// indicators into element indices for
// discretization::refine_mesh(result, marked, options).
//
// Built-in: GradientRecoveryEstimator (Zienkiewicz-Zhu). Assumptions:
// * it estimates the H1 seminorm of the P1 error, |u - u_h|_{H1}; it does
//   not weight by the diffusion tensor and ignores the equation, boundary
//   conditions and time (they are part of the interface for residual
//   estimators);
// * the recovered gradient G is the area-weighted average of the
//   neighbouring element gradients at every node, interpolated linearly;
//   eta_T = ||G - grad u_h||_{L2(T)} is integrated exactly;
// * it is reliable for smooth solutions on shape-regular meshes (effectivity
//   near 1, asymptotically exact under superconvergence). It is weaker at
//   boundary nodes, where averaging is one-sided, and can underestimate
//   errors at singularities or in unresolved layers.
#include "discretization/mesh_validation.hpp"
#include "execution.hpp"
#include "fem/p1_element.hpp"
#include "fem/problem.hpp"
#include "fem/system.hpp"
#include "provenance.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::fem {
struct ErrorEstimate {
  // eta_T >= 0 for every element, in mesh.elements order.
  std::vector<double> indicators;
  // sqrt(sum eta_T^2).
  double global = 0;
  std::string estimator;
};

class ErrorEstimator {
public:
  virtual ~ErrorEstimator() = default;
  virtual std::string name() const = 0;
  // Stable description (name + parameters) for records of an adaptive loop.
  // Without one the estimator is "not replayable" from a record.
  virtual std::optional<pde2d::StrategyDescriptor> descriptor() const {
    return std::nullopt;
  }
  // `solution` holds one nodal value per mesh node at `time`.
  virtual ErrorEstimate estimate(const discretization::TriangleMesh &mesh,
                                 const Problem &problem, const Vector &solution,
                                 double time) const = 0;
};

class GradientRecoveryEstimator final : public ErrorEstimator {
public:
  pde2d::ExecutionOptions execution{};
  GradientRecoveryEstimator() = default;
  explicit GradientRecoveryEstimator(pde2d::ExecutionOptions options)
      : execution(options) {}
  std::string name() const override { return "gradient recovery (ZZ)"; }
  // Execution does not change the result (serial/OpenMP bit-identical).
  std::optional<pde2d::StrategyDescriptor> descriptor() const override {
    return pde2d::StrategyDescriptor{"GradientRecovery"};
  }
  ErrorEstimate estimate(const discretization::TriangleMesh &mesh,
                         const Problem &, const Vector &solution,
                         double time) const override {
    discretization::validate_mesh_geometry(mesh);
    if (solution.size() != static_cast<Eigen::Index>(mesh.nodes.size()) ||
        !solution.allFinite() || !std::isfinite(time))
      throw std::invalid_argument(
          "Error estimation needs one finite value per node and a finite "
          "time");
    const auto n = mesh.elements.size();
    struct Local {
      double area, gx, gy;
    };
    std::vector<Local> local(n);
    pde2d::parallel_for(n, execution, [&](std::size_t i) {
      const auto &t = mesh.elements[i];
      const auto g = p1::triangle_geometry(mesh.nodes[t[0]], mesh.nodes[t[1]],
                                           mesh.nodes[t[2]]);
      double gx = 0, gy = 0;
      for (int j = 0; j < 3; ++j) {
        gx += g.gx[j] * solution[static_cast<Eigen::Index>(t[j])];
        gy += g.gy[j] * solution[static_cast<Eigen::Index>(t[j])];
      }
      local[i] = {0.5 * g.twice_area, gx, gy};
    });
    // Serial, element-order accumulation keeps the result deterministic.
    std::vector<std::array<double, 3>> recovered(mesh.nodes.size(), {0, 0, 0});
    for (std::size_t i = 0; i < n; ++i)
      for (auto v : mesh.elements[i]) {
        recovered[v][0] += local[i].area * local[i].gx;
        recovered[v][1] += local[i].area * local[i].gy;
        recovered[v][2] += local[i].area;
      }
    ErrorEstimate result;
    result.estimator = name();
    result.indicators.resize(n);
    pde2d::parallel_for(n, execution, [&](std::size_t i) {
      // For d = sum_j lambda_j d_j: int_T |d|^2 = |T|/12 (sum|d_j|^2 +
      // |sum d_j|^2), since int_T lambda_i lambda_j = |T| (1 + delta_ij)/12.
      double squares = 0, sx = 0, sy = 0;
      for (auto v : mesh.elements[i]) {
        const double dx = recovered[v][0] / recovered[v][2] - local[i].gx,
                     dy = recovered[v][1] / recovered[v][2] - local[i].gy;
        squares += dx * dx + dy * dy;
        sx += dx;
        sy += dy;
      }
      result.indicators[i] =
          std::sqrt(local[i].area / 12 * (squares + sx * sx + sy * sy));
    });
    double sum = 0;
    for (auto eta : result.indicators)
      sum += eta * eta;
    result.global = std::sqrt(sum);
    return result;
  }
};

inline ErrorEstimate
estimate_error(const discretization::TriangleMesh &mesh, const Problem &problem,
               const Vector &solution, double time = 0,
               const ErrorEstimator &estimator = GradientRecoveryEstimator{}) {
  auto estimate = estimator.estimate(mesh, problem, solution, time);
  if (estimate.indicators.size() != mesh.elements.size() ||
      std::any_of(estimate.indicators.begin(), estimate.indicators.end(),
                  [](double eta) { return !std::isfinite(eta) || eta < 0; }) ||
      !std::isfinite(estimate.global) || estimate.global < 0)
    throw std::runtime_error("Error estimator '" + estimator.name() +
                             "' returned invalid indicators");
  return estimate;
}

// Dorfler (bulk) marking: the smallest set of elements, taken in decreasing
// indicator order (ties: lower index first), whose squared indicators sum to
// at least `fraction` of the total. Returns ascending element indices.
// fraction must be in (0, 1]; all-zero indicators mark nothing.
inline std::vector<std::size_t> mark_bulk(const std::vector<double> &indicators,
                                          double fraction) {
  if (!(fraction > 0 && fraction <= 1))
    throw std::invalid_argument("Bulk marking fraction must be in (0, 1]");
  double total = 0;
  for (auto eta : indicators) {
    if (!std::isfinite(eta) || eta < 0)
      throw std::invalid_argument(
          "Error indicators must be finite and nonnegative");
    total += eta * eta;
  }
  std::vector<std::size_t> order(indicators.size());
  std::iota(order.begin(), order.end(), std::size_t{0});
  std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) {
    return indicators[a] > indicators[b];
  });
  std::vector<std::size_t> marked;
  if (!(total > 0))
    return marked;
  double sum = 0;
  for (auto i : order) {
    if (sum >= fraction * total)
      break;
    marked.push_back(i);
    sum += indicators[i] * indicators[i];
  }
  std::sort(marked.begin(), marked.end());
  return marked;
}
} // namespace pde2d::fem

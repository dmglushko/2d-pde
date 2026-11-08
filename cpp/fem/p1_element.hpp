#pragma once
// Linear (P1) triangle and boundary-edge geometry plus the local numeric
// kernels of the weak form. Everything here is pure arithmetic: no symbolic
// expressions, no mesh-wide state. Assembly (assembly.hpp) prepares inputs,
// calls these kernels per quadrature point and scatters the local blocks.
//
// Adding a different element type means adding a sibling of this header and
// teaching assembly which one to use; the weak-form coefficients
// (weak_form.hpp) and boundary classification are element independent.
#include "discretization/geometry_base.hpp"
#include "weak_form.hpp"
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace pde2d::fem {
namespace p1 {
inline double checked(double v) {
  if (!std::isfinite(v))
    throw std::domain_error("Nonfinite FEM assembly contribution");
  return v;
}

// Local 3x3 matrices (row = test function, column = trial function) and load.
// Boundary edges reuse the storage with only the leading 2x2 block in use.
struct LocalBlock {
  static constexpr std::size_t nodes = 3;
  std::array<double, nodes * nodes> stiffness{}, first{}, second{};
  std::array<double, nodes> load{};
  // Row-major position of (test i, trial j) in the matrix arrays.
  static constexpr std::size_t index(std::size_t i, std::size_t j) {
    return nodes * i + j;
  }
};

// Constant gradients of the three barycentric basis functions.
struct TriangleGeometry {
  double twice_area;
  std::array<double, 3> gx, gy;
};
inline TriangleGeometry triangle_geometry(discretization::Point2 a,
                                          discretization::Point2 b,
                                          discretization::Point2 c) {
  const double twice_area =
      checked((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
  if (twice_area <= 0)
    throw std::invalid_argument("Unrepresentable triangle area");
  return {twice_area,
          {(b.y - c.y) / twice_area, (c.y - a.y) / twice_area,
           (a.y - b.y) / twice_area},
          {(c.x - b.x) / twice_area, (a.x - c.x) / twice_area,
           (b.x - a.x) / twice_area}};
}

// A straight boundary edge from a to b with the unit normal pointing away
// from the opposite interior vertex.
struct EdgeFrame {
  discretization::Point2 a, b;
  double length, normal_x, normal_y;
};
inline EdgeFrame edge_frame(discretization::Point2 a, discretization::Point2 b,
                            discretization::Point2 inside) {
  const double dx = b.x - a.x, dy = b.y - a.y,
               length = checked(std::hypot(dx, dy));
  if (length == 0)
    throw std::invalid_argument("Zero boundary edge length");
  double normal_x = dy / length, normal_y = -dx / length;
  if (normal_x * (inside.x - a.x) + normal_y * (inside.y - a.y) > 0) {
    normal_x = -normal_x;
    normal_y = -normal_y;
  }
  return {a, b, length, normal_x, normal_y};
}

// Which local matrices a set of enabled coefficients can touch.
struct ActiveBlocks {
  bool stiffness, first, second, load;
};
inline ActiveBlocks active_blocks(const WeakFlags &enabled) {
  bool stiffness = false;
  for (std::size_t k = weak::DiffusionXX; k <= weak::Reaction; ++k)
    stiffness = stiffness || enabled[k];
  return {stiffness, enabled[weak::FirstMass], enabled[weak::SecondMass],
          enabled[weak::Source]};
}

// Add one interior quadrature point with weight w (area already included).
inline void accumulate_element_point(LocalBlock &out,
                                     const WeakCoefficients<double> &v,
                                     const std::array<double, 3> &phi,
                                     const TriangleGeometry &g, double w,
                                     ActiveBlocks active) {
  using namespace weak;
  const auto &gx = g.gx;
  const auto &gy = g.gy;
  for (int i = 0; i < 3; ++i) {
    if (active.load)
      out.load[i] = checked(out.load[i] + w * v[Source] * phi[i]);
    for (int j = 0; j < 3; ++j) {
      int ij = 3 * i + j;
      if (active.stiffness)
        out.stiffness[ij] =
            checked(out.stiffness[ij] +
                    w * (-v[DiffusionXX] * gx[i] * gx[j] -
                         v[DiffusionXY] * (gx[i] * gy[j] + gy[i] * gx[j]) -
                         v[DiffusionYY] * gy[i] * gy[j] +
                         phi[i] * (v[DriftX] * gx[j] + v[DriftY] * gy[j]) +
                         v[Reaction] * phi[i] * phi[j]));
      if (active.first)
        out.first[ij] =
            checked(out.first[ij] + w * v[FirstMass] * phi[i] * phi[j]);
      if (active.second)
        out.second[ij] =
            checked(out.second[ij] + w * v[SecondMass] * phi[i] * phi[j]);
    }
  }
}

// Add one natural-boundary quadrature point at edge coordinate s in [0,1].
// Inputs are already divided by the flux multiplier beta: alpha/beta adds to
// stiffness, gamma/beta to the first mass and value/beta to the load.
inline void accumulate_edge_point(LocalBlock &out, double s, double w,
                                  double alpha, double gamma, double value) {
  const std::array<double, 2> phi{1 - s, s};
  for (int i = 0; i < 2; ++i) {
    out.load[i] = checked(out.load[i] + w * value * phi[i]);
    for (int j = 0; j < 2; ++j) {
      out.stiffness[3 * i + j] =
          checked(out.stiffness[3 * i + j] + w * alpha * phi[i] * phi[j]);
      out.first[3 * i + j] =
          checked(out.first[3 * i + j] + w * gamma * phi[i] * phi[j]);
    }
  }
}
} // namespace p1
} // namespace pde2d::fem

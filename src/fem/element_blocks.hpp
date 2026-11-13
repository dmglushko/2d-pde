#pragma once

// Private: the element blocks of assembly. Computing a block is a pure
// function of the element, the compiled problem and t (no shared state, so
// a later slice may call it from several threads); scattering is done by
// the caller (assembler.cpp).

#include "pde2d/fem/meshed_problem.hpp"
#include "pde2d/geometry/point.hpp"

#include <array>
#include <cstddef>

namespace pde2d::fem::detail {

// Where a term's element block goes: not computed (zero as written, or a
// constant term reused from an earlier assembly), into the constant part
// of its operator, or into its time-dependent part.
enum class Target { skip, constant, time };

// The terms of a triangle: K, C, R (stiffness), M1 (damping), M2 (mass), F
// (load).
struct TriangleTerms {
  Target k = Target::skip, c = Target::skip, r = Target::skip;
  Target m1 = Target::skip, m2 = Target::skip, f = Target::skip;
};

// The terms of a boundary edge of a natural or dynamic condition: S, T
// (stiffness), D (damping), G (load).
struct EdgeTerms {
  Target s = Target::skip, t = Target::skip, d = Target::skip, g = Target::skip;
};

// One element's contribution to each operator and the load, for N nodes.
// Entry [i][j] belongs to row node i and column node j of the element.
template <std::size_t N> struct OperatorBlocks {
  using Block = std::array<std::array<double, N>, N>;
  Block mass{}, damping{}, stiffness{};
  std::array<double, N> load{};
};

// The blocks of the terms marked `constant` and of those marked `time`,
// each summed per operator with the signs of the system
// (damping M1 - D, stiffness -K + C + R - S + T, load F - G), always
// adding the terms in the order K, C, R, M1, M2, F and S, T, D, G.
template <std::size_t N> struct ElementBlocks {
  OperatorBlocks<N> constant{}, time{};
};

// The blocks of a counterclockwise triangle with the 3-point rule at the
// points with barycentric coordinates (2/3, 1/6, 1/6) and their
// rotations, weight area / 3 each (exact for degree 2). Throws
// std::invalid_argument when a coefficient a term needs is not finite at a
// quadrature point.
ElementBlocks<3> triangle_blocks(const MeshedProblem &problem,
                                 const std::array<geometry::Point, 3> &nodes,
                                 double t, const TriangleTerms &terms);

// The blocks of a boundary edge from node a to node b (the domain on its
// left) of region `region`, whose condition is natural or dynamic, with
// 2-point Gauss-Legendre. Node indices are only for messages. Throws
// std::invalid_argument for a coefficient that is not finite at a
// quadrature point, and when beta_n + (beta_x, beta_y).n is zero there.
ElementBlocks<2> edge_blocks(const MeshedProblem &problem, std::size_t region,
                             const std::array<std::size_t, 2> &nodes, double t,
                             const EdgeTerms &terms);

} // namespace pde2d::fem::detail

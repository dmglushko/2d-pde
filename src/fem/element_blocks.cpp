#include "element_blocks.hpp"

#include <cmath>
#include <format>
#include <stdexcept>
#include <string>

namespace pde2d::fem::detail {

namespace {

using geometry::Point;

// A coefficient at a point: 0 without a call when it is zero as written;
// a value that is not finite throws, naming the coefficient.
template <class Label>
double value(const CompiledExpression &coefficient, const Label &label,
             const Point &p, double t) {
  if (coefficient.facts().zero)
    return 0.0;
  const double v = coefficient(p.x, p.y, t);
  if (!std::isfinite(v))
    throw std::invalid_argument(std::format("{} is not finite at ({}, {}), "
                                            "t = {}: {}",
                                            label(), p.x, p.y, t, v));
  return v;
}

template <std::size_t N>
OperatorBlocks<N> *destination(ElementBlocks<N> &blocks, Target target) {
  switch (target) {
  case Target::constant:
    return &blocks.constant;
  case Target::time:
    return &blocks.time;
  case Target::skip:
    break;
  }
  return nullptr;
}

template <std::size_t N>
void add(std::array<std::array<double, N>, N> &target, double sign,
         const std::array<std::array<double, N>, N> &block) {
  for (std::size_t i = 0; i < N; ++i)
    for (std::size_t j = 0; j < N; ++j)
      target[i][j] += sign * block[i][j];
}

using Block3 = std::array<std::array<double, 3>, 3>;
using Block2 = std::array<std::array<double, 2>, 2>;

} // namespace

ElementBlocks<3> triangle_blocks(const MeshedProblem &problem,
                                 const std::array<Point, 3> &nodes, double t,
                                 const TriangleTerms &terms) {
  ElementBlocks<3> out;
  const auto &e = problem.equation();
  const auto &divergence = problem.divergence_of_a();
  const auto equation_label = [](const char *name) {
    return [name] { return std::string("Equation coefficient ") + name; };
  };

  // Geometry: twice the area and the gradients of the barycentric
  // coordinates (constant on the triangle).
  const auto &[p0, p1, p2] = nodes;
  const double det =
      (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
  const double area = det / 2.0;
  std::array<Point, 3> gradient{};
  for (std::size_t i = 0; i < 3; ++i) {
    const Point &pj = nodes[(i + 1) % 3], &pk = nodes[(i + 2) % 3];
    gradient[i] = {(pj.y - pk.y) / det, (pk.x - pj.x) / det};
  }
  // Quadrature: barycentric (2/3, 1/6, 1/6) and its rotations.
  std::array<std::array<double, 3>, 3> lambda{};
  std::array<Point, 3> point{};
  for (std::size_t q = 0; q < 3; ++q) {
    for (std::size_t i = 0; i < 3; ++i)
      lambda[q][i] = q == i ? 2.0 / 3.0 : 1.0 / 6.0;
    point[q] = {lambda[q][0] * p0.x + lambda[q][1] * p1.x + lambda[q][2] * p2.x,
                lambda[q][0] * p0.y + lambda[q][1] * p1.y +
                    lambda[q][2] * p2.y};
  }
  const double weight = area / 3.0;

  // K: integral of grad phi_i . A grad phi_j, A = [[a_xx, a_xy/2],
  // [a_xy/2, a_yy]].
  if (auto *d = destination(out, terms.k)) {
    Block3 k{};
    for (std::size_t q = 0; q < 3; ++q) {
      const double xx = value(e.a_xx, equation_label("a_xx"), point[q], t);
      const double xy = value(e.a_xy, equation_label("a_xy"), point[q], t) / 2;
      const double yy = value(e.a_yy, equation_label("a_yy"), point[q], t);
      for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) {
          const Point &gi = gradient[i], &gj = gradient[j];
          k[i][j] += weight * (gi.x * (xx * gj.x + xy * gj.y) +
                               gi.y * (xy * gj.x + yy * gj.y));
        }
    }
    add(d->stiffness, -1.0, k);
  }
  // C: integral of (b - div A) . grad phi_j phi_i.
  if (auto *d = destination(out, terms.c)) {
    Block3 c{};
    for (std::size_t q = 0; q < 3; ++q) {
      const double bx =
          value(e.b_x, equation_label("b_x"), point[q], t) -
          value(
              divergence.x,
              [] {
                return std::string(
                    "Equation: the x component of the divergence of A");
              },
              point[q], t);
      const double by =
          value(e.b_y, equation_label("b_y"), point[q], t) -
          value(
              divergence.y,
              [] {
                return std::string(
                    "Equation: the y component of the divergence of A");
              },
              point[q], t);
      for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
          c[i][j] +=
              weight * lambda[q][i] * (bx * gradient[j].x + by * gradient[j].y);
    }
    add(d->stiffness, 1.0, c);
  }
  // R, M1, M2: integral of coefficient * phi_j * phi_i.
  const auto mass_like = [&](const CompiledExpression &coefficient,
                             const char *name) {
    Block3 m{};
    for (std::size_t q = 0; q < 3; ++q) {
      const double v = value(coefficient, equation_label(name), point[q], t);
      for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
          m[i][j] += weight * v * lambda[q][i] * lambda[q][j];
    }
    return m;
  };
  if (auto *d = destination(out, terms.r))
    add(d->stiffness, 1.0, mass_like(e.c, "c"));
  if (auto *d = destination(out, terms.m1))
    add(d->damping, 1.0, mass_like(e.m1, "m1"));
  if (auto *d = destination(out, terms.m2))
    add(d->mass, 1.0, mass_like(e.m2, "m2"));
  // F: integral of f phi_i.
  if (auto *d = destination(out, terms.f)) {
    std::array<double, 3> f{};
    for (std::size_t q = 0; q < 3; ++q) {
      const double v = value(e.f, equation_label("f"), point[q], t);
      for (std::size_t i = 0; i < 3; ++i)
        f[i] += weight * v * lambda[q][i];
    }
    for (std::size_t i = 0; i < 3; ++i)
      d->load[i] += f[i];
  }
  return out;
}

ElementBlocks<2> edge_blocks(const MeshedProblem &problem, std::size_t region,
                             const std::array<std::size_t, 2> &nodes, double t,
                             const EdgeTerms &terms) {
  ElementBlocks<2> out;
  if (terms.s == Target::skip && terms.t == Target::skip &&
      terms.d == Target::skip && terms.g == Target::skip)
    return out;
  const auto &mesh = *problem.mesh();
  const auto &e = problem.equation();
  const auto &condition = problem.conditions()[region];
  const std::string &region_name = mesh.region_names[region];
  const auto equation_label = [](const char *name) {
    return [name] { return std::string("Equation coefficient ") + name; };
  };
  const auto region_label = [&](const char *name) {
    return [&region_name, name] {
      return "Region '" + region_name + "', coefficient " + name;
    };
  };

  // The straight edge: length, unit tangent tau (along the edge, the domain
  // on its left) and outward normal n (tau is n turned counterclockwise).
  const Point a = mesh.nodes[nodes[0]], b = mesh.nodes[nodes[1]];
  const double length = std::hypot(b.x - a.x, b.y - a.y);
  const Point tau{(b.x - a.x) / length, (b.y - a.y) / length};
  const Point n{tau.y, -tau.x};
  // d/dtau of phi_a = 1 - s and phi_b = s.
  const std::array<double, 2> phi_tau{-1.0 / length, 1.0 / length};
  // 2-point Gauss-Legendre on [0, 1]: s = 1/2 -+ 1/(2 sqrt 3), weight
  // length / 2 each.
  const double offset = 0.5 / std::sqrt(3.0);
  const std::array<double, 2> s{0.5 - offset, 0.5 + offset};
  const double weight = length / 2.0;

  Block2 s_block{}, t_block{}, d_block{};
  std::array<double, 2> g_block{};
  for (std::size_t q = 0; q < 2; ++q) {
    const Point p{a.x + s[q] * (b.x - a.x), a.y + s[q] * (b.y - a.y)};
    const std::array<double, 2> phi{1.0 - s[q], s[q]};
    // (A grad u).n = kappa_n u_n + kappa_tau u_tau.
    const double xx = value(e.a_xx, equation_label("a_xx"), p, t);
    const double xy = value(e.a_xy, equation_label("a_xy"), p, t) / 2;
    const double yy = value(e.a_yy, equation_label("a_yy"), p, t);
    const double kappa_n =
        n.x * (xx * n.x + xy * n.y) + n.y * (xy * n.x + yy * n.y);
    const double kappa_tau =
        n.x * (xx * tau.x + xy * tau.y) + n.y * (xy * tau.x + yy * tau.y);
    // The condition: alpha u + beta_n_hat u_n + beta_tau_hat u_tau
    // + gamma u_t = g, with (beta_x, beta_y) split along n and tau.
    const double beta_x = value(condition.beta_x, region_label("beta_x"), p, t);
    const double beta_y = value(condition.beta_y, region_label("beta_y"), p, t);
    const double beta_n_hat =
        value(condition.beta_n, region_label("beta_n"), p, t) + beta_x * n.x +
        beta_y * n.y;
    if (beta_n_hat == 0.0)
      throw std::invalid_argument(std::format(
          "Region '{}': beta_n + beta_x n_x + beta_y n_y is zero at ({}, {}) "
          "on the boundary edge from node {} to node {}, t = {}, so the "
          "condition cannot be solved for the normal derivative u_n",
          region_name, p.x, p.y, nodes[0], nodes[1], t));
    // u_n = (g - alpha u - beta_tau_hat u_tau - gamma u_t) / beta_n_hat.
    const double factor = kappa_n / beta_n_hat;
    if (terms.s != Target::skip) {
      const double coefficient =
          factor * value(condition.alpha, region_label("alpha"), p, t);
      for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
          s_block[i][j] += weight * coefficient * phi[i] * phi[j];
    }
    if (terms.t != Target::skip) {
      const double beta_tau_hat =
          value(condition.beta_tau, region_label("beta_tau"), p, t) +
          beta_x * tau.x + beta_y * tau.y;
      const double coefficient = kappa_tau - factor * beta_tau_hat;
      for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
          t_block[i][j] += weight * coefficient * phi[i] * phi_tau[j];
    }
    if (terms.d != Target::skip) {
      const double coefficient =
          factor * value(condition.gamma, region_label("gamma"), p, t);
      for (std::size_t i = 0; i < 2; ++i)
        for (std::size_t j = 0; j < 2; ++j)
          d_block[i][j] += weight * coefficient * phi[i] * phi[j];
    }
    if (terms.g != Target::skip) {
      const double coefficient =
          factor * value(condition.g, region_label("g"), p, t);
      for (std::size_t i = 0; i < 2; ++i)
        g_block[i] += weight * coefficient * phi[i];
    }
  }
  if (auto *d = destination(out, terms.s))
    add(d->stiffness, -1.0, s_block);
  if (auto *d = destination(out, terms.t))
    add(d->stiffness, 1.0, t_block);
  if (auto *d = destination(out, terms.d))
    add(d->damping, -1.0, d_block);
  if (auto *d = destination(out, terms.g))
    for (std::size_t i = 0; i < 2; ++i)
      d->load[i] -= g_block[i];
  return out;
}

} // namespace pde2d::fem::detail

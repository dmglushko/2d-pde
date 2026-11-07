#pragma once
#include "fem/error_norms.hpp"
// Independent original quadrature loop, intentionally rebuilding rules.
ErrorNorms reference(const discretization::TriangleMesh &mesh,
                     const fem::Vector &v, const fem::Expr &exact,
                     double time) {
  fem::Evaluator u(exact), ux(fem::diff(exact, fem::x)),
      uy(fem::diff(exact, fem::y));
  ErrorNorms r;
  for (const auto &tri : mesh.elements) {
    const auto a = mesh.nodes[tri[0]], b = mesh.nodes[tri[1]],
               c = mesh.nodes[tri[2]];
    const double d = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    const std::array<double, 3> gx = {(b.y - c.y) / d, (c.y - a.y) / d,
                                      (a.y - b.y) / d},
                                gy = {(c.x - b.x) / d, (a.x - c.x) / d,
                                      (b.x - a.x) / d};
    double dx = 0, dy = 0;
    for (int j = 0; j < 3; j++) {
      dx += gx[j] * v[tri[j]];
      dy += gy[j] * v[tri[j]];
    }
    for (const auto &q : fem::triangle_quadrature(8)) {
      fem::EvaluationPoint p{0, 0, time};
      double uh = 0;
      for (int j = 0; j < 3; j++) {
        p.x += q.barycentric[j] * mesh.nodes[tri[j]].x;
        p.y += q.barycentric[j] * mesh.nodes[tri[j]].y;
        uh += q.barycentric[j] * v[tri[j]];
      }
      const double e = uh - u(p), ex = dx - ux(p), ey = dy - uy(p),
                   w = std::abs(d) * .5 * q.weight;
      r.l2 += w * e * e;
      r.h1_seminorm += w * (ex * ex + ey * ey);
    }
  }
  return {std::sqrt(r.l2), std::sqrt(r.h1_seminorm)};
}

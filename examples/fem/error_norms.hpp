#pragma once
#include "fem/assembly.hpp"
#include "fem/quadrature.hpp"
#include <cmath>

struct ErrorNorms { double l2 = 0, h1_seminorm = 0; };
inline ErrorNorms error_norms(const discretization::TriangleMesh& mesh,
                             const fem::Vector& values, const fem::Expr& exact,
                             double time = 0, int degree = 8) {
  fem::Evaluator u(exact), ux(fem::diff(exact,fem::x)), uy(fem::diff(exact,fem::y));
  ErrorNorms result;
  for (const auto& tri:mesh.elements) {
    const auto a=mesh.nodes[tri[0]], b=mesh.nodes[tri[1]], c=mesh.nodes[tri[2]];
    const double det=(b.x-a.x)*(c.y-a.y)-(c.x-a.x)*(b.y-a.y);
    const std::array<double,3> gx={(b.y-c.y)/det,(c.y-a.y)/det,(a.y-b.y)/det};
    const std::array<double,3> gy={(c.x-b.x)/det,(a.x-c.x)/det,(b.x-a.x)/det};
    double dx=0,dy=0;
    for(int j=0;j<3;++j) { dx+=gx[j]*values[tri[j]];dy+=gy[j]*values[tri[j]]; }
    for (const auto& q:fem::triangle_quadrature(degree)) {
      fem::EvaluationPoint p; p.t=time;
      double uh=0;
      for (int j=0;j<3;++j) {
        p.x+=q.barycentric[j]*mesh.nodes[tri[j]].x;
        p.y+=q.barycentric[j]*mesh.nodes[tri[j]].y;
        uh+=q.barycentric[j]*values[tri[j]];
      }
      const double e=uh-u(p),ex=dx-ux(p),ey=dy-uy(p);
      const double weight=std::abs(det)*.5*q.weight;
      result.l2+=weight*e*e;
      result.h1_seminorm+=weight*(ex*ex+ey*ey);
    }
  }
  result.l2=std::sqrt(result.l2);
  result.h1_seminorm=std::sqrt(result.h1_seminorm);
  return result;
}

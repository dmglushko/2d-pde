#pragma once

#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace fem {
struct TriangleQuadraturePoint { std::array<double,3> barycentric; double weight; };
struct EdgeQuadraturePoint { double coordinate; double weight; };
namespace quadrature_detail {
// A bounded supported range also prevents accidental enormous allocations.
inline void validate_degree(int degree) {
    if (degree < 0 || degree > 64)
        throw std::invalid_argument("Quadrature exactness degree must be in [0,64]");
}
// Gauss-Legendre on [0,1], constructed from symmetric roots of P_n.
inline std::vector<EdgeQuadraturePoint> gauss(int n) {
    std::vector<EdgeQuadraturePoint> points(n);
    for (int i=0; i<(n+1)/2; ++i) {
        double z = std::cos(std::numbers::pi*(i+0.75)/(n+0.5));
        double derivative=0;
        bool converged=false;
        for (int iteration=0; iteration<100; ++iteration) {
            double p=1, previous=0;
            for (int k=1; k<=n; ++k) {
                double next=((2*k-1)*z*p-(k-1)*previous)/k;
                previous=p; p=next;
            }
            derivative=n*(z*p-previous)/(z*z-1);
            const double change=p/derivative;
            z-=change;
            if (std::abs(change)<4e-16) { converged=true; break; }
        }
        if (!converged) throw std::runtime_error("Gauss-Legendre root iteration failed");
        // Recompute derivative at the accepted root for accurate weights.
        double p=1, previous=0;
        for (int k=1; k<=n; ++k) {
            double next=((2*k-1)*z*p-(k-1)*previous)/k;
            previous=p; p=next;
        }
        derivative=n*(z*p-previous)/(z*z-1);
        const double weight=1/((1-z*z)*derivative*derivative);
        points[i]={(1-z)/2,weight};
        points[n-1-i]={(1+z)/2,weight};
    }
    return points;
}
} // namespace quadrature_detail

inline std::vector<EdgeQuadraturePoint> edge_quadrature(int degree=2) {
    quadrature_detail::validate_degree(degree);
    return quadrature_detail::gauss((degree+2)/2);
}
inline std::vector<TriangleQuadraturePoint> triangle_quadrature(int degree=2) {
    quadrature_detail::validate_degree(degree);
    if (degree<=2) return {{{2./3,1./6,1./6},1./3},
                           {{1./6,2./3,1./6},1./3},
                           {{1./6,1./6,2./3},1./3}};
    // (u,v) -> (u,(1-u)*v) has Jacobian (1-u). A degree d
    // polynomial becomes degree <=d+1 in u and <=d in v.
    const auto u=quadrature_detail::gauss((degree+3)/2);
    const auto v=quadrature_detail::gauss((degree+2)/2);
    std::vector<TriangleQuadraturePoint> points;
    points.reserve(u.size()*v.size());
    for (const auto& a:u) for (const auto& b:v) {
        const double X=a.coordinate, Y=(1-X)*b.coordinate;
        points.push_back({{(1-X)*(1-b.coordinate),X,Y},2*a.weight*b.weight*(1-X)});
    }
    return points;
}
} // namespace fem

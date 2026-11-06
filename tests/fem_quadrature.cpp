#include "fem/quadrature.hpp"
#include <iostream>
#include <limits>

namespace {
void check(bool ok,const char* message) { if (!ok) throw std::runtime_error(message); }
void near(long double a,long double b) {
    check(std::abs(a-b)<3e-13L*std::abs(b),"Quadrature monomial integral mismatch");
}
template<class F> void rejects(F f) {
    bool rejected=false;
    try { f(); } catch(const std::invalid_argument&) { rejected=true; }
    check(rejected,"Invalid degree accepted");
}
}
int main() {
    try {
        check(fem::triangle_quadrature().size()==3,"Baseline must have three points");
        for(int degree=0;degree<=64;++degree) {
            const auto edge=fem::edge_quadrature(degree);
            const auto triangle=fem::triangle_quadrature(degree);
            long double edge_sum=0, triangle_sum=0;
            for(auto p:edge) {
                check(p.coordinate>0 && p.coordinate<1 && p.weight>0,"Invalid edge point");
                edge_sum+=p.weight;
            }
            for(auto p:triangle) {
                for(double b:p.barycentric) check(b>0 && b<1,"Invalid barycentric coordinate");
                near(p.barycentric[0]+p.barycentric[1]+p.barycentric[2],1);
                check(p.weight>0,"Nonpositive triangle weight");
                triangle_sum+=p.weight;
            }
            near(edge_sum,1); near(triangle_sum,1);
            for(int i=0;i<=degree;++i) {
                long double integral=0;
                for(auto p:edge) integral+=p.weight*std::pow(static_cast<long double>(p.coordinate),i);
                near(integral,1.L/(i+1));
                for(int j=0;j<=degree-i;++j) {
                    integral=0;
                    for(auto p:triangle)
                        integral+=p.weight*std::pow(static_cast<long double>(p.barycentric[1]),i)
                                          *std::pow(static_cast<long double>(p.barycentric[2]),j);
                    // Normalized reference-triangle integral: 2*i!*j!/(i+j+2)!.
                    const long double exact=2*std::exp(std::lgamma(static_cast<long double>(i+1))+
                        std::lgamma(static_cast<long double>(j+1))-std::lgamma(static_cast<long double>(i+j+3)));
                    near(integral,exact);
                }
            }
        }
        for(int degree:{-1,65,std::numeric_limits<int>::max(),std::numeric_limits<int>::min()}) {
            rejects([=] { fem::edge_quadrature(degree); });
            rejects([=] { fem::triangle_quadrature(degree); });
        }
        std::cout<<"FEM quadrature tests passed (all monomials, degrees 0 through 64)\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}

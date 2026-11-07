#include "discretization/geometry.hpp"
#include "discretization/boundary_discretization.hpp"
#include "discretization/svg_geometry.hpp"
#include <chrono>
#include <ctime>
#include <iostream>
#include <fstream>
#include <filesystem>
using namespace discretization;
int main(int argc,char**argv) {
 const std::filesystem::path file=argc>1?argv[1]:"build/geometry-benchmark.svg";
 if(file.has_parent_path())std::filesystem::create_directories(file.parent_path());
 std::vector<CurveRegion> regions;
 std::ofstream svg;svg.exceptions(std::ios::failbit|std::ios::badbit);svg.open(file); svg.precision(17); svg<<"<svg>";
 constexpr int n=1800;
 for(int i=0;i<n;++i){double a=2*std::numbers::pi*i/n,b=2*std::numbers::pi*(i+1)/n;Point2 p{std::cos(a),std::sin(a)},q{std::cos(b),std::sin(b)};regions.push_back({"r"+std::to_string(i%90),{LineCurve{p,q}}});}
 // Odd/even permutation, reversed alternate paths; first path remains anchor.
 for(int k=0;k<n;++k){int i=k==0?0:1+(k*701)%(n-1); auto c=std::get<LineCurve>(regions[i].segments[0]);if(k%2)std::swap(c.start,c.end);svg<<"<path id=\"p"<<i<<"\" d=\"M "<<c.start.x<<' '<<c.start.y<<" L "<<c.end.x<<' '<<c.end.y<<"\"/>";}svg<<"</svg>";svg.close();
 CurveGeometry g(regions); double sum=0;
 auto bench=[&](const char*name,auto work){auto c=std::clock();auto w=std::chrono::steady_clock::now();work();std::cout<<name<<" cpu="<<double(std::clock()-c)/CLOCKS_PER_SEC<<" wall="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-w).count()<<'\n';};
 bench("queries",[&]{for(int k=0;k<6;++k)for(const auto&c:g.curves()){sum+=g.length(c.id);sum+=g.regionLength(c.region);}});
 bench("copy",[&]{for(int k=0;k<40;++k){auto copy=g;sum+=copy.length(copy.curves()[0].id);}});
 bench("inversion",[&]{for(int k=0;k<80;++k)for(const CurveSegment&c:std::vector<CurveSegment>{CubicBezier{{0,0},{0,3},{1,-3},{1,0}},EllipticalArc{{0,0},2,1,.2,.3,5}})sum+=curveParameterAtLengthFraction(c,.01+.98*(k+.5)/80);});
 bench("sampling",[&]{for(int k=0;k<20;++k){EllipseGeometry e(2,1);auto b=discretize_boundary(e,BoundaryOptions{.04,.0003});sum+=b.boundary.points.size();}});
 bench("svg",[&]{for(int k=0;k<5;++k)sum+=importSvg(file.string()).curves().size();});
 std::cout<<"checksum="<<sum<<'\n';
}

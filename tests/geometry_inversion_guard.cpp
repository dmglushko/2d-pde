#include "discretization/curve_length.hpp"
#include <iostream>
using namespace discretization;
int main(){
 for(double scale:{1e-100,1.,1e100})for(const CurveSegment& c:std::vector<CurveSegment>{
   QuadraticBezier{{scale,0},{-scale,0},{scale,0}},
   CubicBezier{{0,0},{0,0},{0,0},{scale,0}},
   CubicBezier{{0,0},{0,3*scale},{scale,-3*scale},{scale,0}},
   EllipticalArc{{0,0},2*scale,scale,.2,.3,5}}) {
   double length=curveLength(c),previous=0;
   for(double f:{1e-14,1e-9,.001,.1,.3,.5,.9,.999,1-1e-9,1-1e-14}){
     double t=curveParameterAtLengthFraction(c,f);
     if(!(t>previous && t<1))throw std::runtime_error("Inversion lost monotonic bracket");
     previous=t;
     double target=std::min(f,1-f);
     double measured=f>.5?curveLength(splitCurve(c,t).second)/length:curveLength(c,t)/length;
     double rounding=32*std::numeric_limits<double>::epsilon()*std::abs(t)*curve_detail::speed(c,t)/length;
     if(std::abs(measured-target)>2e-9*target+rounding)throw std::runtime_error("Endpoint-relative inversion accuracy lost");
   }
 }
 CurveSegment mutableCurve=CubicBezier{{0,0},{0,0},{0,0},{1,0}};
 double before=curveLength(mutableCurve);std::get<CubicBezier>(mutableCurve).end.x=2;
 if(curveLength(mutableCurve)!=2*before)throw std::runtime_error("Public fields cached unsafely");
 std::cout<<"Safeguarded inversion grid: cusp, stationary endpoint, scale and near-end accuracy passed\n";
}

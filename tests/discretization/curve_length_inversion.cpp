#include "discretization/curve_length.hpp"
#include "support/geometry.hpp"
#include <iostream>
using namespace pde2d::discretization;
using pde2d_test::check;
void safeguarded_inversion() {
  for (double scale : {1e-100, 1., 1e100})
    for (const CurveSegment &c : std::vector<CurveSegment>{
             QuadraticBezier{{scale, 0}, {-scale, 0}, {scale, 0}},
             CubicBezier{{0, 0}, {0, 0}, {0, 0}, {scale, 0}},
             CubicBezier{
                 {0, 0}, {0, 3 * scale}, {scale, -3 * scale}, {scale, 0}},
             EllipticalArc{{0, 0}, 2 * scale, scale, .2, .3, 5}}) {
      double length = curveLength(c), previous = 0;
      for (double f :
           {1e-14, 1e-9, .001, .1, .3, .5, .9, .999, 1 - 1e-9, 1 - 1e-14}) {
        double t = curveParameterAtLengthFraction(c, f);
        if (!(t > previous && t < 1))
          pde2d_test::fail("Inversion lost monotonic bracket");
        previous = t;
        double target = std::min(f, 1 - f);
        double measured = f > .5 ? curveLength(splitCurve(c, t).second) / length
                                 : curveLength(c, t) / length;
        double rounding = 32 * std::numeric_limits<double>::epsilon() *
                          std::abs(t) *
                          pde2d_test::norm(curveDerivative(c, t)) / length;
        if (std::abs(measured - target) > 2e-9 * target + rounding)
          pde2d_test::fail("Endpoint-relative inversion accuracy lost");
      }
    }
  CurveSegment mutableCurve = CubicBezier{{0, 0}, {0, 0}, {0, 0}, {1, 0}};
  double before = curveLength(mutableCurve);
  std::get<CubicBezier>(mutableCurve).end.x = 2;
  if (curveLength(mutableCurve) != 2 * before)
    pde2d_test::fail("Public fields cached unsafely");
}
int main() {
  return pde2d_test::run({{"safeguarded_inversion", safeguarded_inversion}});
}

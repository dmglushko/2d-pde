#include "discretization/curve_length.hpp"
#include "support/check.hpp"
#include <iostream>
#include <limits>
using namespace discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;
namespace {
void near(double a, double b, double tolerance = 2e-9) {
  pde2d_test::near_rel(a, b, tolerance);
}
void closed_forms() {
  near(curveLength(LineCurve{{0, 0}, {3, 4}}), 5);
  near(curveLength(EllipticalArc{{0, 0}, 3, 3, .2, .7, -2}), 6);
  near(curveLength(EllipticalArc{{0, 0}, 2, 1, .4, 0, std::numbers::pi / 2}),
       2.422112055136919);
  near(curveLength(EllipticalArc{
           {0, 0}, 2, 1, 0, std::numbers::pi / 2, -std::numbers::pi / 2}),
       2.422112055136919);
  near(curveLength(QuadraticBezier{{0, 0}, {.5, 0}, {1, 1}}),
       std::sqrt(5.) / 2 + std::asinh(2.) / 4);
  near(curveLength(QuadraticBezier{{.09, 0}, {-.21, 0}, {.49, 0}}), .58);
}
void scaled_inversion() {
  for (double scale : {1e-100, 1., 1e100}) {
    CurveSegment quadratic = QuadraticBezier{{0, 0}, {0, 0}, {scale, 0}};
    CurveSegment cubic = CubicBezier{{0, 0}, {0, 0}, {0, 0}, {scale, 0}};
    near(curveLength(quadratic), scale);
    near(curveLength(cubic), scale);
    near(curveParameterAtLengthFraction(quadratic, .25), .5);
    near(curveParameterAtLengthFraction(cubic, .125), .5);
    double tiny = curveParameterAtLengthFraction(cubic, 1e-14);
    near(curveLength(cubic, tiny) / curveLength(cubic), 1e-14);
    double late = curveParameterAtLengthFraction(cubic, 1 - 1e-5);
    near(curveLength(splitCurve(cubic, late).second) / curveLength(cubic), 1e-5,
         1e-7);
  }
}
void split_additivity() {
  for (const CurveSegment &c :
       std::vector<CurveSegment>{QuadraticBezier{{0, 0}, {1, 2}, {2, 0}},
                                 CubicBezier{{0, 0}, {0, 3}, {1, -3}, {1, 0}},
                                 EllipticalArc{{0, 0}, 2, 1, .2, .3, 5}}) {
    auto halves = splitCurve(c, .31);
    near(curveLength(halves.first) + curveLength(halves.second),
         curveLength(c));
    double t = curveParameterAtLengthFraction(c, .37);
    near(curveLength(c, t) / curveLength(c), .37);
  }
}
void invalid_input() {
  CurveSegment zero = LineCurve{{0, 0}, {0, 0}};
  check(curveLength(zero) == 0, "Zero length");
  expect_throws([&] { curveParameterAtLengthFraction(zero, .5); });
  for (double bad : {-1., 0., 1., 2., std::numeric_limits<double>::quiet_NaN()})
    expect_throws([&] {
      curveParameterAtLengthFraction(LineCurve{{0, 0}, {1, 0}}, bad);
    });
  expect_throws([] { curveLength(EllipticalArc{{0, 0}, 1, 1, 0, 0, 0}); });
  expect_throws([] {
    curveLength(QuadraticBezier{
        {0, 0}, {std::numeric_limits<double>::infinity(), 0}, {1, 1}});
  });
  expect_throws([] { curveLength(LineCurve{{0, 0}, {1, 0}}, 2); });
}
} // namespace
int main() {
  return pde2d_test::run({{"closed_forms", closed_forms},
                          {"scaled_inversion", scaled_inversion},
                          {"split_additivity", split_additivity},
                          {"invalid_input", invalid_input}});
}

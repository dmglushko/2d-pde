#include "discretization/curve_io.hpp"
#include "discretization/curve_length.hpp"
#include "discretization/mesh_types.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace discretization;
namespace {
void check(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
void near(double a, double b, double tolerance = 2e-11) {
  check(std::abs(a - b) <= tolerance * std::max({1., std::abs(a), std::abs(b)}),
        "Scalar mismatch");
}
void near(Point2 a, Point2 b) {
  near(a.x, b.x);
  near(a.y, b.y);
}
Point2 times(Point2 p, double s) { return {p.x * s, p.y * s}; }
template <class F> void rejects(F f) {
  bool threw = false;
  try {
    f();
  } catch (const std::exception &) {
    threw = true;
  }
  check(threw, "Invalid input accepted");
}
void contract(const CurveSegment &c) {
  const auto bounds = curveDerivativeBounds(c);
  const auto halves = splitCurve(c, .31);
  const auto reversed = reverseCurve(c);
  const CurveTransform f{2.3, .7, {4, -3}};
  const auto transformed = transformCurve(c, f);
  const CurveTransform vectors{f.scale, f.rotation, {0, 0}};
  const auto transformedBounds = curveDerivativeBounds(transformed);
  near(transformedBounds.first, f.scale * bounds.first);
  near(transformedBounds.second, f.scale * bounds.second);
  for (int i = 0; i <= 100; ++i) {
    double t = i / 100.;
    near(curvePoint(halves.first, t), curvePoint(c, .31 * t));
    near(curvePoint(halves.second, t), curvePoint(c, .31 + .69 * t));
    near(curveDerivative(halves.first, t),
         times(curveDerivative(c, .31 * t), .31));
    near(curveDerivative(halves.second, t),
         times(curveDerivative(c, .31 + .69 * t), .69));
    near(curveSecondDerivative(halves.first, t),
         times(curveSecondDerivative(c, .31 * t), .31 * .31));
    near(curveSecondDerivative(halves.second, t),
         times(curveSecondDerivative(c, .31 + .69 * t), .69 * .69));
    near(curvePoint(reversed, t), curvePoint(c, 1 - t));
    near(curveDerivative(reversed, t), times(curveDerivative(c, 1 - t), -1));
    near(curveSecondDerivative(reversed, t), curveSecondDerivative(c, 1 - t));
    near(curvePoint(transformed, t), f(curvePoint(c, t)));
    near(curveDerivative(transformed, t), vectors(curveDerivative(c, t)));
    near(curveSecondDerivative(transformed, t),
         vectors(curveSecondDerivative(c, t)));
    // Regression checks only: analytical justification is documented with the
    // implementations. Finite samples cannot prove the interval contract.
    check(curve_detail::norm(curveDerivative(c, t)) <=
              bounds.first * (1 + 1e-13) + 1e-14,
          "First bound violated");
    check(curve_detail::norm(curveSecondDerivative(c, t)) <=
              bounds.second * (1 + 1e-13) + 1e-14,
          "Second bound violated");
    if (i > 0 && i < 100) {
      constexpr double h = 1e-5;
      auto a = curvePoint(c, t - h), b = curvePoint(c, t + h);
      auto d = curveDerivative(c, t);
      auto da = curveDerivative(c, t - h), db = curveDerivative(c, t + h);
      auto second = curveSecondDerivative(c, t);
      near((db.x - da.x) / (2 * h), second.x, 1e-8);
      near((db.y - da.y) / (2 * h), second.y, 1e-8);
      near((b.x - a.x) / (2 * h), d.x, 1e-8);
      near((b.y - a.y) / (2 * h), d.y, 1e-8);
    }
  }
  for (auto [piece, width] : std::vector<std::pair<CurveSegment, double>>{
           {halves.first, .31}, {halves.second, .69}}) {
    auto b = curveDerivativeBounds(piece);
    check(b.first <= bounds.first * width * (1 + 1e-13) + 1e-14,
          "Split first bound did not shrink");
    check(b.second <= bounds.second * width * width * (1 + 1e-13) + 1e-14,
          "Split second bound did not shrink");
  }
  near(curveLength(halves.first) + curveLength(halves.second), curveLength(c));
  rejects([&] { splitCurve(c, 0); });
  rejects([&] { splitCurve(c, 1); });
  rejects([&] { transformCurve(c, {-1, 0, {0, 0}}); });
}
} // namespace
int main() {
  try {
    std::vector<CurveSegment> curves{
        LineCurve{{0, 0}, {3, 4}},
        QuadraticBezier{{0, 0}, {1, 2}, {2, 0}},
        CubicBezier{{0, 0}, {1, 3}, {-1, -2}, {2, 1}},
        EllipticalArc{{1, -2}, 3, 1, .4, .7, -5},
        CubicBezier{{0, 0},
                    {0, 0},
                    {1, 0},
                    {1, 0}}, // zero endpoint speeds, positive interior speed
        QuadraticBezier{{0, 0}, {0, 0}, {1, 0}},
        LineCurve{{1, 1}, {1, 1}}};
    for (const auto &c : curves)
      contract(c);
    auto hump = curves[4];
    check(curve_detail::norm(curveDerivative(hump, 0)) == 0 &&
              curve_detail::norm(curveDerivative(hump, 1)) == 0,
          "Expected stationary endpoints");
    near(curve_detail::norm(curveDerivative(hump, .5)), 1.5);
    check(curveDerivativeBounds(hump).first >= 1.5,
          "Endpoint samples used as bound");
    near(curveDerivativeBounds(curves[1]).first, 2 * std::sqrt(5.));
    near(curveDerivativeBounds(curves[1]).second, 8);
    near(curveDerivativeBounds(curves[3]).first, 15);
    near(curveDerivativeBounds(curves[3]).second, 75);

    // Recenter before scaling: avoid losing small offsets through cancellation
    // or overflowing the translation when SVG coordinates have a large origin.
    const CurveTransform recenter{1e100, 0, {}, {1e200, 1e200}};
    near(recenter({1e200, 1e200}), Point2{0, 0});
    const CurveTransform offset{3, 0, {}, {1e12, 1e12}};
    near(offset({1e12 + .25, 1e12 + .5}), Point2{.75, 1.5});
    // Pin the existing v3 wire records and prove that decoding and encoding
    // do not depend on variant order.
    using Reordered =
        std::variant<EllipticalArc, CubicBezier, LineCurve, QuadraticBezier>;
    const std::vector<std::string> records = {"0 0 0 3 4", "1 0 0 1 2 2 0",
                                              "2 0 0 1 3 -1 -2 2 1",
                                              "3 1 -2 3 1 0.4 0.7 -5"};
    for (const auto &record : records) {
      std::istringstream in(record);
      unsigned id;
      in >> id;
      auto c = curve_io::read<Reordered>(in, id);
      std::ostringstream out;
      curve_io::write(out, c);
      check(out.str() == record, "Wire record changed with variant order");
    }
    rejects([] {
      std::istringstream in;
      curve_io::read(in, 99);
    });
    rejects([] {
      std::istringstream in("0 0 1");
      curve_io::read(in, 0);
    });
    rejects([] {
      std::istringstream in("0 0 -1 2 0 0 1");
      curve_io::read(in, 3);
    });
    MeshOptions o;
    validateMeshOptionValues(o);
    o.algorithm = static_cast<InteriorAlgorithm>(2);
    rejects([&] { validateMeshOptionValues(o); });
    o.algorithm = InteriorAlgorithm::EarClipping;
    o.boundary_treatment = static_cast<BoundaryTreatment>(3);
    rejects([&] { validateMeshOptionValues(o); });
    std::cout << "Curve operation contracts, interval bounds, split scaling, "
                 "and stable wire IDs passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

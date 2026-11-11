#include "pde2d/geometry/shapes.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace pde2d::geometry {
namespace {

constexpr double pi = std::numbers::pi;

void require_positive(double value, const std::string &what) {
  if (!(value > 0) || !std::isfinite(value))
    throw std::invalid_argument(what + " must be finite and positive");
}

Loop single(const std::string &region, Curve curve) {
  return Loop{{BoundaryCurve{curve, region}}};
}

} // namespace

CurveGeometry circle(double radius) {
  require_positive(radius, "Circle radius");
  return CurveGeometry({single(
      "boundary", EllipticalArc{{0, 0}, radius, radius, 0, 0, 2 * pi})});
}

CurveGeometry ellipse(double a, double b) {
  require_positive(a, "Ellipse semi-axis a");
  require_positive(b, "Ellipse semi-axis b");
  return CurveGeometry(
      {single("boundary", EllipticalArc{{0, 0}, a, b, 0, 0, 2 * pi})});
}

CurveGeometry rectangle(double width, double height) {
  require_positive(width, "Rectangle width");
  require_positive(height, "Rectangle height");
  double x = width / 2, y = height / 2;
  return CurveGeometry({Loop{{{Line{{-x, -y}, {x, -y}}, "bottom"},
                              {Line{{x, -y}, {x, y}}, "right"},
                              {Line{{x, y}, {-x, y}}, "top"},
                              {Line{{-x, y}, {-x, -y}}, "left"}}}});
}

CurveGeometry annulus(double outer_radius, double inner_radius) {
  require_positive(outer_radius, "Annulus outer radius");
  require_positive(inner_radius, "Annulus inner radius");
  if (!(inner_radius < outer_radius))
    throw std::invalid_argument(
        "Annulus inner radius must be smaller than the outer radius");
  return CurveGeometry(
      {single("outer",
              EllipticalArc{{0, 0}, outer_radius, outer_radius, 0, 0, 2 * pi}),
       single("inner", EllipticalArc{
                           {0, 0}, inner_radius, inner_radius, 0, 0, 2 * pi})});
}

CurveGeometry rectangle_with_semicircle(double width, double height,
                                        SemicircleEnd end) {
  require_positive(width, "Rectangle width");
  require_positive(height, "Rectangle height");
  double x = width / 2, r = height / 2;
  if (end == SemicircleEnd::positive_x)
    return CurveGeometry(
        {Loop{{{Line{{-x, -r}, {x, -r}}, "bottom"},
               {EllipticalArc{{x, 0}, r, r, 0, -pi / 2, pi}, "cap"},
               {Line{{x, r}, {-x, r}}, "top"},
               {Line{{-x, r}, {-x, -r}}, "left"}}}});
  return CurveGeometry(
      {Loop{{{Line{{-x, -r}, {x, -r}}, "bottom"},
             {Line{{x, -r}, {x, r}}, "right"},
             {Line{{x, r}, {-x, r}}, "top"},
             {EllipticalArc{{-x, 0}, r, r, 0, pi / 2, pi}, "cap"}}}});
}

CurveGeometry polygon(std::vector<Point> vertices) {
  if (vertices.size() < 3)
    throw std::invalid_argument("A polygon needs at least three vertices");
  Loop loop;
  for (std::size_t i = 0; i < vertices.size(); ++i)
    loop.curves.push_back(
        {Line{vertices[i], vertices[(i + 1) % vertices.size()]}, "boundary"});
  return CurveGeometry({std::move(loop)});
}

CurveGeometry regular_polygon(std::size_t sides, double side_length) {
  if (sides < 3)
    throw std::invalid_argument("A regular polygon needs at least 3 sides");
  require_positive(side_length, "Side length");
  const double n = static_cast<double>(sides);
  double r = side_length / (2 * std::sin(pi / n));
  std::vector<Point> vertices;
  for (std::size_t i = 0; i < sides; ++i) {
    double a = 2 * pi * static_cast<double>(i) / n;
    vertices.push_back({r * std::cos(a), r * std::sin(a)});
  }
  return polygon(std::move(vertices));
}

} // namespace pde2d::geometry

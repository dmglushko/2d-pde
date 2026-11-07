#include "discretization/geometry.hpp"
#include "discretization/boundary_discretization.hpp"
#include "support/check.hpp"
#include <bit>
#include <iostream>
using namespace pde2d::discretization;
// Byte-exact signatures recorded against the pre-optimization implementation.
// Cover points, region tags, protected vertices and every provenance interval.
std::uint64_t signature(const DiscretizedBoundary &result) {
  std::uint64_t h = 14695981039346656037ull;
  auto add = [&](std::uint64_t v) {
    for (int i = 0; i < 8; ++i) {
      h ^= (v >> (8 * i)) & 255;
      h *= 1099511628211ull;
    }
  };
  const auto &b = result.boundary;
  const auto &c = *result.context;
  add(b.points.size());
  for (auto p : b.points) {
    add(std::bit_cast<std::uint64_t>(p.x));
    add(std::bit_cast<std::uint64_t>(p.y));
  }
  for (const auto &tag : b.edge_tags) {
    add(tag.size());
    for (unsigned char v : tag)
      add(v);
  }
  for (auto v : c.protected_vertices)
    add(v);
  for (const auto &edge : c.edge_sources) {
    add(edge.size());
    for (auto span : edge) {
      add(span.curve);
      add(std::bit_cast<std::uint64_t>(span.begin));
      add(std::bit_cast<std::uint64_t>(span.end));
    }
  }
  return h;
}
std::vector<std::uint64_t> signatures() {
  std::vector<std::uint64_t> values;
  EllipseGeometry e(2, 1);
  for (auto o : {BoundaryOptions{.04, .0003}, BoundaryOptions{.5, .01}})
    values.push_back(signature(discretize_boundary(e, o)));
  std::vector<CurveRegion> regions;
  for (int side = 0; side < 4; ++side)
    for (int i = 0; i < 32; ++i) {
      auto point = [&](double t) -> Point2 {
        switch (side) {
        case 0:
          return {t, 0};
        case 1:
          return {1, t};
        case 2:
          return {1 - t, 1};
        default:
          return {0, 1 - t};
        }
      };
      regions.push_back(
          {side % 2 ? "b" : "a",
           {LineCurve{point(double(i) / 32), point(double(i + 1) / 32)}}});
    }
  CurveGeometry square(regions);
  for (bool preserve : {false, true})
    for (bool corners : {false, true}) {
      BoundaryOptions o{.4, .001};
      o.preserve_corners = corners;
      values.push_back(signature(discretize_boundary(square, o, preserve)));
    }
  return values;
}
int main(int argc, char **) {
  if (argc > 1) { // Regenerate the recorded signatures.
    for (auto v : signatures())
      std::cout << v << ",";
    std::cout << '\n';
    return 0;
  }
  const std::vector<std::uint64_t> expected = {
      11149290545666623588ull, 626144503457228534ull,  13003977193416062361ull,
      12007940997374442325ull, 6797412203164507221ull, 6797412203164507221ull,
  };
  return pde2d_test::run({{"byte_exact_signatures", [&] {
                             pde2d_test::check(
                                 signatures() == expected,
                                 "Sampling order/provenance changed");
                           }}});
}

#include "discretization/geometry.hpp"
#include <barrier>
#include <thread>
#include <iostream>
using namespace discretization;
void check(bool ok) {
  if (!ok)
    throw std::runtime_error("Geometry query invariant failed");
}
int main() {
  // Every fresh geometry exercises shared cold misses, hits, and copies while
  // other readers fill the source cache. Workers never edit the shared source.
  for (int pass = 0; pass < 8; ++pass) {
    CurveGeometry g(
        {{"a", {LineCurve{{0, 0}, {1, 0}}}},
         {"b", {LineCurve{{1, 0}, {1, 1}}}},
         {"a", {LineCurve{{1, 1}, {0, 1}}, LineCurve{{0, 1}, {0, 0}}}}});
    EllipseGeometry ellipse(2, 1);
    const auto ellipseId = ellipse.curves()[0].id;
    const double expected = curveLength(ellipse.curves()[0].segment);
    std::barrier start(8);
    std::vector<std::jthread> workers;
    for (int k = 0; k < 8; ++k)
      workers.emplace_back([&, k] {
        start.arrive_and_wait();
        if (k % 2) {
          auto coldCopy = ellipse;
          check(coldCopy.length(ellipseId) == expected);
        } else
          check(ellipse.regionLength("boundary") == expected);
        for (int i = 0; i < 80; ++i) {
          check(g.regionLength("a") == 3 && g.regionLength("b") == 1);
          for (const auto &c : g.curves())
            check(g.length(c.id) == 1);
          CurveGeometry copy;
          copy = g;
          auto ellipseCopy = ellipse;
          check(ellipseCopy.length(ellipseId) == expected);
          if (k % 2)
            copy.renameRegion("a", "c");
          check(copy.regionLength(k % 2 ? "c" : "a") == 3);
          auto moved = std::move(copy);
          check(moved.length(moved.curves()[0].id) == 1);
        }
      });
  }
  CurveGeometry seam(
      {{"a", {LineCurve{{0, 0}, {1, 0}}}},
       {"b", {LineCurve{{1, 0}, {1, 1}}}},
       {"a", {LineCurve{{1, 1}, {0, 1}}, LineCurve{{0, 1}, {0, 0}}}}});
  auto r = seam.region("a");
  check(r.connected());
  check(r.curves[0] == seam.curves()[0].id &&
        r.pieces[0][0] == seam.curves()[2].id);
  auto old = seam.curves()[0].id;
  auto parts = seam.splitCurve(old, .3, "x", "y");
  check(std::abs(seam.length(parts[0]) - .3) < 1e-12);
  bool retired = false;
  try {
    (void)seam.curve(old);
  } catch (const std::out_of_range &) {
    retired = true;
  }
  check(retired);
  EllipseGeometry e(2, 1);
  auto copy = e;
  auto id = e.curves()[0].id;
  double length = e.length(id);
  copy.setA(3);
  check(e.length(id) == length && copy.length(id) > length);
  e.splitCurve(id, .3, "p", "q");
  auto ids = e.region("p").curves;
  e.setB(1.2);
  check(e.region("p").curves == ids);
  auto before = e.curves();
  try {
    e.setA(-1);
  } catch (const std::invalid_argument &) {
  }
  check(e.curves().size() == before.size());
  std::cout << "Concurrent cold/hit/copy queries, seam indexing and "
               "independent edits passed\n";
}

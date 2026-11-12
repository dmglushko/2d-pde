#include "pde2d/geometry/bounding_box.hpp"

#include <algorithm>
#include <stdexcept>

namespace pde2d::geometry {
namespace {

BoundingBox box_of(Point p) { return {p, p}; }

} // namespace

BoundingBox merged(const BoundingBox &a, const BoundingBox &b) {
  return {{std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y)},
          {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y)}};
}

BoundingBox merged(const BoundingBox &box, Point p) {
  return merged(box, box_of(p));
}

BoundingBox padded(const BoundingBox &box, double margin) {
  return {{box.min.x - margin, box.min.y - margin},
          {box.max.x + margin, box.max.y + margin}};
}

bool contains(const BoundingBox &box, Point p) {
  return p.x >= box.min.x && p.x <= box.max.x && p.y >= box.min.y &&
         p.y <= box.max.y;
}

double width(const BoundingBox &box) { return box.max.x - box.min.x; }
double height(const BoundingBox &box) { return box.max.y - box.min.y; }

double diagonal(const BoundingBox &box) { return distance(box.min, box.max); }

BoundingBox bounding_box(std::span<const Point> points) {
  if (points.empty())
    throw std::invalid_argument("bounding_box: no points");
  BoundingBox box = box_of(points.front());
  for (Point p : points)
    box = merged(box, p);
  return box;
}

} // namespace pde2d::geometry

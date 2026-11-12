#include "pde2d/mesh/boundary_polygon.hpp"

#include "pde2d/geometry/curve_geometry.hpp"
#include "pde2d/geometry/polygon.hpp"

#include <stdexcept>
#include <string>

namespace pde2d::mesh {

void validate(const BoundaryPolygon &polygon) {
  using std::to_string;
  const auto &loops = polygon.loops;
  if (loops.empty())
    throw std::invalid_argument("Boundary polygon has no loops");
  for (const auto &name : polygon.region_names)
    if (!geometry::is_valid_region_name(name))
      throw std::invalid_argument("Boundary polygon: invalid region name '" +
                                  name + "'");
  std::vector<std::vector<geometry::Point>> points;
  for (std::size_t l = 0; l < loops.size(); ++l) {
    const auto &loop = loops[l];
    std::string where = "Boundary polygon loop " + to_string(l);
    if (loop.points.size() < 3)
      throw std::invalid_argument(where + " has fewer than 3 points");
    if (loop.edges.size() != loop.points.size())
      throw std::invalid_argument(where + " needs one edge per point");
    for (const auto &e : loop.edges) {
      if (e.region >= polygon.region_names.size())
        throw std::invalid_argument(where + " has a region id out of range");
      if (e.link && !is_valid_link(*e.link, polygon.chains))
        throw std::invalid_argument(where + " has an invalid curve link");
    }
    if (loop.parent) {
      if (*loop.parent >= loops.size() ||
          loops[*loop.parent].depth + 1 != loop.depth)
        throw std::invalid_argument(where + " has an inconsistent parent");
    } else if (loop.depth != 0) {
      throw std::invalid_argument(where + " has depth " +
                                  to_string(loop.depth) + " but no parent");
    }
    bool ccw = geometry::polygon_orientation(loop.points) > 0;
    if (ccw != (loop.depth % 2 == 0))
      throw std::invalid_argument(
          where + " runs in the wrong direction for depth " +
          to_string(loop.depth) +
          " (even depth counterclockwise, odd depth clockwise)");
    points.push_back(loop.points);
  }
  if (auto c = geometry::find_crossing(points)) {
    throw std::invalid_argument(
        "Boundary polygon edges cross or touch (loop " +
        to_string(c->first.loop) + " edge " + to_string(c->first.segment) +
        ", loop " + to_string(c->second.loop) + " edge " +
        to_string(c->second.segment) +
        "); the chords of nearby curves intersect, so use a smaller "
        "max_edge_length or curve deviation");
  }
  for (std::size_t i = 0; i < loops.size(); ++i) {
    std::size_t depth = 0;
    for (std::size_t j = 0; j < loops.size(); ++j)
      if (i != j &&
          geometry::locate_in_polygon(points[j], points[i].front()) > 0)
        ++depth;
    if (depth != loops[i].depth)
      throw std::invalid_argument("Boundary polygon loop " + to_string(i) +
                                  " has depth " + to_string(loops[i].depth) +
                                  " but lies inside " + to_string(depth) +
                                  " loops");
  }
}

} // namespace pde2d::mesh

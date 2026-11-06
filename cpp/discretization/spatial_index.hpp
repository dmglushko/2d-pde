#pragma once
#include "geometry_base.hpp"
#include <algorithm>
#include <cstddef>
#include <vector>

namespace discretization::mesh_detail {
// Inclusive AABBs are only a broad phase; exact predicates decide all contacts.
struct Box {
  double xmin, xmax, ymin, ymax;
  static Box segment(Point2 a, Point2 b) {
    return {std::min(a.x, b.x), std::max(a.x, b.x), std::min(a.y, b.y),
            std::max(a.y, b.y)};
  }
  bool overlaps(const Box &b) const {
    return xmin <= b.xmax && b.xmin <= xmax && ymin <= b.ymax && b.ymin <= ymax;
  }
};
class BoxIndex {
  struct Node {
    Box box;
    std::size_t begin, end, left = 0, right = 0;
  };
  std::vector<Box> boxes;
  std::vector<std::size_t> order;
  std::vector<Node> tree;
  std::size_t build(std::size_t begin, std::size_t end) {
    Box b = boxes[order[begin]];
    for (auto i = begin + 1; i < end; ++i) {
      auto q = boxes[order[i]];
      b = {std::min(b.xmin, q.xmin), std::max(b.xmax, q.xmax),
           std::min(b.ymin, q.ymin), std::max(b.ymax, q.ymax)};
    }
    auto n = tree.size();
    tree.push_back({b, begin, end});
    if (end - begin > 8) {
      bool x = b.xmax - b.xmin >= b.ymax - b.ymin;
      auto middle = begin + (end - begin) / 2;
      std::nth_element(order.begin() + begin, order.begin() + middle,
                       order.begin() + end, [&](auto i, auto j) {
                         double a = x ? boxes[i].xmin : boxes[i].ymin,
                                c = x ? boxes[j].xmin : boxes[j].ymin;
                         return a < c || (a == c && i < j);
                       });
      auto l = build(begin, middle), r = build(middle, end);
      tree[n].left = l;
      tree[n].right = r;
    }
    return n;
  }
  void query(std::size_t n, const Box &b, std::vector<std::size_t> &out) const {
    const auto &t = tree[n];
    if (!t.box.overlaps(b))
      return;
    if (t.left) {
      query(t.left, b, out);
      query(t.right, b, out);
    } else
      for (auto i = t.begin; i < t.end; ++i)
        if (boxes[order[i]].overlaps(b))
          out.push_back(order[i]);
  }

public:
  explicit BoxIndex(std::vector<Box> b)
      : boxes(std::move(b)), order(boxes.size()) {
    for (std::size_t i = 0; i < order.size(); ++i)
      order[i] = i;
    if (!order.empty())
      build(0, order.size());
  }
  std::vector<std::size_t> candidates(const Box &b) const {
    std::vector<std::size_t> out;
    if (!tree.empty())
      query(0, b, out);
    std::sort(out.begin(), out.end());
    return out;
  }
};
} // namespace discretization::mesh_detail

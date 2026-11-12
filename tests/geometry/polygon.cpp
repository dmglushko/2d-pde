#include "support/check.hpp"

#include "pde2d/geometry/polygon.hpp"

using namespace pde2d::geometry;
using namespace pde2d::test;

int main() {
  std::vector<Point> square = {{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  return run({
      {"signed_area",
       [&] {
         near(signed_area(square), 4, 0, "counterclockwise");
         std::vector<Point> cw(square.rbegin(), square.rend());
         near(signed_area(cw), -4, 0, "clockwise");
       }},
      {"signed_area_far_from_origin",
       [] {
         // A unit square at (1e9, 1e9): the products of absolute
         // coordinates (about 1e18) cancel, and their rounding errors are
         // larger than the area.
         std::vector<Point> far = {
             {1e9, 1e9}, {1e9 + 1, 1e9}, {1e9 + 1, 1e9 + 1}, {1e9, 1e9 + 1}};
         near(signed_area(far), 1, 0, "unit square far away");
         std::vector<Point> cw(far.rbegin(), far.rend());
         near(signed_area(cw), -1, 0, "clockwise far away");
         std::vector<Point> thin = {
             {1e15, 3}, {1e15 + 8, 3}, {1e15 + 8, 3.5}, {1e15, 3.5}};
         near(signed_area(thin), 4, 0, "thin rectangle at 1e15");
       }},
      {"orientation_at_any_scale",
       [&] {
         check(polygon_orientation(square) == 1, "counterclockwise");
         std::vector<Point> cw(square.rbegin(), square.rend());
         check(polygon_orientation(cw) == -1, "clockwise");
         // The area of these squares underflows, overflows or cancels.
         for (double s : {1e-300, 1e300, 0x1p-1070}) {
           std::vector<Point> p = {{0, 0}, {s, 0}, {s, s}, {0, s}};
           check(polygon_orientation(p) == 1, "counterclockwise at any scale");
           std::vector<Point> q(p.rbegin(), p.rend());
           check(polygon_orientation(q) == -1, "clockwise at any scale");
         }
         std::vector<Point> far = {
             {1e17, 1e17}, {1e17 + 16, 1e17}, {1e17 + 16, 1e17 + 16}};
         check(polygon_orientation(far) == 1, "far from the origin");
         // A non-convex polygon whose lowest point is a repeated vertex.
         std::vector<Point> l_shape = {{0, 0}, {0, 0}, {2, 0}, {2, 1},
                                       {1, 1}, {1, 2}, {0, 2}};
         check(polygon_orientation(l_shape) == 1, "repeated lowest point");
         check(polygon_orientation(std::vector<Point>{{0, 0}, {1, 1}}) == 0,
               "too few points");
       }},
      {"locate",
       [&] {
         check(locate_in_polygon(square, {1, 1}) == 1, "inside");
         check(locate_in_polygon(square, {3, 1}) == -1, "outside");
         check(locate_in_polygon(square, {2, 1}) == 0, "on an edge");
         check(locate_in_polygon(square, {0, 0}) == 0, "on a vertex");
         check(locate_in_polygon(square, {-1, 0}) == -1, "ray through vertex");
         check(locate_in_polygon(square, {1, 2.5}) == -1, "above");
       }},
      {"crossings",
       [&] {
         std::vector<std::vector<Point>> ok = {
             square, {{0.5, 0.5}, {0.5, 1.5}, {1.5, 1.5}, {1.5, 0.5}}};
         check(!find_crossing(ok), "nested squares");
         std::vector<std::vector<Point>> bowtie = {
             {{0, 0}, {2, 2}, {2, 0}, {0, 2}}};
         check(find_crossing(bowtie).has_value(), "bowtie");
         std::vector<std::vector<Point>> touching = {square,
                                                     {{2, 1}, {3, 0}, {3, 2}}};
         check(find_crossing(touching).has_value(), "touching loops");
         std::vector<std::vector<Point>> spike = {
             {{0, 0}, {2, 0}, {1, 0}, {1, 1}}};
         check(find_crossing(spike).has_value(), "folding back");
         std::vector<std::vector<Point>> repeated = {
             {{0, 0}, {1, 0}, {1, 0}, {0, 1}}};
         check(find_crossing(repeated).has_value(), "repeated point");
         std::vector<std::vector<Point>> collinear = {
             {{0, 0}, {1, 0}, {2, 0}, {2, 1}}};
         check(!find_crossing(collinear), "collinear neighbours are fine");
       }},
  });
}

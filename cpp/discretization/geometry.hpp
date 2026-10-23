#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace discretization {

struct Point2 { double x=0.0, y=0.0; };

struct Boundary {
  // One simple, closed boundary represented without a repeated last point.
  std::vector<Point2> points;
};

struct TriangleMesh {
  std::vector<Point2> nodes;
  // Every P1 triangle contains exactly three vertex indices.
  std::vector<std::array<std::size_t,3>> elements;
  // Vertex endpoints of exterior edges.
  std::vector<std::pair<std::size_t,std::size_t>> boundary_edges;
};

class Geometry2D {
public:
  virtual ~Geometry2D()=default;
  virtual Boundary createBoundary() const=0;
  void setCenter(double x,double y) { center_={x,y}; }
  void setCenter(Point2 c) { center_=c; }
  Point2 center() const { return center_; }
  // Rotation in radians, counterclockwise about the geometry center.
  void setRotation(double radians) {
    if(!std::isfinite(radians)) throw std::invalid_argument("Rotation must be finite");
    rotation_=radians;
  }
  double rotation() const { return rotation_; }

protected:
  Point2 transform(Point2 p) const {
    const double c=std::cos(rotation_),s=std::sin(rotation_);
    return {center_.x+c*p.x-s*p.y,center_.y+s*p.x+c*p.y};
  }

private:
  Point2 center_{};
  double rotation_=0.0;
};

class CircleGeometry final : public Geometry2D {
public:
  explicit CircleGeometry(double radius=1.0,std::size_t boundarySegments=64) {
    setRadius(radius); setBoundarySegments(boundarySegments);
  }
  void setRadius(double r) {
    if(!(r>0.0)||!std::isfinite(r)) throw std::invalid_argument("Circle radius must be finite and positive");
    radius_=r;
  }
  double radius() const { return radius_; }
  void setBoundarySegments(std::size_t n) {
    if(n<8) throw std::invalid_argument("A circle needs at least 8 boundary segments");
    boundarySegments_=n;
  }
  std::size_t boundarySegments() const { return boundarySegments_; }
  Boundary createBoundary() const override {
    Boundary b; b.points.reserve(boundarySegments_);
    for(std::size_t i=0;i<boundarySegments_;++i) {
      const double a=rotation()+2.0*pi()*static_cast<double>(i)/boundarySegments_;
      b.points.push_back({center().x+radius_*std::cos(a),center().y+radius_*std::sin(a)});
    }
    return b;
  }
private:
  static constexpr double pi(){return 3.14159265358979323846;}
  double radius_=1.0;
  std::size_t boundarySegments_=64;
};

class EllipseGeometry final : public Geometry2D {
public:
  EllipseGeometry(double a=2.0,double b=1.0,std::size_t boundarySegments=96) {
    setA(a);setB(b);setBoundarySegments(boundarySegments);
  }
  // A and B are the semi-axis lengths, not the full width and height.
  void setA(double a){if(!(a>0.0)||!std::isfinite(a))throw std::invalid_argument("Ellipse A must be finite and positive");a_=a;}
  void setB(double b){if(!(b>0.0)||!std::isfinite(b))throw std::invalid_argument("Ellipse B must be finite and positive");b_=b;}
  double A()const{return a_;} double B()const{return b_;}
  void setBoundarySegments(std::size_t n){if(n<8)throw std::invalid_argument("An ellipse needs at least 8 boundary segments");boundarySegments_=n;}
  Boundary createBoundary()const override {
    Boundary out;out.points.reserve(boundarySegments_);
    for(std::size_t i=0;i<boundarySegments_;++i){
      const double t=2.0*pi()*static_cast<double>(i)/boundarySegments_;
      out.points.push_back(transform({a_*std::cos(t),b_*std::sin(t)}));
    }
    return out;
  }
private:
  static constexpr double pi(){return 3.14159265358979323846;}
  double a_=2.0,b_=1.0;std::size_t boundarySegments_=96;
};

class RectangleGeometry final : public Geometry2D {
public:
  RectangleGeometry(double width=2.0,double height=1.0){setWidth(width);setHeight(height);}
  void setWidth(double width){if(!(width>0.0)||!std::isfinite(width))throw std::invalid_argument("Rectangle width must be finite and positive");width_=width;}
  void setHeight(double height){if(!(height>0.0)||!std::isfinite(height))throw std::invalid_argument("Rectangle height must be finite and positive");height_=height;}
  double width()const{return width_;} double height()const{return height_;}
  Boundary createBoundary()const override{
    const double x=width_/2.0,y=height_/2.0;
    Boundary b;b.points={transform({-x,-y}),transform({x,-y}),transform({x,y}),transform({-x,y})};return b;
  }
private:
  double width_=2.0,height_=1.0;
};

enum class SemicircleEnd { PositiveX, NegativeX };

// Rectangle with one short end replaced by a semicircular cap. The cap
// diameter equals rectangle height, so the total length is width + height/2.
class RectangleWithSemicircleGeometry final : public Geometry2D {
public:
  RectangleWithSemicircleGeometry(double width=2.0,double height=1.0,
      SemicircleEnd end=SemicircleEnd::PositiveX,std::size_t semicircleSegments=32){
    setWidth(width);setHeight(height);setSemicircleEnd(end);setSemicircleSegments(semicircleSegments);
  }
  void setWidth(double width){if(!(width>0.0)||!std::isfinite(width))throw std::invalid_argument("Rectangle width must be finite and positive");width_=width;}
  void setHeight(double height){if(!(height>0.0)||!std::isfinite(height))throw std::invalid_argument("Rectangle height must be finite and positive");height_=height;}
  double width()const{return width_;} double height()const{return height_;}
  void setSemicircleEnd(SemicircleEnd end){end_=end;}
  SemicircleEnd semicircleEnd()const{return end_;}
  void setSemicircleSegments(std::size_t n){if(n<4)throw std::invalid_argument("A semicircle needs at least 4 boundary segments");semicircleSegments_=n;}
  std::size_t semicircleSegments()const{return semicircleSegments_;}
  Boundary createBoundary()const override{
    constexpr double pi=3.14159265358979323846;
    const double x=width_/2.0,r=height_/2.0;
    Boundary out;out.points.reserve(semicircleSegments_+3);
    if(end_==SemicircleEnd::PositiveX){
      out.points.push_back(transform({-x,-r}));
      out.points.push_back(transform({ x,-r}));
      for(std::size_t i=1;i<=semicircleSegments_;++i){
        const double t=-pi/2.0+pi*static_cast<double>(i)/semicircleSegments_;
        out.points.push_back(transform({x+r*std::cos(t),r*std::sin(t)}));
      }
      out.points.push_back(transform({-x,r}));
    }else{
      out.points.push_back(transform({-x,-r}));
      out.points.push_back(transform({ x,-r}));
      out.points.push_back(transform({ x, r}));
      out.points.push_back(transform({-x, r}));
      for(std::size_t i=1;i<semicircleSegments_;++i){
        const double t=pi/2.0+pi*static_cast<double>(i)/semicircleSegments_;
        out.points.push_back(transform({-x+r*std::cos(t),r*std::sin(t)}));
      }
    }
    return out;
  }
private:
  double width_=2.0,height_=1.0;
  SemicircleEnd end_=SemicircleEnd::PositiveX;
  std::size_t semicircleSegments_=32;
};

// A general simple n-gon. Vertices are local coordinates relative to center;
// unequal side lengths and arbitrary concave outlines are allowed.
class PolygonGeometry final : public Geometry2D {
public:
  PolygonGeometry()=default;
  explicit PolygonGeometry(std::vector<Point2> vertices){setVertices(std::move(vertices));}
  void setVertices(std::vector<Point2> vertices){
    if(vertices.size()<3)throw std::invalid_argument("A polygon needs at least three vertices");
    for(const auto& p:vertices)if(!std::isfinite(p.x)||!std::isfinite(p.y))throw std::invalid_argument("Polygon vertices must be finite");
    vertices_=std::move(vertices);
  }
  const std::vector<Point2>& vertices()const{return vertices_;}
  Boundary createBoundary()const override{
    if(vertices_.size()<3)throw std::logic_error("Set at least three polygon vertices before meshing");
    Boundary b;b.points.reserve(vertices_.size());for(auto p:vertices_)b.points.push_back(transform(p));return b;
  }
private:
  std::vector<Point2> vertices_;
};

// Regular n-gon, centered at center(). The first vertex is oriented at the
// configured rotation angle. A triangle is n=3; a square is n=4.
class RegularPolygonGeometry final : public Geometry2D {
public:
  RegularPolygonGeometry(std::size_t sides=6,double sideLength=1.0){setSideCount(sides);setSideLength(sideLength);}
  void setSideCount(std::size_t n){if(n<3)throw std::invalid_argument("A regular polygon needs at least 3 sides");sides_=n;}
  std::size_t sideCount()const{return sides_;}
  void setSideLength(double s){if(!(s>0.0)||!std::isfinite(s))throw std::invalid_argument("Side length must be finite and positive");sideLength_=s;}
  double sideLength()const{return sideLength_;}
  Boundary createBoundary()const override{
    const double radius=sideLength_/(2.0*std::sin(pi()/sides_));
    Boundary b;b.points.reserve(sides_);
    for(std::size_t i=0;i<sides_;++i){const double a=2.0*pi()*i/sides_;b.points.push_back(transform({radius*std::cos(a),radius*std::sin(a)}));}
    return b;
  }
private:
  static constexpr double pi(){return 3.14159265358979323846;}
  std::size_t sides_=6;double sideLength_=1.0;
};

} // namespace discretization

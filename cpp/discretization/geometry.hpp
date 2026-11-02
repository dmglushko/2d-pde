#pragma once

#include "curve_geometry.hpp"

namespace discretization {

class CircleGeometry final : public CurveGeometry {
public:
  explicit CircleGeometry(double radius=1.0,std::size_t boundarySegments=64) {
    setRadius(radius); setBoundarySegments(boundarySegments);
  }
  void setRadius(double r) {
    if(!(r>0.0)||!std::isfinite(r)) throw std::invalid_argument("Circle radius must be finite and positive");
    auto next=*this; next.radius_=r; next.rebuild(); *this=std::move(next);
  }
  double radius() const { return radius_; }
  void setBoundarySegments(std::size_t n) {
    if(n<8) throw std::invalid_argument("A circle needs at least 8 boundary segments");
    boundarySegments_=n;
  }
  std::size_t boundarySegments() const { return boundarySegments_; }
  void rebuild(){ rebuildRegions({{"boundary",{EllipticalArc{{0,0},radius_,radius_,0,0,2*std::numbers::pi}}}}); }

private:
  double radius_=1.0;
  std::size_t boundarySegments_=64;
};

class EllipseGeometry final : public CurveGeometry {
public:
  EllipseGeometry(double a=2.0,double b=1.0,std::size_t boundarySegments=96) {
    setA(a);setB(b);setBoundarySegments(boundarySegments);
  }
  // A and B are the semi-axis lengths, not the full width and height.
  void setA(double a){if(!(a>0.0)||!std::isfinite(a))throw std::invalid_argument("Ellipse A must be finite and positive");auto next=*this; next.a_=a; next.rebuild(); *this=std::move(next);}
  void setB(double b){if(!(b>0.0)||!std::isfinite(b))throw std::invalid_argument("Ellipse B must be finite and positive");auto next=*this; next.b_=b; next.rebuild(); *this=std::move(next);}
  double A()const{return a_;} double B()const{return b_;}
  void setBoundarySegments(std::size_t n){if(n<8)throw std::invalid_argument("An ellipse needs at least 8 boundary segments");boundarySegments_=n;}
  void rebuild(){ rebuildRegions({{"boundary",{EllipticalArc{{0,0},a_,b_,0,0,2*std::numbers::pi}}}}); }

private:
  double a_=2.0,b_=1.0;std::size_t boundarySegments_=96;
};

class RectangleGeometry final : public CurveGeometry {
public:
  RectangleGeometry(double width=2.0,double height=1.0){setWidth(width);setHeight(height);}
  void setWidth(double width){if(!(width>0.0)||!std::isfinite(width))throw std::invalid_argument("Rectangle width must be finite and positive");auto next=*this; next.width_=width; next.rebuild(); *this=std::move(next);}
  void setHeight(double height){if(!(height>0.0)||!std::isfinite(height))throw std::invalid_argument("Rectangle height must be finite and positive");auto next=*this; next.height_=height; next.rebuild(); *this=std::move(next);}
  double width()const{return width_;} double height()const{return height_;}
  void rebuild(){ double x=width_/2,y=height_/2; rebuildRegions({{"bottom",{LineCurve{{-x,-y},{x,-y}}}},{"right",{LineCurve{{x,-y},{x,y}}}},{"top",{LineCurve{{x,y},{-x,y}}}},{"left",{LineCurve{{-x,y},{-x,-y}}}}}); }

private:
  double width_=2.0,height_=1.0;
};

enum class SemicircleEnd { PositiveX, NegativeX };

// Rectangle with one short end replaced by a semicircular cap. The cap
// diameter equals rectangle height, so the total length is width + height/2.
class RectangleWithSemicircleGeometry final : public CurveGeometry {
public:
  RectangleWithSemicircleGeometry(double width=2.0,double height=1.0,
      SemicircleEnd end=SemicircleEnd::PositiveX,std::size_t semicircleSegments=32){
    setWidth(width);setHeight(height);setSemicircleEnd(end);setSemicircleSegments(semicircleSegments);
  }
  void setWidth(double width){if(!(width>0.0)||!std::isfinite(width))throw std::invalid_argument("Rectangle width must be finite and positive");auto next=*this; next.width_=width; next.rebuild(); *this=std::move(next);}
  void setHeight(double height){if(!(height>0.0)||!std::isfinite(height))throw std::invalid_argument("Rectangle height must be finite and positive");auto next=*this; next.height_=height; next.rebuild(); *this=std::move(next);}
  double width()const{return width_;} double height()const{return height_;}
  void setSemicircleEnd(SemicircleEnd end){auto next=*this; next.end_=end; next.rebuild(); *this=std::move(next);}
  SemicircleEnd semicircleEnd()const{return end_;}
  void setSemicircleSegments(std::size_t n){if(n<4)throw std::invalid_argument("A semicircle needs at least 4 boundary segments");semicircleSegments_=n;}
  std::size_t semicircleSegments()const{return semicircleSegments_;}
  void rebuild(){ double x=width_/2,r=height_/2,pi=std::numbers::pi;
    if(end_==SemicircleEnd::PositiveX) rebuildRegions({{"bottom",{LineCurve{{-x,-r},{x,-r}}}},{"cap",{EllipticalArc{{x,0},r,r,0,-pi/2,pi}}},{"top",{LineCurve{{x,r},{-x,r}}}},{"left",{LineCurve{{-x,r},{-x,-r}}}}});
    else rebuildRegions({{"bottom",{LineCurve{{-x,-r},{x,-r}}}},{"right",{LineCurve{{x,-r},{x,r}}}},{"top",{LineCurve{{x,r},{-x,r}}}},{"cap",{EllipticalArc{{-x,0},r,r,0,pi/2,pi}}}}); }

private:
  double width_=2.0,height_=1.0;
  SemicircleEnd end_=SemicircleEnd::PositiveX;
  std::size_t semicircleSegments_=32;
};

// A general simple n-gon. Vertices are local coordinates relative to center;
// unequal side lengths and arbitrary concave outlines are allowed.
class PolygonGeometry final : public CurveGeometry {
public:
  PolygonGeometry()=default;
  explicit PolygonGeometry(std::vector<Point2> vertices){setVertices(std::move(vertices));}
  void setVertices(std::vector<Point2> vertices){
    if(vertices.size()<3)throw std::invalid_argument("A polygon needs at least three vertices");
    for(const auto& p:vertices)if(!std::isfinite(p.x)||!std::isfinite(p.y))throw std::invalid_argument("Polygon vertices must be finite");
    auto next=*this; next.vertices_=std::move(vertices); next.rebuild(); *this=std::move(next);
  }
  const std::vector<Point2>& vertices()const{return vertices_;}
  void rebuild(){ if(vertices_.size()<3) return; std::vector<CurveSegment> lines; for(std::size_t i=0;i<vertices_.size();++i) lines.push_back(LineCurve{vertices_[i],vertices_[(i+1)%vertices_.size()]}); rebuildRegions({{"boundary",std::move(lines)}}); }

private:
  std::vector<Point2> vertices_;
};

// Regular n-gon, centered at center(). The first vertex is oriented at the
// configured rotation angle. A triangle is n=3; a square is n=4.
class RegularPolygonGeometry final : public CurveGeometry {
public:
  RegularPolygonGeometry(std::size_t sides=6,double sideLength=1.0){setSideCount(sides);setSideLength(sideLength);}
  void setSideCount(std::size_t n){if(n<3)throw std::invalid_argument("A regular polygon needs at least 3 sides");auto next=*this; next.sides_=n; next.rebuild(); *this=std::move(next);}
  std::size_t sideCount()const{return sides_;}
  void setSideLength(double s){if(!(s>0.0)||!std::isfinite(s))throw std::invalid_argument("Side length must be finite and positive");auto next=*this; next.sideLength_=s; next.rebuild(); *this=std::move(next);}
  double sideLength()const{return sideLength_;}
  void rebuild(){ double r=sideLength_/(2*std::sin(std::numbers::pi/sides_)); std::vector<CurveSegment> lines; for(std::size_t i=0;i<sides_;++i){double a=2*std::numbers::pi*i/sides_,b=2*std::numbers::pi*(i+1)/sides_; lines.push_back(LineCurve{{r*std::cos(a),r*std::sin(a)},{r*std::cos(b),r*std::sin(b)}});} rebuildRegions({{"boundary",std::move(lines)}}); }

private:
  std::size_t sides_=6;double sideLength_=1.0;
};

} // namespace discretization

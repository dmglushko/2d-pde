#pragma once

#include "geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <stdexcept>
#include <vector>

namespace discretization {

enum class MeshAlgorithm {
  EarClipping,       // robust polygon triangulation; no interior refinement
  QualityEarClipping,// chooses the best-shaped legal ear at each step
  UniformRefinement, // conforming red/green splits to a global target size
  DelaunayEdgeFlips, // uniform refinement followed by local Delaunay edge flips
  LocalSizeRefinement // conforming local splits driven by h(x,y), then edge flips
};

struct MeshOptions {
  MeshAlgorithm algorithm = MeshAlgorithm::QualityEarClipping;
  double max_element_size = 1.0;
  // Optional local target size h(x,y), used by local refinement.
  std::function<double(double,double)> size_field;
  int order = 1;                    // 1: P1 (3 nodes); 2: P2 (6 nodes)
  int optimization_iterations = 5; // interior Laplacian smoothing passes
  int max_refinement_passes = 12;
};

class Triangulator2d {
public:
  static TriangleMesh generate(const Boundary& boundary, const MeshOptions& options = {}) {
    if (boundary.points.size()<3) throw std::runtime_error("A closed boundary needs at least 3 points");
    if (options.max_element_size<=0 || !std::isfinite(options.max_element_size))
      throw std::runtime_error("max_element_size must be finite and positive");
    if (options.order!=1 && options.order!=2) throw std::runtime_error("Mesh order must be 1 or 2");
    if (options.max_refinement_passes<0 || options.max_refinement_passes>20)
      throw std::runtime_error("max_refinement_passes must be between 0 and 20");
    validate_boundary(boundary);
    TriangleMesh mesh;
    mesh.nodes=boundary.points;
    const bool quality_seed=options.algorithm==MeshAlgorithm::QualityEarClipping;
    mesh.elements=ear_clip(mesh.nodes,quality_seed);
    for (std::size_t i=0;i<mesh.nodes.size();++i) mesh.boundary_edges.push_back({i,(i+1)%mesh.nodes.size()});

    if (options.algorithm!=MeshAlgorithm::EarClipping) {
      refine(mesh,options);
      if (options.algorithm==MeshAlgorithm::QualityEarClipping ||
          options.algorithm==MeshAlgorithm::DelaunayEdgeFlips ||
          options.algorithm==MeshAlgorithm::LocalSizeRefinement) flip_edges(mesh,options);
    }
    smooth(mesh,options);
    if (options.order==2) add_quadratic_nodes(mesh);
    mesh.order=options.order;
    return mesh;
  }

private:
  static double cross(Point2 a,Point2 b,Point2 c) { return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x); }
  static void validate_boundary(const Boundary& b) {
    if(b.points.size()<3)throw std::runtime_error("Geometry boundary needs at least three vertices");
    for(std::size_t i=0;i<b.points.size();++i){
      const auto a=b.points[i],c=b.points[(i+1)%b.points.size()];
      if(!std::isfinite(a.x)||!std::isfinite(a.y)||dist(a,c)<=1e-14)throw std::runtime_error("Geometry has a non-finite point or zero-length boundary edge");
      const auto prev=b.points[(i+b.points.size()-1)%b.points.size()];
      // Consecutive edges may be collinear, but must not double back.
      if(std::abs(cross(prev,a,c))<=1e-13 &&
          (prev.x-a.x)*(c.x-a.x)+(prev.y-a.y)*(c.y-a.y)>0)
        throw std::runtime_error("Geometry boundary has overlapping adjacent edges");
    }
    const double area=signed_area(b.points);
    if(!std::isfinite(area)||std::abs(area)<=1e-14)throw std::runtime_error("Geometry boundary has zero area or exceeds supported numeric scale");
    for(std::size_t i=0;i<b.points.size();++i)for(std::size_t j=i+1;j<b.points.size();++j){
      const std::size_t in=(i+1)%b.points.size(),jn=(j+1)%b.points.size();
      if(i==j||in==j||jn==i)continue;
      const auto a=b.points[i],c=b.points[in],d=b.points[j],e=b.points[jn];
      const double p=cross(a,c,d),q=cross(a,c,e),r=cross(d,e,a),s=cross(d,e,c);
      auto on_segment=[](Point2 u,Point2 v,Point2 w,double orientation){
        return std::abs(orientation)<=1e-13 &&
          w.x>=std::min(u.x,v.x)-1e-14 && w.x<=std::max(u.x,v.x)+1e-14 &&
          w.y>=std::min(u.y,v.y)-1e-14 && w.y<=std::max(u.y,v.y)+1e-14;
      };
      if(!std::isfinite(p)||!std::isfinite(q)||!std::isfinite(r)||!std::isfinite(s))
        throw std::runtime_error("Geometry exceeds supported numeric scale");
      if((((p>1e-13&&q< -1e-13)||(p< -1e-13&&q>1e-13))&&((r>1e-13&&s< -1e-13)||(r< -1e-13&&s>1e-13))) ||
          on_segment(a,c,d,p)||on_segment(a,c,e,q)||on_segment(d,e,a,r)||on_segment(d,e,c,s))
        throw std::runtime_error("Geometry boundary self-intersects");
    }
  }
  static double signed_area(const std::vector<Point2>& p) {
    // Translate to the first vertex before accumulating: the shoelace formula
    // on world coordinates loses small areas far from the origin.
    double a=0; for(std::size_t i=1;i+1<p.size();++i) a+=cross(p[0],p[i],p[i+1]);
    return 0.5*a;
  }
  static bool in_triangle(Point2 p,Point2 a,Point2 b,Point2 c) {
    constexpr double e=1e-13;
    return cross(a,b,p)>=-e && cross(b,c,p)>=-e && cross(c,a,p)>=-e;
  }
  static std::vector<std::vector<std::size_t>> ear_clip(const std::vector<Point2>& pts,bool quality) {
    std::vector<std::size_t> ring(pts.size());
    for(std::size_t i=0;i<ring.size();++i) ring[i]=i;
    // If original loop was clockwise, use its reversed index order.
    if(signed_area(pts)<0) std::reverse(ring.begin(),ring.end());
    std::vector<std::vector<std::size_t>> tris;
    std::size_t guard=0;
    while(ring.size()>3) {
      double best=quality ? -1.0 : -1e300; std::size_t chosen=ring.size();
      for(std::size_t j=0;j<ring.size();++j) {
        const auto ia=ring[(j+ring.size()-1)%ring.size()], ib=ring[j], ic=ring[(j+1)%ring.size()];
        const Point2 a=pts[ia], b=pts[ib], c=pts[ic];
        const double area=cross(a,b,c); if(area<=1e-14) continue;
        bool contains=false;
        for(auto k:ring) if(k!=ia&&k!=ib&&k!=ic&&in_triangle(pts[k],a,b,c)){contains=true;break;}
        if(contains) continue;
        // The quality-ranked front prefers well-shaped ears; plain mode takes
        // the first legal ear. This also makes concave outlines supported.
        const double ab=dist(a,b),bc=dist(b,c),ca=dist(c,a);
        const double q=2.0*std::sqrt(3.0)*area/(ab*ab+bc*bc+ca*ca);
        if(!quality) { chosen=j; break; }
        if(q>best) {best=q;chosen=j;}
      }
      if(chosen==ring.size()) throw std::runtime_error("Boundary is self-intersecting or numerically degenerate");
      const auto n=ring.size(), ia=ring[(chosen+n-1)%n], ib=ring[chosen], ic=ring[(chosen+1)%n];
      tris.push_back({ia,ib,ic}); ring.erase(ring.begin()+static_cast<std::ptrdiff_t>(chosen));
      if(++guard>pts.size()*pts.size()) throw std::runtime_error("Ear-clipping did not converge");
    }
    if(ring.size()==3) {
      if(cross(pts[ring[0]],pts[ring[1]],pts[ring[2]])<=1e-14)
        throw std::runtime_error("Final ear is numerically degenerate");
      tris.push_back({ring[0],ring[1],ring[2]});
    }
    return tris;
  }
  static double dist(Point2 a,Point2 b) { return std::hypot(a.x-b.x,a.y-b.y); }
  static double target(const MeshOptions& o,Point2 p) {
    double h=o.max_element_size;
    if(o.algorithm==MeshAlgorithm::LocalSizeRefinement && o.size_field) h=o.size_field(p.x,p.y);
    if(!(h>0) || !std::isfinite(h)) throw std::runtime_error("Size field must return finite positive sizes");
    return std::min(h,o.max_element_size);
  }
  static bool within_size(const std::vector<Point2>& nodes,const std::vector<std::size_t>& t,const MeshOptions& o) {
    if(o.algorithm==MeshAlgorithm::EarClipping) return true;
    const auto a=nodes[t[0]],b=nodes[t[1]],c=nodes[t[2]];
    const Point2 centroid{a.x+(b.x-a.x)/3.0+(c.x-a.x)/3.0,
                          a.y+(b.y-a.y)/3.0+(c.y-a.y)/3.0};
    return std::max({dist(a,b),dist(b,c),dist(c,a)})<=target(o,centroid)*(1.0+1e-12);
  }
  static void refine(TriangleMesh& m,const MeshOptions& o) {
    for(int pass=0;pass<o.max_refinement_passes;++pass) {
      bool changed=false;
      for(const auto& t:m.elements) {
        if(!within_size(m.nodes,t,o)) changed=true;
      }
      if(!changed) return;
      // Mark all three edges of oversized cells, then use red/green triangle
      // splits. Neighboring triangles receive matching midpoint nodes, so local
      // refinement stays conforming without refining the whole domain.
      std::map<std::pair<std::size_t,std::size_t>,bool> marked;
      auto key_of=[](std::size_t a,std::size_t b){if(a>b)std::swap(a,b);return std::make_pair(a,b);};
      for(const auto& t:m.elements) {
        if(!within_size(m.nodes,t,o)) for(int e=0;e<3;++e) marked[key_of(t[e],t[(e+1)%3])]=true;
      }
      std::map<std::pair<std::size_t,std::size_t>,std::size_t> mids;
      auto mid=[&](std::size_t a,std::size_t b) {
        auto key=key_of(a,b); auto it=mids.find(key); if(it!=mids.end())return it->second;
        const auto k=m.nodes.size(); m.nodes.push_back({(m.nodes[a].x+m.nodes[b].x)/2,(m.nodes[a].y+m.nodes[b].y)/2});mids[key]=k;return k;
      };
      std::vector<std::vector<std::size_t>> next; next.reserve(m.elements.size()*4);
      for(const auto& t:m.elements) {
        const bool s0=marked.count(key_of(t[0],t[1])),s1=marked.count(key_of(t[1],t[2])),s2=marked.count(key_of(t[2],t[0]));
        const int mask=(s0?1:0)|(s1?2:0)|(s2?4:0);
        if(mask==0) {next.push_back(t);continue;}
        const auto ab=s0?mid(t[0],t[1]):0,bc=s1?mid(t[1],t[2]):0,ca=s2?mid(t[2],t[0]):0;
        if(mask==1) {next.push_back({t[0],ab,t[2]});next.push_back({ab,t[1],t[2]});}
        else if(mask==2) {next.push_back({t[1],bc,t[0]});next.push_back({bc,t[2],t[0]});}
        else if(mask==4) {next.push_back({t[2],ca,t[1]});next.push_back({ca,t[0],t[1]});}
        else if(mask==3) {next.push_back({t[1],bc,ab});next.push_back({t[0],ab,t[2]});next.push_back({ab,bc,t[2]});}
        else if(mask==5) {next.push_back({t[0],ab,ca});next.push_back({ab,t[1],t[2]});next.push_back({ab,t[2],ca});}
        else if(mask==6) {next.push_back({t[2],ca,bc});next.push_back({t[1],bc,t[0]});next.push_back({bc,ca,t[0]});}
        else {next.push_back({t[0],ab,ca});next.push_back({ab,t[1],bc});next.push_back({ca,bc,t[2]});next.push_back({ab,bc,ca});}
      }
      m.elements.swap(next);
      std::vector<std::pair<std::size_t,std::size_t>> edges; edges.reserve(m.boundary_edges.size()*2);
      for(auto e:m.boundary_edges) {
        if(marked.count(key_of(e.first,e.second))) {auto k=mid(e.first,e.second);edges.push_back({e.first,k});edges.push_back({k,e.second});}
        else edges.push_back(e);
      }
      m.boundary_edges.swap(edges);
    }
    // A hard pass limit avoids unbounded work. Report if the requested size
    // could not be reached instead of silently claiming it was met.
    for(const auto& t:m.elements) {
      if(!within_size(m.nodes,t,o))
        throw std::runtime_error("Refinement pass limit reached before target element size");
    }
  }
  static void flip_edges(TriangleMesh& m,const MeshOptions& o) {
    std::map<std::pair<std::size_t,std::size_t>,bool> boundary;
    for(auto e:m.boundary_edges){if(e.first>e.second)std::swap(e.first,e.second);boundary[{e.first,e.second}]=true;}
    const std::size_t limit=std::max<std::size_t>(1,m.elements.size()*8);
    for(std::size_t flip=0;flip<limit;++flip) {
      std::map<std::pair<std::size_t,std::size_t>,std::vector<std::size_t>> owners;
      for(std::size_t i=0;i<m.elements.size();++i) for(int e=0;e<3;++e) {
        auto a=m.elements[i][e],b=m.elements[i][(e+1)%3]; if(a>b)std::swap(a,b); owners[{a,b}].push_back(i);
      }
      bool changed=false;
      for(const auto& item:owners) {
        const auto edge=item.first; const auto& own=item.second;
        if(own.size()!=2 || boundary.count(edge)) continue;
        auto& t1=m.elements[own[0]]; auto& t2=m.elements[own[1]];
        std::size_t c=t1[0],d=t2[0];
        for(auto v:t1)if(v!=edge.first&&v!=edge.second)c=v;
        for(auto v:t2)if(v!=edge.first&&v!=edge.second)d=v;
        const Point2 a=m.nodes[edge.first],b=m.nodes[edge.second],pc=m.nodes[c],pd=m.nodes[d];
        if(c==d || cross(pc,a,pd)*cross(pc,b,pd)>=0) continue; // non-convex quad
        auto angle=[](Point2 p,Point2 q,Point2 r) {
          const double ux=p.x-r.x,uy=p.y-r.y,vx=q.x-r.x,vy=q.y-r.y;
          const double den=std::hypot(ux,uy)*std::hypot(vx,vy);
          return den==0.0?0.0:std::acos(std::max(-1.0,std::min(1.0,(ux*vx+uy*vy)/den)));
        };
        if(angle(a,b,pc)+angle(a,b,pd)<=3.14159265358979323846+1e-12) continue;
        std::vector<std::size_t> next1{c,d,edge.first},next2{d,c,edge.second};
        if(cross(m.nodes[next1[0]],m.nodes[next1[1]],m.nodes[next1[2]])<0)std::swap(next1[0],next1[1]);
        if(cross(m.nodes[next2[0]],m.nodes[next2[1]],m.nodes[next2[2]])<0)std::swap(next2[0],next2[1]);
        if(!within_size(m.nodes,next1,o)||!within_size(m.nodes,next2,o)) continue;
        t1=std::move(next1);t2=std::move(next2);
        changed=true; break; // rebuild adjacency after each diagonal change
      }
      if(!changed) break;
    }
  }
  static void smooth(TriangleMesh& m,const MeshOptions& o) {
    const int iterations=o.optimization_iterations;
    if(iterations<=0) return;
    std::vector<bool> fixed(m.nodes.size(),false); for(auto e:m.boundary_edges) fixed[e.first]=fixed[e.second]=true;
    for(int it=0;it<iterations;++it) {
      std::vector<std::vector<std::size_t>> adj(m.nodes.size());
      for(auto t:m.elements) for(int e=0;e<3;++e) {auto a=t[e],b=t[(e+1)%3];adj[a].push_back(b);adj[b].push_back(a);}
      auto next=m.nodes;
      for(std::size_t i=0;i<m.nodes.size();++i) if(!fixed[i]&&!adj[i].empty()) {
        double x=0,y=0; for(auto j:adj[i]){x+=m.nodes[j].x;y+=m.nodes[j].y;}
        next[i]={x/adj[i].size(),y/adj[i].size()};
      }
      bool valid=true;
      for(const auto& t:m.elements) if(cross(next[t[0]],next[t[1]],next[t[2]])<=1e-14 || !within_size(next,t,o)) {valid=false;break;}
      if(!valid) break;
      m.nodes.swap(next);
    }
  }
  static void add_quadratic_nodes(TriangleMesh& m) {
    std::map<std::pair<std::size_t,std::size_t>,std::size_t> mids;
    auto midpoint=[&](std::size_t a,std::size_t b) {
      auto key=std::minmax(a,b); auto it=mids.find(key); if(it!=mids.end())return it->second;
      const auto i=m.nodes.size();m.nodes.push_back({(m.nodes[a].x+m.nodes[b].x)/2,(m.nodes[a].y+m.nodes[b].y)/2});mids[key]=i;return i;
    };
    for(auto& t:m.elements) {const auto a=t[0],b=t[1],c=t[2];t={a,b,c,midpoint(a,b),midpoint(b,c),midpoint(c,a)};}
    for(const auto& e:m.boundary_edges) midpoint(e.first,e.second);
  }
};

} // namespace discretization

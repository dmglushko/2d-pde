#pragma once

#include "geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace discretization {
namespace detail {

inline std::pair<std::size_t,std::size_t> edge_key(std::size_t a,std::size_t b) {
  if(a>b)std::swap(a,b);
  return {a,b};
}

inline void validate_mesh(const TriangleMesh& mesh) {
  if(mesh.order!=1&&mesh.order!=2)throw std::runtime_error("Mesh order must be 1 or 2");
  if(mesh.nodes.empty()||mesh.elements.empty())throw std::runtime_error("Mesh must contain nodes and triangles");
  if(mesh.boundary_edges.empty())throw std::runtime_error("Mesh must contain boundary edges");
  for(const auto& p:mesh.nodes)if(!std::isfinite(p.x)||!std::isfinite(p.y))throw std::runtime_error("Mesh contains a non-finite node coordinate");

  double xmin=mesh.nodes[0].x,xmax=xmin,ymin=mesh.nodes[0].y,ymax=ymin;
  for(const auto& p:mesh.nodes){xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);ymin=std::min(ymin,p.y);ymax=std::max(ymax,p.y);}
  const double span=std::max(xmax-xmin,ymax-ymin);
  if(!std::isfinite(span)||!std::isfinite(span*span))throw std::runtime_error("Mesh exceeds supported numeric scale");
  const double area_tol=1e-14*span*span;
  using Edge=std::pair<std::size_t,std::size_t>;
  std::map<Edge,std::size_t> edge_owners;
  std::map<Edge,int> edge_directions;
  std::map<Edge,std::size_t> quadratic_midpoints;
  std::map<std::size_t,Edge> midpoint_edges;
  std::set<std::size_t> corner_nodes;
  std::set<std::array<std::size_t,3>> triangle_keys;
  for(const auto& e:mesh.elements) {
    const std::size_t expected=mesh.order==1?3:6;
    if(e.size()!=expected)throw std::runtime_error("Triangle has the wrong number of nodes for its mesh order");
    for(auto i:e)if(i>=mesh.nodes.size())throw std::runtime_error("Triangle references a node outside the node array");
    std::set<std::size_t> unique(e.begin(),e.end());
    if(unique.size()!=e.size())throw std::runtime_error("Triangle repeats a node index");
    std::array<std::size_t,3> tri{{e[0],e[1],e[2]}}; auto sorted=tri;std::sort(sorted.begin(),sorted.end());
    if(!triangle_keys.insert(sorted).second)throw std::runtime_error("Mesh contains a duplicate triangle");
    const auto a=mesh.nodes[e[0]],b=mesh.nodes[e[1]],c=mesh.nodes[e[2]];
    const double twice_area=(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
    if(!std::isfinite(twice_area)||twice_area<=area_tol)throw std::runtime_error("Triangle is degenerate, clockwise, or exceeds supported numeric scale");
    corner_nodes.insert(e.begin(),e.begin()+3);
    const std::array<Edge,3> edges{{edge_key(e[0],e[1]),edge_key(e[1],e[2]),edge_key(e[2],e[0])}};
    for(std::size_t k=0;k<3;++k){
      if(++edge_owners[edges[k]]>2)throw std::runtime_error("Mesh has a non-manifold edge shared by more than two triangles");
      const int direction=e[k]<e[(k+1)%3]?1:-1;
      if(edge_owners[edges[k]]==2 && edge_directions[edges[k]]==direction)
        throw std::runtime_error("Adjacent triangles traverse a shared edge in the same direction");
      edge_directions[edges[k]]=direction;
      if(mesh.order==2){
        const std::size_t mid=e[3+k];
        auto inserted=quadratic_midpoints.emplace(edges[k],mid);
        if(!inserted.second&&inserted.first->second!=mid)throw std::runtime_error("Adjacent P2 triangles disagree on a shared edge midpoint");
        const auto owner=midpoint_edges.emplace(mid,edges[k]);
        if(!owner.second&&owner.first->second!=edges[k])throw std::runtime_error("P2 midpoint node is reused by distinct edges");
      }
    }
  }

  for(const auto& mid:midpoint_edges)if(corner_nodes.count(mid.first))
    throw std::runtime_error("P2 midpoint node is also used as a triangle corner");

  std::set<Edge> boundary;
  std::map<std::size_t,std::size_t> degree;
  for(const auto& edge:mesh.boundary_edges){
    const auto key=edge_key(edge.first,edge.second);
    if(edge.first>=mesh.nodes.size()||edge.second>=mesh.nodes.size()||edge.first==edge.second)
      throw std::runtime_error("Boundary edge has an invalid endpoint");
    if(!boundary.insert(key).second)throw std::runtime_error("Mesh contains a duplicate boundary edge");
    ++degree[edge.first];++degree[edge.second];
  }
  for(const auto& d:degree)if(d.second!=2)throw std::runtime_error("Boundary edges do not form closed loops");
  for(const auto& e:edge_owners){
    const bool is_boundary=boundary.count(e.first)!=0;
    if((e.second==1)!=is_boundary)throw std::runtime_error("Boundary-edge records do not match the exterior triangle edges");
    if(mesh.order==2&&is_boundary&&!quadratic_midpoints.count(e.first))throw std::runtime_error("P2 boundary edge is missing its midpoint node");
  }
  for(const auto& e:boundary)if(!edge_owners.count(e))throw std::runtime_error("Boundary edge is not used by any triangle");
}

inline std::size_t read_count(std::istream& in,const std::string& label) {
  std::string actual;std::size_t n=0;
  if(!(in>>actual>>n)||actual!=label)throw std::runtime_error("Expected section " + label);
  constexpr std::size_t max_records=50000000;
  if(n>max_records)throw std::runtime_error("Mesh section is too large: "+label);
  return n;
}

} // namespace detail

// Save a validated linear or quadratic triangular mesh in the project's
// versioned, zero-based-index text format.
inline void export_mesh(const TriangleMesh& mesh,const std::string& path) {
  detail::validate_mesh(mesh);
  std::ofstream out(path);
  if(!out)throw std::runtime_error("Cannot create mesh file: "+path);
  out<<"PDE2D_MESH 1\nORDER "<<mesh.order<<"\n";
  out<<"NODES "<<mesh.nodes.size()<<'\n'<<std::setprecision(17);
  for(const auto& p:mesh.nodes)out<<p.x<<' '<<p.y<<'\n';
  out<<"ELEMENTS "<<mesh.elements.size()<<'\n';
  for(const auto& e:mesh.elements){out<<e.size();for(auto i:e)out<<' '<<i;out<<'\n';}
  out<<"BOUNDARY_EDGES "<<mesh.boundary_edges.size()<<'\n';
  for(const auto& e:mesh.boundary_edges)out<<e.first<<' '<<e.second<<'\n';
  out.close(); // Detect buffered writes and close failures before reporting success.
  if(!out)throw std::runtime_error("Failed while writing mesh file: "+path);
}

// Load a mesh saved by export_mesh. Rejects truncated, malformed, unsupported,
// or structurally inconsistent data before returning it to the caller.
inline TriangleMesh import_mesh(const std::string& path) {
  std::ifstream in(path);
  if(!in)throw std::runtime_error("Cannot open mesh file: "+path);
  std::string magic;int version=0;
  if(!(in>>magic>>version)||magic!="PDE2D_MESH"||version!=1)
    throw std::runtime_error("Unsupported or invalid PDE2D mesh-file header");
  TriangleMesh mesh;std::string label;
  if(!(in>>label>>mesh.order)||label!="ORDER"||(mesh.order!=1&&mesh.order!=2))
    throw std::runtime_error("Invalid mesh order record");
  const auto node_count=detail::read_count(in,"NODES");mesh.nodes.resize(node_count);
  for(auto& p:mesh.nodes)if(!(in>>p.x>>p.y))throw std::runtime_error("Truncated or invalid node coordinates");
  const auto element_count=detail::read_count(in,"ELEMENTS");mesh.elements.resize(element_count);
  for(auto& e:mesh.elements){
    std::size_t n=0;if(!(in>>n))throw std::runtime_error("Truncated triangle record");
    if(n!=(mesh.order==1?3u:6u))throw std::runtime_error("Triangle node count does not match mesh order");
    e.resize(n);for(auto& i:e)if(!(in>>i))throw std::runtime_error("Truncated triangle connectivity");
  }
  const auto edge_count=detail::read_count(in,"BOUNDARY_EDGES");mesh.boundary_edges.resize(edge_count);
  for(auto& e:mesh.boundary_edges)if(!(in>>e.first>>e.second))throw std::runtime_error("Truncated boundary-edge connectivity");
  std::string extra;if(in>>extra)throw std::runtime_error("Unexpected content after final mesh section");
  detail::validate_mesh(mesh);
  return mesh;
}

} // namespace discretization

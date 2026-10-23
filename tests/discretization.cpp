#include "discretization/mesh_generators.hpp"
#include "discretization/mesh_io.hpp"

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

using namespace discretization;

namespace {
void require(bool condition,const std::string& message) {
  if(!condition) throw std::runtime_error(message);
}

template<class F> void rejects(F operation,const std::string& message) {
  bool threw=false;
  try { operation(); } catch(const std::runtime_error&) { threw=true; }
  require(threw,message);
}

double cross(Point2 a,Point2 b,Point2 c) {
  return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
}

void check_mesh(const TriangleMesh& mesh,const Boundary& boundary,const MeshOptions& options) {
  detail::validate_mesh(mesh);
  double boundary_area=0,mesh_area=0;
  for(std::size_t i=1;i+1<boundary.points.size();++i)
    boundary_area+=cross(boundary.points[0],boundary.points[i],boundary.points[i+1]);
  for(const auto& t:mesh.elements) {
    const auto a=mesh.nodes[t[0]],b=mesh.nodes[t[1]],c=mesh.nodes[t[2]];
    mesh_area+=cross(a,b,c);
    if(options.algorithm!=MeshAlgorithm::EarClipping) {
      const Point2 centroid{a.x+(b.x-a.x)/3+(c.x-a.x)/3,a.y+(b.y-a.y)/3+(c.y-a.y)/3};
      const double h=options.algorithm==MeshAlgorithm::LocalSizeRefinement && options.size_field
        ? std::min(options.max_element_size,options.size_field(centroid.x,centroid.y))
        : options.max_element_size;
      for(int k=0;k<3;++k) {
        const auto u=mesh.nodes[t[k]],v=mesh.nodes[t[(k+1)%3]];
        require(std::hypot(u.x-v.x,u.y-v.y)<=h*(1+1e-10),"Final mesh exceeds size target");
      }
    }
    if(mesh.order==2) for(int k=0;k<3;++k) {
      const auto u=mesh.nodes[t[k]],v=mesh.nodes[t[(k+1)%3]],mid=mesh.nodes[t[3+k]];
      require(std::hypot(mid.x-(u.x+v.x)/2,mid.y-(u.y+v.y)/2)<1e-12,"Incorrect P2 midpoint");
    }
  }
  require(std::abs(mesh_area-std::abs(boundary_area))<=1e-10*std::abs(boundary_area),"Meshing changed domain area");
}

TriangleMesh square() {
  TriangleMesh m;
  m.nodes={{0,0},{1,0},{1,1},{0,1}};
  m.elements={{0,1,2},{0,2,3}};
  m.boundary_edges={{0,1},{1,2},{2,3},{3,0}};
  return m;
}

TriangleMesh quadratic_square() {
  auto m=square();m.order=2;
  m.nodes.insert(m.nodes.end(),{{.5,0},{1,.5},{.5,.5},{.5,1},{0,.5}});
  m.elements={{0,1,2,4,5,6},{0,2,3,6,7,8}};
  return m;
}
} // namespace

int main(int argc,char** argv) {
  try {
    require(argc==2,"Usage: discretization_test TEMP_DIRECTORY");
    const auto path=(std::filesystem::path(argv[1])/"roundtrip.pde2d").string();
    std::vector<std::unique_ptr<Geometry2D>> shapes;
    shapes.push_back(std::make_unique<CircleGeometry>(1,12));
    shapes.push_back(std::make_unique<EllipseGeometry>(1.5,1,12));
    shapes.push_back(std::make_unique<RectangleGeometry>(2,1));
    shapes.push_back(std::make_unique<RectangleWithSemicircleGeometry>(2,1,SemicircleEnd::PositiveX,8));
    shapes.push_back(std::make_unique<RectangleWithSemicircleGeometry>(2,1,SemicircleEnd::NegativeX,8));
    shapes.push_back(std::make_unique<RegularPolygonGeometry>(5,1));
    shapes.push_back(std::make_unique<PolygonGeometry>(std::vector<Point2>{{0,0},{2,0},{2,1},{1,1},{1,2},{0,2}}));
    shapes.push_back(std::make_unique<PolygonGeometry>(std::vector<Point2>{{0,1},{2,1},{2,0},{1,0},{0,0}}));
    const std::vector<MeshAlgorithm> algorithms{MeshAlgorithm::EarClipping,MeshAlgorithm::QualityEarClipping,
      MeshAlgorithm::UniformRefinement,MeshAlgorithm::DelaunayEdgeFlips,MeshAlgorithm::LocalSizeRefinement};
    for(const auto& shape:shapes) for(const auto algorithm:algorithms) for(int order:{1,2}) {
      MeshOptions options;options.algorithm=algorithm;options.order=order;options.max_element_size=.6;
      options.size_field=[](double x,double){return x<0?.35:.6;};
      const auto boundary=shape->createBoundary();
      const auto mesh=Triangulator2d::generate(boundary,options);
      check_mesh(mesh,boundary,options);
      export_mesh(mesh,path);
      const auto restored=import_mesh(path);
      require(restored.order==mesh.order && restored.elements==mesh.elements &&
        restored.boundary_edges==mesh.boundary_edges && restored.nodes.size()==mesh.nodes.size(),"Round trip changed connectivity");
      for(std::size_t i=0;i<mesh.nodes.size();++i)
        require(restored.nodes[i].x==mesh.nodes[i].x && restored.nodes[i].y==mesh.nodes[i].y,"Round trip changed coordinates");
    }
    std::cout<<"80 shape/algorithm/order combinations: area, topology, final size, P2 nodes, and file round trips passed\n";

    for(bool clockwise:{false,true}) {
      auto b=RectangleGeometry(2,1).createBoundary();
      for(auto& p:b.points){p.x+=1e9;p.y+=1e9;}
      if(clockwise)std::reverse(b.points.begin(),b.points.end());
      MeshOptions options;options.algorithm=MeshAlgorithm::EarClipping;
      check_mesh(Triangulator2d::generate(b,options),b,options);
    }
    const Boundary irregular{{{1.38987,0},{.779598,.977585},{-.265557,1.16348},{-.911567,.438988},
      {-.868775,-.41838},{-.237748,-1.04164},{.500381,-.627458}}};
    for(auto algorithm:algorithms) {
      MeshOptions options;options.algorithm=algorithm;options.max_element_size=.3;
      check_mesh(Triangulator2d::generate(irregular,options),irregular,options);
    }
    for(const Boundary& bad:std::vector<Boundary>{
        {{{0,0},{2,0},{1,0},{1,1},{0,1}}}, // Adjacent edges double back.
        {{{0,0},{2,0},{2,2},{1,0},{0,2}}}, // A vertex touches a nonadjacent edge.
        {{{0,0},{2,2},{0,2},{2,0}}},       // Crossing edges.
        {{{0,0},{1,0},{2,0}}}})           // Zero area.
      rejects([&]{Triangulator2d::generate(bad);},"Invalid boundary accepted");
    MeshOptions options;options.max_refinement_passes=0;options.max_element_size=.01;
    rejects([&]{Triangulator2d::generate(shapes[2]->createBoundary(),options);},"Unmet size target accepted");
    options=MeshOptions{};options.order=3;
    rejects([&]{Triangulator2d::generate(shapes[2]->createBoundary(),options);},"Invalid order accepted");
    options=MeshOptions{};options.algorithm=MeshAlgorithm::LocalSizeRefinement;
    options.size_field=[](double,double){return std::numeric_limits<double>::quiet_NaN();};
    rejects([&]{Triangulator2d::generate(shapes[2]->createBoundary(),options);},"Invalid size field accepted");

    auto m=square();m.nodes[2]={0,1};m.nodes[3]={.5,1};m.elements={{0,1,2},{0,1,3}};
    m.boundary_edges={{1,2},{2,0},{1,3},{3,0}};
    rejects([&]{detail::validate_mesh(m);},"Same-direction shared edge accepted");
    m=quadratic_square();m.elements[1][4]=4;
    rejects([&]{detail::validate_mesh(m);},"Midpoint reused on different edges accepted");
    m=quadratic_square();m.elements[0][3]=3;
    rejects([&]{detail::validate_mesh(m);},"Midpoint/corner alias accepted");
    m=quadratic_square();m.nodes.push_back({.5,.5});m.elements[1][3]=9;
    rejects([&]{detail::validate_mesh(m);},"Unshared P2 midpoint accepted");
    m=square();m.boundary_edges.pop_back();
    rejects([&]{detail::validate_mesh(m);},"Missing boundary accepted");
    m=square();m.elements.push_back(m.elements[0]);
    rejects([&]{detail::validate_mesh(m);},"Duplicate triangle accepted");
    m=square();m.nodes[1].x=1e308;
    rejects([&]{detail::validate_mesh(m);},"Unsupported numeric scale accepted");
    {std::ofstream out(path);out<<"PDE2D_MESH 1 ORDER 1 NODES 3 0 0";}
    rejects([&]{import_mesh(path);},"Truncated file accepted");
    export_mesh(square(),path);
    {std::ofstream out(path,std::ios::app);out<<" EXTRA";}
    rejects([&]{import_mesh(path);},"Trailing data accepted");
    if(std::filesystem::exists("/dev/full"))
      rejects([&]{export_mesh(square(),"/dev/full");},"Buffered write failure ignored");
    for(int order:{1,2}) detail::validate_mesh(import_mesh("examples/mesh_viewer/rectangle_p"+std::to_string(order)+".pde2d"));
    std::cout<<"Translation, optimization size cap, invalid boundaries/topology, malformed files, write failure, and fixture checks passed\n";
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';return 1;
  }
}

#include "discretization/mesh_generators.hpp"
#include "discretization/mesh_io.hpp"
#include <iostream>
#include <limits>
#include <set>
#include <source_location>
using namespace discretization;
namespace {
void check(bool ok,const char* message) { if(!ok) throw std::runtime_error(message); }
void near(double a,double b) { check(std::abs(a-b)<2e-8*std::max(1.,std::abs(b)),"Geometry/length mismatch"); }
template<class F> void rejects(F f,std::source_location at=std::source_location::current()) {
  bool caught=false; try {f();} catch(const std::exception&) {caught=true;}
  if(!caught) throw std::runtime_error("Invalid edit accepted at line " + std::to_string(at.line()));
}
std::vector<CurveId> ids(const CurveGeometry& g) { std::vector<CurveId> out; for(const auto& c:g.curves()) out.push_back(c.id); return out; }
void unique(const CurveGeometry& g) { auto all=ids(g); std::set<CurveId> keys(all.begin(),all.end()); check(keys.size()==all.size(),"Duplicate curve IDs"); }
}
int main(int argc,char** argv) {
  try {
    check(argc==2,"Missing temp directory");
    RectangleGeometry rectangle(4,2);
    auto bottom=rectangle.curveIdAt("bottom",0),right=rectangle.curveIdAt("right",0);
    auto original=ids(rectangle);
    rectangle.renameRegion("bottom","wall");
    rectangle.renameRegion("top","wall");
    check(!rectangle.region("wall").connected(),"Disconnected group reported connected");
    check(ids(rectangle)==original,"Rename changed IDs/order");
    rejects([&]{rectangle.splitRegion("wall",.5,"a","b");});
    check(ids(rectangle)==original,"Invalid split mutated geometry");
    rectangle.moveCurve(bottom,"inlet");
    check(rectangle.curve(bottom).region=="inlet","Transfer failed");
    auto children=rectangle.splitCurve(bottom,.25,"inlet","wall");
    rejects([&]{(void)rectangle.curve(bottom);});
    check(children.size()==2 && children[0]!=children[1],"Missing split IDs");
    near(rectangle.length(children[0]),1); near(rectangle.length(children[1]),3);
    check(rectangle.curve(right).id==right,"Unselected ID changed");
    rectangle.mergeRegionInto("inlet","wall");
    rejects([&]{(void)rectangle.region("inlet");});
    check(rectangle.curve(children[0]).region=="wall","Merge into existing failed");
    rectangle.setWidth(8);
    near(rectangle.length(children[0]),2); near(rectangle.length(children[1]),6);
    check(rectangle.curve(children[0]).region=="wall","Resize lost labels");
    rectangle.setHeight(4); near(rectangle.length(right),4);
    rectangle.setCenter(10,20); rectangle.setRotation(.4); near(rectangle.length(children[0]),2);
    unique(rectangle);

    RectangleGeometry wrapping(4,2);
    wrapping.renameRegion("bottom","wall"); wrapping.renameRegion("left","wall");
    auto wrap=wrapping.region("wall");
    check(wrap.connected() && wrap.pieces[0].front()==wrapping.curveIdAt("wall",1),"Wraparound traversal incorrect");
    wrapping.splitRegion("wall",1./3,"left_part","bottom_part");
    near(wrapping.regionLength("left_part"),2); near(wrapping.regionLength("bottom_part"),4);

    PolygonGeometry polygon({{0,0},{4,0},{4,2},{0,2}});
    auto parts=polygon.splitRegion("boundary",{.25,.75},{"a","b","c"});
    near(polygon.regionLength("a"),3); near(polygon.regionLength("b"),6); near(polygon.regionLength("c"),3);
    polygon.setVertices({{0,0},{8,0},{8,2},{0,2}});
    near(polygon.regionLength("a"),5); near(polygon.regionLength("b"),10); near(polygon.regionLength("c"),5);
    auto before=ids(polygon); auto vertices=polygon.vertices();
    rejects([&]{polygon.setVertices({{0,0},{8,0},{4,2}});});
    check(ids(polygon)==before && polygon.vertices().size()==vertices.size(),"Topology failure changed polygon");
    check(!parts.empty(),"Missing region split IDs");

    // A fractional cut crosses a source junction after resizing. Its old
    // fragment cannot satisfy a later explicit-ID transfer: reject atomically.
    RectangleGeometry conflict(4,2);
    conflict.renameRegion("bottom","wall"); conflict.renameRegion("right","wall");
    auto fragments=conflict.splitRegion("wall",.5,"a","b");
    conflict.moveCurve(fragments[1],"special");
    auto conflictIds=ids(conflict);
    rejects([&]{conflict.setWidth(1);});
    near(conflict.width(),4); near(conflict.regionLength("special"),1);
    check(ids(conflict)==conflictIds,"Failed resize changed IDs");

    RectangleGeometry noConflict(4,2);
    noConflict.renameRegion("bottom","wall"); noConflict.renameRegion("right","wall");
    auto retiredBottom=noConflict.curveIdAt("wall",0);
    auto surviving=noConflict.splitRegion("wall",.5,"a","b").front();
    noConflict.setWidth(1);
    near(noConflict.regionLength("a"),1.5); near(noConflict.regionLength("b"),1.5);
    near(noConflict.length(surviving),1);
    rejects([&]{(void)noConflict.curve(retiredBottom);});
    noConflict.setWidth(4);
    near(noConflict.length(surviving),3);
    rejects([&]{(void)noConflict.curve(retiredBottom);});
    unique(noConflict);

    // Group operations retain their name-based intent when an earlier cut
    // moves across a source junction and changes the group's curve list.
    RectangleGeometry groupReplay(4,2);
    groupReplay.renameRegion("bottom","wall"); groupReplay.renameRegion("right","wall");
    groupReplay.splitRegion("wall",.5,"a","b");
    groupReplay.renameRegion("a","inlet");
    groupReplay.splitRegion("inlet",.5,"front","back");
    groupReplay.mergeRegionInto("back","b");
    groupReplay.setWidth(1);
    near(groupReplay.regionLength("front"),.75); near(groupReplay.regionLength("b"),2.25);
    rejects([&]{(void)groupReplay.region("a");});
    rejects([&]{(void)groupReplay.region("inlet");});

    RectangleWithSemicircleGeometry capped(4,2);
    capped.renameRegion("cap","outlet");
    auto capParts=capped.splitRegion("outlet",.3,"a","b");
    capped.setHeight(4); near(capped.length(capParts[0]),.6*std::numbers::pi);
    rejects([&]{capped.setSemicircleEnd(SemicircleEnd::NegativeX);});
    check(capped.semicircleEnd()==SemicircleEnd::PositiveX,"Failed cap edit changed end");
    RegularPolygonGeometry regular(5,2);
    auto polygonParts=regular.splitRegion("boundary",.2,"a","b");
    regular.setSideLength(4); near(regular.regionLength("a"),4);
    auto regularIds=ids(regular);
    rejects([&]{regular.setSideCount(6);});
    check(regular.sideCount()==5 && ids(regular)==regularIds && !polygonParts.empty(),"Failed side-count edit changed state");
    EllipseGeometry ellipse(2,1);
    auto ellipseParts=ellipse.splitRegion("boundary",.3,"a","b");
    ellipse.setA(3); ellipse.setB(2);
    near(ellipse.regionLength("a")/ellipse.regionLength("b"),.3/.7);
    check(ellipse.curve(ellipseParts.front()).region=="a","Ellipse resize lost ID");

    CircleGeometry circle(2);
    auto circleParts=circle.splitRegion("boundary",{.25,.5,.75},{"quarter","rest","rest","rest"});
    near(circle.regionLength("quarter"),std::numbers::pi);
    check(circle.region("rest").connected() && circleParts.size()==4,"Same-name outputs not grouped");
    circle.setRadius(4); near(circle.regionLength("quarter"),2*std::numbers::pi);
    check(circle.curve(circleParts.front()).region=="quarter","Resize lost split IDs");

    // A separate closed mixed outline for actual meshing and tag propagation.
    CurveGeometry meshed({{"bottom",{LineCurve{{0,0},{2,0}}}},
      {"right",{QuadraticBezier{{2,0},{2.5,1},{2,2}}}},
      {"top",{CubicBezier{{2,2},{1.5,2.5},{.5,2.5},{0,2}}}},
      {"left",{EllipticalArc{{0,1},1,1,0,std::numbers::pi/2,std::numbers::pi}}}});
    meshed.renameRegion("right","bottom");
    double combined=meshed.regionLength("bottom");
    meshed.splitRegion("bottom",.6,"inlet","wall");
    near(meshed.regionLength("inlet"),combined*.6);
    near(meshed.regionLength("wall"),combined*.4);
    auto mesh=generate_mesh(meshed).mesh;
    check(std::find(mesh.boundary_tags.begin(),mesh.boundary_tags.end(),"inlet")!=mesh.boundary_tags.end(),"Tags lost");
    auto path=std::string(argv[1])+"/edited.pde2d";
    export_mesh(mesh,path); check(import_mesh(path).boundary_tags==mesh.boundary_tags,"Edited tags lost in IO");

    RectangleGeometry invalid;
    auto unchanged=ids(invalid);
    for(const std::vector<double>& cuts:std::vector<std::vector<double>>{{},{0},{1},{.5,.5},{.7,.3},{std::numeric_limits<double>::quiet_NaN()}})
      rejects([&]{invalid.splitRegion("bottom",cuts,std::vector<std::string>(cuts.size()+1,"a"));});
    rejects([&]{invalid.splitRegion("bottom",{.5},{"a"});});
    rejects([&]{invalid.renameRegion("bottom","bad name");});
    rejects([&]{invalid.moveCurve({999999},"wall");});
    rejects([&]{invalid.mergeRegionInto("bottom","missing");});
    rejects([&]{invalid.setWidth(-1);});
    rejects([&]{invalid.setCenter(std::numeric_limits<double>::infinity(),0);});
    check(ids(invalid)==unchanged && invalid.region("bottom").curves.size()==1,"Invalid request changed state");
    auto equalNames=invalid.splitCurve(invalid.curveIdAt("bottom",0),.5,"wall","wall");
    check(equalNames.size()==2 && invalid.region("wall").curves.size()==2 && invalid.region("wall").connected(),"Equal split names failed");
    auto oldIds=ids(invalid);
    invalid.setRegions({{"new",{LineCurve{{0,0},{1,0}},LineCurve{{1,0},{0,0}}}}});
    for(auto id:oldIds) rejects([&]{(void)invalid.curve(id);});
    CurveGeometry precision({{"boundary",{LineCurve{{1,0},{2,0}},LineCurve{{2,0},{1,0}}}}});
    auto precisionIds=ids(precision);
    rejects([&]{precision.splitCurve(precisionIds.front(),1e-20,"a","b");});
    rejects([&]{precision.splitRegion("boundary",{.5,.5+1e-15},{"a","b","c"});});
    check(ids(precision)==precisionIds,"Unresolvable cut changed geometry");
    std::cout << "Stable IDs, grouped editing, length cuts, wraparound, resize replay/rollback, and mesh tags passed\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}

#include "discretization/svg_geometry.hpp"
#include <fstream>
#include <iostream>
using namespace discretization;
int main(int argc,char** argv) {
 if(argc!=2) return 1;
 std::string file=std::string(argv[1])+"/test.svg";
 auto write=[&](const std::string& s){std::ofstream f(file); f<<s;};
 auto check=[](bool b){if(!b) throw std::runtime_error("SVG test failed");};
 write("<svg><circle id='origin' cx='1' cy='1'/><path id='bottom' d='M0 0 L2 0'/><path id='top' d='M0 2 Q1 3 2 2'/><path id='right' d='M2 2 C3 2 3 0 2 0'/><path id='left' d='M0 0 A1 1 0 0 0 0 2'/></svg>");
 auto imported=importSvg(file,{2,SvgScaleMode::Factor,16});
 auto b=imported.createBoundary();
 auto importedId=imported.curveIdAt("top",0);
 imported.renameRegion("top","wall");
 imported.moveCurve(importedId,"bottom");
 check(imported.curve(importedId).region=="bottom" && imported.region("bottom").curves.size()==2);
 auto children=imported.splitCurve(importedId,.5,"a","b");
 check(children.size()==2 && children[0]!=importedId && children[1]!=importedId);
 check(b.points.size()>4 && b.edge_tags.front()=="bottom" && std::find(b.edge_tags.begin(),b.edge_tags.end(),"right")!=b.edge_tags.end());
 check(std::abs(b.points.front().x+2)<1e-9 && std::abs(b.points.front().y+2)<1e-9);
 auto scaled=importSvg(file,{3,SvgScaleMode::Farthest,16}).createBoundary(); double max=0; for(auto p:scaled.points) max=std::max(max,std::hypot(p.x,p.y)); check(std::abs(max-3)<1e-9);
 write("<svg><circle id='origin' cx='1' cy='1'/><path id='a' d='M0 0 l2 0 0 2 -2 0 z'/></svg>");
 auto near=importSvg(file,{2,SvgScaleMode::Nearest,8}).createBoundary(); check(std::abs(near.points[0].x+2)<1e-9);
 for(const std::string bad : {"<svg><path id='x' d='M0 0 H2'/></svg>", "<svg><g/></svg>", "<svg><path id='x' d='M0 0 L1 0'/></svg>", "<svg><path id='x' transform='scale(2)' d='M0 0 L1 0'/></svg>"}) { write(bad); bool rejected=false; try { (void)importSvg(file); } catch(const std::invalid_argument&) { rejected=true; } check(rejected); }
 return 0;
}

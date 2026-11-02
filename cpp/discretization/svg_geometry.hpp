#pragma once
#include "curve_geometry.hpp"
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <cctype>
#include <cstdlib>
#include <memory>
#include <set>
#include <fstream>
#include <iomanip>
#include <locale>

namespace discretization {
enum class SvgScaleMode { Factor, Nearest, Farthest };
// samples_per_curve controls distance scaling only; meshing uses BoundaryOptions.
struct SvgOptions { double scale=1; SvgScaleMode mode=SvgScaleMode::Factor; std::size_t samples_per_curve=32; };
namespace svg_detail {
inline bool near(Point2 a,Point2 b) { return std::hypot(a.x-b.x,a.y-b.y)<=1e-9*std::max({1.,std::hypot(a.x,a.y),std::hypot(b.x,b.y)}); }
inline std::string attr(xmlNode* n,const char* key) { xmlChar* p=xmlGetProp(n,BAD_CAST key); if(!p) return {}; std::string s(reinterpret_cast<char*>(p)); xmlFree(p); return s; }
struct Reader {
 const char* p;
 void space() { while(std::isspace(static_cast<unsigned char>(*p)) || *p==',') ++p; }
 bool more() { space(); return *p; }
 double number() { space(); char* e=nullptr; double x=std::strtod(p,&e); if(e==p||!std::isfinite(x)) throw std::invalid_argument("Invalid SVG number"); p=e; return x; }
 Point2 point(Point2 base,bool rel) { double x=number(),y=number(); return {x+(rel?base.x:0),y+(rel?base.y:0)}; }
};
inline CurveSegment arc(Point2 a,Point2 b,double rx,double ry,double degrees,int large,int sweep) {
 if(rx<0||ry<0 || (large!=0&&large!=1)||(sweep!=0&&sweep!=1)) throw std::invalid_argument("Invalid SVG arc");
 if(near(a,b)) throw std::invalid_argument("SVG arc endpoints coincide");
 if(rx==0||ry==0) return LineCurve{a,b};
 double phi=degrees*std::numbers::pi/180, c=std::cos(phi),s=std::sin(phi), dx=(a.x-b.x)/2,dy=(a.y-b.y)/2;
 double x=c*dx+s*dy,y=-s*dx+c*dy, lambda=x*x/(rx*rx)+y*y/(ry*ry);
 if(lambda>1) { double f=std::sqrt(lambda); rx*=f; ry*=f; }
 double num=std::max(0.,rx*rx*ry*ry-rx*rx*y*y-ry*ry*x*x), den=rx*rx*y*y+ry*ry*x*x;
 double k=(large==sweep?-1.:1.)*std::sqrt(num/den), cx=k*rx*y/ry,cy=-k*ry*x/rx;
 Point2 center{c*cx-s*cy+(a.x+b.x)/2,s*cx+c*cy+(a.y+b.y)/2};
 double ux=(x-cx)/rx,uy=(y-cy)/ry,vx=(-x-cx)/rx,vy=(-y-cy)/ry;
 double start=std::atan2(uy,ux), delta=std::atan2(ux*vy-uy*vx,ux*vx+uy*vy);
 if(!sweep&&delta>0) delta-=2*std::numbers::pi;
 if(sweep&&delta<0) delta+=2*std::numbers::pi;
 return EllipticalArc{center,rx,ry,phi,start,delta};
}
inline CurveRegion path(std::string name,const std::string& data) {
 Reader r{data.c_str()}; CurveRegion out{std::move(name),{}}; Point2 cur{},first{}; bool started=false,closed=false; char cmd=0;
 while(r.more()) {
  if(std::isalpha(static_cast<unsigned char>(*r.p))) cmd=*r.p++;
  else if(!cmd) throw std::invalid_argument("Missing SVG command");
  bool rel=std::islower(static_cast<unsigned char>(cmd)); char op=static_cast<char>(std::toupper(static_cast<unsigned char>(cmd)));
  if(op=='M') { if(started) throw std::invalid_argument("Multiple SVG subpaths unsupported"); cur=r.point(cur,rel); first=cur; started=true; cmd=rel?'l':'L'; continue; }
  if(!started||closed) throw std::invalid_argument("Invalid SVG path");
  if(op=='Z') { if(!near(cur,first)) out.segments.push_back(LineCurve{cur,first}); cur=first; closed=true; cmd=0; continue; }
  Point2 end{};
  if(op=='L') { end=r.point(cur,rel); out.segments.push_back(LineCurve{cur,end}); }
  else if(op=='Q') { auto c=r.point(cur,rel); end=r.point(cur,rel); out.segments.push_back(QuadraticBezier{cur,c,end}); }
  else if(op=='C') { auto c1=r.point(cur,rel),c2=r.point(cur,rel); end=r.point(cur,rel); out.segments.push_back(CubicBezier{cur,c1,c2,end}); }
  else if(op=='A') { double rx=r.number(),ry=r.number(),rot=r.number(),l=r.number(),s=r.number(); if((l!=0&&l!=1)||(s!=0&&s!=1)) throw std::invalid_argument("Invalid SVG arc flag"); end=r.point(cur,rel); out.segments.push_back(arc(cur,end,rx,ry,rot,static_cast<int>(l),static_cast<int>(s))); }
  else throw std::invalid_argument("Unsupported SVG path command");
  if(near(cur,end)) throw std::invalid_argument("Zero length SVG segment");
  cur=end;
 }
 if(out.segments.empty()) throw std::invalid_argument("Empty SVG path");
 return out;
}
inline void reverse(CurveRegion& r) { std::reverse(r.segments.begin(),r.segments.end()); for(auto& seg:r.segments) std::visit([](auto& c){ using T=std::decay_t<decltype(c)>; if constexpr(std::is_same_v<T,EllipticalArc>) { c.start_angle+=c.sweep_angle; c.sweep_angle=-c.sweep_angle; } else { std::swap(c.start,c.end); if constexpr(std::is_same_v<T,CubicBezier>) std::swap(c.control1,c.control2); } },seg); }
}
// Parses a single SVG document. The optional <circle id="origin" cx="..." cy="..."/> marks the origin.
inline CurveGeometry importSvg(const std::string& filename,SvgOptions options={}) {
 using namespace svg_detail;
 if(!(options.scale>0)||!std::isfinite(options.scale)||options.samples_per_curve==0) throw std::invalid_argument("Invalid SVG scale or samples");
 std::unique_ptr<xmlDoc,decltype(&xmlFreeDoc)> doc(xmlReadFile(filename.c_str(),nullptr,XML_PARSE_NONET|XML_PARSE_NOERROR|XML_PARSE_NOWARNING),xmlFreeDoc);
 if(!doc||doc->intSubset||doc->extSubset) throw std::invalid_argument("Invalid SVG XML or unsupported DTD");
 xmlNode* root=xmlDocGetRootElement(doc.get()); if(!root||xmlStrcmp(root->name,BAD_CAST "svg")) throw std::invalid_argument("Expected SVG root");
 std::vector<CurveRegion> regions; std::set<std::string> names; Point2 origin{}; bool marker=false;
 auto attributes=[](xmlNode* n,const std::set<std::string>& allowed) { for(auto a=n->properties;a;a=a->next) if(!allowed.count(reinterpret_cast<const char*>(a->name))) throw std::invalid_argument("Unsupported SVG attribute: "+std::string(reinterpret_cast<const char*>(a->name))); };
 attributes(root,{"width","height","version"});
 for(xmlNode* n=root->children;n;n=n->next) {
  if(n->type==XML_COMMENT_NODE) continue;
  if(n->type==XML_TEXT_NODE) { for(const xmlChar* p=n->content;*p;++p) if(!std::isspace(*p)) throw std::invalid_argument("Unexpected SVG text"); continue; }
  if(n->type!=XML_ELEMENT_NODE) throw std::invalid_argument("Unsupported SVG node");
  std::string type(reinterpret_cast<const char*>(n->name)),id=attr(n,"id");
  if(n->children) throw std::invalid_argument("Nested SVG content unsupported");
  if(type=="path") { attributes(n,{"id","d","data-region"}); if(id.empty()||!names.insert(id).second||id=="origin") throw std::invalid_argument("SVG paths require unique IDs"); std::string name=xmlHasProp(n,BAD_CAST "data-region")?attr(n,"data-region"):id; regions.push_back(path(name,attr(n,"d"))); }
  else if(type=="circle"&&id=="origin") { attributes(n,{"id","cx","cy","r"}); if(marker) throw std::invalid_argument("Duplicate origin"); marker=true; auto parse=[](const std::string& s) { Reader r{s.c_str()}; double v=r.number(); if(r.more()) throw std::invalid_argument("Invalid origin"); return v; }; origin={parse(attr(n,"cx")),parse(attr(n,"cy"))}; }
  else throw std::invalid_argument("Unsupported SVG element");
 }
 if(regions.empty()) throw std::invalid_argument("SVG has no paths");
 for(std::size_t i=1;i<regions.size();++i) { Point2 end=curvePoint(regions[i-1].segments.back(),1); std::size_t match=regions.size(); bool flip=false; for(std::size_t j=i;j<regions.size();++j) { bool a=near(end,curvePoint(regions[j].segments.front(),0)),b=near(end,curvePoint(regions[j].segments.back(),1)); if(a||b) { if(match!=regions.size()|| (a&&b)) throw std::invalid_argument("Ambiguous SVG boundary"); match=j; flip=b; } } if(match==regions.size()) throw std::invalid_argument("Disconnected SVG boundary"); std::swap(regions[i],regions[match]); if(flip) reverse(regions[i]); }
 if(!near(curvePoint(regions.back().segments.back(),1),curvePoint(regions.front().segments.front(),0))) throw std::invalid_argument("SVG boundary is open");
 double distance=options.mode==SvgScaleMode::Nearest?INFINITY:0;
 if(options.mode!=SvgScaleMode::Factor) for(auto& region:regions) for(auto& segment:region.segments) { std::size_t n=std::holds_alternative<LineCurve>(segment)?1:options.samples_per_curve; for(std::size_t i=0;i<n;++i) { Point2 a=curvePoint(segment,double(i)/n),b=curvePoint(segment,double(i+1)/n); double dx=b.x-a.x,dy=b.y-a.y,den=dx*dx+dy*dy; double t=den?std::clamp(((origin.x-a.x)*dx+(origin.y-a.y)*dy)/den,0.,1.):0.; double d=options.mode==SvgScaleMode::Nearest?std::hypot(a.x+t*dx-origin.x,a.y+t*dy-origin.y):std::max(std::hypot(a.x-origin.x,a.y-origin.y),std::hypot(b.x-origin.x,b.y-origin.y)); distance=options.mode==SvgScaleMode::Nearest?std::min(distance,d):std::max(distance,d); } }
 double factor=options.mode==SvgScaleMode::Factor?options.scale:options.scale/distance; if(!std::isfinite(factor)||factor<=0) throw std::invalid_argument("Invalid SVG distance scale");
 auto adjust=[&](Point2& p){p={(p.x-origin.x)*factor,(p.y-origin.y)*factor};};
 for(auto& region:regions) for(auto& segment:region.segments) std::visit([&](auto& c){ using T=std::decay_t<decltype(c)>; if constexpr(std::is_same_v<T,EllipticalArc>) { adjust(c.center); c.radius_x*=factor; c.radius_y*=factor; } else { adjust(c.start); adjust(c.end); if constexpr(std::is_same_v<T,QuadraticBezier>) adjust(c.control); if constexpr(std::is_same_v<T,CubicBezier>) { adjust(c.control1); adjust(c.control2); } } },segment);
 return CurveGeometry(std::move(regions));
}

namespace svg_detail {
// Serialize before touching the destination, including validation at the same
// coordinate precision/tolerance as importSvg. No boundary sampling is used.
inline std::string document(const CurveGeometry& geometry) {
 const auto runs=geometry.regions();
 if(runs.empty()) throw std::invalid_argument("SVG geometry requires curves");
 auto stream=[] { std::ostringstream s; s.imbue(std::locale::classic()); s<<std::setprecision(std::numeric_limits<double>::max_digits10); return s; };
 const double rotationCos=std::cos(geometry.rotation()),rotationSin=std::sin(geometry.rotation());
 auto transform=[&](Point2 p) {
  Point2 q{geometry.center().x+rotationCos*p.x-rotationSin*p.y,geometry.center().y+rotationSin*p.x+rotationCos*p.y};
  if(!std::isfinite(q.x)||!std::isfinite(q.y)) throw std::invalid_argument("SVG transformed coordinate is not finite");
  return q;
 };
 std::map<std::string,std::size_t> counts;
 std::set<std::string> used{"origin"};
 auto validId=[](const std::string& name) { return !name.empty() && (std::isalpha(static_cast<unsigned char>(name[0])) || name[0]=='_'); };
 for(const auto& r:runs) ++counts[r.name];
 for(const auto& r:runs) if(counts[r.name]==1 && validId(r.name)) used.insert(r.name);
 auto out=stream(); out<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\">\n";
 std::size_t serial=0;
 Point2 first{},previous{}; bool started=false;
 for(const auto& r:runs) {
  // CurveGeometry restricts region names to ASCII letters/digits/_.-, so
  // both the ID and data attribute are already XML-safe.
  std::string id=r.name;
  bool metadata=counts[r.name]!=1 || !validId(r.name) || r.name=="origin";
  if(metadata) { do { id="pde2d_run_"+std::to_string(++serial); } while(used.count(id)); used.insert(id); }
  auto d=stream(); bool pathStarted=false;
  std::vector<CurveSegment> segments;
  auto append=[&](auto&& self,const CurveSegment& segment,int depth)->void {
   curve_detail::validateCurve(segment);
   // SVG cannot express a coincident-end Bezier in this import subset.
   // De Casteljau subdivision preserves it analytically instead.
   if((std::holds_alternative<QuadraticBezier>(segment) || std::holds_alternative<CubicBezier>(segment)) &&
      near(transform(curvePoint(segment,0)),transform(curvePoint(segment,1)))) {
    if(depth==0) throw std::invalid_argument("SVG Bezier is below supported endpoint precision");
    auto halves=splitCurve(segment,.5);
    self(self,halves.first,depth-1); self(self,halves.second,depth-1);
   } else segments.push_back(segment);
  };
  for(const auto& segment:r.segments) append(append,segment,12);
  for(const auto& segment:segments) {
   curve_detail::validateCurve(segment);
   Point2 begin=transform(curvePoint(segment,0)),end=transform(curvePoint(segment,1));
   if(started && !near(previous,begin)) throw std::invalid_argument("SVG curve boundary is disconnected");
   if(!started) { first=begin; started=true; }
   if(!pathStarted) { d<<"M "<<begin.x<<' '<<begin.y; pathStarted=true; }
   auto point=[&](Point2 p) { auto q=transform(p); d<<' '<<q.x<<' '<<q.y; };
   std::visit([&](const auto& c) {
    using T=std::decay_t<decltype(c)>;
    if constexpr(std::is_same_v<T,LineCurve>) { d<<" L"; point(c.end); }
    else if constexpr(std::is_same_v<T,QuadraticBezier>) { d<<" Q"; point(c.control); point(c.end); }
    else if constexpr(std::is_same_v<T,CubicBezier>) { d<<" C"; point(c.control1); point(c.control2); point(c.end); }
    else {
     // Quarter-turn maximum avoids coincident endpoints for full arcs and
     // the numerically sensitive SVG diameter/large-arc case.
     const int pieces=static_cast<int>(std::ceil(std::abs(c.sweep_angle)/(std::numbers::pi/2)));
     const double axisCos=std::cos(c.rotation),axisSin=std::sin(c.rotation);
     const double degrees=std::atan2(rotationSin*axisCos+rotationCos*axisSin,
                                     rotationCos*axisCos-rotationSin*axisSin)*180/std::numbers::pi;
     for(int i=1;i<=pieces;++i) {
      d<<" A "<<c.radius_x<<' '<<c.radius_y<<' '<<degrees<<" 0 "<<(c.sweep_angle>0?1:0);
      point(curvePoint(segment,double(i)/pieces));
     }
    }
   },segment);
   previous=end;
  }
  // Reject curves too small for the importer's endpoint tolerance rather
  // than emitting unusable paths.
  const auto parsed=path(r.name,d.str());
  for(const auto& segment:parsed.segments) curve_detail::validateCurve(segment);
  out<<"  <path id=\""<<id<<'"';
  if(metadata) out<<" data-region=\""<<r.name<<'"';
  out<<" d=\""<<d.str()<<"\"/>\n";
 }
 if(!near(previous,first)) throw std::invalid_argument("SVG curve boundary is open");
 out<<"</svg>\n";
 return out.str();
}
}

// Export current analytical curves and labels in world coordinates. SVG is a
// geometry interchange format; CurveIds and edit history are not serialized.
// See README.md for data-region and arc segmentation roundtrip semantics.
inline void exportSvg(const CurveGeometry& geometry,std::ostream& output) {
 const auto data=svg_detail::document(geometry);
 output<<data;
 if(!output) throw std::runtime_error("Failed to write SVG geometry");
}
inline void exportSvg(const CurveGeometry& geometry,const std::string& filename) {
 const auto data=svg_detail::document(geometry);
 std::ofstream output(filename,std::ios::binary);
 if(!output) throw std::runtime_error("Failed to open SVG geometry output: "+filename);
 output<<data;
 output.close();
 if(!output) throw std::runtime_error("Failed to write SVG geometry: "+filename);
}
}

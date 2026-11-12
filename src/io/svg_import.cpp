#include "pde2d/io/svg.hpp"

#include "svg_path.hpp"
#include "text.hpp"

#include <libxml/parser.h>
#include <libxml/tree.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace pde2d::io {

using geometry::Point;

namespace {

constexpr const char *svg_namespace = "http://www.w3.org/2000/svg";

std::string text(const xmlChar *s) {
  return s ? std::string(reinterpret_cast<const char *>(s)) : std::string();
}

std::string element_name(const xmlNode *n) { return text(n->name); }

bool has_attribute(xmlNode *n, const char *name) {
  return xmlHasProp(n, reinterpret_cast<const xmlChar *>(name)) != nullptr;
}

std::string attribute(xmlNode *n, const char *name) {
  xmlChar *value = xmlGetProp(n, reinterpret_cast<const xmlChar *>(name));
  std::string s = text(value);
  xmlFree(value);
  return s;
}

// A run of curves of one region: one subpath of one <path>.
struct Run {
  std::vector<geometry::Curve> curves;
  std::string region;
  bool closed = false;
  Point start{}, end{};
};

class Reader {
public:
  std::vector<Run> runs;

  void element(xmlNode *n) {
    if (n->ns && text(n->ns->href) != svg_namespace)
      return; // another vocabulary (an editor's metadata, for example)
    const std::string name = element_name(n);
    if (name == "title" || name == "desc" || name == "metadata" ||
        name == "defs" || name == "style")
      return;
    if (has_attribute(n, "transform"))
      throw std::invalid_argument(
          "SVG <" + name +
          "> has a transform attribute, which is not supported; apply the "
          "transform to the path data first");
    if (name == "g") {
      children(n);
    } else if (name == "path") {
      path(n);
    } else if (name == "rect" || name == "circle" || name == "ellipse" ||
               name == "line" || name == "polyline" || name == "polygon") {
      throw std::invalid_argument("SVG <" + name +
                                  "> is not supported; convert it to a "
                                  "<path>");
    } else {
      throw std::invalid_argument("SVG element <" + name +
                                  "> is not supported");
    }
  }

  void children(xmlNode *n) {
    for (xmlNode *c = n->children; c; c = c->next)
      if (c->type == XML_ELEMENT_NODE)
        element(c);
  }

private:
  std::size_t paths_ = 0;

  void path(xmlNode *n) {
    ++paths_;
    const std::string id = attribute(n, "id");
    const std::string label = id.empty() ? "SVG path " + std::to_string(paths_)
                                         : "SVG path '" + id + "'";
    std::string region;
    if (has_attribute(n, "data-region"))
      region = attribute(n, "data-region");
    else if (has_attribute(n, "id"))
      region = id;
    else
      throw std::invalid_argument(label +
                                  " has no id or data-region attribute for "
                                  "its region name");
    if (!geometry::is_valid_region_name(region))
      throw std::invalid_argument(
          label + ": region name '" + region +
          "' is invalid; use ASCII letters, digits, '_', '.' or '-'");
    if (!has_attribute(n, "d"))
      throw std::invalid_argument(label + " has no d attribute");
    std::vector<detail::SvgSubpath> subpaths;
    try {
      subpaths = detail::parse_path_data(attribute(n, "d"));
    } catch (const std::invalid_argument &e) {
      throw std::invalid_argument(label + ": " + e.what());
    }
    for (auto &s : subpaths)
      runs.push_back({std::move(s.curves), region, s.closed, s.start, s.end});
  }
};

// Joins runs into closed loops (see import_svg).
std::vector<geometry::Loop> join_runs(std::vector<Run> runs) {
  double xmin = INFINITY, xmax = -INFINITY, ymin = INFINITY, ymax = -INFINITY;
  for (const auto &r : runs)
    for (const auto &c : r.curves)
      for (Point p : {geometry::start_point(c), geometry::end_point(c)}) {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
      }
  const double tolerance = 1e-9 * std::hypot(xmax - xmin, ymax - ymin);
  auto near = [&](Point a, Point b) {
    return geometry::distance(a, b) <= tolerance;
  };
  auto describe = [](Point p) {
    return "(" + detail::shortest(p.x) + ", " + detail::shortest(p.y) + ")";
  };
  // Z closes a subpath with a line where it does not end at its start.
  for (auto &r : runs)
    if (r.closed && !near(r.end, r.start)) {
      r.curves.push_back(geometry::Line{r.end, r.start});
      r.end = r.start;
    }

  std::vector<geometry::Loop> loops;
  std::vector<bool> used(runs.size(), false);
  for (std::size_t first = 0; first < runs.size(); ++first) {
    if (used[first])
      continue;
    used[first] = true;
    geometry::Loop loop;
    auto append = [&](Run &r) {
      for (auto &c : r.curves)
        loop.curves.push_back({std::move(c), r.region});
    };
    const Point start = runs[first].start;
    Point end = runs[first].end;
    append(runs[first]);
    while (!near(end, start)) {
      std::size_t match = runs.size();
      bool reverse = false;
      for (std::size_t j = 0; j < runs.size(); ++j) {
        if (used[j])
          continue;
        for (bool at_end : {false, true})
          if (near(end, at_end ? runs[j].end : runs[j].start)) {
            if (match != runs.size())
              throw std::invalid_argument(
                  "SVG boundary is ambiguous: more than two path ends meet "
                  "at " +
                  describe(end));
            match = j;
            reverse = at_end;
          }
      }
      if (match == runs.size())
        throw std::invalid_argument("SVG boundary is open: nothing continues "
                                    "it at " +
                                    describe(end));
      Run &r = runs[match];
      used[match] = true;
      if (reverse) {
        std::reverse(r.curves.begin(), r.curves.end());
        for (auto &c : r.curves)
          c = geometry::reversed(c);
        std::swap(r.start, r.end);
      }
      end = r.end;
      append(r);
    }
    loops.push_back(std::move(loop));
  }
  return loops;
}

geometry::CurveGeometry import_document(const std::string &document) {
  if (document.size() > static_cast<std::size_t>(INT_MAX))
    throw std::invalid_argument("SVG document is too large");
  std::unique_ptr<xmlDoc, decltype(&xmlFreeDoc)> doc(
      xmlReadMemory(document.data(), static_cast<int>(document.size()), nullptr,
                    nullptr,
                    XML_PARSE_NONET | XML_PARSE_NOERROR | XML_PARSE_NOWARNING),
      xmlFreeDoc);
  if (!doc)
    throw std::invalid_argument("SVG document is not well-formed XML");
  if (doc->intSubset || doc->extSubset)
    throw std::invalid_argument("SVG documents with a DTD are not supported");
  xmlNode *root = xmlDocGetRootElement(doc.get());
  if (!root || element_name(root) != "svg" ||
      (root->ns && text(root->ns->href) != svg_namespace))
    throw std::invalid_argument("The root element is not <svg>");
  if (has_attribute(root, "transform"))
    throw std::invalid_argument(
        "SVG <svg> has a transform attribute, which is not supported");
  Reader reader;
  reader.children(root);
  if (reader.runs.empty())
    throw std::invalid_argument("SVG document has no path with curves");
  // Runs are joined in the file's coordinates, so error messages name
  // points as they are written there; then y is flipped to point up.
  auto loops = join_runs(std::move(reader.runs));
  for (auto &loop : loops)
    for (auto &c : loop.curves)
      c.curve = detail::mirrored_y(c.curve);
  try {
    return geometry::CurveGeometry(std::move(loops));
  } catch (const std::invalid_argument &e) {
    throw std::invalid_argument(std::string("SVG geometry is invalid: ") +
                                e.what());
  }
}

} // namespace

geometry::CurveGeometry import_svg(std::istream &in) {
  return import_document(detail::read_all(in));
}

geometry::CurveGeometry import_svg(const std::filesystem::path &path) {
  auto in = detail::open_for_reading(path);
  return import_svg(in);
}

} // namespace pde2d::io

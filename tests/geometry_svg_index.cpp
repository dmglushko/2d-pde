#include "discretization/svg_geometry.hpp"
#include <filesystem>
#include <iostream>
using namespace discretization;
void check(bool ok) {
  if (!ok)
    throw std::runtime_error("Indexed SVG ordering mismatch");
}
int main(int argc, char **argv) {
  if (argc != 2)
    throw std::runtime_error("Supply private temporary directory");
  auto file = std::filesystem::path(argv[1]) / "geometry-index.svg";
  auto load = [&](const std::string &body) {
    std::ofstream out(file);
    out << "<svg>" << body << "</svg>";
    out.close();
    return importSvg(file.string());
  };
  auto rejects = [&](const std::string &body) {
    bool caught = false;
    try {
      load(body);
    } catch (const std::invalid_argument &) {
      caught = true;
    }
    check(caught);
  };
  auto g = load("<path id=\"first\" data-region=\"a\" d=\"M0 0 L1 0\"/>"
                "<path id=\"third\" data-region=\"a\" d=\"M0 1 L1 1\"/>"
                "<path id=\"last\" d=\"M0 0 L0 1\"/>"
                "<path id=\"second\" d=\"M1 1 L1 0\"/>");
  check(g.curves()[0].region == "a" && g.curves()[1].region == "second" &&
        g.curves()[2].region == "a");
  for (std::size_t i = 0; i < 4; ++i)
    check(svg_detail::near(curvePoint(g.curves()[i].segment, 1),
                           curvePoint(g.curves()[(i + 1) % 4].segment, 0)));
  // A path matching at BOTH endpoints must remain ambiguous.
  rejects(
      "<path id=\"a\" d=\"M0 0 L1 0\"/><path id=\"b\" d=\"M1 0 L2 1 L1 0\"/>");
  // Multiple paths at a matching endpoint, duplicate IDs and open boundaries.
  rejects("<path id=\"a\" d=\"M0 0 L1 0\"/><path id=\"b\" d=\"M1 0 L0 "
          "1\"/><path id=\"c\" d=\"M1 0 L0 0\"/>");
  rejects("<path id=\"a\" d=\"M0 0 L1 0\"/><path id=\"a\" d=\"M1 0 L0 0\"/>");
  rejects("<path id=\"a\" d=\"M0 0 L1 0\"/>");
  // Overflowing arc construction takes the exhaustive nonfinite fallback and
  // must still reject the malformed boundary.
  rejects("<path id=\"a\" d=\"M0 0 A1e200 1e200 0 0 1 1 0\"/><path id=\"b\" "
          "d=\"M1 0 L0 0\"/>");
  // At a large translation, candidate expansion must use relative near
  // tolerance.
  auto translated = load(
      "<path id=\"a\" d=\"M1000000000 0 L1000000100 0\"/>"
      "<path id=\"c\" d=\"M1000000100 100 L1000000000 100 L1000000000 0\"/>"
      "<path id=\"b\" d=\"M1000000100.5 0 L1000000100 100\"/>");
  check(translated.curves()[1].region == "b");
  std::cout << "Indexed SVG reversal, anchor, repeated labels, relative "
               "tolerance and ambiguity passed\n";
}

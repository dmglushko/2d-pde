#include "discretization/svg_geometry.hpp"
#include "support/check.hpp"
#include "support/geometry.hpp"
#include <filesystem>
#include <iostream>
using namespace discretization;
using pde2d_test::check;
using pde2d_test::expect_throws;
void indexed_svg_ordering(const std::string &dir) {
  auto file = std::filesystem::path(dir) / "geometry-index.svg";
  auto load = [&](const std::string &body) {
    std::ofstream out(file);
    out << "<svg>" << body << "</svg>";
    out.close();
    return importSvg(file.string());
  };
  auto rejects = [&](const std::string &body) {
    expect_throws<std::invalid_argument>([&] { load(body); });
  };
  auto g = load("<path id=\"first\" data-region=\"a\" d=\"M0 0 L1 0\"/>"
                "<path id=\"third\" data-region=\"a\" d=\"M0 1 L1 1\"/>"
                "<path id=\"last\" d=\"M0 0 L0 1\"/>"
                "<path id=\"second\" d=\"M1 1 L1 0\"/>");
  check(g.curves()[0].region == "a" && g.curves()[1].region == "second" &&
        g.curves()[2].region == "a");
  for (std::size_t i = 0; i < 4; ++i)
    check(pde2d_test::points_near(
        curvePoint(g.curves()[i].segment, 1),
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
}
int main(int argc, char **argv) {
  return pde2d_test::run(
      {{"indexed_svg_ordering", [&] {
          pde2d_test::check(argc == 2, "Supply private temporary directory");
          indexed_svg_ordering(argv[1]);
        }}});
}

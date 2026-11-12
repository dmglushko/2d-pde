#include "svg_path.hpp"

#include "text.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace pde2d::io::detail {

using geometry::Point;

namespace {

bool is_space(char c) {
  return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f';
}

bool is_digit(char c) { return c >= '0' && c <= '9'; }

// Reads the tokens of path data: command letters, numbers and arc flags.
class Scanner {
public:
  explicit Scanner(std::string_view text) : text_(text) {}

  // Skips white space and commas between tokens.
  void skip_separators() {
    while (at_ < text_.size() && (is_space(text_[at_]) || text_[at_] == ','))
      ++at_;
  }
  bool done() {
    skip_separators();
    return at_ >= text_.size();
  }
  // True if the next token is a number (not a command letter).
  bool number_next() {
    skip_separators();
    if (at_ >= text_.size())
      return false;
    char c = text_[at_];
    return is_digit(c) || c == '.' || c == '-' || c == '+';
  }
  char command() {
    skip_separators();
    char c = text_[at_];
    if (!std::isalpha(static_cast<unsigned char>(c)))
      fail("expected a command letter");
    ++at_;
    return c;
  }
  // number: [sign] (digits [. digits*] | . digits) [(e|E) [sign] digits]
  double number() {
    skip_separators();
    std::size_t begin = at_;
    if (at_ < text_.size() && (text_[at_] == '+' || text_[at_] == '-'))
      ++at_;
    std::size_t digits = 0;
    while (at_ < text_.size() && is_digit(text_[at_]))
      ++at_, ++digits;
    if (at_ < text_.size() && text_[at_] == '.') {
      ++at_;
      while (at_ < text_.size() && is_digit(text_[at_]))
        ++at_, ++digits;
    }
    if (digits == 0) {
      at_ = begin;
      fail("expected a number");
    }
    if (at_ < text_.size() && (text_[at_] == 'e' || text_[at_] == 'E')) {
      std::size_t mark = at_++;
      if (at_ < text_.size() && (text_[at_] == '+' || text_[at_] == '-'))
        ++at_;
      if (at_ < text_.size() && is_digit(text_[at_])) {
        while (at_ < text_.size() && is_digit(text_[at_]))
          ++at_;
      } else {
        at_ = mark; // "e" belongs to what follows
      }
    }
    double value = 0;
    if (!parse_double(text_.substr(begin, at_ - begin), value))
      fail("number '" + std::string(text_.substr(begin, at_ - begin)) +
           "' is not a finite double");
    return value;
  }
  // An arc flag: a single 0 or 1 (it may be followed directly by the next
  // number, as in "a1 1 0 011 1").
  bool flag() {
    skip_separators();
    if (at_ < text_.size() && (text_[at_] == '0' || text_[at_] == '1'))
      return text_[at_++] == '1';
    fail("expected an arc flag 0 or 1");
    return false;
  }
  Point point() {
    double x = number();
    double y = number();
    return {x, y};
  }

  [[noreturn]] void fail(const std::string &what) const {
    throw std::invalid_argument("SVG path data: " + what + " at character " +
                                std::to_string(at_ + 1));
  }

private:
  std::string_view text_;
  std::size_t at_ = 0;
};

bool all_equal(std::initializer_list<Point> points) {
  return std::all_of(points.begin(), points.end(),
                     [&](Point p) { return p == *points.begin(); });
}

} // namespace

geometry::Curve svg_arc(Point from, Point to, double rx, double ry,
                        double rotation_degrees, bool large_arc, bool sweep) {
  rx = std::abs(rx);
  ry = std::abs(ry);
  if (rx == 0 || ry == 0)
    return geometry::Line{from, to};
  const double phi = rotation_degrees * std::numbers::pi / 180;
  const double c = std::cos(phi), s = std::sin(phi);
  // Step 1: the midpoint of the chord in the ellipse's axes.
  const double dx = (from.x - to.x) / 2, dy = (from.y - to.y) / 2;
  const double x = c * dx + s * dy, y = -s * dx + c * dy;
  // Radii too small are scaled up until the ellipse just reaches.
  const double lambda = x * x / (rx * rx) + y * y / (ry * ry);
  if (lambda > 1) {
    rx *= std::sqrt(lambda);
    ry *= std::sqrt(lambda);
  }
  // Step 2: the center in the ellipse's axes.
  const double num =
      std::max(0.0, rx * rx * ry * ry - rx * rx * y * y - ry * ry * x * x);
  const double den = rx * rx * y * y + ry * ry * x * x;
  const double k = (large_arc == sweep ? -1.0 : 1.0) * std::sqrt(num / den);
  const double cx = k * rx * y / ry, cy = -k * ry * x / rx;
  // Step 3: the center in user coordinates.
  const Point center{c * cx - s * cy + (from.x + to.x) / 2,
                     s * cx + c * cy + (from.y + to.y) / 2};
  // Step 4: start angle and sweep.
  const double ux = (x - cx) / rx, uy = (y - cy) / ry;
  const double vx = (-x - cx) / rx, vy = (-y - cy) / ry;
  const double start = std::atan2(uy, ux);
  double delta = std::atan2(ux * vy - uy * vx, ux * vx + uy * vy);
  if (!sweep && delta > 0)
    delta -= 2 * std::numbers::pi;
  if (sweep && delta < 0)
    delta += 2 * std::numbers::pi;
  return geometry::EllipticalArc{center, rx, ry, phi, start, delta};
}

std::vector<SvgSubpath> parse_path_data(std::string_view data) {
  Scanner in(data);
  std::vector<SvgSubpath> subpaths;
  SvgSubpath current;
  bool open = false; // a subpath has been started by M
  Point cur{}, start{};
  // The control point to reflect for S (after C/S) and T (after Q/T).
  char previous = 0;
  Point last_control{};

  // Ends the current subpath at the current point.
  auto finish = [&] {
    current.end = cur;
    if (!current.curves.empty())
      subpaths.push_back(std::move(current));
    current = SvgSubpath{};
  };
  auto add = [&](geometry::Curve curve) { current.curves.push_back(curve); };

  if (in.done())
    in.fail("the path data is empty");
  char cmd = in.command();
  if (cmd != 'M' && cmd != 'm')
    in.fail("path data must start with M or m");
  while (true) {
    const bool rel = std::islower(static_cast<unsigned char>(cmd)) != 0;
    const char op =
        static_cast<char>(std::toupper(static_cast<unsigned char>(cmd)));
    const Point base = rel ? cur : Point{};
    auto point = [&] {
      Point p = in.point();
      return Point{p.x + base.x, p.y + base.y};
    };
    char done_as = op;
    switch (op) {
    case 'M': {
      if (open)
        finish();
      cur = start = point();
      current.start = cur;
      open = true;
      // Further coordinate pairs are implicit line commands.
      cmd = rel ? 'l' : 'L';
      break;
    }
    case 'Z': {
      if (!open)
        in.fail("Z without a subpath");
      current.closed = true;
      finish(); // ends where the last segment ended
      cur = start;
      // A command after Z (other than M) starts a new subpath at the same
      // start point.
      current.start = start;
      break;
    }
    case 'L': {
      Point end = point();
      if (!(end == cur))
        add(geometry::Line{cur, end});
      cur = end;
      break;
    }
    case 'H': {
      double v = in.number();
      Point end{rel ? cur.x + v : v, cur.y};
      if (!(end == cur))
        add(geometry::Line{cur, end});
      cur = end;
      break;
    }
    case 'V': {
      double v = in.number();
      Point end{cur.x, rel ? cur.y + v : v};
      if (!(end == cur))
        add(geometry::Line{cur, end});
      cur = end;
      break;
    }
    case 'C':
    case 'S': {
      Point c1;
      if (op == 'C')
        c1 = point();
      else
        c1 = (previous == 'C' || previous == 'S')
                 ? Point{2 * cur.x - last_control.x, 2 * cur.y - last_control.y}
                 : cur;
      Point c2 = point();
      Point end = point();
      if (!all_equal({cur, c1, c2, end}))
        add(geometry::CubicBezier{cur, c1, c2, end});
      last_control = c2;
      cur = end;
      break;
    }
    case 'Q':
    case 'T': {
      Point c;
      if (op == 'Q')
        c = point();
      else
        c = (previous == 'Q' || previous == 'T')
                ? Point{2 * cur.x - last_control.x, 2 * cur.y - last_control.y}
                : cur;
      Point end = point();
      if (!all_equal({cur, c, end}))
        add(geometry::QuadraticBezier{cur, c, end});
      last_control = c;
      cur = end;
      break;
    }
    case 'A': {
      double rx = in.number(), ry = in.number(), rotation = in.number();
      bool large = in.flag(), sweep = in.flag();
      Point end = point();
      // An arc with equal end points is omitted (SVG 1.1, F.6.2).
      if (!(end == cur))
        add(svg_arc(cur, end, rx, ry, rotation, large, sweep));
      cur = end;
      break;
    }
    default:
      in.fail(std::string("unsupported path command '") + cmd + "'");
    }
    previous = done_as;
    if (op == 'Z') {
      if (in.done())
        break;
      if (in.number_next())
        in.fail("numbers after Z");
      cmd = in.command();
      if (cmd != 'M' && cmd != 'm')
        open = true; // continues from the closed subpath's start
      continue;
    }
    if (in.done())
      break;
    // Implicit repetition: more numbers repeat the command.
    if (!in.number_next())
      cmd = in.command();
  }
  if (open)
    finish();
  return subpaths;
}

} // namespace pde2d::io::detail

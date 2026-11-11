#pragma once

// A tiny test helper without dependencies. A test program calls run() with
// named cases; each case uses check(), near() and expect_throws(). run()
// prints "PASS <case>" or "FAIL <case>: <reason>" and returns the exit
// code for main().

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <source_location>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace pde2d::test {

struct Failure : std::runtime_error {
  using std::runtime_error::runtime_error;
};

inline std::string location(const std::source_location &where) {
  return std::string(where.file_name()) + ":" + std::to_string(where.line());
}

inline void
check(bool condition, std::string_view what,
      std::source_location where = std::source_location::current()) {
  if (!condition)
    throw Failure(location(where) + ": check failed: " + std::string(what));
}

inline void near(double actual, double expected, double tolerance,
                 std::string_view what,
                 std::source_location where = std::source_location::current()) {
  if (!(std::abs(actual - expected) <= tolerance)) {
    std::ostringstream s;
    s.precision(17);
    s << location(where) << ": " << what << ": got " << actual << ", expected "
      << expected << " (tolerance " << tolerance << ")";
    throw Failure(s.str());
  }
}

// Expects f() to throw an exception of type E whose message contains
// `fragment`.
template <class E = std::exception, class F>
void expect_throws(
    F &&f, std::string_view fragment,
    std::source_location where = std::source_location::current()) {
  try {
    f();
  } catch (const Failure &) {
    throw;
  } catch (const E &e) {
    if (std::string_view(e.what()).find(fragment) == std::string_view::npos)
      throw Failure(location(where) + ": message '" + e.what() +
                    "' does not contain '" + std::string(fragment) + "'");
    return;
  } catch (const std::exception &e) {
    throw Failure(location(where) + ": unexpected exception type: " + e.what());
  }
  throw Failure(location(where) + ": expected an exception containing '" +
                std::string(fragment) + "'");
}

using Case = std::pair<std::string_view, std::function<void()>>;

inline int run(const std::vector<Case> &cases) {
  int failures = 0;
  for (const auto &[name, body] : cases) {
    try {
      body();
      std::cout << "PASS " << name << '\n';
    } catch (const std::exception &e) {
      ++failures;
      std::cout << "FAIL " << name << ": " << e.what() << '\n';
    }
  }
  std::cout << (cases.size() - static_cast<std::size_t>(failures)) << '/'
            << cases.size() << " cases passed\n";
  return failures == 0 ? 0 : 1;
}

} // namespace pde2d::test

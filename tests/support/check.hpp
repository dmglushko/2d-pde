#pragma once
// Minimal dependency-free test support: assertions that throw with a source
// location, tolerance comparisons, expect_throws and a named-case runner.
// Eigen comparisons live in check_eigen.hpp, geometry helpers in geometry.hpp.
#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <source_location>
#include <stdexcept>
#include <string>
#include <string_view>

namespace pde2d_test {

struct Failure : std::runtime_error {
  using std::runtime_error::runtime_error;
};

inline std::string where(const std::source_location &at) {
  return std::string(" [") + at.file_name() + ":" + std::to_string(at.line()) +
         "]";
}

[[noreturn]] inline void
fail(std::string_view message,
     std::source_location at = std::source_location::current()) {
  throw Failure(std::string(message) + where(at));
}

inline void check(bool ok, std::string_view message = "check failed",
                  std::source_location at = std::source_location::current()) {
  if (!ok)
    fail(message, at);
}

namespace detail {
inline std::string describe(long double a, long double b, long double tol) {
  return "values " + std::to_string(static_cast<double>(a)) + " and " +
         std::to_string(static_cast<double>(b)) + " differ by more than " +
         std::to_string(static_cast<double>(tol));
}
} // namespace detail

// |a - b| <= tol.
inline void near_abs(long double a, long double b, long double tol,
                     std::source_location at = std::source_location::current()) {
  if (!(std::abs(a - b) <= tol))
    fail("near_abs: " + detail::describe(a, b, tol), at);
}

// |a - b| <= tol * |b| (pure relative; b is the reference value).
inline void near_rel(long double a, long double b, long double tol,
                     std::source_location at = std::source_location::current()) {
  if (!(std::abs(a - b) <= tol * std::abs(b)))
    fail("near_rel: " + detail::describe(a, b, tol), at);
}

// |a - b| <= tol * max(1, |a|, |b|): absolute near zero, relative elsewhere.
inline void near(long double a, long double b, long double tol,
                 std::source_location at = std::source_location::current()) {
  const long double scale =
      std::max({1.L, std::abs(a), std::abs(b)});
  if (!(std::abs(a - b) <= tol * scale))
    fail("near: " + detail::describe(a, b, tol), at);
}

// Passes when f() throws Exception (and, if given, what() contains fragment).
// Other exception types propagate and fail the enclosing case.
template <class Exception = std::exception, class F>
void expect_throws(
    F &&f, std::string_view fragment = {},
    std::source_location at = std::source_location::current()) {
  try {
    f();
  } catch (const Exception &e) {
    if (!fragment.empty() &&
        std::string_view(e.what()).find(fragment) == std::string_view::npos)
      fail("unexpected diagnostic \"" + std::string(e.what()) +
               "\" (expected to contain \"" + std::string(fragment) + "\")",
           at);
    return;
  }
  fail("expected an exception but none was thrown", at);
}

// Returns what() of the std::exception thrown by f(); fails if f() returns.
template <class F>
std::string
thrown_what(F &&f, std::source_location at = std::source_location::current()) {
  try {
    f();
  } catch (const std::exception &e) {
    return e.what();
  }
  fail("expected an exception but none was thrown", at);
}

struct Case {
  std::string_view name;
  std::function<void()> body;
};

// Runs every case, reports each one, and returns nonzero if any failed.
inline int run(std::initializer_list<Case> cases) {
  int failures = 0;
  for (const Case &c : cases) {
    try {
      c.body();
      std::cout << "PASS " << c.name << '\n';
    } catch (const std::exception &e) {
      ++failures;
      std::cerr << "FAIL " << c.name << ": " << e.what() << '\n';
    } catch (...) {
      ++failures;
      std::cerr << "FAIL " << c.name << ": unknown exception\n";
    }
  }
  return failures == 0 ? 0 : 1;
}

} // namespace pde2d_test

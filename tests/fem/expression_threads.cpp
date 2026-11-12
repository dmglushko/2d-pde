// Evaluates every allowed construct of an Expression from several threads
// at once and compares the results with serial ones. On its own it checks
// the values; run under ThreadSanitizer or Helgrind it also looks for data
// races (shared state written during a call).
#include "pde2d/fem/compiled_expression.hpp"
#include "pde2d/fem/expression.hpp"
#include "support/check.hpp"

#include <array>
#include <cstddef>
#include <thread>
#include <vector>

using namespace pde2d::fem;
using pde2d::test::check;

namespace {

// Together these use every construct of the allow-list (expression.hpp).
constexpr const char *texts[] = {
    "x + y - t",
    "x*y/(t + 2) - 3",
    "x^2 + y**3 + 2^t",
    "sqrt(abs(x) + 1)",
    "2.5*pi + E + 1/3 + x",
    "sin(x) + cos(y) + tan(t)",
    "asin(x/4) + acos(y/4) + atan(t)",
    "sinh(x) + cosh(y) + tanh(t)",
    "exp(x) + log(abs(y) + 1) + ln(2 + t)",
    "Piecewise((1, x < y), (2, x <= t), (3, x > 0), (4, y >= 0), "
    "(5, Eq(x, t)), (6, Ne(y, t)), (7, True))",
    "Piecewise((1, And(x < 0, y < 0)), (2, Or(x < 0, t < 0)), "
    "(3, Not(And(y < 0, t > 0))), (x*y, True))",
    "Piecewise((x, x < 0), (Piecewise((y, y < 0), (t, True)), True))",
};

constexpr std::array<double, 5> values{-1.5, -0.5, 0.0, 0.5, 1.5};

} // namespace

int main() {
  return pde2d::test::run({
      {"compiled expressions are evaluated from several threads at once",
       [] {
         std::vector<CompiledExpression> functions;
         for (const char *text : texts)
           functions.emplace_back(Expression(text), Parameters{});
         std::vector<double> expected;
         for (const auto &f : functions)
           for (const double x : values)
             for (const double y : values)
               for (const double t : values)
                 expected.push_back(f(x, y, t));

         constexpr int thread_count = 4;
         std::array<std::size_t, thread_count> mismatches{};
         {
           std::vector<std::jthread> threads;
           for (int i = 0; i < thread_count; ++i)
             threads.emplace_back([&, i] {
               for (int repeat = 0; repeat < 20; ++repeat) {
                 std::size_t k = 0;
                 for (const auto &f : functions)
                   for (const double x : values)
                     for (const double y : values)
                       for (const double t : values)
                         if (f(x, y, t) != expected[k++])
                           ++mismatches[static_cast<std::size_t>(i)];
               }
             });
         }
         for (const std::size_t count : mismatches)
           check(count == 0, "the same values in every thread");
       }},
  });
}

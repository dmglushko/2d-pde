#pragma once
// How well a judgement about a problem or a choice is supported. Shared by
// problem analysis (analysis.hpp) and decision records (decision.hpp), so a
// reader never has to guess whether a statement was proven or estimated.
#include <string>

namespace pde2d::fem {
enum class Evidence {
  // Established exactly: symbolic algebra, or a fact of the input itself.
  Proven,
  // Checked numerically at sample points (quadrature points, edge samples,
  // chosen times, one assembled matrix). Not a proof between samples.
  Sampled,
  // Not established; `value` is a default or a best guess.
  Unknown
};

inline const char *evidence_name(Evidence evidence) {
  switch (evidence) {
  case Evidence::Proven:
    return "Proven";
  case Evidence::Sampled:
    return "Sampled";
  case Evidence::Unknown:
    return "Unknown";
  }
  return "Invalid";
}

// One judged property: its value, how it was established and a short reason
// a person can read.
template <class T> struct Property {
  T value{};
  Evidence evidence = Evidence::Unknown;
  std::string reason{};
};
} // namespace pde2d::fem

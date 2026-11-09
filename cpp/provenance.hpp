#pragma once
// Reproducibility building blocks shared by meshing and FEM: a portable
// deterministic fingerprint, stable descriptors of user strategies and the
// library version.
//
//   Fingerprint f;            // FNV-1a, 64 bit, independent of the platform
//   f.add("label").add(1.5);  // doubles by bit pattern, strings
//   length-prefixed f.hex();                  // "c0ffee..." (16 lowercase hex
//   digits)
//
// A fingerprint identifies inputs, it is not a cryptographic hash: equal
// inputs give equal fingerprints on every platform and build, different
// inputs almost always differ.
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#if __has_include(<pde2d_version.hpp>)
#include <pde2d_version.hpp>
#endif

namespace pde2d {
class Fingerprint {
  std::uint64_t state_ = 14695981039346656037ull; // FNV-1a offset basis

  void byte(unsigned char b) {
    state_ ^= b;
    state_ *= 1099511628211ull; // FNV-1a prime
  }

public:
  // Integers as 64-bit little-endian bytes, so the value does not depend on
  // the host or on the integer type's width.
  template <std::unsigned_integral U> Fingerprint &add(U value) {
    const auto wide = static_cast<std::uint64_t>(value);
    for (int k = 0; k < 8; ++k)
      byte(static_cast<unsigned char>(wide >> (8 * k)));
    return *this;
  }
  template <std::signed_integral S> Fingerprint &add(S value) {
    return add(static_cast<std::uint64_t>(static_cast<std::int64_t>(value)));
  }
  Fingerprint &add(bool value) { return add(std::uint64_t{value ? 1u : 0u}); }
  // By bit pattern: -0.0 and 0.0 differ, as do distinct NaN payloads.
  Fingerprint &add(double value) {
    return add(std::bit_cast<std::uint64_t>(value));
  }
  // Raw bytes, without a length prefix (plain FNV-1a of the bytes).
  Fingerprint &add_bytes(std::string_view bytes) {
    for (const char c : bytes)
      byte(static_cast<unsigned char>(c));
    return *this;
  }
  // Length-prefixed, so ("ab", "c") and ("a", "bc") differ.
  Fingerprint &add(std::string_view text) {
    add(static_cast<std::uint64_t>(text.size()));
    return add_bytes(text);
  }
  Fingerprint &add(const char *text) { return add(std::string_view(text)); }
  std::uint64_t value() const noexcept { return state_; }
  std::string hex() const {
    static constexpr char digits[] = "0123456789abcdef";
    std::string out(16, '0');
    for (int k = 0; k < 16; ++k)
      out[15 - k] = digits[(state_ >> (4 * k)) & 0xf];
    return out;
  }
};

// One parameter of a strategy, as text the strategy itself formats (use a
// round-trip format for numbers, e.g. std::format("{}", value)).
struct StrategyParameter {
  std::string name, value;
  bool operator==(const StrategyParameter &) const = default;
};
// Stable description of a user strategy (linear solver backend, time
// integrator, step controller, error estimator, size field). Factories are
// std::function objects and cannot be replayed from a record; a descriptor
// at least says what ran, so a reader can rebuild it. Strategies without
// one are recorded as "not replayable".
struct StrategyDescriptor {
  std::string name;
  std::vector<StrategyParameter> parameters{};
  bool operator==(const StrategyDescriptor &) const = default;
};

// "Name(a=1, b=2)", or "Name" without parameters: a readable label of a
// descriptor for decision records.
inline std::string describe_strategy(const StrategyDescriptor &descriptor) {
  std::string text = descriptor.name;
  if (descriptor.parameters.empty())
    return text;
  text += '(';
  for (std::size_t k = 0; k < descriptor.parameters.size(); ++k) {
    if (k)
      text += ", ";
    text +=
        descriptor.parameters[k].name + '=' + descriptor.parameters[k].value;
  }
  return text + ')';
}

// Version of this library: the CMake project version and the git commit at
// configure time. Both are "unknown" (git_sha empty) when the generated
// header is absent (header-only use without CMake) or the source tree is
// not a git checkout.
struct LibraryVersion {
  std::string version, git_sha;
  bool operator==(const LibraryVersion &) const = default;
};
inline LibraryVersion library_version() {
#ifdef PDE2D_VERSION_STRING
  return {PDE2D_VERSION_STRING, PDE2D_GIT_SHA};
#else
  return {"unknown", ""};
#endif
}
} // namespace pde2d

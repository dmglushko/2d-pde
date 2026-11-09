// Reproducibility building blocks (provenance.hpp). Oracles: the published
// FNV-1a 64-bit test vectors, invariants of the integer and string
// encodings, and the configure-time version header's documented shape.
#include "provenance.hpp"
#include "support/check.hpp"
#include <cctype>
#include <cstdint>
#include <string>

using pde2d_test::check;
namespace {
void fnv1a_test_vectors() {
  using pde2d::Fingerprint;
  // FNV-1a 64 reference values (Fowler/Noll/Vo test suite).
  check(Fingerprint{}.hex() == "cbf29ce484222325", "Offset basis");
  check(Fingerprint{}.add_bytes("a").hex() == "af63dc4c8601ec8c", "\"a\"");
  check(Fingerprint{}.add_bytes("foobar").hex() == "85944171f73967e8",
        "\"foobar\"");
}

void encodings_are_portable() {
  using pde2d::Fingerprint;
  // Every integer type is hashed as the same 64-bit little-endian value.
  const auto wide = Fingerprint{}.add(std::uint64_t{7}).value();
  check(Fingerprint{}.add(std::size_t{7}).value() == wide &&
            Fingerprint{}.add(unsigned{7}).value() == wide &&
            Fingerprint{}.add(7).value() == wide,
        "Integer width must not matter");
  check(Fingerprint{}.add_bytes(std::string("\x07\0\0\0\0\0\0\0", 8)).value() ==
            wide,
        "Integers are little-endian bytes");
  // Strings are length-prefixed.
  check(Fingerprint{}.add("ab").add("c").value() !=
            Fingerprint{}.add("a").add("bc").value(),
        "Concatenation must not collide");
  // Doubles by bit pattern.
  check(Fingerprint{}.add(0.0).value() != Fingerprint{}.add(-0.0).value(),
        "Signed zeros differ");
  check(Fingerprint{}.add(0.1).value() == Fingerprint{}.add(0.1).value(),
        "Deterministic");
}

void descriptors_and_version() {
  const pde2d::StrategyDescriptor d{"Theta", {{"theta", "0.5"}, {"k", "2"}}};
  check(pde2d::describe_strategy(d) == "Theta(theta=0.5, k=2)", "Label");
  check(pde2d::describe_strategy({"Plain"}) == "Plain", "Label, no parameters");
  const auto v = pde2d::library_version();
  check(!v.version.empty(), "A version, \"unknown\" at worst");
  bool hex = v.git_sha.size() == 40;
  for (const char c : v.git_sha)
    hex = hex && std::isxdigit(static_cast<unsigned char>(c));
  check(v.git_sha.empty() || hex, "git SHA: empty or 40 hex digits");
}
} // namespace
int main() {
  return pde2d_test::run(
      {{"fnv1a_test_vectors", fnv1a_test_vectors},
       {"encodings_are_portable", encodings_are_portable},
       {"descriptors_and_version", descriptors_and_version}});
}

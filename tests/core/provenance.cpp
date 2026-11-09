// Reproducibility building blocks (provenance.hpp). Oracles: the published
// FNV-1a 64-bit test vectors, invariants of the integer and string
// encodings, and the configure-time version header's documented shape. The
// shared JSON document (json.hpp) is checked here too: the round trip,
// escapes, the strict parser and the record diff.
#include "json.hpp"
#include "provenance.hpp"
#include "support/check.hpp"
#include <cctype>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

using pde2d_test::check;
using pde2d_test::expect_throws;
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

void library_version() {
  const auto v = pde2d::library_version();
  check(!v.version.empty(), "A version, \"unknown\" at worst");
  bool hex = v.git_sha.size() == 40;
  for (const char c : v.git_sha)
    hex = hex && std::isxdigit(static_cast<unsigned char>(c));
  check(v.git_sha.empty() || hex, "git SHA: empty or 40 hex digits");
}

void json_documents() {
  using pde2d::JsonValue;
  auto doc = JsonValue::object();
  doc.set("a", pde2d::json_number(0.1));
  doc.set("b", pde2d::json_string("x\"y\n"));
  auto list = JsonValue::array();
  list.push(pde2d::json_bool(true)).push(JsonValue::null());
  doc.set("list", list);
  auto timings = JsonValue::object();
  timings.set("total_seconds", pde2d::json_number(1.5));
  doc.set("timings", timings);
  const auto text = pde2d::json_text(doc);
  const auto back = pde2d::parse_json(text);
  check(back == doc && pde2d::json_text(back) == text, "Round trip");
  check(back.at("a").number() == 0.1, "max_digits10 round-trips doubles");
  check(pde2d::json_number(std::numeric_limits<double>::infinity()).text ==
            "inf",
        "Non-finite as text");
  check(pde2d::parse_json("\"a\\u00e9\\ud83d\\ude00\"").text ==
            "a\xc3\xa9\xf0\x9f\x98\x80",
        "Unicode escapes decode to UTF-8");
  expect_throws<std::invalid_argument>(
      [] { (void)pde2d::parse_json("{\"a\": 1,}"); }, "Invalid JSON");
  expect_throws<std::invalid_argument>([] { (void)pde2d::parse_json("[1] x"); },
                                       "trailing characters");

  // The diff skips timings unless asked, and reports absent leaves.
  auto other = doc;
  other.members[3].second.members[0].second = pde2d::json_number(2.5);
  check(pde2d::diff_records(doc, other).empty(), "Timings ignored");
  const auto with_timings =
      pde2d::diff_records(doc, other, {.ignore_timings = false});
  check(with_timings.size() == 1 &&
            with_timings[0].path == "timings.total_seconds" &&
            with_timings[0].left == "1.5" && with_timings[0].right == "2.5",
        "Timings compared on request");
  other.set("c", pde2d::json_number(3));
  other.members[2].second.items[1] = pde2d::json_number(7);
  const auto diff = pde2d::diff_records(doc, other);
  check(diff.size() == 2 && diff[0].path == "list[1]" &&
            diff[0].left == "null" && diff[0].right == "7" &&
            diff[1].path == "c" && diff[1].left == "<absent>",
        "Changed and added leaves");
}
} // namespace
int main() {
  return pde2d_test::run({{"fnv1a_test_vectors", fnv1a_test_vectors},
                          {"encodings_are_portable", encodings_are_portable},
                          {"library_version", library_version},
                          {"json_documents", json_documents}});
}

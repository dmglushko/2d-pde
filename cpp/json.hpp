#pragma once
// Minimal JSON document shared by both modules (mesh run records,
// discretization/mesh_record_json.hpp; FEM run records, fem/record_json.hpp;
// reference comparisons, fem/reference/compare.hpp): a value tree, a
// deterministic writer, a strict parser and a leaf-by-leaf record diff. Numbers
// keep their text, so a parsed document writes back byte for byte and two
// documents compare exactly; json_number() formats doubles with the classic
// locale and max_digits10 digits, which round-trips every finite double.
// Non-finite doubles have no JSON number and are written as the strings "nan",
// "inf" and "-inf".
//
//   JsonValue v = JsonValue::object();
//   v.set("nodes", json_number(std::size_t{9}));
//   write_json(out, v);                        // two-space indentation
//   JsonValue back = parse_json(text);         // throws "Invalid JSON: ..."
//   for (const auto &d : diff_records(a, b))  // differing leaves, timings
//     std::cout << d.path << ": " << d.left << " -> " << d.right << '\n';
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <istream>
#include <iterator>
#include <limits>
#include <locale>
#include <map>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace pde2d {
class JsonValue {
public:
  enum class Kind { Null, Bool, Number, String, Array, Object };
  Kind kind = Kind::Null;
  bool boolean = false;
  // Number: its literal token. String: the decoded text.
  std::string text{};
  std::vector<JsonValue> items{};                           // Array
  std::vector<std::pair<std::string, JsonValue>> members{}; // Object, in order

  static JsonValue null() { return {}; }
  static JsonValue array() {
    JsonValue v;
    v.kind = Kind::Array;
    return v;
  }
  static JsonValue object() {
    JsonValue v;
    v.kind = Kind::Object;
    return v;
  }
  static JsonValue string(std::string text) {
    JsonValue v;
    v.kind = Kind::String;
    v.text = std::move(text);
    return v;
  }
  static JsonValue boolean_value(bool value) {
    JsonValue v;
    v.kind = Kind::Bool;
    v.boolean = value;
    return v;
  }
  // `token` must be a valid JSON number literal.
  static JsonValue number_token(std::string token) {
    JsonValue v;
    v.kind = Kind::Number;
    v.text = std::move(token);
    return v;
  }

  // Object: append a member (keys are not deduplicated; callers keep them
  // unique). Array: append an item.
  JsonValue &set(std::string key, JsonValue value) {
    members.emplace_back(std::move(key), std::move(value));
    return *this;
  }
  JsonValue &push(JsonValue value) {
    items.push_back(std::move(value));
    return *this;
  }
  const JsonValue *find(std::string_view key) const {
    for (const auto &[name, value] : members)
      if (name == key)
        return &value;
    return nullptr;
  }
  const JsonValue &at(std::string_view key) const {
    if (const auto *value = find(key))
      return *value;
    throw std::out_of_range("JSON object has no member " + std::string(key));
  }
  // The value of a number (std::from_chars, locale independent).
  double number() const {
    if (kind != Kind::Number)
      throw std::invalid_argument("JSON value is not a number");
    double value = 0;
    const auto *end = text.data() + text.size();
    const auto [ptr, error] = std::from_chars(text.data(), end, value);
    if (error != std::errc() || ptr != end)
      throw std::invalid_argument("Invalid JSON number: " + text);
    return value;
  }
  bool operator==(const JsonValue &) const = default;
};

inline JsonValue json_number(double value) {
  if (std::isnan(value))
    return JsonValue::string("nan");
  if (std::isinf(value))
    return JsonValue::string(value > 0 ? "inf" : "-inf");
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out.precision(std::numeric_limits<double>::max_digits10);
  out << value;
  return JsonValue::number_token(out.str());
}
inline JsonValue json_number(std::size_t value) {
  return JsonValue::number_token(std::to_string(value));
}
inline JsonValue json_number(int value) {
  return JsonValue::number_token(std::to_string(value));
}
inline JsonValue json_string(std::string_view text) {
  return JsonValue::string(std::string(text));
}
inline JsonValue json_bool(bool value) {
  return JsonValue::boolean_value(value);
}

namespace json_detail {
inline void escape(std::string &out, std::string_view text) {
  static constexpr char hex[] = "0123456789abcdef";
  out += '"';
  for (const char c : text) {
    const auto u = static_cast<unsigned char>(c);
    switch (c) {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\t':
      out += "\\t";
      break;
    case '\r':
      out += "\\r";
      break;
    default:
      if (u < 0x20) {
        out += "\\u00";
        out += hex[u >> 4];
        out += hex[u & 0xf];
      } else
        out += c;
    }
  }
  out += '"';
}
inline void write(std::string &out, const JsonValue &v, std::size_t depth) {
  const auto indent = [&](std::size_t d) { out.append(2 * d, ' '); };
  switch (v.kind) {
  case JsonValue::Kind::Null:
    out += "null";
    return;
  case JsonValue::Kind::Bool:
    out += v.boolean ? "true" : "false";
    return;
  case JsonValue::Kind::Number:
    out += v.text;
    return;
  case JsonValue::Kind::String:
    escape(out, v.text);
    return;
  case JsonValue::Kind::Array:
    if (v.items.empty()) {
      out += "[]";
      return;
    }
    out += "[\n";
    for (std::size_t k = 0; k < v.items.size(); ++k) {
      indent(depth + 1);
      write(out, v.items[k], depth + 1);
      out += k + 1 < v.items.size() ? ",\n" : "\n";
    }
    indent(depth);
    out += ']';
    return;
  case JsonValue::Kind::Object:
    if (v.members.empty()) {
      out += "{}";
      return;
    }
    out += "{\n";
    for (std::size_t k = 0; k < v.members.size(); ++k) {
      indent(depth + 1);
      escape(out, v.members[k].first);
      out += ": ";
      write(out, v.members[k].second, depth + 1);
      out += k + 1 < v.members.size() ? ",\n" : "\n";
    }
    indent(depth);
    out += '}';
    return;
  }
}

class Parser {
  std::string_view in_;
  std::size_t at_ = 0;

  [[noreturn]] void fail(const char *what) const {
    throw std::invalid_argument("Invalid JSON: " + std::string(what) +
                                " at offset " + std::to_string(at_));
  }
  void skip() {
    while (at_ < in_.size() && (in_[at_] == ' ' || in_[at_] == '\n' ||
                                in_[at_] == '\t' || in_[at_] == '\r'))
      ++at_;
  }
  bool consume(std::string_view word) {
    if (in_.substr(at_, word.size()) != word)
      return false;
    at_ += word.size();
    return true;
  }
  void expect(char c) {
    skip();
    if (at_ >= in_.size() || in_[at_] != c)
      fail("unexpected character");
    ++at_;
  }
  unsigned hex4() {
    if (at_ + 4 > in_.size())
      fail("truncated escape");
    unsigned value = 0;
    for (int k = 0; k < 4; ++k) {
      const char c = in_[at_++];
      value <<= 4;
      if (c >= '0' && c <= '9')
        value |= unsigned(c - '0');
      else if (c >= 'a' && c <= 'f')
        value |= unsigned(c - 'a' + 10);
      else if (c >= 'A' && c <= 'F')
        value |= unsigned(c - 'A' + 10);
      else
        fail("invalid escape");
    }
    return value;
  }
  static void utf8(std::string &out, unsigned code) {
    if (code < 0x80)
      out += char(code);
    else if (code < 0x800) {
      out += char(0xc0 | (code >> 6));
      out += char(0x80 | (code & 0x3f));
    } else if (code < 0x10000) {
      out += char(0xe0 | (code >> 12));
      out += char(0x80 | ((code >> 6) & 0x3f));
      out += char(0x80 | (code & 0x3f));
    } else {
      out += char(0xf0 | (code >> 18));
      out += char(0x80 | ((code >> 12) & 0x3f));
      out += char(0x80 | ((code >> 6) & 0x3f));
      out += char(0x80 | (code & 0x3f));
    }
  }
  std::string string_body() {
    expect('"');
    std::string out;
    while (true) {
      if (at_ >= in_.size())
        fail("unterminated string");
      const char c = in_[at_++];
      if (c == '"')
        return out;
      if (static_cast<unsigned char>(c) < 0x20)
        fail("control character in string");
      if (c != '\\') {
        out += c;
        continue;
      }
      if (at_ >= in_.size())
        fail("truncated escape");
      switch (const char e = in_[at_++]) {
      case '"':
      case '\\':
      case '/':
        out += e;
        break;
      case 'b':
        out += '\b';
        break;
      case 'f':
        out += '\f';
        break;
      case 'n':
        out += '\n';
        break;
      case 'r':
        out += '\r';
        break;
      case 't':
        out += '\t';
        break;
      case 'u': {
        unsigned code = hex4();
        if (code >= 0xd800 && code < 0xdc00) {
          if (!consume("\\u"))
            fail("unpaired surrogate");
          const unsigned low = hex4();
          if (low < 0xdc00 || low >= 0xe000)
            fail("unpaired surrogate");
          code = 0x10000 + ((code - 0xd800) << 10) + (low - 0xdc00);
        }
        utf8(out, code);
        break;
      }
      default:
        fail("invalid escape");
      }
    }
  }
  JsonValue number() {
    const auto begin = at_;
    auto digits = [&] {
      const auto first = at_;
      while (at_ < in_.size() && in_[at_] >= '0' && in_[at_] <= '9')
        ++at_;
      if (at_ == first)
        fail("invalid number");
    };
    if (at_ < in_.size() && in_[at_] == '-')
      ++at_;
    if (at_ < in_.size() && in_[at_] == '0')
      ++at_;
    else
      digits();
    if (at_ < in_.size() && in_[at_] == '.') {
      ++at_;
      digits();
    }
    if (at_ < in_.size() && (in_[at_] == 'e' || in_[at_] == 'E')) {
      ++at_;
      if (at_ < in_.size() && (in_[at_] == '+' || in_[at_] == '-'))
        ++at_;
      digits();
    }
    return JsonValue::number_token(std::string(in_.substr(begin, at_ - begin)));
  }

public:
  explicit Parser(std::string_view in) : in_(in) {}
  JsonValue value(std::size_t depth = 0) {
    if (depth > 256)
      fail("nesting too deep");
    skip();
    if (at_ >= in_.size())
      fail("unexpected end");
    const char c = in_[at_];
    if (c == '{') {
      ++at_;
      auto v = JsonValue::object();
      skip();
      if (at_ < in_.size() && in_[at_] == '}') {
        ++at_;
        return v;
      }
      while (true) {
        skip();
        auto key = string_body();
        expect(':');
        v.set(std::move(key), value(depth + 1));
        skip();
        if (at_ < in_.size() && in_[at_] == ',') {
          ++at_;
          continue;
        }
        expect('}');
        return v;
      }
    }
    if (c == '[') {
      ++at_;
      auto v = JsonValue::array();
      skip();
      if (at_ < in_.size() && in_[at_] == ']') {
        ++at_;
        return v;
      }
      while (true) {
        v.push(value(depth + 1));
        skip();
        if (at_ < in_.size() && in_[at_] == ',') {
          ++at_;
          continue;
        }
        expect(']');
        return v;
      }
    }
    if (c == '"')
      return JsonValue::string(string_body());
    if (consume("true"))
      return json_bool(true);
    if (consume("false"))
      return json_bool(false);
    if (consume("null"))
      return JsonValue::null();
    if (c == '-' || (c >= '0' && c <= '9'))
      return number();
    fail("unexpected character");
  }
  void finish() {
    skip();
    if (at_ != in_.size())
      fail("trailing characters");
  }
};
} // namespace json_detail

inline std::string json_text(const JsonValue &value) {
  std::string out;
  json_detail::write(out, value, 0);
  out += '\n';
  return out;
}
inline std::ostream &write_json(std::ostream &out, const JsonValue &value) {
  const auto text = json_text(value);
  return out.write(text.data(), static_cast<std::streamsize>(text.size()));
}
inline JsonValue parse_json(std::string_view text) {
  json_detail::Parser parser(text);
  auto value = parser.value();
  parser.finish();
  return value;
}
inline JsonValue parse_json(std::istream &in) {
  const std::string text{std::istreambuf_iterator<char>(in),
                         std::istreambuf_iterator<char>()};
  return parse_json(text);
}
// One differing leaf of two record documents: its dotted path
// ("options.solver.method", "entries[1].errors.l2") and the JSON text on
// each side ("<absent>" when one side lacks the path).
struct RecordDifference {
  std::string path, left, right;
  bool operator==(const RecordDifference &) const = default;
};
struct RecordDiffOptions {
  // Skip every "timings" object, also nested ("mesh.run.timings.*"): wall
  // clock is the only part that differs between identical runs.
  bool ignore_timings = true;
  // Skip every "outcome" object: what a solve produced from its inputs
  // (iterations, residuals). They are reproducible, but a changed input
  // changes them, so by default a diff compares inputs, settings and work.
  bool ignore_outcomes = true;
};

namespace json_detail {
// Flattened leaves: "options.solver.method" -> "\"SparseLU\"".
inline void flatten(const JsonValue &v, const std::string &path,
                    std::vector<std::pair<std::string, std::string>> &out) {
  switch (v.kind) {
  case JsonValue::Kind::Object:
    if (v.members.empty())
      out.emplace_back(path, "{}");
    for (const auto &[key, member] : v.members)
      flatten(member, path.empty() ? key : path + '.' + key, out);
    return;
  case JsonValue::Kind::Array:
    if (v.items.empty())
      out.emplace_back(path, "[]");
    for (std::size_t k = 0; k < v.items.size(); ++k)
      flatten(v.items[k], path + '[' + std::to_string(k) + ']', out);
    return;
  default: {
    std::string text;
    write(text, v, 0);
    out.emplace_back(path, std::move(text));
  }
  }
}
// `path` lies in an object called `key` (at the top or nested).
inline bool in_group(const std::string &path, const std::string &key) {
  return path == key || path.starts_with(key + '.') ||
         path.ends_with('.' + key) ||
         path.find('.' + key + '.') != std::string::npos;
}
} // namespace json_detail

// Leaves that differ, in the order of `left`, then leaves only `right` has.
inline std::vector<RecordDifference>
diff_records(const JsonValue &left, const JsonValue &right,
             const RecordDiffOptions &options = {}) {
  std::vector<std::pair<std::string, std::string>> a, b;
  json_detail::flatten(left, "", a);
  json_detail::flatten(right, "", b);
  const auto ignored = [&](const std::string &path) {
    return (options.ignore_timings && json_detail::in_group(path, "timings")) ||
           (options.ignore_outcomes && json_detail::in_group(path, "outcome"));
  };
  const std::map<std::string, std::string> rhs(b.begin(), b.end()),
      lhs(a.begin(), a.end());
  std::vector<RecordDifference> differences;
  for (const auto &[path, value] : a) {
    if (ignored(path))
      continue;
    const auto found = rhs.find(path);
    if (found == rhs.end())
      differences.push_back({path, value, "<absent>"});
    else if (found->second != value)
      differences.push_back({path, value, found->second});
  }
  for (const auto &[path, value] : b)
    if (!ignored(path) && !lhs.contains(path))
      differences.push_back({path, "<absent>", value});
  return differences;
}

namespace json_detail {
// Parse a record document and check its schema name and version. Throws
// std::invalid_argument ("Invalid JSON: ...", "Not a <what>",
// "Unsupported <what> schema version: N").
inline JsonValue read_schema(std::istream &in, std::string_view schema_name,
                             int schema_version, const std::string &what) {
  auto document = parse_json(in);
  const auto *schema = document.kind == JsonValue::Kind::Object
                           ? document.find("schema")
                           : nullptr;
  if (!schema || schema->kind != JsonValue::Kind::String ||
      schema->text != schema_name)
    throw std::invalid_argument("Not a " + what);
  const auto *version = document.find("schema_version");
  if (!version || version->kind != JsonValue::Kind::Number ||
      version->text != std::to_string(schema_version))
    throw std::invalid_argument(
        "Unsupported " + what + " schema version: " +
        (version ? version->text : std::string("missing")));
  return document;
}
} // namespace json_detail
} // namespace pde2d

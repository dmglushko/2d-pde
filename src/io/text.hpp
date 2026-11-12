#pragma once

// Private to the io library: files, numbers and tokens.

#include <charconv>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <istream>
#include <iterator>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

namespace pde2d::io::detail {

// The whole stream as a string.
inline std::string read_all(std::istream &in) {
  std::string text{std::istreambuf_iterator<char>(in),
                   std::istreambuf_iterator<char>()};
  if (in.bad())
    throw std::runtime_error("Reading the input failed");
  return text;
}

// Opens a file for reading or throws std::runtime_error naming it.
inline std::ifstream open_for_reading(const std::filesystem::path &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("Cannot open '" + path.string() + "' for reading");
  return in;
}

// Writes into a file through `write(std::ostream &)`; throws
// std::runtime_error if the file cannot be created or written.
template <class F>
void write_file(const std::filesystem::path &path, F &&write) {
  std::ofstream out(path, std::ios::binary);
  if (!out)
    throw std::runtime_error("Cannot open '" + path.string() + "' for writing");
  write(out);
  out.close();
  if (!out)
    throw std::runtime_error("Writing '" + path.string() + "' failed");
}

// Throws std::runtime_error if the stream failed.
inline void check_written(const std::ostream &out) {
  if (!out)
    throw std::runtime_error("Writing the output failed");
}

// The shortest decimal text that reads back as exactly the same double.
inline std::string shortest(double value) {
  char buffer[64];
  auto [end, ec] = std::to_chars(buffer, buffer + sizeof buffer, value);
  if (ec != std::errc{})
    throw std::logic_error("Number formatting failed");
  return std::string(buffer, end);
}

// A finite double from the whole token, or false.
inline bool parse_double(std::string_view token, double &value) {
  if (token.empty() || token.front() == '+')
    token.remove_prefix(token.empty() ? 0 : 1);
  if (token.empty())
    return false;
  auto [end, ec] =
      std::from_chars(token.data(), token.data() + token.size(), value);
  return ec == std::errc{} && end == token.data() + token.size() &&
         std::isfinite(value);
}

// A non-negative integer from the whole token, or false.
inline bool parse_unsigned(std::string_view token, std::size_t &value) {
  if (token.empty())
    return false;
  auto [end, ec] =
      std::from_chars(token.data(), token.data() + token.size(), value);
  return ec == std::errc{} && end == token.data() + token.size();
}

} // namespace pde2d::io::detail

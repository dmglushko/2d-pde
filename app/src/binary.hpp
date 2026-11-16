#pragma once

#include <bit>
#include <cstdint>
#include <string>

// Numbers in little-endian bytes, for the binary answers (solution values,
// mesh arrays, differences).
namespace pde2d::app {

inline void append_uint32(std::string &out, std::uint32_t bits) {
  for (int byte = 0; byte < 4; ++byte)
    out.push_back(static_cast<char>((bits >> (8 * byte)) & 0xffU));
}

inline void append_float32(std::string &out, double value) {
  append_uint32(out, std::bit_cast<std::uint32_t>(static_cast<float>(value)));
}

inline void append_float64(std::string &out, double value) {
  const auto bits = std::bit_cast<std::uint64_t>(value);
  for (int byte = 0; byte < 8; ++byte)
    out.push_back(static_cast<char>((bits >> (8 * byte)) & 0xffU));
}

} // namespace pde2d::app

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>

// SHA-256 (FIPS 180-4), for the content hash of a case in an export's
// provenance. Small and self-contained; not meant for secrets.
namespace pde2d::app {

class Sha256 {
public:
  Sha256();
  void update(std::string_view bytes);
  // The digest as 64 lowercase hex digits; the object is spent after it.
  std::string hex();

private:
  void block(const unsigned char *data);

  std::array<std::uint32_t, 8> state_;
  std::array<unsigned char, 64> buffer_{};
  std::size_t buffered_ = 0;
  std::uint64_t length_ = 0; // bytes
};

// SHA-256 of `bytes` in hex.
std::string sha256_hex(std::string_view bytes);

} // namespace pde2d::app

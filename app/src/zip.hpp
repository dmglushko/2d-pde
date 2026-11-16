#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

// A zip archive of stored (uncompressed) members, written as it goes: each
// member is given whole (its CRC-32 and size go in its local header), the
// central directory at the end. Offsets past 4 GiB use the Zip64
// extensions. Enough for the app's exports; nothing is compressed.
namespace pde2d::app {

// CRC-32 (the zip and PNG one: reflected, polynomial 0xEDB88320),
// continued from `crc` (0 to start).
std::uint32_t crc32(std::string_view data, std::uint32_t crc = 0);

class ZipWriter {
public:
  using Sink = std::function<void(std::string_view)>;
  // `zip64_from`: an offset from which Zip64 records are written (4 GiB -
  // 1 in a real archive; lower in tests, to see them).
  explicit ZipWriter(Sink sink, std::uint64_t zip64_from = 0xFFFFFFFFu);
  // Writes one member. Throws std::length_error for a member of 4 GiB or
  // more, std::logic_error after finish().
  void add(std::string_view name, std::string_view data);
  // Writes the central directory. Members cannot be added after it.
  void finish();

private:
  struct Entry {
    std::string name;
    std::uint32_t crc = 0;
    std::uint32_t size = 0;
    std::uint64_t offset = 0;
  };
  void write(std::string_view bytes);

  Sink sink_;
  std::uint64_t zip64_from_;
  std::uint64_t offset_ = 0;
  std::vector<Entry> entries_;
  std::uint16_t time_ = 0;
  std::uint16_t date_ = 0;
  bool finished_ = false;
};

} // namespace pde2d::app

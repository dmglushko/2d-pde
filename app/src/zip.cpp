#include "zip.hpp"

#include <array>
#include <chrono>
#include <stdexcept>

namespace pde2d::app {

namespace {

constexpr std::array<std::uint32_t, 256> crc_table = [] {
  std::array<std::uint32_t, 256> table{};
  for (std::uint32_t n = 0; n < 256; ++n) {
    std::uint32_t c = n;
    for (int bit = 0; bit < 8; ++bit)
      c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
    table[n] = c;
  }
  return table;
}();

// Little-endian fields.
void put16(std::string &out, std::uint64_t value) {
  for (int i = 0; i < 2; ++i)
    out += static_cast<char>((value >> (8 * i)) & 0xFF);
}
void put32(std::string &out, std::uint64_t value) {
  for (int i = 0; i < 4; ++i)
    out += static_cast<char>((value >> (8 * i)) & 0xFF);
}
void put64(std::string &out, std::uint64_t value) {
  for (int i = 0; i < 8; ++i)
    out += static_cast<char>((value >> (8 * i)) & 0xFF);
}

constexpr std::uint16_t utf8_names = 0x0800; // general purpose flag bit 11

} // namespace

std::uint32_t crc32(std::string_view data, std::uint32_t crc) {
  crc = ~crc;
  for (const char c : data)
    crc = crc_table[(crc ^ static_cast<unsigned char>(c)) & 0xFF] ^ (crc >> 8);
  return ~crc;
}

ZipWriter::ZipWriter(Sink sink, std::uint64_t zip64_from)
    : sink_(std::move(sink)), zip64_from_(zip64_from) {
  // The members' modification time: now, as MS-DOS local time fields
  // (UTC here; zip has no time zone).
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto day = floor<days>(now);
  const year_month_day date{day};
  const hh_mm_ss time{floor<seconds>(now - day)};
  const int year = std::max(1980, static_cast<int>(date.year()));
  date_ = static_cast<std::uint16_t>(
      ((year - 1980) << 9) | (static_cast<unsigned>(date.month()) << 5) |
      static_cast<unsigned>(date.day()));
  time_ = static_cast<std::uint16_t>((time.hours().count() << 11) |
                                     (time.minutes().count() << 5) |
                                     (time.seconds().count() / 2));
}

void ZipWriter::write(std::string_view bytes) {
  sink_(bytes);
  offset_ += bytes.size();
}

void ZipWriter::add(std::string_view name, std::string_view data) {
  if (finished_)
    throw std::logic_error("zip: a member after the central directory");
  if (data.size() >= 0xFFFFFFFFu)
    throw std::length_error("zip: a member of 4 GiB or more");
  const Entry entry{.name = std::string(name),
                    .crc = crc32(data),
                    .size = static_cast<std::uint32_t>(data.size()),
                    .offset = offset_};
  std::string header;
  put32(header, 0x04034b50);
  put16(header, 10); // version needed: stored
  put16(header, utf8_names);
  put16(header, 0); // stored
  put16(header, time_);
  put16(header, date_);
  put32(header, entry.crc);
  put32(header, entry.size); // compressed
  put32(header, entry.size);
  put16(header, entry.name.size());
  put16(header, 0); // extra field
  header += entry.name;
  write(header);
  write(data);
  entries_.push_back(entry);
}

void ZipWriter::finish() {
  if (finished_)
    return;
  finished_ = true;
  const std::uint64_t directory_offset = offset_;
  bool zip64 = directory_offset >= zip64_from_;
  for (const Entry &entry : entries_) {
    const bool large = entry.offset >= zip64_from_;
    zip64 = zip64 || large;
    std::string record;
    put32(record, 0x02014b50);
    put16(record, large ? 45 : 20); // version made by (MS-DOS)
    put16(record, large ? 45 : 10); // version needed
    put16(record, utf8_names);
    put16(record, 0);
    put16(record, time_);
    put16(record, date_);
    put32(record, entry.crc);
    put32(record, entry.size);
    put32(record, entry.size);
    put16(record, entry.name.size());
    put16(record, large ? 12 : 0); // extra field
    put16(record, 0);              // comment
    put16(record, 0);              // disk
    put16(record, 0);              // internal attributes
    put32(record, 0);              // external attributes
    put32(record, large ? 0xFFFFFFFFu : entry.offset);
    record += entry.name;
    if (large) {
      put16(record, 0x0001); // Zip64 extended information
      put16(record, 8);
      put64(record, entry.offset);
    }
    write(record);
  }
  const std::uint64_t directory_size = offset_ - directory_offset;
  std::string end;
  if (zip64) {
    const std::uint64_t record_offset = offset_;
    put32(end, 0x06064b50);
    put64(end, 44); // the record's size after this field
    put16(end, 45);
    put16(end, 45);
    put32(end, 0);
    put32(end, 0);
    put64(end, entries_.size());
    put64(end, entries_.size());
    put64(end, directory_size);
    put64(end, directory_offset);
    put32(end, 0x07064b50); // the locator
    put32(end, 0);
    put64(end, record_offset);
    put32(end, 1);
  }
  const auto count = std::min<std::uint64_t>(entries_.size(), 0xFFFF);
  put32(end, 0x06054b50);
  put16(end, 0);
  put16(end, 0);
  put16(end, count);
  put16(end, count);
  put32(end, std::min<std::uint64_t>(directory_size, 0xFFFFFFFFu));
  put32(end, zip64 ? 0xFFFFFFFFu : directory_offset);
  put16(end, 0); // comment
  write(end);
}

} // namespace pde2d::app

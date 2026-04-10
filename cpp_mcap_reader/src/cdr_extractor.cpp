#include "cpp_mcap_reader/cdr_extractor.hpp"
#include <cstring>

namespace mcap_reader {

static inline uint32_t read_u32_le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
}

static inline size_t align_up(size_t offset, size_t alignment) {
    return (offset + alignment - 1) & ~(alignment - 1);
}

bool extract_compressed_image(const uint8_t* cdr, size_t size,
                              ImageExtractResult& result)
{
    if (size < 26) return false;

    size_t offset = 4;  // skip CDR encapsulation header

    // Header.stamp.sec
    if (offset + 4 > size) return false;
    result.stamp_sec = read_u32_le(cdr + offset);
    offset += 4;

    // Header.stamp.nanosec
    if (offset + 4 > size) return false;
    result.stamp_nsec = read_u32_le(cdr + offset);
    offset += 4;

    // Header.frame_id (string: uint32 len + bytes + padding)
    if (offset + 4 > size) return false;
    uint32_t fid_len = read_u32_le(cdr + offset);
    offset += 4;
    if (offset + fid_len > size) return false;
    offset += fid_len;
    offset = align_up(offset, 4);

    // format (string)
    if (offset + 4 > size) return false;
    uint32_t fmt_len = read_u32_le(cdr + offset);
    offset += 4;
    if (offset + fmt_len > size || fmt_len == 0) return false;
    result.format = std::string(reinterpret_cast<const char*>(cdr + offset),
                                fmt_len - 1);  // exclude null
    offset += fmt_len;
    offset = align_up(offset, 4);

    // data (sequence<uint8>)
    if (offset + 4 > size) return false;
    uint32_t data_len = read_u32_le(cdr + offset);
    offset += 4;
    if (offset + data_len > size) return false;
    result.data_ptr = cdr + offset;
    result.data_size = data_len;

    return true;
}

bool extract_pose(const uint8_t* cdr, size_t size, PoseExtractResult& result)
{
    // CDR encap (4) + padding to 8-byte align (4) + 7 doubles (56) = 64
    if (size < 64) return false;

    size_t offset = 4;
    offset = align_up(offset, 8);

    if (offset + 56 > size) return false;
    std::memcpy(result.values, cdr + offset, 56);

    return true;
}

}  // namespace mcap_reader

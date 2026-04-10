#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace mcap_reader {

struct ImageExtractResult {
    std::string format;
    const uint8_t* data_ptr = nullptr;
    size_t data_size = 0;
    uint32_t stamp_sec = 0;
    uint32_t stamp_nsec = 0;
};

struct PoseExtractResult {
    double values[7] = {};
    // values[0-2] = position (x, y, z)
    // values[3-6] = orientation (x, y, z, w)
};

bool extract_compressed_image(const uint8_t* cdr, size_t size,
                              ImageExtractResult& result);

bool extract_pose(const uint8_t* cdr, size_t size,
                  PoseExtractResult& result);

}  // namespace mcap_reader

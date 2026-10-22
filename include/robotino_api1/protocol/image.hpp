// Camera images sent by the robot over UDP to the port given in IOControl::image_port.
//
// An image is split into datagrams. The first one starts with a 26-byte header
// ("StartStart", data size, type, resolution, width, height, channels, bits per channel),
// the last one ends with "StoppStopp"; a small image may fit into a single datagram.
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "robotino_api1/export.hpp"
#include "robotino_api1/protocol/bytes.hpp"

namespace robotino_api1::protocol {

enum class ImageType : std::uint8_t { JPEG = 0, JPEG2000 = 1, Raw = 2, BMP = 3, PNG = 4, TIFF = 5, Unknown = 6 };

inline constexpr std::size_t kImageHeaderSize = 26;
inline constexpr std::size_t kImageStopSize = 10;

struct ImageHeader {
    std::uint32_t data_size = 0;
    ImageType type = ImageType::Unknown;
    std::uint8_t resolution = 2;  // CameraResolution, 2 = custom
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint8_t channels = 0;
    std::uint8_t bits_per_channel = 0;
};

struct ImageData {
    ImageHeader header;
    std::vector<std::uint8_t> data;
};

ROBOTINO_API1_EXPORT std::vector<std::uint8_t> encode(const ImageHeader& header);
ROBOTINO_API1_EXPORT std::optional<ImageHeader> decode_image_header(ByteView data);

// Splits an image into datagrams of at most max_datagram_size bytes (used by the emulator).
ROBOTINO_API1_EXPORT std::vector<std::vector<std::uint8_t>> split_image(const ImageData& image,
                                                                   std::size_t max_datagram_size);

// Reassembles images from datagrams. A lost or reordered datagram drops the image.
class ROBOTINO_API1_EXPORT ImageAssembler {
public:
    // Returns a complete image when the datagram finishes one.
    std::optional<ImageData> push(ByteView datagram);
    std::uint64_t dropped() const { return dropped_; }

private:
    void drop();

    ImageData current_;
    bool active_ = false;
    std::uint64_t dropped_ = 0;
};

}  // namespace robotino_api1::protocol

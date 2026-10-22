#include "robotino_api1/protocol/image.hpp"

#include <algorithm>
#include <cstring>

namespace robotino_api1::protocol {
namespace {

constexpr std::uint8_t kStartSequence[10] = {'S', 't', 'a', 'r', 't', 'S', 't', 'a', 'r', 't'};
constexpr std::uint8_t kStopSequence[kImageStopSize] = {'S', 't', 'o', 'p', 'p', 'S', 't', 'o', 'p', 'p'};

// Images larger than this are rejected as corrupt headers (VGA RGB is 921600 bytes).
constexpr std::uint32_t kMaxImageSize = 16 * 1024 * 1024;

bool ends_with_stop(ByteView data) {
    return data.size >= kImageStopSize &&
           std::memcmp(data.data + data.size - kImageStopSize, kStopSequence, kImageStopSize) == 0;
}

}  // namespace

std::vector<std::uint8_t> encode(const ImageHeader& header) {
    std::vector<std::uint8_t> out;
    ByteWriter w(out);
    w.bytes(kStartSequence, sizeof kStartSequence);
    w.u32(header.data_size);
    w.u8(static_cast<std::uint8_t>(header.type));
    w.u8(header.resolution);
    w.u32(header.width);
    w.u32(header.height);
    w.u8(header.channels);
    w.u8(header.bits_per_channel);
    return out;
}

std::optional<ImageHeader> decode_image_header(ByteView data) {
    if (data.size < kImageHeaderSize || std::memcmp(data.data, kStartSequence, sizeof kStartSequence) != 0) {
        return std::nullopt;
    }
    ByteReader r(data);
    r.skip(sizeof kStartSequence);
    ImageHeader h;
    h.data_size = r.u32();
    const std::uint8_t type = r.u8();
    h.type = type <= static_cast<std::uint8_t>(ImageType::Unknown) ? static_cast<ImageType>(type) : ImageType::Unknown;
    h.resolution = r.u8();
    h.width = r.u32();
    h.height = r.u32();
    h.channels = r.u8();
    h.bits_per_channel = r.u8();
    if (h.data_size > kMaxImageSize) {
        return std::nullopt;
    }
    return h;
}

std::vector<std::vector<std::uint8_t>> split_image(const ImageData& image, std::size_t max_datagram_size) {
    ImageHeader header = image.header;
    header.data_size = static_cast<std::uint32_t>(image.data.size());
    std::vector<std::uint8_t> stream = encode(header);
    stream.insert(stream.end(), image.data.begin(), image.data.end());
    stream.insert(stream.end(), std::begin(kStopSequence), std::end(kStopSequence));

    // The stop sequence must not be split, otherwise the receiver cannot find it.
    std::vector<std::vector<std::uint8_t>> datagrams;
    std::size_t pos = 0;
    while (pos < stream.size()) {
        std::size_t n = std::min(max_datagram_size, stream.size() - pos);
        const std::size_t rest = stream.size() - pos - n;
        if (rest > 0 && rest < kImageStopSize) {
            n -= kImageStopSize - rest;
        }
        datagrams.emplace_back(stream.begin() + static_cast<std::ptrdiff_t>(pos),
                               stream.begin() + static_cast<std::ptrdiff_t>(pos + n));
        pos += n;
    }
    return datagrams;
}

void ImageAssembler::drop() {
    if (active_) {
        ++dropped_;
    }
    active_ = false;
    current_.data.clear();
}

std::optional<ImageData> ImageAssembler::push(ByteView datagram) {
    ByteView part = datagram;
    if (auto header = decode_image_header(datagram)) {
        drop();
        current_.header = *header;
        current_.data.clear();
        current_.data.reserve(header->data_size);
        active_ = true;
        part = datagram.subview(kImageHeaderSize, datagram.size - kImageHeaderSize);
    } else if (!active_) {
        return std::nullopt;
    }

    const bool last = ends_with_stop(part);
    if (last) {
        part.size -= kImageStopSize;
    }
    if (current_.data.size() + part.size > current_.header.data_size) {
        drop();
        return std::nullopt;
    }
    current_.data.insert(current_.data.end(), part.data, part.data + part.size);
    if (!last) {
        return std::nullopt;
    }
    if (current_.data.size() != current_.header.data_size) {
        drop();
        return std::nullopt;
    }
    active_ = false;
    ImageData image = std::move(current_);
    current_ = ImageData{};
    return image;
}

}  // namespace robotino_api1::protocol

#include <doctest/doctest.h>

#include "robotino_api1/protocol/image.hpp"

using namespace robotino_api1::protocol;

namespace {

ImageData make_image(std::size_t size) {
    ImageData image;
    image.header.type = ImageType::Raw;
    image.header.width = 4;
    image.header.height = 2;
    image.header.channels = 3;
    image.header.bits_per_channel = 8;
    image.data.resize(size);
    for (std::size_t i = 0; i < size; ++i) {
        image.data[i] = static_cast<std::uint8_t>(i * 7);
    }
    return image;
}

}  // namespace

TEST_CASE("Image header is 26 bytes") {
    ImageHeader h;
    h.data_size = 1234;
    h.width = 640;
    h.height = 480;
    const auto bytes = encode(h);
    CHECK(bytes.size() == kImageHeaderSize);
    auto d = decode_image_header(bytes);
    REQUIRE(d);
    CHECK(d->data_size == 1234);
    CHECK(d->width == 640);
    CHECK(d->height == 480);
}

TEST_CASE("Images are split and reassembled") {
    for (std::size_t size : {std::size_t{0}, std::size_t{5}, std::size_t{100}, std::size_t{10000}}) {
        for (std::size_t datagram : {std::size_t{40}, std::size_t{41}, std::size_t{1400}, std::size_t{65000}}) {
            CAPTURE(size);
            CAPTURE(datagram);
            const ImageData image = make_image(size);
            const auto parts = split_image(image, datagram);
            ImageAssembler assembler;
            std::optional<ImageData> result;
            for (std::size_t i = 0; i < parts.size(); ++i) {
                CHECK(parts[i].size() <= datagram);
                result = assembler.push(parts[i]);
                CHECK(result.has_value() == (i + 1 == parts.size()));
            }
            REQUIRE(result);
            CHECK(result->data == image.data);
            CHECK(result->header.width == 4);
            CHECK(result->header.type == ImageType::Raw);
        }
    }
}

TEST_CASE("A lost datagram drops the image, the next image arrives") {
    const ImageData image = make_image(5000);
    const auto parts = split_image(image, 1000);
    REQUIRE(parts.size() > 3);
    ImageAssembler assembler;
    for (std::size_t i = 0; i < parts.size(); ++i) {
        if (i != 2) {
            CHECK_FALSE(assembler.push(parts[i]));
        }
    }
    CHECK(assembler.dropped() == 1);
    std::optional<ImageData> result;
    for (const auto& part : parts) {
        result = assembler.push(part);
    }
    REQUIRE(result);
    CHECK(result->data == image.data);
}

// Little-endian byte reader and writer used by all protocol codecs.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace robotino_api1::protocol {

// Non-owning view of a byte range (std::span is C++20).
struct ByteView {
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;

    ByteView() = default;
    ByteView(const std::uint8_t* d, std::size_t n) : data(d), size(n) {}
    ByteView(const std::vector<std::uint8_t>& v) : data(v.data()), size(v.size()) {}  // NOLINT(google-explicit-constructor)
    template <std::size_t N>
    ByteView(const std::uint8_t (&a)[N]) : data(a), size(N) {}  // NOLINT(google-explicit-constructor)

    const std::uint8_t& operator[](std::size_t i) const { return data[i]; }
    ByteView subview(std::size_t offset, std::size_t n) const { return {data + offset, n}; }
};

// Appends little-endian values to a byte vector.
class ByteWriter {
public:
    explicit ByteWriter(std::vector<std::uint8_t>& out) : out_(out) {}

    void u8(std::uint8_t v) { out_.push_back(v); }
    void i8(std::int8_t v) { u8(static_cast<std::uint8_t>(v)); }
    void u16(std::uint16_t v) {
        u8(static_cast<std::uint8_t>(v));
        u8(static_cast<std::uint8_t>(v >> 8));
    }
    void i16(std::int16_t v) { u16(static_cast<std::uint16_t>(v)); }
    void u32(std::uint32_t v) {
        u16(static_cast<std::uint16_t>(v));
        u16(static_cast<std::uint16_t>(v >> 16));
    }
    void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
    void f32(float v) {
        std::uint32_t bits;
        std::memcpy(&bits, &v, sizeof bits);
        u32(bits);
    }
    void bytes(const std::uint8_t* data, std::size_t n) { out_.insert(out_.end(), data, data + n); }
    void zeros(std::size_t n) { out_.insert(out_.end(), n, 0); }

private:
    std::vector<std::uint8_t>& out_;
};

// Reads little-endian values from a byte range. Reading past the end yields zeros and
// sets ok() to false, so decoders can read a whole message and check once at the end.
class ByteReader {
public:
    explicit ByteReader(ByteView in) : in_(in) {}

    bool ok() const { return ok_; }
    std::size_t remaining() const { return ok_ ? in_.size - pos_ : 0; }

    std::uint8_t u8() {
        if (pos_ >= in_.size) {
            ok_ = false;
            return 0;
        }
        return in_[pos_++];
    }
    std::int8_t i8() { return static_cast<std::int8_t>(u8()); }
    std::uint16_t u16() {
        const std::uint16_t lo = u8();
        return static_cast<std::uint16_t>(lo | (u8() << 8));
    }
    std::int16_t i16() { return static_cast<std::int16_t>(u16()); }
    std::uint32_t u32() {
        const std::uint32_t lo = u16();
        return lo | (static_cast<std::uint32_t>(u16()) << 16);
    }
    std::int32_t i32() { return static_cast<std::int32_t>(u32()); }
    float f32() {
        const std::uint32_t bits = u32();
        float v;
        std::memcpy(&v, &bits, sizeof v);
        return v;
    }
    ByteView bytes(std::size_t n) {
        if (n > remaining()) {
            ok_ = false;
            pos_ = in_.size;
            return {};
        }
        ByteView v = in_.subview(pos_, n);
        pos_ += n;
        return v;
    }
    void skip(std::size_t n) { bytes(n); }

private:
    ByteView in_;
    std::size_t pos_ = 0;
    bool ok_ = true;
};

}  // namespace robotino_api1::protocol

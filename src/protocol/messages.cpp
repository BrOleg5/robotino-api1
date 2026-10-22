#include "robotino_api1/protocol/messages.hpp"

#include <algorithm>

namespace robotino_api1::protocol {
namespace {

std::vector<std::uint8_t> begin_message(MessageId id, std::size_t payload_size) {
    std::vector<std::uint8_t> out;
    out.reserve(kHeaderSize + payload_size);
    ByteWriter w(out);
    w.u8(static_cast<std::uint8_t>(id));
    w.u16(static_cast<std::uint16_t>(payload_size));
    return out;
}

}  // namespace

std::vector<std::uint8_t> encode(const IOControl& msg) {
    std::vector<std::uint8_t> out = begin_message(MessageId::IOControl, kIOControlSize);
    ByteWriter w(out);
    w.u8(msg.image_request ? 1 : 0);
    w.u8(msg.shutdown ? 1 : 0);
    w.u16(msg.image_port);
    w.i8(msg.northstar.room_id);
    w.u8(msg.northstar.cal_flag);
    w.u8(msg.northstar.cal_state);
    w.u16(msg.northstar.cal_distance);
    const auto p2q = encode(msg.p2q);
    w.bytes(p2q.data(), p2q.size());
    w.u8(msg.set_odometry ? 1 : 0);
    w.f32(msg.odometry_x);
    w.f32(msg.odometry_y);
    w.f32(msg.odometry_phi);
    w.f32(msg.northstar_ceiling_cal);
    w.zeros(3);
    return out;
}

std::optional<IOControl> decode_io_control(ByteView payload) {
    if (payload.size != kIOControlSize) {
        return std::nullopt;
    }
    ByteReader r(payload);
    IOControl msg;
    msg.image_request = r.u8() != 0;
    msg.shutdown = r.u8() != 0;
    msg.image_port = r.u16();
    msg.northstar.room_id = r.i8();
    msg.northstar.cal_flag = r.u8();
    msg.northstar.cal_state = r.u8();
    msg.northstar.cal_distance = r.u16();
    auto p2q = decode_p2q(r.bytes(kP2QSize));
    if (!p2q) {
        return std::nullopt;
    }
    msg.p2q = *p2q;
    msg.set_odometry = r.u8() != 0;
    msg.odometry_x = r.f32();
    msg.odometry_y = r.f32();
    msg.odometry_phi = r.f32();
    msg.northstar_ceiling_cal = r.f32();
    r.skip(3);
    if (!r.ok()) {
        return std::nullopt;
    }
    return msg;
}

std::vector<std::uint8_t> encode(const IOStatus& msg) {
    std::vector<std::uint8_t> out = begin_message(MessageId::IOStatus, kIOStatusSize);
    ByteWriter w(out);
    w.u16(msg.serial_frequency);
    const auto q2p = encode(msg.q2p);
    w.bytes(q2p.data(), q2p.size());
    w.u32(msg.northstar.sequence);
    w.u8(msg.northstar.room_id);
    w.u8(msg.northstar.num_spots_visible);
    w.i16(msg.northstar.pos_x);
    w.i16(msg.northstar.pos_y);
    w.f32(msg.northstar.pos_theta);
    w.u16(msg.northstar.mag_spot0);
    w.u16(msg.northstar.mag_spot1);
    w.u8(msg.passive_mode ? 1 : 0);
    w.bytes(msg.reserved.data(), msg.reserved.size());
    return out;
}

std::optional<IOStatus> decode_io_status(ByteView payload) {
    if (payload.size != kIOStatusSize) {
        return std::nullopt;
    }
    ByteReader r(payload);
    IOStatus msg;
    msg.serial_frequency = r.u16();
    auto q2p = decode_q2p(r.bytes(kQ2PSize));
    if (!q2p) {
        return std::nullopt;
    }
    msg.q2p = *q2p;
    msg.northstar.sequence = r.u32();
    msg.northstar.room_id = r.u8();
    msg.northstar.num_spots_visible = r.u8();
    msg.northstar.pos_x = r.i16();
    msg.northstar.pos_y = r.i16();
    msg.northstar.pos_theta = r.f32();
    msg.northstar.mag_spot0 = r.u16();
    msg.northstar.mag_spot1 = r.u16();
    msg.passive_mode = r.u8() != 0;
    const ByteView reserved = r.bytes(msg.reserved.size());
    if (!r.ok()) {
        return std::nullopt;
    }
    std::copy(reserved.data, reserved.data + reserved.size, msg.reserved.begin());
    return msg;
}

std::vector<std::uint8_t> encode(const CameraControl& msg) {
    std::vector<std::uint8_t> out = begin_message(MessageId::CameraControl, kCameraControlSize);
    ByteWriter w(out);
    w.u8(static_cast<std::uint8_t>(msg.resolution));
    w.u8(static_cast<std::uint8_t>(msg.compression));
    w.u8(msg.brightness);
    w.u8(msg.contrast);
    w.u8(msg.auto_white_balance ? 1 : 0);
    return out;
}

std::optional<CameraControl> decode_camera_control(ByteView payload) {
    if (payload.size != kCameraControlSize || payload[0] > 2 || payload[1] > 2) {
        return std::nullopt;
    }
    CameraControl msg;
    msg.resolution = static_cast<CameraResolution>(payload[0]);
    msg.compression = static_cast<CameraCompression>(payload[1]);
    msg.brightness = payload[2];
    msg.contrast = payload[3];
    msg.auto_white_balance = payload[4] != 0;
    return msg;
}

std::vector<std::uint8_t> encode(const Info& msg) {
    std::vector<std::uint8_t> out = begin_message(MessageId::Info, kInfoSize);
    ByteWriter w(out);
    const std::size_t n = std::min(msg.text.size(), kInfoTextSize);
    w.bytes(reinterpret_cast<const std::uint8_t*>(msg.text.data()), n);
    w.zeros(kInfoTextSize - n);
    w.u8(msg.passive_mode ? 1 : 0);
    return out;
}

std::optional<Info> decode_info(ByteView payload) {
    if (payload.size != kInfoSize) {
        return std::nullopt;
    }
    const auto* text = reinterpret_cast<const char*>(payload.data);
    const auto* end = std::find(text, text + kInfoTextSize, '\0');
    Info msg;
    msg.text.assign(text, end);
    msg.passive_mode = payload[kInfoTextSize] != 0;
    return msg;
}

std::vector<std::uint8_t> encode_empty(MessageId id) { return begin_message(id, 0); }

void MessageParser::feed(const std::uint8_t* data, std::size_t size) {
    // Drop consumed bytes before growing the buffer.
    if (pos_ > 0 && pos_ == buffer_.size()) {
        buffer_.clear();
        pos_ = 0;
    } else if (pos_ > 4096) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(pos_));
        pos_ = 0;
    }
    buffer_.insert(buffer_.end(), data, data + size);
}

std::optional<Message> MessageParser::next() {
    if (error_ || buffer_.size() - pos_ < kHeaderSize) {
        return std::nullopt;
    }
    const std::uint8_t* header = buffer_.data() + pos_;
    const std::size_t length = static_cast<std::size_t>(header[1] | (header[2] << 8));
    if (length > kMaxPayloadSize) {
        error_ = true;
        return std::nullopt;
    }
    if (buffer_.size() - pos_ < kHeaderSize + length) {
        return std::nullopt;
    }
    Message msg;
    msg.id = header[0];
    msg.payload.assign(header + kHeaderSize, header + kHeaderSize + length);
    pos_ += kHeaderSize + length;
    return msg;
}

void MessageParser::reset() {
    buffer_.clear();
    pos_ = 0;
    error_ = false;
}

}  // namespace robotino_api1::protocol

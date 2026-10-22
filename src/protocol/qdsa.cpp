#include "robotino_api1/protocol/qdsa.hpp"

namespace robotino_api1::protocol {
namespace {

constexpr std::uint8_t kStart[3] = {'R', 'E', 'C'};
constexpr std::uint8_t kStop[3] = {'r', 'e', 'c'};

constexpr std::size_t kP2QMaster = 3;
constexpr std::size_t kP2QSlave = 4;
constexpr std::size_t kP2QSlaveSize = 10;

constexpr std::size_t kQ2PMaster = 3;
constexpr std::size_t kQ2PSlave = 14;
constexpr std::size_t kQ2PSlaveSize = 21;

bool has_markers(ByteView data, std::size_t size) {
    if (data.size < size) {
        return false;
    }
    for (std::size_t i = 0; i < 3; ++i) {
        if (data[i] != kStart[i] || data[size - 3 + i] != kStop[i]) {
            return false;
        }
    }
    return true;
}

// An ADC block stores the high 8 bits of eight 10-bit channels, then two bytes with the
// low 2 bits of channels 0-3 and 4-7.
void encode_adc(std::uint8_t* block, const std::array<std::uint16_t, 8>& adc) {
    block[8] = 0;
    block[9] = 0;
    for (std::size_t i = 0; i < 8; ++i) {
        block[i] = static_cast<std::uint8_t>((adc[i] >> 2) & 0xFF);
        block[8 + i / 4] |= static_cast<std::uint8_t>((adc[i] & 0x3) << (2 * (i % 4)));
    }
}

std::array<std::uint16_t, 8> decode_adc(const std::uint8_t* block) {
    std::array<std::uint16_t, 8> adc{};
    for (std::size_t i = 0; i < 8; ++i) {
        adc[i] = static_cast<std::uint16_t>((block[i] << 2) | ((block[8 + i / 4] >> (2 * (i % 4))) & 0x3));
    }
    return adc;
}

void put_u32(std::uint8_t* p, std::uint32_t v) {
    for (std::size_t i = 0; i < 4; ++i) {
        p[i] = static_cast<std::uint8_t>(v >> (8 * i));
    }
}

std::uint32_t get_u32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

}  // namespace

P2Q::P2Q() {
    // The official client sends zero gains to the power output slave.
    slaves[3].kp = 0;
    slaves[3].ki = 0;
    slaves[3].kd = 0;
}

std::array<std::uint8_t, kP2QSize> encode(const P2Q& p2q) {
    std::array<std::uint8_t, kP2QSize> out{};
    for (std::size_t i = 0; i < 3; ++i) {
        out[i] = kStart[i];
        out[kP2QSize - 3 + i] = kStop[i];
    }
    out[kP2QMaster] = p2q.master_out;
    for (std::size_t s = 0; s < kNumSlaves; ++s) {
        const SlaveSetpoint& sp = p2q.slaves[s];
        std::uint8_t* block = out.data() + kP2QSlave + s * kP2QSlaveSize;
        block[0] = sp.digital_out;
        block[1] = sp.misc_out;
        block[2] = sp.velocity;
        put_u32(block + 3, static_cast<std::uint32_t>(sp.position));
        block[7] = sp.kp;
        block[8] = sp.ki;
        block[9] = sp.kd;
    }
    return out;
}

std::optional<P2Q> decode_p2q(ByteView data) {
    if (!has_markers(data, kP2QSize)) {
        return std::nullopt;
    }
    P2Q p2q;
    p2q.master_out = data[kP2QMaster];
    for (std::size_t s = 0; s < kNumSlaves; ++s) {
        SlaveSetpoint& sp = p2q.slaves[s];
        const std::uint8_t* block = data.data + kP2QSlave + s * kP2QSlaveSize;
        sp.digital_out = block[0];
        sp.misc_out = block[1];
        sp.velocity = block[2];
        sp.position = static_cast<std::int32_t>(get_u32(block + 3));
        sp.kp = block[7];
        sp.ki = block[8];
        sp.kd = block[9];
    }
    return p2q;
}

std::array<std::uint8_t, kQ2PSize> encode(const Q2P& q2p) {
    std::array<std::uint8_t, kQ2PSize> out{};
    for (std::size_t i = 0; i < 3; ++i) {
        out[i] = kStart[i];
        out[kQ2PSize - 3 + i] = kStop[i];
    }
    encode_adc(out.data() + kQ2PMaster, q2p.master_adc);
    out[kQ2PMaster + 10] = q2p.master_time;
    for (std::size_t s = 0; s < kNumSlaves; ++s) {
        const SlaveReadings& r = q2p.slaves[s];
        std::uint8_t* block = out.data() + kQ2PSlave + s * kQ2PSlaveSize;
        encode_adc(block, r.adc);
        block[10] = r.misc_in;
        block[11] = r.velocity;
        put_u32(block + 12, static_cast<std::uint32_t>(r.position));
        block[16] = r.digital_in;
        put_u32(block + 17, r.time);
    }
    return out;
}

std::optional<Q2P> decode_q2p(ByteView data) {
    if (!has_markers(data, kQ2PSize)) {
        return std::nullopt;
    }
    Q2P q2p;
    q2p.master_adc = decode_adc(data.data + kQ2PMaster);
    q2p.master_time = data[kQ2PMaster + 10];
    for (std::size_t s = 0; s < kNumSlaves; ++s) {
        SlaveReadings& r = q2p.slaves[s];
        const std::uint8_t* block = data.data + kQ2PSlave + s * kQ2PSlaveSize;
        r.adc = decode_adc(block);
        r.misc_in = block[10];
        r.velocity = block[11];
        r.position = static_cast<std::int32_t>(get_u32(block + 12));
        r.digital_in = block[16];
        r.time = get_u32(block + 17);
    }
    return q2p;
}

}  // namespace robotino_api1::protocol

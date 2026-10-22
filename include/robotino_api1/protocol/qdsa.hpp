// Frames of the serial protocol between the robot's PC and its IO board ("QDSA").
//
// The robot forwards them over the network unchanged inside IOControl/IOStatus messages:
// P2Q (PC to board, 47 bytes) carries setpoints, Q2P (board to PC, 101 bytes) carries readings.
// Both start with "REC", end with "rec" and consist of one master block and four slave blocks.
// Slaves 0-2 drive the motors, slave 3 drives the power output and the encoder input.
//
// These structures are raw: values are in board units, see robotino_api1/state.hpp for SI conversion.
#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "robotino_api1/export.hpp"
#include "robotino_api1/protocol/bytes.hpp"

namespace robotino_api1::protocol {

inline constexpr std::size_t kP2QSize = 47;
inline constexpr std::size_t kQ2PSize = 101;
inline constexpr std::size_t kNumSlaves = 4;

// Master output byte bits.
inline constexpr std::uint8_t kMasterShutdown = 1 << 1;
// Set by every client since Robotino View 1.7; without it the LPC IO board ignores kp/ki/kd.
inline constexpr std::uint8_t kMasterNewSoftware = 1 << 3;

// Slave digital output byte bits.
inline constexpr std::uint8_t kSlaveBrakeOff = 1 << 0;
inline constexpr std::uint8_t kSlaveDigitalOut0 = 1 << 1;  // DO0..DO3 are bits 1..4
inline constexpr std::uint8_t kSlaveRelay = 1 << 5;

// Slave misc output byte bits.
inline constexpr std::uint8_t kSlaveDirectionPositive = 1 << 1;
inline constexpr std::uint8_t kSlaveResetPosition = 1 << 3;

// Slave misc input byte bits.
inline constexpr std::uint8_t kSlaveVelocityPositive = 1 << 0;

// Slave digital input byte bits.
inline constexpr std::uint8_t kSlaveBumper = 1 << 4;  // DI0..DI3 are bits 0..3

inline constexpr std::uint8_t kPidDefault = 255;  // the board uses its built-in gains

struct SlaveSetpoint {
    std::uint8_t digital_out = kSlaveBrakeOff;
    std::uint8_t misc_out = kSlaveDirectionPositive;
    std::uint8_t velocity = 0;  // magnitude, encoder ticks per control period (1/900 s)
    std::int32_t position = 0;  // unused in velocity mode
    std::uint8_t kp = kPidDefault;
    std::uint8_t ki = kPidDefault;
    std::uint8_t kd = kPidDefault;
};

struct P2Q {
    std::uint8_t master_out = kMasterNewSoftware;
    std::array<SlaveSetpoint, kNumSlaves> slaves{};

    ROBOTINO_API1_EXPORT P2Q();
};

struct SlaveReadings {
    std::array<std::uint16_t, 8> adc{};  // 10-bit
    std::uint8_t misc_in = 0;
    std::uint8_t velocity = 0;           // magnitude, encoder ticks per control period
    std::int32_t position = 0;           // encoder ticks as sent, see state.hpp for the sign
    std::uint8_t digital_in = 0;
    std::uint32_t time = 0;
};

struct Q2P {
    std::array<std::uint16_t, 8> master_adc{};  // 10-bit
    std::uint8_t master_time = 0;
    std::array<SlaveReadings, kNumSlaves> slaves{};
};

ROBOTINO_API1_EXPORT std::array<std::uint8_t, kP2QSize> encode(const P2Q& p2q);
ROBOTINO_API1_EXPORT std::optional<P2Q> decode_p2q(ByteView data);

ROBOTINO_API1_EXPORT std::array<std::uint8_t, kQ2PSize> encode(const Q2P& q2p);
ROBOTINO_API1_EXPORT std::optional<Q2P> decode_q2p(ByteView data);

}  // namespace robotino_api1::protocol

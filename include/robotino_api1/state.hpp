// Robot commands and readings in SI units, and their conversion to and from the raw protocol.
#pragma once

#include <array>
#include <chrono>
#include <cstdint>

#include "robotino_api1/export.hpp"
#include "robotino_api1/protocol/messages.hpp"

namespace robotino_api1 {

inline constexpr std::size_t kNumMotors = 3;
inline constexpr std::size_t kNumDistanceSensors = 9;
inline constexpr std::size_t kNumDigitalInputs = 8;
inline constexpr std::size_t kNumDigitalOutputs = 8;
inline constexpr std::size_t kNumAnalogInputs = 8;
inline constexpr std::size_t kNumRelays = 2;

// Motor encoder resolution and the IO board control loop rate. Velocities travel over the
// wire as encoder ticks per control period, so one unit is 2*pi*900/2000 = 2.83 rad/s
// (27 rpm) at the motor shaft, i.e. 0.18 rad/s at the wheel.
inline constexpr double kEncoderTicksPerRevolution = 2000.0;
inline constexpr double kControlFrequency = 900.0;
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kMotorVelocityResolution = 2.0 * kPi * kControlFrequency / kEncoderTicksPerRevolution;

// Converts encoder ticks to motor shaft angle in rad.
inline constexpr double ticks_to_rad(std::int32_t ticks) { return ticks * 2.0 * kPi / kEncoderTicksPerRevolution; }

// Heading of a distance sensor relative to the robot's x axis, rad (sensors are 40 deg apart).
inline constexpr double distance_sensor_heading(std::size_t sensor) { return static_cast<double>(sensor) * 2.0 * kPi / 9.0; }

// PID gains of a motor controller, 0-255; 255 selects the IO board's built-in value.
struct Pid {
    std::uint8_t kp = protocol::kPidDefault;
    std::uint8_t ki = protocol::kPidDefault;
    std::uint8_t kd = protocol::kPidDefault;
};

struct Command {
    std::array<double, kNumMotors> motor_velocity{};  // rad/s at the motor shaft (before the gearbox)
    std::array<Pid, kNumMotors> pid{};
    std::array<bool, kNumMotors> brake{};  // true engages the brake; brakes are released by default
    std::array<bool, kNumMotors> reset_motor_position{};
    std::array<bool, kNumDigitalOutputs> digital_output{};
    std::array<bool, kNumRelays> relay{};
    double power_output = 0.0;  // duty cycle, -1..1
    bool reset_encoder_input = false;
    bool shutdown = false;  // switches the robot off
};

struct SensorState {
    std::chrono::steady_clock::time_point stamp{};  // when the client received it
    std::uint64_t sequence = 0;                     // counts received states since connect

    std::array<double, kNumMotors> motor_velocity{};       // rad/s at the motor shaft
    std::array<std::int32_t, kNumMotors> motor_position{};  // encoder ticks
    std::array<double, kNumMotors> motor_current{};        // A

    std::array<double, kNumDistanceSensors> distance_voltage{};  // V, higher is closer
    std::array<double, kNumAnalogInputs> analog_input{};         // V, 0-10
    std::array<bool, kNumDigitalInputs> digital_input{};
    bool bumper = false;

    double battery_voltage = 0.0;  // V
    double battery_current = 0.0;  // A
    double power_output_current = 0.0;  // A

    std::int32_t encoder_input_position = 0;  // ticks
    std::int32_t encoder_input_velocity = 0;   // ticks per control period

    std::uint16_t serial_frequency = 0;  // Hz
    std::uint32_t board_time = 0;        // IO board counter
    unsigned firmware_version = 0;       // IO board firmware: 411, 670 or 1000 (EA09); 0 if unknown
    bool passive_mode = false;           // another client controls the robot

    protocol::IOStatus raw;
};

// Fills the P2Q frame of an IOControl message from a command.
ROBOTINO_API1_EXPORT protocol::P2Q to_p2q(const Command& command);
// The inverse of to_p2q (velocities are quantized), used by the emulator.
ROBOTINO_API1_EXPORT Command from_p2q(const protocol::P2Q& p2q);

ROBOTINO_API1_EXPORT SensorState from_io_status(const protocol::IOStatus& status);
// The inverse of from_io_status, used by the emulator.
ROBOTINO_API1_EXPORT protocol::IOStatus to_io_status(const SensorState& state);

}  // namespace robotino_api1

#include "robotino_api1/state.hpp"

#include <algorithm>
#include <cmath>

namespace robotino_api1 {
namespace {

using protocol::kSlaveBrakeOff;
using protocol::kSlaveBumper;
using protocol::kSlaveDigitalOut0;
using protocol::kSlaveDirectionPositive;
using protocol::kSlaveRelay;
using protocol::kSlaveResetPosition;
using protocol::kSlaveVelocityPositive;

// ADC scales of the IO board.
constexpr double kScaleBatteryCurrent = 61.44;  // counts per A
constexpr double kScaleBatteryVoltage = 36.10;  // counts per V
constexpr double kScaleMotorCurrent = 156.0;    // counts per A
constexpr double kDistanceFullScale = 2.55;     // V
constexpr double kAnalogFullScale = 10.0;       // V
constexpr double kAdcCounts = 1024.0;

// Master ADC channels.
constexpr std::size_t kAdcBatteryCurrent = 0;
constexpr std::size_t kAdcBatteryVoltage = 2;

// Slave ADC channel 0 measures the motor (or power output) current, 3-6 are analog inputs.
constexpr std::size_t kAdcMotorCurrent = 0;
constexpr std::size_t kAdcFirstAnalogInput = 3;
constexpr std::size_t kAnalogInputsPerSlave = 4;

struct AdcChannel {
    std::size_t slave;
    std::size_t channel;
};

// Where each distance sensor is wired on the IO board.
constexpr std::array<AdcChannel, kNumDistanceSensors> kDistanceSensorAdc = {{
    {2, 2}, {2, 1}, {1, 7}, {1, 2}, {1, 1}, {0, 1}, {0, 2}, {0, 7}, {2, 7},
}};

constexpr std::array<unsigned, 3> kFirmwareVersions = {411, 670, 1000};

std::uint16_t to_adc(double value, double counts_per_unit) {
    return static_cast<std::uint16_t>(std::clamp(std::lround(value * counts_per_unit), 0L, 1023L));
}

std::uint8_t to_velocity_units(double rad_s) {
    return static_cast<std::uint8_t>(std::min(std::lround(std::fabs(rad_s) / kMotorVelocityResolution), 255L));
}

double signed_velocity(const protocol::SlaveReadings& r) {
    const double magnitude = r.velocity * kMotorVelocityResolution;
    return (r.misc_in & kSlaveVelocityPositive) ? magnitude : -magnitude;
}

void set_velocity(protocol::SlaveReadings& r, int units) {
    r.velocity = static_cast<std::uint8_t>(std::min(std::abs(units), 255));
    r.misc_in = static_cast<std::uint8_t>((r.misc_in & ~kSlaveVelocityPositive) | (units >= 0 ? kSlaveVelocityPositive : 0));
}

}  // namespace

protocol::P2Q to_p2q(const Command& command) {
    protocol::P2Q p2q;
    p2q.master_out = protocol::kMasterNewSoftware | (command.shutdown ? protocol::kMasterShutdown : 0);

    for (std::size_t m = 0; m < kNumMotors; ++m) {
        protocol::SlaveSetpoint& sp = p2q.slaves[m];
        sp.digital_out = command.brake[m] ? 0 : kSlaveBrakeOff;
        sp.misc_out = static_cast<std::uint8_t>((command.motor_velocity[m] >= 0.0 ? kSlaveDirectionPositive : 0) |
                                                (command.reset_motor_position[m] ? kSlaveResetPosition : 0));
        sp.velocity = to_velocity_units(command.motor_velocity[m]);
        sp.kp = command.pid[m].kp;
        sp.ki = command.pid[m].ki;
        sp.kd = command.pid[m].kd;
    }
    // Slaves 0 and 1 carry four digital outputs and one relay each.
    for (std::size_t s = 0; s < 2; ++s) {
        protocol::SlaveSetpoint& sp = p2q.slaves[s];
        for (std::size_t i = 0; i < 4; ++i) {
            if (command.digital_output[4 * s + i]) {
                sp.digital_out |= static_cast<std::uint8_t>(kSlaveDigitalOut0 << i);
            }
        }
        if (command.relay[s]) {
            sp.digital_out |= kSlaveRelay;
        }
    }

    protocol::SlaveSetpoint& power = p2q.slaves[3];
    const double duty = std::clamp(command.power_output, -1.0, 1.0);
    power.misc_out = static_cast<std::uint8_t>((duty >= 0.0 ? kSlaveDirectionPositive : 0) |
                                               (command.reset_encoder_input ? kSlaveResetPosition : 0));
    power.velocity = static_cast<std::uint8_t>(std::lround(std::fabs(duty) * 255.0));
    return p2q;
}

Command from_p2q(const protocol::P2Q& p2q) {
    Command command;
    command.shutdown = (p2q.master_out & protocol::kMasterShutdown) != 0;
    for (std::size_t m = 0; m < kNumMotors; ++m) {
        const protocol::SlaveSetpoint& sp = p2q.slaves[m];
        const double magnitude = sp.velocity * kMotorVelocityResolution;
        command.motor_velocity[m] = (sp.misc_out & kSlaveDirectionPositive) ? magnitude : -magnitude;
        command.brake[m] = (sp.digital_out & kSlaveBrakeOff) == 0;
        command.reset_motor_position[m] = (sp.misc_out & kSlaveResetPosition) != 0;
        command.pid[m] = Pid{sp.kp, sp.ki, sp.kd};
    }
    for (std::size_t s = 0; s < 2; ++s) {
        const protocol::SlaveSetpoint& sp = p2q.slaves[s];
        for (std::size_t i = 0; i < 4; ++i) {
            command.digital_output[4 * s + i] = (sp.digital_out & (kSlaveDigitalOut0 << i)) != 0;
        }
        command.relay[s] = (sp.digital_out & kSlaveRelay) != 0;
    }
    const protocol::SlaveSetpoint& power = p2q.slaves[3];
    command.power_output = ((power.misc_out & kSlaveDirectionPositive) ? 1.0 : -1.0) * power.velocity / 255.0;
    command.reset_encoder_input = (power.misc_out & kSlaveResetPosition) != 0;
    return command;
}

SensorState from_io_status(const protocol::IOStatus& status) {
    const protocol::Q2P& q = status.q2p;
    SensorState s;
    s.raw = status;

    for (std::size_t m = 0; m < kNumMotors; ++m) {
        const protocol::SlaveReadings& r = q.slaves[m];
        s.motor_velocity[m] = signed_velocity(r);
        // Positions arrive with the opposite sign to velocities.
        s.motor_position[m] = -r.position;
        s.motor_current[m] = r.adc[kAdcMotorCurrent] / kScaleMotorCurrent;
    }
    for (std::size_t i = 0; i < kNumDistanceSensors; ++i) {
        const AdcChannel& a = kDistanceSensorAdc[i];
        s.distance_voltage[i] = kDistanceFullScale / kAdcCounts * q.slaves[a.slave].adc[a.channel];
    }
    for (std::size_t i = 0; i < kNumAnalogInputs; ++i) {
        const auto& adc = q.slaves[i / kAnalogInputsPerSlave].adc;
        s.analog_input[i] = kAnalogFullScale / kAdcCounts * adc[kAdcFirstAnalogInput + i % kAnalogInputsPerSlave];
    }
    for (std::size_t i = 0; i < kNumDigitalInputs; ++i) {
        s.digital_input[i] = (q.slaves[i / 4].digital_in & (1 << (i % 4))) != 0;
    }
    s.bumper = (q.slaves[0].digital_in & kSlaveBumper) != 0;

    s.battery_current = q.master_adc[kAdcBatteryCurrent] / kScaleBatteryCurrent;
    s.battery_voltage = q.master_adc[kAdcBatteryVoltage] / kScaleBatteryVoltage;

    const protocol::SlaveReadings& power = q.slaves[3];
    s.power_output_current = power.adc[kAdcMotorCurrent] / kScaleMotorCurrent;
    s.encoder_input_velocity = (power.misc_in & kSlaveVelocityPositive) ? power.velocity : -power.velocity;
    s.encoder_input_position = -power.position;

    // The master writes the firmware version bit-reversed into bits 4-7 of slave 0.
    const std::uint8_t misc = q.slaves[0].misc_in;
    const unsigned version_bits = ((misc >> 4) & 1) << 3 | ((misc >> 5) & 1) << 2 | ((misc >> 6) & 1) << 1 | ((misc >> 7) & 1);
    s.firmware_version = version_bits < kFirmwareVersions.size() ? kFirmwareVersions[version_bits] : 0;

    s.serial_frequency = status.serial_frequency;
    s.board_time = q.slaves[0].time;
    s.passive_mode = status.passive_mode;
    return s;
}

protocol::IOStatus to_io_status(const SensorState& state) {
    protocol::IOStatus status;
    protocol::Q2P& q = status.q2p;

    for (std::size_t m = 0; m < kNumMotors; ++m) {
        protocol::SlaveReadings& r = q.slaves[m];
        set_velocity(r, static_cast<int>(std::lround(state.motor_velocity[m] / kMotorVelocityResolution)));
        r.position = -state.motor_position[m];
        r.adc[kAdcMotorCurrent] = to_adc(state.motor_current[m], kScaleMotorCurrent);
    }
    for (std::size_t i = 0; i < kNumDistanceSensors; ++i) {
        const AdcChannel& a = kDistanceSensorAdc[i];
        q.slaves[a.slave].adc[a.channel] = to_adc(state.distance_voltage[i], kAdcCounts / kDistanceFullScale);
    }
    for (std::size_t i = 0; i < kNumAnalogInputs; ++i) {
        q.slaves[i / kAnalogInputsPerSlave].adc[kAdcFirstAnalogInput + i % kAnalogInputsPerSlave] =
            to_adc(state.analog_input[i], kAdcCounts / kAnalogFullScale);
    }
    for (std::size_t i = 0; i < kNumDigitalInputs; ++i) {
        if (state.digital_input[i]) {
            q.slaves[i / 4].digital_in |= static_cast<std::uint8_t>(1 << (i % 4));
        }
    }
    if (state.bumper) {
        q.slaves[0].digital_in |= kSlaveBumper;
    }

    q.master_adc[kAdcBatteryCurrent] = to_adc(state.battery_current, kScaleBatteryCurrent);
    q.master_adc[kAdcBatteryVoltage] = to_adc(state.battery_voltage, kScaleBatteryVoltage);

    protocol::SlaveReadings& power = q.slaves[3];
    power.adc[kAdcMotorCurrent] = to_adc(state.power_output_current, kScaleMotorCurrent);
    set_velocity(power, state.encoder_input_velocity);
    power.position = -state.encoder_input_position;

    const auto it = std::find(kFirmwareVersions.begin(), kFirmwareVersions.end(), state.firmware_version);
    if (it != kFirmwareVersions.end()) {
        const auto bits = static_cast<unsigned>(it - kFirmwareVersions.begin());
        q.slaves[0].misc_in |= static_cast<std::uint8_t>(((bits >> 3) & 1) << 4 | ((bits >> 2) & 1) << 5 |
                                                         ((bits >> 1) & 1) << 6 | (bits & 1) << 7);
    }

    q.slaves[0].time = state.board_time;
    status.serial_frequency = state.serial_frequency;
    status.passive_mode = state.passive_mode;
    return status;
}

}  // namespace robotino_api1

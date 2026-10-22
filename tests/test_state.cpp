#include <doctest/doctest.h>

#include "robotino_api1/state.hpp"

using namespace robotino_api1;

TEST_CASE("Motor velocity is quantized to 27 rpm steps and clamped") {
    Command c;
    c.motor_velocity = {kMotorVelocityResolution * 10.4, -kMotorVelocityResolution * 2.6, 10000.0};
    const protocol::P2Q p2q = to_p2q(c);
    CHECK(p2q.slaves[0].velocity == 10);
    CHECK(p2q.slaves[0].misc_out == protocol::kSlaveDirectionPositive);
    CHECK(p2q.slaves[1].velocity == 3);
    CHECK(p2q.slaves[1].misc_out == 0);
    CHECK(p2q.slaves[2].velocity == 255);
    CHECK(kMotorVelocityResolution * 60.0 / (2.0 * kPi) == doctest::Approx(27.0));
}

TEST_CASE("Command survives the P2Q round trip") {
    Command c;
    c.motor_velocity = {3 * kMotorVelocityResolution, -5 * kMotorVelocityResolution, 0.0};
    c.pid[1] = Pid{200, 10, 0};
    c.brake[2] = true;
    c.reset_motor_position[0] = true;
    c.digital_output = {true, false, true, false, false, true, false, true};
    c.relay = {false, true};
    c.power_output = -1.0;
    c.reset_encoder_input = true;
    c.shutdown = true;

    const auto bytes = protocol::encode(to_p2q(c));
    auto p2q = protocol::decode_p2q(protocol::ByteView(bytes.data(), bytes.size()));
    REQUIRE(p2q);
    const Command d = from_p2q(*p2q);
    for (std::size_t i = 0; i < kNumMotors; ++i) {
        CHECK(d.motor_velocity[i] == doctest::Approx(c.motor_velocity[i]));
        CHECK(d.pid[i].kp == c.pid[i].kp);
    }
    CHECK(d.brake == c.brake);
    CHECK(d.reset_motor_position == c.reset_motor_position);
    CHECK(d.digital_output == c.digital_output);
    CHECK(d.relay == c.relay);
    CHECK(d.power_output == doctest::Approx(-1.0));
    CHECK(d.reset_encoder_input);
    CHECK(d.shutdown);
}

TEST_CASE("SensorState survives the IOStatus round trip") {
    SensorState s;
    s.motor_velocity = {-4 * kMotorVelocityResolution, 0.0, 255 * kMotorVelocityResolution};
    s.motor_position = {-123456, 0, 2147483647};
    s.motor_current = {0.5, 1.0, 2.0};
    for (std::size_t i = 0; i < kNumDistanceSensors; ++i) {
        s.distance_voltage[i] = 0.25 * static_cast<double>(i);
    }
    for (std::size_t i = 0; i < kNumAnalogInputs; ++i) {
        s.analog_input[i] = 1.1 * static_cast<double>(i);
    }
    s.digital_input = {false, true, false, true, true, false, true, false};
    s.bumper = true;
    s.battery_voltage = 25.3;
    s.battery_current = 2.5;
    s.power_output_current = 1.5;
    s.encoder_input_position = -42;
    s.encoder_input_velocity = -7;
    s.serial_frequency = 55;
    s.board_time = 123456789;
    s.firmware_version = 1000;
    s.passive_mode = true;

    const auto bytes = protocol::encode(to_io_status(s));
    auto status = protocol::decode_io_status(protocol::ByteView(bytes.data() + 3, bytes.size() - 3));
    REQUIRE(status);
    const SensorState d = from_io_status(*status);

    // One ADC count of tolerance.
    for (std::size_t i = 0; i < kNumMotors; ++i) {
        CHECK(d.motor_velocity[i] == doctest::Approx(s.motor_velocity[i]));
        CHECK(d.motor_current[i] == doctest::Approx(s.motor_current[i]).epsilon(0.01));
    }
    CHECK(d.motor_position == s.motor_position);
    for (std::size_t i = 0; i < kNumDistanceSensors; ++i) {
        CHECK(d.distance_voltage[i] == doctest::Approx(s.distance_voltage[i]).epsilon(0.01));
    }
    for (std::size_t i = 0; i < kNumAnalogInputs; ++i) {
        CHECK(d.analog_input[i] == doctest::Approx(s.analog_input[i]).epsilon(0.01));
    }
    CHECK(d.digital_input == s.digital_input);
    CHECK(d.bumper);
    CHECK(d.battery_voltage == doctest::Approx(25.3).epsilon(0.01));
    CHECK(d.battery_current == doctest::Approx(2.5).epsilon(0.01));
    CHECK(d.power_output_current == doctest::Approx(1.5).epsilon(0.01));
    CHECK(d.encoder_input_position == -42);
    CHECK(d.encoder_input_velocity == -7);
    CHECK(d.serial_frequency == 55);
    CHECK(d.board_time == 123456789);
    CHECK(d.firmware_version == 1000);
    CHECK(d.passive_mode);
}

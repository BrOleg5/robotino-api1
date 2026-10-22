#include <doctest/doctest.h>

#include "robotino_api1/protocol/messages.hpp"
#include "robotino_api1/state.hpp"
#include "test_util.hpp"

using namespace robotino_api1;
using namespace robotino_api1::protocol;

TEST_CASE("IOControl of an idle robot matches OpenRobotinoAPI byte for byte") {
    const auto captured = read_fixture("iocontrol_0.bin");
    REQUIRE(captured.size() == 79);

    IOControl msg;
    msg.p2q = to_p2q(Command{});
    CHECK(encode(msg) == captured);
}

TEST_CASE("IOControl with reset motor positions matches OpenRobotinoAPI") {
    const auto captured = read_fixture("iocontrol_457.bin");
    Command command;
    command.reset_motor_position = {true, true, true};
    IOControl msg;
    msg.p2q = to_p2q(command);
    CHECK(encode(msg) == captured);
}

TEST_CASE("IOControl decodes what it encodes") {
    const auto captured = read_fixture("iocontrol_457.bin");
    REQUIRE(captured[0] == static_cast<std::uint8_t>(MessageId::IOControl));
    auto msg = decode_io_control(payload_of(captured));
    REQUIRE(msg);
    CHECK_FALSE(msg->image_request);
    CHECK(msg->image_port == 8080);
    CHECK(msg->northstar_ceiling_cal == 1.0f);
    CHECK(encode(*msg) == captured);
    const Command c = from_p2q(msg->p2q);
    CHECK(c.reset_motor_position == std::array<bool, 3>{true, true, true});
    CHECK(c.pid[0].kp == 255);
}

TEST_CASE("CameraControl at connect matches OpenRobotinoAPI") {
    CHECK(encode(CameraControl{}) == read_fixture("cameracontrol_0.bin"));
}

TEST_CASE("IOStatus from a real robot") {
    const auto captured = read_fixture("iostatus_0.bin");
    REQUIRE(captured.size() == 144);
    auto status = decode_io_status(payload_of(captured));
    REQUIRE(status);
    CHECK(status->serial_frequency == 54);
    CHECK(encode(*status) == captured);

    const SensorState s = from_io_status(*status);
    CHECK(s.motor_position == std::array<std::int32_t, 3>{820, 1527, -736});
    CHECK(s.motor_velocity == std::array<double, 3>{0.0, 0.0, 0.0});
    CHECK(s.firmware_version == 670);  // the Info message says "firmware: 670"
    CHECK(s.battery_voltage == doctest::Approx(1020 / 36.10));
    CHECK_FALSE(s.bumper);
    // Unconnected inputs read high.
    CHECK(s.digital_input == std::array<bool, 8>{true, true, true, true, true, true, true, true});
    CHECK_FALSE(s.passive_mode);
}

TEST_CASE("IOStatus after reset reports zero positions") {
    auto status = decode_io_status(payload_of(read_fixture("iostatus_489.bin")));
    REQUIRE(status);
    CHECK(from_io_status(*status).motor_position == std::array<std::int32_t, 3>{0, 0, 0});
}

TEST_CASE("Info from a real robot") {
    const auto captured = read_fixture("info_0.bin");
    REQUIRE(captured.size() == 2004);
    auto info = decode_info(payload_of(captured));
    REQUIRE(info);
    CHECK(info->text.find("firmware: 670") != std::string::npos);
    CHECK_FALSE(info->passive_mode);
    CHECK(encode(*info) == captured);
}

TEST_CASE("Decoders reject wrong sizes and markers") {
    auto payload = payload_of(read_fixture("iostatus_0.bin"));
    CHECK_FALSE(decode_io_status(ByteView(payload.data(), payload.size() - 1)));
    payload[2] = 'X';  // first byte of "REC"
    CHECK_FALSE(decode_io_status(payload));
    CHECK_FALSE(decode_io_control(ByteView()));
    CHECK_FALSE(decode_info(ByteView()));
    const std::uint8_t bad_camera[] = {9, 0, 0, 0, 0};
    CHECK_FALSE(decode_camera_control(bad_camera));
}

TEST_CASE("MessageParser splits and joins TCP segments") {
    std::vector<std::uint8_t> stream = read_fixture("info_0.bin");
    const auto status = read_fixture("iostatus_0.bin");
    stream.insert(stream.end(), status.begin(), status.end());
    stream.insert(stream.end(), status.begin(), status.end());

    for (std::size_t chunk : {std::size_t{1}, std::size_t{7}, std::size_t{1460}, stream.size()}) {
        CAPTURE(chunk);
        MessageParser parser;
        std::vector<std::uint8_t> ids;
        for (std::size_t pos = 0; pos < stream.size(); pos += chunk) {
            const std::size_t n = std::min(chunk, stream.size() - pos);
            parser.feed(stream.data() + pos, n);
            while (auto msg = parser.next()) {
                ids.push_back(msg->id);
                if (msg->message_id() == MessageId::IOStatus) {
                    CHECK(decode_io_status(msg->payload));
                }
            }
        }
        CHECK(ids == std::vector<std::uint8_t>{6, 1, 1});
        CHECK_FALSE(parser.error());
    }
}

TEST_CASE("MessageParser stops on an impossible length") {
    MessageParser parser;
    const std::uint8_t garbage[] = {1, 0xFF, 0xFF, 0, 0};
    parser.feed(garbage, sizeof garbage);
    CHECK_FALSE(parser.next());
    CHECK(parser.error());
    parser.reset();
    const auto keep_alive = encode_empty(MessageId::KeepAlive);
    parser.feed(keep_alive.data(), keep_alive.size());
    auto msg = parser.next();
    REQUIRE(msg);
    CHECK(msg->message_id() == MessageId::KeepAlive);
    CHECK(msg->payload.empty());
}

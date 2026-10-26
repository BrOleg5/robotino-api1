// Client against the emulator over loopback.
#include <doctest/doctest.h>

#include <atomic>
#include <cmath>
#include <thread>

#include "robotino_api1/client.hpp"
#include "simulator.hpp"

using namespace robotino_api1;
using namespace std::chrono_literals;

namespace {

sim::SimulatorOptions sim_options() {
    sim::SimulatorOptions o;
    o.address = "127.0.0.1";
    o.port = 0;
    o.camera_period = 20ms;
    o.camera_width = 64;
    o.camera_height = 48;
    return o;
}

ClientOptions client_options(const sim::Simulator& s) {
    ClientOptions o;
    o.host = "127.0.0.1";
    o.port = s.port();
    o.cycle = 10ms;
    o.timeout = 300ms;
    o.image_port = 0;  // any free port
    return o;
}

// Polls a condition for up to the given time.
template <typename F>
bool eventually(F condition, std::chrono::milliseconds timeout = 2000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (condition()) {
            return true;
        }
        std::this_thread::sleep_for(5ms);
    }
    return condition();
}

}  // namespace

TEST_CASE("Client connects, receives Info and readings") {
    sim::Simulator s(sim_options());
    REQUIRE_FALSE(s.start());
    s.modify([](SensorState& st) {
        st.bumper = true;
        st.distance_voltage[3] = 1.5;
    });

    Client client(client_options(s));
    std::atomic<int> updates{0};
    client.on_update([&](const SensorState&) { ++updates; });
    REQUIRE_FALSE(client.connect());
    CHECK(client.connection_state() == ConnectionState::Connected);
    CHECK(client.info().text.find("robotino-api1-sim") != std::string::npos);

    CHECK(client.wait_for_update(1000ms));
    const SensorState st = client.sensors();
    CHECK(st.bumper);
    CHECK(st.distance_voltage[3] == doctest::Approx(1.5).epsilon(0.01));
    CHECK(st.battery_voltage == doctest::Approx(24.0).epsilon(0.01));
    CHECK(st.firmware_version == 670);
    CHECK(eventually([&] { return updates > 10; }));

    client.disconnect();
    CHECK(client.connection_state() == ConnectionState::Disconnected);
}

TEST_CASE("Commands reach the robot and the motors move") {
    sim::Simulator s(sim_options());
    REQUIRE_FALSE(s.start());
    Client client(client_options(s));
    REQUIRE_FALSE(client.connect());

    client.set_motor_velocities({10 * kMotorVelocityResolution, -5 * kMotorVelocityResolution, 0.0});
    client.set_digital_output(5, true);
    client.set_relay(1, true);
    client.set_pid(2, Pid{100, 20, 3});
    CHECK(eventually([&] { return s.command().motor_velocity[0] > 0.0; }));
    const Command c = s.command();
    CHECK(c.motor_velocity[0] == doctest::Approx(10 * kMotorVelocityResolution));
    CHECK(c.motor_velocity[1] == doctest::Approx(-5 * kMotorVelocityResolution));
    CHECK(c.digital_output[5]);
    CHECK(c.relay[1]);
    CHECK(c.pid[2].kp == 100);

    CHECK(eventually([&] {
        const SensorState st = client.sensors();
        return st.motor_position[0] > 1000 && st.motor_position[1] < -500;
    }));
    CHECK(client.sensors().motor_velocity[0] == doctest::Approx(10 * kMotorVelocityResolution).epsilon(0.15));

    client.reset_motor_positions();
    client.stop();
    CHECK(eventually([&] { return std::abs(client.sensors().motor_position[0]) < 100; }));

    // Disconnecting leaves the motors stopped.
    client.set_motor_velocities({20.0, 20.0, 20.0});
    CHECK(eventually([&] { return s.command().motor_velocity[0] > 0.0; }));
    client.disconnect();
    CHECK(eventually([&] { return s.command().motor_velocity[0] == 0.0; }));
}

TEST_CASE("Odometry follows a driven robot") {
    sim::SimulatorOptions so = sim_options();
    so.motor_time_constant = 0.0;
    sim::Simulator s(so);
    REQUIRE_FALSE(s.start());
    Client client(client_options(s));
    REQUIRE_FALSE(client.connect());

    // 0.1 m/s is about 40 rad/s at the motor, well above the 2.8 rad/s resolution.
    client.set_velocity(Twist{0.1, 0.0, 0.0});
    std::this_thread::sleep_for(1000ms);
    client.stop();
    std::this_thread::sleep_for(100ms);
    const Pose p = client.pose();
    CHECK(p.x == doctest::Approx(0.1).epsilon(0.3));
    CHECK(std::abs(p.y) < 0.02);
    CHECK(std::abs(p.theta) < 0.1);
}

TEST_CASE("Command watchdog stops the motors") {
    sim::Simulator s(sim_options());
    REQUIRE_FALSE(s.start());
    ClientOptions o = client_options(s);
    o.command_timeout = 100ms;
    Client client(o);
    REQUIRE_FALSE(client.connect());
    client.set_motor_velocity(0, 30.0);
    CHECK(eventually([&] { return s.command().motor_velocity[0] > 0.0; }, 500ms));
    CHECK(eventually([&] { return s.command().motor_velocity[0] == 0.0; }, 1000ms));
}

TEST_CASE("Connection errors") {
    SUBCASE("refused") {
        sim::Simulator s(sim_options());
        REQUIRE_FALSE(s.start());
        const std::uint16_t port = s.port();
        s.stop();
        ClientOptions o;
        o.host = "127.0.0.1";
        o.port = port;
        // Windows retransmits the SYN after a RST and reports the refusal only after ~2 s.
        o.timeout = 5000ms;
        Client client(o);
        const Error e = client.connect();
        CHECK(e.code == ErrorCode::ConnectionRefused);
        CHECK(client.connection_state() == ConnectionState::Disconnected);
    }
    SUBCASE("unknown host") {
        ClientOptions o;
        o.host = "no-such-host.invalid";
        o.timeout = 300ms;
        Client client(o);
        CHECK(client.connect().code == ErrorCode::HostNotFound);
    }
    SUBCASE("busy") {
        sim::SimulatorOptions so = sim_options();
        SUBCASE("passive mode") {}
        SUBCASE("ConnectionBusy message") { so.busy_message = true; }
        sim::Simulator s(so);
        REQUIRE_FALSE(s.start());
        Client first(client_options(s));
        REQUIRE_FALSE(first.connect());
        Client second(client_options(s));
        CHECK(second.connect().code == ErrorCode::ConnectionBusy);
        CHECK(second.connection_state() == ConnectionState::Disconnected);
        CHECK(first.connection_state() == ConnectionState::Connected);
    }
}

TEST_CASE("A second client with allow_passive reads but does not control") {
    sim::Simulator s(sim_options());
    REQUIRE_FALSE(s.start());
    Client first(client_options(s));
    REQUIRE_FALSE(first.connect());
    ClientOptions o = client_options(s);
    o.allow_passive = true;
    Client second(o);
    REQUIRE_FALSE(second.connect());
    CHECK(second.info().passive_mode);
    CHECK(second.wait_for_update(1000ms));
    CHECK(second.sensors().passive_mode);
    CHECK_FALSE(first.sensors().passive_mode);

    second.set_motor_velocities({20.0, 20.0, 20.0});
    first.set_motor_velocities({-20.0, 0.0, 0.0});
    CHECK(eventually([&] { return s.command().motor_velocity[0] < 0.0; }));
    std::this_thread::sleep_for(100ms);
    CHECK(s.command().motor_velocity[1] == 0.0);
    CHECK(eventually([&] { return second.sensors().motor_velocity[0] < -10.0; }));
}

TEST_CASE("Client reconnects after the connection is lost") {
    sim::Simulator s(sim_options());
    REQUIRE_FALSE(s.start());
    Client client(client_options(s));
    std::atomic<int> errors{0};
    client.on_error([&](const Error&) { ++errors; });
    REQUIRE_FALSE(client.connect());

    s.kick();
    CHECK(eventually([&] { return errors > 0; }));
    CHECK(eventually([&] { return s.connections() == 2 && client.connection_state() == ConnectionState::Connected; }));
    CHECK(client.wait_for_update(1000ms));

    // A robot that stops answering times out.
    errors = 0;
    s.set_paused(true);
    CHECK(eventually([&] { return errors > 0; }));
    s.set_paused(false);
    CHECK(eventually([&] { return client.connection_state() == ConnectionState::Connected; }, 3000ms));
    CHECK(client.wait_for_update(1000ms));
}

TEST_CASE("Reconnecting retries while another client controls the robot") {
    sim::Simulator s(sim_options());
    REQUIRE_FALSE(s.start());
    Client client(client_options(s));
    std::atomic<int> busy{0};
    client.on_error([&](const Error& e) { busy += e.code == ErrorCode::ConnectionBusy; });
    REQUIRE_FALSE(client.connect());

    // Another client takes the robot while this one reconnects.
    Client other(client_options(s));
    s.kick();
    CHECK(eventually([&] { return !other.connect(); }));
    CHECK(eventually([&] { return busy > 1; }, 3000ms));
    CHECK(client.connection_state() == ConnectionState::Connecting);

    other.disconnect();
    CHECK(eventually([&] { return client.connection_state() == ConnectionState::Connected; }, 4000ms));
    CHECK(client.wait_for_update(1000ms));
}

TEST_CASE("Camera images arrive over UDP") {
    sim::Simulator s(sim_options());
    REQUIRE_FALSE(s.start());
    Client client(client_options(s));
    std::atomic<int> images{0};
    client.on_image([&](const Image& img) {
        if (img.width == 64 && img.height == 48 && img.data.size() == 64 * 48 * 3) {
            ++images;
        }
    });
    REQUIRE_FALSE(client.set_camera_streaming(true));
    REQUIRE_FALSE(client.connect());
    CHECK(eventually([&] { return images >= 3; }));
    Image latest;
    CHECK(client.latest_image(latest));
    CHECK(latest.type == protocol::ImageType::Raw);

    REQUIRE_FALSE(client.set_camera_streaming(false));
    const auto sent = s.images_sent();
    std::this_thread::sleep_for(200ms);
    CHECK(s.images_sent() <= sent + 1);
}

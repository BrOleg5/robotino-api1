// Connection to a Robotino over the OpenRobotinoAPI 1 protocol.
//
// A background thread exchanges an IOControl/IOStatus pair with the robot every cycle,
// reconnects after errors and, while streaming is on, receives camera images. All methods
// are thread-safe. Callbacks run on the background threads and must not block for long;
// they may call any method except connect(), disconnect() and the destructor.
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "robotino_api1/error.hpp"
#include "robotino_api1/export.hpp"
#include "robotino_api1/kinematics.hpp"
#include "robotino_api1/protocol/image.hpp"
#include "robotino_api1/protocol/messages.hpp"
#include "robotino_api1/state.hpp"

namespace robotino_api1 {

using Milliseconds = std::chrono::milliseconds;

struct ClientOptions {
    std::string host = "172.26.1.0";
    std::uint16_t port = 80;
    // Period of the IOControl/IOStatus exchange. OpenRobotinoAPI uses 30 ms by default;
    // the IO board itself is read at about 55 Hz.
    Milliseconds cycle{20};
    // Connection attempt and answer timeout.
    Milliseconds timeout{1000};
    // Reconnect after the connection is lost (not after ConnectionBusy).
    bool auto_reconnect = true;
    // Stop the motors if no velocity was set for this long; 0 disables the watchdog.
    Milliseconds command_timeout{0};
    // Local UDP port for camera images; the next free port is used if it is taken.
    std::uint16_t image_port = 8080;
    DriveLayout drive_layout{};
    // How many cycles reset flags (motor positions, encoder input) are kept set.
    int reset_cycles = 3;
};

enum class ConnectionState { Disconnected, Connecting, Connected };

ROBOTINO_API1_EXPORT const char* to_string(ConnectionState state);

struct CameraSettings {
    protocol::CameraResolution resolution = protocol::CameraResolution::QVGA;
    protocol::CameraCompression compression = protocol::CameraCompression::High;
    std::uint8_t brightness = 0;
    std::uint8_t contrast = 0;
    bool auto_white_balance = false;
};

struct Image {
    std::chrono::steady_clock::time_point stamp{};
    protocol::ImageType type = protocol::ImageType::Unknown;  // JPEG or Raw in practice
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint8_t channels = 0;
    std::uint8_t bits_per_channel = 0;
    std::vector<std::uint8_t> data;  // JPEG file or rows of pixels
};

struct RobotInfo {
    std::string text;  // as sent by the robot, e.g. "iod: 695\nfirmware: 670\n..."
    bool passive_mode = false;
};

class ROBOTINO_API1_EXPORT Client {
public:
    explicit Client(ClientOptions options = {});
    // Stops the motors and disconnects.
    ~Client();
    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    // Connects and waits for the first IOStatus. On success the background thread keeps the
    // connection alive until disconnect(). Returns the error if the first attempt fails.
    Error connect();
    // Sends a last command with stopped motors and closes the connection.
    void disconnect();
    ConnectionState connection_state() const;
    const ClientOptions& options() const;

    // Commands. They are sent with the next cycle and stay in effect until changed.
    void set_motor_velocities(const std::array<double, kNumMotors>& rad_s);
    void set_motor_velocity(std::size_t motor, double rad_s);
    // Robot velocity in m/s and rad/s, converted with the drive layout.
    void set_velocity(const Twist& twist);
    void stop();
    void set_pid(std::size_t motor, const Pid& pid);
    void set_brake(std::size_t motor, bool engaged);
    void reset_motor_positions();
    void set_digital_output(std::size_t output, bool value);
    void set_relay(std::size_t relay, bool value);
    void set_power_output(double duty);  // -1..1
    void reset_encoder_input();
    void request_shutdown();  // switches the robot off
    Command command() const;
    // Replaces the whole command at once.
    void set_command(const Command& command);

    // Readings.
    SensorState sensors() const;
    // Waits for an IOStatus newer than the last one returned by sensors() or wait_for_update().
    bool wait_for_update(Milliseconds timeout);
    RobotInfo info() const;

    // Odometry integrated from the motor encoders since connect() or set_pose().
    Pose pose() const;
    void set_pose(const Pose& pose);
    // Robot velocity computed from the measured motor velocities.
    Twist measured_velocity() const;

    // Camera.
    void set_camera_settings(const CameraSettings& settings);
    // Opens the UDP port and asks the robot to stream images.
    Error set_camera_streaming(bool enable);
    bool camera_streaming() const;
    // The latest image, if any.
    bool latest_image(Image& image) const;

    // Callbacks; pass nullptr to remove.
    void on_update(std::function<void(const SensorState&)> callback);
    void on_connection_changed(std::function<void(ConnectionState)> callback);
    void on_error(std::function<void(const Error&)> callback);
    void on_info(std::function<void(const RobotInfo&)> callback);
    void on_image(std::function<void(const Image&)> callback);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace robotino_api1

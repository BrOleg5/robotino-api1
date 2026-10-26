// Emulator of a Robotino with the OpenRobotinoAPI 1 protocol: speaks the robot side of the protocol on a TCP port.
//
// Motors follow their setpoints with a first-order lag and integrate their positions;
// the other readings come from a configurable SensorState. With an image request it sends a
// synthetic raw RGB image over UDP. Only one client controls it at a time, others receive
// ConnectionBusy (the robot serves them in passive mode). Like the robot, it drops a client that
// is silent for 2 s.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "robotino_api1/error.hpp"
#include "robotino_api1/state.hpp"

namespace robotino_api1::sim {

struct SimulatorOptions {
    std::string address = "0.0.0.0";
    std::uint16_t port = 8000;  // 0 picks a free port
    double motor_time_constant = 0.05;  // s
    std::chrono::milliseconds idle_timeout{2000};
    std::chrono::milliseconds camera_period{100};
    std::uint32_t camera_width = 320;
    std::uint32_t camera_height = 240;
    std::string info_text = "camd: 695\niod: 695\nlcdd: 695\ncom: 695\nfirmware: 670\n\nrobotino-api1-sim\n";
};

class Simulator {
public:
    explicit Simulator(SimulatorOptions options = {});
    ~Simulator();
    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;

    Error start();
    void stop();
    std::uint16_t port() const { return port_; }

    // The last command received from the client.
    Command command() const;
    // Changes the emulated readings (bumper, distance sensors, battery...).
    void modify(const std::function<void(SensorState&)>& fn);
    SensorState state() const;
    // While paused, IOControl messages are not answered.
    void set_paused(bool paused) { paused_ = paused; }
    // Closes the current client connection.
    void kick();
    std::uint64_t io_controls() const { return io_controls_; }
    std::uint64_t images_sent() const { return images_sent_; }
    std::uint64_t connections() const { return connections_; }

private:
    void serve();
    void camera_loop();
    void step(std::chrono::steady_clock::time_point now);

    const SimulatorOptions options_;
    std::uint16_t port_ = 0;
    std::thread server_thread_;
    std::thread camera_thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> kick_{false};
    std::atomic<std::uint64_t> io_controls_{0};
    std::atomic<std::uint64_t> images_sent_{0};
    std::atomic<std::uint64_t> connections_{0};

    mutable std::mutex mutex_;
    Command command_;
    SensorState state_;
    std::array<double, kNumMotors> position_{};  // ticks, fractional
    std::chrono::steady_clock::time_point last_step_{};
    std::string client_address_;
    bool image_request_ = false;
    std::uint16_t image_port_ = 0;

    struct Listener;
    std::unique_ptr<Listener> listener_;
};

}  // namespace robotino_api1::sim

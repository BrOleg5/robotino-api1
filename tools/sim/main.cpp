// robotino-api1-sim: runs the robot emulator until interrupted.
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "simulator.hpp"

namespace {

std::atomic<bool> g_stop{false};

void on_signal(int) { g_stop = true; }

void usage() {
    std::cout << "Usage: robotino-api1-sim [--port N] [--address A] [--verbose]\n"
                 "Emulates a Robotino (OpenRobotinoAPI 1 protocol) on TCP port N (default 8000).\n";
}

}  // namespace

int main(int argc, char** argv) {
    robotino_api1::sim::SimulatorOptions options;
    bool verbose = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            options.port = static_cast<std::uint16_t>(std::atoi(argv[++i]));
        } else if (arg == "--address" && i + 1 < argc) {
            options.address = argv[++i];
        } else if (arg == "--verbose") {
            verbose = true;
        } else {
            usage();
            return arg == "--help" || arg == "-h" ? 0 : 2;
        }
    }

    robotino_api1::sim::Simulator sim(options);
    if (robotino_api1::Error e = sim.start()) {
        std::cerr << "robotino-api1-sim: " << e.message << "\n";
        return 1;
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::cout << "robotino-api1-sim listening on " << options.address << ":" << sim.port() << std::endl;

    std::uint64_t connections = 0;
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (sim.connections() != connections) {
            connections = sim.connections();
            std::cout << "client connected (" << connections << " so far)" << std::endl;
        }
        if (verbose) {
            const robotino_api1::Command c = sim.command();
            const robotino_api1::SensorState s = sim.state();
            std::cout << "setpoint rad/s: " << c.motor_velocity[0] << " " << c.motor_velocity[1] << " "
                      << c.motor_velocity[2] << "  position: " << s.motor_position[0] << " " << s.motor_position[1]
                      << " " << s.motor_position[2] << std::endl;
        }
    }
    sim.stop();
    return 0;
}

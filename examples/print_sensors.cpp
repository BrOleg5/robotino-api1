// Prints the robot's readings for ten seconds.
// Usage: print_sensors [host[:port]]
#include <cstdio>
#include <thread>

#include "common.hpp"

int main(int argc, char** argv) {
    robotino_api1::ClientOptions options = options_from_args(argc, argv);
    // Only reads, so it also works while another program controls the robot.
    options.allow_passive = true;
    robotino_api1::Client robot(options);
    print_connection_events(robot);
    if (robotino_api1::Error e = robot.connect()) {
        std::fprintf(stderr, "connect: %s\n", e.message.c_str());
        return 1;
    }
    std::printf("%s\n", robot.info().text.c_str());
    if (robot.info().passive_mode) {
        std::printf("passive mode: another client controls the robot\n");
    }

    for (int i = 0; i < 20; ++i) {
        robot.wait_for_update(std::chrono::milliseconds(500));
        const robotino_api1::SensorState s = robot.sensors();
        std::printf("battery %.2f V %.2f A | bumper %d | position %d %d %d | distance V:", s.battery_voltage,
                    s.battery_current, s.bumper, s.motor_position[0], s.motor_position[1], s.motor_position[2]);
        for (double v : s.distance_voltage) {
            std::printf(" %.2f", v);
        }
        std::printf("\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return 0;
}

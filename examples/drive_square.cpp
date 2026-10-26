// Drives a 0.5 m square without turning and prints the odometry.
// Usage: drive_square [host[:port]]
#include <cstdio>
#include <thread>

#include "common.hpp"

int main(int argc, char** argv) {
    robotino_api1::ClientOptions options = options_from_args(argc, argv);
    // Stop if this program hangs or dies without disconnecting.
    options.command_timeout = std::chrono::milliseconds(500);
    robotino_api1::Client robot(options);
    print_connection_events(robot);
    if (robotino_api1::Error e = robot.connect()) {
        std::fprintf(stderr, "connect: %s\n", e.message.c_str());
        return 1;
    }

    const double speed = 0.1;  // m/s
    const robotino_api1::Twist sides[] = {{speed, 0, 0}, {0, speed, 0}, {-speed, 0, 0}, {0, -speed, 0}};
    for (const robotino_api1::Twist& side : sides) {
        // Repeat the command so that the watchdog does not stop the robot.
        for (int i = 0; i < 50; ++i) {
            robot.set_velocity(side);
            if (robot.sensors().bumper) {
                std::fprintf(stderr, "bumper hit, stopping\n");
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        const robotino_api1::Pose p = robot.pose();
        std::printf("pose: x %.3f m, y %.3f m, theta %.3f rad\n", p.x, p.y, p.theta);
    }
    robot.stop();
    return 0;
}

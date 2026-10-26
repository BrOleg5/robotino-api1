// Hardware check of the motors (step 2 of docs/hardware-testing.md). Put the robot on a stand
// with the wheels in the air. Each motor in turn runs at +30 and -30 rad/s; the program checks
// the measured velocity and position, the brake and reset_motor_positions(), and prints PASS
// or FAIL for each check. Watch that the named wheel is the one that turns.
// Usage: check_motors [host[:port]]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <thread>

#include "common.hpp"

namespace {

using robotino_api1::Client;
using robotino_api1::kNumMotors;
using robotino_api1::Milliseconds;
using robotino_api1::SensorState;
using Clock = std::chrono::steady_clock;

constexpr double kVelocity = 30.0;  // rad/s at the motor shaft
// One velocity unit on the wire is 2.83 rad/s, so 30 rad/s is sent as 31.1 rad/s.
constexpr double kVelocityTolerance = 0.2;  // relative
constexpr double kStillVelocity = robotino_api1::kMotorVelocityResolution * 1.5;
constexpr double kPositionTolerance = 0.3;  // relative, position change against integrated velocity
constexpr int kStillTicks = 20;
constexpr double kTicksPerRad = robotino_api1::kEncoderTicksPerRevolution / (2.0 * robotino_api1::kPi);

int failures = 0;

void check(bool ok, const char* what) {
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) {
        ++failures;
    }
}

// Velocity and position of one motor over a time window.
struct Window {
    double mean_velocity = 0.0;
    double max_other_velocity = 0.0;  // largest |velocity| of the other motors
    double position_change = 0.0;     // ticks
    double expected_change = 0.0;     // ticks, integrated from the measured velocity
};

// Keeps the motor velocity set (the watchdog stops the motors otherwise) for skip + measure,
// and collects the readings of the last `measure`.
Window run(Client& robot, std::size_t motor, double rad_s, Milliseconds skip, Milliseconds measure) {
    const Clock::time_point start = Clock::now();
    Window w;
    int samples = 0;
    bool first = true;
    SensorState previous;
    while (Clock::now() - start < skip + measure) {
        robot.set_motor_velocity(motor, rad_s);
        if (!robot.wait_for_update(Milliseconds(200))) {
            continue;
        }
        const SensorState s = robot.sensors();
        if (s.stamp - start < skip) {
            continue;
        }
        if (first) {
            first = false;
        } else {
            const double dt = std::chrono::duration<double>(s.stamp - previous.stamp).count();
            w.expected_change += 0.5 * (s.motor_velocity[motor] + previous.motor_velocity[motor]) * dt * kTicksPerRad;
            w.position_change += s.motor_position[motor] - previous.motor_position[motor];
        }
        w.mean_velocity += s.motor_velocity[motor];
        ++samples;
        for (std::size_t m = 0; m < kNumMotors; ++m) {
            if (m != motor) {
                w.max_other_velocity = std::max(w.max_other_velocity, std::abs(s.motor_velocity[m]));
            }
        }
        previous = s;
    }
    if (samples > 0) {
        w.mean_velocity /= samples;
    }
    return w;
}

void check_direction(Client& robot, std::size_t motor, double rad_s) {
    std::printf("motor %zu at %+.0f rad/s\n", motor, rad_s);
    const Window w = run(robot, motor, rad_s, Milliseconds(1000), Milliseconds(1500));
    std::printf("  measured %+.2f rad/s, other motors up to %.2f rad/s\n", w.mean_velocity, w.max_other_velocity);
    std::printf("  position change %+.0f ticks, expected %+.0f\n", w.position_change, w.expected_change);
    check(w.mean_velocity * rad_s > 0, "velocity has the commanded sign");
    check(std::abs(w.mean_velocity - rad_s) < kVelocityTolerance * std::abs(rad_s), "velocity is close to the command");
    check(w.max_other_velocity < kStillVelocity, "other motors stand still");
    check(w.position_change * w.mean_velocity > 0, "position changes in the direction of the velocity");
    check(std::abs(w.position_change - w.expected_change) < kPositionTolerance * std::abs(w.expected_change),
          "position change matches the velocity");
    robot.stop();
}

void check_brake(Client& robot, std::size_t motor) {
    std::printf("motor %zu at %+.0f rad/s with the brake engaged\n", motor, kVelocity);
    run(robot, motor, kVelocity, Milliseconds(0), Milliseconds(1000));
    robot.set_brake(motor, true);
    const Window w = run(robot, motor, kVelocity, Milliseconds(1000), Milliseconds(1000));
    robot.set_brake(motor, false);
    robot.stop();
    std::printf("  measured %+.2f rad/s, position change %+.0f ticks\n", w.mean_velocity, w.position_change);
    check(std::abs(w.mean_velocity) < kStillVelocity, "brake stops the wheel");
    check(std::abs(w.position_change) < kStillTicks, "position stays put");
}

void check_reset(Client& robot) {
    std::printf("reset_motor_positions()\n");
    robot.stop();
    std::this_thread::sleep_for(Milliseconds(1000));
    const SensorState before = robot.sensors();
    std::printf("  positions before %d %d %d\n", before.motor_position[0], before.motor_position[1],
                before.motor_position[2]);
    robot.reset_motor_positions();
    // The reset flag is kept for reset_cycles cycles; give the robot a few more.
    for (int i = 0; i < robot.options().reset_cycles + 10; ++i) {
        robot.wait_for_update(Milliseconds(200));
    }
    const SensorState s = robot.sensors();
    std::printf("  positions after %d %d %d\n", s.motor_position[0], s.motor_position[1], s.motor_position[2]);
    bool zero = true;
    for (std::int32_t p : s.motor_position) {
        zero = zero && std::abs(p) < kStillTicks;
    }
    check(zero, "positions are zero");
}

}  // namespace

int main(int argc, char** argv) {
    robotino_api1::ClientOptions options = options_from_args(argc, argv);
    // Stop if this program hangs or dies without disconnecting.
    options.command_timeout = Milliseconds(500);
    Client robot(options);
    if (robotino_api1::Error e = robot.connect()) {
        std::fprintf(stderr, "connect: %s\n", e.message.c_str());
        return 1;
    }
    std::printf("%s\n", robot.info().text.c_str());

    for (std::size_t motor = 0; motor < kNumMotors; ++motor) {
        check_direction(robot, motor, kVelocity);
        check_direction(robot, motor, -kVelocity);
        check_brake(robot, motor);
        check_reset(robot);
    }
    robot.stop();
    std::printf("%s: %d failed checks\n", failures == 0 ? "OK" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}

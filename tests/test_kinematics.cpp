#include <doctest/doctest.h>

#include <cmath>

#include "robotino_api1/kinematics.hpp"

using namespace robotino_api1;

TEST_CASE("Forward kinematics inverts inverse kinematics") {
    const DriveLayout layout;
    for (const Twist t : {Twist{0.3, 0.0, 0.0}, Twist{0.0, -0.2, 0.0}, Twist{0.0, 0.0, 1.0}, Twist{0.1, 0.2, -0.5}}) {
        const Twist r = forward_kinematics(layout, inverse_kinematics(layout, t));
        CHECK(r.vx == doctest::Approx(t.vx));
        CHECK(r.vy == doctest::Approx(t.vy));
        CHECK(r.omega == doctest::Approx(t.omega));
    }
}

TEST_CASE("Inverse kinematics matches OpenRobotinoAPI OmniDrive") {
    // OpenRobotinoAPI: speed_i = (v_i . (vx, vy) + rb * omega) * gear / rw, wheel directions
    // v0 = (-sqrt(3)/2, 1/2), v1 = (0, -1), v2 = (sqrt(3)/2, 1/2).
    const DriveLayout layout;
    const auto m = inverse_kinematics(layout, Twist{1.0, 0.0, 0.0});
    const double k = layout.gear_ratio / layout.wheel_radius;
    CHECK(m[0] == doctest::Approx(-std::sqrt(3.0) / 2.0 * k));
    CHECK(m[1] == doctest::Approx(0.0));
    CHECK(m[2] == doctest::Approx(std::sqrt(3.0) / 2.0 * k));
    const auto r = inverse_kinematics(layout, Twist{0.0, 0.0, 1.0});
    for (double v : r) {
        CHECK(v == doctest::Approx(layout.base_radius * k));
    }
}

TEST_CASE("Odometry integrates straight and rotational motion") {
    const DriveLayout layout;
    auto ticks_for = [&](const Twist& t, double seconds) {
        const auto m = inverse_kinematics(layout, t);
        std::array<std::int32_t, kNumMotors> p{};
        for (std::size_t i = 0; i < kNumMotors; ++i) {
            p[i] = static_cast<std::int32_t>(std::lround(m[i] * seconds * kEncoderTicksPerRevolution / (2.0 * kPi)));
        }
        return p;
    };

    Odometry straight(layout);
    straight.update({0, 0, 0});
    straight.update(ticks_for(Twist{0.5, 0.0, 0.0}, 2.0));
    CHECK(straight.pose().x == doctest::Approx(1.0).epsilon(0.001));
    CHECK(straight.pose().y == doctest::Approx(0.0).scale(1.0).epsilon(0.001));

    Odometry turn(layout);
    turn.update({0, 0, 0});
    const int steps = 100;
    for (int i = 1; i <= steps; ++i) {
        const auto p = ticks_for(Twist{0.0, 0.0, kPi / 2.0}, static_cast<double>(i) / steps);
        turn.update(p);
    }
    CHECK(turn.pose().theta == doctest::Approx(kPi / 2.0).epsilon(0.001));
    CHECK(turn.pose().x == doctest::Approx(0.0).scale(1.0).epsilon(0.001));
}

TEST_CASE("Odometry survives encoder counter overflow") {
    Odometry odo;
    odo.update({2147483000, 0, 0});
    odo.update({-2147483000, 0, 0});  // +1296 ticks with wrap-around
    const Pose p = odo.pose();
    CHECK(std::abs(p.x) + std::abs(p.y) < 0.1);
    CHECK(std::abs(p.x) + std::abs(p.y) > 0.0);
}

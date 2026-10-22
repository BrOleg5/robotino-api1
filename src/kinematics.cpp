#include "robotino_api1/kinematics.hpp"

#include <cmath>

namespace robotino_api1 {

std::array<double, kNumMotors> inverse_kinematics(const DriveLayout& layout, const Twist& twist) {
    std::array<double, kNumMotors> motor{};
    const double k = layout.gear_ratio / layout.wheel_radius;
    for (std::size_t i = 0; i < kNumMotors; ++i) {
        const double a = layout.wheel_angles[i];
        motor[i] = k * (-std::sin(a) * twist.vx + std::cos(a) * twist.vy + layout.base_radius * twist.omega);
    }
    return motor;
}

Twist forward_kinematics(const DriveLayout& layout, const std::array<double, kNumMotors>& motor_velocity) {
    // Pseudo-inverse of the inverse kinematics matrix for wheels 120 deg apart.
    const double k = layout.wheel_radius / layout.gear_ratio;
    Twist t;
    for (std::size_t i = 0; i < kNumMotors; ++i) {
        const double a = layout.wheel_angles[i];
        const double v = k * motor_velocity[i];
        t.vx += -2.0 / 3.0 * std::sin(a) * v;
        t.vy += 2.0 / 3.0 * std::cos(a) * v;
        t.omega += v / (3.0 * layout.base_radius);
    }
    return t;
}

void Odometry::update(const std::array<std::int32_t, kNumMotors>& motor_position) {
    if (!has_reference_) {
        last_ = motor_position;
        has_reference_ = true;
        return;
    }
    std::array<double, kNumMotors> delta{};
    for (std::size_t i = 0; i < kNumMotors; ++i) {
        // Wrapping subtraction keeps working when the 32-bit counter overflows.
        const auto d = static_cast<std::int32_t>(static_cast<std::uint32_t>(motor_position[i]) -
                                                 static_cast<std::uint32_t>(last_[i]));
        delta[i] = ticks_to_rad(d);
    }
    last_ = motor_position;

    // Displacement in the robot frame, applied at the mid-step heading.
    const Twist d = forward_kinematics(layout_, delta);
    const double heading = pose_.theta + d.omega / 2.0;
    pose_.x += d.vx * std::cos(heading) - d.vy * std::sin(heading);
    pose_.y += d.vx * std::sin(heading) + d.vy * std::cos(heading);
    pose_.theta = std::remainder(pose_.theta + d.omega, 2.0 * kPi);
}

}  // namespace robotino_api1

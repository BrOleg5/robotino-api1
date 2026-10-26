// Kinematics of the three-wheeled omnidirectional base and wheel odometry.
//
// Robot frame: x forward, y left, rotation counterclockwise. Motor velocities are at the
// motor shaft, before the gearbox.
#pragma once

#include <array>
#include <cstdint>

#include "robotino_api1/export.hpp"
#include "robotino_api1/state.hpp"

namespace robotino_api1 {

struct DriveLayout {
    double base_radius = 0.132;  // m, wheel contact point to robot center (measured; OpenRobotinoAPI uses 0.125)
    double wheel_radius = 0.040;  // m
    double gear_ratio = 16.0;
    // Wheel i rolls along (-sin(a_i), cos(a_i)) when its motor turns forward (as in OpenRobotinoAPI).
    std::array<double, kNumMotors> wheel_angles = {kPi / 3.0, kPi, 5.0 * kPi / 3.0};
};

struct Twist {
    double vx = 0.0;     // m/s
    double vy = 0.0;     // m/s
    double omega = 0.0;  // rad/s
};

struct Pose {
    double x = 0.0;      // m
    double y = 0.0;      // m
    double theta = 0.0;  // rad
};

// Robot velocity to motor velocities, rad/s.
ROBOTINO_API1_EXPORT std::array<double, kNumMotors> inverse_kinematics(const DriveLayout& layout, const Twist& twist);
// Motor velocities, rad/s, to robot velocity (least squares for inconsistent wheel speeds).
ROBOTINO_API1_EXPORT Twist forward_kinematics(const DriveLayout& layout, const std::array<double, kNumMotors>& motor_velocity);

// Integrates the pose from motor encoder positions.
class ROBOTINO_API1_EXPORT Odometry {
public:
    explicit Odometry(DriveLayout layout = {}) : layout_(layout) {}

    // Feeds new encoder positions (ticks). The first call only sets the reference.
    void update(const std::array<std::int32_t, kNumMotors>& motor_position);
    // Forgets the encoder reference, e.g. after the motor positions were reset.
    void restart() { has_reference_ = false; }
    void set_pose(const Pose& pose) { pose_ = pose; }
    const Pose& pose() const { return pose_; }
    const DriveLayout& layout() const { return layout_; }

private:
    DriveLayout layout_;
    Pose pose_;
    std::array<std::int32_t, kNumMotors> last_{};
    bool has_reference_ = false;
};

}  // namespace robotino_api1

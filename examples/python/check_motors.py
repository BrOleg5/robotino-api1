"""Hardware check of the motors (step 2 of docs/hardware-testing.md).

Put the robot on a stand with the wheels in the air. Each motor in turn runs at +30 and
-30 rad/s; the script checks the measured velocity and position, the brake and
reset_motor_positions(), and prints PASS or FAIL for each check. Watch that the named wheel
is the one that turns.

Usage: python check_motors.py [host[:port]]
"""

import math
import sys
import time

import robotino_api1

VELOCITY = 30.0  # rad/s at the motor shaft
# One velocity unit on the wire is 2.83 rad/s, so 30 rad/s is sent as 31.1 rad/s.
VELOCITY_TOLERANCE = 0.2  # relative
STILL_VELOCITY = robotino_api1.MOTOR_VELOCITY_RESOLUTION * 1.5
POSITION_TOLERANCE = 0.3  # relative, position change against integrated velocity
STILL_TICKS = 20
TICKS_PER_RAD = 2000 / (2 * math.pi)
NUM_MOTORS = 3

failures = 0


def check(ok, what):
    global failures
    print(f"  {'PASS' if ok else 'FAIL'}  {what}")
    if not ok:
        failures += 1


def run(robot, motor, rad_s, skip, measure):
    """Keeps the motor velocity set (the watchdog stops the motors otherwise) for skip + measure
    seconds and returns (mean velocity, max |velocity| of the other motors, position change,
    position change integrated from the velocity) over the last `measure` seconds."""
    start = time.monotonic()
    velocities = []
    max_other = 0.0
    position_change = 0.0
    expected_change = 0.0
    previous = None
    while time.monotonic() - start < skip + measure:
        robot.set_motor_velocity(motor, rad_s)
        if not robot.wait_for_update(0.2):
            continue
        now = time.monotonic()
        s = robot.sensors()
        if now - start < skip:
            continue
        if previous is not None:
            prev_time, prev = previous
            dt = now - prev_time
            expected_change += 0.5 * (s.motor_velocity[motor] + prev.motor_velocity[motor]) * dt * TICKS_PER_RAD
            position_change += s.motor_position[motor] - prev.motor_position[motor]
        velocities.append(s.motor_velocity[motor])
        others = [abs(s.motor_velocity[m]) for m in range(NUM_MOTORS) if m != motor]
        max_other = max([max_other] + others)
        previous = (now, s)
    mean = sum(velocities) / len(velocities) if velocities else 0.0
    return mean, max_other, position_change, expected_change


def check_direction(robot, motor, rad_s):
    print(f"motor {motor} at {rad_s:+.0f} rad/s")
    mean, max_other, change, expected = run(robot, motor, rad_s, 1.0, 1.5)
    robot.stop()
    print(f"  measured {mean:+.2f} rad/s, other motors up to {max_other:.2f} rad/s")
    print(f"  position change {change:+.0f} ticks, expected {expected:+.0f}")
    check(mean * rad_s > 0, "velocity has the commanded sign")
    check(abs(mean - rad_s) < VELOCITY_TOLERANCE * abs(rad_s), "velocity is close to the command")
    check(max_other < STILL_VELOCITY, "other motors stand still")
    check(change * mean > 0, "position changes in the direction of the velocity")
    check(abs(change - expected) < POSITION_TOLERANCE * abs(expected), "position change matches the velocity")


def check_brake(robot, motor):
    print(f"motor {motor} at {VELOCITY:+.0f} rad/s with the brake engaged")
    run(robot, motor, VELOCITY, 0.0, 1.0)
    robot.set_brake(motor, True)
    mean, _, change, _ = run(robot, motor, VELOCITY, 1.0, 1.0)
    robot.set_brake(motor, False)
    robot.stop()
    print(f"  measured {mean:+.2f} rad/s, position change {change:+.0f} ticks")
    check(abs(mean) < STILL_VELOCITY, "brake stops the wheel")
    check(abs(change) < STILL_TICKS, "position stays put")


def check_reset(robot):
    print("reset_motor_positions()")
    robot.stop()
    time.sleep(1.0)
    print("  positions before", *robot.sensors().motor_position)
    robot.reset_motor_positions()
    # The reset flag is kept for reset_cycles cycles (3 by default); give the robot a few more.
    for _ in range(13):
        robot.wait_for_update(0.2)
    positions = robot.sensors().motor_position
    print("  positions after", *positions)
    check(all(abs(p) < STILL_TICKS for p in positions), "positions are zero")


def main():
    host, _, port = (sys.argv[1] if len(sys.argv) > 1 else "172.26.1.1").partition(":")
    # command_timeout stops the robot if this script hangs or dies.
    with robotino_api1.Client(host=host, port=int(port or 80), command_timeout=0.5) as robot:
        print(robot.info)
        for motor in range(NUM_MOTORS):
            check_direction(robot, motor, VELOCITY)
            check_direction(robot, motor, -VELOCITY)
            check_brake(robot, motor)
            check_reset(robot)
        robot.stop()
    print(f"{'OK' if failures == 0 else 'FAILED'}: {failures} failed checks")
    sys.exit(0 if failures == 0 else 1)


if __name__ == "__main__":
    main()

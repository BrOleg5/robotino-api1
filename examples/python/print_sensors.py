"""Prints the robot's readings for ten seconds.

Usage: python print_sensors.py [host[:port]]
"""

import sys
import time

import robotino_api1


def print_connection_events(robot):
    """Prints errors and connection changes reported by the background thread, such as reconnects."""
    robot.on_error(lambda code, message: print(f"error: {code.name}: {message}", file=sys.stderr))
    robot.on_connection_changed(lambda state: print(f"connection: {state.name}", file=sys.stderr))


def main():
    host, _, port = (sys.argv[1] if len(sys.argv) > 1 else "172.26.1.1").partition(":")
    robot = robotino_api1.Client(host=host, port=int(port or 80))
    print_connection_events(robot)
    with robot:
        print(robot.info)
        for _ in range(20):
            robot.wait_for_update(0.5)
            s = robot.sensors()
            distances = " ".join(f"{v:.2f}" for v in s.distance_voltage)
            print(f"battery {s.battery_voltage:.2f} V | bumper {s.bumper} | "
                  f"position {s.motor_position} | distance V: {distances}")
            time.sleep(0.5)


if __name__ == "__main__":
    main()

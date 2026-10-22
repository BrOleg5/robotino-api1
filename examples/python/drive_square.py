"""Drives a 0.5 m square without turning and prints the odometry.

Usage: python drive_square.py [host[:port]]
"""

import sys
import time

import robotino_api1


def main():
    host, _, port = (sys.argv[1] if len(sys.argv) > 1 else "172.26.1.0").partition(":")
    # command_timeout stops the robot if this script hangs or dies.
    with robotino_api1.Client(host=host, port=int(port or 80), command_timeout=0.5) as robot:
        speed = 0.1  # m/s
        for vx, vy in [(speed, 0), (0, speed), (-speed, 0), (0, -speed)]:
            for _ in range(50):
                robot.set_velocity(vx, vy, 0.0)
                if robot.sensors().bumper:
                    sys.exit("bumper hit, stopping")
                time.sleep(0.1)
            print(robot.pose)
        robot.stop()


if __name__ == "__main__":
    main()

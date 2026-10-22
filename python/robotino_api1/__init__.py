"""Open-source client library for Festo Robotino robots with OpenRobotinoAPI 1 (before Robotino 3).

Example:
    import robotino_api1

    with robotino_api1.Client(host="172.26.1.1") as robot:
        robot.set_velocity(0.1, 0.0, 0.0)
        robot.wait_for_update(0.5)
        print(robot.sensors().battery_voltage)
"""

from ._core import (
    CameraCompression,
    CameraResolution,
    CameraSettings,
    Client,
    ConnectionState,
    DriveLayout,
    ErrorCode,
    Image,
    ImageType,
    Pid,
    Pose,
    RobotinoError,
    SensorState,
    Simulator,
    Twist,
    forward_kinematics,
    inverse_kinematics,
    MOTOR_VELOCITY_RESOLUTION,
)

__all__ = [
    "CameraCompression",
    "CameraResolution",
    "CameraSettings",
    "Client",
    "ConnectionState",
    "DriveLayout",
    "ErrorCode",
    "Image",
    "ImageType",
    "Pid",
    "Pose",
    "RobotinoError",
    "SensorState",
    "Simulator",
    "Twist",
    "forward_kinematics",
    "inverse_kinematics",
    "MOTOR_VELOCITY_RESOLUTION",
]
__version__ = "0.1.0"

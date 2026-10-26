import math
import threading
import time

import pytest

import robotino_api1


def eventually(condition, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.005)
    return condition()


@pytest.fixture
def sim():
    with robotino_api1.Simulator(camera_width=32, camera_height=24, camera_period=0.02) as s:
        yield s


def make_client(sim, **kwargs):
    return robotino_api1.Client(host="127.0.0.1", port=sim.port, cycle=0.01, timeout=0.3, image_port=0, **kwargs)


def test_readings(sim):
    sim.set_bumper(True)
    sim.set_distance_voltage(2, 1.25)
    sim.set_battery(23.5, 2.0)
    with make_client(sim) as robot:
        assert robot.connection_state == robotino_api1.ConnectionState.CONNECTED
        assert "robotino-api1-sim" in robot.info
        assert robot.wait_for_update(1.0)
        s = robot.sensors()
        assert s.bumper
        assert s.distance_voltage[2] == pytest.approx(1.25, rel=0.01)
        assert s.battery_voltage == pytest.approx(23.5, rel=0.01)
        assert len(s.motor_position) == 3
    assert robot.connection_state == robotino_api1.ConnectionState.DISCONNECTED


def test_motion_and_callbacks(sim):
    robot = make_client(sim)
    updates = []
    robot.on_update(lambda s: updates.append(s.sequence))
    robot.connect()
    robot.set_velocity(0.1, 0.0, 0.0)
    assert eventually(lambda: robot.pose.x > 0.03)
    assert abs(robot.pose.y) < 0.01
    robot.stop()
    assert eventually(lambda: sim.command.motor_velocity == [0.0, 0.0, 0.0])
    assert len(updates) > 5
    assert updates == sorted(updates)
    # Destruction with a live callback must not deadlock.
    del robot


def test_kinematics_round_trip():
    layout = robotino_api1.DriveLayout()
    m = robotino_api1.inverse_kinematics(layout, robotino_api1.Twist(0.2, -0.1, 0.5))
    t = robotino_api1.forward_kinematics(layout, m)
    assert (t.vx, t.vy, t.omega) == pytest.approx((0.2, -0.1, 0.5))


def test_errors(sim):
    # Windows retransmits the SYN after a RST and reports the refusal only after ~2 s.
    with pytest.raises(robotino_api1.RobotinoError) as info:
        robotino_api1.Client(host="127.0.0.1", port=1, timeout=5.0).connect()
    assert info.value.code == robotino_api1.ErrorCode.CONNECTION_REFUSED

    with make_client(sim):
        with pytest.raises(robotino_api1.RobotinoError) as info:
            make_client(sim).connect()
        assert info.value.code == robotino_api1.ErrorCode.CONNECTION_BUSY
        with make_client(sim, allow_passive=True) as watcher:
            assert watcher.wait_for_update(1.0)
            assert watcher.sensors().passive_mode

    robot = make_client(sim)
    with pytest.raises(IndexError):
        robot.set_motor_velocity(3, 1.0)
    with pytest.raises(ValueError):
        robotino_api1.Client(cycle=0)


def test_reconnect(sim):
    robot = make_client(sim)
    errors = []
    robot.on_error(lambda code, message: errors.append(code))
    robot.connect()
    sim.kick()
    assert eventually(lambda: errors)
    assert eventually(lambda: sim.connections == 2 and robot.connection_state == robotino_api1.ConnectionState.CONNECTED)
    robot.disconnect()


def test_camera(sim):
    robot = make_client(sim)
    got = threading.Event()
    images = []

    def on_image(image):
        images.append(image)
        got.set()

    robot.on_image(on_image)
    robot.set_camera_streaming(True)
    robot.connect()
    assert got.wait(2.0)
    image = images[0]
    assert image.type == robotino_api1.ImageType.RAW
    assert (image.width, image.height, image.channels) == (32, 24, 3)
    assert len(image.data) == 32 * 24 * 3
    assert robot.latest_image() is not None
    robot.disconnect()


def test_motor_velocity_resolution():
    # One unit of the protocol is 27 rpm at the motor shaft.
    assert robotino_api1.MOTOR_VELOCITY_RESOLUTION * 60 / (2 * math.pi) == pytest.approx(27.0)

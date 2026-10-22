# RobotinoAPI1

An open-source client library for **Festo Robotino** robots of the generation before Robotino 3,
the ones controlled with OpenRobotinoAPI 1.x (Robotino 3 and 4 use the different API2). It replaces
[OpenRobotinoAPI](https://wiki.openrobotino.org/OpenRobotinoAPI.html) 1.x. That library was open
source too, but its SVN repository (svn.openrobotino.org) is no longer available, and its build is
outdated for modern systems: the published binaries are only for 32-bit x86 (Linux with Qt 4, or
Windows with MSVC).

- C++17, no dependencies: its own socket layer over BSD sockets and Winsock
- Linux (x86_64, arm64) and Windows (MSVC, MinGW-w64)
- Motors, omnidirectional drive and wheel odometry, distance sensors, bumper, digital and analog
  inputs, digital outputs, relays, power output, encoder input, battery, camera
- Reconnects automatically and can stop the motors when commands stop arriving
- C API for ctypes and other languages, Python module
- Robot emulator for development and tests without a robot

> **Status.** The protocol was checked byte for byte against traffic between OpenRobotinoAPI
> 1.8.54 and a real robot. The following still has to be verified on a robot: the motion
> commands, the camera stream (no capture with images is available) and the meaning of the last
> 20 bytes of the IOControl message (see [docs/protocol.md](docs/protocol.md)). The checklist is in
> [docs/hardware-testing.md](docs/hardware-testing.md).

## Usage

### C++

```cpp
#include <robotino_api1/client.hpp>

robotino_api1::ClientOptions options;
options.host = "172.26.1.1";                         // the robot's factory address
options.command_timeout = std::chrono::milliseconds(500);  // stop if the program hangs
robotino_api1::Client robot(options);
if (robotino_api1::Error e = robot.connect()) {
    std::cerr << e.message << "\n";
    return 1;
}
robot.set_velocity({0.1, 0.0, 0.0});                 // m/s, m/s, rad/s
robot.wait_for_update(std::chrono::milliseconds(100));
robotino_api1::SensorState s = robot.sensors();
std::cout << s.battery_voltage << " V, bumper " << s.bumper << "\n";
```

Commands stay in effect until changed; a background thread sends them every cycle (20 ms by
default) and receives the readings. All methods are thread-safe. See
[include/robotino_api1/client.hpp](include/robotino_api1/client.hpp) and [examples/](examples).

On Windows, connecting to a host where nothing listens on the port fails with `Timeout`
rather than `ConnectionRefused` unless `timeout` is longer than about 2 s: Windows retries the
connection twice before reporting the refusal.

With CMake, after installing:

```cmake
find_package(robotino_api1 REQUIRED)
target_link_libraries(my_app PRIVATE robotino_api1::robotino_api1)
```

or as a subdirectory: `add_subdirectory(robotino_api1)`.

### Python

```python
import robotino_api1

with robotino_api1.Client(host="172.26.1.1", command_timeout=0.5) as robot:
    robot.set_velocity(0.1, 0.0, 0.0)
    robot.wait_for_update(0.1)
    print(robot.sensors().distance_voltage, robot.pose)
```

`robotino_api1.Simulator` runs the emulator in-process, see [python/tests](python/tests).

### C

[include/robotino_api1/c_api.h](include/robotino_api1/c_api.h), library `robotino_api1_c`.

### Units

| Quantity | Unit |
|---|---|
| Robot velocity | m/s, rad/s (x forward, y left, counterclockwise) |
| Motor velocity | rad/s at the motor shaft, before the 16:1 gearbox |
| Motor position | encoder ticks, 2000 per motor revolution |
| Distance sensors | V (higher is closer; sensor `i` looks at `i * 40` deg) |

The robot transfers motor velocities in steps of 27 rpm (2.83 rad/s) at the motor shaft, which
is 0.18 rad/s at the wheel or about 7 mm/s of wheel speed. Setpoints are rounded to that.

## Emulator

`robotino-api1-sim [--port 8000]` speaks the robot side of the protocol: motors follow the setpoints,
other readings are constant, and with camera streaming on it sends a test image. Connect to it
with `print_sensors 127.0.0.1:8000`.

## Building

Everything builds in the development container, nothing has to be installed on the host except
Docker with the Compose plugin:

```bash
docker compose build                           # the image, once
docker compose run --rm dev scripts/build.sh   # gcc build + tests
docker compose run --rm dev scripts/build.sh all   # gcc, clang, MinGW (tests under Wine), Python
docker compose run --rm dev bash               # a shell
```

VS Code: "Dev Containers: Reopen in Container" uses the same image
([.devcontainer/devcontainer.json](.devcontainer/devcontainer.json)). The container uses the host
network, so it reaches the robot by IP.

Without Docker, with CMake 3.16+ and a C++17 compiler:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
cmake --install build --prefix /usr/local
```

CMake options:

| Option | Default | |
|---|---|---|
| `BUILD_SHARED_LIBS` | `OFF` | shared `robotino_api1` library |
| `ROBOTINO_API1_BUILD_C_API` | `ON` | `robotino_api1_c` shared library |
| `ROBOTINO_API1_BUILD_SIM` | top level | `robotino-api1-sim` |
| `ROBOTINO_API1_BUILD_EXAMPLES` | top level | examples |
| `ROBOTINO_API1_BUILD_TESTS` | top level | tests (downloads doctest) |
| `ROBOTINO_API1_BUILD_PYTHON` | `OFF` | Python module (needs nanobind), use `pip install .` instead |

Python module: `pip install .` (needs a C++17 compiler and CMake).

## Protocol

See [docs/protocol.md](docs/protocol.md). It is based on the OpenRobotinoAPI sources published by
REC GmbH ([openrobotino code, revision 23](https://svn.code.sf.net/p/openrobotino/code/trunk/?p=23))
and the headers of OpenRobotinoAPI 1.8.54. [tools/pcap/pcap_extract.py](tools/pcap/pcap_extract.py)
extracts messages from Wireshark captures; the test fixtures in [tests/data](tests/data) come from
a capture of OpenRobotinoAPI 1.8.54 talking to a robot.

## License

Apache License 2.0, see [LICENSE](LICENSE).

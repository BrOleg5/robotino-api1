// Python bindings (nanobind).
#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/string.h>

#include <stdexcept>

#include "robotino_api1/client.hpp"
#include "simulator.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using namespace robotino_api1;

namespace {

PyObject* g_robotino_error = nullptr;

struct RobotinoException : std::exception {
    explicit RobotinoException(Error e) : error(std::move(e)) {}
    const char* what() const noexcept override { return error.message.c_str(); }
    Error error;
};

void throw_if(const Error& e) {
    if (e) {
        throw RobotinoException(e);
    }
}

void check_index(std::size_t index, std::size_t count, const char* what) {
    if (index >= count) {
        throw nb::index_error((std::string(what) + " index out of range").c_str());
    }
}

Milliseconds to_ms(double seconds) {
    if (seconds < 0) {
        throw nb::value_error("negative duration");
    }
    return Milliseconds(static_cast<long long>(seconds * 1000.0 + 0.5));
}

// Background threads call into Python callbacks with the GIL, so every call that waits for
// them (disconnect, destruction) must release it first.
class PyClient : public Client {
public:
    using Client::Client;
    ~PyClient() {
        nb::gil_scoped_release release;
        disconnect();
        set_camera_streaming(false);
    }
};

}  // namespace

NB_MODULE(_core, m) {
    m.doc() = "Open-source client library for Festo Robotino robots with OpenRobotinoAPI 1 (before Robotino 3)";
    m.attr("MOTOR_VELOCITY_RESOLUTION") = kMotorVelocityResolution;

    g_robotino_error = PyErr_NewExceptionWithDoc("robotino_api1.RobotinoError",
                                                 "Error raised by robotino_api1; .code is an ErrorCode.", PyExc_RuntimeError,
                                                 nullptr);
    m.attr("RobotinoError") = nb::handle(g_robotino_error);

    nb::enum_<ErrorCode>(m, "ErrorCode")
        .value("NONE", ErrorCode::None)
        .value("HOST_NOT_FOUND", ErrorCode::HostNotFound)
        .value("CONNECTION_REFUSED", ErrorCode::ConnectionRefused)
        .value("TIMEOUT", ErrorCode::Timeout)
        .value("CONNECTION_CLOSED", ErrorCode::ConnectionClosed)
        .value("CONNECTION_BUSY", ErrorCode::ConnectionBusy)
        .value("PROTOCOL_ERROR", ErrorCode::ProtocolError)
        .value("SOCKET_ERROR", ErrorCode::SocketError)
        .value("NOT_CONNECTED", ErrorCode::NotConnected)
        .value("INVALID_ARGUMENT", ErrorCode::InvalidArgument);

    nb::register_exception_translator([](const std::exception_ptr& p, void*) {
        try {
            std::rethrow_exception(p);
        } catch (const RobotinoException& e) {
            nb::object error = nb::handle(g_robotino_error)(e.error.message);
            error.attr("code") = nb::cast(e.error.code);
            PyErr_SetObject(g_robotino_error, error.ptr());
        }
    });

    nb::enum_<ConnectionState>(m, "ConnectionState")
        .value("DISCONNECTED", ConnectionState::Disconnected)
        .value("CONNECTING", ConnectionState::Connecting)
        .value("CONNECTED", ConnectionState::Connected);

    nb::enum_<protocol::ImageType>(m, "ImageType")
        .value("JPEG", protocol::ImageType::JPEG)
        .value("JPEG2000", protocol::ImageType::JPEG2000)
        .value("RAW", protocol::ImageType::Raw)
        .value("BMP", protocol::ImageType::BMP)
        .value("PNG", protocol::ImageType::PNG)
        .value("TIFF", protocol::ImageType::TIFF)
        .value("UNKNOWN", protocol::ImageType::Unknown);

    nb::enum_<protocol::CameraResolution>(m, "CameraResolution")
        .value("QVGA", protocol::CameraResolution::QVGA)
        .value("VGA", protocol::CameraResolution::VGA);

    nb::enum_<protocol::CameraCompression>(m, "CameraCompression")
        .value("HIGH", protocol::CameraCompression::High)
        .value("LOW", protocol::CameraCompression::Low)
        .value("NONE", protocol::CameraCompression::None);

    nb::class_<Twist>(m, "Twist", "Robot velocity: m/s, m/s, rad/s.")
        .def(nb::init<>())
        .def("__init__", [](Twist* t, double vx, double vy, double omega) { new (t) Twist{vx, vy, omega}; },
             "vx"_a = 0.0, "vy"_a = 0.0, "omega"_a = 0.0)
        .def_rw("vx", &Twist::vx)
        .def_rw("vy", &Twist::vy)
        .def_rw("omega", &Twist::omega)
        .def("__repr__", [](const Twist& t) {
            return "Twist(vx=" + std::to_string(t.vx) + ", vy=" + std::to_string(t.vy) +
                   ", omega=" + std::to_string(t.omega) + ")";
        });

    nb::class_<Pose>(m, "Pose", "Odometry pose: m, m, rad.")
        .def("__init__", [](Pose* p, double x, double y, double theta) { new (p) Pose{x, y, theta}; },
             "x"_a = 0.0, "y"_a = 0.0, "theta"_a = 0.0)
        .def_rw("x", &Pose::x)
        .def_rw("y", &Pose::y)
        .def_rw("theta", &Pose::theta)
        .def("__repr__", [](const Pose& p) {
            return "Pose(x=" + std::to_string(p.x) + ", y=" + std::to_string(p.y) +
                   ", theta=" + std::to_string(p.theta) + ")";
        });

    nb::class_<Pid>(m, "Pid", "Motor controller gains 0-255; 255 selects the IO board default.")
        .def("__init__", [](Pid* p, std::uint8_t kp, std::uint8_t ki, std::uint8_t kd) { new (p) Pid{kp, ki, kd}; },
             "kp"_a = 255, "ki"_a = 255, "kd"_a = 255)
        .def_rw("kp", &Pid::kp)
        .def_rw("ki", &Pid::ki)
        .def_rw("kd", &Pid::kd);

    nb::class_<DriveLayout>(m, "DriveLayout")
        .def(nb::init<>())
        .def_rw("base_radius", &DriveLayout::base_radius)
        .def_rw("wheel_radius", &DriveLayout::wheel_radius)
        .def_rw("gear_ratio", &DriveLayout::gear_ratio)
        .def_rw("wheel_angles", &DriveLayout::wheel_angles);

    m.def("inverse_kinematics", &inverse_kinematics, "layout"_a, "twist"_a,
          "Robot velocity to motor velocities, rad/s at the motor shaft.");
    m.def("forward_kinematics", &forward_kinematics, "layout"_a, "motor_velocity"_a,
          "Motor velocities, rad/s at the motor shaft, to robot velocity.");

    nb::class_<Command>(m, "Command")
        .def(nb::init<>())
        .def_rw("motor_velocity", &Command::motor_velocity)
        .def_rw("pid", &Command::pid)
        .def_rw("brake", &Command::brake)
        .def_rw("reset_motor_position", &Command::reset_motor_position)
        .def_rw("digital_output", &Command::digital_output)
        .def_rw("relay", &Command::relay)
        .def_rw("power_output", &Command::power_output)
        .def_rw("reset_encoder_input", &Command::reset_encoder_input)
        .def_rw("shutdown", &Command::shutdown);

    nb::class_<SensorState>(m, "SensorState")
        .def(nb::init<>())
        .def_ro("sequence", &SensorState::sequence)
        .def_ro("motor_velocity", &SensorState::motor_velocity, "rad/s at the motor shaft")
        .def_ro("motor_position", &SensorState::motor_position, "encoder ticks (2000 per motor revolution)")
        .def_ro("motor_current", &SensorState::motor_current, "A")
        .def_ro("distance_voltage", &SensorState::distance_voltage, "V, higher is closer")
        .def_ro("analog_input", &SensorState::analog_input, "V")
        .def_ro("digital_input", &SensorState::digital_input)
        .def_ro("bumper", &SensorState::bumper)
        .def_ro("battery_voltage", &SensorState::battery_voltage, "V")
        .def_ro("battery_current", &SensorState::battery_current, "A")
        .def_ro("power_output_current", &SensorState::power_output_current, "A")
        .def_ro("encoder_input_position", &SensorState::encoder_input_position)
        .def_ro("encoder_input_velocity", &SensorState::encoder_input_velocity)
        .def_ro("serial_frequency", &SensorState::serial_frequency)
        .def_ro("board_time", &SensorState::board_time)
        .def_ro("firmware_version", &SensorState::firmware_version)
        .def_ro("passive_mode", &SensorState::passive_mode);

    nb::class_<CameraSettings>(m, "CameraSettings")
        .def(nb::init<>())
        .def_rw("resolution", &CameraSettings::resolution)
        .def_rw("compression", &CameraSettings::compression)
        .def_rw("brightness", &CameraSettings::brightness)
        .def_rw("contrast", &CameraSettings::contrast)
        .def_rw("auto_white_balance", &CameraSettings::auto_white_balance);

    nb::class_<Image>(m, "Image")
        .def_ro("type", &Image::type)
        .def_ro("width", &Image::width)
        .def_ro("height", &Image::height)
        .def_ro("channels", &Image::channels)
        .def_ro("bits_per_channel", &Image::bits_per_channel)
        .def_prop_ro("data", [](const Image& img) {
            return nb::bytes(reinterpret_cast<const char*>(img.data.data()), img.data.size());
        }, "JPEG file or raw pixel rows");

    nb::class_<PyClient>(m, "Client")
        .def("__init__",
             [](PyClient* self, const std::string& host, std::uint16_t port, double cycle, double timeout,
                bool auto_reconnect, double command_timeout, std::uint16_t image_port, const DriveLayout& drive_layout,
                int reset_cycles) {
                 ClientOptions o;
                 o.host = host;
                 o.port = port;
                 o.cycle = to_ms(cycle);
                 o.timeout = to_ms(timeout);
                 o.auto_reconnect = auto_reconnect;
                 o.command_timeout = to_ms(command_timeout);
                 o.image_port = image_port;
                 o.drive_layout = drive_layout;
                 o.reset_cycles = reset_cycles;
                 if (o.cycle.count() == 0 || o.timeout.count() == 0) {
                     throw nb::value_error("cycle and timeout must be positive");
                 }
                 new (self) PyClient(o);
             },
             "host"_a = "172.26.1.0", "port"_a = 80, "cycle"_a = 0.02, "timeout"_a = 1.0, "auto_reconnect"_a = true,
             "command_timeout"_a = 0.0, "image_port"_a = 8080, "drive_layout"_a = DriveLayout(), "reset_cycles"_a = 3,
             "Times are in seconds. command_timeout > 0 stops the motors when no velocity was set for that long.")
        .def("connect", [](PyClient& c) {
            Error e;
            {
                nb::gil_scoped_release release;
                e = c.connect();
            }
            throw_if(e);
        }, "Connects and waits for the first reading; raises RobotinoError on failure.")
        .def("disconnect", &PyClient::disconnect, nb::call_guard<nb::gil_scoped_release>())
        .def("__enter__", [](PyClient& c) -> PyClient& {
            Error e;
            {
                nb::gil_scoped_release release;
                e = c.connect();
            }
            throw_if(e);
            return c;
        }, nb::rv_policy::reference)
        .def("__exit__", [](PyClient& c, nb::args) {
            nb::gil_scoped_release release;
            c.disconnect();
        })
        .def_prop_ro("connection_state", &PyClient::connection_state)
        .def("set_motor_velocities", &PyClient::set_motor_velocities, "rad_s"_a, "rad/s at the motor shaft")
        .def("set_motor_velocity", [](PyClient& c, std::size_t motor, double rad_s) {
            check_index(motor, kNumMotors, "motor");
            c.set_motor_velocity(motor, rad_s);
        }, "motor"_a, "rad_s"_a)
        .def("set_velocity", [](PyClient& c, double vx, double vy, double omega) { c.set_velocity({vx, vy, omega}); },
             "vx"_a, "vy"_a = 0.0, "omega"_a = 0.0, "Robot velocity in m/s, m/s, rad/s.")
        .def("stop", &PyClient::stop)
        .def("set_pid", [](PyClient& c, std::size_t motor, std::uint8_t kp, std::uint8_t ki, std::uint8_t kd) {
            check_index(motor, kNumMotors, "motor");
            c.set_pid(motor, {kp, ki, kd});
        }, "motor"_a, "kp"_a, "ki"_a, "kd"_a)
        .def("set_brake", [](PyClient& c, std::size_t motor, bool engaged) {
            check_index(motor, kNumMotors, "motor");
            c.set_brake(motor, engaged);
        }, "motor"_a, "engaged"_a)
        .def("reset_motor_positions", &PyClient::reset_motor_positions)
        .def("set_digital_output", [](PyClient& c, std::size_t output, bool value) {
            check_index(output, kNumDigitalOutputs, "output");
            c.set_digital_output(output, value);
        }, "output"_a, "value"_a)
        .def("set_relay", [](PyClient& c, std::size_t relay, bool value) {
            check_index(relay, kNumRelays, "relay");
            c.set_relay(relay, value);
        }, "relay"_a, "value"_a)
        .def("set_power_output", &PyClient::set_power_output, "duty"_a, "-1..1")
        .def("reset_encoder_input", &PyClient::reset_encoder_input)
        .def("request_shutdown", &PyClient::request_shutdown, "Switches the robot off.")
        .def_prop_ro("command", &PyClient::command)
        .def("sensors", &PyClient::sensors, "The latest readings.")
        .def("wait_for_update", [](PyClient& c, double timeout) {
            const Milliseconds t = to_ms(timeout);
            nb::gil_scoped_release release;
            return c.wait_for_update(t);
        }, "timeout"_a = 1.0, "Waits for new readings; returns False on timeout.")
        .def_prop_ro("info", [](const PyClient& c) { return c.info().text; }, "Version text sent by the robot.")
        .def_prop_rw("pose", &PyClient::pose, &PyClient::set_pose, "Wheel odometry since connect.")
        .def_prop_ro("measured_velocity", &PyClient::measured_velocity)
        .def("set_camera_settings", &PyClient::set_camera_settings, "settings"_a)
        .def("set_camera_streaming", [](PyClient& c, bool enable) {
            Error e;
            {
                nb::gil_scoped_release release;
                e = c.set_camera_streaming(enable);
            }
            throw_if(e);
        }, "enable"_a)
        .def_prop_ro("camera_streaming", &PyClient::camera_streaming)
        .def("latest_image", [](const PyClient& c) -> nb::object {
            Image image;
            if (!c.latest_image(image)) {
                return nb::none();
            }
            return nb::cast(std::move(image));
        })
        .def("on_update", &PyClient::on_update, "callback"_a.none(), "Called from a background thread.")
        .def("on_connection_changed", &PyClient::on_connection_changed, "callback"_a.none())
        .def("on_error", [](PyClient& c, std::function<void(ErrorCode, const std::string&)> cb) {
            if (!cb) {
                c.on_error(nullptr);
                return;
            }
            c.on_error([cb](const Error& e) { cb(e.code, e.message); });
        }, "callback"_a.none(), "callback(code, message)")
        .def("on_image", &PyClient::on_image, "callback"_a.none());

    nb::class_<sim::Simulator>(m, "Simulator", "Robot emulator for tests without a robot.")
        .def("__init__",
             [](sim::Simulator* self, std::uint16_t port, const std::string& address, double motor_time_constant,
                std::uint32_t camera_width, std::uint32_t camera_height, double camera_period) {
                 sim::SimulatorOptions o;
                 o.port = port;
                 o.address = address;
                 o.motor_time_constant = motor_time_constant;
                 o.camera_width = camera_width;
                 o.camera_height = camera_height;
                 o.camera_period = to_ms(camera_period);
                 new (self) sim::Simulator(o);
             },
             "port"_a = 0, "address"_a = "127.0.0.1", "motor_time_constant"_a = 0.05, "camera_width"_a = 320,
             "camera_height"_a = 240, "camera_period"_a = 0.1)
        .def("start", [](sim::Simulator& s) { throw_if(s.start()); })
        .def("stop", &sim::Simulator::stop, nb::call_guard<nb::gil_scoped_release>())
        .def_prop_ro("port", &sim::Simulator::port)
        .def_prop_ro("command", &sim::Simulator::command, "The last command received.")
        .def("state", &sim::Simulator::state)
        .def("set_bumper", [](sim::Simulator& s, bool v) { s.modify([v](SensorState& st) { st.bumper = v; }); })
        .def("set_distance_voltage", [](sim::Simulator& s, std::size_t i, double v) {
            check_index(i, kNumDistanceSensors, "sensor");
            s.modify([i, v](SensorState& st) { st.distance_voltage[i] = v; });
        })
        .def("set_digital_input", [](sim::Simulator& s, std::size_t i, bool v) {
            check_index(i, kNumDigitalInputs, "input");
            s.modify([i, v](SensorState& st) { st.digital_input[i] = v; });
        })
        .def("set_analog_input", [](sim::Simulator& s, std::size_t i, double v) {
            check_index(i, kNumAnalogInputs, "input");
            s.modify([i, v](SensorState& st) { st.analog_input[i] = v; });
        })
        .def("set_battery", [](sim::Simulator& s, double voltage, double current) {
            s.modify([voltage, current](SensorState& st) {
                st.battery_voltage = voltage;
                st.battery_current = current;
            });
        }, "voltage"_a, "current"_a)
        .def("set_paused", &sim::Simulator::set_paused, "paused"_a, "Stop answering, as a hung robot would.")
        .def("kick", &sim::Simulator::kick, "Drop the current client connection.")
        .def_prop_ro("io_controls", &sim::Simulator::io_controls)
        .def_prop_ro("connections", &sim::Simulator::connections)
        .def_prop_ro("images_sent", &sim::Simulator::images_sent)
        .def("__enter__", [](sim::Simulator& s) -> sim::Simulator& {
            throw_if(s.start());
            return s;
        }, nb::rv_policy::reference)
        .def("__exit__", [](sim::Simulator& s, nb::args) {
            nb::gil_scoped_release release;
            s.stop();
        });
}

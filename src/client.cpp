#include "robotino_api1/client.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "net/socket.hpp"

namespace robotino_api1 {
namespace {

using Clock = std::chrono::steady_clock;

// Upper bound for one blocking socket wait, so that disconnect() is noticed quickly.
constexpr Milliseconds kPollSlice{50};
constexpr Milliseconds kMinReconnectDelay{100};
constexpr Milliseconds kMaxReconnectDelay{2000};

// Callbacks are shared, not copied, under the mutex: copying a std::function copies the
// target, which for a Python callable takes the GIL and deadlocks against a Python thread
// that holds the GIL and waits for the mutex.
template <class... Args>
using Callback = std::shared_ptr<const std::function<void(Args...)>>;

template <class... Args>
Callback<Args...> make_callback(std::function<void(Args...)> f) {
    return f ? std::make_shared<const std::function<void(Args...)>>(std::move(f)) : nullptr;
}

// Number of further UDP ports tried when the image port is taken.
constexpr std::uint16_t kImagePortAttempts = 10;
constexpr std::size_t kMaxDatagramSize = 65536;

}  // namespace

const char* to_string(ErrorCode code) {
    switch (code) {
        case ErrorCode::None: return "no error";
        case ErrorCode::HostNotFound: return "host not found";
        case ErrorCode::ConnectionRefused: return "connection refused";
        case ErrorCode::Timeout: return "timeout";
        case ErrorCode::ConnectionClosed: return "connection closed";
        case ErrorCode::ConnectionBusy: return "robot is controlled by another client";
        case ErrorCode::ProtocolError: return "protocol error";
        case ErrorCode::SocketError: return "socket error";
        case ErrorCode::NotConnected: return "not connected";
        case ErrorCode::InvalidArgument: return "invalid argument";
    }
    return "unknown error";
}

const char* to_string(ConnectionState state) {
    switch (state) {
        case ConnectionState::Disconnected: return "disconnected";
        case ConnectionState::Connecting: return "connecting";
        case ConnectionState::Connected: return "connected";
    }
    return "unknown";
}

struct Client::Impl {
    explicit Impl(ClientOptions o) : options(std::move(o)), odometry(options.drive_layout) {}

    const ClientOptions options;

    // Everything below is guarded by mutex unless noted otherwise.
    mutable std::mutex mutex;
    std::condition_variable update_cv;
    std::condition_variable stop_cv;

    Command command;
    int reset_motor_cycles = 0;
    int reset_encoder_cycles = 0;
    Clock::time_point last_velocity_command = Clock::now();
    CameraSettings camera_settings;
    bool camera_settings_dirty = true;

    ConnectionState state = ConnectionState::Disconnected;
    SensorState sensors;
    mutable std::uint64_t seen_sequence = 0;
    RobotInfo info;
    Odometry odometry;
    int odometry_restart_cycles = 0;
    Twist measured_velocity;

    Callback<const SensorState&> update_callback;
    Callback<ConnectionState> connection_callback;
    Callback<const Error&> error_callback;
    Callback<const RobotInfo&> info_callback;
    Callback<const Image&> image_callback;

    // I/O thread; stop is also read without the mutex.
    std::thread io_thread;
    std::atomic<bool> stop{false};

    // Camera thread and its socket (owned by the thread while it runs).
    std::mutex camera_mutex;  // serializes set_camera_streaming()
    std::thread camera_thread;
    std::atomic<bool> camera_stop{false};
    std::atomic<bool> streaming{false};
    std::atomic<std::uint16_t> image_port{0};
    net::UdpSocket udp;
    Image latest_image;
    bool has_image = false;

    // ---- helpers ----

    void set_state(ConnectionState s) {
        Callback<ConnectionState> cb;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (state == s) {
                return;
            }
            state = s;
            cb = connection_callback;
        }
        if (cb) {
            (*cb)(s);
        }
    }

    void report(const Error& e) {
        Callback<const Error&> cb;
        {
            std::lock_guard<std::mutex> lock(mutex);
            cb = error_callback;
        }
        if (cb) {
            (*cb)(e);
        }
    }

    // Waits for the given time or until stop is requested; returns false on stop.
    bool sleep_for(Milliseconds d) {
        std::unique_lock<std::mutex> lock(mutex);
        return !stop_cv.wait_for(lock, d, [this] { return stop.load(); });
    }

    // Builds the next IOControl and advances the one-shot flags.
    std::vector<std::uint8_t> next_io_control(bool final_stop) {
        std::lock_guard<std::mutex> lock(mutex);
        if (options.command_timeout.count() > 0 && Clock::now() - last_velocity_command > options.command_timeout) {
            command.motor_velocity = {};
        }
        Command c = command;
        if (final_stop) {
            c.motor_velocity = {};
        }
        c.reset_motor_position = {};
        if (reset_motor_cycles > 0) {
            c.reset_motor_position = {true, true, true};
            --reset_motor_cycles;
            odometry_restart_cycles = options.reset_cycles + 2;
        }
        c.reset_encoder_input = reset_encoder_cycles > 0;
        if (reset_encoder_cycles > 0) {
            --reset_encoder_cycles;
        }

        protocol::IOControl msg;
        msg.p2q = to_p2q(c);
        msg.shutdown = c.shutdown;
        msg.image_request = streaming.load();
        msg.image_port = msg.image_request ? image_port.load() : options.image_port;
        return protocol::encode(msg);
    }

    void publish_status(const protocol::IOStatus& status) {
        Callback<const SensorState&> cb;
        SensorState s = from_io_status(status);
        s.stamp = Clock::now();
        {
            std::lock_guard<std::mutex> lock(mutex);
            s.sequence = sensors.sequence + 1;
            if (odometry_restart_cycles > 0) {
                --odometry_restart_cycles;
                odometry.restart();
            }
            odometry.update(s.motor_position);
            measured_velocity = forward_kinematics(options.drive_layout, s.motor_velocity);
            sensors = s;
            cb = update_callback;
        }
        update_cv.notify_all();
        if (cb) {
            (*cb)(s);
        }
    }

    void publish_info(const protocol::Info& msg) {
        Callback<const RobotInfo&> cb;
        RobotInfo i{msg.text, msg.passive_mode};
        {
            std::lock_guard<std::mutex> lock(mutex);
            info = i;
            cb = info_callback;
        }
        if (cb) {
            (*cb)(i);
        }
    }

    // Sends IOControl (and CameraControl if needed) and waits for the IOStatus answer.
    Error exchange(net::TcpStream& stream, protocol::MessageParser& parser, bool final_stop = false) {
        bool send_camera = false;
        protocol::CameraControl camera;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (camera_settings_dirty) {
                send_camera = true;
                camera_settings_dirty = false;
                camera.resolution = camera_settings.resolution;
                camera.compression = camera_settings.compression;
                camera.brightness = camera_settings.brightness;
                camera.contrast = camera_settings.contrast;
                camera.auto_white_balance = camera_settings.auto_white_balance;
            }
        }
        std::vector<std::uint8_t> out = next_io_control(final_stop);
        if (send_camera) {
            const auto cc = protocol::encode(camera);
            out.insert(out.end(), cc.begin(), cc.end());
        }
        if (Error e = stream.send_all(out.data(), out.size(), options.timeout)) {
            if (send_camera) {
                std::lock_guard<std::mutex> lock(mutex);
                camera_settings_dirty = true;
            }
            return e;
        }

        const auto deadline = Clock::now() + options.timeout;
        std::uint8_t buffer[4096];
        while (true) {
            while (auto msg = parser.next()) {
                switch (msg->message_id()) {
                    case protocol::MessageId::IOStatus: {
                        auto status = protocol::decode_io_status(msg->payload);
                        if (!status) {
                            return Error(ErrorCode::ProtocolError, "malformed IOStatus");
                        }
                        publish_status(*status);
                        return {};
                    }
                    case protocol::MessageId::Info:
                        if (auto info_msg = protocol::decode_info(msg->payload)) {
                            publish_info(*info_msg);
                        }
                        break;
                    case protocol::MessageId::ConnectionBusy:
                        return Error(ErrorCode::ConnectionBusy, "the robot is controlled by another client");
                    default:
                        break;  // KeepAlive and unknown messages
                }
            }
            if (parser.error()) {
                return Error(ErrorCode::ProtocolError, "invalid message length, stream out of sync");
            }
            const auto now = Clock::now();
            if (now >= deadline) {
                return Error(ErrorCode::Timeout, "no answer from the robot");
            }
            if (stop && !final_stop) {
                return Error(ErrorCode::NotConnected, "stopped");
            }
            const auto left = std::chrono::duration_cast<Milliseconds>(deadline - now) + Milliseconds(1);
            const net::IoResult r = stream.recv(buffer, sizeof buffer, std::min(left, kPollSlice));
            if (r.status == net::IoStatus::Ok) {
                parser.feed(buffer, r.bytes);
            } else if (r.status == net::IoStatus::Closed) {
                return Error(ErrorCode::ConnectionClosed, "the robot closed the connection");
            } else if (r.status == net::IoStatus::Error) {
                return r.error;
            }
        }
    }

    Error establish(net::TcpStream& stream, protocol::MessageParser& parser) {
        parser.reset();
        {
            std::lock_guard<std::mutex> lock(mutex);
            camera_settings_dirty = true;  // the robot expects CameraControl in every session
            odometry.restart();
        }
        if (Error e = stream.connect(options.host, options.port, options.timeout)) {
            return e;
        }
        if (Error e = exchange(stream, parser)) {
            stream.close();
            return e;
        }
        return {};
    }

    void run(net::TcpStream stream, protocol::MessageParser parser) {
        Milliseconds reconnect_delay = kMinReconnectDelay;
        bool busy = false;
        auto next_cycle = Clock::now() + options.cycle;
        while (!stop) {
            if (!stream.connected()) {
                if (!options.auto_reconnect || busy) {
                    break;
                }
                set_state(ConnectionState::Connecting);
                if (!sleep_for(reconnect_delay)) {
                    break;
                }
                if (Error e = establish(stream, parser)) {
                    if (stop) {
                        break;
                    }
                    report(e);
                    busy = e.code == ErrorCode::ConnectionBusy;
                    reconnect_delay = std::min(reconnect_delay * 2, kMaxReconnectDelay);
                    continue;
                }
                reconnect_delay = kMinReconnectDelay;
                set_state(ConnectionState::Connected);
                next_cycle = Clock::now() + options.cycle;
                continue;
            }

            {
                std::unique_lock<std::mutex> lock(mutex);
                if (stop_cv.wait_until(lock, next_cycle, [this] { return stop.load(); })) {
                    break;
                }
            }
            // Keep a fixed rate, but do not try to catch up after a slow answer.
            next_cycle = std::max(next_cycle + options.cycle, Clock::now());
            if (Error e = exchange(stream, parser)) {
                if (stop) {
                    break;
                }
                report(e);
                busy = e.code == ErrorCode::ConnectionBusy;
                stream.close();
                if (!options.auto_reconnect || busy) {
                    break;
                }
            }
        }
        if (stream.connected()) {
            // Leave the robot standing still.
            exchange(stream, parser, true);
            stream.close();
        }
        set_state(ConnectionState::Disconnected);
    }

    void camera_loop() {
        protocol::ImageAssembler assembler;
        std::vector<std::uint8_t> buffer(kMaxDatagramSize);
        while (!camera_stop) {
            const net::IoResult r = udp.recv(buffer.data(), buffer.size(), kPollSlice);
            if (r.status != net::IoStatus::Ok) {
                continue;
            }
            auto data = assembler.push(protocol::ByteView(buffer.data(), r.bytes));
            if (!data) {
                continue;
            }
            Image image;
            image.stamp = Clock::now();
            image.type = data->header.type;
            image.width = data->header.width;
            image.height = data->header.height;
            image.channels = data->header.channels;
            image.bits_per_channel = data->header.bits_per_channel;
            image.data = std::move(data->data);
            Callback<const Image&> cb;
            {
                std::lock_guard<std::mutex> lock(mutex);
                latest_image = image;
                has_image = true;
                cb = image_callback;
            }
            if (cb) {
                (*cb)(image);
            }
        }
    }
};

Client::Client(ClientOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {}

Client::~Client() {
    disconnect();
    set_camera_streaming(false);
}

Error Client::connect() {
    if (impl_->io_thread.joinable()) {
        if (connection_state() != ConnectionState::Disconnected) {
            return Error(ErrorCode::InvalidArgument, "already connected");
        }
        impl_->io_thread.join();  // the thread gave up after an error
    }
    impl_->stop = false;
    impl_->set_state(ConnectionState::Connecting);
    net::TcpStream stream;
    protocol::MessageParser parser;
    if (Error e = impl_->establish(stream, parser)) {
        impl_->set_state(ConnectionState::Disconnected);
        return e;
    }
    impl_->set_state(ConnectionState::Connected);
    impl_->io_thread = std::thread([this, s = std::move(stream), p = std::move(parser)]() mutable {
        impl_->run(std::move(s), std::move(p));
    });
    return {};
}

void Client::disconnect() {
    impl_->stop = true;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->stop_cv.notify_all();
    }
    if (impl_->io_thread.joinable()) {
        impl_->io_thread.join();
    }
    impl_->set_state(ConnectionState::Disconnected);
}

ConnectionState Client::connection_state() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->state;
}

const ClientOptions& Client::options() const { return impl_->options; }

void Client::set_motor_velocities(const std::array<double, kNumMotors>& rad_s) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->command.motor_velocity = rad_s;
    impl_->last_velocity_command = Clock::now();
}

void Client::set_motor_velocity(std::size_t motor, double rad_s) {
    if (motor >= kNumMotors) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->command.motor_velocity[motor] = rad_s;
    impl_->last_velocity_command = Clock::now();
}

void Client::set_velocity(const Twist& twist) {
    set_motor_velocities(inverse_kinematics(impl_->options.drive_layout, twist));
}

void Client::stop() { set_motor_velocities({}); }

void Client::set_pid(std::size_t motor, const Pid& pid) {
    if (motor >= kNumMotors) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->command.pid[motor] = pid;
}

void Client::set_brake(std::size_t motor, bool engaged) {
    if (motor >= kNumMotors) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->command.brake[motor] = engaged;
}

void Client::reset_motor_positions() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->reset_motor_cycles = std::max(1, impl_->options.reset_cycles);
}

void Client::set_digital_output(std::size_t output, bool value) {
    if (output >= kNumDigitalOutputs) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->command.digital_output[output] = value;
}

void Client::set_relay(std::size_t relay, bool value) {
    if (relay >= kNumRelays) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->command.relay[relay] = value;
}

void Client::set_power_output(double duty) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->command.power_output = std::clamp(duty, -1.0, 1.0);
}

void Client::reset_encoder_input() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->reset_encoder_cycles = std::max(1, impl_->options.reset_cycles);
}

void Client::request_shutdown() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->command.shutdown = true;
}

Command Client::command() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->command;
}

void Client::set_command(const Command& command) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->command = command;
    impl_->last_velocity_command = Clock::now();
    if (command.reset_encoder_input) {
        impl_->reset_encoder_cycles = std::max(1, impl_->options.reset_cycles);
    }
    if (std::any_of(command.reset_motor_position.begin(), command.reset_motor_position.end(), [](bool b) { return b; })) {
        impl_->reset_motor_cycles = std::max(1, impl_->options.reset_cycles);
    }
}

SensorState Client::sensors() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->seen_sequence = impl_->sensors.sequence;
    return impl_->sensors;
}

bool Client::wait_for_update(Milliseconds timeout) {
    std::unique_lock<std::mutex> lock(impl_->mutex);
    const bool updated = impl_->update_cv.wait_for(lock, timeout, [this] {
        return impl_->sensors.sequence > impl_->seen_sequence;
    });
    impl_->seen_sequence = impl_->sensors.sequence;
    return updated;
}

RobotInfo Client::info() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->info;
}

Pose Client::pose() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->odometry.pose();
}

void Client::set_pose(const Pose& pose) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->odometry.set_pose(pose);
}

Twist Client::measured_velocity() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->measured_velocity;
}

void Client::set_camera_settings(const CameraSettings& settings) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->camera_settings = settings;
    impl_->camera_settings_dirty = true;
}

Error Client::set_camera_streaming(bool enable) {
    std::lock_guard<std::mutex> camera_lock(impl_->camera_mutex);
    if (enable == impl_->streaming) {
        return {};
    }
    if (!enable) {
        impl_->streaming = false;
        impl_->camera_stop = true;
        if (impl_->camera_thread.joinable()) {
            impl_->camera_thread.join();
        }
        impl_->udp.close();
        return {};
    }
    Error error;
    for (std::uint16_t i = 0; i < kImagePortAttempts; ++i) {
        error = impl_->udp.bind(static_cast<std::uint16_t>(impl_->options.image_port + i));
        if (!error) {
            break;
        }
    }
    if (error) {
        return error;
    }
    impl_->image_port = impl_->udp.port();
    impl_->camera_stop = false;
    impl_->camera_thread = std::thread([this] { impl_->camera_loop(); });
    impl_->streaming = true;
    return {};
}

bool Client::camera_streaming() const { return impl_->streaming; }

bool Client::latest_image(Image& image) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->has_image) {
        return false;
    }
    image = impl_->latest_image;
    return true;
}

void Client::on_update(std::function<void(const SensorState&)> callback) {
    auto cb = make_callback(std::move(callback));
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->update_callback.swap(cb);
}

void Client::on_connection_changed(std::function<void(ConnectionState)> callback) {
    auto cb = make_callback(std::move(callback));
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->connection_callback.swap(cb);
}

void Client::on_error(std::function<void(const Error&)> callback) {
    auto cb = make_callback(std::move(callback));
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->error_callback.swap(cb);
}

void Client::on_info(std::function<void(const RobotInfo&)> callback) {
    auto cb = make_callback(std::move(callback));
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->info_callback.swap(cb);
}

void Client::on_image(std::function<void(const Image&)> callback) {
    auto cb = make_callback(std::move(callback));
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->image_callback.swap(cb);
}

}  // namespace robotino_api1

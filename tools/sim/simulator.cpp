#include "simulator.hpp"

#include <cmath>

#include "net/socket.hpp"
#include "robotino_api1/protocol/image.hpp"
#include "robotino_api1/protocol/messages.hpp"

namespace robotino_api1::sim {
namespace {

using Clock = std::chrono::steady_clock;
constexpr std::chrono::milliseconds kSlice{20};
constexpr std::size_t kDatagramSize = 8192;

}  // namespace

struct Simulator::Listener {
    net::TcpListener tcp;
};

Simulator::Simulator(SimulatorOptions options) : options_(std::move(options)), listener_(std::make_unique<Listener>()) {
    state_.battery_voltage = 24.0;
    state_.battery_current = 1.2;
    state_.serial_frequency = 55;
    state_.firmware_version = 670;
    state_.distance_voltage.fill(0.4);
}

Simulator::~Simulator() { stop(); }

Error Simulator::start() {
    if (Error e = listener_->tcp.listen(options_.port, options_.address)) {
        return e;
    }
    port_ = listener_->tcp.port();
    stop_ = false;
    server_thread_ = std::thread([this] { serve(); });
    camera_thread_ = std::thread([this] { camera_loop(); });
    return {};
}

void Simulator::stop() {
    stop_ = true;
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
    if (camera_thread_.joinable()) {
        camera_thread_.join();
    }
    listener_->tcp.close();
}

Command Simulator::command() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return command_;
}

void Simulator::modify(const std::function<void(SensorState&)>& fn) {
    std::lock_guard<std::mutex> lock(mutex_);
    fn(state_);
}

SensorState Simulator::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void Simulator::kick() { kick_ = true; }

void Simulator::step(Clock::time_point now) {
    // Called with mutex_ held.
    const double dt = last_step_ == Clock::time_point{} ? 0.0 : std::chrono::duration<double>(now - last_step_).count();
    last_step_ = now;
    const double alpha = options_.motor_time_constant > 0 ? 1.0 - std::exp(-dt / options_.motor_time_constant) : 1.0;
    for (std::size_t m = 0; m < kNumMotors; ++m) {
        const double target = command_.brake[m] ? 0.0 : command_.motor_velocity[m];
        double& v = state_.motor_velocity[m];
        v += (target - v) * alpha;
        position_[m] += v * dt * kEncoderTicksPerRevolution / (2.0 * kPi);
        if (command_.reset_motor_position[m]) {
            position_[m] = 0.0;
        }
        state_.motor_position[m] = static_cast<std::int32_t>(std::lround(position_[m]));
        state_.motor_current[m] = 0.1 + 0.002 * std::fabs(v);
    }
    if (command_.reset_encoder_input) {
        state_.encoder_input_position = 0;
    }
    state_.power_output_current = std::fabs(command_.power_output) * 0.5;
    state_.board_time += static_cast<std::uint32_t>(dt * 1000.0);
}

void Simulator::serve() {
    net::TcpStream client;
    protocol::MessageParser parser;
    Clock::time_point last_message;
    std::uint8_t buffer[4096];

    while (!stop_) {
        // Refuse other clients while one is connected, accept a new one otherwise.
        net::TcpStream incoming;
        const auto accept_timeout = client.connected() ? std::chrono::milliseconds(0) : kSlice;
        if (listener_->tcp.accept(incoming, accept_timeout).status == net::IoStatus::Ok) {
            if (client.connected()) {
                const auto busy = protocol::encode_empty(protocol::MessageId::ConnectionBusy);
                incoming.send_all(busy.data(), busy.size(), kSlice);
                // Closing with unread input would reset the connection and could discard
                // ConnectionBusy: half-close and read until the client gives up.
                incoming.shutdown_send();
                std::uint8_t sink[256];
                const auto deadline = Clock::now() + std::chrono::milliseconds(500);
                while (Clock::now() < deadline && incoming.recv(sink, sizeof sink, kSlice).status != net::IoStatus::Closed) {
                }
                incoming.close();
            } else {
                client = std::move(incoming);
                parser.reset();
                last_message = Clock::now();
                ++connections_;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    client_address_ = client.peer_address();
                    image_request_ = false;
                    last_step_ = {};
                }
                protocol::Info info;
                info.text = options_.info_text;
                const auto out = protocol::encode(info);
                client.send_all(out.data(), out.size(), kSlice);
            }
        }
        if (!client.connected()) {
            continue;
        }

        const net::IoResult r = client.recv(buffer, sizeof buffer, kSlice);
        bool drop = kick_.exchange(false) || r.status == net::IoStatus::Closed || r.status == net::IoStatus::Error;
        if (r.status == net::IoStatus::Ok) {
            parser.feed(buffer, r.bytes);
            last_message = Clock::now();
        }
        while (auto msg = parser.next()) {
            if (msg->message_id() != protocol::MessageId::IOControl) {
                continue;  // CameraControl settings are ignored, KeepAlive only resets the timer
            }
            auto control = protocol::decode_io_control(msg->payload);
            if (!control) {
                drop = true;
                break;
            }
            ++io_controls_;
            std::vector<std::uint8_t> out;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                command_ = from_p2q(control->p2q);
                image_request_ = control->image_request;
                image_port_ = control->image_port;
                step(Clock::now());
                out = protocol::encode(to_io_status(state_));
            }
            if (!paused_ && client.send_all(out.data(), out.size(), kSlice)) {
                drop = true;
            }
        }
        if (parser.error() || Clock::now() - last_message > options_.idle_timeout) {
            drop = true;
        }
        if (drop) {
            client.close();
            std::lock_guard<std::mutex> lock(mutex_);
            command_ = Command{};
            image_request_ = false;
        }
    }
}

void Simulator::camera_loop() {
    net::UdpSocket udp;
    udp.open();
    std::uint32_t frame = 0;
    while (!stop_) {
        std::this_thread::sleep_for(options_.camera_period);
        std::string address;
        std::uint16_t port = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!image_request_ || client_address_.empty()) {
                continue;
            }
            address = client_address_;
            port = image_port_;
        }
        // A moving color gradient, 24-bit RGB.
        protocol::ImageData image;
        image.header.type = protocol::ImageType::Raw;
        image.header.width = options_.camera_width;
        image.header.height = options_.camera_height;
        image.header.channels = 3;
        image.header.bits_per_channel = 8;
        image.data.resize(static_cast<std::size_t>(options_.camera_width) * options_.camera_height * 3);
        for (std::uint32_t y = 0; y < options_.camera_height; ++y) {
            for (std::uint32_t x = 0; x < options_.camera_width; ++x) {
                std::uint8_t* p = &image.data[(static_cast<std::size_t>(y) * options_.camera_width + x) * 3];
                p[0] = static_cast<std::uint8_t>(x + frame * 4);
                p[1] = static_cast<std::uint8_t>(y);
                p[2] = static_cast<std::uint8_t>(frame);
            }
        }
        ++frame;
        bool ok = true;
        for (const auto& datagram : protocol::split_image(image, kDatagramSize)) {
            ok = !udp.send_to(address, port, datagram.data(), datagram.size()) && ok;
        }
        if (ok) {
            ++images_sent_;
        }
    }
}

}  // namespace robotino_api1::sim

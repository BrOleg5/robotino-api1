#include "robotino_api1/c_api.h"

#include <algorithm>
#include <cstring>
#include <mutex>
#include <string>

#include "robotino_api1/client.hpp"

using robotino_api1::Client;
using robotino_api1::ErrorCode;
using robotino_api1::Milliseconds;

struct rapi1_client {
    explicit rapi1_client(const robotino_api1::ClientOptions& options) : client(options) {}

    Client client;
    mutable std::mutex mutex;
    std::string last_error;
};

namespace {

rapi1_error to_c(ErrorCode code) { return static_cast<rapi1_error>(static_cast<int>(code)); }

rapi1_error remember(rapi1_client* c, const robotino_api1::Error& e) {
    std::lock_guard<std::mutex> lock(c->mutex);
    c->last_error = e.message;
    return to_c(e.code);
}

rapi1_error check(bool ok) { return ok ? RAPI1_OK : RAPI1_ERROR_INVALID_ARGUMENT; }

void to_c(const robotino_api1::SensorState& s, rapi1_sensor_state* out) {
    std::memset(out, 0, sizeof *out);
    out->sequence = s.sequence;
    for (std::size_t i = 0; i < robotino_api1::kNumMotors; ++i) {
        out->motor_velocity[i] = s.motor_velocity[i];
        out->motor_position[i] = s.motor_position[i];
        out->motor_current[i] = s.motor_current[i];
    }
    std::copy(s.distance_voltage.begin(), s.distance_voltage.end(), out->distance_voltage);
    std::copy(s.analog_input.begin(), s.analog_input.end(), out->analog_input);
    for (std::size_t i = 0; i < robotino_api1::kNumDigitalInputs; ++i) {
        out->digital_input[i] = s.digital_input[i] ? 1 : 0;
    }
    out->bumper = s.bumper ? 1 : 0;
    out->battery_voltage = s.battery_voltage;
    out->battery_current = s.battery_current;
    out->power_output_current = s.power_output_current;
    out->encoder_input_position = s.encoder_input_position;
    out->encoder_input_velocity = s.encoder_input_velocity;
    out->serial_frequency = s.serial_frequency;
    out->board_time = s.board_time;
    out->firmware_version = s.firmware_version;
    out->passive_mode = s.passive_mode ? 1 : 0;
}

}  // namespace

extern "C" {

const char* rapi1_version(void) { return "0.1.0"; }

const char* rapi1_error_string(rapi1_error error) {
    return robotino_api1::to_string(static_cast<ErrorCode>(static_cast<int>(error)));
}

void rapi1_options_init(rapi1_options* options) {
    if (options == nullptr) {
        return;
    }
    const robotino_api1::ClientOptions d;
    // The pointer stays valid: it refers to a string literal.
    options->host = "172.26.1.0";
    options->port = d.port;
    options->cycle_ms = static_cast<uint32_t>(d.cycle.count());
    options->timeout_ms = static_cast<uint32_t>(d.timeout.count());
    options->auto_reconnect = d.auto_reconnect ? 1 : 0;
    options->command_timeout_ms = static_cast<uint32_t>(d.command_timeout.count());
    options->image_port = d.image_port;
    options->base_radius = d.drive_layout.base_radius;
    options->wheel_radius = d.drive_layout.wheel_radius;
    options->gear_ratio = d.drive_layout.gear_ratio;
}

rapi1_client* rapi1_client_create(const rapi1_options* options) {
    rapi1_options o;
    rapi1_options_init(&o);
    if (options != nullptr) {
        o = *options;
    }
    if (o.host == nullptr || o.cycle_ms == 0 || o.timeout_ms == 0 || o.wheel_radius <= 0 || o.base_radius <= 0 ||
        o.gear_ratio <= 0) {
        return nullptr;
    }
    robotino_api1::ClientOptions co;
    co.host = o.host;
    co.port = o.port;
    co.cycle = Milliseconds(o.cycle_ms);
    co.timeout = Milliseconds(o.timeout_ms);
    co.auto_reconnect = o.auto_reconnect != 0;
    co.command_timeout = Milliseconds(o.command_timeout_ms);
    co.image_port = o.image_port;
    co.drive_layout.base_radius = o.base_radius;
    co.drive_layout.wheel_radius = o.wheel_radius;
    co.drive_layout.gear_ratio = o.gear_ratio;
    try {
        return new rapi1_client(co);
    } catch (...) {
        return nullptr;
    }
}

void rapi1_client_destroy(rapi1_client* client) { delete client; }

rapi1_error rapi1_connect(rapi1_client* client) { return remember(client, client->client.connect()); }

void rapi1_disconnect(rapi1_client* client) { client->client.disconnect(); }

rapi1_connection_state rapi1_connection_state_get(const rapi1_client* client) {
    return static_cast<rapi1_connection_state>(static_cast<int>(client->client.connection_state()));
}

const char* rapi1_last_error_message(const rapi1_client* client) {
    std::lock_guard<std::mutex> lock(client->mutex);
    return client->last_error.c_str();
}

void rapi1_set_motor_velocities(rapi1_client* client, const double rad_s[RAPI1_NUM_MOTORS]) {
    client->client.set_motor_velocities({rad_s[0], rad_s[1], rad_s[2]});
}

rapi1_error rapi1_set_motor_velocity(rapi1_client* client, unsigned motor, double rad_s) {
    client->client.set_motor_velocity(motor, rad_s);
    return check(motor < RAPI1_NUM_MOTORS);
}

void rapi1_set_velocity(rapi1_client* client, double vx, double vy, double omega) {
    client->client.set_velocity({vx, vy, omega});
}

void rapi1_stop(rapi1_client* client) { client->client.stop(); }

rapi1_error rapi1_set_pid(rapi1_client* client, unsigned motor, uint8_t kp, uint8_t ki, uint8_t kd) {
    client->client.set_pid(motor, {kp, ki, kd});
    return check(motor < RAPI1_NUM_MOTORS);
}

rapi1_error rapi1_set_brake(rapi1_client* client, unsigned motor, int engaged) {
    client->client.set_brake(motor, engaged != 0);
    return check(motor < RAPI1_NUM_MOTORS);
}

void rapi1_reset_motor_positions(rapi1_client* client) { client->client.reset_motor_positions(); }

rapi1_error rapi1_set_digital_output(rapi1_client* client, unsigned output, int value) {
    client->client.set_digital_output(output, value != 0);
    return check(output < RAPI1_NUM_DIGITAL_OUTPUTS);
}

rapi1_error rapi1_set_relay(rapi1_client* client, unsigned relay, int value) {
    client->client.set_relay(relay, value != 0);
    return check(relay < RAPI1_NUM_RELAYS);
}

void rapi1_set_power_output(rapi1_client* client, double duty) { client->client.set_power_output(duty); }

void rapi1_reset_encoder_input(rapi1_client* client) { client->client.reset_encoder_input(); }

void rapi1_request_shutdown(rapi1_client* client) { client->client.request_shutdown(); }

void rapi1_get_sensors(const rapi1_client* client, rapi1_sensor_state* state) { to_c(client->client.sensors(), state); }

int rapi1_wait_for_update(rapi1_client* client, uint32_t timeout_ms) {
    return client->client.wait_for_update(Milliseconds(timeout_ms)) ? 1 : 0;
}

size_t rapi1_get_info(const rapi1_client* client, char* buffer, size_t size) {
    const std::string text = client->client.info().text;
    if (buffer != nullptr && size > 0) {
        const std::size_t n = std::min(text.size(), size - 1);
        std::memcpy(buffer, text.data(), n);
        buffer[n] = '\0';
    }
    return text.size();
}

void rapi1_get_pose(const rapi1_client* client, rapi1_pose* pose) {
    const robotino_api1::Pose p = client->client.pose();
    *pose = rapi1_pose{p.x, p.y, p.theta};
}

void rapi1_set_pose(rapi1_client* client, const rapi1_pose* pose) { client->client.set_pose({pose->x, pose->y, pose->theta}); }

void rapi1_set_camera_settings(rapi1_client* client, int resolution, int compression, uint8_t brightness, uint8_t contrast,
                             int auto_white_balance) {
    robotino_api1::CameraSettings s;
    s.resolution = static_cast<robotino_api1::protocol::CameraResolution>(std::clamp(resolution, 0, 2));
    s.compression = static_cast<robotino_api1::protocol::CameraCompression>(std::clamp(compression, 0, 2));
    s.brightness = brightness;
    s.contrast = contrast;
    s.auto_white_balance = auto_white_balance != 0;
    client->client.set_camera_settings(s);
}

rapi1_error rapi1_camera_streaming(rapi1_client* client, int enable) {
    return remember(client, client->client.set_camera_streaming(enable != 0));
}

void rapi1_on_update(rapi1_client* client, rapi1_update_callback callback, void* user) {
    if (callback == nullptr) {
        client->client.on_update(nullptr);
        return;
    }
    client->client.on_update([callback, user](const robotino_api1::SensorState& s) {
        rapi1_sensor_state c;
        to_c(s, &c);
        callback(&c, user);
    });
}

void rapi1_on_connection_changed(rapi1_client* client, rapi1_connection_callback callback, void* user) {
    if (callback == nullptr) {
        client->client.on_connection_changed(nullptr);
        return;
    }
    client->client.on_connection_changed([callback, user](robotino_api1::ConnectionState s) {
        callback(static_cast<rapi1_connection_state>(static_cast<int>(s)), user);
    });
}

void rapi1_on_error(rapi1_client* client, rapi1_error_callback callback, void* user) {
    if (callback == nullptr) {
        client->client.on_error(nullptr);
        return;
    }
    client->client.on_error([callback, user](const robotino_api1::Error& e) { callback(to_c(e.code), e.message.c_str(), user); });
}

void rapi1_on_image(rapi1_client* client, rapi1_image_callback callback, void* user) {
    if (callback == nullptr) {
        client->client.on_image(nullptr);
        return;
    }
    client->client.on_image([callback, user](const robotino_api1::Image& img) {
        rapi1_image c{static_cast<int>(img.type), img.width, img.height, img.channels, img.bits_per_channel,
                    img.data.data(), img.data.size()};
        callback(&c, user);
    });
}

}  // extern "C"

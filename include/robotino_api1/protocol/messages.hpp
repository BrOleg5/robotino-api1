// Messages exchanged with the robot over TCP (port 80).
//
// Every message is [id: u8][payload length: u16][payload], all values little-endian.
// A session: the client connects and sends IOControl and CameraControl; the robot sends Info
// once and then answers every IOControl with an IOStatus. Without IOControl the robot stops
// the motors almost at once but keeps the connection open (checked for 3 s).
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "robotino_api1/export.hpp"
#include "robotino_api1/protocol/bytes.hpp"
#include "robotino_api1/protocol/qdsa.hpp"

namespace robotino_api1::protocol {

enum class MessageId : std::uint8_t {
    IOControl = 0,
    IOStatus = 1,
    CameraControl = 2,
    KeepAlive = 3,
    RawData = 4,
    ConnectionBusy = 5,  // another client already controls the robot
    Info = 6,
};

inline constexpr std::size_t kHeaderSize = 3;
inline constexpr std::size_t kMaxPayloadSize = 32000;

inline constexpr std::size_t kIOControlSize = 76;
inline constexpr std::size_t kIOStatusSize = 141;
inline constexpr std::size_t kCameraControlSize = 5;
inline constexpr std::size_t kInfoTextSize = 2000;
inline constexpr std::size_t kInfoSize = kInfoTextSize + 1;

struct NorthStarCommand {
    std::int8_t room_id = 0;
    std::uint8_t cal_flag = 0;
    std::uint8_t cal_state = 0;
    std::uint16_t cal_distance = 0;
};

// PC to robot, sent every cycle.
struct IOControl {
    bool image_request = false;  // stream camera images to image_port over UDP
    bool shutdown = false;
    std::uint16_t image_port = 8080;
    NorthStarCommand northstar;
    P2Q p2q;
    // The last 20 bytes are zero in the published sources (r23) but OpenRobotinoAPI 1.8.54
    // sends 1.0f at offset 13. The layout below is inferred from that and not yet verified:
    // set_odometry u8, odometry x/y/phi f32, NorthStar ceiling calibration f32, 3 bytes padding.
    bool set_odometry = false;
    float odometry_x = 0.0f;
    float odometry_y = 0.0f;
    float odometry_phi = 0.0f;
    float northstar_ceiling_cal = 1.0f;
};

struct NorthStarReadings {
    std::uint32_t sequence = 0;
    std::uint8_t room_id = 0;
    std::uint8_t num_spots_visible = 0;
    std::int16_t pos_x = 0;
    std::int16_t pos_y = 0;
    float pos_theta = 0.0f;
    std::uint16_t mag_spot0 = 0;
    std::uint16_t mag_spot1 = 0;
};

// Robot to PC, the answer to IOControl.
struct IOStatus {
    std::uint16_t serial_frequency = 0;  // update rate of the IO board link, Hz
    Q2P q2p;
    NorthStarReadings northstar;
    bool passive_mode = false;
    std::array<std::uint8_t, 19> reserved{};  // zero in all captures
};

enum class CameraResolution : std::uint8_t { QVGA = 0, VGA = 1, Custom = 2 };
enum class CameraCompression : std::uint8_t { High = 0, Low = 1, None = 2 };

struct CameraControl {
    CameraResolution resolution = CameraResolution::QVGA;
    CameraCompression compression = CameraCompression::High;
    std::uint8_t brightness = 0;
    std::uint8_t contrast = 0;
    bool auto_white_balance = false;
};

// Robot to PC, sent once after connecting: daemon and firmware versions as text.
struct Info {
    std::string text;
    bool passive_mode = false;
};

// A received message: id and payload without the header.
struct Message {
    std::uint8_t id = 0;
    std::vector<std::uint8_t> payload;

    MessageId message_id() const { return static_cast<MessageId>(id); }
};

// Encoders return the complete message including the header.
ROBOTINO_API1_EXPORT std::vector<std::uint8_t> encode(const IOControl& msg);
ROBOTINO_API1_EXPORT std::vector<std::uint8_t> encode(const IOStatus& msg);
ROBOTINO_API1_EXPORT std::vector<std::uint8_t> encode(const CameraControl& msg);
ROBOTINO_API1_EXPORT std::vector<std::uint8_t> encode(const Info& msg);
ROBOTINO_API1_EXPORT std::vector<std::uint8_t> encode_empty(MessageId id);  // KeepAlive, ConnectionBusy

// Decoders take the payload only (Message::payload).
ROBOTINO_API1_EXPORT std::optional<IOControl> decode_io_control(ByteView payload);
ROBOTINO_API1_EXPORT std::optional<IOStatus> decode_io_status(ByteView payload);
ROBOTINO_API1_EXPORT std::optional<CameraControl> decode_camera_control(ByteView payload);
ROBOTINO_API1_EXPORT std::optional<Info> decode_info(ByteView payload);

// Splits a TCP byte stream into messages.
class ROBOTINO_API1_EXPORT MessageParser {
public:
    void feed(const std::uint8_t* data, std::size_t size);
    // Returns the next complete message, if any. After an error (a length above
    // kMaxPayloadSize, i.e. the stream is out of sync) it returns nothing until reset().
    std::optional<Message> next();
    bool error() const { return error_; }
    void reset();

private:
    std::vector<std::uint8_t> buffer_;
    std::size_t pos_ = 0;
    bool error_ = false;
};

}  // namespace robotino_api1::protocol

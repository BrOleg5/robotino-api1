#pragma once

#include <string>

#include "robotino_api1/export.hpp"

namespace robotino_api1 {

enum class ErrorCode {
    None = 0,
    HostNotFound,
    ConnectionRefused,
    Timeout,
    ConnectionClosed,
    ConnectionBusy,  // another client controls the robot
    ProtocolError,
    SocketError,
    NotConnected,
    InvalidArgument,
};

ROBOTINO_API1_EXPORT const char* to_string(ErrorCode code);

struct Error {
    ErrorCode code = ErrorCode::None;
    std::string message;

    Error() = default;
    Error(ErrorCode c, std::string m = {}) : code(c), message(std::move(m)) {}  // NOLINT(google-explicit-constructor)

    explicit operator bool() const { return code != ErrorCode::None; }
};

}  // namespace robotino_api1

#pragma once

#include <cstdio>
#include <cstdlib>
#include <string>

#include "robotino_api1/client.hpp"

// Parses "host[:port]" from the first argument; defaults to the robot's factory address.
inline robotino_api1::ClientOptions options_from_args(int argc, char** argv) {
    robotino_api1::ClientOptions options;
    if (argc > 1) {
        const std::string address = argv[1];
        const auto colon = address.find(':');
        options.host = address.substr(0, colon);
        if (colon != std::string::npos) {
            options.port = static_cast<std::uint16_t>(std::atoi(address.c_str() + colon + 1));
        }
    }
    return options;
}

// Prints errors and connection changes reported by the background thread, such as reconnects.
inline void print_connection_events(robotino_api1::Client& robot) {
    robot.on_error([](const robotino_api1::Error& e) {
        std::fprintf(stderr, "error: %s: %s\n", robotino_api1::to_string(e.code), e.message.c_str());
    });
    robot.on_connection_changed([](robotino_api1::ConnectionState state) {
        std::fprintf(stderr, "connection: %s\n", robotino_api1::to_string(state));
    });
}

#pragma once

#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

// Reads a message captured from a real robot (see tools/pcap/pcap_extract.py).
inline std::vector<std::uint8_t> read_fixture(const std::string& name) {
    std::ifstream in(std::string(ROBOTINO_API1_TEST_DATA_DIR) + "/" + name, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// Payload of a complete message.
inline std::vector<std::uint8_t> payload_of(const std::vector<std::uint8_t>& message) {
    return {message.begin() + 3, message.end()};
}

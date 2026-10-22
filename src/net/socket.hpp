// Minimal blocking TCP/UDP sockets with timeouts over BSD sockets and Winsock.
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "robotino_api1/error.hpp"

namespace robotino_api1::net {

using Duration = std::chrono::milliseconds;

#if defined(_WIN32)
using NativeSocket = std::uintptr_t;  // SOCKET
#else
using NativeSocket = int;
#endif

enum class IoStatus { Ok, Timeout, Closed, Error };

struct IoResult {
    IoStatus status = IoStatus::Error;
    std::size_t bytes = 0;
    Error error;
};

class Socket {
public:
    Socket();
    explicit Socket(NativeSocket s) : s_(s) {}
    ~Socket() { close(); }
    Socket(Socket&& other) noexcept;
    Socket& operator=(Socket&& other) noexcept;
    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;

    bool valid() const;
    void close();
    NativeSocket native() const { return s_; }
    // Local port the socket is bound to, 0 on error.
    std::uint16_t local_port() const;

private:
    NativeSocket s_;
};

class TcpStream {
public:
    Error connect(const std::string& host, std::uint16_t port, Duration timeout);
    Error send_all(const std::uint8_t* data, std::size_t size, Duration timeout);
    // Waits up to timeout for data; Ok means at least one byte was read.
    IoResult recv(std::uint8_t* buffer, std::size_t size, Duration timeout);
    bool connected() const { return socket_.valid(); }
    void close() { socket_.close(); }
    // Sends FIN after the pending data; the peer still receives everything sent so far.
    void shutdown_send();
    // IPv4 address of the peer, e.g. for sending camera images to the client.
    std::string peer_address() const;

private:
    friend class TcpListener;
    Socket socket_;
};

class TcpListener {
public:
    // port 0 picks a free port, see port().
    Error listen(std::uint16_t port, const std::string& address = "0.0.0.0");
    IoResult accept(TcpStream& stream, Duration timeout);
    std::uint16_t port() const { return socket_.local_port(); }
    void close() { socket_.close(); }

private:
    Socket socket_;
};

class UdpSocket {
public:
    // port 0 picks a free port, see port().
    Error bind(std::uint16_t port, const std::string& address = "0.0.0.0");
    // Unbound socket for sending only.
    Error open();
    IoResult recv(std::uint8_t* buffer, std::size_t size, Duration timeout);
    Error send_to(const std::string& host, std::uint16_t port, const std::uint8_t* data, std::size_t size);
    std::uint16_t port() const { return socket_.local_port(); }
    void close() { socket_.close(); }

private:
    Socket socket_;
};

}  // namespace robotino_api1::net

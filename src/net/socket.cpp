#include "net/socket.hpp"

#include <cstring>
#include <memory>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace robotino_api1::net {
namespace {

#if defined(_WIN32)
constexpr NativeSocket kInvalid = static_cast<NativeSocket>(INVALID_SOCKET);
using SockLen = int;

int last_error() { return WSAGetLastError(); }
bool would_block(int e) { return e == WSAEWOULDBLOCK || e == WSAEINPROGRESS; }
bool refused(int e) { return e == WSAECONNREFUSED; }
bool reset(int e) { return e == WSAECONNRESET || e == WSAECONNABORTED; }
constexpr int kSendFlags = 0;

void ensure_initialized() {
    struct Winsock {
        Winsock() {
            WSADATA data;
            WSAStartup(MAKEWORD(2, 2), &data);
        }
        ~Winsock() { WSACleanup(); }
    };
    static Winsock winsock;
}

std::string error_message(int e) {
    char* text = nullptr;
    FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr,
                   static_cast<DWORD>(e), 0, reinterpret_cast<LPSTR>(&text), 0, nullptr);
    std::string message = text ? text : "error " + std::to_string(e);
    LocalFree(text);
    while (!message.empty() && (message.back() == '\n' || message.back() == '\r' || message.back() == '.')) {
        message.pop_back();
    }
    return message;
}

void set_nonblocking(NativeSocket s, bool enable) {
    u_long mode = enable ? 1 : 0;
    ioctlsocket(s, FIONBIO, &mode);
}

void close_native(NativeSocket s) { closesocket(s); }
#else
constexpr NativeSocket kInvalid = -1;
using SockLen = socklen_t;

int last_error() { return errno; }
bool would_block(int e) { return e == EWOULDBLOCK || e == EAGAIN || e == EINPROGRESS; }
bool refused(int e) { return e == ECONNREFUSED; }
bool reset(int e) { return e == ECONNRESET || e == EPIPE; }
#if defined(MSG_NOSIGNAL)
constexpr int kSendFlags = MSG_NOSIGNAL;
#else
constexpr int kSendFlags = 0;
#endif

void ensure_initialized() {}

std::string error_message(int e) { return std::strerror(e); }

void set_nonblocking(NativeSocket s, bool enable) {
    const int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, enable ? (flags | O_NONBLOCK) : (flags & ~O_NONBLOCK));
}

void close_native(NativeSocket s) { ::close(s); }
#endif

Error socket_error(const char* what, int e) {
    return Error(refused(e) ? ErrorCode::ConnectionRefused : ErrorCode::SocketError,
                 std::string(what) + ": " + error_message(e));
}

enum class Wait { Ready, Timeout, Error };

// Waits until the socket is readable (or writable). Windows select() has no FD_SETSIZE
// limit on socket values, POSIX poll() avoids it.
Wait wait_for(NativeSocket s, bool write, Duration timeout) {
#if defined(_WIN32)
    fd_set set;
    FD_ZERO(&set);
    FD_SET(s, &set);
    fd_set except;
    FD_ZERO(&except);
    FD_SET(s, &except);
    timeval tv;
    tv.tv_sec = static_cast<long>(timeout.count() / 1000);
    tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);
    // A failed connect is reported in the exception set.
    const int n = select(0, write ? nullptr : &set, write ? &set : nullptr, &except, &tv);
    if (n < 0) {
        return Wait::Error;
    }
    return n == 0 ? Wait::Timeout : Wait::Ready;
#else
    pollfd p{};
    p.fd = s;
    p.events = write ? POLLOUT : POLLIN;
    int n;
    do {
        n = poll(&p, 1, static_cast<int>(timeout.count()));
    } while (n < 0 && errno == EINTR);
    if (n < 0) {
        return Wait::Error;
    }
    return n == 0 ? Wait::Timeout : Wait::Ready;
#endif
}

struct AddrInfoDeleter {
    void operator()(addrinfo* a) const { freeaddrinfo(a); }
};
using AddrInfoPtr = std::unique_ptr<addrinfo, AddrInfoDeleter>;

Error resolve(const std::string& host, std::uint16_t port, int type, AddrInfoPtr& out) {
    ensure_initialized();
    addrinfo hints{};
    hints.ai_family = AF_INET;  // the robot speaks IPv4 only
    hints.ai_socktype = type;
    addrinfo* result = nullptr;
    const std::string service = std::to_string(port);
    const int rc = getaddrinfo(host.c_str(), service.c_str(), &hints, &result);
    if (rc != 0 || result == nullptr) {
        return Error(ErrorCode::HostNotFound, "cannot resolve " + host);
    }
    out.reset(result);
    return {};
}

Error bind_socket(Socket& socket, int type, std::uint16_t port, const std::string& address) {
    AddrInfoPtr addr;
    if (Error e = resolve(address, port, type, addr)) {
        return e;
    }
    Socket s(::socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol));
    if (!s.valid()) {
        return socket_error("socket", last_error());
    }
    if (type == SOCK_STREAM) {
#if !defined(_WIN32)
        // Lets a restarted emulator reuse its port right away. On Windows this option would
        // allow two processes to bind the same port, so it is not set there.
        int on = 1;
        setsockopt(s.native(), SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&on), sizeof on);
#endif
    }
    if (::bind(s.native(), addr->ai_addr, static_cast<SockLen>(addr->ai_addrlen)) != 0) {
        return socket_error("bind", last_error());
    }
    socket = std::move(s);
    return {};
}

IoResult recv_result(long n) {
    IoResult r;
    if (n > 0) {
        r.status = IoStatus::Ok;
        r.bytes = static_cast<std::size_t>(n);
    } else if (n == 0) {
        r.status = IoStatus::Closed;
    } else {
        const int e = last_error();
        r.status = reset(e) ? IoStatus::Closed : IoStatus::Error;
        r.error = socket_error("recv", e);
    }
    return r;
}

}  // namespace

Socket::Socket() : s_(kInvalid) {}

Socket::Socket(Socket&& other) noexcept : s_(other.s_) { other.s_ = kInvalid; }

Socket& Socket::operator=(Socket&& other) noexcept {
    if (this != &other) {
        close();
        s_ = other.s_;
        other.s_ = kInvalid;
    }
    return *this;
}

bool Socket::valid() const { return s_ != kInvalid; }

void Socket::close() {
    if (valid()) {
        close_native(s_);
        s_ = kInvalid;
    }
}

std::uint16_t Socket::local_port() const {
    sockaddr_in addr{};
    SockLen len = sizeof addr;
    if (!valid() || getsockname(s_, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
        return 0;
    }
    return ntohs(addr.sin_port);
}

Error TcpStream::connect(const std::string& host, std::uint16_t port, Duration timeout) {
    close();
    AddrInfoPtr addr;
    if (Error e = resolve(host, port, SOCK_STREAM, addr)) {
        return e;
    }
    Socket s(::socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol));
    if (!s.valid()) {
        return socket_error("socket", last_error());
    }
#if defined(SO_NOSIGPIPE)
    int nosigpipe = 1;
    setsockopt(s.native(), SOL_SOCKET, SO_NOSIGPIPE, &nosigpipe, sizeof nosigpipe);
#endif
    // Messages are small and answered one by one: send them immediately.
    int nodelay = 1;
    setsockopt(s.native(), IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nodelay), sizeof nodelay);

    set_nonblocking(s.native(), true);
    if (::connect(s.native(), addr->ai_addr, static_cast<SockLen>(addr->ai_addrlen)) != 0) {
        const int e = last_error();
        if (!would_block(e)) {
            return socket_error("connect", e);
        }
        const Wait w = wait_for(s.native(), true, timeout);
        if (w == Wait::Timeout) {
            return Error(ErrorCode::Timeout, "connect to " + host + ":" + std::to_string(port) + " timed out");
        }
        int so_error = 0;
        SockLen len = sizeof so_error;
        getsockopt(s.native(), SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&so_error), &len);
        if (w == Wait::Error || so_error != 0) {
            return socket_error("connect", so_error != 0 ? so_error : last_error());
        }
    }
    set_nonblocking(s.native(), false);
    socket_ = std::move(s);
    return {};
}

Error TcpStream::send_all(const std::uint8_t* data, std::size_t size, Duration timeout) {
    if (!connected()) {
        return Error(ErrorCode::NotConnected, "not connected");
    }
    std::size_t sent = 0;
    while (sent < size) {
        const Wait w = wait_for(socket_.native(), true, timeout);
        if (w == Wait::Timeout) {
            return Error(ErrorCode::Timeout, "send timed out");
        }
        const auto n = ::send(socket_.native(), reinterpret_cast<const char*>(data + sent),
                              static_cast<int>(size - sent), kSendFlags);
        if (n < 0) {
            const int e = last_error();
            return Error(reset(e) ? ErrorCode::ConnectionClosed : ErrorCode::SocketError,
                         std::string("send: ") + error_message(e));
        }
        sent += static_cast<std::size_t>(n);
    }
    return {};
}

IoResult TcpStream::recv(std::uint8_t* buffer, std::size_t size, Duration timeout) {
    IoResult r;
    if (!connected()) {
        r.error = Error(ErrorCode::NotConnected, "not connected");
        return r;
    }
    const Wait w = wait_for(socket_.native(), false, timeout);
    if (w == Wait::Timeout) {
        r.status = IoStatus::Timeout;
        return r;
    }
    if (w == Wait::Error) {
        r.error = socket_error("poll", last_error());
        return r;
    }
    return recv_result(::recv(socket_.native(), reinterpret_cast<char*>(buffer), static_cast<int>(size), 0));
}

void TcpStream::shutdown_send() {
    if (connected()) {
#if defined(_WIN32)
        ::shutdown(socket_.native(), SD_SEND);
#else
        ::shutdown(socket_.native(), SHUT_WR);
#endif
    }
}

std::string TcpStream::peer_address() const {
    sockaddr_in addr{};
    SockLen len = sizeof addr;
    if (!connected() || getpeername(socket_.native(), reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
        return {};
    }
    char text[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &addr.sin_addr, text, sizeof text);
    return text;
}

Error TcpListener::listen(std::uint16_t port, const std::string& address) {
    if (Error e = bind_socket(socket_, SOCK_STREAM, port, address)) {
        return e;
    }
    if (::listen(socket_.native(), 4) != 0) {
        const int e = last_error();
        socket_.close();
        return socket_error("listen", e);
    }
    return {};
}

IoResult TcpListener::accept(TcpStream& stream, Duration timeout) {
    IoResult r;
    const Wait w = wait_for(socket_.native(), false, timeout);
    if (w != Wait::Ready) {
        r.status = w == Wait::Timeout ? IoStatus::Timeout : IoStatus::Error;
        return r;
    }
    Socket s(::accept(socket_.native(), nullptr, nullptr));
    if (!s.valid()) {
        r.error = socket_error("accept", last_error());
        return r;
    }
    int nodelay = 1;
    setsockopt(s.native(), IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nodelay), sizeof nodelay);
    stream.socket_ = std::move(s);
    r.status = IoStatus::Ok;
    return r;
}

Error UdpSocket::bind(std::uint16_t port, const std::string& address) {
    return bind_socket(socket_, SOCK_DGRAM, port, address);
}

Error UdpSocket::open() {
    ensure_initialized();
    Socket s(::socket(AF_INET, SOCK_DGRAM, 0));
    if (!s.valid()) {
        return socket_error("socket", last_error());
    }
    socket_ = std::move(s);
    return {};
}

IoResult UdpSocket::recv(std::uint8_t* buffer, std::size_t size, Duration timeout) {
    IoResult r;
    const Wait w = wait_for(socket_.native(), false, timeout);
    if (w != Wait::Ready) {
        r.status = w == Wait::Timeout ? IoStatus::Timeout : IoStatus::Error;
        return r;
    }
    const auto n = ::recvfrom(socket_.native(), reinterpret_cast<char*>(buffer), static_cast<int>(size), 0, nullptr, nullptr);
    if (n < 0) {
        r.error = socket_error("recvfrom", last_error());
        return r;
    }
    r.status = IoStatus::Ok;
    r.bytes = static_cast<std::size_t>(n);
    return r;
}

Error UdpSocket::send_to(const std::string& host, std::uint16_t port, const std::uint8_t* data, std::size_t size) {
    AddrInfoPtr addr;
    if (Error e = resolve(host, port, SOCK_DGRAM, addr)) {
        return e;
    }
    const auto n = ::sendto(socket_.native(), reinterpret_cast<const char*>(data), static_cast<int>(size), 0,
                            addr->ai_addr, static_cast<SockLen>(addr->ai_addrlen));
    if (n < 0) {
        return socket_error("sendto", last_error());
    }
    return {};
}

}  // namespace robotino_api1::net

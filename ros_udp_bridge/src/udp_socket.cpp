#include "ros_udp_bridge/udp_socket.h"

#include <arpa/inet.h>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace ros_udp_bridge {

UdpSocket::UdpSocket(std::size_t buf_size, int timeout_ms)
    : buf_size_(buf_size), timeout_ms_(timeout_ms) {}

UdpSocket::~UdpSocket() { close(); }

bool UdpSocket::bind(const std::string& host, std::uint16_t port)
{
    if (fd_ >= 0) ::close(fd_);
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) return false;

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (host.empty() || host == "0.0.0.0") {
        addr.sin_addr.s_addr = INADDR_ANY;
    } else {
        if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) return false;
    }
    if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) return false;

    timeval tv;
    tv.tv_sec = timeout_ms_ / 1000;
    tv.tv_usec = (timeout_ms_ % 1000) * 1000;
    setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    return true;
}

bool UdpSocket::recv(std::string& data, std::string& from_ip, std::uint16_t& from_port)
{
    if (fd_ < 0) return false;
    std::vector<char> buf(buf_size_);
    sockaddr_in from;
    socklen_t from_len = sizeof(from);
    const ssize_t n = ::recvfrom(fd_, buf.data(), buf.size(), 0,
                                 reinterpret_cast<sockaddr*>(&from), &from_len);
    if (n <= 0) return false;
    data.assign(buf.data(), static_cast<std::size_t>(n));
    char ip[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
    from_ip = ip;
    from_port = ntohs(from.sin_port);
    return true;
}

bool UdpSocket::send(const std::string& host, std::uint16_t port, const std::string& data)
{
    const int s = (fd_ >= 0) ? fd_ : ::socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) return false;
    sockaddr_in to;
    std::memset(&to, 0, sizeof(to));
    to.sin_family = AF_INET;
    to.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &to.sin_addr) != 1) return false;
    const ssize_t n = ::sendto(s, data.data(), data.size(), 0,
                               reinterpret_cast<sockaddr*>(&to), sizeof(to));
    return n >= 0;
}

void UdpSocket::close()
{
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

}  // namespace ros_udp_bridge

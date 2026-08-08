#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace ros_udp_bridge {

/// 简单 UDP socket 封装（收/发，自包含）。
class UdpSocket {
public:
    UdpSocket(std::size_t buf_size = 4096, int timeout_ms = 100);
    ~UdpSocket();

    /// 绑定本地地址（收）。成功返回 true。
    bool bind(const std::string& host, std::uint16_t port);

    /// 阻塞接收（带超时）。成功返回 true，data 填内容。
    bool recv(std::string& data, std::string& from_ip, std::uint16_t& from_port);

    /// 发送到目标。成功返回 true。
    bool send(const std::string& host, std::uint16_t port, const std::string& data);

    void close();

private:
    int fd_ = -1;
    std::size_t buf_size_;
    int timeout_ms_;
};

}  // namespace ros_udp_bridge

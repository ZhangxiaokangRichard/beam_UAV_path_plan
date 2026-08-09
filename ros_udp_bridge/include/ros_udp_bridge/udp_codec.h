#pragma once

#include <cstdint>
#include <string>

namespace ros_udp_bridge {

/// 解码后的 UDP 位姿报文（red_interceptor / blue_uav / blue 共用字段）。
struct UdpPoseMsg {
    std::int64_t key = 0;
    std::string type;          // red_interceptor / blue_uav / blue ...
    std::string name;
    double t = 0.0;
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    double heading_deg = 0.0;  // 角度制（正北 0° 顺时针）
    double pitch_deg = 0.0;
    double roll_deg = 0.0;
    double velocity = 0.0;
    bool has_heading = false;
    std::int64_t target_key = 0;
    bool has_target_key = false;
    std::string event;
};

/// 解析单条 UDP JSON 报文。成功返回 true。
/// 含 UTF-8 容错清洗（非法字节 → U+FFFD，避免 property_tree 抛异常）。
bool parseUdpMsg(const std::string& payload, UdpPoseMsg& msg, std::string& error);

/// 编码 red_interceptor 发送报文（含 targetKey；fire=true 时带 event:FIRE）。
/// 返回 JSON 字符串（一包一 JSON）。
std::string encodeRedInterceptor(std::int64_t key, const std::string& name, double t,
                                 double lat_deg, double lon_deg, double alt_m,
                                 double heading_deg, double pitch_deg, double roll_deg,
                                 std::int64_t target_key, bool fire);

/// 服务入口指令（UDP 本地 11302，一包一 JSON）。
struct UdpCmdMsg {
    std::string type;   // "nav" | "guide" | "mode"
    bool start = false; // nav/guide：true=启动，false=停止
    std::string mode;   // mode：head-on / tail
};

/// 解析服务入口 UDP 指令 JSON。成功返回 true。
/// 格式：{"type":"nav","start":true} / {"type":"guide","start":true} /
///       {"type":"mode","mode":"head-on"} / {"type":"mode","mode":"tail"}
bool parseUdpCmd(const std::string& payload, UdpCmdMsg& cmd, std::string& error);

/// 编码服务状态上报 JSON（UDP 回传，供地面站查看）。
std::string encodeStatusReport(bool nav_running, bool guide_running, bool tracking,
                               const std::string& mode,
                               double uav_x, double uav_y, double uav_z,
                               double tgt_x, double tgt_y, double tgt_z);

/// 编码命中事件 JSON（UDP 发送）：{"type":"hit","targetKey":...,"t":...}
/// 由 udp_mssn_bridge 在 /target/crashed=true 时发送（target_key 取自参数）。
std::string encodeHitEvent(std::int64_t target_key, double t);

}  // namespace ros_udp_bridge

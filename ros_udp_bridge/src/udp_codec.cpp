#include "ros_udp_bridge/udp_codec.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <sstream>

namespace ros_udp_bridge {
namespace {
using boost::property_tree::ptree;

/// 非法 UTF-8 字节序列 → U+FFFD（EF BF BD），合法 UTF-8 原样保留。
std::string sanitizeJsonBytes(const std::string& payload)
{
    const unsigned char* p = reinterpret_cast<const unsigned char*>(payload.data());
    const std::size_t n = payload.size();
    std::string out;
    out.reserve(n + 8);
    std::size_t i = 0;
    while (i < n) {
        const unsigned char c = p[i];
        if (c < 0x80) {
            out += static_cast<char>(c);
            ++i;
        } else {
            int len = 0;
            if      (c >= 0xC2 && c <= 0xDF) len = 2;
            else if (c >= 0xE0 && c <= 0xEF) len = 3;
            else if (c >= 0xF0 && c <= 0xF4) len = 4;
            bool ok = (len > 0) && (i + len <= n);
            if (ok) {
                for (int k = 1; k < len; ++k)
                    if ((p[i + k] & 0xC0) != 0x80) { ok = false; break; }
            }
            if (ok) {
                out.append(payload, i, static_cast<std::size_t>(len));
                i += static_cast<std::size_t>(len);
            } else {
                out += '\xEF'; out += '\xBF'; out += '\xBD';
                ++i;
            }
        }
    }
    return out;
}
}

bool parseUdpMsg(const std::string& payload, UdpPoseMsg& msg, std::string& error)
{
    try {
        std::stringstream stream(sanitizeJsonBytes(payload));
        ptree tree;
        boost::property_tree::read_json(stream, tree);

        msg.key = tree.get<std::int64_t>("key", 0);
        msg.type = tree.get<std::string>("type", "");
        msg.name = tree.get<std::string>("name", "");
        msg.t = tree.get<double>("t", 0.0);
        msg.lat = tree.get<double>("lat", 0.0);
        msg.lon = tree.get<double>("lon", 0.0);
        msg.alt = tree.get<double>("alt", 0.0);
        msg.heading_deg = tree.get<double>("heading", 0.0);
        msg.has_heading = (tree.count("heading") > 0) && std::isfinite(msg.heading_deg);
        msg.pitch_deg = tree.get<double>("pitch", 0.0);
        msg.roll_deg = tree.get<double>("roll", 0.0);
        msg.velocity = tree.get<double>("velocity", 0.0);
        msg.target_key = tree.get<std::int64_t>("targetKey", 0);
        msg.has_target_key = tree.count("targetKey") > 0;
        msg.event = tree.get<std::string>("event", "");

        if (msg.key == 0 && msg.type.empty()) { error = "empty key/type"; return false; }
        if (!std::isfinite(msg.lat) || !std::isfinite(msg.lon) || !std::isfinite(msg.alt) ||
            msg.lat < -90.0 || msg.lat > 90.0 ||
            msg.lon < -180.0 || msg.lon > 180.0) {
            error = "invalid geodetic"; return false;
        }
        return true;
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }
}

std::string encodeRedInterceptor(std::int64_t key, const std::string& name, double t,
                                 double lat_deg, double lon_deg, double alt_m,
                                 double heading_deg, double pitch_deg, double roll_deg,
                                 std::int64_t target_key, bool fire)
{
    // 手动拼接 JSON：boost property_tree::write_json 会把数值写成字符串，
    // 而协议要求数值（无引号），故手动构造保证类型正确。
    std::ostringstream oss;
    oss << "{\"key\":" << key
        << ",\"type\":\"red_interceptor\""
        << ",\"name\":\"" << name << '"'
        << ",\"t\":" << t
        << ",\"lat\":" << lat_deg
        << ",\"lon\":" << lon_deg
        << ",\"alt\":" << alt_m
        << ",\"heading\":" << heading_deg
        << ",\"pitch\":" << pitch_deg
        << ",\"roll\":" << roll_deg
        << ",\"targetKey\":" << target_key;
    if (fire) oss << ",\"event\":\"FIRE\"";
    oss << '}';
    return oss.str();
}

bool parseUdpCmd(const std::string& payload, UdpCmdMsg& cmd, std::string& error)
{
    try {
        std::stringstream stream(sanitizeJsonBytes(payload));
        ptree tree;
        boost::property_tree::read_json(stream, tree);

        cmd.type = tree.get<std::string>("type", "");
        cmd.start = tree.get<bool>("start", false);
        cmd.mode = tree.get<std::string>("mode", "");

        for (auto& c : cmd.type) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (cmd.type != "nav" && cmd.type != "guide" && cmd.type != "mode") {
            error = "unknown type: " + cmd.type;
            return false;
        }
        if (cmd.type == "mode") {
            for (auto& c : cmd.mode) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            if (cmd.mode != "head-on" && cmd.mode != "tail") {
                error = "invalid mode: " + cmd.mode;
                return false;
            }
        }
        return true;
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }
}

std::string encodeStatusReport(bool nav_running, bool guide_running, bool tracking,
                               const std::string& mode,
                               double uav_x, double uav_y, double uav_z,
                               double tgt_x, double tgt_y, double tgt_z)
{
    // 用单个引号字符拼接，避免手工转义
    std::ostringstream oss;
    oss << '{' << '"' << "type" << '"' << ':' << '"' << "status" << '"'
        << ',' << '"' << "nav" << '"' << ':' << (nav_running ? "true" : "false")
        << ',' << '"' << "guide" << '"' << ':' << (guide_running ? "true" : "false")
        << ',' << '"' << "tracking" << '"' << ':' << (tracking ? "true" : "false")
        << ',' << '"' << "mode" << '"' << ':' << '"' << mode << '"'
        << ',' << '"' << "uav" << '"' << ':' << '[' << uav_x << ',' << uav_y << ',' << uav_z << ']'
        << ',' << '"' << "target" << '"' << ':' << '[' << tgt_x << ',' << tgt_y << ',' << tgt_z << ']'
        << '}';
    return oss.str();
}

std::string encodeHitEvent(std::int64_t target_key, double t)
{
    // 命中事件：数值为 JSON number（非字符串），由 udp_mssn_bridge 在 crashed=true 时发送
    std::ostringstream oss;
    oss << "{\"type\":\"hit\",\"targetKey\":" << target_key
        << ",\"t\":" << t << '}';
    return oss.str();
}

}  // namespace ros_udp_bridge

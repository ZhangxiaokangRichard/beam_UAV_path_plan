/**
 * @file json_codec.cpp
 * @brief MQTT JSON 消息的 schema 校验和编码。
 *
 * 编解码层只负责消息格式，不负责 ROS、MQTT 和坐标系业务。
 * 这样可以在不启动 ROS 的情况下单独测试字段校验和 JSON 契约。
 */

#include "ros_mqtt_bridge/json_codec.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <sstream>
#include <unistd.h>

namespace ros_mqtt_bridge {
namespace {
using boost::property_tree::ptree;

template <typename T>
bool readRequired(const ptree& tree, const std::string& key, T& value, std::string& error)
{
    // 必填字段缺失时整条消息失败，禁止发布“半更新”的目标状态。
    auto optional = tree.get_optional<T>(key);
    if (!optional) { error = "missing field: " + key; return false; }
    value = *optional;
    return true;
}

/**
 * 把非法 UTF-8 字节替换为 '?'。
 *
 * 现场实测（2026-10-09）：`uav_caster/uavs_information` 中某个 `uav_sn` 为
 * 20 字节 `0xFF`（占位/损坏），使 `boost::property_tree::read_json` 抛
 * `invalid code sequence`，导致 **整条列表被丢弃** → 两个 tracker 的 uav_sn
 * 一直为空 → `aoa/uav/state`、`aoa/target/state` 整条断流。
 * 净化后仅损坏项受影响（其 sn 变为一串 '?'，不会匹配任何 filter），
 * 正常项（ASCII SN）字节不变。
 */
std::string sanitizeUtf8(const std::string& in)
{
    std::string out;
    out.reserve(in.size());
    const std::size_t n = in.size();
    std::size_t i = 0;
    while (i < n) {
        const unsigned char c = static_cast<unsigned char>(in[i]);
        std::size_t len = 0;
        if (c < 0x80) {
            len = 1;
        } else if ((c & 0xE0) == 0xC0) {
            len = 2;
        } else if ((c & 0xF0) == 0xE0) {
            len = 3;
        } else if ((c & 0xF8) == 0xF0) {
            len = 4;
        } else {
            out += '?';
            ++i;
            continue;
        }
        if (i + len > n) {
            out += '?';
            ++i;
            continue;
        }
        bool valid = true;
        for (std::size_t k = 1; k < len; ++k) {
            if ((static_cast<unsigned char>(in[i + k]) & 0xC0) != 0x80) { valid = false; break; }
        }
        if (!valid) {
            out += '?';
            ++i;
            continue;
        }
        out.append(in, i, len);
        i += len;
    }
    return out;
}

/** 容错备用路径：不依赖 JSON 解析，直接扫描所有 `"uav_sn":"..."` 字符串值。
 *  仅在严格 JSON 解析失败时使用（载荷结构损坏等）。 */
bool scanUavSnValues(const std::string& payload, std::vector<std::string>& out)
{
    const std::string key = "\"uav_sn\"";
    std::size_t pos = 0;
    while ((pos = payload.find(key, pos)) != std::string::npos) {
        pos += key.size();
        const auto colon = payload.find(':', pos);
        if (colon == std::string::npos) break;
        const auto q1 = payload.find('"', colon);
        if (q1 == std::string::npos) break;
        const auto q2 = payload.find('"', q1 + 1);
        if (q2 == std::string::npos) break;
        std::string sn = payload.substr(q1 + 1, q2 - q1 - 1);
        if (!sn.empty()) out.push_back(std::move(sn));
        pos = q2 + 1;
    }
    return !out.empty();
}
}

bool decodeTargetState(const std::string& payload, TargetState& state, std::string& error)
{
    try {
    // schema 版本是协议演进的入口，未知版本不能静默按旧格式解释。
        std::stringstream stream(sanitizeUtf8(payload));
        ptree tree;
        boost::property_tree::read_json(stream, tree);
        const std::string schema = tree.get<std::string>("schema", "");
        if (schema != "beam_dubins.target_state.v1") { error = "unsupported schema"; return false; }
        if (!readRequired(tree, "target_id", state.target_id, error) ||
            !readRequired(tree, "sequence", state.sequence, error) ||
            !readRequired(tree, "position.latitude_deg", state.geodetic.latitude_deg, error) ||
            !readRequired(tree, "position.longitude_deg", state.geodetic.longitude_deg, error) ||
            !readRequired(tree, "position.altitude_m", state.geodetic.altitude_m, error)) return false;
        state.timestamp = tree.get<std::string>("timestamp", "");
        state.east_mps = tree.get<double>("velocity.east_mps", 0.0);
        state.north_mps = tree.get<double>("velocity.north_mps", 0.0);
        state.up_mps = tree.get<double>("velocity.up_mps", 0.0);
        state.local.yaw = tree.get<double>("yaw_rad", 0.0);
        state.local.pitch = tree.get<double>("pitch_rad", 0.0);
        state.valid_for_ms = tree.get<double>("valid_for_ms", 500.0);
        // 经纬度范围检查在坐标转换前完成，避免非法输入污染 ECEF/TF。
        if (!std::isfinite(state.geodetic.latitude_deg) || state.geodetic.latitude_deg < -90.0 || state.geodetic.latitude_deg > 90.0 ||
            !std::isfinite(state.geodetic.longitude_deg) || state.geodetic.longitude_deg < -180.0 || state.geodetic.longitude_deg > 180.0 ||
            !std::isfinite(state.geodetic.altitude_m)) { error = "invalid geodetic position"; return false; }
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}

bool decodeUavInfo(const std::string& payload, TargetState& state, std::string& error)
{
    // 解析地面站真实 MQTT 消息：method=uav_info，数据嵌套在 data 中。
    // yaw/pitch/roll 为角度制，需由调用方转换为弧度。
    try {
        std::stringstream stream(sanitizeUtf8(payload));
        ptree tree;
        boost::property_tree::read_json(stream, tree);

        const std::string method = tree.get<std::string>("method", "");
        if (method != "uav_info") { error = "unsupported method: " + method; return false; }

        // 顶层字段：mid（去重标识）和 timestamp（epoch 毫秒）
        state.mid    = tree.get<std::string>("mid", "");
        // 使用 unsigned long long 而非 uint64_t：boost property_tree 的 get<>
        // 对标准整数类型更兼容，避免某些平台上的隐式转换失败。
        const auto ts_ms = tree.get<unsigned long long>("timestamp", 0ULL);
        state.timestamp = std::to_string(ts_ms);
        state.sequence  = 0;

        const auto data = tree.get_child_optional("data");
        if (!data) { error = "missing data object"; return false; }
        const auto& d = *data;

        // UAV 标识
        state.uav_sn = d.get<std::string>("uav_sn", "");
        state.mode   = d.get<std::string>("mode", "");
        // 用 uav_sn 的简易哈希生成 target_id（兼容旧字段），
        // 碰撞概率在单 Broker 场景可忽略。
        std::size_t hash = std::hash<std::string>{}(state.uav_sn);
        state.target_id = static_cast<std::uint32_t>(hash);

        // 地理坐标：实际消息中 longitude 在 latitude 之前
        if (!readRequired(d, "longitude", state.geodetic.longitude_deg, error) ||
            !readRequired(d, "latitude",  state.geodetic.latitude_deg,  error) ||
            !readRequired(d, "altitude",  state.geodetic.altitude_m,     error))
            return false;

        // 姿态：MQTT 消息为角度制，航空约定 (0°=北, CW)。
        // 转为弧度后，yaw 需转换为 ENU 约定 (0=东, CCW)。
        const double yaw_deg   = d.get<double>("yaw",   0.0);
        const double pitch_deg = d.get<double>("pitch", 0.0);
        const double roll_deg  = d.get<double>("roll",  0.0);
        state.local.yaw   = (90.0 - yaw_deg)   * M_PI / 180.0;
        state.local.pitch = pitch_deg * M_PI / 180.0;
        state.local.roll  = roll_deg  * M_PI / 180.0;

        // 速度：实际消息只有标量地速 vvg 和爬升率 climb_cur
        state.ground_speed_mps = d.get<double>("vvg", 0.0);
        state.east_mps = 0.0;   // 由 bridge 主循环用 COG×vvg 填充
        state.north_mps = 0.0;
        state.up_mps = d.get<double>("climb_cur", 0.0);

        state.valid_for_ms = 500.0;

        // 经纬度范围检查
        if (!std::isfinite(state.geodetic.latitude_deg) ||
            state.geodetic.latitude_deg < -90.0 || state.geodetic.latitude_deg > 90.0 ||
            !std::isfinite(state.geodetic.longitude_deg) ||
            state.geodetic.longitude_deg < -180.0 || state.geodetic.longitude_deg > 180.0 ||
            !std::isfinite(state.geodetic.altitude_m)) {
            error = "invalid geodetic position"; return false;
        }
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}

bool decodeUavsInfo(const std::string& payload, std::vector<std::string>& uav_list,
                    std::string& error)
{
    // 解析 uav_caster/uavs_information：
    // {"timestamp":...,"sender":"MQTT-CM","mid":"...","data":{"uavs":[{"uav_sn":"..."},...]}}
    try {
        std::stringstream stream(sanitizeUtf8(payload));
        ptree tree;
        boost::property_tree::read_json(stream, tree);
        const auto data = tree.get_child_optional("data");
        if (!data) { error = "missing data object"; return false; }
        const auto uavs = data->get_child_optional("uavs");
        if (!uavs || uavs->empty()) { error = "empty uavs array"; return false; }
        uav_list.clear();
        for (const auto& item : *uavs) {
            std::string sn = item.second.get<std::string>("uav_sn", "");
            if (!sn.empty()) uav_list.push_back(sn);
        }
        if (uav_list.empty()) { error = "no uav_sn found"; return false; }
        return true;
    } catch (const std::exception& exception) {
        // 严格解析失败 → 容错扫描（宁可丢掉一个坏字段，也不能丢掉整条列表）
        std::vector<std::string> scanned;
        if (scanUavSnValues(payload, scanned)) {
            uav_list = std::move(scanned);
            error.clear();
            return true;
        }
        error = exception.what();
        return false;
    }
}

bool decodeFstInfo(const std::string& payload, std::vector<std::string>& uav_list,
                   std::string& error)
{
    // 解析 aoa/uav_center/fst_info：
    // {"method":"fst_info","mid":"...","timestamp":...,"data":{"barrels":[
    //   {"barrel_index":2,"fill":1,"self_check":1,"uav_sn":"V2.4AOACRAFT-NOCONF0"},
    //   ...]}}
    try {
        std::stringstream stream(sanitizeUtf8(payload));
        ptree tree;
        boost::property_tree::read_json(stream, tree);
        const auto data = tree.get_child_optional("data");
        if (!data) { error = "missing data object"; return false; }
        const auto barrels = data->get_child_optional("barrels");
        if (!barrels || barrels->empty()) { error = "empty barrels array"; return false; }
        uav_list.clear();
        for (const auto& item : *barrels) {
            const auto& obj = item.second;
            std::string sn = obj.get<std::string>("uav_sn", "");
            if (!sn.empty()) uav_list.push_back(sn);
        }
        if (uav_list.empty()) { error = "no uav_sn found in fst_info"; return false; }
        return true;
    } catch (const std::exception& exception) {
        // 严格解析失败 → 容错扫描（同 decodeUavsInfo）
        std::vector<std::string> scanned;
        if (scanUavSnValues(payload, scanned)) {
            uav_list = std::move(scanned);
            error.clear();
            return true;
        }
        error = exception.what();
        return false;
    }
}

std::string encodeWaypoint(const Waypoint& waypoint)
{
    // 每个 ROS Path 点独立编码为一帧 MQTT 航点，靠 route_id/wp_id/total 重组路径。
    ptree tree;
    tree.put("schema", "beam_dubins.waypoint.v1");
    tree.put("route_id", waypoint.route_id);
    tree.put("sequence", waypoint.sequence);
    tree.put("wp_id", waypoint.wp_id);
    tree.put("total", waypoint.total);
    tree.put("timestamp", waypoint.timestamp);
    tree.put("frame", "wgs84");
    tree.put("position.latitude_deg", waypoint.geodetic.latitude_deg);
    tree.put("position.longitude_deg", waypoint.geodetic.longitude_deg);
    tree.put("position.altitude_m", waypoint.geodetic.altitude_m);
    tree.put("yaw_rad", waypoint.local.yaw);
    tree.put("curvature_1pm", waypoint.curvature_1pm);
    tree.put("speed_mps", waypoint.speed_mps);
    std::ostringstream stream;
    boost::property_tree::write_json(stream, tree, false);
    return stream.str();
}

std::string encodeRouteStatus(std::uint32_t route_id, const std::string& status,
                              const std::string& detail)
{
    ptree tree;
    tree.put("schema", "beam_dubins.route_status.v1");
    tree.put("route_id", route_id);
    tree.put("status", status);
    tree.put("detail", detail);
    std::ostringstream stream;
    boost::property_tree::write_json(stream, tree, false);
    return stream.str();
}

namespace {
/** 将 double 格式化为 JSON 数值，整数时省略尾部 .0。 */
std::string fmtJsonDouble(double v)
{
    std::ostringstream s;
    s << std::defaultfloat;
    s << v;
    return s.str();
}
}  // namespace

std::string encodeFlyPoint(double lon_deg, double lat_deg,
                           double alt_m, double radius_m)
{
    // 手动构造 JSON 以确保数值字段不被 boost::property_tree 序列化为字符串。
    std::ostringstream stream;
    stream << "{\n"
           << "  \"method\": \"fly_point\",\n"
           << "  \"data\":{\n"
           << "    \"target_longitude\":" << std::fixed << std::setprecision(7) << lon_deg << ",\n"
           << "    \"target_latitude\":" << std::fixed << std::setprecision(7) << lat_deg << ",\n"
           << "    \"target_altitude\":" << std::fixed << std::setprecision(1) << alt_m << ",\n"
           << "    \"target_radius\":" << fmtJsonDouble(radius_m) << "\n"
           << "    }\n"
           << "}";
    return stream.str();
}

bool decodeStateBool(const std::string& payload, const std::string& key,
                     bool& value, std::string& error)
{
    try {
        std::stringstream stream(sanitizeUtf8(payload));
        ptree tree;
        boost::property_tree::read_json(stream, tree);
        const auto optional = tree.get_optional<bool>(key);
        if (!optional) { error = "missing field: " + key; return false; }
        value = *optional;
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}

bool decodeStateString(const std::string& payload, const std::string& key,
                       std::string& value, std::string& error)
{
    try {
        std::stringstream stream(sanitizeUtf8(payload));
        ptree tree;
        boost::property_tree::read_json(stream, tree);
        const auto optional = tree.get_optional<std::string>(key);
        if (!optional) { error = "missing field: " + key; return false; }
        value = *optional;
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        return false;
    }
}

// ── 拦截模式（迎头 / 尾追）入站翻译 ───────────────────────

namespace {

/// 去掉全部空白并转小写
std::string lowerNoSpace(const std::string& in)
{
    std::string out;
    out.reserve(in.size());
    for (const char c : in) {
        const unsigned char u = static_cast<unsigned char>(c);
        if (std::isspace(u)) continue;
        out += static_cast<char>(std::tolower(u));
    }
    return out;
}

/// 取出 `"<key>"` 后的第一个字符串值（不依赖 JSON 解析，容忍结构损坏）
bool findStringAfterKey(const std::string& payload, const std::string& quoted_key,
                        std::string& out)
{
    const auto k = payload.find(quoted_key);
    if (k == std::string::npos) return false;
    const auto colon = payload.find(':', k + quoted_key.size());
    if (colon == std::string::npos) return false;
    const auto q1 = payload.find('"', colon);
    if (q1 == std::string::npos) return false;
    const auto q2 = payload.find('"', q1 + 1);
    if (q2 == std::string::npos) return false;
    out = payload.substr(q1 + 1, q2 - q1 - 1);
    return true;
}

}  // namespace

std::string normalizeInterceptMode(const std::string& raw)
{
    const std::string s = lowerNoSpace(raw);
    if (s == "head-on" || s == "headon" || s == "head_on") return "head-on";
    if (s == "tail") return "tail";
    return {};
}

std::string extractInterceptMode(const std::string& payload)
{
    const auto not_space = [](unsigned char c) { return !std::isspace(c); };
    auto b = std::find_if(payload.begin(), payload.end(), not_space);
    auto e = std::find_if(payload.rbegin(), payload.rend(), not_space).base();
    if (b >= e) return {};
    const std::string trimmed(b, e);

    // 裸串形式："head-on" / "tail"
    if (trimmed.front() != '{') return normalizeInterceptMode(trimmed);

    // JSON：优先 intercept_mode，其次 mode（`find` 不区分嵌套层级，
    // 因此 {"data":{"mode":...}} 同样命中）
    for (const char* key : {"\"intercept_mode\"", "\"mode\""}) {
        std::string value;
        if (!findStringAfterKey(trimmed, key, value)) continue;
        const std::string mode = normalizeInterceptMode(value);
        if (!mode.empty()) return mode;
    }
    return {};
}

std::string encodeStateBool(const std::string& key, bool value)
{
    return "{\"" + key + "\":" + (value ? "true" : "false") + "}";
}

std::string encodeStateString(const std::string& key, const std::string& value)
{
    return "{\"" + key + "\":\"" + value + "\"}";
}

std::string encodeUavStateField(const std::string& key, const UavStateJson& s)
{
    std::ostringstream o;
    o << "\"" << key << "\":{"
      << "\"position\":{\"x\":" << fmtJsonDouble(s.px)
      << ",\"y\":" << fmtJsonDouble(s.py)
      << ",\"z\":" << fmtJsonDouble(s.pz) << "},"
      << "\"orientation\":{\"x\":" << fmtJsonDouble(s.ox)
      << ",\"y\":" << fmtJsonDouble(s.oy)
      << ",\"z\":" << fmtJsonDouble(s.oz)
      << ",\"w\":" << fmtJsonDouble(s.ow) << "},"
      << "\"linear\":{\"x\":" << fmtJsonDouble(s.vx)
      << ",\"y\":" << fmtJsonDouble(s.vy)
      << ",\"z\":" << fmtJsonDouble(s.vz) << "}"
      << "}";
    return o.str();
}

std::string encodePlannedPathField(const std::string& key,
                                   const std::vector<std::array<double, 6>>& path)
{
    std::ostringstream o;
    o << "\"" << key << "\":[";
    for (size_t i = 0; i < path.size(); ++i) {
        if (i) o << ",";
        const auto& p = path[i];
        o << "[" << fmtJsonDouble(p[0]) << "," << fmtJsonDouble(p[1]) << ","
          << fmtJsonDouble(p[2]) << "," << fmtJsonDouble(p[3]) << ","
          << fmtJsonDouble(p[4]) << "," << fmtJsonDouble(p[5]) << "]";
    }
    o << "]";
    return o.str();
}

// ═══════════════════════════════════════════════════════════════
// 2.0 新增：协议基础包与三条控制指令（4.4.17 / 4.4.19 / 4.4.21）
// ═══════════════════════════════════════════════════════════════

namespace {
/// 当前 Unix 毫秒时间戳
long long nowMs()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}
}  // namespace

std::string MidGenerator::next()
{
    std::ostringstream o;
    o << "uavg-" << static_cast<long long>(::getpid()) << "-" << nowMs() << "-" << (++seq_);
    return o.str();
}

std::string makeBasePackage(const std::string& method, const std::string& data_json,
                            const std::string& mid)
{
    std::ostringstream o;
    o << "{\n"
      << "  \"method\": \"" << method << "\",\n"
      << "  \"timestamp\": " << nowMs() << ",\n"
      << "  \"mid\": \"" << mid << "\",\n"
      << "  \"data\": " << data_json << "\n"
      << "}";
    return o.str();
}

std::string encodeSetCruiseMode(const std::string& mid)
{
    return makeBasePackage("set_cruise_mode", "{}", mid);
}

// ── 出站显示通道（零 ROS，可单测）──────────────────────────

namespace {

/// 定点小数格式化：经纬 7 位 / 高度与 ENU 3 位 / 四元数 6 位
std::string fixedStr(double v, int precision)
{
    std::ostringstream s;
    s << std::fixed << std::setprecision(precision) << v;
    return s.str();
}

/// JSON 字符串转义（status_message 可能含引号/反斜杠/控制字符）
std::string escapeJsonString(const std::string& in)
{
    std::string out;
    out.reserve(in.size() + 8);
    for (const unsigned char c : in) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    return out;
}

}  // namespace

std::string encodeDisplayPoseData(const DisplayPose& p)
{
    std::ostringstream o;
    o << "{"
      << "\"uav_sn\":\"" << escapeJsonString(p.uav_sn) << "\","
      << "\"role\":\"" << escapeJsonString(p.role) << "\","
      << "\"frame\":\"" << escapeJsonString(p.frame) << "\","
      << "\"mode\":\"" << escapeJsonString(p.mode) << "\","
      << "\"position\":{\"x\":" << fixedStr(p.x, 3) << ",\"y\":" << fixedStr(p.y, 3)
      << ",\"z\":" << fixedStr(p.z, 3) << "},"
      << "\"orientation\":{\"x\":" << fixedStr(p.qx, 6) << ",\"y\":" << fixedStr(p.qy, 6)
      << ",\"z\":" << fixedStr(p.qz, 6) << ",\"w\":" << fixedStr(p.qw, 6) << "},"
      << "\"linear\":{\"x\":" << fixedStr(p.vx, 3) << ",\"y\":" << fixedStr(p.vy, 3)
      << ",\"z\":" << fixedStr(p.vz, 3) << "}";
    if (p.include_geodetic) {
        o << ",\"geodetic\":{\"longitude\":" << fixedStr(p.longitude_deg, 7)
          << ",\"latitude\":" << fixedStr(p.latitude_deg, 7)
          << ",\"altitude\":" << fixedStr(p.altitude_m, 2) << "}";
    }
    o << "}";
    return o.str();
}

std::string encodeDisplayPathData(const DisplayPath& p)
{
    std::ostringstream o;
    o << "{"
      << "\"uav_sn\":\"" << escapeJsonString(p.uav_sn) << "\","
      << "\"frame\":\"" << escapeJsonString(p.frame) << "\","
      << "\"success\":" << (p.success ? "true" : "false") << ","
      << "\"cost\":" << fixedStr(p.cost, 3) << ","
      << "\"status_message\":\"" << escapeJsonString(p.status_message) << "\","
      << "\"count\":" << p.points.size() << ","
      << encodePlannedPathField("path", p.points) << "}";
    return o.str();
}

std::vector<std::array<double, 6>> decimatePath(const std::vector<std::array<double, 6>>& points,
                                                std::size_t max_points)
{
    if (max_points == 0 || points.size() <= max_points) return points;
    if (max_points == 1) return {points.front()};

    const std::size_t n = points.size();
    std::vector<std::array<double, 6>> out;
    out.reserve(max_points);
    // i*(n-1)/(max-1)：首点索引 0、末点索引 n-1，中间严格递增（因 max<=n）
    for (std::size_t i = 0; i < max_points; ++i) {
        out.push_back(points[(i * (n - 1)) / (max_points - 1)]);
    }
    return out;
}

std::string encodeSetFlyHead(double heading_deg, const std::string& mid)
{
    // 先归一到 [0,360)，再量化到 0.1°，最后回绕——
    // 否则 359.96 经 1 位小数四舍五入会变成 "360.0"，超出协议约定的 [0,360)（现场抓包实测过）。
    double heading = heading_deg - 360.0 * std::floor(heading_deg / 360.0);   // [0,360)
    heading = std::round(heading * 10.0) / 10.0;                              // 量化到 0.1°
    if (heading >= 360.0) heading = 0.0;

    std::ostringstream data;
    data << "{\"heading\":" << std::fixed << std::setprecision(1) << heading << "}";
    return makeBasePackage("set_fly_head", data.str(), mid);
}

std::string encodeSetFlyHeight(double height_m, int height_type, const std::string& mid)
{
    std::ostringstream data;
    data << "{\"height\":" << std::llround(height_m) << ",\"height_type\":" << height_type << "}";
    return makeBasePackage("set_fly_height", data.str(), mid);
}

}  // namespace ros_mqtt_bridge
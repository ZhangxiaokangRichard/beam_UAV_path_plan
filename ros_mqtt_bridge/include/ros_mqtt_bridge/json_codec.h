#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "ros_mqtt_bridge/message_types.h"

namespace ros_mqtt_bridge {

bool decodeTargetState(const std::string& payload, TargetState& state,
                       std::string& error);

/** 解析地面站真实 MQTT 消息格式 (aoa/uav_center/+/uav_info).
 *  yaw/pitch/roll 为角度制航空约定，函数内转为弧度+ENU 约定。 */
bool decodeUavInfo(const std::string& payload, TargetState& state,
                   std::string& error);

/** 解析 uav_caster/uavs_information，提取全部 UAV 序列号。 */
bool decodeUavsInfo(const std::string& payload, std::vector<std::string>& uav_list,
                    std::string& error);

/** 解析 aoa/uav_center/fst_info，提取全部 UAV 序列号。
 *  barrels 数组中每项的 uav_sn 追加到列表，保持原始顺序。 */
bool decodeFstInfo(const std::string& payload, std::vector<std::string>& uav_list,
                   std::string& error);

std::string encodeWaypoint(const Waypoint& waypoint);
std::string encodeRouteStatus(std::uint32_t route_id, const std::string& status,
                              const std::string& detail = "");

/** 编码 fly_point MQTT 命令 JSON。
 *  @param lon_deg  目标经度 (WGS84)
 *  @param lat_deg  目标纬度 (WGS84)
 *  @param alt_m    目标高度 (m)
 *  @param radius_m 转弯半径 (m) */
std::string encodeFlyPoint(double lon_deg, double lat_deg,
                           double alt_m, double radius_m);

/** 解析 {"<key>": <bool>} 状态/命令载荷（如 nav/guide 启停）。 */
bool decodeStateBool(const std::string& payload, const std::string& key,
                     bool& value, std::string& error);

/** 解析 {"<key>": "<string>"} 状态/命令载荷（如拦截模式）。 */
bool decodeStateString(const std::string& payload, const std::string& key,
                       std::string& value, std::string& error);

/** 编码 {"<key>": <bool>} 状态载荷。 */
std::string encodeStateBool(const std::string& key, bool value);

/** 编码 {"<key>": "<string>"} 状态载荷。 */
std::string encodeStateString(const std::string& key, const std::string& value);

// ══════════════════════════════════════════════════════════════
// 2.1 新增：拦截模式（迎头 / 尾追）入站翻译
//   MQTT `aoa/ai_guide/intercept/mode` → ROS `aoa/uav/intercept_mode_cmd`
//   （下游 uav_guide_loop_node 只接受 head-on | tail）
// ══════════════════════════════════════════════════════════════

/** 规范化拦截模式：接受 `head-on` / `headon` / `head_on` / `tail`
 *  （大小写不敏感、容忍空白），返回**规范形式** `"head-on"` / `"tail"`；
 *  非法或空 → 返回空串。零 ROS，可单测。 */
std::string normalizeInterceptMode(const std::string& raw);

/** 从 MQTT 载荷中容错提取拦截模式：
 *  裸串 `"tail"`，或 JSON `{"mode":...}` / `{"intercept_mode":...}` /
 *  `{"data":{"mode":...}}`（含嵌套）。
 *  返回规范形式 `"head-on"` / `"tail"`；无法识别 → 返回空串。 */
std::string extractInterceptMode(const std::string& payload);

/** 编码 <key>: {position, orientation, linear} 字段片段（供对象内拼接）。 */
std::string encodeUavStateField(const std::string& key, const UavStateJson& state);

/** 编码 <key>: [[x,y,z,yaw,pitch,curvature], ...] 字段片段（供对象内拼接）。 */
std::string encodePlannedPathField(const std::string& key,
                                   const std::vector<std::array<double, 6>>& path);

// ═══════════════════════════════════════════════════════════════
// 2.0 新增：协议基础包 + 三条控制指令编码
//   参考《LJD 网络通信协议》4.4.17 / 4.4.19 / 4.4.21
//   （入站解析函数 decode* 行为与 1.0.1 完全一致，未改动）
// ═══════════════════════════════════════════════════════════════

/** 报文 ID 生成器：mid = "uavg-<pid>-<epoch_ms>-<seq>"，进程内单调递增。 */
class MidGenerator {
public:
    std::string next();

private:
    std::uint64_t seq_ = 0;
};

/** 协议基础包（method / timestamp / mid / data）。
 *  @param data_json 已编码好的 data 对象字面量（如 "{}"）。 */
std::string makeBasePackage(const std::string& method, const std::string& data_json,
                            const std::string& mid);

/** set_cruise_mode（协议 4.4.17）：切换巡航模式，data 为空对象。 */
std::string encodeSetCruiseMode(const std::string& mid);

/** set_fly_head（协议 4.4.19）：设置飞行航向 heading ∈ [0,360)，单位度。 */
std::string encodeSetFlyHead(double heading_deg, const std::string& mid);

/** set_fly_height（协议 4.4.21）：设置飞行高度；height_type: 0 场高 / 1 绝对高度。 */
std::string encodeSetFlyHeight(double height_m, int height_type, const std::string& mid);

// ══════════════════════════════════════════════════════════════
// 2.1 新增：**出站显示通道**（ROS → MQTT，供窗口程序 / 地面站叠加显示）
//   话题：aoa/ai_guide/display/{uav_state,target_state,planed_path}
//   payload：基础包（method / timestamp / mid / data），data 见下方结构体
//   说明：这些函数**零 ROS 依赖**，可直接单测真正的 JSON 契约。
// ══════════════════════════════════════════════════════════════

/** 显示位姿（uav_state / target_state 的 data 内容）。 */
struct DisplayPose {
    std::string uav_sn;
    std::string role;             // "own" | "target"
    std::string frame;            // 如 "map"
    std::string mode;             // 飞控模式（如 "Auto"），可为空串
    double x = 0.0, y = 0.0, z = 0.0;                // ENU 位置 (m)
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;   // 姿态四元数
    double vx = 0.0, vy = 0.0, vz = 0.0;             // 线速度 (m/s，ENU)
    bool include_geodetic = false;                   // 是否附带经纬高
    double longitude_deg = 0.0, latitude_deg = 0.0, altitude_m = 0.0;
};

/** 显示路径（planed_path 的 data 内容）。 */
struct DisplayPath {
    std::string uav_sn;
    std::string frame;
    std::string status_message;
    bool success = false;
    double cost = 0.0;
    std::vector<std::array<double, 6>> points;   // [x,y,z,yaw,pitch,curvature]
};

/** 编码 uav_state / target_state 的 data 对象（position/orientation/linear 平铺）。 */
std::string encodeDisplayPoseData(const DisplayPose& pose);

/** 编码 planed_path 的 data 对象（含 success/cost/status_message/count/path）。 */
std::string encodeDisplayPathData(const DisplayPath& path);

/** 路径抽稀：保留**首尾点**，其余等间隔取样。
 *  max_points==0 或 points.size()<=max_points 时原样返回；max_points==1 只留首点。 */
std::vector<std::array<double, 6>> decimatePath(const std::vector<std::array<double, 6>>& points,
                                                std::size_t max_points);

}  // namespace ros_mqtt_bridge
#pragma once

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

}  // namespace ros_mqtt_bridge
/**
 * @file mode_gate.h
 * @brief 制导模式与出站截断矩阵（**零 ROS，便于单测**）
 *
 * 模式权威源：MQTT 入站 `aoa/ai_guide/set_cruise_mode` 的 mode 字段（经 mqtt_mssn_bridge 转入 ROS）。
 * 出站只由 mqtt_control_bridge 执行，因此把判定逻辑抽成纯函数，可离线单测 3×2 矩阵与边沿。
 */

#pragma once

#include <string>

namespace ros_mqtt_bridge {

/// 制导模式（字符串取值固定：auto / setpoint / cruise）
enum class CruiseMode { Auto, Setpoint, Cruise };

/// 解析模式字符串（兼容 data 包裹由调用方处理）；非法返回 false 且不修改 out
bool parseCruiseMode(const std::string& raw, CruiseMode& out);

/// 模式名（"auto" / "setpoint" / "cruise"）
const char* cruiseModeName(CruiseMode mode);

/// 出站通道
enum class OutChannel {
    Setpoint,   // → fly_point（指点：位置 + 半径）
    Guidance,   // → set_fly_head + set_fly_height（巡航：航向 + 高度）
};

/**
 * 出站截断矩阵（Q/R2 裁决）：
 *   auto     : 两路都不发（初始态与失效安全态）
 *   setpoint : 仅 Setpoint 路（**不发** set_cruise_mode）
 *   cruise   : 仅 Guidance 路（并由边沿另发一次 set_cruise_mode）
 */
bool isChannelEnabled(CruiseMode mode, OutChannel channel);

/// 模式跃迁是否需要上发一次 set_cruise_mode（进入 cruise 的边沿）
bool needsCruiseModeCommand(CruiseMode previous, CruiseMode current);

}  // namespace ros_mqtt_bridge

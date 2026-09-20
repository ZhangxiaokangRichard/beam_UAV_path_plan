/**
 * @file mode_gate.h
 * @brief 导航模式与出站「透传映射表」（**零 ROS，便于单测**）
 *
 * 数据流（2026-09-20 裁决）：
 *   MQTT `aoa/ai_guide/nav/mode`  (auto | setpoint | guidance)
 *     → mqtt_mssn_bridge → ROS `aoa/uav/nav_mode`（std_msgs/String，**latched**）
 *     → mqtt_control_bridge 按本表 **1:1 翻译**为 MQTT 指令
 *
 * **纯翻译器语义**：不节流、不去抖、不做阈值判断；出站频率完全等于上游 ROS 话题频率
 * （两个制导节点均为 5 Hz 定时器 → guidance 模式出站亦为 5 Hz）。
 *
 * 模式是**状态**而非边沿：`mqtt_control_bridge` 启动时通过 latched 的 `aoa/uav/nav_mode`
 * 立即得到当前模式，无需等待“模式变化”事件。
 */

#pragma once

#include <string>

namespace ros_mqtt_bridge {

/// 导航模式（字符串取值固定：auto / setpoint / guidance）
enum class NavMode { Auto, Setpoint, Guidance };

/// 解析模式字符串（容忍空白/大小写）；非法返回 false 且不修改 out
bool parseNavMode(const std::string& raw, NavMode& out);

/// 模式名（"auto" / "setpoint" / "guidance"）
const char* navModeName(NavMode mode);

/// 出站指令种类（= MQTT method）
enum class OutMethod {
    FlyPoint,        // ← aoa/uav/setpoint  (setpoint 模式)
    SetCruiseMode,   // ← aoa/uav/guidance  (guidance 模式，与 head/height 同频)
    SetFlyHead,      // ← aoa/uav/guidance
    SetFlyHeight,    // ← aoa/uav/guidance
};

/// 某模式下「每条上游消息」应翻译出的 MQTT 指令集合
struct OutboundPlan {
    bool fly_point = false;
    bool set_cruise_mode = false;
    bool set_fly_head = false;
    bool set_fly_height = false;
};

/**
 * 透传映射表：
 *   auto     : 全 false（失效安全：不出网）
 *   setpoint : fly_point
 *   guidance : set_cruise_mode + set_fly_head + set_fly_height（三条同频发出）
 */
OutboundPlan outboundPlan(NavMode mode);

/// 便捷判定（等价于 outboundPlan(mode) 的对应字段）
bool forwardsSetpoint(NavMode mode);
bool forwardsGuidance(NavMode mode);

}  // namespace ros_mqtt_bridge

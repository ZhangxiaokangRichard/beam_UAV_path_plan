/**
 * @file mode_gate.cpp
 * @brief 导航模式解析与出站透传映射表实现
 */

#include "ros_mqtt_bridge/mode_gate.h"

#include <algorithm>
#include <cctype>

namespace ros_mqtt_bridge {
namespace {

std::string lowerTrim(std::string s)
{
    const auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

}  // namespace

bool parseNavMode(const std::string& raw, NavMode& out)
{
    const std::string s = lowerTrim(raw);
    if (s == "auto") {
        out = NavMode::Auto;
        return true;
    }
    if (s == "setpoint") {
        out = NavMode::Setpoint;
        return true;
    }
    if (s == "guidance") {
        out = NavMode::Guidance;
        return true;
    }
    return false;
}

const char* navModeName(NavMode mode)
{
    switch (mode) {
        case NavMode::Setpoint: return "setpoint";
        case NavMode::Guidance: return "guidance";
        case NavMode::Auto:
        default: return "auto";
    }
}

OutboundPlan outboundPlan(NavMode mode)
{
    OutboundPlan plan;
    switch (mode) {
        case NavMode::Setpoint:
            plan.fly_point = true;
            break;
        case NavMode::Guidance:
            plan.set_cruise_mode = true;
            plan.set_fly_head = true;
            plan.set_fly_height = true;
            break;
        case NavMode::Auto:
        default:
            break;   // 失效安全：不出网
    }
    return plan;
}

bool forwardsSetpoint(NavMode mode) { return outboundPlan(mode).fly_point; }

bool forwardsGuidance(NavMode mode)
{
    const OutboundPlan plan = outboundPlan(mode);
    return plan.set_cruise_mode || plan.set_fly_head || plan.set_fly_height;
}

}  // namespace ros_mqtt_bridge

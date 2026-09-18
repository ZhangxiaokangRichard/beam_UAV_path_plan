/**
 * @file mode_gate.cpp
 * @brief 制导模式解析与出站截断矩阵实现
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

bool parseCruiseMode(const std::string& raw, CruiseMode& out)
{
    const std::string s = lowerTrim(raw);
    if (s == "auto") {
        out = CruiseMode::Auto;
        return true;
    }
    if (s == "setpoint") {
        out = CruiseMode::Setpoint;
        return true;
    }
    if (s == "cruise") {
        out = CruiseMode::Cruise;
        return true;
    }
    return false;
}

const char* cruiseModeName(CruiseMode mode)
{
    switch (mode) {
        case CruiseMode::Setpoint: return "setpoint";
        case CruiseMode::Cruise: return "cruise";
        case CruiseMode::Auto:
        default: return "auto";
    }
}

bool isChannelEnabled(CruiseMode mode, OutChannel channel)
{
    if (mode == CruiseMode::Auto) return false;
    if (mode == CruiseMode::Setpoint) return channel == OutChannel::Setpoint;
    return channel == OutChannel::Guidance;   // CruiseMode::Cruise
}

bool needsCruiseModeCommand(CruiseMode previous, CruiseMode current)
{
    return current == CruiseMode::Cruise && previous != CruiseMode::Cruise;
}

}  // namespace ros_mqtt_bridge

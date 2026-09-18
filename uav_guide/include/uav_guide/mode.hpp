/**
 * @file mode.hpp
 * @brief 拦截模式解析（零 ROS）
 *
 * 取值与 1.0.1 一致：head-on（兼容 headon / head_on）与 tail；非法值回落 head-on。
 * 注意：这与 MQTT 的 set_cruise_mode（auto/setpoint/cruise）是**正交**的两套语义。
 */

#pragma once

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>

#include "uav_guide/types.hpp"

namespace uav_guide::mode {

inline std::string to_lower_trim(std::string s)
{
    const auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

/// 解析拦截模式字符串；非法返回 nullopt
inline std::optional<InterceptMode> parse(const std::string& raw)
{
    const std::string s = to_lower_trim(raw);
    if (s == "head-on" || s == "headon" || s == "head_on") return InterceptMode::HeadOn;
    if (s == "tail") return InterceptMode::Tail;
    return std::nullopt;
}

/// 解析失败时回落 head-on（与 1.0.1 normalize_intercept_mode 一致）
inline InterceptMode normalize(const std::string& raw)
{
    const auto parsed = parse(raw);
    return parsed.has_value() ? *parsed : InterceptMode::HeadOn;
}

inline const char* to_string(InterceptMode m)
{
    return m == InterceptMode::HeadOn ? "head-on" : "tail";
}

}  // namespace uav_guide::mode

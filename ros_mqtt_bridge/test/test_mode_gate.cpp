/**
 * @file test_mode_gate.cpp
 * @brief 模式解析与出站透传映射表单测（纯翻译器语义）
 */

#include <gtest/gtest.h>

#include "ros_mqtt_bridge/mode_gate.h"

using ros_mqtt_bridge::forwardsGuidance;
using ros_mqtt_bridge::forwardsSetpoint;
using ros_mqtt_bridge::NavMode;
using ros_mqtt_bridge::navModeName;
using ros_mqtt_bridge::outboundPlan;
using ros_mqtt_bridge::parseNavMode;
using ros_mqtt_bridge::shouldSendCruiseMode;

// ── 解析 ─────────────────────────────────────────────────────

TEST(ModeGate, ParseValid)
{
    NavMode m = NavMode::Auto;
    ASSERT_TRUE(parseNavMode("auto", m));
    EXPECT_EQ(m, NavMode::Auto);
    ASSERT_TRUE(parseNavMode("setpoint", m));
    EXPECT_EQ(m, NavMode::Setpoint);
    ASSERT_TRUE(parseNavMode("guidance", m));
    EXPECT_EQ(m, NavMode::Guidance);
}

TEST(ModeGate, ParseTolerantCaseAndSpace)
{
    NavMode m = NavMode::Auto;
    ASSERT_TRUE(parseNavMode("  GUIDANCE ", m));
    EXPECT_EQ(m, NavMode::Guidance);
    ASSERT_TRUE(parseNavMode("SetPoint", m));
    EXPECT_EQ(m, NavMode::Setpoint);
    ASSERT_TRUE(parseNavMode("\tAUTO", m));
    EXPECT_EQ(m, NavMode::Auto);
}

TEST(ModeGate, ParseInvalidKeepsValue)
{
    NavMode m = NavMode::Guidance;
    EXPECT_FALSE(parseNavMode("", m));
    EXPECT_FALSE(parseNavMode("   ", m));
    EXPECT_FALSE(parseNavMode("cruise", m)) << "旧模式名 cruise 已废弃，不做别名兼容";
    EXPECT_FALSE(parseNavMode("manual", m));
    EXPECT_EQ(m, NavMode::Guidance) << "非法输入不得修改 out";
}

TEST(ModeGate, NamesRoundTrip)
{
    EXPECT_STREQ(navModeName(NavMode::Auto), "auto");
    EXPECT_STREQ(navModeName(NavMode::Setpoint), "setpoint");
    EXPECT_STREQ(navModeName(NavMode::Guidance), "guidance");

    for (auto mode : {NavMode::Auto, NavMode::Setpoint, NavMode::Guidance}) {
        NavMode back = NavMode::Auto;
        ASSERT_TRUE(parseNavMode(navModeName(mode), back));
        EXPECT_EQ(back, mode);
    }
}

// ── 透传映射表（纯翻译器：1 条上游 → N 条出网）─

TEST(ModeGate, AutoIsSilent)
{
    const auto p = outboundPlan(NavMode::Auto);
    EXPECT_FALSE(p.fly_point);
    EXPECT_FALSE(p.set_cruise_mode);
    EXPECT_FALSE(p.set_fly_head);
    EXPECT_FALSE(p.set_fly_height);
    EXPECT_FALSE(forwardsSetpoint(NavMode::Auto));
    EXPECT_FALSE(forwardsGuidance(NavMode::Auto));
}

TEST(ModeGate, SetpointForwardsOnlyFlyPoint)
{
    const auto p = outboundPlan(NavMode::Setpoint);
    EXPECT_TRUE(p.fly_point);
    EXPECT_FALSE(p.set_cruise_mode);
    EXPECT_FALSE(p.set_fly_head);
    EXPECT_FALSE(p.set_fly_height);
    EXPECT_TRUE(forwardsSetpoint(NavMode::Setpoint));
    EXPECT_FALSE(forwardsGuidance(NavMode::Setpoint));
}

TEST(ModeGate, GuidanceForwardsThreeMethodsPerTick)
{
    // set_cruise_mode 不再边沿触发：与 head/height **同频**随每帧 guidance 发出
    const auto p = outboundPlan(NavMode::Guidance);
    EXPECT_FALSE(p.fly_point);
    EXPECT_TRUE(p.set_cruise_mode);
    EXPECT_TRUE(p.set_fly_head);
    EXPECT_TRUE(p.set_fly_height);
    EXPECT_FALSE(forwardsSetpoint(NavMode::Guidance));
    EXPECT_TRUE(forwardsGuidance(NavMode::Guidance));
}

TEST(ModeGate, SetpointEmitsOnePerUpstreamMessage)
{
    const auto p = outboundPlan(NavMode::Setpoint);
    const int n = static_cast<int>(p.fly_point) + static_cast<int>(p.set_cruise_mode) +
                  static_cast<int>(p.set_fly_head) + static_cast<int>(p.set_fly_height);
    EXPECT_EQ(n, 1) << "上游 5 Hz → fly_point 5 Hz";
}

TEST(ModeGate, GuidanceEmitsThreePerUpstreamMessage)
{
    const auto p = outboundPlan(NavMode::Guidance);
    const int n = static_cast<int>(p.set_cruise_mode) + static_cast<int>(p.set_fly_head) +
                  static_cast<int>(p.set_fly_height);
    EXPECT_EQ(n, 3) << "上游 5 Hz → set_cruise_mode/set_fly_head/set_fly_height 各 5 Hz";
}

TEST(ModeGate, NonAutoAlwaysForwardsSomething)
{
    for (auto mode : {NavMode::Setpoint, NavMode::Guidance}) {
        EXPECT_TRUE(forwardsSetpoint(mode) || forwardsGuidance(mode));
    }
}

// ── set_cruise_mode 发送门控（现场排查开关，见 mode_gate.h）───────

TEST(ModeGate, CruiseModeGateKeepsPerFrameByDefault)
{
    // once_per_mode = false（默认，2026-09-20 裁决）：与 head/height 同频，每帧都发
    EXPECT_TRUE(shouldSendCruiseMode(false, false));
    EXPECT_TRUE(shouldSendCruiseMode(false, true));
    EXPECT_TRUE(shouldSendCruiseMode(false, true));
}

TEST(ModeGate, CruiseModeGateOncePerModeEntry)
{
    // once_per_mode = true（HIL 排查用）：每次进入 guidance 只发 1 条
    EXPECT_TRUE(shouldSendCruiseMode(true, false));
    EXPECT_FALSE(shouldSendCruiseMode(true, true));
    EXPECT_FALSE(shouldSendCruiseMode(true, true));
}

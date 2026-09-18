/**
 * @file test_mode_gate.cpp
 * @brief 出站截断矩阵单测（P6）：3 模式 × 2 通道 + 模式跃迁边沿 + 字符串容错
 */

#include <gtest/gtest.h>

#include "ros_mqtt_bridge/mode_gate.h"

using ros_mqtt_bridge::CruiseMode;
using ros_mqtt_bridge::OutChannel;
using ros_mqtt_bridge::cruiseModeName;
using ros_mqtt_bridge::isChannelEnabled;
using ros_mqtt_bridge::needsCruiseModeCommand;
using ros_mqtt_bridge::parseCruiseMode;

// ── 解析 ─────────────────────────────────────────────────────

TEST(ModeGate, ParseValid)
{
    CruiseMode m = CruiseMode::Cruise;
    ASSERT_TRUE(parseCruiseMode("auto", m));
    EXPECT_EQ(m, CruiseMode::Auto);
    ASSERT_TRUE(parseCruiseMode("setpoint", m));
    EXPECT_EQ(m, CruiseMode::Setpoint);
    ASSERT_TRUE(parseCruiseMode("cruise", m));
    EXPECT_EQ(m, CruiseMode::Cruise);
}

TEST(ModeGate, ParseTolerantCaseAndSpace)
{
    CruiseMode m = CruiseMode::Auto;
    ASSERT_TRUE(parseCruiseMode("  CRUISE ", m));
    EXPECT_EQ(m, CruiseMode::Cruise);
    ASSERT_TRUE(parseCruiseMode("SetPoint", m));
    EXPECT_EQ(m, CruiseMode::Setpoint);
    ASSERT_TRUE(parseCruiseMode("\tAUTO", m));
    EXPECT_EQ(m, CruiseMode::Auto);
}

TEST(ModeGate, ParseInvalidKeepsValue)
{
    CruiseMode m = CruiseMode::Cruise;
    EXPECT_FALSE(parseCruiseMode("", m));
    EXPECT_FALSE(parseCruiseMode("   ", m));
    EXPECT_FALSE(parseCruiseMode("manual", m));
    EXPECT_FALSE(parseCruiseMode("auto2", m));
    EXPECT_EQ(m, CruiseMode::Cruise) << "非法输入不得修改 out";
}

TEST(ModeGate, NamesRoundTrip)
{
    EXPECT_STREQ(cruiseModeName(CruiseMode::Auto), "auto");
    EXPECT_STREQ(cruiseModeName(CruiseMode::Setpoint), "setpoint");
    EXPECT_STREQ(cruiseModeName(CruiseMode::Cruise), "cruise");

    for (auto mode : {CruiseMode::Auto, CruiseMode::Setpoint, CruiseMode::Cruise}) {
        CruiseMode back = CruiseMode::Auto;
        ASSERT_TRUE(parseCruiseMode(cruiseModeName(mode), back));
        EXPECT_EQ(back, mode);
    }
}

// ── 截断矩阵 3 × 2 ───────────────────────────────────────────

TEST(ModeGate, MatrixAutoSilent)
{
    EXPECT_FALSE(isChannelEnabled(CruiseMode::Auto, OutChannel::Setpoint));
    EXPECT_FALSE(isChannelEnabled(CruiseMode::Auto, OutChannel::Guidance));
}

TEST(ModeGate, MatrixSetpointOnlySetpointChannel)
{
    EXPECT_TRUE(isChannelEnabled(CruiseMode::Setpoint, OutChannel::Setpoint));
    EXPECT_FALSE(isChannelEnabled(CruiseMode::Setpoint, OutChannel::Guidance));
}

TEST(ModeGate, MatrixCruiseOnlyGuidanceChannel)
{
    EXPECT_FALSE(isChannelEnabled(CruiseMode::Cruise, OutChannel::Setpoint));
    EXPECT_TRUE(isChannelEnabled(CruiseMode::Cruise, OutChannel::Guidance));
}

TEST(ModeGate, MatrixExactlyOneChannelNonAuto)
{
    for (auto mode : {CruiseMode::Setpoint, CruiseMode::Cruise}) {
        const int enabled = static_cast<int>(isChannelEnabled(mode, OutChannel::Setpoint)) +
                            static_cast<int>(isChannelEnabled(mode, OutChannel::Guidance));
        EXPECT_EQ(enabled, 1) << "非 auto 模式下必须恰有一路出网 (" << cruiseModeName(mode) << ")";
    }
}

// ── set_cruise_mode 边沿 ─────────────────────────────────────

TEST(ModeGate, CruiseEdgeOnlyOnEntry)
{
    EXPECT_TRUE(needsCruiseModeCommand(CruiseMode::Auto, CruiseMode::Cruise));
    EXPECT_TRUE(needsCruiseModeCommand(CruiseMode::Setpoint, CruiseMode::Cruise));
    EXPECT_FALSE(needsCruiseModeCommand(CruiseMode::Cruise, CruiseMode::Cruise)) << "同模式不重发";
}

TEST(ModeGate, NoCruiseCommandLeavingOrStandby)
{
    EXPECT_FALSE(needsCruiseModeCommand(CruiseMode::Cruise, CruiseMode::Auto));
    EXPECT_FALSE(needsCruiseModeCommand(CruiseMode::Cruise, CruiseMode::Setpoint));
    EXPECT_FALSE(needsCruiseModeCommand(CruiseMode::Auto, CruiseMode::Setpoint));
    EXPECT_FALSE(needsCruiseModeCommand(CruiseMode::Setpoint, CruiseMode::Auto));
}

/**
 * @file test_json_codec.cpp
 * @brief json_codec 单测：入站解析（真实报文样本）+ 三条新指令编码 + 基础包
 *
 * 样本取自现场 broker（2026-09-18，uav_sn = V2.4AOACRAFT-NOCONF0）。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "ros_mqtt_bridge/json_codec.h"

namespace {

// ── 现场报文样本（字段与真实报文一致）──
const char* kUavInfo =
    R"({"method":"uav_info","mid":"Px-d7-mXJ-DDz9","timestamp":1789704072000,"data":{)"
    R"("uav_sn":"V2.4AOACRAFT-NOCONF0","mode":"Auto","lock":1,"gnss_num":0,"ins_status":0,)"
    R"("voltage":47,"cmd":0,"rotate_speed":0,"longitude":104.17036,"latitude":30.352673,)"
    R"("altitude":100.0,"pitch":4.67,"roll":-27.1,"yaw":96.72,"vvg":50.23,"climb_cur":1.5}})";

const char* kUavsInfo =
    R"({"timestamp":1789704044000,"sender":"MQTT-CM","mid":"iB-96-yqA-pqSU",)"
    R"("data":{"uavs":[{"uav_sn":"V2.4AOACRAFT-NOCONF0"}]}})";

const char* kFstInfo =
    R"({"method":"fst_info","mid":"bJ-SD-NDQ-r3nU","timestamp":1789704073000,"data":{)"
    R"("barrels":[{"barrel_index":1,"fill":1,"self_check":1,"uav_sn":"V2.4AOACRAFT-NOCONF0"}]}})";

}  // namespace

// ═══════════════════════════════════════════════════════════════
// 1. 入站解析（行为须与 1.0.1 一致）
// ═══════════════════════════════════════════════════════════════

TEST(JsonCodec, DecodeUavInfo)
{
    ros_mqtt_bridge::TargetState state;
    std::string error;
    ASSERT_TRUE(ros_mqtt_bridge::decodeUavInfo(kUavInfo, state, error)) << error;

    EXPECT_EQ(state.uav_sn, "V2.4AOACRAFT-NOCONF0");
    EXPECT_EQ(state.mode, "Auto");
    EXPECT_EQ(state.mid, "Px-d7-mXJ-DDz9");
    EXPECT_EQ(state.timestamp, "1789704072000");

    EXPECT_NEAR(state.geodetic.longitude_deg, 104.17036, 1e-9);
    EXPECT_NEAR(state.geodetic.latitude_deg, 30.352673, 1e-9);
    EXPECT_NEAR(state.geodetic.altitude_m, 100.0, 1e-9);

    // 航空约定 (0°=北, CW) → ENU (0°=东, CCW)：yaw_enu = 90° − yaw_deg
    const double expected_yaw = (90.0 - 96.72) * M_PI / 180.0;
    EXPECT_NEAR(state.local.yaw, expected_yaw, 1e-9);
    EXPECT_NEAR(state.local.roll, -27.1 * M_PI / 180.0, 1e-9);
    EXPECT_NEAR(state.local.pitch, 4.67 * M_PI / 180.0, 1e-9);

    EXPECT_NEAR(state.ground_speed_mps, 50.23, 1e-9);
    EXPECT_NEAR(state.up_mps, 1.5, 1e-9);
}

TEST(JsonCodec, DecodeUavInfoRejectsWrongMethod)
{
    ros_mqtt_bridge::TargetState state;
    std::string error;
    const std::string payload =
        R"({"method":"fst_info","mid":"x","timestamp":1,"data":{"uav_sn":"A",)"
        R"("longitude":1,"latitude":1,"altitude":1}})";
    EXPECT_FALSE(ros_mqtt_bridge::decodeUavInfo(payload, state, error));
    EXPECT_FALSE(error.empty());
}

TEST(JsonCodec, DecodeUavsAndFstInfo)
{
    std::vector<std::string> list;
    std::string error;
    ASSERT_TRUE(ros_mqtt_bridge::decodeUavsInfo(kUavsInfo, list, error)) << error;
    ASSERT_EQ(list.size(), 1u);
    EXPECT_EQ(list[0], "V2.4AOACRAFT-NOCONF0");

    list.clear();
    ASSERT_TRUE(ros_mqtt_bridge::decodeFstInfo(kFstInfo, list, error)) << error;
    ASSERT_EQ(list.size(), 1u);
    EXPECT_EQ(list[0], "V2.4AOACRAFT-NOCONF0");
}

// ═══════════════════════════════════════════════════════════════
// 2. 出站编码：基础包 + 三条新指令（协议 4.4.17 / 4.4.19 / 4.4.21）
// ═══════════════════════════════════════════════════════════════

TEST(JsonCodec, MidGeneratorIsUniqueAndFormatted)
{
    ros_mqtt_bridge::MidGenerator gen;
    const std::string a = gen.next();
    const std::string b = gen.next();
    EXPECT_NE(a, b);
    EXPECT_EQ(a.rfind("uavg-", 0), 0u);   // 前缀
    EXPECT_NE(a.find("-"), std::string::npos);
}

TEST(JsonCodec, SetCruiseModeHasEmptyData)
{
    ros_mqtt_bridge::MidGenerator gen;
    const std::string payload = ros_mqtt_bridge::encodeSetCruiseMode(gen.next());
    EXPECT_NE(payload.find("\"method\": \"set_cruise_mode\""), std::string::npos);
    EXPECT_NE(payload.find("\"data\": {}"), std::string::npos);
    EXPECT_NE(payload.find("\"timestamp\""), std::string::npos);
    EXPECT_NE(payload.find("\"mid\""), std::string::npos);
}

TEST(JsonCodec, SetFlyHeadRangesAndRounding)
{
    ros_mqtt_bridge::MidGenerator gen;

    const std::string normal = ros_mqtt_bridge::encodeSetFlyHead(45.34, gen.next());
    EXPECT_NE(normal.find("\"method\": \"set_fly_head\""), std::string::npos);
    EXPECT_NE(normal.find("\"heading\":45.3"), std::string::npos);   // 1 位小数

    // 归一到 [0,360)
    EXPECT_NE(ros_mqtt_bridge::encodeSetFlyHead(365.0, gen.next()).find("\"heading\":5.0"),
              std::string::npos);
    EXPECT_NE(ros_mqtt_bridge::encodeSetFlyHead(-10.0, gen.next()).find("\"heading\":350.0"),
              std::string::npos);
}

TEST(JsonCodec, SetFlyHeightRoundsAndCarriesType)
{
    ros_mqtt_bridge::MidGenerator gen;

    const std::string ground = ros_mqtt_bridge::encodeSetFlyHeight(451.4, 0, gen.next());
    EXPECT_NE(ground.find("\"method\": \"set_fly_height\""), std::string::npos);
    EXPECT_NE(ground.find("\"height\":451"), std::string::npos);
    EXPECT_NE(ground.find("\"height_type\":0"), std::string::npos);

    const std::string absolute = ros_mqtt_bridge::encodeSetFlyHeight(599.6, 1, gen.next());
    EXPECT_NE(absolute.find("\"height\":600"), std::string::npos);   // 四舍五入
    EXPECT_NE(absolute.find("\"height_type\":1"), std::string::npos);
}

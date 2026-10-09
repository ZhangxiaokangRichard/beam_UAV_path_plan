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

    // 量化回绕：359.96 不得舍入成 360.0（超出 [0,360)）；359.94 应保留 359.9
    EXPECT_NE(ros_mqtt_bridge::encodeSetFlyHead(359.96, gen.next()).find("\"heading\":0.0"),
              std::string::npos);
    EXPECT_EQ(ros_mqtt_bridge::encodeSetFlyHead(359.96, gen.next()).find("360.0"),
              std::string::npos) << "heading 不得出现 360.0";
    EXPECT_NE(ros_mqtt_bridge::encodeSetFlyHead(359.94, gen.next()).find("\"heading\":359.9"),
              std::string::npos);
    EXPECT_NE(ros_mqtt_bridge::encodeSetFlyHead(360.0, gen.next()).find("\"heading\":0.0"),
              std::string::npos);
    EXPECT_NE(ros_mqtt_bridge::encodeSetFlyHead(-0.04, gen.next()).find("\"heading\":0.0"),
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

// ═══════════════════════════════════════════════════════════════
// 3. 出站显示通道（aoa/ai_guide/display/*）
// ═══════════════════════════════════════════════════════════════

TEST(JsonCodec, DisplayPoseDataShape)
{
    ros_mqtt_bridge::DisplayPose p;
    p.uav_sn = "V2.4AOACRAFT-NOCONF0";
    p.role = "own";
    p.frame = "map";
    p.mode = "Auto";
    p.x = 1234.5;
    p.y = -678.9;
    p.z = 207.1;
    p.qx = 0.0;
    p.qy = 0.0;
    p.qz = 0.12;
    p.qw = 0.992756;
    p.vx = 50.1;
    p.vy = 1.2;
    p.vz = 0.0;
    p.include_geodetic = true;
    p.longitude_deg = 104.9271017;
    p.latitude_deg = 31.5934765;
    p.altitude_m = 307.1;

    const std::string data = ros_mqtt_bridge::encodeDisplayPoseData(p);

    EXPECT_NE(data.find("\"uav_sn\":\"V2.4AOACRAFT-NOCONF0\""), std::string::npos);
    EXPECT_NE(data.find("\"role\":\"own\""), std::string::npos);
    EXPECT_NE(data.find("\"frame\":\"map\""), std::string::npos);
    EXPECT_NE(data.find("\"mode\":\"Auto\""), std::string::npos);
    // position/orientation/linear 平铺在 data 下
    EXPECT_NE(data.find("\"position\":{\"x\":1234.500,\"y\":-678.900,\"z\":207.100}"),
              std::string::npos);
    EXPECT_NE(data.find("\"orientation\":{\"x\":0.000000,\"y\":0.000000,\"z\":0.120000"),
              std::string::npos);
    EXPECT_NE(data.find("\"linear\":{\"x\":50.100,\"y\":1.200,\"z\":0.000}"), std::string::npos);
    // 经纬 7 位小数
    EXPECT_NE(data.find("\"longitude\":104.9271017"), std::string::npos);
    EXPECT_NE(data.find("\"latitude\":31.5934765"), std::string::npos);
    EXPECT_NE(data.find("\"altitude\":307.10"), std::string::npos);
    EXPECT_EQ(data.front(), '{');
    EXPECT_EQ(data.back(), '}');
}

TEST(JsonCodec, DisplayPoseOptionalGeodeticAndEscaping)
{
    ros_mqtt_bridge::DisplayPose p;
    p.uav_sn = "SN\"with\\quote";
    p.include_geodetic = false;

    const std::string data = ros_mqtt_bridge::encodeDisplayPoseData(p);
    // 未开启 geodetic → 整段不出现
    EXPECT_EQ(data.find("geodetic"), std::string::npos);
    // 字符串必须转义：SN"with\quote → SN\"with\\quote
    EXPECT_NE(data.find("\"uav_sn\":\"SN\\\"with\\\\quote\""), std::string::npos);
}

TEST(JsonCodec, DisplayPathDataShape)
{
    ros_mqtt_bridge::DisplayPath d;
    d.uav_sn = "V2.4AOACRAFT-NOCONF0";
    d.frame = "map";
    d.success = true;
    d.cost = 3574.68;
    d.status_message = "goal reached";
    d.points = {{0.0, 0.0, 100.0, 0.0, 0.0, 0.0},
                {10.5, -2.25, 101.5, 1.57, 0.01, 0.002}};

    const std::string data = ros_mqtt_bridge::encodeDisplayPathData(d);
    EXPECT_NE(data.find("\"success\":true"), std::string::npos);
    EXPECT_NE(data.find("\"cost\":3574.680"), std::string::npos);
    EXPECT_NE(data.find("\"status_message\":\"goal reached\""), std::string::npos);
    EXPECT_NE(data.find("\"count\":2"), std::string::npos);
    EXPECT_NE(data.find("\"path\":[[0,0,100,0,0,0],[10.5,-2.25,101.5,1.57,0.01,0.002]]"),
              std::string::npos);
}

TEST(JsonCodec, DisplayPathEscapesStatusMessage)
{
    ros_mqtt_bridge::DisplayPath d;
    d.status_message = "say \"hi\"\tnow";
    const std::string data = ros_mqtt_bridge::encodeDisplayPathData(d);
    EXPECT_NE(data.find("\"status_message\":\"say \\\"hi\\\"\\tnow\""), std::string::npos);
}

TEST(JsonCodec, DisplayPackageCarriesBaseFields)
{
    ros_mqtt_bridge::MidGenerator gen;
    ros_mqtt_bridge::DisplayPath d;
    d.points = {{1.0, 2.0, 3.0, 0.0, 0.0, 0.0}};
    const std::string mid = gen.next();
    const std::string pkg =
        ros_mqtt_bridge::makeBasePackage("planed_path", ros_mqtt_bridge::encodeDisplayPathData(d), mid);

    EXPECT_NE(pkg.find("\"method\": \"planed_path\""), std::string::npos);
    EXPECT_NE(pkg.find("\"mid\": \"" + mid + "\""), std::string::npos);
    EXPECT_NE(pkg.find("\"timestamp\": "), std::string::npos);
    EXPECT_NE(pkg.find("\"data\": {"), std::string::npos);
}

TEST(JsonCodec, DecimatePathKeepsEndpoints)
{
    std::vector<std::array<double, 6>> pts;
    for (int i = 0; i < 100; ++i) pts.push_back({static_cast<double>(i), 0, 0, 0, 0, 0});

    // max=0 或不超过上限 → 原样返回
    EXPECT_EQ(ros_mqtt_bridge::decimatePath(pts, 0).size(), 100u);
    EXPECT_EQ(ros_mqtt_bridge::decimatePath(pts, 100).size(), 100u);

    const auto d10 = ros_mqtt_bridge::decimatePath(pts, 10);
    ASSERT_EQ(d10.size(), 10u);
    EXPECT_EQ(d10.front()[0], 0.0);
    EXPECT_EQ(d10.back()[0], 99.0);        // 末点必须保留
    EXPECT_EQ(d10[1][0], 11.0);            // (1*99)/9 = 11

    const auto d1 = ros_mqtt_bridge::decimatePath(pts, 1);
    ASSERT_EQ(d1.size(), 1u);
    EXPECT_EQ(d1.front()[0], 0.0);
}

// ═══════════════════════════════════════════════════════════════
// 4. 载荷健壮性（2026-10-09 现场回归：列表含非法 UTF-8 字节）
//    现场 aoa/uav_center/uavs_information 中某项 uav_sn 为 20 字节 0xFF，
//    曾使 boost::property_tree 抛 "invalid code sequence" →
//    **整条列表被丢弃** → tracker 的 uav_sn 为空 → aoa/uav/state、
//    aoa/target/state 断流。
// ═══════════════════════════════════════════════════════════════

TEST(JsonCodec, UavsInfoToleratesInvalidUtf8Sn)
{
    const std::string garbage(20, '\xff');   // 与现场抓包一致
    const std::string payload =
        std::string("{\"timestamp\":1791515060000,\"sender\":\"MQTT-CM\","
                    "\"mid\":\"8z-Ox-8IU-CQp4\",") +
        "\"data\":{\"uavs\":[{\"uav_sn\":\"V2.4AOACRAFT-NOCONF0\"}," +
        "{\"uav_sn\":\"V2.4AOACRAFT-SIL3NF0\"},{\"uav_sn\":\"" + garbage + "\"}]}}";

    std::vector<std::string> list;
    std::string err;
    ASSERT_TRUE(ros_mqtt_bridge::decodeUavsInfo(payload, list, err)) << err;
    ASSERT_EQ(list.size(), 3u);
    // 正常项必须逐字节不变，否则 own/target filter 匹配会失效
    EXPECT_EQ(list[0], "V2.4AOACRAFT-NOCONF0");
    EXPECT_EQ(list[1], "V2.4AOACRAFT-SIL3NF0");
    EXPECT_NE(list[1].find("SIL"), std::string::npos);
    // 非法字节逐个替换为 '?'
    EXPECT_EQ(list[2], std::string(20, '?'));
}

TEST(JsonCodec, UavsInfoStillFailsWithoutSn)
{
    // 合法 JSON 但没有 uav_sn → 仍须失败（不得退化为“空列表也算成功”）
    const std::string payload = R"({"data":{"uavs":[{"foo":1}]}})";
    std::vector<std::string> list;
    std::string err;
    EXPECT_FALSE(ros_mqtt_bridge::decodeUavsInfo(payload, list, err));
    EXPECT_FALSE(err.empty());
}

TEST(JsonCodec, UavsInfoTolerantFallbackOnBrokenJson)
{
    // 结构损坏（缺尾部括号）→ 严格解析失败，退化为扫描 uav_sn
    const std::string payload = R"({"data":{"uavs":[{"uav_sn":"A-1"},{"uav_sn":"B-2"})";
    std::vector<std::string> list;
    std::string err;
    ASSERT_TRUE(ros_mqtt_bridge::decodeUavsInfo(payload, list, err)) << err;
    ASSERT_EQ(list.size(), 2u);
    EXPECT_EQ(list[0], "A-1");
    EXPECT_EQ(list[1], "B-2");
}

TEST(JsonCodec, FstInfoToleratesInvalidUtf8AndBrokenJson)
{
    const std::string garbage(16, '\xfe');
    const std::string bad =
        std::string("{\"method\":\"fst_info\",\"data\":{\"barrels\":[{\"barrel_index\":1,") +
        "\"uav_sn\":\"OK-1\"},{\"barrel_index\":2,\"uav_sn\":\"" + garbage + "\"}]}}";

    std::vector<std::string> list;
    std::string err;
    ASSERT_TRUE(ros_mqtt_bridge::decodeFstInfo(bad, list, err)) << err;
    ASSERT_EQ(list.size(), 2u);
    EXPECT_EQ(list[0], "OK-1");
    EXPECT_EQ(list[1], std::string(16, '?'));

    // 结构损坏 → 扫描兜底
    std::vector<std::string> list2;
    EXPECT_TRUE(ros_mqtt_bridge::decodeFstInfo(R"({"data":{"barrels":[{"uav_sn":"C-3"})", list2, err));
    ASSERT_EQ(list2.size(), 1u);
    EXPECT_EQ(list2[0], "C-3");
}

// ═══════════════════════════════════════════════════════════════
// 5. 拦截模式（迎头/尾追）入站翻译
//    MQTT aoa/ai_guide/intercept/mode → ROS aoa/uav/intercept_mode_cmd
//    下游 uav_guide_loop_node 只接受 head-on | tail
// ═══════════════════════════════════════════════════════════════

TEST(JsonCodec, NormalizeInterceptModeAcceptsAliases)
{
    using ros_mqtt_bridge::normalizeInterceptMode;
    EXPECT_EQ(normalizeInterceptMode("head-on"), "head-on");
    EXPECT_EQ(normalizeInterceptMode("headon"), "head-on");
    EXPECT_EQ(normalizeInterceptMode("head_on"), "head-on");
    EXPECT_EQ(normalizeInterceptMode("HEAD-ON"), "head-on");     // 大小写不敏感
    EXPECT_EQ(normalizeInterceptMode("  Tail  "), "tail");        // 容忍空白
    EXPECT_EQ(normalizeInterceptMode("TAIL"), "tail");
    // 非法值 → 空串（调用方据此拒发，而不是发脏数据给下游）
    EXPECT_EQ(normalizeInterceptMode(""), "");
    EXPECT_EQ(normalizeInterceptMode("head"), "");
    EXPECT_EQ(normalizeInterceptMode("pursuit"), "");
}

TEST(JsonCodec, ExtractInterceptModeFromBareAndJsonForms)
{
    using ros_mqtt_bridge::extractInterceptMode;
    // 裸串
    EXPECT_EQ(extractInterceptMode("tail"), "tail");
    EXPECT_EQ(extractInterceptMode("  head-on\n"), "head-on");
    // JSON 顶层
    EXPECT_EQ(extractInterceptMode(R"({"mode":"tail"})"), "tail");
    EXPECT_EQ(extractInterceptMode(R"({"intercept_mode":"headon"})"), "head-on");
    // JSON 嵌套（data 包裹）
    EXPECT_EQ(extractInterceptMode(R"({"data":{"mode":"TAIL"}})"), "tail");
    // intercept_mode 优先于 mode
    EXPECT_EQ(extractInterceptMode(R"({"mode":"tail","intercept_mode":"head-on"})"), "head-on");
    // 非法 / 无该字段
    EXPECT_EQ(extractInterceptMode(R"({"mode":"guidance"})"), "");
    EXPECT_EQ(extractInterceptMode(R"({"foo":1})"), "");
    EXPECT_EQ(extractInterceptMode(""), "");
    EXPECT_EQ(extractInterceptMode("   "), "");
}

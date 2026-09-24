/**
 * @file test_uav_guide.cpp
 * @brief uav_guide 算法层单元测试（零 ROS；gtest）
 *
 * 覆盖：几何/航向换算、WGS84↔ENU 往返、路径切分与采样点检索、
 *      指点几何（圆心/偏移点）、巡航制导（heads/高度）、APPROACH/TRACKING 状态机。
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "uav_guide/geo.hpp"
#include "uav_guide/guidance.hpp"
#include "uav_guide/orchestrator.hpp"
#include "uav_guide/path.hpp"
#include "uav_guide/setpoint.hpp"
#include "uav_guide/state.hpp"
#include "uav_guide/types.hpp"

namespace {

using uav_guide::GuidanceConfig;
using uav_guide::HeadsSource;
using uav_guide::HeightSource;
using uav_guide::InterceptMode;
using uav_guide::OrchestratorConfig;
using uav_guide::PathPoint;
using uav_guide::Phase;
using uav_guide::SampleRule;
using uav_guide::SetpointConfig;
using uav_guide::State5;

// 测试用地理原点（与 uav_guide.yaml 一致）
const uav_guide::GeoOrigin kOrigin{31.58461930, 104.88313540, 515.0};

PathPoint make_point(double x, double y, double z = 100.0, double yaw = 0.0, double k = 0.0)
{
    PathPoint p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.yaw = yaw;
    p.pitch = 0.0;
    p.curvature = k;
    return p;
}

/// 沿 +x（东）直线：n 点，间隔 step
std::vector<PathPoint> straight_east(int n, double step = 100.0, double z = 100.0)
{
    std::vector<PathPoint> path;
    for (int i = 0; i < n; ++i) {
        path.push_back(make_point(static_cast<double>(i) * step, 0.0, z, 0.0, 0.0));
    }
    return path;
}

/// 先左转弧（曲率 1/500，yaw 递增）后直线：前 5 点为弧，后 5 点为直线
std::vector<PathPoint> arc_then_straight()
{
    std::vector<PathPoint> path;
    for (int i = 0; i < 5; ++i) {
        const double yaw = 0.1 * static_cast<double>(i);
        path.push_back(make_point(static_cast<double>(i) * 50.0, static_cast<double>(i) * 5.0, 100.0,
                                  yaw, 1.0 / 500.0));
    }
    for (int i = 5; i < 10; ++i) {
        const double yaw = 0.1 * 4.0;   // 与弧末端航向一致 → 判为直线段
        path.push_back(make_point(static_cast<double>(i) * 50.0, 20.0 + static_cast<double>(i - 5) * 50.0,
                                  100.0, yaw, 0.0));
    }
    return path;
}

}  // namespace

// ═══════════════════════════════════════════════════════════════
// 1. geo：角度与航向换算
// ═══════════════════════════════════════════════════════════════

TEST(Geo, WrapAngle)
{
    const double pi = uav_guide::geo::kPi;
    EXPECT_NEAR(uav_guide::geo::wrap_angle(0.0), 0.0, 1e-12);
    EXPECT_NEAR(uav_guide::geo::wrap_angle(pi + 0.5), -pi + 0.5, 1e-9);
    EXPECT_NEAR(std::abs(uav_guide::geo::wrap_angle(3.0 * pi)), pi, 1e-9);
}

TEST(Geo, HeadingConversion)
{
    // ENU(0=东, CCW) → 航空(0=北, CW)
    EXPECT_NEAR(uav_guide::geo::enu_yaw_to_heading_deg(0.0), 90.0, 1e-9);              // 东 → 90°
    EXPECT_NEAR(uav_guide::geo::enu_yaw_to_heading_deg(uav_guide::geo::kPi / 2.0), 0.0, 1e-9);   // 北 → 0°
    EXPECT_NEAR(uav_guide::geo::enu_yaw_to_heading_deg(-uav_guide::geo::kPi / 2.0), 180.0, 1e-9); // 南 → 180°
    EXPECT_NEAR(uav_guide::geo::enu_yaw_to_heading_deg(uav_guide::geo::kPi), 270.0, 1e-9);       // 西 → 270°

    // 往返一致
    for (double deg : {0.0, 45.0, 90.0, 179.9, 270.0, 359.0}) {
        const double yaw = uav_guide::geo::heading_deg_to_enu_yaw(deg);
        EXPECT_NEAR(uav_guide::geo::enu_yaw_to_heading_deg(yaw), deg, 1e-6) << "deg=" << deg;
    }
    // 输出范围 [0,360)
    const double h = uav_guide::geo::enu_yaw_to_heading_deg(2.0);
    EXPECT_GE(h, 0.0);
    EXPECT_LT(h, 360.0);
}

TEST(Geo, GeodeticEnuRoundTrip)
{
    const uav_guide::geo::Geodetic p{31.59461930, 104.89313540, 600.0};   // 约 +1.1 km 北/东
    const auto enu = uav_guide::geo::to_enu(kOrigin, p);
    EXPECT_GT(enu.north, 1000.0);   // 纬度增大 → 北向为正
    EXPECT_GT(enu.east, 800.0);     // 经度增大 → 东向为正
    EXPECT_NEAR(enu.up, 85.0, 1.0); // 高度差（515 → 600）

    const auto back = uav_guide::geo::to_geodetic(kOrigin, enu);
    EXPECT_NEAR(back.lat_deg, p.lat_deg, 1e-8);
    EXPECT_NEAR(back.lon_deg, p.lon_deg, 1e-8);
    EXPECT_NEAR(back.alt_m, p.alt_m, 1e-3);
}

// ═══════════════════════════════════════════════════════════════
// 2. path：切分、最近点、采样点检索
// ═══════════════════════════════════════════════════════════════

TEST(Path, SegmentStraightIsLine)
{
    const auto path = straight_east(10);
    const auto segs = uav_guide::path::segment(path);
    ASSERT_EQ(segs.size(), 1u);
    EXPECT_EQ(segs[0].kind, uav_guide::path::Kind::Line);
    EXPECT_EQ(segs[0].start_index, 0u);
    EXPECT_EQ(segs[0].end_index, 9u);
}

TEST(Path, SegmentArcDetection)
{
    const auto path = arc_then_straight();
    const auto segs = uav_guide::path::segment(path);
    ASSERT_GE(segs.size(), 2u);
    EXPECT_EQ(segs[0].kind, uav_guide::path::Kind::Arc);
    EXPECT_TRUE(segs[0].is_left);                       // yaw 递增 → 左转
    EXPECT_NEAR(segs[0].radius_m, 500.0, 1e-6);         // 曲率 1/500
    EXPECT_EQ(segs.back().kind, uav_guide::path::Kind::Line);
}

TEST(Path, ClosestIndexAndSamplePoint)
{
    const auto path = arc_then_straight();
    EXPECT_EQ(uav_guide::path::closest_index(path, 0.0, 0.0), 0u);
    EXPECT_EQ(uav_guide::path::closest_index(path, 300.0, 25.0), 6u);

    // 默认规则：自 uav 起首个**曲率为 0** 的点（弧段 0..4 之后）
    EXPECT_EQ(uav_guide::path::find_sample_index(path, 0, SampleRule::FirstZeroCurvature, 1e-4), 5u);
    // 旧规则：首个曲率非 0 的点
    EXPECT_EQ(uav_guide::path::find_sample_index(path, 0, SampleRule::FirstNonzeroCurvature, 1e-4), 0u);
    // 全弧段 → 无命中（返回 size）
    std::vector<PathPoint> all_arc;
    for (int i = 0; i < 5; ++i) all_arc.push_back(make_point(i * 10.0, 0.0, 100.0, 0.1 * i, 1.0 / 500.0));
    EXPECT_EQ(uav_guide::path::find_sample_index(all_arc, 0, SampleRule::FirstZeroCurvature, 1e-4),
              all_arc.size());
}

TEST(Path, FarthestRemainingArc)
{
    const auto path = arc_then_straight();
    const auto segs = uav_guide::path::segment(path);

    const auto arc_from_start = uav_guide::path::farthest_remaining_arc(segs, 0);
    ASSERT_TRUE(arc_from_start.has_value());
    EXPECT_EQ(arc_from_start->start_index, 0u);

    // 已飞过弧段（最近点索引 6 > arc.end_index 4）→ 无剩余弧
    EXPECT_FALSE(uav_guide::path::farthest_remaining_arc(segs, 6).has_value());
}

// ═══════════════════════════════════════════════════════════════
// 3. setpoint：偏移点与圆心引导
// ═══════════════════════════════════════════════════════════════

TEST(Setpoint, OffsetGoalDirection)
{
    const State5 target = uav_guide::state::make(0.0, 0.0, 100.0, 0.0);   // 航向朝东

    double x = 0.0;
    double y = 0.0;
    uav_guide::setpoint::offset_goal_xy(target, InterceptMode::HeadOn, 50.0, 750.0, x, y);
    EXPECT_NEAR(x, -(50.0 + 0.4 * 750.0), 1e-9);   // head-on：目标**身后**
    EXPECT_NEAR(y, 0.0, 1e-9);

    uav_guide::setpoint::offset_goal_xy(target, InterceptMode::Tail, 50.0, 750.0, x, y);
    EXPECT_NEAR(x, 50.0 + 750.0, 1e-9);            // tail：目标**前方**
    EXPECT_NEAR(y, 0.0, 1e-9);
}

TEST(Setpoint, StraightPathFallsBackToOffsetGuide)
{
    const SetpointConfig cfg;   // 默认：offset_radius_m = 10
    const auto path = straight_east(10);
    const State5 uav = uav_guide::state::make(0.0, 0.0, 100.0, 0.0);
    const State5 target = uav_guide::state::make(1000.0, 0.0, 600.0, 0.0);

    const auto out = uav_guide::setpoint::build(path, uav, target, InterceptMode::Tail, kOrigin, cfg);
    EXPECT_NEAR(out.radius_m, cfg.offset_radius_m, 1e-9);

    // tail 模式：偏移点在目标**前方**（东侧）→ 经度大于目标经度
    const auto target_geo =
        uav_guide::geo::to_geodetic(kOrigin, uav_guide::geo::EnuPoint{target.x, target.y, target.z});
    EXPECT_GT(out.longitude_deg, target_geo.lon_deg);
    EXPECT_NEAR(out.latitude_deg, target_geo.lat_deg, 1e-6);
    EXPECT_GT(out.altitude_m, 500.0);                      // WGS84 高度
}

TEST(Setpoint, ArcCenterGuideWithSignedRadius)
{
    const SetpointConfig cfg;
    const auto path = arc_then_straight();
    const State5 uav = uav_guide::state::make(0.0, 0.0, 100.0, 0.0);
    const State5 target = uav_guide::state::make(1000.0, 0.0, 600.0, 0.0);

    const auto out = uav_guide::setpoint::build(path, uav, target, InterceptMode::Tail, kOrigin, cfg);
    // 左转弧 → 半径取负（与 1.0.1 的 _signed_radius 一致），大小 = 500
    EXPECT_NEAR(out.radius_m, -500.0, 1e-6);
}

// ═══════════════════════════════════════════════════════════════
// 4. guidance：采样点 → heads / 高度
// ═══════════════════════════════════════════════════════════════

TEST(Guidance, StraightEastGivesHeading90)
{
    GuidanceConfig cfg;
    cfg.fly_speed_mps = 42.0;
    const auto path = straight_east(10, 100.0, 250.0);
    const State5 uav = uav_guide::state::make(-100.0, 0.0, 250.0, 0.0);
    const State5 target = uav_guide::state::make(2000.0, 0.0, 300.0, 0.0);

    const auto out = uav_guide::guidance::build(path, uav, target, cfg);
    EXPECT_NEAR(out.heads_deg, 90.0, 1e-6);   // 东 → 90°
    EXPECT_NEAR(out.fly_height, 250.0, 1e-9); // 采样点 z
    EXPECT_NEAR(out.fly_speed_mps, 42.0, 1e-9);
    EXPECT_FALSE(out.sample_fallback);
}

TEST(Guidance, StraightNorthGivesHeading0)
{
    GuidanceConfig cfg;
    std::vector<PathPoint> path;
    for (int i = 0; i < 5; ++i) {
        path.push_back(make_point(0.0, static_cast<double>(i) * 100.0, 200.0,
                                  uav_guide::geo::kPi / 2.0, 0.0));
    }
    const State5 uav = uav_guide::state::make(0.0, -100.0, 200.0, uav_guide::geo::kPi / 2.0);
    const State5 target = uav_guide::state::make(0.0, 1000.0, 200.0, uav_guide::geo::kPi / 2.0);

    const auto out = uav_guide::guidance::build(path, uav, target, cfg);
    EXPECT_NEAR(out.heads_deg, 0.0, 1e-6);   // 北 → 0°
}

TEST(Guidance, FirstZeroCurvaturePointSelected)
{
    GuidanceConfig cfg;
    const auto path = arc_then_straight();
    const State5 uav = uav_guide::state::make(0.0, 0.0, 100.0, 0.0);
    const State5 target = uav_guide::state::make(1000.0, 0.0, 600.0, 0.0);

    const auto out = uav_guide::guidance::build(path, uav, target, cfg);
    EXPECT_EQ(out.sample_index, 5u);           // 首个曲率为 0 的点
    EXPECT_NEAR(out.fly_height, 100.0, 1e-9);  // 采样点 z
    EXPECT_NEAR(out.heads_deg, uav_guide::geo::enu_yaw_to_heading_deg(0.4), 1e-6);
    EXPECT_FALSE(out.sample_fallback);
}

TEST(Guidance, AllArcFallsBackToEndPoint)
{
    GuidanceConfig cfg;
    std::vector<PathPoint> all_arc;
    for (int i = 0; i < 6; ++i) {
        all_arc.push_back(make_point(i * 10.0, 0.0, 150.0, 0.1 * i, 1.0 / 500.0));
    }
    const State5 uav = uav_guide::state::make(0.0, 0.0, 150.0, 0.0);
    const State5 target = uav_guide::state::make(1000.0, 0.0, 150.0, 0.0);

    const auto out = uav_guide::guidance::build(all_arc, uav, target, cfg);
    EXPECT_TRUE(out.sample_fallback);
    EXPECT_EQ(out.sample_index, all_arc.size() - 1);
}

TEST(Guidance, BearingToSampleSource)
{
    GuidanceConfig cfg;
    cfg.heads_source = HeadsSource::BearingToSample;
    cfg.height_source = HeightSource::TargetZ;
    const auto path = straight_east(5, 100.0, 250.0);
    const State5 uav = uav_guide::state::make(0.0, -100.0, 250.0, 0.0);   // 采样点正北方向
    const State5 target = uav_guide::state::make(1000.0, 0.0, 333.0, 0.0);

    const auto out = uav_guide::guidance::build(path, uav, target, cfg);
    EXPECT_NEAR(out.heads_deg, 0.0, 1e-6);      // 本机 → 采样点指向正北
    EXPECT_NEAR(out.fly_height, 333.0, 1e-9);   // 高度取目标 z
}

// ═══════════════════════════════════════════════════════════════
// 4b. L1 路径跟踪（heads_source = l1；设计见 doc/L1_tracking_plan.md）
// ═══════════════════════════════════════════════════════════════

/// 沿 +x 的直线路径（y=0），总长 length_m，点距 step（默认取现场实测的 91.559 m 大点距）
std::vector<PathPoint> l1_straight(double length_m, double step = 91.559)
{
    std::vector<PathPoint> path;
    for (double x = 0.0; x < length_m - 1e-9; x += step) {
        path.push_back(make_point(x, 0.0, 100.0, 0.0, 0.0));
    }
    path.push_back(make_point(length_m, 0.0, 100.0, 0.0, 0.0));
    return path;
}

TEST(PathGeometry, ProjectInterpolatesInsideSegment)
{
    const auto path = l1_straight(1000.0, 100.0);      // 点距 100 m
    const auto p = uav_guide::path::project(path, 350.0, 30.0);
    ASSERT_TRUE(p.valid);
    EXPECT_EQ(p.seg_index, 3u);                        // 段 [300,400]
    EXPECT_NEAR(p.t, 0.5, 1e-9);
    EXPECT_NEAR(p.x, 350.0, 1e-9);
    EXPECT_NEAR(p.y, 0.0, 1e-9);
    EXPECT_NEAR(p.s_m, 350.0, 1e-9);
    EXPECT_NEAR(p.distance_m, 30.0, 1e-9);
}

TEST(PathGeometry, ProjectClampsBeyondBothEnds)
{
    const auto path = l1_straight(1000.0, 100.0);
    const auto front = uav_guide::path::project(path, -250.0, 0.0);
    EXPECT_NEAR(front.x, 0.0, 1e-9);
    EXPECT_NEAR(front.s_m, 0.0, 1e-9);

    const auto back = uav_guide::path::project(path, 1500.0, 0.0);
    EXPECT_NEAR(back.x, 1000.0, 1e-9);
    EXPECT_NEAR(back.s_m, 1000.0, 1e-9);
}

TEST(PathGeometry, ArclengthLookupInterpolatesAndClamps)
{
    const auto path = l1_straight(1000.0, 100.0);
    EXPECT_NEAR(uav_guide::path::length(path), 1000.0, 1e-9);

    std::size_t idx = 99u;
    bool clamped = true;
    const auto mid = uav_guide::path::point_at_arclength(path, 250.0, &idx, &clamped);
    EXPECT_NEAR(mid.x, 250.0, 1e-9);
    EXPECT_EQ(idx, 2u);
    EXPECT_FALSE(clamped);

    const auto endp = uav_guide::path::point_at_arclength(path, 5000.0, &idx, &clamped);
    EXPECT_NEAR(endp.x, 1000.0, 1e-9);
    EXPECT_TRUE(clamped);
}

TEST(L1, LookaheadIsConstantAlongPath)
{
    GuidanceConfig cfg;                                 // 默认 L1=1000 / R_c=500 / T_lead=1 / V=50
    const auto path = l1_straight(5000.0);

    for (const double x : {0.0, 137.0, 900.0, 2500.0}) {
        const State5 uav = uav_guide::state::make(x, 0.0, 100.0, 0.0);
        const auto sol = uav_guide::guidance::solve_l1(path, uav, cfg);
        ASSERT_TRUE(sol.ok);
        ASSERT_FALSE(sol.guarded);
        // 核心性质：参考点沿弧长恒定在前方 L1 处（插值后不随点距跳变）
        EXPECT_NEAR(sol.l1_eff_m, 1000.0, 1e-9);
        EXPECT_NEAR(sol.ref.x, x + 1000.0, 1e-6);
        EXPECT_NEAR(sol.beta_deg, 0.0, 1e-6);
        EXPECT_NEAR(sol.kappa_cmd, 0.0, 1e-12);
    }
}

TEST(L1, LateralOffsetCommandsTurnTowardPath)
{
    GuidanceConfig cfg;
    cfg.heads_source = HeadsSource::L1;
    const auto path = l1_straight(5000.0);
    const State5 uav = uav_guide::state::make(0.0, 200.0, 100.0, 0.0);   // 路径北侧 200 m

    const auto sol = uav_guide::guidance::solve_l1(path, uav, cfg);
    ASSERT_TRUE(sol.ok);
    ASSERT_FALSE(sol.guarded);

    const double beta = std::atan2(-200.0, 1000.0);
    EXPECT_NEAR(sol.beta_rad, beta, 1e-9);
    EXPECT_LT(sol.kappa_cmd, 0.0);                      // 右转回路径
    EXPECT_NEAR(sol.kappa_cmd, 2.0 * std::sin(beta) / 1000.0, 1e-12);
    EXPECT_FALSE(sol.saturated);

    const State5 target = uav_guide::state::make(0.0, 0.0, 0.0, 0.0);
    const auto out = uav_guide::guidance::build(path, uav, target, cfg);
    EXPECT_GT(out.heads_deg, 90.0);                     // 从正东右转 → 罗盘航向 > 90°
    EXPECT_NEAR(out.heads_deg, uav_guide::geo::enu_yaw_to_heading_deg(sol.psi_cmd_enu), 1e-9);
}

TEST(L1, CurvatureSaturatesAtTurnRadius)
{
    GuidanceConfig cfg;
    cfg.l1_distance_m = 500.0;                          // < 2·R_c ⇒ 存在可饱和区间
    cfg.turn_radius_m = 500.0;
    const auto path = l1_straight(5000.0);

    // 航向偏出路径 60°：κ_ideal = 2·sin60°/500 = 0.00346 > 1/R_c = 0.002 → 饱和
    const double yaw_enu = -60.0 * uav_guide::geo::kPi / 180.0;
    const State5 uav = uav_guide::state::make(0.0, 0.0, 100.0, yaw_enu);

    const auto sol = uav_guide::guidance::solve_l1(path, uav, cfg);
    ASSERT_TRUE(sol.ok);
    ASSERT_FALSE(sol.guarded);
    EXPECT_NEAR(sol.beta_deg, 60.0, 1e-6);
    EXPECT_TRUE(sol.saturated);
    EXPECT_NEAR(sol.kappa_cmd, 1.0 / 500.0, 1e-12);
}

TEST(L1, GuardHoldsWhenReferenceIsBehind)
{
    GuidanceConfig cfg;
    cfg.heads_source = HeadsSource::L1;
    // 路径朝西（背离本机航向）→ 参考点落到本机后方，|β| ≈ 180°
    std::vector<PathPoint> path;
    for (int i = 0; i <= 20; ++i) {
        path.push_back(make_point(-100.0 * i, 0.0, 100.0, uav_guide::geo::kPi, 0.0));
    }
    const State5 uav = uav_guide::state::make(0.0, 0.0, 100.0, 0.0);   // 朝东
    const auto sol = uav_guide::guidance::solve_l1(path, uav, cfg);
    ASSERT_TRUE(sol.ok);
    EXPECT_TRUE(sol.guarded);
    EXPECT_GT(std::abs(sol.beta_deg), 90.0);

    const State5 target = uav_guide::state::make(0.0, 0.0, 0.0, 0.0);
    const auto out = uav_guide::guidance::build(path, uav, target, cfg);
    EXPECT_TRUE(out.hold_previous);                     // 节点据此保持上一指令
    EXPECT_NEAR(out.los_heading_deg, 270.0, 1e-6);      // 参考点在正西
}

TEST(L1, NearEndClampsReferenceToPathEnd)
{
    GuidanceConfig cfg;                                 // L1 = 1000 > 路径总长 400
    const auto path = l1_straight(400.0, 100.0);
    const State5 uav = uav_guide::state::make(0.0, 0.0, 100.0, 0.0);

    const auto sol = uav_guide::guidance::solve_l1(path, uav, cfg);
    ASSERT_TRUE(sol.ok);
    EXPECT_TRUE(sol.near_end);
    EXPECT_NEAR(sol.l1_eff_m, 400.0, 1e-9);
    EXPECT_NEAR(sol.ref.x, 400.0, 1e-9);
}

TEST(L1, MissingPoseHoldsPrevious)
{
    GuidanceConfig cfg;
    cfg.heads_source = HeadsSource::L1;
    const auto path = l1_straight(3000.0);
    const State5 invalid;                               // valid = false
    const State5 target = uav_guide::state::make(0.0, 0.0, 0.0, 0.0);

    const auto sol = uav_guide::guidance::solve_l1(path, invalid, cfg);
    EXPECT_FALSE(sol.ok);

    const auto out = uav_guide::guidance::build(path, invalid, target, cfg);
    EXPECT_TRUE(out.hold_previous);
    EXPECT_NEAR(out.l1_eff_m, 0.0, 1e-12);              // 无解标志：节点应跳过发布
}

// 闭环验证：横向偏置收敛的阻尼/超调应匹配解析结果（ζ=0.707、超调 4.3%、4τ≈80 s）
TEST(L1, ClosedLoopConvergesWithTheoryDamping)
{
    GuidanceConfig cfg;                                 // L1=1000, R_c=500, V=50
    // 关掉死区以验证控制律本身的动力学；生产默认 0.5° 会留下 ~L1·tan(0.5°)=8.7 m 无差区（有意设计）
    cfg.beta_deadband_deg = 0.0;
    const auto path = l1_straight(20000.0, 100.0);
    State5 uav = uav_guide::state::make(0.0, 200.0, 100.0, 0.0);   // 初始横向偏置 200 m
    uav.speed_mps = cfg.speed_mps;

    const double dt = 0.05;
    double y_min = 0.0;
    double y_final = 0.0;
    for (int i = 0; i < 6000; ++i) {                    // 300 s
        const auto sol = uav_guide::guidance::solve_l1(path, uav, cfg);
        ASSERT_TRUE(sol.ok);
        // 理想执行：实际转弯角速率 = V·κ（T_lead 只影响指令前馈，不影响闭环）
        const double omega = cfg.speed_mps * sol.kappa_cmd;
        uav.yaw = uav_guide::geo::wrap_angle(uav.yaw + omega * dt);
        uav.x += cfg.speed_mps * std::cos(uav.yaw) * dt;
        uav.y += cfg.speed_mps * std::sin(uav.yaw) * dt;
        y_min = std::min(y_min, uav.y);
        y_final = uav.y;
    }

    const double overshoot = -y_min / 200.0;
    EXPECT_GT(overshoot, 0.01) << "超调应≈4.3%（ζ=0.707）";
    EXPECT_LT(overshoot, 0.08) << "超调不应超过 8%（否则阻尼不足）";
    EXPECT_LT(std::abs(y_final), 1.0) << "4τ≈80 s 后横向误差应收敛";
}

// 死区效应：默认 0.5° 死区会在路径附近留下有界无差区（~L1·tan(deadband)）
TEST(L1, DeadbandLeavesBoundedResidual)
{
    GuidanceConfig cfg;
    const auto path = l1_straight(20000.0, 100.0);
    State5 uav = uav_guide::state::make(0.0, 200.0, 100.0, 0.0);
    uav.speed_mps = cfg.speed_mps;

    const double dt = 0.05;
    for (int i = 0; i < 6000; ++i) {
        const auto sol = uav_guide::guidance::solve_l1(path, uav, cfg);
        ASSERT_TRUE(sol.ok);
        uav.yaw = uav_guide::geo::wrap_angle(uav.yaw + cfg.speed_mps * sol.kappa_cmd * dt);
        uav.x += cfg.speed_mps * std::cos(uav.yaw) * dt;
        uav.y += cfg.speed_mps * std::sin(uav.yaw) * dt;
    }
    // 死区对应的横向无差区上限：L1·tan(beta_deadband) ≈ 8.73 m，实际应落在此界线内
    const double dead_zone =
        cfg.l1_distance_m * std::tan(cfg.beta_deadband_deg * uav_guide::geo::kPi / 180.0);
    EXPECT_LT(std::abs(uav.y), dead_zone + 1.0);
    EXPECT_GT(std::abs(uav.y), 0.1) << "死区存在时不应声称无静差";
}

// ═══════════════════════════════════════════════════════════════
// 5. orchestrator：APPROACH / TRACKING 状态机
// ═══════════════════════════════════════════════════════════════

TEST(Orchestrator, ExpectedYawAndVirtualGoal)
{
    const State5 target = uav_guide::state::make(1000.0, 0.0, 100.0, 0.0);   // 航向朝东

    EXPECT_NEAR(uav_guide::orchestrator::expected_intercept_yaw(target, InterceptMode::Tail), 0.0, 1e-9);
    // 注意：wrap_angle 值域为 [-π, π)（与 1.0.1 的 Python `(a+π)%2π-π` 完全一致），
    // 因此 ψ_tgt+π 落在 π 时结果为 -π（几何等价），按模 2π 比较。
    EXPECT_NEAR(uav_guide::geo::wrap_angle(
                    uav_guide::orchestrator::expected_intercept_yaw(target, InterceptMode::HeadOn) -
                    uav_guide::geo::kPi),
                0.0, 1e-9);

    const auto tail_goal = uav_guide::orchestrator::virtual_goal(target, InterceptMode::Tail, 1000.0);
    EXPECT_NEAR(tail_goal.x, 0.0, 1e-6);       // 目标后方
    const auto headon_goal =
        uav_guide::orchestrator::virtual_goal(target, InterceptMode::HeadOn, 1000.0);
    EXPECT_NEAR(headon_goal.x, 2000.0, 1e-6);  // 目标前方
}

TEST(Orchestrator, EntersTrackingWhenAlignedAndClosing)
{
    OrchestratorConfig cfg;
    uav_guide::orchestrator::StateMachine sm(cfg);
    const State5 target = uav_guide::state::make(1000.0, 0.0, 100.0, 0.0);

    // 第 1 帧：无历史 → closure = 0 → 不进入
    auto d1 = sm.update(uav_guide::state::make(0.0, 0.0, 100.0, 0.0), target, InterceptMode::Tail);
    EXPECT_FALSE(d1.tracking_changed);
    EXPECT_EQ(d1.phase, Phase::Approach);

    // 第 2 帧：接近 10 m ≥ 3 m 且共线 → 进入
    auto d2 = sm.update(uav_guide::state::make(10.0, 0.0, 100.0, 0.0), target, InterceptMode::Tail);
    EXPECT_TRUE(d2.tracking_changed);
    EXPECT_EQ(d2.phase, Phase::Tracking);
    EXPECT_TRUE(sm.tracking());
    EXPECT_NEAR(d2.closure, 10.0, 1e-9);
}

TEST(Orchestrator, NoEntryWhenMisaligned)
{
    OrchestratorConfig cfg;
    uav_guide::orchestrator::StateMachine sm(cfg);
    const State5 target = uav_guide::state::make(1000.0, 0.0, 100.0, 0.0);

    sm.update(uav_guide::state::make(0.0, 0.0, 100.0, uav_guide::geo::kPi), target, InterceptMode::Tail);
    auto d = sm.update(uav_guide::state::make(10.0, 0.0, 100.0, uav_guide::geo::kPi), target,
                       InterceptMode::Tail);
    EXPECT_FALSE(sm.tracking());            // 航向反向（align 180° > 70°）
    EXPECT_GT(d.align_err_deg, 90.0);
}

TEST(Orchestrator, ExitsTrackingAfterRepeatedDivergence)
{
    OrchestratorConfig cfg;   // window = 100 m, divergence_threshold = 3
    uav_guide::orchestrator::StateMachine sm(cfg);
    const State5 target = uav_guide::state::make(1000.0, 0.0, 100.0, 0.0);

    sm.update(uav_guide::state::make(0.0, 0.0, 100.0, 0.0), target, InterceptMode::Tail);
    sm.update(uav_guide::state::make(10.0, 0.0, 100.0, 0.0), target, InterceptMode::Tail);
    ASSERT_TRUE(sm.tracking());

    // 反向远离：每轮 4×30 m = 120 m ≥ window(100)，连续 3 轮距离增大 → 退出
    double x = 10.0;
    for (int round = 0; round < 3; ++round) {
        for (int i = 0; i < 4; ++i) {
            x -= 30.0;
            sm.update(uav_guide::state::make(x, 0.0, 100.0, 0.0), target, InterceptMode::Tail);
        }
    }
    EXPECT_FALSE(sm.tracking());
}

TEST(Orchestrator, ResetExitsTrackingImmediately)
{
    OrchestratorConfig cfg;
    uav_guide::orchestrator::StateMachine sm(cfg);
    const State5 target = uav_guide::state::make(1000.0, 0.0, 100.0, 0.0);

    sm.update(uav_guide::state::make(0.0, 0.0, 100.0, 0.0), target, InterceptMode::Tail);
    sm.update(uav_guide::state::make(10.0, 0.0, 100.0, 0.0), target, InterceptMode::Tail);
    ASSERT_TRUE(sm.tracking());

    sm.reset(InterceptMode::HeadOn);
    EXPECT_FALSE(sm.tracking());
}

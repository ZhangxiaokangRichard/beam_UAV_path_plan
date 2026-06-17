/**
 * @file test_beam_search.cpp
 * @brief Beam Dubins 核心算法单元测试（gtest，无 ROS 依赖）
 *
 * 测试覆盖：
 *  1. 几何工具（角度规范化、距离、圆心）
 *  2. 碰撞检测（球-AABB、胶囊-AABB、扫掠路径）
 *  3. Dubins 路径（4 型计算、最优路径查找）
 *  4. 基元生成（直行、转弯、自适应角度选择）
 *  5. Beam 搜索（无障碍直线、通道穿越、目标到达）
 */

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "beam_dubins/beam_scorer.h"
#include "beam_dubins/beam_search.h"
#include "beam_dubins/collision.h"
#include "beam_dubins/dubins_path.h"
#include "beam_dubins/geometry.h"
#include "beam_dubins/primitive_gen.h"
#include "beam_dubins/types.h"

using namespace beam_dubins;

// ═══════════════════════════════════════════════════════════════
// 测试1：几何工具
// ═══════════════════════════════════════════════════════════════

TEST(GeometryTest, Distance2D) {
    double p1[2] = {0.0, 0.0};
    double p2[2] = {3.0, 4.0};
    EXPECT_NEAR(distance_2d(p1, p2), 5.0, 1e-9);
}

TEST(GeometryTest, Distance3D) {
    double p1[3] = {0.0, 0.0, 0.0};
    double p2[3] = {1.0, 2.0, 2.0};
    // sqrt(1+4+4) = 3
    EXPECT_NEAR(distance_3d(p1, p2), 3.0, 1e-9);
}

TEST(GeometryTest, WrapAngle) {
    // 检查角度规范化到 (-π, π] 范围
    double v1 = wrap_angle(3.5 * M_PI);
    EXPECT_TRUE(std::abs(v1 - (-0.5 * M_PI)) < 1e-9 || std::abs(v1 - (1.5 * M_PI)) < 1e-9);

    double v2 = wrap_angle(-3.5 * M_PI);
    EXPECT_NEAR(v2, 0.5 * M_PI, 1e-9);

    double v3 = wrap_angle(0.0);
    EXPECT_NEAR(v3, 0.0, 1e-9);

    // π 在范围右边界，应规范化为 +π（不是 -π）
    double v4 = wrap_angle(M_PI);
    EXPECT_NEAR(v4, M_PI, 1e-9);

    // 2π 应规范化为 0
    double v5 = wrap_angle(2.0 * M_PI);
    EXPECT_NEAR(v5, 0.0, 1e-9);
}

TEST(GeometryTest, CCWArcAngle) {
    double ang = ccw_arc_angle(0.0, M_PI / 2.0);
    EXPECT_NEAR(ang, M_PI / 2.0, 1e-9);

    double ang2 = ccw_arc_angle(M_PI, -M_PI / 2.0);
    EXPECT_NEAR(ang2, 0.5 * M_PI, 1e-9);  // π→-π/2 逆时针 = 270°→90° = π/2
}

TEST(GeometryTest, CWArcAngle) {
    double ang = cw_arc_angle(0.0, -M_PI / 2.0);
    EXPECT_NEAR(ang, M_PI / 2.0, 1e-9);
}

TEST(GeometryTest, LeftCenter) {
    double pos[3] = {0.0, 0.0, 0.0};  // x, y, ψ=0（向东）
    double out[2];
    left_center(pos, 250.0, out);
    // 左圆心：(0 - 250*sin0, 0 + 250*cos0) = (0, 250)
    EXPECT_NEAR(out[0], 0.0, 1e-6);
    EXPECT_NEAR(out[1], 250.0, 1e-6);
}

TEST(GeometryTest, RightCenter) {
    double pos[3] = {0.0, 0.0, 0.0};  // ψ=0（向东）
    double out[2];
    right_center(pos, 250.0, out);
    // 右圆心：(0 + 250*sin0, 0 - 250*cos0) = (0, -250)
    EXPECT_NEAR(out[0], 0.0, 1e-6);
    EXPECT_NEAR(out[1], -250.0, 1e-6);
}

TEST(GeometryTest, SampleArc) {
    double center[2] = {0.0, 0.0};
    auto pts = sample_arc_2d(center, 10.0, 0.0, M_PI / 2.0, 5);
    ASSERT_EQ(pts.size(), 5u);
    // 第一个点：angle=0 → (10, 0)
    EXPECT_NEAR(pts[0][0], 10.0, 1e-6);
    EXPECT_NEAR(pts[0][1], 0.0, 1e-6);
    // 最后一个点：angle=π/2 → (0, 10)
    EXPECT_NEAR(pts[4][0], 0.0, 1e-6);
    EXPECT_NEAR(pts[4][1], 10.0, 1e-6);
}

// ═══════════════════════════════════════════════════════════════
// 测试2：碰撞检测
// ═══════════════════════════════════════════════════════════════

TEST(CollisionTest, SphereAABB_Inside) {
    double center[3] = {5.0, 5.0, 5.0};
    double aabb_min[3] = {0.0, 0.0, 0.0};
    double aabb_max[3] = {10.0, 10.0, 10.0};
    EXPECT_TRUE(sphere_aabb_intersect(center, 1.0, aabb_min, aabb_max));
}

TEST(CollisionTest, SphereAABB_Outside) {
    double center[3] = {15.0, 5.0, 5.0};
    double aabb_min[3] = {0.0, 0.0, 0.0};
    double aabb_max[3] = {10.0, 10.0, 10.0};
    EXPECT_FALSE(sphere_aabb_intersect(center, 1.0, aabb_min, aabb_max));
}

TEST(CollisionTest, SphereAABB_NearEdge) {
    double center[3] = {11.0, 5.0, 5.0};
    double aabb_min[3] = {0.0, 0.0, 0.0};
    double aabb_max[3] = {10.0, 10.0, 10.0};
    EXPECT_TRUE(sphere_aabb_intersect(center, 1.5, aabb_min, aabb_max));
    EXPECT_FALSE(sphere_aabb_intersect(center, 0.5, aabb_min, aabb_max));
}

TEST(CollisionTest, CapsuleAABB_Through) {
    double p1[3] = {-10.0, 5.0, 5.0};
    double p2[3] = {20.0, 5.0, 5.0};
    double aabb_min[3] = {0.0, 0.0, 0.0};
    double aabb_max[3] = {10.0, 10.0, 10.0};
    // 胶囊体穿过 AABB → 碰撞
    EXPECT_TRUE(capsule_aabb_intersect(p1, p2, 0.5, aabb_min, aabb_max));
}

TEST(CollisionTest, CapsuleAABB_Miss) {
    double p1[3] = {-10.0, 15.0, 5.0};
    double p2[3] = {20.0, 15.0, 5.0};
    double aabb_min[3] = {0.0, 0.0, 0.0};
    double aabb_max[3] = {10.0, 10.0, 10.0};
    // 胶囊体从 AABB 上方通过 → 无碰撞
    EXPECT_FALSE(capsule_aabb_intersect(p1, p2, 0.5, aabb_min, aabb_max));
}

// ═══════════════════════════════════════════════════════════════
// 测试3：Dubins 路径
// ═══════════════════════════════════════════════════════════════

TEST(DubinsPathTest, StraightLine) {
    // 起点→终点 在同一水平线上，航向一致 → 应近似直线
    DubinsPath3D dubins(250.0, 0.087266, 5.0);

    double start[5] = {0.0, 0.0, 0.0, 0.0, 0.0};       // (0,0), ψ=0（向东）
    double goal[5]  = {1000.0, 0.0, 0.0, 0.0, 0.0};    // (1000,0), ψ=0

    auto candidates = dubins.find_horizontal_tangents(start, goal);
    ASSERT_GE(candidates.size(), 1u);
    // 最短路径应近似 1000m
    EXPECT_NEAR(candidates[0].total_len, 1000.0, 10.0);
}

TEST(DubinsPathTest, OppositeHeading) {
    // 起点向东，终点向西 → 需要 U 形转弯
    DubinsPath3D dubins(250.0, 0.087266, 5.0);

    double start[5] = {0.0, 0.0, 0.0, 0.0, 0.0};           // ψ=0
    double goal[5]  = {1000.0, 0.0, 0.0, M_PI, 0.0};       // ψ=π

    auto candidates = dubins.find_horizontal_tangents(start, goal);
    ASSERT_GE(candidates.size(), 1u);
    // 路径应有明显弧线，长度 > 1000
    EXPECT_GT(candidates[0].total_len, 1200.0);
}

TEST(DubinsPathTest, BestPath) {
    DubinsPath3D dubins(250.0, 0.087266, 5.0);

    double start[5] = {2000.0, 1000.0, 500.0, M_PI / 2.0, 0.0};  // 向北
    double goal[5]  = {5000.0, 2500.0, 600.0, 0.0, 0.0};          // 向东

    std::vector<Waypoint> path;
    double cost = dubins.best_path(start, goal, path);
    EXPECT_GT(cost, 0.0);
    EXPECT_GE(path.size(), 3u);   // 至少 3 个点（弧1+直线+弧2）
}

// ═══════════════════════════════════════════════════════════════
// 测试4：基元生成
// ═══════════════════════════════════════════════════════════════

TEST(PrimitiveGenTest, GenerateInOpenSpace) {
    PrimitiveGenerator gen(250.0, 0.087266, 5.0);

    BeamNode node;
    node.pos.x = 0.0; node.pos.y = 0.0; node.pos.z = 0.0;
    node.pos.yaw = 0.0; node.pos.pitch = 0.0;

    double clearance = 600.0;     // 开阔区
    double step_size = 500.0;

    auto prims = gen.generate(node, step_size, clearance);
    // 开阔区角度集合：{0, 30, 60} → 直行S + L_30 + R_30 + L_60 + R_60 = 5个
    EXPECT_EQ(prims.size(), 5u);

    // 检查类型标签
    EXPECT_EQ(prims[0].type, "S");        // 直行优先
    EXPECT_EQ(prims[0].turn_direction, 'S');
    EXPECT_NEAR(prims[0].arc_length, step_size, 1e-6);
}

TEST(PrimitiveGenTest, GenerateInTightSpace) {
    PrimitiveGenerator gen(250.0, 0.087266, 5.0);

    BeamNode node;
    node.pos.x = 0.0; node.pos.y = 0.0; node.pos.z = 0.0;
    node.pos.yaw = 0.0; node.pos.pitch = 0.0;

    double clearance = 50.0;      // 密集区
    double step_size = 150.0;

    auto prims = gen.generate(node, step_size, clearance);
    // 密集区角度集合：{0, 15, 30, 45, 60, 90} → 1 + 2*5 = 11个
    EXPECT_EQ(prims.size(), 11u);
}

TEST(PrimitiveGenTest, StraightPrimitive) {
    PrimitiveGenerator gen(250.0, 0.087266, 5.0);

    BeamNode node;
    node.pos.x = 1000.0; node.pos.y = 2000.0; node.pos.z = 500.0;
    node.pos.yaw = M_PI / 4.0;  // 45° 东北方向
    node.pos.pitch = 0.0;

    double step_size = 500.0;
    // 仅生成直行（clearance>500 角度集中包含 0）
    auto prims = gen.generate(node, step_size, 600.0);

    // 找到直行基元
    const DubinsPrimitive* straight = nullptr;
    for (const auto& p : prims) {
        if (p.type == "S") {
            straight = &p;
            break;
        }
    }
    ASSERT_NE(straight, nullptr);
    EXPECT_NEAR(straight->arc_length, step_size, 1e-6);

    // 终点应在东北方向 500m 处
    double dx = straight->waypoints.back().x - node.pos.x;
    double dy = straight->waypoints.back().y - node.pos.y;
    EXPECT_NEAR(dx, step_size * std::cos(M_PI / 4.0), 1.0);
    EXPECT_NEAR(dy, step_size * std::sin(M_PI / 4.0), 1.0);
}

// ═══════════════════════════════════════════════════════════════
// 测试5：Beam 搜索 — 无障碍直线
// ═══════════════════════════════════════════════════════════════

TEST(BeamSearchTest, StraightLineNoObstacles) {
    // 起点→终点 无障碍，应快速找到近似直线路径
    DubinsPath3D dubins(250.0, 0.087266, 5.0);

    BeamConfig cfg;
    cfg.max_depth = 50;
    cfg.max_extend_length = 200.0;
    cfg.min_extend_length = 100.0;
    cfg.goal_tolerance_xy = 50.0;

    double start[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double goal[5]  = {1000.0, 0.0, 0.0, 0.0, 0.0};

    BeamDubins planner(nullptr, dubins, start, goal, cfg);

    std::vector<Waypoint> path;
    std::vector<BeamNode> all_nodes;
    double cost = planner.search(path, all_nodes);

    EXPECT_LT(cost, INFINITY);
    EXPECT_GT(cost, 900.0);    // 不应该短于直线距离
    EXPECT_LT(cost, 1200.0);   // 不应太长（允许轻微绕路）
    EXPECT_GT(path.size(), 2u);
    EXPECT_GT(all_nodes.size(), 10u);
}

// ═══════════════════════════════════════════════════════════════
// 测试6：Beam 搜索 — 通道场景
// ═══════════════════════════════════════════════════════════════

TEST(BeamSearchTest, ChannelObstacles) {
    // 构造通道场景：左右两堵墙，中间有 100m 宽的通道
    // 起点在通道左侧，目标在通道右侧

    // 左墙：x ∈ [2000, 4000], y ∈ [0, 2450]
    AABB left_wall;
    left_wall.min = {2000.0, 0.0, 0.0};
    left_wall.max = {4000.0, 2450.0, 1000.0};

    // 右墙：x ∈ [2000, 4000], y ∈ [2550, 5000]
    AABB right_wall;
    right_wall.min = {2000.0, 2550.0, 0.0};
    right_wall.max = {4000.0, 5000.0, 1000.0};

    std::vector<AABB> obstacles = {left_wall, right_wall};

    DubinsPath3D dubins(250.0, 0.087266, 5.0, &obstacles);

    BeamConfig cfg;
    cfg.max_depth = 100;
    cfg.max_extend_length = 300.0;
    cfg.min_extend_length = 100.0;
    cfg.goal_tolerance_xy = 50.0;
    cfg.beam_width = 4;
    cfg.beam_width_max = 8;

    // 起点在通道左侧 (x=1000, y=2500)
    double start[5] = {1000.0, 2500.0, 500.0, 0.0, 0.0};
    // 目标在通道右侧 (x=5000, y=2500)
    double goal[5]  = {5000.0, 2500.0, 600.0, 0.0, 0.0};

    BeamDubins planner(&obstacles, dubins, start, goal, cfg);

    std::vector<Waypoint> path;
    std::vector<BeamNode> all_nodes;
    double cost = planner.search(path, all_nodes);

    // 应能找到路径并通过通道
    EXPECT_LT(cost, INFINITY);
    EXPECT_GT(path.size(), 3u);

    // 路径的所有 y 坐标应在 [2450, 2550] 通道范围内（通过通道时）
    // 松检测：至少有一个路径点 y 在通道范围内
    bool passes_channel = false;
    for (const auto& pp : path) {
        if (pp.x > 2000.0 && pp.x < 4000.0 && pp.y > 2450.0 && pp.y < 2550.0) {
            passes_channel = true;
            break;
        }
    }
    EXPECT_TRUE(passes_channel) << "路径应穿过通道（y∈[2450,2550], x∈[2000,4000]）";
}

// ═══════════════════════════════════════════════════════════════
// 测试7：评分器
// ═══════════════════════════════════════════════════════════════

TEST(BeamScorerTest, FastVsExact) {
    DubinsPath3D dubins(250.0, 0.087266, 5.0);

    double start[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double goal[5]  = {2000.0, 0.0, 0.0, 0.0, 0.0};

    BeamScorer scorer(dubins, start, goal);

    BeamNode node1;
    node1.pos.x = 500.0; node1.pos.y = 0.0; node1.pos.z = 0.0;
    node1.pos.yaw = 0.0; node1.pos.pitch = 0.0;
    node1.g = 500.0;
    node1.clearance = 800.0;

    BeamNode node2;
    node2.pos.x = 500.0; node2.pos.y = 200.0; node2.pos.z = 0.0;
    node2.pos.yaw = 0.0; node2.pos.pitch = 0.0;
    node2.g = 550.0;
    node2.clearance = 800.0;

    // node1 更接近直线方向，应获得更好分数（更小 f）
    double f1_fast = scorer.evaluate_fast(node1);
    double f2_fast = scorer.evaluate_fast(node2);
    EXPECT_LT(f1_fast, f2_fast) << "沿直线方向的节点应有更低（更好）的评分";

    double f1_exact = scorer.evaluate_exact(node1);
    double f2_exact = scorer.evaluate_exact(node2);
    EXPECT_LT(f1_exact, f2_exact);
}

// ═══════════════════════════════════════════════════════════════
// 测试8：Dubins 启发式缓存
// ═══════════════════════════════════════════════════════════════

TEST(BeamScorerTest, HeuristicCache) {
    DubinsPath3D dubins(250.0, 0.087266, 5.0);

    double start[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double goal[5]  = {2000.0, 0.0, 0.0, 0.0, 0.0};

    BeamScorer scorer(dubins, start, goal);

    double pos1[5] = {500.0, 0.0, 0.0, 0.0, 0.0};
    double pos2[5] = {510.0, 0.0, 0.0, 0.0, 0.0};  // 同一 50m 网格

    double h1 = scorer.dubins_heuristic(pos1, goal);
    double h2 = scorer.dubins_heuristic(pos2, goal);

    // 同一网格应返回相同值（缓存命中）
    EXPECT_NEAR(h1, h2, 1e-6);
}

// ═══════════════════════════════════════════════════════════════
// 运行所有测试
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

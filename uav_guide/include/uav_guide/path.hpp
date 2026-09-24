/**
 * @file path.hpp
 * @brief 路径几何：直线/弧段切分、最近点、弧心、**采样点检索**（零 ROS；两制导节点共享）
 */

#pragma once

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "uav_guide/types.hpp"

namespace uav_guide::path {

/// 段类型
enum class Kind { Line, Arc };

/// 切分后的段（索引区间为闭区间）
struct Segment {
    Kind kind = Kind::Line;
    std::size_t start_index = 0;
    std::size_t end_index = 0;
    bool is_left = false;    // 仅 Arc：左转
    double radius_m = 0.0;   // 仅 Arc：正半径（m）
};

/// 将路径点列切分为直线/弧段（判据：|curvature| 或相邻 yaw 差）
std::vector<Segment> segment(const std::vector<PathPoint>& points,
                             double curvature_eps = 1e-4,
                             double default_radius_m = 500.0,
                             double yaw_eps = 0.03);

/// 距给定平面点最近的路径点索引（空路径返回 0）
std::size_t closest_index(const std::vector<PathPoint>& points, double x, double y);

/// 路径总长（相邻点折线长度累加，m）
double length(const std::vector<PathPoint>& points);

/// 路径上的投影点（**段内线性插值**，用于消除大点距造成的指令跳变）
struct Projection {
    bool valid = false;
    std::size_t seg_index = 0;  // 投影片段起点索引（段 = [seg_index, seg_index+1]）
    double t = 0.0;             // 段内参数 [0,1]
    double s_m = 0.0;           // 沿路径里程（m）
    double x = 0.0, y = 0.0, z = 0.0;
    double yaw = 0.0;           // 取该段起点航向（yaw 有回绕，不做插值）
    double distance_m = 0.0;    // 本机 → 投影点距离
};

/// 求本机 (x,y) 在路径折线上的最近投影（单点路径退化为该点；空路径 valid=false）
Projection project(const std::vector<PathPoint>& points, double x, double y);

/// 取沿路径里程 s_m 处的插值点；s_m 超界则夹到端点（clamped 非空时回写是否夹到）
PathPoint point_at_arclength(const std::vector<PathPoint>& points, double s_m,
                             std::size_t* seg_index = nullptr, bool* clamped = nullptr);

/// 圆弧圆心：ENU 下已知起点位姿、半径与转向
std::pair<double, double> arc_center_xy(double x, double y, double yaw_enu,
                                        double radius_m, bool is_left);

/// 采样点检索（Q12/Q16）：自 from_index 起按 rule 向后扫描，返回索引；无命中返回 points.size()
std::size_t find_sample_index(const std::vector<PathPoint>& points,
                              std::size_t from_index,
                              SampleRule rule,
                              double curvature_eps = 1e-4);

/// [closest_index, end] 范围内"最远"的未飞过弧段（无则 nullopt）
std::optional<Segment> farthest_remaining_arc(const std::vector<Segment>& segments,
                                              std::size_t closest_index);

/// 该索引所在段（越界返回 nullptr）
const Segment* segment_at(const std::vector<Segment>& segments, std::size_t index);

}  // namespace uav_guide::path

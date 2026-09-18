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

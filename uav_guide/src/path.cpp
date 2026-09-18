/**
 * @file path.cpp
 * @brief 路径几何实现（移植 1.0.1 的 path_segmenter.py 判据，并新增采样点检索）
 */

#include "uav_guide/path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "uav_guide/geo.hpp"

namespace uav_guide::path {
namespace {

/// 单点类型判定（与 Python _point_mode 一致）
Kind point_kind(const std::vector<PathPoint>& pts, std::size_t i,
                double curvature_eps, double yaw_eps)
{
    const double k = std::abs(pts[i].curvature);
    if (k > curvature_eps) return Kind::Arc;
    if (i > 0) {
        const double dyaw = std::abs(geo::wrap_angle(pts[i].yaw - pts[i - 1].yaw));
        if (dyaw > yaw_eps) return Kind::Arc;
    }
    if (i + 1 < pts.size()) {
        const double dyaw = std::abs(geo::wrap_angle(pts[i + 1].yaw - pts[i].yaw));
        if (dyaw > yaw_eps) return Kind::Arc;
    }
    return Kind::Line;
}

/// 转向判定（与 Python _infer_turn_direction 一致）
bool infer_is_left(const std::vector<PathPoint>& chunk)
{
    if (chunk.size() < 2) return false;
    const double dyaw = geo::wrap_angle(chunk.back().yaw - chunk.front().yaw);
    return dyaw > 0.0;
}

/// 平均半径（与 Python _mean_radius 一致）
double mean_radius(const std::vector<PathPoint>& chunk, double default_radius,
                   double curvature_eps)
{
    double sum = 0.0;
    int count = 0;
    for (const auto& p : chunk) {
        if (std::abs(p.curvature) > curvature_eps) {
            sum += 1.0 / p.curvature;
            ++count;
        }
    }
    if (count == 0) return default_radius;
    return std::abs(sum / static_cast<double>(count));
}

}  // namespace

std::vector<Segment> segment(const std::vector<PathPoint>& points,
                             double curvature_eps,
                             double default_radius_m,
                             double yaw_eps)
{
    std::vector<Segment> segments;
    if (points.empty()) return segments;

    std::size_t i = 0;
    const std::size_t n = points.size();
    while (i < n) {
        const Kind kind = point_kind(points, i, curvature_eps, yaw_eps);
        std::size_t j = i + 1;
        while (j < n && point_kind(points, j, curvature_eps, yaw_eps) == kind) ++j;

        Segment seg;
        seg.kind = kind;
        seg.start_index = i;
        seg.end_index = j - 1;
        if (kind == Kind::Arc) {
            const std::vector<PathPoint> chunk(points.begin() + static_cast<long>(i),
                                               points.begin() + static_cast<long>(j));
            seg.is_left = infer_is_left(chunk);
            seg.radius_m = mean_radius(chunk, default_radius_m, curvature_eps);
        }
        segments.push_back(seg);
        i = j;
    }
    return segments;
}

std::size_t closest_index(const std::vector<PathPoint>& points, double x, double y)
{
    std::size_t best_i = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < points.size(); ++i) {
        const double d = std::hypot(points[i].x - x, points[i].y - y);
        if (d < best_d) {
            best_d = d;
            best_i = i;
        }
    }
    return best_i;
}

std::pair<double, double> arc_center_xy(double x, double y, double yaw_enu,
                                        double radius_m, bool is_left)
{
    if (is_left) {
        return {x - radius_m * std::sin(yaw_enu), y + radius_m * std::cos(yaw_enu)};
    }
    return {x + radius_m * std::sin(yaw_enu), y - radius_m * std::cos(yaw_enu)};
}

std::size_t find_sample_index(const std::vector<PathPoint>& points,
                              std::size_t from_index,
                              SampleRule rule,
                              double curvature_eps)
{
    for (std::size_t i = from_index; i < points.size(); ++i) {
        const double k = std::abs(points[i].curvature);
        if (rule == SampleRule::FirstZeroCurvature) {
            if (k <= curvature_eps) return i;
        } else {
            if (k > curvature_eps) return i;
        }
    }
    return points.size();
}

std::optional<Segment> farthest_remaining_arc(const std::vector<Segment>& segments,
                                              std::size_t closest_index)
{
    std::optional<Segment> best;
    for (const auto& seg : segments) {
        if (seg.kind != Kind::Arc) continue;
        if (seg.end_index < closest_index) continue;
        if (!best.has_value() || seg.start_index > best->start_index) best = seg;
    }
    return best;
}

const Segment* segment_at(const std::vector<Segment>& segments, std::size_t index)
{
    for (const auto& seg : segments) {
        if (index >= seg.start_index && index <= seg.end_index) return &seg;
    }
    return nullptr;
}

}  // namespace uav_guide::path

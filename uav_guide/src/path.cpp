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

double length(const std::vector<PathPoint>& points)
{
    double total = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
        total += std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    }
    return total;
}

Projection project(const std::vector<PathPoint>& points, double x, double y)
{
    Projection p;
    if (points.empty()) return p;

    if (points.size() == 1) {
        p.valid = true;
        p.seg_index = 0;
        p.t = 0.0;
        p.s_m = 0.0;
        p.x = points[0].x;
        p.y = points[0].y;
        p.z = points[0].z;
        p.yaw = points[0].yaw;
        p.distance_m = std::hypot(x - p.x, y - p.y);
        return p;
    }

    double s_acc = 0.0;
    double best_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        const double ax = points[i].x;
        const double ay = points[i].y;
        const double dx = points[i + 1].x - ax;
        const double dy = points[i + 1].y - ay;
        const double seg_len = std::hypot(dx, dy);

        double t = 0.0;
        if (seg_len > 1e-9) {
            t = ((x - ax) * dx + (y - ay) * dy) / (seg_len * seg_len);
            t = std::min(1.0, std::max(0.0, t));   // 夹到段内（投影落在段外时取端点）
        }
        const double cx = ax + t * dx;
        const double cy = ay + t * dy;
        const double d = std::hypot(x - cx, y - cy);

        if (d < best_d) {
            best_d = d;
            p.valid = true;
            p.seg_index = i;
            p.t = t;
            p.s_m = s_acc + t * seg_len;
            p.x = cx;
            p.y = cy;
            p.z = points[i].z + t * (points[i + 1].z - points[i].z);
            p.yaw = points[i].yaw;   // yaw 有回绕，不插值（仅诊断用）
            p.distance_m = d;
        }
        s_acc += seg_len;
    }
    return p;
}

PathPoint point_at_arclength(const std::vector<PathPoint>& points, double s_m,
                             std::size_t* seg_index, bool* clamped)
{
    if (seg_index != nullptr) *seg_index = 0;
    if (clamped != nullptr) *clamped = false;
    if (points.empty()) return PathPoint{};
    if (points.size() == 1) return points.front();

    const double total = length(points);
    if (s_m <= 0.0) {
        if (clamped != nullptr) *clamped = true;
        return points.front();
    }
    if (s_m >= total) {
        if (clamped != nullptr) *clamped = true;
        if (seg_index != nullptr) *seg_index = points.size() - 2;
        return points.back();
    }

    double s_acc = 0.0;
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
        const double dx = points[i + 1].x - points[i].x;
        const double dy = points[i + 1].y - points[i].y;
        const double seg_len = std::hypot(dx, dy);
        if (s_m <= s_acc + seg_len) {
            const double t = (seg_len > 1e-9) ? ((s_m - s_acc) / seg_len) : 0.0;
            PathPoint out;
            out.x = points[i].x + t * dx;
            out.y = points[i].y + t * dy;
            out.z = points[i].z + t * (points[i + 1].z - points[i].z);
            out.yaw = points[i].yaw;   // yaw 有回绕，不插值
            out.pitch = points[i].pitch;
            out.curvature = points[i].curvature;
            if (seg_index != nullptr) *seg_index = i;
            return out;
        }
        s_acc += seg_len;
    }
    return points.back();   // 不可达（s_m < total 已保证命中）
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

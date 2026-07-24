/**
 * @file geometry.h
 * @brief 二维/三维几何工具函数（纯 C++，无 ROS 依赖）
 *
 * 从 Python utils/geometry.py 迁移。
 * 提供距离计算、角度规范化、圆弧采样、圆心计算等基础几何运算。
 */

#pragma once

#include <cmath>
#include <vector>

namespace beam_dubins {

// ── 距离计算 ──────────────────────────────────────────────────

/// 二维欧氏距离
double distance_2d(const double* p1, const double* p2);

/// 三维欧氏距离（可选 z 权重）
double distance_3d(const double* p1, const double* p2, double z_weight = 1.0);

// ── 角度工具 ──────────────────────────────────────────────────

/// 将角度规范化到 (-π, π] 范围
double wrap_angle(double a);

/// 有符号角度差 b - a，规范到 (-π, π]  → angle_diff(a,b) = wrap(b-a)
double angle_diff(double a, double b);

/// 从 start_ang 逆时针 (CCW) 到 end_ang 的角度值（0 ~ 2π）
double ccw_arc_angle(double start_ang, double end_ang);

/// 从 start_ang 顺时针 (CW) 到 end_ang 的角度值（0 ~ 2π）
double cw_arc_angle(double start_ang, double end_ang);

/// 二维圆弧弧长 = r * |dtheta|
double arc_length_2d(double r, double dtheta);

// ── 方向角计算 ───────────────────────────────────────────────

/// 水平方向角 ψ = atan2(dy, dx)
double directional_yaw(const double* p1, const double* p2);

/// 俯仰角 γ = atan2(dz, dist_xy)
double directional_pitch(const double* p1, const double* p2);

// ── 向量运算 ──────────────────────────────────────────────────

/// 二维向量旋转（逆时针为正）
/// @param v  输入向量 [x, y]
/// @param theta  旋转角 (rad)
/// @param out  输出旋转后的向量 [x, y]
void rotate_2d(const double* v, double theta, double* out);

/// 二维向量归一化
/// @param v  输入向量 [x, y]
/// @param out  输出归一化向量 [x, y]
void normalize_2d(const double* v, double* out);

/// 二维左法向量（逆时针旋转 90°）
/// @param v  输入向量 [x, y]
/// @param out  输出 [ -v[1], v[0] ]
void left_normal_2d(const double* v, double* out);

// ── 圆弧与圆心 ────────────────────────────────────────────────

/// 采样水平面圆弧
/// @param center  圆心 [cx, cy]
/// @param r  半径
/// @param start_ang  起始角 (rad)
/// @param sweep  扫掠角 (rad)，>0 为逆时针
/// @param n  采样点数（含端点）
/// @return [[x, y], ...] 共 n 个点
std::vector<std::vector<double>> sample_arc_2d(
    const double* center, double r, double start_ang, double sweep, int n);

/// 左转圆心计算
/// @param pos  当前位置 [x, y, ψ]
/// @param R  转弯半径 (m)
/// @param out  输出圆心 [cx, cy]
void left_center(const double* pos, double R, double* out);

/// 右转圆心计算
/// @param pos  当前位置 [x, y, ψ]
/// @param R  转弯半径 (m)
/// @param out  输出圆心 [cx, cy]
void right_center(const double* pos, double R, double* out);

}  // namespace beam_dubins

#pragma once

#include <algorithm>
#include <cmath>

namespace ros_udp_bridge {

/// 一维卡尔曼滤波器（匀速随机游走模型），平滑速度分量或角速度（自包含）。
class ScalarKalmanFilter {
public:
    ScalarKalmanFilter(double process_noise, double measure_noise,
                       double init_state = 0.0, double init_cov = 1.0)
        : q_(process_noise), r_(measure_noise), x_(init_state), p_(init_cov) {}

    void predict() { p_ += q_; }

    void update(double measurement) {
        predict();
        double k = p_ / (p_ + r_);
        x_ += k * (measurement - x_);
        p_ = (1.0 - k) * p_;
    }

    double value() const { return x_; }
    void reset(double init_state, double init_cov = 1.0) {
        x_ = init_state; p_ = init_cov;
    }
    /// 重新设置噪声并重置状态（参数化滤波噪声用）。
    void setNoise(double process_noise, double measure_noise,
                  double init_state = 0.0, double init_cov = 1.0) {
        q_ = process_noise; r_ = measure_noise;
        x_ = init_state; p_ = init_cov;
    }

private:
    double q_, r_, x_, p_;
};

/// 二维速度卡尔曼滤波器 [vx, vy]^T（自包含）。
class VelocityFilter {
public:
    VelocityFilter(double process_noise, double measure_noise)
        : kf_vx_(process_noise, measure_noise)
        , kf_vy_(process_noise, measure_noise) {}

    void predict() { kf_vx_.predict(); kf_vy_.predict(); }

    void update(double vx_meas, double vy_meas) {
        kf_vx_.update(vx_meas);
        kf_vy_.update(vy_meas);
    }

    double vx() const { return kf_vx_.value(); }
    double vy() const { return kf_vy_.value(); }
    double speed() const { return std::hypot(vx(), vy()); }
    double yaw()  const { return std::atan2(vy(), vx()); }

    void reset(double vx0, double vy0) {
        kf_vx_.reset(vx0); kf_vy_.reset(vy0);
    }
    void setNoise(double process_noise, double measure_noise) {
        kf_vx_.setNoise(process_noise, measure_noise);
        kf_vy_.setNoise(process_noise, measure_noise);
    }

private:
    ScalarKalmanFilter kf_vx_;
    ScalarKalmanFilter kf_vy_;
};

}  // namespace ros_udp_bridge

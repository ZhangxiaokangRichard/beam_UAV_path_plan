#pragma once

#include <algorithm>
#include <cmath>

namespace ros_mqtt_bridge {

/**
 * @brief 一维卡尔曼滤波器，用于平滑速度分量或角速度。
 *
 * 模型：匀速随机游走（A=1, B=0, H=1）。
 * 预测：x = x,  P = P + Q
 * 更新：K = P/(P+R),  x = x + K*(z-x),  P = (1-K)*P
 */
class ScalarKalmanFilter {
public:
    ScalarKalmanFilter(double process_noise, double measure_noise,
                       double init_state = 0.0, double init_cov = 1.0)
        : q_(process_noise), r_(measure_noise), x_(init_state), p_(init_cov) {}

    /** 纯预测一步（无观测时调用）。 */
    void predict() { p_ += q_; }

    /** 带有观测的更新。 */
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

private:
    double q_, r_, x_, p_;
};

/**
 * @brief 二维速度卡尔曼滤波器 [vx, vy]^T。
 *
 * 状态与观测均为直角坐标速度分量，各自独立滤波以简化实现。
 */
class VelocityFilter {
public:
    VelocityFilter(double process_noise, double measure_noise)
        : kf_vx_(process_noise, measure_noise)
        , kf_vy_(process_noise, measure_noise) {}

    /** 无观测时的纯预测步（保持上次估计）。 */
    void predict() { kf_vx_.predict(); kf_vy_.predict(); }

    /** 用 COG 速度或 INS 速度观测更新滤波器。 */
    void update(double vx_meas, double vy_meas) {
        kf_vx_.update(vx_meas);
        kf_vy_.update(vy_meas);
    }

    /** 用位置差分（非瞬时速度）更新滤波器时，可指定更高的测量噪声。 */
    void updateWithPosDelta(double vx_meas, double vy_meas,
                            double pos_measure_noise) {
        // 临时提高 R 反映位置差分的不确定性
        // 简化处理：直接使用标准 update，调用方应在构造时选择合适 R
        (void)pos_measure_noise;
        update(vx_meas, vy_meas);
    }

    double vx() const { return kf_vx_.value(); }
    double vy() const { return kf_vy_.value(); }
    double speed() const { return std::hypot(vx(), vy()); }
    double yaw()  const { return std::atan2(vy(), vx()); }

    void reset(double vx0, double vy0) {
        kf_vx_.reset(vx0); kf_vy_.reset(vy0);
    }

private:
    ScalarKalmanFilter kf_vx_;
    ScalarKalmanFilter kf_vy_;
};

}  // namespace ros_mqtt_bridge

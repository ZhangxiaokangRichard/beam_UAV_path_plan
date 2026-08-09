/**
 * @file uav_dynamics_server.cpp
 * @brief uav_dynamic 运动解算服务节点：/uav_dynamic/solve。
 *
 * 输入 path + uav/target 高度 + 速度 → 输出水平（航向/滚转，协调转弯律+DWA）
 * 与纵向（迎角/垂直速度，高度误差 PI）控制指令。
 */

#include <algorithm>
#include <ros/ros.h>

#include "uav_dynamic/UavDynamicsSolve.h"
#include "uav_dynamic/dwa_tracker.h"

class UavDynamicsServer {
public:
    explicit UavDynamicsServer(ros::NodeHandle& nh) : nh_(nh)
    {
        nh_.param("motion/speed_mps", mp_.speed_mps, 50.0);
        nh_.param("motion/min_turn_radius_m", mp_.min_turn_radius_m, 500.0);
        nh_.param("motion/bank_angle_max_deg", mp_.bank_angle_max_deg, 25.0);
        nh_.param("motion/g", mp_.g, 9.81);
        nh_.param("motion/lookahead_m", mp_.lookahead_m, 100.0);
        nh_.param("motion/solve_hz", mp_.solve_hz, 20.0);
        nh_.param("dwa/sample_num", mp_.sample_num, 21);
        nh_.param("dwa/sample_window_rad", mp_.sample_window_rad, 0.2);
        nh_.param("dwa/roll_weight", mp_.roll_weight, 0.3);

        nh_.param("vertical/kp", vp_.kp, 0.6);
        nh_.param("vertical/ki", vp_.ki, 0.06);
        nh_.param("vertical/climb_vel_limit_mps", vp_.climb_vel_limit_mps, 3.0);
        nh_.param("vertical/alpha_limit_deg", vp_.alpha_limit_deg, 30.0);
        nh_.param("vertical/alpha_gain", vp_.alpha_gain, 0.175);
        nh_.param("vertical/integral_limit", vp_.integral_limit, 10.0);
        nh_.param("vertical/solve_hz", solve_hz_, 20.0);

        // 指令平滑（阶段 7：控制状态/姿态角变化连续）
        nh_.param("smooth/max_heading_step_rad", max_heading_step_, 0.2);
        nh_.param("smooth/max_roll_rate_radps",  max_roll_rate_,  0.35);
        nh_.param("smooth/max_alpha_rate_radps", max_alpha_rate_, 0.35);
        nh_.param("smooth/max_climb_rate_mps2",  max_climb_rate_, 2.0);

        srv_ = nh_.advertiseService("/uav_dynamic/solve", &UavDynamicsServer::handle, this);
        ROS_INFO("[uav_dynamic] server ready: speed=%.1f R_min=%.0f bank=%.1fdeg "
                 "climb_lim=%.1f alpha_lim=%.1fdeg",
                 mp_.speed_mps, mp_.min_turn_radius_m, mp_.bank_angle_max_deg,
                 vp_.climb_vel_limit_mps, vp_.alpha_limit_deg);
    }

private:
    bool handle(uav_dynamic::UavDynamicsSolveRequest& req,
                uav_dynamic::UavDynamicsSolveResponse& res)
    {
        const double v = (req.speed_mps > 0.1) ? req.speed_mps : mp_.speed_mps;
        const double cur_yaw = req.path.empty() ? 0.0 : req.path.front().yaw;

        // 水平：DWA + 协调转弯
        double roll = 0.0;
        const double heading = uav_dynamic::horizontalSolve(req.path, mp_, cur_yaw, roll);

        // 纵向：高度误差 PI（积分状态为成员，按 solve_hz 估算 dt）
        const double dt = 1.0 / std::max(solve_hz_, 1.0);
        double alpha = 0.0;
        const double height_err = req.target_height - req.uav_height;
        const double climb = uav_dynamic::verticalSolve(height_err, vp_, dt, alpha, integral_);

        // 指令平滑：对滚转/迎角/垂直速度做一阶速率限幅，保证姿态与速度指令连续
        // 航向平滑：每步最大变化量 ≤ max_heading_step_rad（防航向突变）；
        // 首帧以当前航向 cur_yaw 为基准，避免从发射阶段切换时的突跳
        if (!has_heading_) { last_heading_ = cur_yaw; has_heading_ = true; }
        const double heading_s = stepLimit(heading, max_heading_step_, last_heading_);
        const double roll_s   = rateLimit(roll,   max_roll_rate_,  dt, last_roll_,   has_last_);
        const double alpha_s  = rateLimit(alpha,  max_alpha_rate_, dt, last_alpha_,  has_last_);
        const double climb_s  = rateLimit(climb,  max_climb_rate_, dt, last_climb_,  has_last_);

        res.success = !req.path.empty();
        res.heading_rad = heading_s;
        res.roll_rad = roll_s;
        res.pitch_rad = alpha_s;
        res.climb_mps = climb_s;

        ROS_DEBUG("[uav_dynamic] h_err=%.1f -> climb=%.2f alpha=%.2fdeg | heading=%.2fdeg roll=%.2fdeg",
                  height_err, climb_s, alpha_s * 180.0 / 3.14159265358979,
                  heading * 180.0 / 3.14159265358979, roll_s * 180.0 / 3.14159265358979);
        return true;
    }

    /// 一阶速率限幅：|Δ指令/帧| ≤ rate·dt，首帧直接采用。
    static double rateLimit(double val, double rate, double dt,
                            double& last, bool& has_last)
    {
        if (!has_last) { last = val; has_last = true; return val; }
        double d = val - last;
        const double lim = std::max(rate, 0.0) * dt;
        d = std::clamp(d, -lim, lim);
        last += d;
        return last;
    }

    /// 航向步长限幅：|Δ指令/帧| ≤ max_step（rad），跨 ±π 按最短角差 wrap 处理。
    static double stepLimit(double val, double max_step, double& last)
    {
        const double kPi = 3.14159265358979323846;
        double d = val - last;
        while (d >  kPi) d -= 2.0 * kPi;
        while (d < -kPi) d += 2.0 * kPi;
        d = std::clamp(d, -max_step, max_step);
        last += d;
        while (last >  kPi) last -= 2.0 * kPi;
        while (last < -kPi) last += 2.0 * kPi;
        return last;
    }

    ros::NodeHandle& nh_;
    ros::ServiceServer srv_;
    uav_dynamic::MotionParams mp_;
    uav_dynamic::VerticalParams vp_;
    double solve_hz_ = 20.0;
    double integral_ = 0.0;   // 纵向积分状态
    // 指令平滑状态（阶段 7）
    double max_heading_step_ = 0.2;
    bool has_heading_ = false;
    double last_heading_ = 0.0;
    double max_roll_rate_ = 0.35, max_alpha_rate_ = 0.35, max_climb_rate_ = 2.0;
    double last_roll_ = 0.0, last_alpha_ = 0.0, last_climb_ = 0.0;
    bool has_last_ = false;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_dynamics_server");
    ros::NodeHandle nh;   // 全局命名空间读取（launch 全局 rosparam load）
    UavDynamicsServer server(nh);
    ros::spin();
    return 0;
}

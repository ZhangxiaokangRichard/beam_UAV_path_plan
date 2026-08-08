# uav_dynamic 运动解算输出指令（uav_guide_loop 调用 /uav_dynamic/solve 后发布）
std_msgs/Header header
bool success
float64 heading_rad             # 期望航向角（协调转弯律，ENU 弧度）
float64 roll_rad                # 期望滚转角
float64 pitch_rad               # 期望迎角
float64 climb_mps               # 期望垂直速度

# 巡航（cruise）制导输出：飞控按"航向 + 定高"飞行
# 发布者：uav_guidance_node → aoa/uav/guidance
# 消费：mqtt_control_bridge（mode=cruise 时编码为 MQTT set_fly_head / set_fly_height）
std_msgs/Header header

float64 fly_height    # 期望高度（高度类型 height_type 由 bridge 参数决定：0 场高 / 1 绝对高度）
float64 heads         # 期望航向，航空约定 0=正北、顺时针，单位度，范围 [0,360)
float64 fly_speed     # 期望速度 (m/s)：本轮由 yaml 巡航速度给定，为后续 set_fly_speed 预留

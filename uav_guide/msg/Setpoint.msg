# 指点（setpoint）制导输出：飞控按"点/圆盘旋"跟随
# 发布者：uav_guide_point_node → aoa/uav/setpoint
# 消费：mqtt_control_bridge（mode=setpoint 时编码为 MQTT fly_point）
std_msgs/Header header

float64 target_longitude    # WGS84 经度
float64 target_latitude     # WGS84 纬度
float64 target_altitude     # WGS84 高度 (m)
float64 target_radius       # >0 顺时针盘旋 / <0 逆时针盘旋（圆心为 ±R）；偏移点为 +finalshot.radius_m

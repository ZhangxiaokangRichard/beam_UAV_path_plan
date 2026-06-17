# 轴对齐长方体障碍物（AABB）
# 用于在规划请求中传递环境障碍物快照

float64[3] aabb_min    # [x_min, y_min, z_min] 最小角点 (m)
float64[3] aabb_max    # [x_max, y_max, z_max] 最大角点 (m)
uint32 id               # 障碍物唯一 ID（用于追踪和调试）
bool is_dynamic         # 是否为动态障碍物
float64[3] velocity     # 速度矢量 [vx, vy, vz]（静态障碍物 = [0,0,0]）

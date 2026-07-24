# 单个路径点（3D 位姿 + 曲率元数据）
# 用于规划响应中返回路径点序列

float64 x              # X 坐标 (m)
float64 y              # Y 坐标 (m)
float64 z              # Z 坐标 (m)
float64 yaw            # 偏航角 ψ (rad)，范围 (-π, π]
float64 pitch          # 俯仰角 γ (rad)，范围 [-γ_max, +γ_max]
float64 curvature      # 路径曲率 κ = 1/R（1/m），直行=0，用于速度自适应

# uav_guide_env (2.0.0)

ROS2 Humble **可视化包**（1.0.1 的精简版）。1.0.1 中该包还包含仿真闭环
（`simulation_loop` / `target_publisher` / `target_trajectory` / `uav_model` /
`obstacle_manager` / `uav_logger`）；2.0 的真实数据全部由 `ros_mqtt_bridge` 从现场 MQTT
获取，**仿真职责取消**，因此本包只保留 `uav_viz_node`（原 `rviz_publisher`）。

## 文件

| 文件 | 说明 |
|---|---|
| `src/uav_viz_node.cpp` | 3D Marker 发布节点（保留 1.0.1 的全部视觉设计） |
| `config/viz.yaml` | 空间尺寸、默认占位位置、刷新率、话题名 |
| `config/rviz/beam_dubins_3d.rviz` | **rviz2 格式**配置（1.0.1 的 rviz1 配置不兼容，已重建） |
| `launch/viz.launch.py` | 启动 `uav_viz_node` + `rviz2`（`rviz:=false` 可只起节点） |

## 保留的 1.0.1 可视化设计（逐项一致）

| 对象 | ns | 类型 | 尺寸 | 颜色 (RGBA) | 存活期 |
|---|---|---|---|---|---|
| UAV 航向 | `uav_model` | ARROW（杆长 600 m） | 杆 30 / 头 80 | 1.0, 0.45, 0.0, 1.0（橙） | 0.5 s |
| UAV 位置 | `uav_sphere` | SPHERE | 直径 100 m | 同上 | 0.5 s |
| 目标位置 | `target_model` | SPHERE | 直径 120 m | 1.0, 0.15, 0.15, 0.9（红） | 0.5 s |
| 目标航向 | `target_arrow` | ARROW（杆长 400 m） | 杆 20 / 头 60 | 1.0, 0.15, 0.15, 1.0 | 0.5 s |
| 空间边界 | `env_bounds` | LINE_LIST（12 条棱） | 线宽 3.0 m | 0.6, 0.6, 0.6, 0.3 | 持久（latched） |
| 状态文本 | `sim_status` | TEXT_VIEW_FACING | 字高 80 m | 白色 0.9 | 2.0 s |

文本 Marker 位置仍为 `(500, 4500, 950)`；刷新率 5 Hz（模型）/ 1 Hz（文本），与 1.0.1 相同。
**移除**：`/obstacles` MarkerArray（1.0.1 已废弃）、`/sim/status` 仿真状态流、
未被使用的 `makeObstacleMarker` 通道墙逻辑。

## 2.0 新增

| 新增 | 说明 |
|---|---|
| `aoa/viz/plan_text` | 规划摘要文本：`plan OK  cost=3574 m  pts=1234`（失败时附 `status_message`） |
| `aoa/viz/uav_trail` | `nav_msgs/Path` 航迹（`trail_min_step_m` 去抖 + `trail_max_points` 上限） |
| `aoa/viz/planed_path` | `uav_guide/UavPlannedPath`(**带曲率**) → `nav_msgs/Path` 转换，供 rviz2 `Path` 显示 |

## 话题

| 方向 | 话题 | 类型 | QoS |
|---|---|---|---|
| 订阅 | `aoa/uav/state` | `nav_msgs/Odometry` | 10 |
| 订阅 | `aoa/target/state` | `nav_msgs/Odometry` | 10 |
| 订阅 | `aoa/uav/planed_path` | `uav_guide/UavPlannedPath` | 10（volatile，兼容 latched 发布者） |
| 订阅 | `aoa/viz/status_text` | `std_msgs/String` | 10 |
| 发布 | `aoa/viz/env_bounds` | `Marker` | **transient_local** |
| 发布 | `aoa/viz/uav_model`、`uav_sphere`、`target_model`、`target_arrow` | `Marker` | 10 |
| 发布 | `aoa/viz/status`、`aoa/viz/plan_text` | `Marker` | 10 |
| 发布 | `aoa/viz/uav_trail`、`aoa/viz/planed_path` | `nav_msgs/Path` | transient_local |

## 用法

```bash
cd ~/beamDubins_ws && colcon build --packages-select uav_guide_env
source install/setup.bash

# 可视化（需先有 aoa/uav/state 与 aoa/target/state：ros2 launch ros_mqtt_bridge mqtt_bridge.launch.py）
ros2 launch uav_guide_env viz.launch.py
ros2 launch uav_guide_env viz.launch.py rviz:=false          # 只起 Marker 节点
```

## 重要工程约束（`geodesy` 局部 ENU 的适用范围）

`ros_mqtt_bridge` 的 `geodesy` 与 1.0.1 **逐字节相同**，使用**切平面**投影
（ECEF 差向量投影到原点 ENU 基），因此只在**作业区尺度（≲10 km）**内有效：

- 目标距原点 115 km 时，地球曲率造成的高度误差约 $d^2/2R \approx 1.0$ km
  （实测目标真实高度 700 m，却显示 `z = -385 m`）。**这不是 Bug，是切平面模型的固有误差。**
- 结论：**本机与目标必须在同一作业区内**（默认 10 km × 5 km × 1 km），
  `geodesy.origin_*` 取作业区中心；跨区/远距数据不要送入规划器。

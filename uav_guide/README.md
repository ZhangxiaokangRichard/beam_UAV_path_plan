# uav_guide

UAV 路径引导 ROS 包：订阅本机 / 目标状态，调用 `beam_dubins` 规划路径，向飞控发布 WGS84 引导点。

---

## Quick Start

```bash
# 1. 编译（首次或改代码后）
cd ~/catkin_ws
catkin_make --pkg beam_dubins ros_mqtt_bridge uav_guide
source devel/setup.bash

# 2. 终端 A：启动引导（MQTT 状态输入 + 规划 + 引导点）
roslaunch uav_guide uav_guide.launch

# 3. 终端 B：启动引导点 → MQTT 转发（fly_point）
source ~/catkin_ws/devel/setup.bash
rosrun ros_mqtt_bridge mqtt_waypoint_bridge

# 4. 终端 C：验证（可选）
source ~/catkin_ws/devel/setup.bash
rostopic hz /uav/state /target/state
rostopic echo /uav/guide_point -n 1
rostopic echo /uav_guide/intercept_mode -n 1

# 5. 切换拦截模式（发一次即可）
rostopic pub -1 /uav_guide/intercept_mode_cmd std_msgs/String "data: 'tail'"
rostopic pub -1 /uav_guide/intercept_mode_cmd std_msgs/String "data: 'head-on'"
```

**说明：**

- `uav_guide.launch` 启动 `beam_dubins`、`mqtt_ros_bridge`（MQTT→`/uav/state`、`/target/state`）、`uav_guide_loop`、`uav_guide_point`。
- `mqtt_waypoint_bridge` 需**单独** `rosrun`：订阅 `/uav/guide_point`，在引导点更新时向 MQTT 发布 `fly_point`（主题 `aoa/uav_control/${uav_sn}/event_services`）。
- MQTT 连接参数见 `ros_mqtt_bridge/config/mqtt_bridge.yaml`（默认 broker `192.168.70.62:1883`）。

---

## 1. 在工作空间中的位置

本包位于仓库 `beam_UAV_path_plan` 子目录内，与以下包同级：

```
catkin_ws/src/beam_UAV_path_plan/
├── beam_dubins/        # 路径规划服务 /beam_dubins/plan_path
├── ros_mqtt_bridge/    # MQTT ↔ state；guide_point → MQTT fly_point
└── uav_guide/          # 本包（引导逻辑）
```

**依赖：** `beam_dubins`、`ros_mqtt_bridge`、`uav_guide`

---

## 2. 编译

```bash
cd ~/catkin_ws
sudo apt install -y libmosquitto-dev python3-matplotlib   # 首次

catkin_make --pkg beam_dubins ros_mqtt_bridge uav_guide
source devel/setup.bash
```

验证：

```bash
rospack find uav_guide
rosmsg show uav_guide/UavGuidePoint
rosservice list | grep plan_path
```

---

## 3. 启动

### 3.1 主流程

| 终端 | 命令 | 作用 |
|------|------|------|
| A | `roslaunch uav_guide uav_guide.launch` | 规划 + 引导点 + MQTT 状态桥 |
| B | `rosrun ros_mqtt_bridge mqtt_waypoint_bridge` | 引导点 → MQTT `fly_point` |

两个终端均需 `source devel/setup.bash`。

`mqtt_waypoint_bridge` 行为：

- 订阅 `/uav/guide_point`（`uav_guide/UavGuidePoint`）
- 从 `/uav/state` 的 `child_frame_id` 读取 `uav_sn`
- 引导点**时间戳更新**时才发送，避免重复下发同一指令
- MQTT 主题：`aoa/uav_control/${uav_sn}/event_services`

### 3.2 拦截模式（热切换）

启动时使用 yaml 默认 `intercept_mode`（当前为 **head-on**）。运行中可通过**命令话题**切换，**发一次即可**，无需持续发布。

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/uav_guide/intercept_mode_cmd` | `std_msgs/String` | 输入 | 发送 `head-on` 或 `tail` 触发切换 |
| `/uav_guide/intercept_mode` | `std_msgs/String` | 输出（latched） | 当前生效模式 |

切换后会：**清空路径缓存、退出 TRACKING、立即重规划**（若已有 state）。

```bash
rostopic echo /uav_guide/intercept_mode -n 1
rostopic pub -1 /uav_guide/intercept_mode_cmd std_msgs/String "data: 'tail'"
rostopic pub -1 /uav_guide/intercept_mode_cmd std_msgs/String "data: 'head-on'"
```

将 `intercept_mode_topic` 设为空字符串 `""` 可禁用命令话题，仅使用 yaml 默认。

配置项见 `config/uav_guide.yaml`：`intercept_mode`、`intercept_mode_topic`、`intercept_mode_status_topic`。

---

## 4. 架构与数据流

```
MQTT (uav_info / fst_info)
    ↓  mqtt_ros_bridge
/uav/state、/target/state  (nav_msgs/Odometry, ~2Hz)
    ↓
┌─────────────────────────────────────┐
│  uav_guide_loop_node                  │
│  plan_orchestrator                    │
│    → /beam_dubins/plan_path           │
│    → /uav/planned_path                │
│    → /uav_guide/tracking_mode         │
└─────────────────────────────────────┘
    ↓
┌─────────────────────────────────────┐
│  uav_guide_point_node  (5Hz)        │
│    → /uav/guide_point  (WGS84)       │  ← 飞控跟这个，不跟 planned_path
└─────────────────────────────────────┘
    ↓  mqtt_waypoint_bridge (rosrun)
MQTT fly_point → aoa/uav_control/${uav_sn}/event_services
```

| 节点 | 脚本 / 可执行文件 | 作用 |
|------|-------------------|------|
| `uav_guide_loop` | `uav_guide_loop_node.py` | 每次 state 更新触发规划 |
| `uav_guide_point` | `uav_guide_point_node.py` | 5Hz 发布引导点 |
| `mqtt_ros_bridge` | `mqtt_target_bridge` | MQTT → `/uav/state`、`/target/state` |
| `mqtt_waypoint_bridge` | `mqtt_waypoint_bridge` | `/uav/guide_point` → MQTT `fly_point` |

---

## 5. 核心逻辑（交接必读）

系统有两层，**互相独立**：

| 层次 | 决定什么 | 判据 |
|------|----------|------|
| **引导点输出** | 飞控收到 `/uav/guide_point` | **只看路径几何**，与 TRACKING 无关 |
| **内部模式** | 规划终点 + `phase` 日志 | **共线 + 接近 + 距离**，见第 6 节 |

### 5.1 引导点（`/uav/guide_point`）

| 路径状态 | 输出 |
|----------|------|
| 还有 **未飞过** 的弧段 | 沿航路 **最远** 剩余弧段的 **圆心**（`target_radius` = ±500） |
| **无剩余弧段**（末段直线 / 全直线） | 目标偏移点（`finalshot.offset_m`，`target_radius` = +500） |

偏移方向：

- **head-on**：目标 **身后** `offset_m` 米  
- **tail**：目标 **前方** `offset_m` 米  

仍在 **最后一段圆弧上** 且该弧未飞完 → 发 **该弧圆心**，不是偏移点。

### 5.2 规划（beam_dubins）

| 内部模式 | 规划终点 | 期望 yaw |
|----------|----------|----------|
| **APPROACH** | 目标延长线上 **固定距离** 的虚拟点（见下） | head-on：ψ_tgt+180°；tail：ψ_tgt |
| **TRACKING** | **目标本体** 实时位置 | 同上 |

**虚拟点（APPROACH）：**

- 恒为：`approach_distance_headon`（迎头）或 `approach_distance_tail`（尾追）  
- head-on：目标沿航向 **前方** L 米的一个 **点**（不是圆）

### 5.3 模式与引导点的关系

典型时序：

```
APPROACH + 远弧圆心
    → 末段无弧：APPROACH + 偏移引导点
    → 共线+接近满足：TRACKING + 偏移引导点（规划改跟目标本体）
    → miss：回到 APPROACH，清路径，恢复远弧圆心规划
```

---

## 6. TRACKING 进入 / 退出

### 6.1 进入（须同时满足，连续 `entry_hold_count` 帧）

使用 state 中 **四元数转 yaw**（`uav[3]`、`target[3]`）。

| 条件 | 配置项 | 当前 yaml 默认 |
|------|--------|----------------|
| 距离 | `d ≤ tracking_entry_dist` | **1900 m** |
| 共线 | `\|wrap(ψ_uav − ψ_ref)\| ≤ velocity_align_deg` | **70°**（±70°，总宽 140°） |
| 接近 | `closure ≥ min_closure_m`（head-on） | **3 m/帧** |
| 接近 | `closure ≥ min_closure_m_tail`（tail） | **1.5 m/帧** |
| 防抖 | 连续满足帧数 `entry_hold_count` | **1 帧** |

`ψ_ref`：head-on = `ψ_target + 180°`；tail = `ψ_target`。  
`closure` = 上一帧距离 − 当前距离（相邻 state 间）。

### 6.2 退出

| 条件 | 配置项 | 当前默认 |
|------|--------|----------|
| 累计实际飞行 **100 m** 评估一次 | `tracking_window` | 100 m |
| **连续 3 次** 距离增大 | `divergence_threshold` | 3 |

退出后：`tracking_mode=false`，清空缓存路径，回到 APPROACH。

```bash
rosparam get /uav_guide/tracking_mode
```

---

## 7. 话题与消息

### 输入

| 话题 | 类型 | 说明 |
|------|------|------|
| `/uav/state` | `nav_msgs/Odometry` | 本机 map 位姿 + 四元数 yaw |
| `/target/state` | `nav_msgs/Odometry` | 目标 map 位姿 |

Geodesy 原点由 `ros_mqtt_bridge/config/mqtt_bridge.yaml` 加载；`uav_guide.yaml` 内 `geodesy` 为后备。

### 输出

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/uav/planned_path` | `uav_guide/UavPlannedPath` | ~2 Hz | 规划路径（调试 / 引导点计算） |
| `/uav/guide_point` | `uav_guide/UavGuidePoint` | 5 Hz | **飞控引导点** |

`UavGuidePoint` 字段：

```text
float64 target_longitude
float64 target_latitude
float64 target_altitude    # 目标实时高度（WGS84）
float64 target_radius      # 弧段圆心：±500；偏移引导点：+500
```

---

## 8. 配置说明

主配置文件：**`config/uav_guide.yaml`**

### 8.1 规划虚拟到达点（APPROACH）

| 参数 | 当前值 | 含义 |
|------|--------|------|
| `target.approach_distance_headon` | 1800.0 | 迎头：目标航向前方虚拟点距离 (m) |
| `target.approach_distance_tail` | 1500.0 | 尾追：目标航向后方虚拟点距离 (m) |

### 8.2 TRACKING 判定

| 参数 | 当前值 | 含义 |
|------|--------|------|
| `target.tracking_entry_dist` | 1900.0 | 进入 TRACKING 最大距离 (m) |
| `target.velocity_align_deg` | 70.0 | 共线角度容差 (°) |
| `target.min_closure_m` | 3.0 | head-on 每帧最小接近量 (m) |
| `target.min_closure_m_tail` | 1.5 | tail 每帧最小接近量 (m) |
| `target.entry_hold_count` | 1 | 连续满足帧数 |
| `target.tracking_window` | 100.0 | 退出评估窗口航程 (m) |
| `target.divergence_threshold` | 3 | 连续发散次数 |

### 8.3 偏移引导点（无剩余弧段时）

| 参数 | 当前值 | 含义 |
|------|--------|------|
| `finalshot.offset_m` | 50 | 相对目标偏移 (m) |
| `finalshot.radius_m` | 500.0 | 输出半径（+500） |

### 8.4 其它

| 参数 | 说明 |
|------|------|
| `guide_point.publish_rate_hz` | 引导点发布频率，默认 5 |
| `segmentation.*` | 路径直线/弧段切分 |
| `space.*` | 规划空间边界（与 beam_dubins 一致） |

MQTT 相关配置：**`ros_mqtt_bridge/config/mqtt_bridge.yaml`**

---

## 9. 调试

### 9.1 话题检查

```bash
source ~/catkin_ws/devel/setup.bash

rostopic hz /uav/state /target/state
rostopic hz /uav/planned_path
rostopic hz /uav/guide_point

rostopic echo /uav/guide_point -n 1
rostopic echo /uav/planned_path -n 1
```

### 9.2 日志关键字

**`uav_guide_loop` 终端：**

| 日志 | 含义 |
|------|------|
| `plan ok ... phase=APPROACH` | 规划成功，未进 TRACKING |
| `plan ok ... phase=TRACKING` | 规划成功，已进 TRACKING |
| `tracking gate (...)` | 进入判定过程（2s 节流） |
| `>>> 进入 TRACKING 模式` | 进入 TRACKING |
| `MODE TRACKING -> APPROACH (miss)` | 退出 TRACKING |
| `intercept mode -> ...` | 拦截模式热切换 |

**`mqtt_waypoint_bridge` 终端：**

| 日志 | 含义 |
|------|------|
| `fly_point sent to ...` | 引导点已通过 MQTT 下发 |
| `waiting for /uav/state to get uav_sn` | 尚未收到本机 state，无法确定 UAV SN |
| `MQTT publish failed` | broker 连接或发布失败 |

### 9.3 工具脚本

```bash
rosrun uav_guide uav_guide_dump_all_points.py
rosrun uav_guide uav_guide_route_viz.py
roslaunch uav_guide uav_guide_viz.launch
rosrun uav_guide uav_guide_viz.py
```

### 9.4 常见问题

| 现象 | 排查 |
|------|------|
| 无 `/uav/planned_path` | `beam_dubins` 是否启动；日志 `plan failed` |
| 无 `/uav/state` | MQTT broker 是否可达；`mqtt_ros_bridge` 是否运行 |
| 不进 TRACKING | 看 `tracking gate`：`align`、`closure`、`d` |
| 已在跟偏移点但 phase=APPROACH | **正常**：引导点与 TRACKING 解耦 |
| 引导点 lon/lat 异常 | 检查 `/geodesy/*` 是否与 `mqtt_bridge.yaml` 一致 |
| MQTT 无 fly_point | 是否已 `rosrun ros_mqtt_bridge mqtt_waypoint_bridge`；`/uav/guide_point` 是否有更新；`uav_sn` 是否已从 `/uav/state` 获取 |

---

## 10. Launch 文件

| 文件 | 用途 |
|------|------|
| `launch/uav_guide.launch` | 主启动（规划 + 引导点 + MQTT 状态桥） |
| `launch/uav_guide_viz.launch` | 路径可视化（route_viz） |

---

## 11. 源码索引

| 文件 | 内容 |
|------|------|
| `uav_guide/intercept_mode_manager.py` | 拦截模式 latch / 热切换 |
| `uav_guide/plan_orchestrator.py` | APPROACH/TRACKING 状态机、规划调度 |
| `uav_guide/guide_point_builder.py` | 远弧圆心 / 偏移引导点 |
| `uav_guide/state_adapter.py` | 订阅 state、提取 5D |
| `scripts/uav_guide_loop_node.py` | 规划主循环 |
| `scripts/uav_guide_point_node.py` | 引导点发布 |

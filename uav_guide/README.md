# uav_guide

UAV 路径引导 ROS 包：订阅本机 / 目标状态，调用 `beam_dubins` 规划路径，调用 `uav_dynamic` 解算运动指令（航向/滚转/俯仰/垂直速度），发布规划路径与指令。

> 原 guide_point 引导点（MQTT fly_point）功能已移除；状态输入改为 UDP 桥（`ros_udp_bridge`）。

---

## Quick Start

```bash
# 1. 编译
cd ~/catkin_ws
catkin_make --pkg beam_dubins ros_udp_bridge uav_dynamic uav_guide
source devel/setup.bash

# 2. 启动引导（UDP 状态输入 + 规划 + uav_dynamic 解算 + 指令发布）
roslaunch uav_guide uav_guide.launch

# 3. 验证
rostopic hz /uav/state /target/state          # 状态来源（ros_udp_bridge udp_receiver）
rostopic echo /uav/planned_path -n 1          # 规划路径（uav_guide/UavPlannedPath）
rostopic echo /uav/cmd -n 1                   # 运动指令（uav_guide/UavCmd）
rostopic echo /uav_guide/intercept_mode -n 1  # 拦截模式状态

# 4. 切换拦截模式（发一次即可）
rostopic pub -1 /uav_guide/intercept_mode_cmd std_msgs/String "data: 'tail'"
rostopic pub -1 /uav_guide/intercept_mode_cmd std_msgs/String "data: 'head-on'"
```

**说明：**

- `uav_guide.launch` 启动：`beam_dubins`（planner）、`ros_udp_bridge`（UDP 收/发 → `/uav/state`、`/target/state`）、`uav_dynamic`（运动解算服务）、`uav_guide_loop`。
- `uav_guide_loop`：每次状态更新 → 规划 → 成功则调用 `/uav_dynamic/solve` → 发布 `/uav/cmd`。
- 拦截模式：默认 `head-on`；通过 `/uav_guide/intercept_mode_cmd` 热切换（`uav_guide.yaml` 的 `intercept_mode_topic`）。

---

## 1. 工作空间位置

与以下包同级（`src/aos-ljd-beam-pathplan/`）：

```
├── beam_dubins/        # 路径规划服务 /beam_dubins/plan_path
├── ros_udp_bridge/     # UDP 收/发桥 + 服务入口 udp_mssn_bridge
├── uav_dynamic/        # 运动解算服务 /uav_dynamic/solve（DWA + 纵向 PI）
└── uav_guide/          # 本包（引导逻辑）
```

**依赖：** `beam_dubins`、`ros_udp_bridge`（launch 提供状态）、`uav_dynamic`（服务）。

---

## 2. 编译与依赖

```bash
cd ~/catkin_ws
catkin_make --pkg beam_dubins ros_udp_bridge uav_dynamic uav_guide
```

依赖：`rospy`、`std_msgs`、`nav_msgs`、`beam_dubins`（消息/服务）、`uav_dynamic`（服务，运行时）。

---

## 3. 节点

| 节点 | 文件 | 作用 |
|------|------|------|
| `uav_guide_loop` | `scripts/uav_guide_loop_node.py` | 主循环：状态更新 → 规划 → uav_dynamic 解算 → 发布 `/uav/cmd` |

核心模块（`uav_guide/`）：

| 模块 | 作用 |
|------|------|
| `state_adapter.py` | 订阅 `/uav/state`、`/target/state`（Odometry）→ 提取 5D 状态 |
| `intercept_mode_manager.py` | 拦截模式：yaml 默认 + `/uav_guide/intercept_mode_cmd` 热切换，发布 `/uav_guide/intercept_mode` |
| `plan_orchestrator.py` | 规划调度：APPROACH / TRACKING 状态机，调用 `/beam_dubins/plan_path` |
| `finalshot.py` | 末段偏移点计算 `compute_offset_goal_xy`（head-on/tail） |
| `field_extractor.py` | 状态字段提取 / 角度 wrap |
| `geodesy.py` | WGS84 ↔ map ENU |

---

## 4. 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/uav/state` | nav_msgs/Odometry | ← | 本机状态（ros_udp_bridge） |
| `/target/state` | nav_msgs/Odometry | ← | 目标状态（ros_udp_bridge） |
| `/uav/planned_path` | uav_guide/UavPlannedPath | → | 规划路径（latch） |
| `/uav/cmd` | uav_guide/UavCmd | → | 运动指令 heading/roll/pitch/climb（latch） |
| `/uav_guide/intercept_mode_cmd` | std_msgs/String | ← | 模式切换指令 head-on/tail |
| `/uav_guide/intercept_mode` | std_msgs/String | → | 当前模式（latch） |
| `/uav_guide/tracking_mode` | rosparam(bool) | – | TRACKING 进出状态 |

---

## 5. 配置（config/uav_guide.yaml）

- `intercept_mode` / `intercept_mode_topic` / `intercept_mode_status_topic`：拦截模式默认值与切换/状态话题
- `planning.service`：规划服务名
- `target.*`：APPROACH/TRACKING 切换阈值
- `finalshot.*`：末段偏移量（`offset_m`/`offset_m_t`/`radius_m`，plan_orchestrator 使用）
- `dynamics.speed_mps`：传给 `/uav_dynamic/solve` 的水平速度
- `output.planned_path_topic` / `output.dynamics_cmd_topic`：发布话题

---

## 6. 服务入口（外部指令）

外部通过 UDP 向 `ros_udp_bridge` 的 `udp_mssn_bridge`（本地 11302）发送指令可远程启停 `uav_guide.launch` 与切换拦截模式：

```bash
echo '{"type":"nav","start":true}' | nc -u -w1 127.0.0.1 11302      # 启动 uav_guide.launch
echo '{"type":"mode","mode":"head-on"}' | nc -u -w1 127.0.0.1 11302 # 切换迎头
echo '{"type":"nav","start":false}' | nc -u -w1 127.0.0.1 11302     # 停止
```

详见 `ros_udp_bridge/scripts/UDP_INTERFACE.md`。

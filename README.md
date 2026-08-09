# beam_UAV_path_plan

ROS1 (Noetic) workspace source for UAV path planning based on Beam Search + Dubins primitives.

## Packages

- `beam_dubins`: Core planner library, ROS messages/services, and planner server（仅水平 xy 路径规划）.
- `uav_guide`: UAV 引导主循环核心 `guide_core.GuideCore`（发射/规划/DWA/state_update/relaunch/crashed）；
  发射许可由 `/uav/launch_cmd`（std_msgs/Bool）门控——未获许可且仍停发射起点时待命不推进，离开起点后不再检测；
  `uav_guide_loop_node.py` 发布 `/uav/state`（→ `udp_sender` → JSON-UDP → 真实外部）.
- `uav_dynamic`: 运动解算服务 `/uav_dynamic/solve`（水平跟踪 horizontalSolve + 纵向高度 PI；
  航向快速调整：取 Dubins 最后一个圆弧入口切线航向 + 协调转弯 roll）.
- `ros_udp_bridge`: UDP 桥：
  - `udp_receiver`：收 UDP → `/target/state`（经纬高 → ENU 直解 + 差分速度/姿态角速度）+
    `/uav/init_pos` + `/uav/init_heading` + `/uav/relaunch`（warmup 3s，未收到 uav 用默认起飞点）
  - `udp_sender`：订阅 `/uav/state` → 逆变换 geodetic → red_interceptor JSON-UDP 外发
  - `udp_mssn_bridge`（极简）：10Hz 订阅 `/uav/state` + `/target/state` 算欧氏距离，
    < 阈值（7km）→ 发布 `/uav/launch_cmd`；订阅 `/target/crashed` → 命中协议 UDP；
    启动时 tmux 依次拉起 `uav_guide.launch`（主循环）与 `uav_bridge.launch`（receiver+sender）
- `uav_guide_env`: RViz 可视化辅助（`rviz_publisher`：订阅 uav/target 状态与规划路径，发布球体+箭头 Marker）.

## Build

```bash
cd ~/catkin_ws
catkin_make
```

## Run

### 真实任务（推荐入口：tmux 一键拉起全套）

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch ros_udp_bridge udp_mssn_bridge.launch
```

启动后自动用 tmux 依次拉起两个独立窗口：
- `uav_guide`  → `roslaunch uav_guide uav_guide.launch`（planner + uav_dynamic + 主循环 uav_guide_loop）
- `uav_bridge` → `roslaunch ros_udp_bridge uav_bridge.launch`（udp_receiver + udp_sender）

流程：主循环等待 `/target/state` → target 发布 → 主循环更新并发布 `/uav/state` → `udp_sender` 外发 →
`udp_mssn_bridge` 按距离（< 7km）发布 `/uav/launch_cmd=true` → 主循环发射；命中（dist < 20m）→
`/target/crashed` → 命中协议 UDP 上报。

### 主循环（单独启动，需自行准备 UDP 桥）

```bash
roslaunch uav_guide uav_guide.launch
```

### RViz 可视化（对接运行中的节点，仅启动 rviz，不重复开节点）

```bash
roslaunch uav_guide_env start_guide_external.launch
```

仅启动 `rviz_publisher` + `rviz`，数据来自已运行的真实节点话题（`/uav/state`、`/target/state` 等）。

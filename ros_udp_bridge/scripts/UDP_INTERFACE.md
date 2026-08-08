# ros_udp_bridge UDP 接口设计文档

> 本文档汇总 `ros_udp_bridge` 功能包使用的**全部 UDP 接口**：端口、载荷（JSON）、用途、使用方式。
> 该功能包替代原 `ros_mqtt_bridge`（MQTT 通信 → 本地 UDP 通信），并新增 `udp_mssn_bridge` 服务入口。

---

## 1. 端口总览

| 端口 | 方向 | 用途 | 节点 |
|------|------|------|------|
| 11301 | 收（bind 0.0.0.0） | 现场 UDP 位姿数据（red_interceptor / blue_uav / blue） | `udp_receiver` |
| 9100 | 发（remote_host:9100） | 本机 red_interceptor 发送（含 targetKey） | `udp_sender` |
| 11302 | 收（bind 0.0.0.0） | **服务入口指令**（启停 launch / 切换拦截模式） | `udp_mssn_bridge` |
| 11303 | 发（可选） | 服务状态回传 JSON | `udp_mssn_bridge` |

---

## 2. 数据流（收 / 发）

### 2.1 接收（现场 → 本机）

- 绑定：`udp/local_host:udp/local_port`（默认 `0.0.0.0:11301`）
- 报文：一包一 JSON，UTF-8（非法字节自动清洗为 U+FFFD）
- 过滤：按 `key` 字段
  - `bridge/uav_key`（默认字符串 `"425201827841"`）→ 本机拦截弹，用于 uav 初始化
  - `bridge/target_key`（默认字符串 `"30064967681"`）→ 蓝方目标，卡尔曼滤波后发布 `/target/state`
- 坐标锚点：`geodesy/origin_*`（默认 `30.337989, 104.152629, 493.0`）

接收报文 JSON 字段：

```json
{
  "key": 425201827841,
  "type": "red_interceptor",   // red_interceptor | blue_uav | blue
  "name": "IM-01",
  "t": 1786186475.76,
  "lat": 30.3398,
  "lon": 104.1537,
  "alt": 1003.0,
  "heading": 61.35,            // 正北 0° 顺时针（度）
  "pitch": 0.0,
  "roll": 0.0,
  "velocity": 50.0,
  "targetKey": 30064967681,
  "event": "FIRE"
}
```

### 2.2 发送（本机 → 现场）

- 订阅 `/uav/state`（20Hz）→ 逆变换为经纬高 → 组装 `red_interceptor` JSON → UDP 发送
- 目标：`udp/remote_host:udp/remote_port`（默认 `10.119.228.21:9100`，config 配置）
- 首帧带 `event: "FIRE"`

发送 JSON 字段（数值为 JSON number，非字符串）：

```json
{"key":425201827841,"type":"red_interceptor","name":"IM-01","t":1786186475.76,
 "lat":30.3398,"lon":104.1537,"alt":1003.0,"heading":61.35,"pitch":0.0,"roll":0.0,
 "targetKey":30064967681,"event":"FIRE"}
```

---

## 3. 服务入口指令（udp_mssn_bridge，端口 11302）

> 该节点为整个服务的入口：接收本地 UDP 指令，执行进程管理 / 模式切换。
> 指令与状态均为一包一 JSON。**指令 JSON 数值/布尔为 JSON 类型（非字符串）。**

### 3.1 指令格式

```json
{"type":"nav",  "start":true}          // 启动 uav_guide.launch（含 planner/udp_bridge/uav_dynamic/uav_guide_loop）
{"type":"nav",  "start":false}         // 停止 uav_guide.launch
{"type":"guide","start":true}          // 启动可配置辅助进程（默认未配置则忽略）
{"type":"guide","start":false}         // 停止辅助进程
{"type":"mode", "mode":"head-on"}      // 切换为迎头拦截
{"type":"mode", "mode":"tail"}         // 切换为尾追拦截
```

### 3.2 指令示例（nc / 脚本）

```bash
# 启动导航服务
echo '{"type":"nav","start":true}' | nc -u -w1 127.0.0.1 11302
# 切换迎头模式（会先确保 launch 已启动，见 ensure_nav_on_mode）
echo '{"type":"mode","mode":"head-on"}' | nc -u -w1 127.0.0.1 11302
# 停止导航服务
echo '{"type":"nav","start":false}' | nc -u -w1 127.0.0.1 11302
```

### 3.3 指令执行动作

| 指令 | 动作 |
|------|------|
| `nav start=true` | 若 `uav_guide.launch` 未运行则启动（xterm / tmux / 后台日志） |
| `nav start=false` | 停止 `uav_guide.launch` |
| `guide start=true` | 启动 `mssn/guide_package` + `mssn/guide_executable`（默认空，不启用） |
| `mode` | 发布 `/uav_guide/intercept_mode_cmd`（head-on / tail）；`ensure_nav_on_mode=true` 时先确保 launch 已启动 |

### 3.4 状态回传（可选）

- 开关：`cmd/report_enable`（默认 `false`）
- 频率：`cmd/status_poll_hz`（默认 2Hz）
- 目标：`cmd/remote_host:cmd/remote_port`（默认 `127.0.0.1:11303`）

状态 JSON：

```json
{"type":"status","nav":true,"guide":false,"tracking":false,
 "mode":"head-on","uav":[1000.0,1000.0,500.0],"target":[4500.0,2500.0,600.0]}
```

字段：
- `nav`：`uav_guide.launch` 是否运行
- `guide`：辅助进程是否运行
- `tracking`：`/uav_guide/tracking_mode` 是否进入 TRACKING
- `mode`：当前拦截模式（`/uav_guide/intercept_mode`）
- `uav` / `target`：本机 / 目标位置 `[x, y, z]`

---

## 4. ROS 话题桥接

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/target/state` | nav_msgs/Odometry | udp_receiver → | 目标位姿（卡尔曼 40Hz） |
| `/uav/state` | nav_msgs/Odometry | udp_receiver → | 本机位姿（20Hz，起点） |
| `/uav/init_pos` | geometry_msgs/PoseStamped | udp_receiver → | 初始位置（1Hz latch） |
| `/uav/init_heading` | std_msgs/Float64 | udp_receiver → | 初始朝向（1Hz latch） |
| `/uav_guide/intercept_mode_cmd` | std_msgs/String | udp_mssn_bridge → | 模式切换指令（head-on/tail） |
| `/uav_guide/intercept_mode` | std_msgs/String | ← | 当前模式状态（订阅缓存） |
| `/uav/cmd_roll|pitch|climb` | std_msgs/Float64 | simulation_loop → | uav_dynamic 控制指令（调试） |

---

## 5. 部署

- 启动：`bash scripts/start_udp_mssn_bridge.sh`（起 roscore + udp_mssn_bridge，tmux）
- 停止：`bash scripts/stop_udp_mssn_bridge.sh`
- systemd 自启动：`scripts/beam_dubins_autostart.service`（ExecStart/ExecStop 指向上述脚本）

```bash
sudo cp scripts/beam_dubins_autostart.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now beam_dubins_autostart
```

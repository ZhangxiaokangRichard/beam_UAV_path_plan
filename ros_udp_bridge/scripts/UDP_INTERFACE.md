# ros_udp_bridge UDP 接口设计文档

> 本文档汇总 `ros_udp_bridge` 功能包使用的**全部 UDP 接口**：端口、载荷（JSON）、用途、使用方式。
> 该功能包替代原 `ros_mqtt_bridge`（MQTT 通信 → 本地 UDP 通信），并新增 `udp_mssn_bridge` 服务入口。

---

## 1. 端口总览

| 端口 | 方向 | 用途 | 节点 |
|------|------|------|------|
| 11301 | 收（bind 0.0.0.0） | 现场 UDP 位姿数据（red_interceptor / blue_uav / blue） | `udp_receiver` |
| 9100 | 发（remote_host:9100） | 本机 red_interceptor 发送（含 targetKey） | `udp_sender` |
| 11302 | —（已移除） | 极简版不再收服务入口指令 | — |
| 11303 | —（已移除） | 极简版不再状态回传 | — |

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

## 3. udp_mssn_bridge（极简版职责，无 11302 指令）

> 极简版已**移除** 11302 指令接收与 11303 状态回传。该节点职责仅为：

| 职责 | 说明 |
|------|------|
| 发射距离判断 | 订阅 `/uav/state` + `/target/state`，`mission/check_hz`(10Hz) 算欧氏距离，`dist < launch_dist_m`(7km) → 发布 `/uav/launch_cmd`（std_msgs/Bool，latch） |
| 命中上报 | 订阅 `/target/crashed`，true 时按命中协议经 UDP 发 `encodeHitEvent(target_key, t)` 到 `cmd/remote_host:cmd/remote_port`(9100) |
| 自动拉起 | 启动时 tmux 依次拉起 `uav_guide.launch`（主循环）与 `uav_bridge.launch`（udp_receiver + udp_sender）两个窗口 |

发射许可由主循环 `guide_core` 消费：`launch_cmd=false` 且仍停发射原点 → 待命；离开原点后不再检测。

---

## 4. ROS 话题桥接

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/target/state` | nav_msgs/Odometry | udp_receiver → | 目标位姿（位置直解+差分速度/姿态角速度） |
| `/uav/state` | nav_msgs/Odometry | uav_guide_loop → | 本机位姿（主循环发布，供 udp_sender 外发） |
| `/uav/init_pos` | geometry_msgs/PoseStamped | udp_receiver → | 初始位置（latch） |
| `/uav/init_heading` | std_msgs/Float64 | udp_receiver → | 初始朝向（latch） |
| `/uav/relaunch` | std_msgs/Bool | udp_receiver → | 位置重置信号 |
| `/uav/launch_cmd` | std_msgs/Bool | udp_mssn_bridge → | 发射许可（距离<阈值，latch） |
| `/target/crashed` | std_msgs/Bool | uav_guide_loop → | 命中判定 |

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

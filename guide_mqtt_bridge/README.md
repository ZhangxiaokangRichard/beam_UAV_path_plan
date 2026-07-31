# guide_mqtt_bridge

Linux 虚拟机侧 MQTT 命令桥，**独立于** `beam_UAV_path_plan`，不修改仿真仓库。

通过 **192.168.70.62:1883** 接收 Windows 控制台指令，在 VM 内执行 `roslaunch` / `rostopic` / `rosrun`。

---

## 依赖

```bash
sudo apt install -y tmux python3-pip
pip3 install -r requirements.txt
```

---

## 配置

编辑 `config.yaml`：

- `mqtt.host`：Broker 地址（默认 `192.168.70.62`）
- `ros.setup_shell`：ROS 与 catkin 的 source 路径（按 VM 实际修改）

---

## 启动（虚拟机）

```bash
cd ~/guide_mqtt_bridge   # 或拷贝到 VM 的任意目录
python3 guide_mqtt_bridge.py
```

建议保持该终端运行；或使用 `tmux new -s guide_bridge 'python3 guide_mqtt_bridge.py'`。

---

## MQTT 协议

### 命令 `beam_dubins/guide/cmd`

```json
{"method": "session_start"}
{"method": "strike_start"}
{"method": "strike_stop"}
{"method": "set_intercept_mode", "data": {"mode": "head-on"}}
{"method": "set_intercept_mode", "data": {"mode": "tail"}}
```

### 状态 `beam_dubins/guide/status`

```json
{
  "schema": "beam_dubins.guide_status.v1",
  "intercept_mode": "head-on",
  "strike_state": "planning",
  "guide_launch_running": true,
  "waypoint_bridge_running": false,
  "tracking_mode": false
}
```

`strike_state`：`idle` | `planning` | `striking` | `stopped`

---

## 行为说明

| 命令 | 动作 |
|------|------|
| `session_start` | 启动 `uav_guide.launch`（若未运行），不启 `mqtt_waypoint_bridge` |
| `strike_start` | 启动 `mqtt_waypoint_bridge`，向 70.62 发 fly_point |
| `strike_stop` | 停止 `mqtt_waypoint_bridge`，规划继续 |
| `set_intercept_mode` | `rostopic pub` 切换迎头/尾追 |

进程通过 **tmux** 后台运行，SSH 断开不影响。

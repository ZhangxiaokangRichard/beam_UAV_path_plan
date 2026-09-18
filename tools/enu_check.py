#!/usr/bin/env python3
"""ENU 坐标交叉校验工具：把 MQTT 原始大地坐标用**独立 Python WGS84 实现**换算成 ENU，
再与 mqtt_target_bridge 发布的 aoa/uav/state、aoa/target/state 对比。

用途：换作业区/改原点后，验证「坐标原点 + 换算实现」是否正确（现场必做）。

用法：
  source install/setup.bash
  ros2 launch ros_mqtt_bridge mqtt_bridge.launch.py &        # 先让桥跑起来
  python3 tools/enu_check.py --config install/ros_mqtt_bridge/share/ros_mqtt_bridge/config/mqtt_bridge.yaml

说明：
  ROS 消息时间戳取自**接收时刻**，与 MQTT 报文时间戳不同源；本工具按时间戳就近配对，
  飞机机动时残差主要来自链路时延（~50 m/s 时 1 s ≈ 50 m），故默认容差 150 m。
  若想看实现级精度，请用 --strict（要求配对时间差 < 120 ms，容差 5 m）。
"""
import argparse
import json
import math
import re
import subprocess
import sys
import threading
import time

K_A = 6378137.0
K_E2 = 6.6943799901413165e-3


def ecef(lat_deg, lon_deg, alt_m):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    r = K_A / math.sqrt(1.0 - K_E2 * sl * sl)
    return ((r + alt_m) * cl * co, (r + alt_m) * cl * so, (r * (1.0 - K_E2) + alt_m) * sl)


def enu(lat_deg, lon_deg, alt_m, origin):
    ox, oy, oz = ecef(*origin)
    x, y, z = ecef(lat_deg, lon_deg, alt_m)
    dx, dy, dz = x - ox, y - oy, z - oz
    lat0, lon0 = math.radians(origin[0]), math.radians(origin[1])
    sl, cl = math.sin(lat0), math.cos(lat0)
    so, co = math.sin(lon0), math.cos(lon0)
    return (-so * dx + co * dy,
            -sl * co * dx - sl * so * dy + cl * dz,
            cl * co * dx + cl * so * dy + sl * dz)


def read_origin(path):
    """从 mqtt_bridge.yaml 读取 geodesy.origin_*（不依赖 pyyaml）。"""
    txt = open(path).read()
    def grab(key):
        m = re.search(rf"^\s*{key}\s*:\s*([-\d.eE+]+)", txt, re.M)
        if not m:
            sys.exit(f"无法从 {path} 读取 {key}")
        return float(m.group(1))
    return (grab("origin_latitude_deg"), grab("origin_longitude_deg"), grab("origin_altitude_m"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="install/ros_mqtt_bridge/share/ros_mqtt_bridge/config/mqtt_bridge.yaml")
    ap.add_argument("--host", default="192.168.70.62")
    ap.add_argument("--uav-topic", default="aoa/uav/state")
    ap.add_argument("--target-topic", default="aoa/target/state")
    ap.add_argument("--capture-s", type=float, default=6.0)
    ap.add_argument("--strict", action="store_true", help="仅用时间差 <120 ms 的配对，容差 5 m")
    args = ap.parse_args()

    origin = read_origin(args.config)
    print(f"原点 (from {args.config}): lat={origin[0]} lon={origin[1]} alt={origin[2]}")

    samples, lock = [], threading.Lock()
    proc = subprocess.Popen(["mosquitto_sub", "-h", args.host, "-t", "aoa/uav_center/+/uav_info", "-v"],
                            stdout=subprocess.PIPE, text=True)

    def reader():
        for line in proc.stdout:
            _, _, payload = line.strip().partition(" ")
            try:
                top = json.loads(payload)
                d = top["data"]
                with lock:
                    samples.append((d["uav_sn"], int(top["timestamp"]),
                                    (d["latitude"], d["longitude"], d["altitude"])))
            except Exception:
                pass

    threading.Thread(target=reader, daemon=True).start()
    time.sleep(args.capture_s)
    # ROS echo 与 MQTT 采集并行：这里采集已完成，改用最近样本配对
    ros = []
    for topic in (args.uav_topic, args.target_topic):
        out = subprocess.run(["ros2", "topic", "echo", "--once", topic],
                             capture_output=True, text=True, timeout=25).stdout
        sn = re.search(r"^child_frame_id:\s*(\S+)", out, re.M)
        st = re.search(r"sec:\s*(\d+)\s*\n\s*nanosec:\s*(\d+)", out)
        pos = re.search(r"position:\s*\n\s*x:\s*([-\d.eE+]+)\s*\n\s*y:\s*([-\d.eE+]+)\s*\n\s*z:\s*([-\d.eE+]+)", out)
        if sn and st and pos:
            ros.append((topic, sn.group(1), int(st.group(1)) + int(st.group(2)) * 1e-9,
                        tuple(float(pos.group(i)) for i in (1, 2, 3))))
        else:
            print(f"!! {topic} 未取到 sn/stamp/position（桥是否在跑？话题名是否正确？）")
    proc.terminate()

    tol = 5.0 if args.strict else 150.0
    print(f"\n{'topic':18s} {'sn':22s} {'Δt(ms)':>8s} {'ROS x':>10s} {'py x':>10s} "
          f"{'ROS y':>10s} {'py y':>10s} {'ROS z':>8s} {'py z':>8s} {'d3D(m)':>8s}  判定")
    n, bad = 0, 0
    for topic, sn, stamp, p in ros:
        cand = [(ts, geo) for s, ts, geo in samples if s == sn]
        if not cand:
            print(f"{topic:18s} {sn:22s}  !! 未采到该机 MQTT 报文（sn/filter 是否匹配？）")
            bad += 1
            continue
        ts, geo = min(cand, key=lambda s: abs(s[0] / 1000.0 - stamp))
        dt_ms = (stamp - ts / 1000.0) * 1000.0
        if args.strict and abs(dt_ms) > 120.0:
            continue
        ex, ny, up = enu(*geo, origin)
        d = math.dist(p, (ex, ny, up))
        n += 1
        fail = d > tol
        bad += fail
        print(f"{topic:18s} {sn:22s} {dt_ms:8.0f} {p[0]:10.1f} {ex:10.1f} {p[1]:10.1f} {ny:10.1f} "
              f"{p[2]:8.1f} {up:8.1f} {d:8.1f}  {'OK' if not fail else 'BAD'}")
    print()
    if n == 0:
        print("结论: 无效（无配对，可能未采集到报文或时间窗不重叠）")
        return 1
    print(f"结论: {'通过' if bad == 0 else '失败'} —— {n - bad}/{n} 项在 {tol:.0f} m 容差内"
          f"{'（strict：实现级精度）' if args.strict else '（含链路时延残差）'}")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

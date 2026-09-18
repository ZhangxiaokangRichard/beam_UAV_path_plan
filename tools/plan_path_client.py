#!/usr/bin/env python3
"""beam_dubins 规划服务测试客户端（精简输出，避免打印上千个路径点）。

用法示例：
  python3 tools/plan_path_client.py                                  # 默认 2 km 直飞用例
  python3 tools/plan_path_client.py --goal 458 4965 400 --start 0 0 200
  python3 tools/plan_path_client.py --obstacle 700 300 0 1300 900 1000   # xmin ymin zmin xmax ymax zmax
  python3 tools/plan_path_client.py --space 10000 5000 1000 --margin 50
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from beam_dubins.srv import PlanPath
from beam_dubins.msg import Obstacle


def main() -> int:
    ap = argparse.ArgumentParser(description="beam_dubins plan_path 测试客户端")
    ap.add_argument("--service", default="/aoa/beam_dubins/plan_path")
    ap.add_argument("--start", nargs=5, type=float, default=[0.0, 0.0, 200.0, 0.0, 0.0],
                    metavar=("x", "y", "z", "yaw", "pitch"))
    ap.add_argument("--goal", nargs=5, type=float, default=[2000.0, 1000.0, 200.0, 0.0, 0.0],
                    metavar=("x", "y", "z", "yaw", "pitch"))
    ap.add_argument("--space", nargs=3, type=float, default=[10000.0, 5000.0, 1000.0],
                    metavar=("lx", "ly", "lz"))
    ap.add_argument("--margin", type=float, default=50.0)
    ap.add_argument("--obstacle", nargs=6, type=float, action="append", default=[],
                    metavar=("xmin", "ymin", "zmin", "xmax", "ymax", "zmax"))
    ap.add_argument("--use-3d", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = Node("plan_path_test_client")
    client = node.create_client(PlanPath, args.service)
    if not client.wait_for_service(timeout_sec=5.0):
        print(f"服务不可用: {args.service}")
        rclpy.shutdown()
        return 2

    req = PlanPath.Request()
    req.header.frame_id = "map"
    req.start = args.start
    req.goal = args.goal
    req.space_lx, req.space_ly, req.space_lz = args.space
    req.boundary_margin = args.margin
    req.use_3d = args.use_3d
    req.time_limit_ms = 0.0
    for i, o in enumerate(args.obstacle):
        obs = Obstacle()
        obs.aabb_min = list(o[0:3])
        obs.aabb_max = list(o[3:6])
        obs.id = i + 1
        obs.is_dynamic = False
        obs.velocity = [0.0, 0.0, 0.0]
        req.obstacles.append(obs)

    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60.0)
    res = fut.result()
    if res is None:
        print("服务调用超时/失败")
        rclpy.shutdown()
        return 3

    dx = req.goal[0] - req.start[0]
    dy = req.goal[1] - req.start[1]
    straight = (dx * dx + dy * dy) ** 0.5
    print(f"success        = {res.success}")
    print(f"status_message = {res.status_message!r}")
    print(f"cost           = {res.cost:.2f} m   (直线距离 {straight:.1f} m, 绕行比 {res.cost/straight:.3f})")
    print(f"path points    = {len(res.path)}")
    if res.path:
        p0, pn = res.path[0], res.path[-1]
        print(f"first point    = ({p0.x:.1f}, {p0.y:.1f}, {p0.z:.1f}) yaw={p0.yaw:+.3f} k={p0.curvature:.5f}")
        print(f"last  point    = ({pn.x:.1f}, {pn.y:.1f}, {pn.z:.1f}) yaw={pn.yaw:+.3f} k={pn.curvature:.5f}")
        print(f"goal error     = {((pn.x-req.goal[0])**2 + (pn.y-req.goal[1])**2)**0.5:.3f} m")
        kz = sum(1 for p in res.path if abs(p.curvature) > 1e-4)
        print(f"弧段点/直段点   = {kz} / {len(res.path) - kz}")
    print(f"nodes_explored = {res.nodes_explored}   depth = {res.depth_reached}   "
          f"time = {res.planning_time_ms:.1f} ms")
    rclpy.shutdown()
    return 0 if res.success else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uav 引导主循环核心（仿真/真实共用，阶段 8 重构）

simulation_loop（仿真模式）与 uav_guide_loop_node（真实模式）共用本核心：
发射 / 规划 / DWA / state_update / relaunch / crashed 判定完全一致，
二者唯一区别是 uav 状态的输出端：
  - uav_guide_loop_node（真实模式）：on_state → 发布 uav/uav_state
    → udp_sender 编码 JSON-UDP → 发送到真实外部
  - simulation_loop（仿真模式）    ：on_state → 广播 TF + 航迹 → rviz 仿真

主循环（20Hz 定时器驱动）：
 1. 发射阶段：节点启动后按当前朝向/姿态向本地坐标系运动，发布 uav/cmd 共 launch_frames 条
 2. 规划：uav_state（本核心维护）+ target_state（/target/state）→ /beam_dubins/plan_path
    （beam_dubins 仅水平 xy 路径规划，只输入水平坐标 x/y 与朝向 yaw，见 plan_orchestrator）
 3. DWA 控制：path → /uav_dynamic/solve → 发布 uav/cmd（heading/roll/pitch/climb）；
    纵向（高度）由 uav_dynamic 高度 PI 控制
 4. state_update：接受 uav/cmd → 运动学推进 → on_state 回调（节点决定输出端）
 5. relaunch：订阅 uav/relaunch（udp_receiver 检测到新 uav UDP 即位置重置）→ 返回发射起点
 6. tracking 模式检测（plan_orchestrator 状态机，维护 /uav_guide/tracking_mode）
 7. 目标距离判定：dist(uav_state, target_state) < crashed_dist_m → 发布 /target/crashed
 8. 命中 → 沿当前状态持续输出；未命中 → 重新发射（回第 1 步）
"""

from __future__ import annotations

import math
from typing import Callable, List, Optional

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from uav_dynamic.srv import UavDynamicsSolve, UavDynamicsSolveRequest

from uav_guide.geodesy import Geodesy
from uav_guide.intercept_mode_manager import InterceptModeManager
from uav_guide.msg import UavCmd
from uav_guide.plan_orchestrator import PlanOrchestrator

G = 9.81


class GuideCore:
    def __init__(
        self,
        config: dict,
        on_state: Optional[Callable[[List[float]], None]] = None,
    ):
        """引导主循环核心。

        on_state(state)：每帧 state_update 之后回调，参数为 [x, y, z, yaw, pitch]。
        节点通过该回调决定 uav 状态输出端（真实 → uav/uav_state；仿真 → rviz）。
        """
        self._config = config
        self._on_state = on_state or (lambda s: None)

        # ── 参数 ──
        output = config.get("output", {})
        self._cmd_topic = output.get("dynamics_cmd_topic", "/uav/cmd")
        self._crashed_topic = output.get("crashed_topic", "/target/crashed")
        self._relaunch_topic = output.get("relaunch_topic", "/uav/relaunch")
        self._launch_cmd_topic = output.get("launch_cmd_topic", "/uav/launch_cmd")

        loop_cfg = config.get("loop", {})
        self._speed_mps = float(config.get("dynamics", {}).get("speed_mps", 50.0))
        self._dt = float(loop_cfg.get("dt", 0.05))
        self._launch_frames_max = int(loop_cfg.get("launch_frames", 100))
        self._plan_interval = int(loop_cfg.get("plan_interval", 2))
        self._crashed_dist_m = float(loop_cfg.get("crashed_dist_m", 20.0))

        # 发射起点：优先 start_geodetic（经纬高 → ENU 本地坐标），否则 start（ENU 本地坐标）
        self._start_state = self._resolve_start(config)
        self._uav_state = list(self._start_state)

        # 内部状态
        self._launch_frames_remaining = self._launch_frames_max
        self._hit = False
        self._tick_count = 0
        self._relaunch_pending = False
        self._launch_allowed = False   # 发射许可：/uav/launch_cmd（udp_mssn_bridge 距离判断）
        self._target_state = None
        self._path = []
        self._uav_state_ready = False   # uav/state 是否已由主循环 state_update 更新（规划前判断用）
        self._uav_roll = 0.0            # 最近一次 uav/cmd 滚转指令（用于完整姿态四元数输出）
        self._tracking_mode_prev = False  # tracking 模式状态（进入/退出检测）

        # ── 发布器：uav/cmd + /target/crashed + 规划路径 + 全局原点（供 rviz_publisher） ──
        self._cmd_pub = rospy.Publisher(self._cmd_topic, UavCmd, queue_size=1, latch=True)
        self._crashed_pub = rospy.Publisher(self._crashed_topic, Bool, queue_size=1, latch=True)
        self._path_pub = rospy.Publisher("/uav/planned_path", Path, queue_size=10)
        self._origin_pub = rospy.Publisher("/beam_dubins/origin_pos", PoseStamped, queue_size=1, latch=True)
        self._start_pub = rospy.Publisher("/uav/start_pos", PoseStamped, queue_size=1, latch=True)  # 发射起点（ENU）
        # 发布全局原点（map 系 ENU 原点，rviz 图形中心定位用）
        origin = PoseStamped()
        origin.header.stamp = rospy.Time.now()
        origin.header.frame_id = "map"
        origin.pose.orientation.w = 1.0
        self._origin_pub.publish(origin)
        # 发布发射起点（ENU 本地坐标）
        self._publish_start_pos()

        # ── 订阅：目标状态 + 重新发射信号 + 发射许可 ──
        rospy.Subscriber("/target/state", Odometry, self._on_target, queue_size=10)
        rospy.Subscriber(self._relaunch_topic, Bool, self._on_relaunch, queue_size=10)
        rospy.Subscriber(self._launch_cmd_topic, Bool, self._on_launch_cmd, queue_size=10)

        # ── 规划器（APPROACH/TRACKING 状态机）+ 拦截模式 ──
        self._orchestrator = PlanOrchestrator(config, self._on_planned)
        self._mode_manager = InterceptModeManager(config, on_mode_change=self._on_mode_change)

        # ── uav_dynamic 客户端（DWA 控制） ──
        self._solve_client = rospy.ServiceProxy("/uav_dynamic/solve", UavDynamicsSolve)
        rospy.wait_for_service("/uav_dynamic/solve", timeout=10.0)

        # ── 主循环定时器（20Hz） ──
        self._timer = rospy.Timer(rospy.Duration(self._dt), self._tick)
        rospy.loginfo(
            "[guide_core] ready: launch=%d帧 dt=%.2fs speed=%.1fm/s "
            "crashed_dist=%.1fm",
            self._launch_frames_max, self._dt, self._speed_mps, self._crashed_dist_m,
        )

    # ── 对外只读状态（供节点输出端使用） ──

    @property
    def uav_state(self) -> List[float]:
        return list(self._uav_state)

    @property
    def target_state(self) -> Optional[List[float]]:
        return list(self._target_state) if self._target_state is not None else None

    @property
    def dt(self) -> float:
        return self._dt

    @property
    def mode(self) -> str:
        return self._mode_manager.get_mode()

    @property
    def path(self):
        """当前规划路径点列表（beam_dubins.PathPoint[]，供 rviz 显示）。"""
        return list(self._path) if self._path else []

    @property
    def uav_state_ready(self) -> bool:
        return self._uav_state_ready

    # ── 订阅回调 ──

    def _on_target(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        # 完整姿态角提取（roll/pitch/yaw）
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        sp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sp)))
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        self._target_state = [
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.pose.pose.position.z, yaw, pitch, roll,
        ]

    def _on_relaunch(self, msg: Bool) -> None:
        if msg.data:
            rospy.loginfo("[guide_core] relaunch 收到 -> 返回发射起点")
            self._relaunch_pending = True

    def _on_launch_cmd(self, msg: Bool) -> None:
        self._launch_allowed = bool(msg.data)
        rospy.loginfo_throttle(1.0, "[guide_core] launch_cmd -> %s",
                               "true(允许发射)" if self._launch_allowed else "false(待命)")

    def _on_mode_change(self, _old: str, new: str) -> None:
        self._orchestrator.reset_on_mode_change(new)

    def _on_planned(self, resp) -> None:
        if resp.success:
            self._path = list(resp.path)
            self._publish_path()

    def _publish_path(self) -> None:
        """把当前规划路径发布到 /uav/planned_path（供 rviz_publisher / rviz 显示）。"""
        p = Path()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        for wp in self._path:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = float(wp.x)
            ps.pose.position.y = float(wp.y)
            ps.pose.position.z = float(wp.z)
            ps.pose.orientation.w = 1.0
            p.poses.append(ps)
        self._path_pub.publish(p)

    def _resolve_start(self, config) -> List[float]:
        """发射起点：优先 start_geodetic（经纬高 → ENU 本地坐标），否则 start（ENU）。"""
        sg = config.get("start_geodetic")
        if sg:
            try:
                lat = float(sg.get("latitude_deg"))
                lon = float(sg.get("longitude_deg"))
                alt = float(sg.get("altitude_m", 0.0))
                heading = float(sg.get("heading_deg", 0.0))
                e, n, u = self._geodesy().to_local(lat, lon, alt)
                yaw = math.radians(90.0 - heading)   # 正北 0°顺时针 → ENU yaw（0=东）
                rospy.loginfo(
                    "[guide_core] 发射起点：经纬高 (%.7f, %.7f, %.2f) → ENU (%.2f, %.2f, %.2f) "
                    "heading=%.1fdeg yaw=%.3frad", lat, lon, alt, e, n, u, heading, yaw)
                return [e, n, u, yaw, 0.0]
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn("[guide_core] start_geodetic 转换失败，回退 start：%s", exc)
        start = config.get("start", [0.0, 0.0, 500.0, 0.0, 0.0])
        return [float(v) for v in start[:5]]

    def _geodesy(self) -> Geodesy:
        """读取 geodesy 锚点（与 ros_udp_bridge 一致），构建 Geodesy。"""
        geo_cfg = self._config.get("geodesy", {})
        lat = rospy.get_param("/geodesy/origin_latitude_deg",
                              float(geo_cfg.get("origin_latitude_deg", 0.0)))
        lon = rospy.get_param("/geodesy/origin_longitude_deg",
                              float(geo_cfg.get("origin_longitude_deg", 0.0)))
        alt = rospy.get_param("/geodesy/origin_altitude_m",
                              float(geo_cfg.get("origin_altitude_m", 0.0)))
        return Geodesy(lat, lon, alt)

    def _publish_start_pos(self) -> None:
        """发布发射起点（ENU 本地坐标，PoseStamped latch，/uav/start_pos）。"""
        x, y, z, yaw, _pitch = self._start_state
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        ps.pose.orientation.w = cy
        ps.pose.orientation.z = sy
        self._start_pub.publish(ps)

    # ── 主循环 ──

    def _tick(self, _evt) -> None:
        # 5. 启动位置检查：relaunch → 返回发射起点
        if self._relaunch_pending:
            self._relaunch_pending = False
            self._reset_to_launch()

        target = self._target_state
        if target is None:
            return  # 目标数据未就绪

        mode = self._mode_manager.get_mode()

        # 6. tracking 状态变化检测（进入/退出日志）
        tracking = bool(self._orchestrator.tracking_mode)
        if tracking != self._tracking_mode_prev:
            if tracking:
                rospy.loginfo("[guide_core] ➤ 进入 TRACKING 模式（直飞目标本体）")
            else:
                rospy.loginfo("[guide_core] ➤ 退出 TRACKING 模式 → APPROACH")
            self._tracking_mode_prev = tracking

        # 1b. 发射许可：uav 仍停在发射起点（未发射）且 launch_cmd=false → 待命不推进
        if (self._launch_frames_remaining == self._launch_frames_max
                and not self._launch_allowed):
            rospy.logwarn_throttle(1.0,
                "[guide_core] 待命：等待发射许可（/uav/launch_cmd=true）...")
            self._emit_idle()   # 持续发布原点静止的 uav/state 供外部监控
            return

        # 1. 发射阶段（前 launch_frames 帧沿当前朝向运动） / 3. DWA
        if self._launch_frames_remaining > 0:
            if self._launch_frames_remaining == self._launch_frames_max:
                rospy.loginfo("[guide_core] ★ 发射信号：进入发射阶段（%d 帧，沿当前朝向/姿态）",
                              self._launch_frames_max)
            cmd = self._launch_cmd()
            self._launch_frames_remaining -= 1
        else:
            # 2. 规划：请求 beam_dubins 前判断 uav/state 是否已由主循环更新。
            #    若未就绪（如第一次循环尚未 state_update）则跳过本次规划，
            #    等下一步 uav/state 被主循环更新后再进入导航服务。
            if not self._uav_state_ready:
                rospy.logwarn_throttle(1.0,
                    "[guide_core] uav/state 未就绪 → 跳过 beam_dubins 规划，等待主循环更新")
            elif self._tick_count % self._plan_interval == 0:
                self._orchestrator.plan_on_update(self._uav_state, target, mode)
            # 3. DWA → uav/cmd
            cmd = self._solve_dynamics(target)

        # 4. 发布 uav/cmd + state_update → on_state 回调
        self._emit_cmd(cmd)

        # 每帧 uav/state 更新后打印 uav 与 target 欧氏距离
        d = math.hypot(self._uav_state[0] - target[0],
                       self._uav_state[1] - target[1],
                       self._uav_state[2] - target[2])
        rospy.loginfo("[guide_core] uav_state 更新：uav=(%.1f,%.1f,%.1f) target=(%.1f,%.1f,%.1f) dist=%.2fm",
                      self._uav_state[0], self._uav_state[1], self._uav_state[2],
                      target[0], target[1], target[2], d)

        # 7. 目标距离判定（攻击有效判定）
        self._check_crash(target)

        self._tick_count += 1

    # ── 待命状态发布（未获发射许可时） ──

    def _emit_idle(self) -> None:
        """未获发射许可时：发布原点静止的 uav/state（不推进位置、不产生控制指令）。"""
        x, y, z, yaw, pitch = self._uav_state
        self._on_state([x, y, z, yaw, pitch, self._uav_roll])

    # ── 发射阶段指令（沿当前朝向/姿态直线运动） ──

    def _launch_cmd(self) -> UavCmd:
        yaw = self._uav_state[3]
        pitch = self._uav_state[4]
        cmd = UavCmd()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "map"
        cmd.success = True
        cmd.heading_rad = yaw
        cmd.roll_rad = 0.0
        cmd.pitch_rad = pitch
        cmd.climb_mps = self._speed_mps * math.sin(pitch)
        return cmd

    # ── DWA 控制（path → uav_dynamic → uav/cmd） ──

    def _solve_dynamics(self, target) -> UavCmd:
        cmd = UavCmd()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "map"
        cmd.success = False
        if not self._path:
            # 无规划路径 → 沿当前航向巡航（发射阶段指令）
            return self._launch_cmd()
        try:
            req = UavDynamicsSolveRequest()
            req.path = list(self._path)
            req.uav_height = float(self._uav_state[2])
            req.target_height = float(target[2])
            req.speed_mps = self._speed_mps
            resp = self._solve_client(req)
            cmd.success = bool(resp.success)
            cmd.heading_rad = float(resp.heading_rad)
            cmd.roll_rad = float(resp.roll_rad)
            cmd.pitch_rad = float(resp.pitch_rad)
            cmd.climb_mps = float(resp.climb_mps)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5.0, "[guide_core] uav_dynamic solve failed: %s", exc)
        return cmd

    # ── 发布 uav/cmd + state_update ──

    def _emit_cmd(self, cmd: UavCmd) -> None:
        cmd.header.stamp = rospy.Time.now()
        self._cmd_pub.publish(cmd)
        self._state_update(cmd)

    # ── uav_state_update：接受 uav/cmd 结构 → 运动学推进 → 回调输出端 ──

    def _state_update(self, cmd: UavCmd) -> None:
        x, y, z, yaw, _pitch = self._uav_state
        v = self._speed_mps
        dt = self._dt

        heading = cmd.heading_rad
        climb = cmd.climb_mps
        gamma = math.asin(max(-1.0, min(1.0, climb / v))) if v > 0 else 0.0
        x += v * math.cos(gamma) * math.cos(heading) * dt
        y += v * math.cos(gamma) * math.sin(heading) * dt
        z += v * math.sin(gamma) * dt

        # 航向角直接采用指令航向（horizontalSolve 输出的进入最后圆弧前的切线航向），
        # 快速调整航向跟上 target；roll 为协调转弯姿态角（horizontalSolve 已算好）
        yaw = heading
        roll = cmd.roll_rad

        # 姿态：俯仰取指令迎角（限幅后由 uav_dynamic 保证），滚转记录瞬时指令
        self._uav_state = [x, y, z, yaw, float(cmd.pitch_rad)]
        self._uav_roll = float(cmd.roll_rad)
        self._uav_state_ready = True    # uav/state 已更新 → 后续可请求 beam_dubins
        # on_state 传完整状态 [x, y, z, yaw, pitch, roll]（含滚转 → 完整姿态四元数输出）
        self._on_state([x, y, z, yaw, float(cmd.pitch_rad), self._uav_roll])

    # ── 目标距离判定（攻击有效） ──

    def _check_crash(self, target) -> None:
        d = math.hypot(self._uav_state[0] - target[0],
                       self._uav_state[1] - target[1],
                       self._uav_state[2] - target[2])
        hit = d < self._crashed_dist_m
        if hit and not self._hit:
            self._hit = True
            self._publish_crashed(True)
            rospy.loginfo("[guide_core] ★ 打击命中 target d=%.1fm", d)
        elif not hit and self._hit:
            # 未命中 → 返回发射阶段
            self._hit = False
            self._publish_crashed(False)
            rospy.logwarn("[guide_core] 打击未命中 d=%.1fm -> 重新发射", d)
            self._reset_to_launch()

    def _publish_crashed(self, val: bool) -> None:
        m = Bool()
        m.data = val
        self._crashed_pub.publish(m)

    # ── 返回发射起点 ──

    def _reset_to_launch(self) -> None:
        """relaunch / 未命中：返回发射起点（第 1 点），重新进入发射阶段。"""
        self._uav_state = list(self._start_state)
        self._launch_frames_remaining = self._launch_frames_max
        self._hit = False
        self._launch_allowed = False   # relaunch 后需重新等待发射许可（launch_cmd=true）
        self._path = []
        self._publish_path()   # 清空规划路径显示
        mode = self._mode_manager.get_mode()
        self._orchestrator.reset_on_mode_change(mode)
        rospy.loginfo("[guide_core] reset to launch start")

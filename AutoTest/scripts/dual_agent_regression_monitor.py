#!/usr/bin/env python3
"""
@file dual_agent_regression_monitor.py
@brief 多機（2～4 台）SITL 回歸：起飛懸停、單機 MOVE、YAW、一字編隊、全體 MOVE、
       燕行隊形切換、取消編隊、時鐘偏差注入與全體降落

腳本必須在 Agent Docker 容器內執行，透過既有 command topic 下達命令，並以
AgentState、AgentAttitude、PlannerStatus 判定結果。時鐘偏差注入只發布一個不存在的
同儕（agent 7）、位於 1 km 外且時間戳偏移的 AgentTrajectory，不影響受測機。
每個步驟輸出一行 STEP_RESULT，最後輸出 REGRESSION_RESULT。檔名沿用雙機版本，
以 --agents 指定機數（預設 2；最終展示以 3 機驗收）。
"""

import argparse
import itertools
import math
import sys
import time
from typing import Callable, Dict, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from std_msgs.msg import String

from taiphoon_agent.msg import AgentAttitude
from taiphoon_agent.msg import AgentState
from taiphoon_agent.msg import AgentTrajectory
from taiphoon_agent.msg import PlannerStatus

EARTH_RADIUS_M = 6378137.0
FAKE_AGENT_ID = 7
FAKE_STAMP_OFFSET_S = -3.0
FAKE_DISTANCE_M = 1000.0
# 每台 Agent 的 BasicController 與 ego_swarm_planner 都訂閱 /uav_all_agent/command。
SUBSCRIBERS_PER_AGENT = 2


def wrap_degrees(angle_deg: float) -> float:
    """將角度正規化到 [-180, 180)。"""
    return (angle_deg + 180.0) % 360.0 - 180.0


def quaternion_yaw_deg(q: List[float]) -> float:
    """以 NED 慣例自四元數 [w, x, y, z] 取得 yaw（度，順時針為正）。"""
    w, x, y, z = (float(value) for value in q)
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


class MultiAgentRegression(Node):
    """以 topic 觀測驅動多機回歸步驟。"""

    def __init__(self, args: argparse.Namespace):
        super().__init__("multi_agent_regression_monitor")
        self.args = args
        self.agent_ids = tuple(args.agent_ids)
        # 單機 MOVE 由第一台執行，YAW 由第二台執行；只有一台時兩者都由它執行。
        self.mover_id = self.agent_ids[0]
        self.yaw_agent_id = (self.agent_ids[1] if len(self.agent_ids) > 1
                             else self.agent_ids[0])
        self.states: Dict[int, AgentState] = {}
        self.yaw_deg: Dict[int, float] = {}
        # YAW 步驟期間記錄 (PX4 時間戳秒, yaw 度)，用來估計轉向角速率。
        self.yaw_trace: Optional[List[Tuple[float, float]]] = None
        self.yaw_trace_agent = self.yaw_agent_id
        self.reasons: Dict[int, str] = {}
        # 每台曾出現過的 PlannerStatus.reason，用來確認預期的阻擋原因。
        self.seen_reasons: Dict[int, set] = {i: set() for i in self.agent_ids}
        self.reference: Optional[Tuple[float, float]] = None
        self.ground_altitude_m: Dict[int, float] = {}
        self.minimum_distance_m = math.inf
        # 每個步驟期間的最近機間距離，於 STEP_RESULT 輸出後重設。
        self.step_minimum_distance_m = math.inf
        self.started_s = time.monotonic()
        self.last_trace_s = 0.0
        self.results: List[Tuple[str, bool, str]] = []

        for agent_id in self.agent_ids:
            self.create_subscription(
                AgentState, f"/uav_{agent_id}_agent/state",
                lambda msg, current=agent_id: self.state_cb(msg, current), 20)
            self.create_subscription(
                AgentAttitude, f"/uav_{agent_id}_agent/attitude",
                lambda msg, current=agent_id: self.attitude_cb(msg, current),
                20)
            self.create_subscription(
                PlannerStatus, f"/uav_{agent_id}_agent/planning/status",
                lambda msg, current=agent_id: self.status_cb(msg, current),
                20)
        self.all_command_pub = self.create_publisher(
            String, "/uav_all_agent/command", 10)
        self.agent_command_pub = {
            agent_id: self.create_publisher(
                String, f"/uav_{agent_id}_agent/command", 10)
            for agent_id in self.agent_ids
        }
        self.trajectory_pub = self.create_publisher(
            AgentTrajectory, "/uav_all_agent/swarm/agent_trajectories", 10)

    # ------------------------------------------------------------------
    # 觀測
    # ------------------------------------------------------------------
    def state_cb(self, msg: AgentState, agent_id: int) -> None:
        latitude, longitude, altitude = (float(v) for v in msg.position_global)
        if not all(math.isfinite(v) for v in (latitude, longitude, altitude)):
            return
        self.states[agent_id] = msg
        if self.reference is None:
            self.reference = (latitude, longitude)
        if msg.landed or not msg.armed:
            self.ground_altitude_m.setdefault(agent_id, altitude)
        if len(self.states) == len(self.agent_ids) and self.flying():
            distance = self.minimum_pairwise_distance()
            self.minimum_distance_m = min(self.minimum_distance_m, distance)
            self.step_minimum_distance_m = min(
                self.step_minimum_distance_m, distance)
            now = time.monotonic()
            if now - self.last_trace_s >= 0.5:
                # 每 0.5 秒保存各機位置，供離線分析接近事件發生的步驟與時間。
                self.last_trace_s = now
                positions = " ".join(
                    "uav_{}=({:.2f},{:.2f},{:.2f})".format(i, *self.position(i))
                    for i in self.agent_ids)
                print(f"TRACE_POS t={now - self.started_s:.1f} "
                      f"min_peer_m={distance:.2f} {positions}", flush=True)

    def attitude_cb(self, msg: AgentAttitude, agent_id: int) -> None:
        if all(math.isfinite(float(v)) for v in msg.q):
            self.yaw_deg[agent_id] = quaternion_yaw_deg(list(msg.q))
            if (self.yaw_trace is not None and
                    agent_id == self.yaw_trace_agent):
                stamp_s = msg.stamp.sec + msg.stamp.nanosec * 1.0e-9
                self.yaw_trace.append((stamp_s, self.yaw_deg[agent_id]))

    def status_cb(self, msg: PlannerStatus, agent_id: int) -> None:
        self.reasons[agent_id] = msg.reason
        self.seen_reasons[agent_id].add(msg.reason)

    def position(self, agent_id: int) -> Tuple[float, float, float]:
        """回傳相對共同參考點的 North／East（公尺）與相對地面高度。"""
        msg = self.states[agent_id]
        latitude, longitude, altitude = (float(v) for v in msg.position_global)
        reference_latitude, reference_longitude = self.reference
        north = math.radians(latitude - reference_latitude) * EARTH_RADIUS_M
        east = (math.radians(longitude - reference_longitude) *
                EARTH_RADIUS_M * math.cos(math.radians(reference_latitude)))
        ground = self.ground_altitude_m.get(agent_id, altitude)
        return north, east, altitude - ground

    def distance_between(self, first: int, second: int) -> float:
        a = self.position(first)
        b = self.position(second)
        return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def minimum_pairwise_distance(self) -> float:
        pairs = list(itertools.combinations(self.agent_ids, 2))
        if not pairs:
            return math.inf
        return min(self.distance_between(a, b) for a, b in pairs)

    def horizontal_drift(self, start: Dict[int, Tuple[float, float, float]],
                         agent_id: int) -> float:
        now = self.position(agent_id)
        return math.hypot(now[0] - start[agent_id][0],
                          now[1] - start[agent_id][1])

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------
    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, condition: Callable[[], bool],
                   timeout_s: float, hold_s: float = 0.0) -> bool:
        deadline = time.monotonic() + timeout_s
        satisfied_since: Optional[float] = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                is_satisfied = condition()
            except (KeyError, TypeError):
                is_satisfied = False
            now = time.monotonic()
            if is_satisfied:
                if satisfied_since is None:
                    satisfied_since = now
                if now - satisfied_since >= hold_s:
                    return True
            else:
                satisfied_since = None
        return False

    def wait_for_command_subscribers(self) -> bool:
        """確認所有 Agent 都已配對全體命令 topic，避免命令只送到部分機體。"""
        expected = SUBSCRIBERS_PER_AGENT * len(self.agent_ids)
        return self.wait_until(
            lambda: self.all_command_pub.get_subscription_count() >= expected,
            30.0)

    def send_all(self, command: str) -> None:
        self.get_logger().info(f"send /uav_all_agent/command: {command}")
        self.all_command_pub.publish(String(data=command))
        self.spin_for(0.2)

    def send_agent(self, agent_id: int, command: str) -> None:
        self.get_logger().info(f"send /uav_{agent_id}_agent/command: {command}")
        self.agent_command_pub[agent_id].publish(String(data=command))
        self.spin_for(0.2)

    def record(self, name: str, passed: bool, detail: str) -> bool:
        self.results.append((name, passed, detail))
        step_minimum = ("n/a" if not math.isfinite(self.step_minimum_distance_m)
                        else f"{self.step_minimum_distance_m:.2f}")
        print(f"STEP_RESULT step={name} pass={passed} {detail} "
              f"step_min_peer_m={step_minimum}", flush=True)
        self.step_minimum_distance_m = math.inf
        return passed

    def flying(self) -> bool:
        return all(self.states[i].armed and not self.states[i].landed
                   for i in self.agent_ids)

    def altitudes(self) -> List[float]:
        return [round(self.position(i)[2], 2) for i in self.agent_ids]

    # ------------------------------------------------------------------
    # 步驟
    # ------------------------------------------------------------------
    def step_takeoff_hover(self) -> bool:
        if not self.wait_until(lambda: len(self.states) == len(self.agent_ids)
                               and all(i in self.yaw_deg
                                       for i in self.agent_ids), 60.0):
            return self.record("takeoff_hover", False, "reason=no_state")
        if not self.wait_for_command_subscribers():
            return self.record("takeoff_hover", False,
                               "reason=command_subscribers_missing")
        self.spin_for(2.0)
        initial_yaw = dict(self.yaw_deg)
        self.send_all("MODE_OFFBOARD")
        self.spin_for(3.0)
        self.send_all("ARM")
        self.spin_for(2.0)
        self.send_all(f"TAKEOFF {self.args.takeoff_altitude_m}")
        target = self.args.takeoff_altitude_m - 0.5
        reached = self.wait_until(
            lambda: self.flying() and all(
                self.position(i)[2] >= target for i in self.agent_ids),
            60.0, hold_s=2.0)
        if not reached:
            return self.record("takeoff_hover", False,
                               f"reason=altitude_timeout altitudes_m="
                               f"{self.altitudes()}")
        start = {i: self.position(i) for i in self.agent_ids}
        max_horizontal_drift = 0.0
        max_vertical_drift = 0.0
        deadline = time.monotonic() + self.args.hover_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            for i in self.agent_ids:
                max_horizontal_drift = max(max_horizontal_drift,
                                           self.horizontal_drift(start, i))
                max_vertical_drift = max(
                    max_vertical_drift, abs(self.position(i)[2] - start[i][2]))
        yaw_change = max(abs(wrap_degrees(self.yaw_deg[i] - initial_yaw[i]))
                         for i in self.agent_ids)
        passed = max_horizontal_drift <= 0.5 and max_vertical_drift <= 0.3
        return self.record(
            "takeoff_hover", passed,
            f"altitudes_m={self.altitudes()} hover_s={self.args.hover_s} "
            f"max_horizontal_drift_m={max_horizontal_drift:.3f} "
            f"max_vertical_drift_m={max_vertical_drift:.3f} "
            f"takeoff_yaw_change_deg={yaw_change:.1f}")

    def step_single_move(self) -> bool:
        mover = self.mover_id
        start = {i: self.position(i) for i in self.agent_ids}
        self.send_agent(mover, f"MOVE_FORWARD {self.args.move_m}")
        passed = self.wait_until(
            lambda: self.horizontal_drift(start, mover) >=
            self.args.move_m - 0.5,
            45.0, hold_s=2.0)
        others = [round(self.horizontal_drift(start, i), 2)
                  for i in self.agent_ids if i != mover]
        passed = passed and all(value <= 1.0 for value in others)
        return self.record(
            "single_move", passed,
            f"mover=uav_{mover} "
            f"moved_m={self.horizontal_drift(start, mover):.2f} "
            f"others_drift_m={others} "
            f"reason='{self.reasons.get(mover, '')}'")

    @staticmethod
    def peak_yaw_rate_deg_s(trace: List[Tuple[float, float]],
                            window_s: float) -> float:
        """以 window_s 秒滑動視窗估計峰值偏航角速率，降低 10 Hz 取樣抖動。"""
        peak = 0.0
        start = 0
        for end in range(1, len(trace)):
            while (start < end and
                   trace[end][0] - trace[start + 1][0] >= window_s):
                start += 1
            elapsed = trace[end][0] - trace[start][0]
            if elapsed < window_s:
                continue
            swept = sum(wrap_degrees(trace[i][1] - trace[i - 1][1])
                        for i in range(start + 1, end + 1))
            peak = max(peak, abs(swept) / elapsed)
        return peak

    def step_yaw(self) -> bool:
        agent = self.yaw_agent_id
        start_yaw = self.yaw_deg[agent]
        start_altitude = self.position(agent)[2]
        self.yaw_trace = []
        command_time = time.monotonic()
        self.send_agent(agent, f"YAW {self.args.yaw_deg}")
        max_altitude_deviation = 0.0
        reached_time: Optional[float] = None

        def rotated() -> bool:
            nonlocal max_altitude_deviation, reached_time
            max_altitude_deviation = max(
                max_altitude_deviation,
                abs(self.position(agent)[2] - start_altitude))
            change = wrap_degrees(self.yaw_deg[agent] - start_yaw)
            is_reached = abs(change - self.args.yaw_deg) <= 5.0
            if is_reached and reached_time is None:
                reached_time = time.monotonic()
            return is_reached

        passed = self.wait_until(rotated, 30.0, hold_s=1.0)
        change = wrap_degrees(self.yaw_deg[agent] - start_yaw)
        trace = self.yaw_trace
        self.yaw_trace = None
        if trace:
            # 保留轉向軌跡供離線檢查 PX4 追蹤暫態（相對時間秒, yaw 度）。
            origin = trace[0][0]
            samples = " ".join(f"{t - origin:.2f}:{yaw:.1f}" for t, yaw in trace)
            print(f"TRACE_YAW agent={self.yaw_trace_agent} {samples}", flush=True)
        peak_rate = self.peak_yaw_rate_deg_s(trace, 0.5)
        duration = (reached_time - command_time
                    if reached_time is not None else math.nan)
        # 限速的主要證據是轉向時間：到達容許誤差內至少需理論時間的 80%
        # （瞬間跳轉時 PX4 以 MC_YAWRATE_MAX 追趕，遠短於此）。峰值角速率只作
        # 上限防呆：SIH 實測 PX4 起步落後約 10 度後以約 1.5 倍限速追趕，故取 2 倍。
        limit = self.args.yaw_rate_limit_deg_s
        minimum_duration = 0.8 * max(abs(self.args.yaw_deg) - 5.0, 0.0) / limit
        duration_ok = math.isfinite(duration) and duration >= minimum_duration
        rate_ok = peak_rate <= limit * 2.0
        altitude_ok = max_altitude_deviation <= self.args.max_yaw_altitude_deviation_m
        return self.record(
            "yaw", passed and duration_ok and rate_ok and altitude_ok,
            f"agent=uav_{agent} yaw_change_deg={change:.1f} "
            f"yaw_duration_s={duration:.2f} "
            f"min_duration_s={minimum_duration:.2f} "
            f"peak_yaw_rate_deg_s={peak_rate:.1f} "
            f"yaw_rate_limit_deg_s={self.args.yaw_rate_limit_deg_s} "
            f"max_altitude_deviation_m={max_altitude_deviation:.3f}")

    def wait_formation_converged(self, timeout_s: float) -> bool:
        return self.wait_until(
            lambda: all("formation_converged" in self.reasons.get(i, "")
                        for i in self.agent_ids),
            timeout_s, hold_s=2.0)

    def step_formation(self, name: str, command: str) -> bool:
        """下達編隊命令並確認全部收斂；一字與燕行隊形的最近機間距離皆為 spacing。"""
        self.send_all(command)
        passed = self.wait_formation_converged(120.0)
        distance = self.minimum_pairwise_distance()
        passed = passed and abs(distance - self.args.formation_spacing_m) <= 1.0
        return self.record(
            name, passed,
            f"min_pairwise_distance_m={distance:.2f} "
            f"altitudes_m={self.altitudes()} "
            f"reasons={[self.reasons.get(i, '') for i in self.agent_ids]}")

    def step_group_move(self) -> bool:
        start = {i: self.position(i) for i in self.agent_ids}
        self.send_all(f"MOVE_FORWARD {self.args.move_m}")
        passed = self.wait_until(
            lambda: all(self.position(i)[0] - start[i][0] >=
                        self.args.move_m - 1.0 for i in self.agent_ids),
            90.0, hold_s=1.0)
        converged = self.wait_formation_converged(60.0)
        north = [round(self.position(i)[0] - start[i][0], 2)
                 for i in self.agent_ids]
        return self.record(
            "group_move", passed and converged,
            f"north_moved_m={north} converged={converged}")

    def step_move_blocked(self) -> bool:
        """實機預設沒有障礙物地圖（obstacle_source=none）：MOVE 必須原地等待並回報缺少地圖。"""
        mover = self.mover_id
        start = {i: self.position(i) for i in self.agent_ids}
        self.seen_reasons[mover].clear()
        self.send_agent(mover, f"MOVE_FORWARD {self.args.move_m}")
        self.spin_for(10.0)
        drift = max(self.horizontal_drift(start, i) for i in self.agent_ids)
        blocked_by_map = any("obstacle map is missing" in reason
                             for reason in self.seen_reasons[mover])
        return self.record(
            "move_blocked_without_obstacle_map",
            drift <= 0.5 and blocked_by_map and self.flying(),
            f"mover=uav_{mover} max_drift_m={drift:.3f} "
            f"blocked_by_missing_map={blocked_by_map} "
            f"reasons={sorted(self.seen_reasons[mover])}")

    def step_formation_rejected(self) -> bool:
        """單機下達編隊必須被拒絕（至少 2 台），飛機維持原位懸停。"""
        start = {i: self.position(i) for i in self.agent_ids}
        self.send_all(f"FORMATION_HORI_LINEAR {self.args.formation_spacing_m} "
                      f"{self.args.formation_altitude_m}")
        self.spin_for(10.0)
        drift = max(self.horizontal_drift(start, i) for i in self.agent_ids)
        vertical = max(abs(self.position(i)[2] - start[i][2])
                       for i in self.agent_ids)
        converged = any("formation" in self.reasons.get(i, "").lower()
                        for i in self.agent_ids)
        return self.record(
            "formation_rejected_single",
            drift <= 0.5 and vertical <= 0.3 and not converged and self.flying(),
            f"max_drift_m={drift:.3f} max_vertical_m={vertical:.3f} "
            f"reasons={[self.reasons.get(i, '') for i in self.agent_ids]}")

    def step_cancel_hold(self) -> bool:
        self.send_all("FORMATION_CANCEL")
        self.spin_for(2.0)
        start = {i: self.position(i) for i in self.agent_ids}
        self.spin_for(5.0)
        drift = max(self.horizontal_drift(start, i) for i in self.agent_ids)
        return self.record("formation_cancel_hold",
                           drift <= 0.5 and self.flying(),
                           f"max_drift_m={drift:.3f}")

    def publish_fake_trajectory(self) -> None:
        """發布 agent 7 的終點佔位軌跡：位於第一台北方 1 km，時間戳早 3 秒（接收端估計 +3 秒）。"""
        latitude, longitude, altitude = (
            float(v) for v in self.states[self.mover_id].position_global)
        fake_latitude = latitude + math.degrees(FAKE_DISTANCE_M / EARTH_RADIUS_M)
        now_ns = (self.get_clock().now().nanoseconds +
                  int(FAKE_STAMP_OFFSET_S * 1e9))
        stamp = TimeMsg(sec=now_ns // 1_000_000_000,
                        nanosec=now_ns % 1_000_000_000)
        message = AgentTrajectory()
        message.stamp = stamp
        message.start_time = stamp
        message.trajectory_id = 1
        message.agent_index = FAKE_AGENT_ID
        message.coordinate_frame = AgentTrajectory.COORDINATE_FRAME_WGS84_GEODETIC
        message.frame_revision = 1
        message.order = 3
        message.knot_interval_s = 1.0
        message.duration_s = 1.0
        message.terminal_hold = True
        message.valid = True
        message.control_points = [fake_latitude, longitude, altitude] * 4
        self.trajectory_pub.publish(message)

    def step_skew_injection(self) -> bool:
        start = {i: self.position(i) for i in self.agent_ids}
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            self.publish_fake_trajectory()
            self.spin_for(0.1)
        self.spin_for(2.0)
        drift = max(self.horizontal_drift(start, i) for i in self.agent_ids)
        # WARN log 由外層腳本自容器 log 檢查；此處確認受測機不受影響。
        return self.record("skew_injection_isolated",
                           drift <= 0.5 and self.flying(),
                           f"fake_agent={FAKE_AGENT_ID} "
                           f"stamp_offset_s={FAKE_STAMP_OFFSET_S} "
                           f"max_drift_m={drift:.3f}")

    def step_land(self) -> bool:
        self.send_all("MODE_LAND")
        passed = self.wait_until(
            lambda: all(self.states[i].landed for i in self.agent_ids), 90.0,
            hold_s=1.0)
        return self.record(
            "land", passed,
            f"landed={[bool(self.states[i].landed) for i in self.agent_ids]} "
            f"altitudes_m={self.altitudes()}")

    def run(self) -> int:
        spacing = self.args.formation_spacing_m
        altitude = self.args.formation_altitude_m
        if self.args.expect_move_blocked:
            steps = (
                self.step_takeoff_hover,
                self.step_move_blocked,
                self.step_yaw,
                self.step_skew_injection,
            )
        elif len(self.agent_ids) == 1:
            steps = (
                self.step_takeoff_hover,
                self.step_single_move,
                self.step_yaw,
                self.step_formation_rejected,
                self.step_skew_injection,
            )
        else:
            steps = (
                self.step_takeoff_hover,
                self.step_single_move,
                self.step_yaw,
                lambda: self.step_formation(
                    "formation_hori_linear",
                    f"FORMATION_HORI_LINEAR {spacing} {altitude}"),
                self.step_group_move,
                lambda: self.step_formation(
                    "formation_v_shape",
                    f"FORMATION_V_SHAPE {spacing} {altitude}"),
                self.step_cancel_hold,
                self.step_skew_injection,
            )
        for step in steps:
            if not step():
                break
        # 不論前面成功與否都要讓飛機落地，避免 SITL 留在空中。
        self.step_land()
        if len(self.agent_ids) == 1:
            self.record("minimum_peer_distance", True, "single_agent=n/a")
        else:
            minimum_distance_ok = (
                self.minimum_distance_m >= self.args.min_distance_m)
            self.record("minimum_peer_distance", minimum_distance_ok,
                        f"minimum_distance_m={self.minimum_distance_m:.2f} "
                        f"limit_m={self.args.min_distance_m}")
        expected_results = len(steps) + 2
        passed = (len(self.results) == expected_results and
                  all(result[1] for result in self.results))
        print(f"REGRESSION_RESULT pass={passed} "
              f"agents={','.join(str(i) for i in self.agent_ids)} "
              f"steps={len(self.results)}/{expected_results}", flush=True)
        return 0 if passed else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--agents", type=int, default=2, choices=(2, 3, 4))
    parser.add_argument("--takeoff-altitude-m", type=float, default=8.0)
    parser.add_argument("--hover-s", type=float, default=10.0)
    parser.add_argument("--move-m", type=float, default=5.0)
    parser.add_argument("--yaw-deg", type=float, default=90.0)
    # 需與 Agent 的 yaw_rate_limit_deg_s 參數一致（預設 30 deg/s）。
    parser.add_argument("--yaw-rate-limit-deg-s", type=float, default=30.0)
    parser.add_argument("--max-yaw-altitude-deviation-m", type=float,
                        default=0.5)
    parser.add_argument("--formation-spacing-m", type=float, default=4.0)
    parser.add_argument("--formation-altitude-m", type=float, default=10.0)
    parser.add_argument("--min-distance-m", type=float, default=2.0)
    # 指定實際參與的機號（例如 2 或 2,3），用於驗證缺少 uav_1 的組合；
    # 未指定時使用 1..--agents。
    parser.add_argument("--agent-ids", type=str, default="")
    # 驗證實機預設 obstacle_source=none：起飛、YAW、降落正常，MOVE 被擋下。
    parser.add_argument("--expect-move-blocked", action="store_true")
    args = parser.parse_args()
    if args.agent_ids:
        ids = [int(value) for value in args.agent_ids.split(",") if value]
        if (not ids or len(set(ids)) != len(ids) or
                any(value < 1 or value > 6 for value in ids) or
                FAKE_AGENT_ID in ids):
            parser.error("--agent-ids 必須是 1～6 的不重複機號，且不可包含 7")
        args.agent_ids = ids
    else:
        args.agent_ids = list(range(1, args.agents + 1))
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = MultiAgentRegression(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

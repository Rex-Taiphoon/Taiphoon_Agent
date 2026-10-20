#!/usr/bin/env python3
"""
@file dual_agent_regression_monitor.py
@brief 雙機 SITL 回歸：起飛懸停、單機 MOVE、YAW、編隊、全體 MOVE、取消編隊、時鐘偏差注入與全體降落

腳本必須在 Agent Docker 容器內執行，透過既有 command topic 下達命令，並以
AgentState、AgentAttitude、PlannerStatus 判定結果。時鐘偏差注入只發布一個不存在的
同儕（agent 7）、位於 1 km 外且時間戳偏移的 AgentTrajectory，不影響兩台受測機。
每個步驟輸出一行 STEP_RESULT，最後輸出 REGRESSION_RESULT。
"""

import argparse
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
AGENT_IDS = (1, 2)
FAKE_AGENT_ID = 7
FAKE_STAMP_OFFSET_S = -3.0
FAKE_DISTANCE_M = 1000.0


def wrap_degrees(angle_deg: float) -> float:
    """將角度正規化到 [-180, 180)。"""
    return (angle_deg + 180.0) % 360.0 - 180.0


def quaternion_yaw_deg(q: List[float]) -> float:
    """以 NED 慣例自四元數 [w, x, y, z] 取得 yaw（度，順時針為正）。"""
    w, x, y, z = (float(value) for value in q)
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


class DualAgentRegression(Node):
    """以 topic 觀測驅動雙機回歸步驟。"""

    def __init__(self, args: argparse.Namespace):
        super().__init__("dual_agent_regression_monitor")
        self.args = args
        self.states: Dict[int, AgentState] = {}
        self.yaw_deg: Dict[int, float] = {}
        self.reasons: Dict[int, str] = {}
        self.reference: Optional[Tuple[float, float]] = None
        self.ground_altitude_m: Dict[int, float] = {}
        self.minimum_distance_m = math.inf
        self.results: List[Tuple[str, bool, str]] = []

        for agent_id in AGENT_IDS:
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
            for agent_id in AGENT_IDS
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
        if len(self.states) == len(AGENT_IDS) and all(
                self.states[i].armed and not self.states[i].landed
                for i in AGENT_IDS):
            self.minimum_distance_m = min(
                self.minimum_distance_m, self.distance_between(1, 2))

    def attitude_cb(self, msg: AgentAttitude, agent_id: int) -> None:
        if all(math.isfinite(float(v)) for v in msg.q):
            self.yaw_deg[agent_id] = quaternion_yaw_deg(list(msg.q))

    def status_cb(self, msg: PlannerStatus, agent_id: int) -> None:
        self.reasons[agent_id] = msg.reason

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
        print(f"STEP_RESULT step={name} pass={passed} {detail}", flush=True)
        return passed

    def flying(self) -> bool:
        return all(self.states[i].armed and not self.states[i].landed
                   for i in AGENT_IDS)

    # ------------------------------------------------------------------
    # 步驟
    # ------------------------------------------------------------------
    def step_takeoff_hover(self) -> bool:
        if not self.wait_until(lambda: len(self.states) == len(AGENT_IDS) and
                               all(i in self.yaw_deg for i in AGENT_IDS), 60.0):
            return self.record("takeoff_hover", False, "reason=no_state")
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
                self.position(i)[2] >= target for i in AGENT_IDS),
            60.0, hold_s=2.0)
        if not reached:
            return self.record("takeoff_hover", False, "reason=altitude_timeout")
        start = {i: self.position(i) for i in AGENT_IDS}
        max_horizontal_drift = 0.0
        max_vertical_drift = 0.0
        deadline = time.monotonic() + self.args.hover_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            for i in AGENT_IDS:
                now = self.position(i)
                max_horizontal_drift = max(max_horizontal_drift, math.hypot(
                    now[0] - start[i][0], now[1] - start[i][1]))
                max_vertical_drift = max(
                    max_vertical_drift, abs(now[2] - start[i][2]))
        yaw_change = max(abs(wrap_degrees(self.yaw_deg[i] - initial_yaw[i]))
                         for i in AGENT_IDS)
        passed = max_horizontal_drift <= 0.5 and max_vertical_drift <= 0.3
        return self.record(
            "takeoff_hover", passed,
            f"altitudes_m={[round(self.position(i)[2], 2) for i in AGENT_IDS]} "
            f"hover_s={self.args.hover_s} "
            f"max_horizontal_drift_m={max_horizontal_drift:.3f} "
            f"max_vertical_drift_m={max_vertical_drift:.3f} "
            f"takeoff_yaw_change_deg={yaw_change:.1f}")

    def step_single_move(self) -> bool:
        start = {i: self.position(i) for i in AGENT_IDS}
        self.send_agent(1, f"MOVE_FORWARD {self.args.move_m}")

        def moved() -> bool:
            now = self.position(1)
            return math.hypot(now[0] - start[1][0],
                              now[1] - start[1][1]) >= self.args.move_m - 0.5

        passed = self.wait_until(moved, 45.0, hold_s=2.0)
        moved_m = math.hypot(self.position(1)[0] - start[1][0],
                             self.position(1)[1] - start[1][1])
        other_m = math.hypot(self.position(2)[0] - start[2][0],
                             self.position(2)[1] - start[2][1])
        passed = passed and other_m <= 1.0
        return self.record(
            "single_move", passed,
            f"uav_1_moved_m={moved_m:.2f} uav_2_drift_m={other_m:.2f} "
            f"uav_1_reason='{self.reasons.get(1, '')}'")

    def step_yaw(self) -> bool:
        start_yaw = self.yaw_deg[2]
        start_altitude = self.position(2)[2]
        self.send_agent(2, f"YAW {self.args.yaw_deg}")
        max_altitude_deviation = 0.0

        def rotated() -> bool:
            nonlocal max_altitude_deviation
            max_altitude_deviation = max(
                max_altitude_deviation,
                abs(self.position(2)[2] - start_altitude))
            change = wrap_degrees(self.yaw_deg[2] - start_yaw)
            return abs(change - self.args.yaw_deg) <= 5.0

        passed = self.wait_until(rotated, 20.0, hold_s=1.0)
        change = wrap_degrees(self.yaw_deg[2] - start_yaw)
        return self.record(
            "yaw", passed,
            f"uav_2_yaw_change_deg={change:.1f} "
            f"max_altitude_deviation_m={max_altitude_deviation:.3f}")

    def wait_formation_converged(self, timeout_s: float) -> bool:
        return self.wait_until(
            lambda: all("formation_converged" in self.reasons.get(i, "")
                        for i in AGENT_IDS),
            timeout_s, hold_s=2.0)

    def step_formation(self) -> bool:
        self.send_all(f"FORMATION_HORI_LINEAR {self.args.formation_spacing_m} "
                      f"{self.args.formation_altitude_m}")
        passed = self.wait_formation_converged(120.0)
        distance = self.distance_between(1, 2)
        passed = passed and abs(distance - self.args.formation_spacing_m) <= 1.0
        return self.record(
            "formation", passed,
            f"distance_m={distance:.2f} "
            f"reasons={[self.reasons.get(i, '') for i in AGENT_IDS]}")

    def step_group_move(self) -> bool:
        start = {i: self.position(i) for i in AGENT_IDS}
        self.send_all(f"MOVE_FORWARD {self.args.move_m}")

        def moved_north() -> bool:
            return all(self.position(i)[0] - start[i][0] >=
                       self.args.move_m - 1.0 for i in AGENT_IDS)

        passed = self.wait_until(moved_north, 90.0, hold_s=1.0)
        converged = self.wait_formation_converged(60.0)
        north = [round(self.position(i)[0] - start[i][0], 2) for i in AGENT_IDS]
        return self.record(
            "group_move", passed and converged,
            f"north_moved_m={north} converged={converged}")

    def step_cancel_hold(self) -> bool:
        self.send_all("FORMATION_CANCEL")
        self.spin_for(2.0)
        start = {i: self.position(i) for i in AGENT_IDS}
        self.spin_for(5.0)
        drift = max(math.hypot(self.position(i)[0] - start[i][0],
                               self.position(i)[1] - start[i][1])
                    for i in AGENT_IDS)
        return self.record("formation_cancel_hold", drift <= 0.5 and self.flying(),
                           f"max_drift_m={drift:.3f}")

    def publish_fake_trajectory(self) -> None:
        """發布 agent 7 的終點佔位軌跡：位於 uav_1 北方 1 km，時間戳早 3 秒（接收端估計 +3 秒）。"""
        latitude, longitude, altitude = (
            float(v) for v in self.states[1].position_global)
        fake_latitude = latitude + math.degrees(FAKE_DISTANCE_M / EARTH_RADIUS_M)
        now_ns = self.get_clock().now().nanoseconds + int(FAKE_STAMP_OFFSET_S * 1e9)
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
        start = {i: self.position(i) for i in AGENT_IDS}
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            self.publish_fake_trajectory()
            self.spin_for(0.1)
        self.spin_for(2.0)
        drift = max(math.hypot(self.position(i)[0] - start[i][0],
                               self.position(i)[1] - start[i][1])
                    for i in AGENT_IDS)
        # WARN log 由外層腳本自容器 log 檢查；此處確認受測機不受影響。
        return self.record("skew_injection_isolated",
                           drift <= 0.5 and self.flying(),
                           f"fake_agent={FAKE_AGENT_ID} stamp_offset_s={FAKE_STAMP_OFFSET_S} "
                           f"max_drift_m={drift:.3f}")

    def step_land(self) -> bool:
        self.send_all("MODE_LAND")
        passed = self.wait_until(
            lambda: all(self.states[i].landed for i in AGENT_IDS), 90.0,
            hold_s=1.0)
        return self.record(
            "land", passed,
            f"landed={[bool(self.states[i].landed) for i in AGENT_IDS]} "
            f"altitudes_m={[round(self.position(i)[2], 2) for i in AGENT_IDS]}")

    def run(self) -> int:
        steps = (self.step_takeoff_hover, self.step_single_move, self.step_yaw,
                 self.step_formation, self.step_group_move,
                 self.step_cancel_hold, self.step_skew_injection)
        for step in steps:
            if not step():
                break
        # 不論前面成功與否都要讓飛機落地，避免 SITL 留在空中。
        self.step_land()
        minimum_distance_ok = self.minimum_distance_m >= self.args.min_distance_m
        self.record("minimum_peer_distance", minimum_distance_ok,
                    f"minimum_distance_m={self.minimum_distance_m:.2f} "
                    f"limit_m={self.args.min_distance_m}")
        passed = len(self.results) == 9 and all(r[1] for r in self.results)
        print(f"REGRESSION_RESULT pass={passed} steps={len(self.results)}",
              flush=True)
        return 0 if passed else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--takeoff-altitude-m", type=float, default=8.0)
    parser.add_argument("--hover-s", type=float, default=10.0)
    parser.add_argument("--move-m", type=float, default=5.0)
    parser.add_argument("--yaw-deg", type=float, default=90.0)
    parser.add_argument("--formation-spacing-m", type=float, default=4.0)
    parser.add_argument("--formation-altitude-m", type=float, default=10.0)
    parser.add_argument("--min-distance-m", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = DualAgentRegression(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

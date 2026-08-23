#!/usr/bin/env python3
"""
@file ego_swarm_acceptance_monitor.py
@brief 以 ROS2 topic 量測 EGO-Swarm 兩組機群避障驗收條件
@author Taiphoon Agent Team
@date 2026-08-17

此腳本只讀取 AgentState、PlannerStatus、AgentTrajectory 與 command topic，
不發布任何控制訊息。腳本必須在 Agent Docker 容器內執行。
"""

import argparse
import math
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from taiphoon_agent.msg import AgentState
from taiphoon_agent.msg import AgentTrajectory
from taiphoon_agent.msg import PlannerStatus


class AcceptanceMonitor(Node):
    """收集單次多機驗收的可重複安全與時序指標。"""

    def __init__(self, mode: int, agent_count: int, timeout_s: float,
                 minimum_distance_m: float, max_short_violation_s: float,
                 require_post_cancel_moves: bool):
        super().__init__("ego_swarm_acceptance_monitor")
        self.mode = mode
        self.agent_ids = list(range(1, agent_count + 1))
        self.timeout_s = timeout_s
        self.minimum_distance_m = minimum_distance_m
        self.max_short_violation_s = max_short_violation_s
        self.require_post_cancel_moves = require_post_cancel_moves
        self.started_at = time.monotonic()
        self.states: Dict[int, Tuple[float, List[float]]] = {}
        self.global_reference: Optional[List[float]] = None
        self.statuses: Dict[int, Tuple[float, PlannerStatus]] = {}
        self.z_baselines: Dict[int, float] = {}
        self.command_phase = "waiting"
        self.yaw_seen = False
        self.formation_seen = False
        self.formation_ready = False
        self.formation_ready_since: Optional[float] = None
        self.formation_radius_valid = False
        self.formation_altitude_valid = False
        self.formation_target_radius_m: Optional[float] = None
        self.formation_target_altitude_m: Optional[float] = None
        self.max_formation_radius_error_m = math.inf
        self.cancel_seen = False
        self.move_results: Dict[str, Dict[str, float]] = {}
        self.active_move: Optional[str] = None
        self.active_move_baseline: Optional[List[float]] = None
        self.active_move_started_at = 0.0
        self.active_move_distance_m = 0.0
        self.min_peer_distance_m = math.inf
        self.violation_started_at: Optional[float] = None
        self.max_contiguous_violation_s = 0.0
        self.total_violation_s = 0.0
        self.max_state_z_deviation_m = 0.0
        self.max_trajectory_z_span_m = 0.0
        self.trajectory_count = 0
        self.last_trajectory_id: Dict[int, int] = {}
        self.terminal_hold_seen: Dict[int, bool] = {}
        self.invalid_control_point_count = False
        self.reason_counts: Dict[str, int] = {}
        self.last_report_at = 0.0
        self.last_status_trajectory_id = 0
        self.finished_at: Optional[float] = None
        self.completed = False
        self.failed = False

        for agent_id in self.agent_ids:
            self.create_subscription(
                AgentState,
                f"/fmu{agent_id}_agent/state",
                lambda msg, current_id=agent_id: self.state_cb(msg, current_id),
                20,
            )
            self.create_subscription(
                PlannerStatus,
                f"/fmu{agent_id}_agent/planning/status",
                lambda msg, current_id=agent_id: self.status_cb(msg, current_id),
                20,
            )
        self.create_subscription(
            AgentTrajectory,
            "/fmu_all_agent/swarm/agent_trajectories",
            self.trajectory_cb,
            20,
        )

        self.create_subscription(
            String, "/fmu1_agent/command", self.agent_command_cb, 20)
        self.create_subscription(
            String, "/fmu_all_agent/command", self.all_command_cb, 20)
        self.create_timer(0.1, self.timer_cb)

    @staticmethod
    def distance(first: List[float], second: List[float]) -> float:
        return math.sqrt(sum((a - b) * (a - b)
                             for a, b in zip(first, second)))

    def state_cb(self, msg: AgentState, agent_id: int) -> None:
        global_position = [float(value) for value in msg.position_global]
        if len(global_position) != 3 or not all(
                math.isfinite(value) for value in global_position):
            return
        if self.global_reference is None:
            self.global_reference = list(global_position)
        position = self.global_to_common_ned(global_position)
        if len(position) != 3 or not all(math.isfinite(value)
                                         for value in position):
            return
        now = time.monotonic()
        self.states[agent_id] = (now, position)
        if agent_id in self.z_baselines:
            deviation = abs(position[2] - self.z_baselines[agent_id])
            self.max_state_z_deviation_m = max(
                self.max_state_z_deviation_m, deviation)

        if self.active_move == "" or self.active_move_baseline is None:
            return
        if agent_id != 1:
            return
        displacement = self.distance(position, self.active_move_baseline)
        self.active_move_distance_m = max(
            self.active_move_distance_m, displacement)
        target_distance = {
            "forward5": 4.0,
            "right20": 19.0,
            "forward30": 29.0,
            "backward30": 29.0,
        }.get(self.active_move, math.inf)
        if displacement + 1.0e-6 >= target_distance:
            elapsed = now - self.active_move_started_at
            self.move_results[self.active_move] = {
                "elapsed_s": elapsed,
                "distance_m": displacement,
            }
            if self.active_move == "forward5":
                self.command_phase = "forward5_reached"
            elif self.active_move == "forward30":
                self.command_phase = "forward30_reached"
            else:
                self.command_phase = "target_reached"
                self.finished_at = now
            self.active_move = None

    def global_to_common_ned(self, position: List[float]) -> List[float]:
        """把所有 Agent 的 WGS84 狀態投影到同一個公尺制局部平面。"""
        if self.global_reference is None:
            return [math.nan, math.nan, math.nan]
        latitude_scale_m = 111319.49079327358
        reference_latitude_rad = math.radians(self.global_reference[0])
        north_m = (position[0] - self.global_reference[0]) * latitude_scale_m
        east_m = (position[1] - self.global_reference[1]) * latitude_scale_m * \
            math.cos(reference_latitude_rad)
        down_m = -(position[2] - self.global_reference[2])
        return [north_m, east_m, down_m]

    def status_cb(self, msg: PlannerStatus, agent_id: int) -> None:
        now = time.monotonic()
        self.statuses[agent_id] = (now, msg)
        reason = str(msg.reason)
        self.reason_counts[reason] = self.reason_counts.get(reason, 0) + 1
        if self.active_move is not None and agent_id == 1 and \
                self.last_status_trajectory_id != msg.trajectory_id:
            self.last_status_trajectory_id = msg.trajectory_id
            print("TRACE_STATUS agent=1 id={} state={} min_peer={:.3f} "
                  "planning_ms={:.3f} reason={}".format(
                      msg.trajectory_id, msg.state,
                      float(msg.min_peer_distance_m),
                      float(msg.planning_time_ms), reason), flush=True)
        # formation_ready 由 timer_cb 以連續 2 秒的狀態、幾何與高度條件
        # 一起判定，避免只收到單次 status 就誤判編隊完成。

    def trajectory_cb(self, msg: AgentTrajectory) -> None:
        if not msg.valid or msg.agent_index not in self.agent_ids:
            return
        points = list(msg.control_points)
        if len(points) == 0 or len(points) % 3 != 0:
            return
        point_count = len(points) // 3
        if point_count != 10:
            self.invalid_control_point_count = True
        if msg.terminal_hold and self.formation_seen:
            self.terminal_hold_seen[int(msg.agent_index)] = True
        heights = [float(points[index]) for index in range(2, len(points), 3)]
        if not all(math.isfinite(value) for value in heights):
            return
        self.trajectory_count += 1
        agent_id = int(msg.agent_index)
        if self.active_move is not None and agent_id == 1:
            self.max_trajectory_z_span_m = max(
                self.max_trajectory_z_span_m, max(heights) - min(heights))
        if self.active_move is not None and agent_id == 1 and \
                self.last_trajectory_id.get(agent_id) != msg.trajectory_id:
            self.last_trajectory_id[agent_id] = msg.trajectory_id
            common_points = []
            if self.global_reference is not None:
                for index in range(0, len(points), 3):
                    common_points.append(self.global_to_common_ned([
                        float(points[index]), float(points[index + 1]),
                        float(points[index + 2])]))
            print("TRACE_TRAJECTORY agent=1 id={} hold={} duration_s={:.3f} "
                  "points={}".format(
                      msg.trajectory_id, msg.terminal_hold,
                      float(msg.duration_s), common_points), flush=True)

    def handle_command(self, command: str, broadcast: bool) -> None:
        del broadcast
        text = command.strip().upper()
        now = time.monotonic()
        if self.mode == 1:
            if text.startswith("YAW "):
                self.yaw_seen = True
                self.command_phase = "yaw_seen"
                return
            if text == "MOVE_FORWARD 30":
                if not self.yaw_seen:
                    self.failed = True
                    self.command_phase = "forward_without_yaw"
                    return
                self.begin_move("forward30", now, 30.0)
            elif text == "MOVE_BACKWARD 30" and \
                    self.command_phase == "forward30_reached":
                self.begin_move("backward30", now, 30.0)
            return

        if text.startswith("FORMATION_CIRCLE "):
            self.formation_seen = True
            self.command_phase = "formation_requested"
            fields = text.split()
            if len(fields) == 3:
                try:
                    radius_m = float(fields[1])
                    altitude_m = float(fields[2])
                    if (math.isfinite(radius_m) and radius_m > 0.0 and
                            math.isfinite(altitude_m) and altitude_m > 0.0):
                        self.formation_target_radius_m = radius_m
                        self.formation_target_altitude_m = altitude_m
                except ValueError:
                    pass
            self.set_z_baselines()
        elif text == "FORMATION_CANCEL":
            self.cancel_seen = True
            self.command_phase = "formation_cancelled"
            if self.mode == 2 and not self.require_post_cancel_moves:
                self.finished_at = now
        elif text == "MOVE_FORWARD 5" and self.cancel_seen:
            self.begin_move("forward5", now, 5.0)
        elif text == "MOVE_RIGHT 20" and self.cancel_seen:
            self.begin_move("right20", now, 20.0)

    def agent_command_cb(self, msg: String) -> None:
        if msg is None:
            return
        self.handle_command(msg.data, False)

    def all_command_cb(self, msg: String) -> None:
        if msg is None:
            return
        self.handle_command(msg.data, True)

    def set_z_baselines(self) -> None:
        for agent_id, (_, position) in self.states.items():
            self.z_baselines.setdefault(agent_id, position[2])

    def begin_move(self, move_name: str, now: float, distance_m: float) -> None:
        del distance_m
        if 1 not in self.states:
            self.failed = True
            self.command_phase = f"{move_name}_without_state"
            return
        self.set_z_baselines()
        self.active_move = move_name
        self.active_move_baseline = list(self.states[1][1])
        self.active_move_started_at = now
        self.active_move_distance_m = 0.0
        self.command_phase = f"{move_name}_running"

    def update_peer_metrics(self, now: float) -> None:
        if not all(agent_id in self.states for agent_id in self.agent_ids):
            return
        if any(now - self.states[agent_id][0] > 0.5
               for agent_id in self.agent_ids):
            return
        minimum = math.inf
        for index, first_id in enumerate(self.agent_ids):
            for second_id in self.agent_ids[index + 1:]:
                distance = self.distance(
                    self.states[first_id][1], self.states[second_id][1])
                minimum = min(minimum, distance)
        if not math.isfinite(minimum):
            return
        self.min_peer_distance_m = min(self.min_peer_distance_m, minimum)
        if minimum < self.minimum_distance_m:
            if self.violation_started_at is None:
                self.violation_started_at = now
            return
        if self.violation_started_at is not None:
            duration = now - self.violation_started_at
            self.total_violation_s += duration
            self.max_contiguous_violation_s = max(
                self.max_contiguous_violation_s, duration)
            self.violation_started_at = None

    def timer_cb(self) -> None:
        now = time.monotonic()
        self.update_peer_metrics(now)
        if self.mode == 2 and self.formation_seen and not self.cancel_seen:
            statuses_ready = all(
                agent_id in self.statuses and
                now - self.statuses[agent_id][0] <= 0.5 and
                self.statuses[agent_id][1].reason ==
                "formation_converged; terminal hold active"
                for agent_id in self.agent_ids)
            geometry_ready = False
            altitude_ready = False
            if len(self.states) == len(self.agent_ids):
                positions = [self.states[agent_id][1]
                             for agent_id in self.agent_ids]
                centroid = [sum(position[index] for position in positions) /
                            len(positions) for index in range(3)]
                radii = [self.distance(position, centroid)
                         for position in positions]
                target_radius = self.formation_target_radius_m
                geometry_ready = (
                    target_radius is not None and
                    all(abs(radius - target_radius) <= 1.5
                        for radius in radii))
                target_altitude = self.formation_target_altitude_m
                self.max_formation_radius_error_m = min(
                    self.max_formation_radius_error_m,
                    max(abs(radius - target_radius) for radius in radii)
                    if target_radius is not None else math.inf)
                # 此 monitor 在 takeoff_and_wait 之後才啟動，無法從自身
                # local NED 原點推回地面絕對高度；起飛等待器已先確認
                # target_altitude ±0.5 m，這裡確認編隊期間相對該起飛高度
                # 持續穩定 ±0.5 m。
                altitude_ready = all(
                    target_altitude is not None and
                    agent_id in self.z_baselines and
                    abs(self.states[agent_id][1][2] -
                        self.z_baselines[agent_id]) <= 0.5
                    for agent_id in self.agent_ids)
            ready_now = statuses_ready and geometry_ready and altitude_ready
            if ready_now:
                self.formation_radius_valid = True
                self.formation_altitude_valid = True
                if self.formation_ready_since is None:
                    self.formation_ready_since = now
                if now - self.formation_ready_since >= 2.0:
                    self.formation_ready = True
                    self.command_phase = "formation_converged"
            else:
                self.formation_ready_since = None
                self.formation_ready = False
        if now - self.last_report_at >= 2.0:
            self.last_report_at = now
            print(
                "MONITOR phase={} formation_ready={} min_peer={:.3f} "
                "max_under={:.3f} z_dev={:.3f} traj_z_span={:.3f}".format(
                    self.command_phase, self.formation_ready,
                    self.min_peer_distance_m
                    if math.isfinite(self.min_peer_distance_m) else -1.0,
                    self.max_contiguous_violation_s,
                    self.max_state_z_deviation_m,
                    self.max_trajectory_z_span_m),
                flush=True,
            )
        if self.active_move is not None and (
                now - self.active_move_started_at > 20.0):
            self.move_results[self.active_move] = {
                "elapsed_s": now - self.active_move_started_at,
                "distance_m": self.active_move_distance_m,
            }
            self.failed = True
            self.command_phase = f"{self.active_move}_timeout"
            self.active_move = None
        if self.command_phase == "target_reached" and \
                self.active_move is None and \
                now - self.active_move_started_at >= 2.0:
            self.completed = True
        if self.finished_at is None:
            return
        if now - self.finished_at < 2.0:
            return
        # 由 main loop 在 callback 返回後執行 finish，避免在 ROS timer
        # callback 內 shutdown executor 導致驗收摘要遺失。
        self.completed = True

    def finish(self) -> None:
        if self.finished_at is None:
            self.finished_at = time.monotonic()
        if self.violation_started_at is not None:
            duration = time.monotonic() - self.violation_started_at
            self.total_violation_s += duration
            self.max_contiguous_violation_s = max(
                self.max_contiguous_violation_s, duration)
            self.violation_started_at = None
        for move_name, result in self.move_results.items():
            if result["elapsed_s"] > 20.0:
                self.failed = True
        if self.mode == 1 and not self.yaw_seen:
            self.failed = True
        if self.mode == 2 and not (
                self.formation_seen and self.formation_ready and
                self.formation_radius_valid and self.formation_altitude_valid and
                self.cancel_seen and
                (not self.require_post_cancel_moves or
                 ("forward5" in self.move_results and
                  "right20" in self.move_results)) and
                all(self.terminal_hold_seen.get(agent_id, False)
                    for agent_id in self.agent_ids)):
            self.failed = True
        if self.mode == 1 and not (
                "forward30" in self.move_results and
                "backward30" in self.move_results):
            self.failed = True
        if self.min_peer_distance_m < self.minimum_distance_m and \
                self.max_contiguous_violation_s > self.max_short_violation_s:
            self.failed = True
        if any("emergency" in reason or "timeout" in reason or
               "no_feasible" in reason or "no feasible" in reason or
               "infeasible" in reason
               for reason in self.reason_counts):
            self.failed = True
        if self.max_trajectory_z_span_m > 0.05 or \
                self.invalid_control_point_count:
            self.failed = True
        print("ACCEPTANCE_RESULT mode={} pass={} phase={}".format(
            self.mode, not self.failed, self.command_phase), flush=True)
        print("METRICS min_peer_distance_m={:.6f} "
              "max_contiguous_below_min_s={:.6f} "
              "total_below_min_s={:.6f} max_state_z_deviation_m={:.6f} "
              "max_trajectory_z_span_m={:.6f} trajectory_count={}".format(
                  self.min_peer_distance_m
                  if math.isfinite(self.min_peer_distance_m) else -1.0,
                  self.max_contiguous_violation_s, self.total_violation_s,
                  self.max_state_z_deviation_m,
                  self.max_trajectory_z_span_m, self.trajectory_count),
              flush=True)
        print("FORMATION_METRICS ready={} radius_valid={} altitude_valid={} "
              "max_radius_error_m={:.6f} terminal_hold_agents={}".format(
                  self.formation_ready, self.formation_radius_valid,
                  self.formation_altitude_valid,
                  self.max_formation_radius_error_m
                  if math.isfinite(self.max_formation_radius_error_m) else -1.0,
                  sorted(self.terminal_hold_seen)), flush=True)
        for move_name, result in sorted(self.move_results.items()):
            print("MOVE_RESULT {} elapsed_s={:.6f} distance_m={:.6f}".format(
                move_name, result["elapsed_s"], result["distance_m"]),
                flush=True)
        for reason, count in sorted(self.reason_counts.items()):
            if ("emergency" in reason or "timeout" in reason or
                    "no_feasible" in reason or "no feasible" in reason or
                    "infeasible" in reason):
                print("REASON_COUNT {} count={}".format(reason, count),
                      flush=True)
        self.destroy_node()
        rclpy.shutdown()

    def timed_out(self) -> bool:
        return time.monotonic() - self.started_at > self.timeout_s


def parse_args() -> argparse.Namespace:
    """解析驗收監控參數。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", type=int, choices=(1, 2), required=True)
    parser.add_argument("--agents", type=int, default=4)
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--minimum-distance-m", type=float, default=3.0)
    parser.add_argument("--max-short-violation-s", type=float, default=0.5)
    parser.add_argument("--require-post-cancel-moves", action="store_true")
    return parser.parse_args()


def main() -> int:
    """啟動 ROS2 collector 並回傳驗收結果。"""
    args = parse_args()
    rclpy.init()
    monitor = AcceptanceMonitor(
        args.mode, args.agents, args.timeout_s,
        args.minimum_distance_m, args.max_short_violation_s,
        args.require_post_cancel_moves)
    try:
        while rclpy.ok() and not monitor.timed_out() and not monitor.completed:
            rclpy.spin_once(monitor, timeout_sec=0.1)
        if monitor.completed:
            monitor.finish()
            return 0 if not monitor.failed else 1
        if rclpy.ok():
            monitor.failed = True
            monitor.command_phase = "monitor_timeout"
            monitor.finish()
            return 1
    except KeyboardInterrupt:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 130
    return 0 if not monitor.failed else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
@file ego_route_planning_monitor.py
@brief EGO 時空路徑規劃 SITL 的路徑與安全指標觀測器

此程式只讀取 AgentState、AgentAttitude、AgentTrajectory 與 PlannerStatus；
所有控制命令仍由外層驗證腳本透過既有 command topic 發布。
"""

import argparse
import math
import sys
import time
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node

from taiphoon_agent.msg import AgentAttitude, AgentState, AgentTrajectory, PlannerStatus


class RoutePlanningMonitor(Node):
    """收集單機或多機路徑驗收所需的可重現指標。"""

    def __init__(self, args: argparse.Namespace):
        super().__init__("ego_route_planning_sitl_monitor")
        self.agent_ids = list(range(1, args.agents + 1))
        self.moving_agent_ids = args.moving_agents or list(self.agent_ids)
        self.timeout_s = args.timeout_s
        self.min_distance_m = args.minimum_distance_m
        self.progress_threshold_m = (args.minimum_progress_m
                                     if args.minimum_progress_m is not None
                                     else args.minimum_displacement_m)
        self.require_lateral_m = args.require_lateral_m
        self.max_lateral_m = args.max_lateral_m
        self.min_lateral_m = args.minimum_lateral_m
        self.max_path_length_m = args.max_path_length_m
        self.min_obstacle_distance_required_m = args.minimum_obstacle_distance_m
        self.max_state_z_deviation_required_m = args.max_state_z_deviation_m
        self.expected_control_points = args.expected_control_points
        self.travel_axis = 0 if args.travel_axis == "north" else 1
        self.lateral_axis = 1 - self.travel_axis
        self.expect_failure = args.expect_failure
        self.metric_warmup_s = args.metric_warmup_s
        self.require_initial_heading_north = args.require_initial_heading_north
        self.heading_tolerance_deg = args.heading_tolerance_deg
        self.started_at = time.monotonic()
        self.global_reference: Optional[List[float]] = None
        self.states: Dict[int, Tuple[float, List[float]]] = {}
        self.baselines: Dict[int, List[float]] = {}
        self.previous_positions: Dict[int, List[float]] = {}
        self.reached: Set[int] = set()
        self.progress_m: Dict[int, float] = {}
        self.path_length_m: Dict[int, float] = {}
        self.min_peer_distance_m = math.inf
        self.min_obstacle_distance_m = math.inf
        self.max_state_north_offset_m = 0.0
        self.max_trajectory_north_offset_m = 0.0
        self.max_state_lateral_offset_m = 0.0
        self.max_trajectory_lateral_offset_m = 0.0
        self.max_state_z_deviation_m = 0.0
        self.max_trajectory_z_span_m = 0.0
        self.trajectory_ids: Dict[int, int] = {}
        self.trajectory_count = 0
        self.invalid_control_point_count = False
        self.control_point_counts: Set[int] = set()
        self.timeout_status_count = 0
        self.failure_status_count = 0
        self.latest_reason: Dict[int, str] = {}
        self.violation_started_at: Optional[float] = None
        self.max_contiguous_violation_s = 0.0
        self.failed = False
        self.completed = False
        self.metrics_started = False
        self.initial_yaw_deg: Optional[float] = None
        self.heading_failure = False

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
            AgentAttitude, "/fmu1_agent/attitude", self.attitude_cb, 20)
        self.create_subscription(
            AgentTrajectory,
            "/fmu_all_agent/swarm/agent_trajectories",
            self.trajectory_cb,
            20,
        )
        self.create_timer(0.1, self.timer_cb)

    @staticmethod
    def distance(first: List[float], second: List[float]) -> float:
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))

    @staticmethod
    def wrap_degrees(angle_deg: float) -> float:
        return (angle_deg + 180.0) % 360.0 - 180.0

    def global_to_common_ned(self, position: List[float]) -> List[float]:
        if self.global_reference is None:
            return [math.nan, math.nan, math.nan]
        latitude_scale_m = 111319.49079327358
        reference_latitude_rad = math.radians(self.global_reference[0])
        return [
            (position[0] - self.global_reference[0]) * latitude_scale_m,
            (position[1] - self.global_reference[1]) * latitude_scale_m *
            math.cos(reference_latitude_rad),
            -(position[2] - self.global_reference[2]),
        ]

    def attitude_cb(self, msg: AgentAttitude) -> None:
        values = [float(value) for value in msg.q]
        if len(values) != 4 or not all(math.isfinite(value) for value in values):
            return
        w, x, y, z = values
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if not math.isfinite(norm) or norm <= 1.0e-6:
            return
        w, x, y, z = w / norm, x / norm, y / norm, z / norm
        yaw = math.degrees(math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        ))
        if not math.isfinite(yaw):
            return
        if self.initial_yaw_deg is None:
            self.initial_yaw_deg = yaw
            print("TRACE_INITIAL_YAW yaw_deg={:.3f}".format(yaw), flush=True)
            if (self.require_initial_heading_north and
                    abs(self.wrap_degrees(yaw)) > self.heading_tolerance_deg):
                self.heading_failure = True

    def state_cb(self, msg: AgentState, agent_id: int) -> None:
        global_position = [float(value) for value in msg.position_global]
        if len(global_position) != 3 or not all(
                math.isfinite(value) for value in global_position):
            return
        if self.global_reference is None:
            self.global_reference = list(global_position)
        position = self.global_to_common_ned(global_position)
        if not all(math.isfinite(value) for value in position):
            return
        now = time.monotonic()
        self.states[agent_id] = (now, position)
        self.baselines.setdefault(agent_id, list(position))
        if not self.metrics_started or agent_id not in self.moving_agent_ids:
            return
        baseline = self.baselines[agent_id]
        previous = self.previous_positions.setdefault(agent_id, list(position))
        self.path_length_m[agent_id] = self.path_length_m.get(agent_id, 0.0) + \
            self.distance(position, previous)
        self.previous_positions[agent_id] = list(position)
        delta = [position[index] - baseline[index] for index in range(3)]
        self.progress_m[agent_id] = max(
            self.progress_m.get(agent_id, 0.0), delta[self.travel_axis])
        self.max_state_north_offset_m = max(
            self.max_state_north_offset_m, abs(delta[0]))
        self.max_state_lateral_offset_m = max(
            self.max_state_lateral_offset_m, abs(delta[self.lateral_axis]))
        self.max_state_z_deviation_m = max(
            self.max_state_z_deviation_m, abs(delta[2]))
        if self.progress_m[agent_id] + 1.0e-6 >= self.progress_threshold_m:
            self.reached.add(agent_id)

    def status_cb(self, msg: PlannerStatus, agent_id: int) -> None:
        reason = str(msg.reason).lower()
        if self.latest_reason.get(agent_id) != reason:
            print(
                "TRACE_STATUS agent={} state={} reason={} planning_ms={:.3f} "
                "min_obstacle_m={:.3f} min_peer_m={:.3f}".format(
                    agent_id, msg.state, reason, float(msg.planning_time_ms),
                    float(msg.min_obstacle_distance_m),
                    float(msg.min_peer_distance_m)), flush=True)
        self.latest_reason[agent_id] = reason
        obstacle_distance = float(msg.min_obstacle_distance_m)
        peer_distance = float(msg.min_peer_distance_m)
        # PlannerStatus 在尚未完成第一次規劃或沒有 peer 時會以 0 表示
        # unavailable；不可把該 sentinel 當成實際安全距離。
        if math.isfinite(obstacle_distance) and obstacle_distance > 0.0:
            self.min_obstacle_distance_m = min(
                self.min_obstacle_distance_m, obstacle_distance)
        if math.isfinite(peer_distance) and peer_distance > 0.0:
            self.min_peer_distance_m = min(
                self.min_peer_distance_m, peer_distance)
        if "timeout" in reason:
            self.timeout_status_count += 1
        if ("no_feasible" in reason or "no feasible" in reason or
                "infeasible" in reason or "emergency" in reason):
            self.failure_status_count += 1

    def trajectory_cb(self, msg: AgentTrajectory) -> None:
        if not msg.valid or msg.agent_index not in self.agent_ids or \
                msg.agent_index not in self.moving_agent_ids:
            return
        points = list(msg.control_points)
        if len(points) % 3 != 0 or len(points) == 0:
            return
        point_count = len(points) // 3
        self.control_point_counts.add(point_count)
        if point_count != self.expected_control_points:
            self.invalid_control_point_count = True
        global_points = [
            [float(points[index]), float(points[index + 1]),
             float(points[index + 2])]
            for index in range(0, len(points), 3)
        ]
        if not all(all(math.isfinite(value) for value in point)
                   for point in global_points):
            return
        common_points = [self.global_to_common_ned(point)
                         for point in global_points]
        if not all(all(math.isfinite(value) for value in point)
                   for point in common_points):
            return
        if time.monotonic() - self.started_at < self.metric_warmup_s:
            return
        self.trajectory_count += 1
        heights = [point[2] for point in common_points]
        self.max_trajectory_z_span_m = max(
            self.max_trajectory_z_span_m, max(heights) - min(heights))
        baseline = self.baselines.get(msg.agent_index)
        if baseline is not None:
            north_offset = max(abs(point[0] - baseline[0])
                               for point in common_points)
            lateral_offset = max(abs(point[self.lateral_axis] -
                                      baseline[self.lateral_axis])
                                 for point in common_points)
            self.max_trajectory_north_offset_m = max(
                self.max_trajectory_north_offset_m, north_offset)
            self.max_trajectory_lateral_offset_m = max(
                self.max_trajectory_lateral_offset_m, lateral_offset)
        previous_id = self.trajectory_ids.get(msg.agent_index)
        if previous_id != msg.trajectory_id:
            self.trajectory_ids[msg.agent_index] = msg.trajectory_id
            print(
                "TRACE_TRAJECTORY agent={} id={} hold={} duration_s={} "
                "control_points={}".format(
                    msg.agent_index, msg.trajectory_id, msg.terminal_hold,
                    float(msg.duration_s), point_count), flush=True)

    def update_peer_distance(self, now: float) -> None:
        if len(self.states) < len(self.agent_ids):
            return
        if any(now - self.states[agent_id][0] > 0.5
               for agent_id in self.agent_ids):
            return
        current_minimum = math.inf
        for index, first_id in enumerate(self.agent_ids):
            for second_id in self.agent_ids[index + 1:]:
                current_minimum = min(
                    current_minimum,
                    self.distance(self.states[first_id][1],
                                  self.states[second_id][1]),)
        if not math.isfinite(current_minimum):
            return
        self.min_peer_distance_m = min(self.min_peer_distance_m,
                                       current_minimum)
        if current_minimum < self.min_distance_m:
            if self.violation_started_at is None:
                self.violation_started_at = now
        elif self.violation_started_at is not None:
            duration = now - self.violation_started_at
            self.max_contiguous_violation_s = max(
                self.max_contiguous_violation_s, duration)
            self.violation_started_at = None

    def timer_cb(self) -> None:
        now = time.monotonic()
        if (not self.metrics_started and
                now - self.started_at >= self.metric_warmup_s):
            for agent_id, (_, position) in self.states.items():
                self.baselines[agent_id] = list(position)
                self.previous_positions[agent_id] = list(position)
                self.progress_m[agent_id] = 0.0
                self.path_length_m[agent_id] = 0.0
            self.reached.clear()
            self.metrics_started = True
        self.update_peer_distance(now)
        if now - self.started_at >= self.timeout_s:
            self.completed = True
            return
        if self.expect_failure:
            if self.failure_status_count > 0 or self.timeout_status_count > 0:
                self.completed = True
            return
        if (set(self.moving_agent_ids) <= self.reached and
                self.trajectory_count > 0):
            self.completed = True

    def finish(self) -> int:
        if self.violation_started_at is not None:
            self.max_contiguous_violation_s = max(
                self.max_contiguous_violation_s,
                time.monotonic() - self.violation_started_at,)
        max_path_length = max(self.path_length_m.values(), default=0.0)
        reached_ok = set(self.moving_agent_ids) <= self.reached
        obstacle_ok = (self.min_obstacle_distance_required_m <= 0.0 or
                       (math.isfinite(self.min_obstacle_distance_m) and
                        self.min_obstacle_distance_m + 1.0e-6 >=
                        self.min_obstacle_distance_required_m))
        if self.expect_failure:
            passed = ((self.failure_status_count > 0 or
                       self.timeout_status_count > 0) and not reached_ok)
        else:
            passed = (
                reached_ok and self.trajectory_count > 0 and
                not self.invalid_control_point_count and
                self.max_trajectory_z_span_m <= 0.05 and
                self.max_state_z_deviation_m <=
                self.max_state_z_deviation_required_m + 1.0e-6 and
                self.max_contiguous_violation_s <= 0.5 and
                self.timeout_status_count == 0 and
                self.failure_status_count == 0 and
                self.max_state_lateral_offset_m + 1.0e-6 >=
                self.min_lateral_m and
                self.max_trajectory_lateral_offset_m + 1.0e-6 >=
                self.require_lateral_m and
                (self.max_lateral_m <= 0.0 or
                 self.max_state_lateral_offset_m <= self.max_lateral_m) and
                (self.max_lateral_m <= 0.0 or
                 self.max_trajectory_lateral_offset_m <= self.max_lateral_m) and
                (self.max_path_length_m <= 0.0 or
                 max_path_length <= self.max_path_length_m) and
                obstacle_ok and not self.heading_failure and
                (not self.require_initial_heading_north or
                 self.initial_yaw_deg is not None))
        self.failed = not passed
        print(
            "ROUTE_RESULT pass={} reached={} east_progress_m={:.6f} "
            "max_state_north_offset_m={:.6f} "
            "max_trajectory_north_offset_m={:.6f} "
            "flown_path_length_m={:.6f} max_state_z_deviation_m={:.6f} "
            "max_trajectory_z_span_m={:.6f} min_obstacle_distance_m={:.6f} "
            "min_peer_distance_m={:.6f} timeout_status_count={} "
            "failure_status_count={} trajectory_count={} control_point_counts={}".format(
                passed, sorted(self.reached),
                max(self.progress_m.values(), default=0.0),
                self.max_state_north_offset_m,
                self.max_trajectory_north_offset_m, max_path_length,
                self.max_state_z_deviation_m, self.max_trajectory_z_span_m,
                self.min_obstacle_distance_m
                if math.isfinite(self.min_obstacle_distance_m) else -1.0,
                self.min_peer_distance_m
                if math.isfinite(self.min_peer_distance_m) else -1.0,
                self.timeout_status_count, self.failure_status_count,
                self.trajectory_count, sorted(self.control_point_counts)),
            flush=True,)
        self.destroy_node()
        rclpy.shutdown()
        return 0 if passed else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--agents", type=int, required=True)
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument("--minimum-distance-m", type=float, default=3.0)
    parser.add_argument("--minimum-displacement-m", type=float, default=8.0,
                        help="舊參數；未指定 minimum-progress 時作為沿軸進度門檻")
    parser.add_argument("--minimum-progress-m", type=float, default=None)
    parser.add_argument("--metric-warmup-s", type=float, default=3.0)
    parser.add_argument("--moving-agents", type=int, nargs="+", default=None)
    parser.add_argument("--require-lateral-m", type=float, default=0.0)
    parser.add_argument("--minimum-lateral-m", type=float, default=0.0)
    parser.add_argument("--max-lateral-m", type=float, default=0.0)
    parser.add_argument("--max-path-length-m", type=float, default=0.0)
    parser.add_argument("--minimum-obstacle-distance-m", type=float,
                        default=0.0)
    parser.add_argument("--max-state-z-deviation-m", type=float, default=0.5)
    parser.add_argument("--expected-control-points", type=int, default=10)
    parser.add_argument("--require-initial-heading-north", action="store_true")
    parser.add_argument("--heading-tolerance-deg", type=float, default=10.0)
    parser.add_argument("--travel-axis", choices=("north", "east"),
                        default="north")
    parser.add_argument("--expect-failure", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.agents <= 0 or args.expected_control_points <= 0:
        return 2
    if (not args.moving_agents or
            any(agent_id <= 0 or agent_id > args.agents
                for agent_id in args.moving_agents)):
        args.moving_agents = list(range(1, args.agents + 1))
    rclpy.init()
    monitor = RoutePlanningMonitor(args)
    try:
        while rclpy.ok() and not monitor.completed:
            rclpy.spin_once(monitor, timeout_sec=0.1)
        return monitor.finish()
    except KeyboardInterrupt:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 130


if __name__ == "__main__":
    sys.exit(main())

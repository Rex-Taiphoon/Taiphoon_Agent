#!/usr/bin/env python3
"""
@file ego_route_planning_monitor.py
@brief EGO 時空路徑規劃 SITL 的最小觀測器

此程式只讀取 AgentState、AgentTrajectory 與 PlannerStatus，所有控制命令
由外層驗證腳本透過既有 command topic 發布。它不修改任何 runtime safety
規則，也不新增 ROS2 topic。
"""

import argparse
import math
import sys
import time
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node

from taiphoon_agent.msg import AgentState, AgentTrajectory, PlannerStatus


class RoutePlanningMonitor(Node):
    """收集單機或多機路徑驗收所需的少量指標。"""

    def __init__(self, args: argparse.Namespace):
        super().__init__("ego_route_planning_sitl_monitor")
        self.agent_ids = list(range(1, args.agents + 1))
        self.moving_agent_ids = args.moving_agents or list(self.agent_ids)
        self.timeout_s = args.timeout_s
        self.min_distance_m = args.minimum_distance_m
        self.min_displacement_m = args.minimum_displacement_m
        self.require_lateral_m = args.require_lateral_m
        self.max_lateral_m = args.max_lateral_m
        self.lateral_axis = 1 if args.travel_axis == "north" else 0
        self.expect_failure = args.expect_failure
        self.metric_warmup_s = args.metric_warmup_s
        self.started_at = time.monotonic()
        self.global_reference: Optional[List[float]] = None
        self.states: Dict[int, Tuple[float, List[float]]] = {}
        self.baselines: Dict[int, List[float]] = {}
        self.reached: Set[int] = set()
        self.min_peer_distance_m = math.inf
        self.max_trajectory_z_span_m = 0.0
        self.max_lateral_offset_m = 0.0
        self.trajectory_ids: Dict[int, int] = {}
        self.trajectory_count = 0
        self.timeout_status_count = 0
        self.failure_status_count = 0
        self.latest_reason: Dict[int, str] = {}
        self.violation_started_at: Optional[float] = None
        self.max_contiguous_violation_s = 0.0
        self.failed = False
        self.completed = False
        self.metrics_started = False

        for agent_id in self.agent_ids:
            self.create_subscription(
                AgentState,
                f"/fmu{agent_id}_agent/state",
                lambda msg, current_id=agent_id: self.state_cb(
                    msg, current_id),
                20,
            )
            self.create_subscription(
                PlannerStatus,
                f"/fmu{agent_id}_agent/planning/status",
                lambda msg, current_id=agent_id: self.status_cb(
                    msg, current_id),
                20,
            )
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
        if self.metrics_started and \
                self.distance(position, self.baselines[agent_id]) >= \
                self.min_displacement_m:
            self.reached.add(agent_id)

    def status_cb(self, msg: PlannerStatus, agent_id: int) -> None:
        reason = str(msg.reason).lower()
        if self.latest_reason.get(agent_id) != reason:
            print(
                "TRACE_STATUS agent={} state={} reason={} planning_ms={:.3f} "
                "min_obstacle_m={:.3f} min_peer_m={:.3f}".format(
                    agent_id, msg.state, reason, float(msg.planning_time_ms),
                    float(msg.min_obstacle_distance_m),
                    float(msg.min_peer_distance_m)),
                flush=True,
            )
        self.latest_reason[agent_id] = reason
        if "timeout" in reason:
            self.timeout_status_count += 1
        if "no_feasible" in reason or "emergency" in reason:
            self.failure_status_count += 1

    def trajectory_cb(self, msg: AgentTrajectory) -> None:
        if not msg.valid or msg.agent_index not in self.agent_ids:
            return
        if msg.agent_index not in self.moving_agent_ids:
            return
        points = list(msg.control_points)
        if len(points) < 6 or len(points) % 3 != 0:
            return
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
        heights = [point[2] for point in global_points]
        self.max_trajectory_z_span_m = max(
            self.max_trajectory_z_span_m, max(heights) - min(heights))
        baseline = self.baselines.get(msg.agent_index)
        if baseline is not None:
            self.max_lateral_offset_m = max(
                self.max_lateral_offset_m,
                max(abs(point[self.lateral_axis] -
                        baseline[self.lateral_axis])
                    for point in common_points),
            )
        previous_id = self.trajectory_ids.get(msg.agent_index)
        if previous_id != msg.trajectory_id:
            self.trajectory_ids[msg.agent_index] = msg.trajectory_id
            print(
                "TRACE_TRAJECTORY agent={} id={} hold={} duration_s={:.3f}"
                .format(msg.agent_index, msg.trajectory_id,
                        msg.terminal_hold, float(msg.duration_s)),
                flush=True,
            )

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
                                  self.states[second_id][1]),
                )
        if not math.isfinite(current_minimum):
            return
        self.min_peer_distance_m = min(
            self.min_peer_distance_m, current_minimum)
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
                time.monotonic() - self.violation_started_at,
            )
        if self.expect_failure:
            passed = (self.failure_status_count > 0 or
                      self.timeout_status_count > 0) and not self.reached
        else:
            passed = (set(self.moving_agent_ids) <= self.reached and
                      self.trajectory_count > 0 and
                      self.max_trajectory_z_span_m <= 0.05 and
                      self.max_contiguous_violation_s <= 0.5 and
                      self.timeout_status_count == 0 and
                      self.max_lateral_offset_m + 1.0e-6 >=
                      self.require_lateral_m and
                      (self.max_lateral_m <= 0.0 or
                       self.max_lateral_offset_m <= self.max_lateral_m))
        self.failed = not passed
        print(
            "ROUTE_RESULT pass={} reached={} min_peer_distance_m={:.6f} "
            "max_contiguous_below_min_s={:.6f} max_trajectory_z_span_m={:.6f} "
            "max_lateral_offset_m={:.6f} timeout_status_count={} "
            "failure_status_count={} trajectory_count={}".format(
                passed, sorted(self.reached),
                self.min_peer_distance_m
                if math.isfinite(self.min_peer_distance_m) else -1.0,
                self.max_contiguous_violation_s,
                self.max_trajectory_z_span_m, self.max_lateral_offset_m,
                self.timeout_status_count, self.failure_status_count,
                self.trajectory_count),
            flush=True,
        )
        self.destroy_node()
        rclpy.shutdown()
        return 0 if passed else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--agents", type=int, required=True)
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument("--minimum-distance-m", type=float, default=3.0)
    parser.add_argument("--minimum-displacement-m", type=float, default=8.0)
    parser.add_argument("--metric-warmup-s", type=float, default=3.0)
    parser.add_argument(
        "--moving-agents", type=int, nargs="+", default=None,
        help="需要到達位移門檻的 Agent；未指定時為全部 Agent",
    )
    parser.add_argument("--require-lateral-m", type=float, default=0.0)
    parser.add_argument("--max-lateral-m", type=float, default=0.0)
    parser.add_argument(
        "--travel-axis", choices=("north", "east"), default="north",
        help="主要移動軸；側向偏移使用其垂直軸計算",
    )
    parser.add_argument("--expect-failure", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.agents <= 0:
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

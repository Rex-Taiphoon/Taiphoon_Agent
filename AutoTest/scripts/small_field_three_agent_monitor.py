#!/usr/bin/env python3
"""以既有 ROS2 介面驗證小場域三機腳本的前五步。

@file small_field_three_agent_monitor.py
@brief 9×9×8 m 三機編隊與 Agent 失效驗證；失效機另須查核 PX4 日誌
@author Taiphoon Agent Team
@date 2026-10-06
"""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import SetBool
from taiphoon_agent.msg import AgentState, AgentTrajectory, FormationEpoch, PlannerStatus


class SmallFieldMonitor(Node):
    """命令只走 Agent topic／service，原始觀測另輸出給離線查核。"""

    def __init__(self, failed_agent, scenario="formation"):
        super().__init__("small_field_three_agent_validation", namespace="autotest")
        self.failed_agent = failed_agent
        self.scenario = scenario
        self.members = [1, 2, 3]
        self.states = {}
        self.statuses = {}
        self.epochs = {}
        self.reference = None
        self.phase = "ready"
        self.phase_started = time.monotonic()
        self.stable_since = None
        self.last_report = 0.0
        self.min_distance = math.inf
        self.max_height = 0.0
        self.max_horizontal = 0.0
        self.failure = None
        self.completed = False
        self.shutdown_future = None
        self.crossing_targets = {}
        self.subscriptions_keepalive = []
        self.commands = self.create_publisher(String, "/uav_all_agent/command", 10)
        self.individual_commands = {agent: self.create_publisher(
            String, f"/uav_{agent}_agent/command", 10) for agent in (1, 3)}
        self.shutdown_client = self.create_client(SetBool, f"/uav_{failed_agent}_agent/shutdown")
        for agent in self.members:
            self.subscriptions_keepalive.append(self.create_subscription(
                AgentState, f"/uav_{agent}_agent/state",
                lambda msg, agent_id=agent: self.state_cb(msg, agent_id), 20))
            self.subscriptions_keepalive.append(self.create_subscription(
                PlannerStatus, f"/uav_{agent}_agent/planning/status",
                lambda msg, agent_id=agent: self.status_cb(msg, agent_id), 20))
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_keepalive.append(self.create_subscription(
            FormationEpoch, "/uav_all_agent/swarm/formation_epochs", self.epoch_cb, qos))
        self.subscriptions_keepalive.append(self.create_subscription(
            AgentTrajectory, "/uav_all_agent/swarm/agent_trajectories", self.trajectory_cb, 20))
        self.timer = self.create_timer(0.05, self.tick)

    def emit(self, event, **values):
        print(json.dumps({"event": event, "time_s": time.time(), "phase": self.phase,
                          **values}, ensure_ascii=False), flush=True)

    def project(self, point):
        scale = 111319.49079327358
        return [(point[0] - self.reference[0]) * scale,
                (point[1] - self.reference[1]) * scale * math.cos(math.radians(self.reference[0])),
                point[2] - self.reference[2]]

    def state_cb(self, msg, agent):
        if msg is None:
            return
        point = list(msg.position_global)
        if not all(math.isfinite(value) for value in point):
            return
        self.states[agent] = (time.monotonic(), point, msg)
        if self.reference is not None:
            self.emit("state", agent=agent, position=self.project(point),
                      armed=msg.armed, landed=msg.landed, nav_state=msg.px4_nav_state)

    def status_cb(self, msg, agent):
        if msg is not None:
            self.statuses[agent] = (time.monotonic(), msg)

    def epoch_cb(self, msg):
        if msg is not None and msg.valid and msg.status == FormationEpoch.ADOPTED:
            self.epochs[msg.sender_agent_id] = (time.monotonic(), msg)

    def trajectory_cb(self, msg):
        if self.reference is None or msg is None or not msg.valid:
            return
        if len(msg.control_points) % 3 != 0:
            self.failure = "invalid_trajectory_control_points"
            return
        points = []
        for index in range(0, len(msg.control_points), 3):
            point = list(msg.control_points[index:index + 3])
            if not all(math.isfinite(value) for value in point):
                self.failure = "nonfinite_trajectory"
                return
            local = self.project(point)
            points.append(local)
        if not points or msg.order != 3 or len(points) <= 3:
            self.failure = "unsupported_trajectory"
            return
        if all(max(abs(point[0]), abs(point[1])) <= 4.5 and point[2] <= 8.0
               for point in points):
            return
        # 控制點凸包超界不代表曲線超界。對每個 cubic knot span 求座標極值。
        spans = len(points) - 3
        knots = [0.0] * 4 + list(range(1, spans)) + [float(spans)] * 4

        def basis(index, degree, value):
            if degree == 0:
                return float(knots[index] <= value < knots[index + 1])
            left = knots[index + degree] - knots[index]
            right = knots[index + degree + 1] - knots[index + 1]
            return ((value - knots[index]) / left * basis(index, degree - 1, value) if left else 0.0) + (
                (knots[index + degree + 1] - value) / right * basis(index + 1, degree - 1, value)
                if right else 0.0)

        def sample(value, axis):
            if value >= spans:
                return points[-1][axis]
            return sum(point[axis] * basis(index, 3, value) for index, point in enumerate(points))

        for span in range(spans):
            for axis in range(3):
                values = [sample(span + index / 3.0, axis) for index in range(4)]
                first = values[1] - values[0]
                second = values[2] - 2.0 * values[1] + values[0]
                third = values[3] - 3.0 * values[2] + 3.0 * values[1] - values[0]
                a, b, c = 4.5 * third, 4.5 * (second - third), 3.0 * first - 1.5 * second + third
                roots = [0.0, 1.0]
                if abs(a) > 1e-10:
                    discriminant = 4.0 * b * b - 12.0 * a * c
                    if discriminant >= 0:
                        roots.extend([(-2.0 * b + sign * math.sqrt(discriminant)) / (6.0 * a)
                                      for sign in (-1.0, 1.0)])
                elif abs(b) > 1e-10:
                    roots.append(-c / (2.0 * b))
                for root in roots:
                    if 0.0 <= root <= 1.0:
                        value = ((a * root + b) * root + c) * root + values[0]
                        if (axis < 2 and abs(value) > 4.5 + 1e-6) or (axis == 2 and value > 8.0 + 1e-6):
                            self.emit("trajectory_boundary_violation", agent=msg.agent_index,
                                      axis=axis, value_m=value, trajectory_id=msg.trajectory_id)
                            self.failure = f"trajectory_outside_field_agent_{msg.agent_index}"
                            return

    def publish(self, command):
        self.commands.publish(String(data=command))
        self.emit("command", command=command)

    def advance(self, phase):
        self.phase = phase
        self.phase_started = time.monotonic()
        self.stable_since = None
        self.emit("phase_started")

    def fresh(self, now):
        return all(agent in self.states and now - self.states[agent][0] < 0.5
                   for agent in self.members)

    def stable(self, condition, now, duration=1.0):
        if not condition:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        return now - self.stable_since >= duration

    def formation_ready(self, now, north):
        if not self.fresh(now):
            return False
        identities = set()
        for agent in self.members:
            if agent not in self.statuses or now - self.statuses[agent][0] > 0.5:
                return False
            if self.statuses[agent][1].reason != "formation_converged; terminal hold active":
                return False
            if agent not in self.epochs or now - self.epochs[agent][0] > 2.0:
                return False
            epoch = self.epochs[agent][1]
            if list(epoch.member_agent_ids) != self.members:
                return False
            identities.add((epoch.sequence, epoch.snapshot_hash))
        positions = [self.project(self.states[agent][1]) for agent in self.members]
        center = [sum(point[axis] for point in positions) / len(positions) for axis in range(3)]
        return (len(identities) == 1 and abs(center[0] - north) < 0.35 and
                abs(center[1]) < 0.35 and
                all(abs(point[2] - 2.0) < 0.35 and
                    abs(math.hypot(point[0] - center[0], point[1] - center[1]) - 2.0) < 0.35
                    for point in positions))

    def tick(self):
        if self.completed or self.failure is not None:
            return
        now = time.monotonic()
        if now - self.phase_started > 120.0:
            self.failure = f"phase_timeout_{self.phase}"
            return
        if self.reference is None:
            if not self.fresh(now):
                return
            if not all(self.states[agent][2].landed and not self.states[agent][2].armed and
                       self.states[agent][2].px4_state_valid for agent in self.members):
                self.failure = "expected_three_landed_disarmed_simulators"
                return
            self.reference = [sum(self.states[agent][1][axis] for agent in self.members) / 3.0
                              for axis in range(3)]
            self.emit("field_reference", reference=self.reference)
            self.publish("MODE_OFFBOARD")
            self.advance("offboard")
            return
        if self.fresh(now):
            points = [self.project(self.states[agent][1]) for agent in self.members]
            for point in points:
                self.max_height = max(self.max_height, point[2])
                self.max_horizontal = max(self.max_horizontal, abs(point[0]), abs(point[1]))
                if max(abs(point[0]), abs(point[1])) > 4.5 or point[2] > 8.0:
                    self.failure = "observed_field_violation"
            for index, first in enumerate(points):
                for second in points[index + 1:]:
                    distance = math.dist(first, second)
                    self.min_distance = min(self.min_distance, distance)
                    if distance < 2.0:
                        self.failure = "observed_separation_below_2m"
        elif self.phase not in ("shutdown", "two_members"):
            self.failure = "unexpected_state_loss"
        if self.failure is not None:
            return
        if now - self.last_report > 2.0:
            self.last_report = now
            self.emit("progress", members=self.members, min_distance_m=self.min_distance)
        if self.phase == "offboard":
            if self.fresh(now) and all(self.states[agent][2].px4_nav_state == 14 for agent in self.members):
                self.publish("ARM")
                self.advance("armed")
        elif self.phase == "armed":
            if (now - self.phase_started >= 2.0 and self.fresh(now) and
                    all(self.states[agent][2].armed for agent in self.members)):
                self.publish("TAKEOFF 2")
                self.advance("takeoff")
        elif self.phase == "takeoff":
            ready = self.fresh(now) and all(abs(self.project(self.states[agent][1])[2] - 2.0) < 0.3
                                           for agent in self.members)
            if self.stable(ready, now):
                self.emit("step_pass", step=1)
                if self.scenario == "crossing":
                    first = self.project(self.states[1][1])
                    third = self.project(self.states[3][1])
                    self.crossing_targets = {1: third, 3: first}
                    distance = third[1] - first[1]
                    for agent, command in ((1, f"MOVE_RIGHT {distance}"),
                                           (3, f"MOVE_LEFT {distance}")):
                        self.individual_commands[agent].publish(String(data=command))
                        self.emit("command", agent=agent, command=command)
                    self.advance("crossing")
                else:
                    self.publish("FORMATION_CIRCLE 2 2")
                    self.advance("circle")
        elif self.phase == "crossing":
            reached = self.fresh(now) and all(
                math.dist(self.project(self.states[agent][1]), target) < 0.4
                for agent, target in self.crossing_targets.items())
            if self.stable(reached, now, 3.0):
                self.emit("crossing_pass")
                self.completed = True
        elif self.phase == "circle":
            if self.stable(self.formation_ready(now, 0.0), now):
                self.emit("step_pass", step=2)
                self.publish("MOVE_FORWARD 1")
                self.advance("forward")
        elif self.phase == "forward":
            if self.stable(self.formation_ready(now, 1.0), now):
                self.emit("forward_pass")
                self.publish("MOVE_BACKWARD 1")
                self.advance("backward")
        elif self.phase == "backward":
            if self.stable(self.formation_ready(now, 0.0), now):
                self.emit("step_pass", step=3)
                if not self.shutdown_client.service_is_ready():
                    self.failure = "shutdown_service_unavailable"
                    return
                request = SetBool.Request()
                request.data = True
                self.shutdown_future = self.shutdown_client.call_async(request)
                self.advance("shutdown")
        elif self.phase == "shutdown":
            if self.shutdown_future.done():
                response = self.shutdown_future.result()
                if response is None or not response.success:
                    self.failure = "shutdown_request_rejected"
                    return
                if now - self.states[self.failed_agent][0] > 1.0:
                    self.emit("step_pass", step=4, failed_agent=self.failed_agent)
                    self.members = [agent for agent in self.members if agent != self.failed_agent]
                    self.advance("two_members")
        elif self.phase == "two_members":
            if self.stable(self.formation_ready(now, 0.0), now, 15.0):
                self.emit("step_pass", step=5)
                self.completed = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--failed-agent", type=int, choices=(1, 2, 3), default=2)
    parser.add_argument("--scenario", choices=("formation", "crossing"), default="formation")
    args = parser.parse_args()
    rclpy.init()
    monitor = SmallFieldMonitor(args.failed_agent, args.scenario)
    try:
        while rclpy.ok() and not monitor.completed and monitor.failure is None:
            rclpy.spin_once(monitor, timeout_sec=0.1)
        monitor.emit("result", pass_live=monitor.completed and monitor.failure is None,
                     failure=monitor.failure, min_distance_m=monitor.min_distance,
                     max_height_m=monitor.max_height, max_horizontal_m=monitor.max_horizontal,
                     requires_failed_vehicle_log_check=True)
        return 0 if monitor.completed and monitor.failure is None else 1
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

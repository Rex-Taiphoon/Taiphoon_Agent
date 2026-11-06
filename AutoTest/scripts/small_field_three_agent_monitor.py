#!/usr/bin/env python3
"""以既有 ROS2 介面驗證小場域三機腳本的前五步。

@file small_field_three_agent_monitor.py
@brief 小場域（預設 9×9×8 m，可設定旋轉矩形）三機編隊與 Agent 失效驗證；失效機另須查核 PX4 日誌
@author Taiphoon Agent Team
@date 2026-10-06
"""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from taiphoon_agent.msg import (AgentAttitude, AgentState, AgentTrajectory, FormationEpoch,
                                Geofence, GeofenceStatus, GeofenceVertex, PlannerStatus)

# 本驗證發布的圍籬版本號；每個案例都重新啟動 Agent，因此固定即可。
GEOFENCE_ID = 1


def parse_geofence_polygon(text):
    """解析 'lat,lon;lat,lon;...'；空字串回傳空列表。"""
    vertices = []
    for item in text.split(";"):
        if item.strip():
            latitude, longitude = (float(value) for value in item.split(","))
            vertices.append((latitude, longitude))
    if vertices and len(vertices) < 3:
        raise ValueError("geofence polygon needs at least 3 vertices")
    return vertices


class SmallFieldMonitor(Node):
    """命令只走 Agent topic／service，原始觀測另輸出給離線查核。"""

    def __init__(self, failed_agent, scenario="formation", tolerance_scale=1.0,
                 min_separation_m=2.0, field_half_m=4.5, field_half_long_m=None,
                 field_half_short_m=None, field_bearing_deg=0.0, heading_deg=None,
                 single_move_m=0.0, circle_radius_m=2.0, takeoff_alt_m=2.0,
                 group_move_m=1.0, geofence_vertices=(), geofence_margin_m=0.5):
        super().__init__("small_field_three_agent_validation", namespace="autotest")
        self.failed_agent = failed_agent
        self.scenario = scenario
        # 預設值為驗收標準；放寬僅供模擬器精度不足時診斷功能。
        self.tolerance_scale = tolerance_scale
        self.min_separation_m = min_separation_m
        # 場域為以三機起始中心為原點的矩形；長軸方位 field_bearing_deg（北=0、順時針）。
        self.field_half_long_m = field_half_m if field_half_long_m is None else field_half_long_m
        self.field_half_short_m = field_half_m if field_half_short_m is None else field_half_short_m
        self.field_bearing_rad = math.radians(field_bearing_deg)
        # heading_deg 不為 None 時，起飛後以 YAW 轉到該航向，模擬實機起飛前的機頭朝向。
        self.heading_deg = heading_deg
        self.single_move_m = single_move_m
        self.circle_radius_m = circle_radius_m
        self.takeoff_alt_m = takeoff_alt_m
        self.group_move_m = group_move_m
        # 圍籬由地面站設定：起飛前發布，三台 GeofenceStatus 都 ACTIVE 後才開始。
        self.geofence_vertices = list(geofence_vertices)
        self.geofence_margin_m = geofence_margin_m
        self.geofence_statuses = {}
        self.attitudes = {}
        self.single_move_start = None
        self.last_status_reason = {}
        self.logged_epochs = set()
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
        self.max_field_long = 0.0
        self.max_field_short = 0.0
        self.observed_field_margin_m = 0.0
        self.observed_excursions = 0
        self.failure = None
        self.completed = False
        self.shutdown_future = None
        self.crossing_targets = {}
        self.subscriptions_keepalive = []
        self.commands = self.create_publisher(String, "/uav_all_agent/command", 10)
        self.individual_commands = {agent: self.create_publisher(
            String, f"/uav_{agent}_agent/command", 10) for agent in (1, 2, 3)}
        self.shutdown_client = self.create_client(SetBool, f"/uav_{failed_agent}_agent/shutdown")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.geofence_pub = self.create_publisher(Geofence, "/uav_all_agent/geofence", latched)
        for agent in self.members:
            self.subscriptions_keepalive.append(self.create_subscription(
                GeofenceStatus, f"/uav_{agent}_agent/geofence/status",
                lambda msg, agent_id=agent: self.geofence_status_cb(msg, agent_id), latched))
        for agent in self.members:
            self.subscriptions_keepalive.append(self.create_subscription(
                AgentState, f"/uav_{agent}_agent/state",
                lambda msg, agent_id=agent: self.state_cb(msg, agent_id), 20))
            self.subscriptions_keepalive.append(self.create_subscription(
                PlannerStatus, f"/uav_{agent}_agent/planning/status",
                lambda msg, agent_id=agent: self.status_cb(msg, agent_id), 20))
            self.subscriptions_keepalive.append(self.create_subscription(
                AgentAttitude, f"/uav_{agent}_agent/attitude",
                lambda msg, agent_id=agent: self.attitude_cb(msg, agent_id), 20))
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_keepalive.append(self.create_subscription(
            FormationEpoch, "/uav_all_agent/swarm/formation_epochs", self.epoch_cb, qos))
        self.subscriptions_keepalive.append(self.create_subscription(
            AgentTrajectory, "/uav_all_agent/swarm/agent_trajectories", self.trajectory_cb, 20))
        self.timer = self.create_timer(0.05, self.tick)

    def geofence_status_cb(self, msg, agent):
        if msg is None:
            return
        self.geofence_statuses[agent] = msg
        self.emit("geofence_status", agent=agent, state=msg.state, fence_id=msg.fence_id,
                  last_request_id=msg.last_request_id, last_result=msg.last_result,
                  position_inside=msg.position_inside, detail=msg.detail)

    def publish_geofence(self):
        msg = Geofence()
        msg.stamp = self.get_clock().now().to_msg()
        msg.fence_id = GEOFENCE_ID
        msg.action = Geofence.ACTION_SET
        msg.margin_m = float(self.geofence_margin_m)
        for latitude, longitude in self.geofence_vertices:
            vertex = GeofenceVertex()
            vertex.latitude_deg = latitude
            vertex.longitude_deg = longitude
            msg.vertices.append(vertex)
        self.geofence_pub.publish(msg)
        self.emit("command", command=f"GEOFENCE_SET {GEOFENCE_ID}",
                  vertices=len(msg.vertices), margin_m=msg.margin_m)

    def emit(self, event, **values):
        print(json.dumps({"event": event, "time_s": time.time(), "phase": self.phase,
                          **values}, ensure_ascii=False), flush=True)

    def project(self, point):
        scale = 111319.49079327358
        return [(point[0] - self.reference[0]) * scale,
                (point[1] - self.reference[1]) * scale * math.cos(math.radians(self.reference[0])),
                point[2] - self.reference[2]]

    def field_frame(self, local):
        """將 [北, 東, 上] 轉成 [場域長軸, 場域短軸, 上]。"""
        cos_b = math.cos(self.field_bearing_rad)
        sin_b = math.sin(self.field_bearing_rad)
        return [local[0] * cos_b + local[1] * sin_b,
                -local[0] * sin_b + local[1] * cos_b,
                local[2]]

    def outside_field(self, local, margin=0.0):
        point = self.field_frame(local)
        return (abs(point[0]) > self.field_half_long_m + margin or
                abs(point[1]) > self.field_half_short_m + margin or point[2] > 8.0 + margin)

    def heading_unit(self, offset_deg=0.0):
        heading = math.radians((self.heading_deg or 0.0) + offset_deg)
        return [math.cos(heading), math.sin(heading)]

    def attitude_cb(self, msg, agent):
        if msg is None:
            return
        w, x, y, z = (float(value) for value in msg.q)
        if not all(math.isfinite(value) for value in (w, x, y, z)):
            return
        yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
        self.attitudes[agent] = (time.monotonic(), yaw)

    def yaw_error_deg(self, agent, target_deg):
        return abs((self.attitudes[agent][1] - target_deg + 180.0) % 360.0 - 180.0)

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
            # 診斷：規劃狀態原因改變時記錄，供離線追查編隊轉換與煞車。
            if self.reference is not None and self.last_status_reason.get(agent) != msg.reason:
                self.last_status_reason[agent] = msg.reason
                self.emit("planner_status", agent=agent, state=int(msg.state), reason=msg.reason,
                          min_peer_distance_m=float(msg.min_peer_distance_m))

    def epoch_cb(self, msg):
        if msg is not None and msg.valid and msg.status == FormationEpoch.ADOPTED:
            self.epochs[msg.sender_agent_id] = (time.monotonic(), msg)
            identity = (msg.sequence, msg.snapshot_hash)
            # 診斷：每個 epoch 只記錄一次中心、成員快照與 parent 槽位。
            if self.reference is not None and identity not in self.logged_epochs:
                self.logged_epochs.add(identity)
                center = self.project([msg.center_latitude_deg, msg.center_longitude_deg,
                                       msg.center_altitude_m])
                snapshots = [self.project([lat, lon, alt]) for lat, lon, alt in zip(
                    msg.member_latitudes_deg, msg.member_longitudes_deg, msg.member_altitudes_m)]
                self.emit("epoch", sequence=msg.sequence, parent_sequence=msg.parent_sequence,
                          snapshot_hash=msg.snapshot_hash, proposer=msg.proposer_agent_id,
                          formation_type=int(msg.formation_type), radius_m=msg.radius_m,
                          spacing_m=msg.spacing_m, center=center,
                          members=list(msg.member_agent_ids), member_snapshots=snapshots,
                          parent_agents=list(msg.parent_assignment_agent_ids),
                          parent_slots=list(msg.parent_assignment_slot_indices))

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
            # B-spline 對仿射轉換不變，直接以場域座標的控制點檢查長／短軸邊界。
            local = self.field_frame(self.project(point))
            points.append(local)
        if not points or msg.order != 3 or len(points) <= 3:
            self.failure = "unsupported_trajectory"
            return
        if all(abs(point[0]) <= self.field_half_long_m and
               abs(point[1]) <= self.field_half_short_m and point[2] <= 8.0
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
                        limits = (self.field_half_long_m, self.field_half_short_m)
                        if (axis < 2 and abs(value) > limits[axis] + 1e-6) or (axis == 2 and value > 8.0 + 1e-6):
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
        tolerance = 0.35 * self.tolerance_scale
        return (len(identities) == 1 and abs(center[0] - north) < tolerance and
                abs(center[1]) < tolerance and
                all(abs(point[2] - self.takeoff_alt_m) < tolerance and
                    abs(math.hypot(point[0] - center[0], point[1] - center[1]) -
                        self.circle_radius_m) < tolerance
                    for point in positions))

    def start_after_takeoff(self):
        """起飛（及機頭、單機移動）完成後，開始交叉或編隊情境。"""
        if self.scenario == "crossing":
            first = self.project(self.states[1][1])
            third = self.project(self.states[3][1])
            right = self.heading_unit(90.0)
            distance = (third[0] - first[0]) * right[0] + (third[1] - first[1]) * right[1]
            self.crossing_targets = {
                1: [first[0] + distance * right[0], first[1] + distance * right[1], first[2]],
                3: [third[0] - distance * right[0], third[1] - distance * right[1], third[2]]}
            for agent, command in ((1, f"MOVE_RIGHT {distance:.2f}"),
                                   (3, f"MOVE_LEFT {distance:.2f}")):
                self.individual_commands[agent].publish(String(data=command))
                self.emit("command", agent=agent, command=command)
            self.advance("crossing")
        else:
            self.publish(f"FORMATION_CIRCLE {self.circle_radius_m:g} {self.takeoff_alt_m:g}")
            self.advance("circle")

    def single_move_target(self, sign):
        forward = self.heading_unit()
        start = self.single_move_start
        return [start[0] + sign * self.single_move_m * forward[0],
                start[1] + sign * self.single_move_m * forward[1], start[2]]

    def after_heading(self):
        if self.single_move_m > 0.0:
            self.single_move_start = self.project(self.states[2][1])
            command = f"MOVE_FORWARD {self.single_move_m:g}"
            self.individual_commands[2].publish(String(data=command))
            self.emit("command", agent=2, command=command)
            self.advance("single_forward")
        else:
            self.start_after_takeoff()

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
            if self.geofence_vertices:
                self.publish_geofence()
                self.advance("geofence")
            else:
                self.publish("MODE_OFFBOARD")
                self.advance("offboard")
            return
        if self.fresh(now):
            points = [self.project(self.states[agent][1]) for agent in self.members]
            for point in points:
                self.max_height = max(self.max_height, point[2])
                self.max_horizontal = max(self.max_horizontal, abs(point[0]), abs(point[1]))
                field_point = self.field_frame(point)
                self.max_field_long = max(self.max_field_long, abs(field_point[0]))
                self.max_field_short = max(self.max_field_short, abs(field_point[1]))
                # 規劃軌跡一律嚴格檢查；實測位置另允許 observed_field_margin_m 的 PX4 追蹤誤差，
                # 越界量全部記錄（observed_field_excursion）供人工判讀。
                if self.outside_field(point):
                    self.observed_excursions += 1
                    if self.observed_excursions == 1 or self.observed_excursions % 20 == 0:
                        self.emit("observed_field_excursion", field_position=field_point)
                if self.outside_field(point, self.observed_field_margin_m):
                    self.failure = "observed_field_violation"
            for index, first in enumerate(points):
                for second in points[index + 1:]:
                    distance = math.dist(first, second)
                    self.min_distance = min(self.min_distance, distance)
                    if distance < self.min_separation_m:
                        self.failure = "observed_separation_below_2m"
        elif self.phase not in ("shutdown", "two_members"):
            self.failure = "unexpected_state_loss"
        if self.failure is not None:
            return
        if now - self.last_report > 2.0:
            self.last_report = now
            self.emit("progress", members=self.members, min_distance_m=self.min_distance)
        if self.phase == "geofence":
            statuses = [self.geofence_statuses.get(agent) for agent in self.members]
            if any(status is not None and status.last_request_id == GEOFENCE_ID and
                   status.last_result in (GeofenceStatus.RESULT_REJECTED_INVALID,
                                          GeofenceStatus.RESULT_REJECTED_TOO_FAR)
                   for status in statuses):
                self.failure = "geofence_rejected"
                return
            if all(status is not None and status.state == GeofenceStatus.STATE_ACTIVE and
                   status.fence_id == GEOFENCE_ID for status in statuses):
                self.emit("geofence_active", fence_id=GEOFENCE_ID)
                self.publish("MODE_OFFBOARD")
                self.advance("offboard")
        elif self.phase == "offboard":
            if self.fresh(now) and all(self.states[agent][2].px4_nav_state == 14 for agent in self.members):
                self.publish("ARM")
                self.advance("armed")
        elif self.phase == "armed":
            if (now - self.phase_started >= 2.0 and self.fresh(now) and
                    all(self.states[agent][2].armed for agent in self.members)):
                self.publish(f"TAKEOFF {self.takeoff_alt_m:g}")
                self.advance("takeoff")
        elif self.phase == "takeoff":
            ready = self.fresh(now) and all(
                abs(self.project(self.states[agent][1])[2] - self.takeoff_alt_m) < 0.3
                for agent in self.members)
            # Agent 每 3 秒掃描一次同伴，需連續 2 次一致才列為穩定成員；離地後最多約 6 秒
            # 才會接受編隊命令，因此到位後先懸停 7 秒再下達，避免單機以「成員不足」拒絕。
            if self.stable(ready, now, 7.0):
                self.emit("step_pass", step=1)
                if self.heading_deg is not None:
                    if not all(agent in self.attitudes and now - self.attitudes[agent][0] < 0.5
                               for agent in self.members):
                        self.failure = "attitude_unavailable"
                        return
                    # YAW 為相對角度，依各機目前航向換算，使三機都轉到同一絕對航向。
                    for agent in self.members:
                        delta = (self.heading_deg - self.attitudes[agent][1] + 180.0) % 360.0 - 180.0
                        command = f"YAW {delta:.1f}"
                        self.individual_commands[agent].publish(String(data=command))
                        self.emit("command", agent=agent, command=command)
                    self.advance("heading")
                else:
                    self.after_heading()
        elif self.phase == "heading":
            aligned = all(agent in self.attitudes and now - self.attitudes[agent][0] < 0.5 and
                          self.yaw_error_deg(agent, self.heading_deg) < 5.0
                          for agent in self.members)
            if self.stable(aligned, now, 2.0):
                self.emit("heading_pass", yaw_deg={agent: self.attitudes[agent][1]
                                                   for agent in self.members})
                self.after_heading()
        elif self.phase in ("single_forward", "single_backward"):
            sign = 1.0 if self.phase == "single_forward" else 0.0
            target = self.single_move_target(sign)
            reached = self.fresh(now) and math.dist(
                self.project(self.states[2][1])[:2], target[:2]) < 0.4 * self.tolerance_scale
            if self.stable(reached, now, 2.0):
                self.emit(f"{self.phase}_pass", position=self.project(self.states[2][1]),
                          target=target)
                if self.phase == "single_forward":
                    command = f"MOVE_BACKWARD {self.single_move_m:g}"
                    self.individual_commands[2].publish(String(data=command))
                    self.emit("command", agent=2, command=command)
                    self.advance("single_backward")
                else:
                    self.start_after_takeoff()
        elif self.phase == "crossing":
            reached = self.fresh(now) and all(
                math.dist(self.project(self.states[agent][1]), target) < 0.4 * self.tolerance_scale
                for agent, target in self.crossing_targets.items())
            if self.stable(reached, now, 3.0):
                self.emit("crossing_pass")
                self.completed = True
        elif self.phase == "circle":
            if self.stable(self.formation_ready(now, 0.0), now):
                self.emit("step_pass", step=2)
                self.publish(f"MOVE_FORWARD {self.group_move_m:g}")
                self.advance("forward")
        elif self.phase == "forward":
            if self.stable(self.formation_ready(now, self.group_move_m), now):
                self.emit("forward_pass")
                self.publish(f"MOVE_BACKWARD {self.group_move_m:g}")
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
    # 位置容差倍率（不含起飛高度，避免爬升中就下達編隊）、最小機間距離與場域半寬；預設值為驗收標準。
    parser.add_argument("--tolerance-scale", type=float, default=1.0)
    parser.add_argument("--min-separation-m", type=float, default=2.0)
    parser.add_argument("--field-half-m", type=float, default=4.5)
    # 旋轉矩形場域：長／短軸半長（未指定時沿用 --field-half-m）與長軸方位（北=0、順時針）。
    parser.add_argument("--field-half-long-m", type=float, default=None)
    parser.add_argument("--field-half-short-m", type=float, default=None)
    parser.add_argument("--field-bearing-deg", type=float, default=0.0)
    # 起飛後轉到的絕對航向（度）；省略時維持起飛航向。
    parser.add_argument("--heading-deg", type=float, default=None)
    # 中間機（uav_2）沿機頭前進再退回的距離；0 表示略過單機移動步驟。
    parser.add_argument("--single-move-m", type=float, default=0.0)
    parser.add_argument("--circle-radius-m", type=float, default=2.0)
    parser.add_argument("--takeoff-alt-m", type=float, default=2.0)
    parser.add_argument("--group-move-m", type=float, default=1.0)
    # 實測位置超出場域的容許量（公尺）；預設 0 為嚴格判定，規劃軌跡不受此值影響。
    parser.add_argument("--observed-field-margin-m", type=float, default=0.0)
    # 地面站圍籬 'lat,lon;lat,lon;...'（WGS84）；空字串表示不設定。
    parser.add_argument("--geofence-polygon", default="")
    parser.add_argument("--geofence-margin-m", type=float, default=0.5)
    args = parser.parse_args()
    rclpy.init()
    monitor = SmallFieldMonitor(args.failed_agent, args.scenario, args.tolerance_scale,
                                args.min_separation_m, args.field_half_m, args.field_half_long_m,
                                args.field_half_short_m, args.field_bearing_deg, args.heading_deg,
                                args.single_move_m, args.circle_radius_m, args.takeoff_alt_m,
                                args.group_move_m, parse_geofence_polygon(args.geofence_polygon),
                                args.geofence_margin_m)
    monitor.observed_field_margin_m = args.observed_field_margin_m
    try:
        while rclpy.ok() and not monitor.completed and monitor.failure is None:
            rclpy.spin_once(monitor, timeout_sec=0.1)
        monitor.emit("result", pass_live=monitor.completed and monitor.failure is None,
                     failure=monitor.failure, min_distance_m=monitor.min_distance,
                     max_height_m=monitor.max_height, max_horizontal_m=monitor.max_horizontal,
                     max_field_long_m=monitor.max_field_long,
                     max_field_short_m=monitor.max_field_short,
                     observed_field_excursion_samples=monitor.observed_excursions,
                     requires_failed_vehicle_log_check=True)
        return 0 if monitor.completed and monitor.failure is None else 1
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

"""隔離 DDS domain 下的真實 planner 降落生命週期回歸。"""
import os
import subprocess
import tempfile
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from taiphoon_agent.msg import AgentState, AgentTrajectory, FormationEpoch, PlannerStatus


class LandingPlannerIntegrationTest(unittest.TestCase):
    def test_landing_clears_formation_and_targets(self):
        self.assertEqual(os.environ.get("ROS_DOMAIN_ID"), "77",
                         "禁止連接人工操作的機群，必須使用隔離 DDS domain")
        rclpy.init()
        node = Node("landing_lifecycle_observer")
        grounded_peer_node = Node("basic_controller_node", namespace="/uav_2")
        best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        epoch_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        state_pub = node.create_publisher(AgentState, "/uav_1_agent/state", best)
        grounded_peer_pub = node.create_publisher(AgentState, "/uav_2_agent/state", best)
        cloud_pub = node.create_publisher(PointCloud2, "/uav_1_agent/perception/obstacles", best)
        command_pub = node.create_publisher(String, "/uav_1_agent/command", 10)
        group_pub = node.create_publisher(String, "/uav_all_agent/command", 10)
        epoch_pub = node.create_publisher(FormationEpoch, "/uav_all_agent/swarm/formation_epochs", epoch_qos)
        peer_trajectory_pub = node.create_publisher(
            AgentTrajectory, "/uav_all_agent/swarm/agent_trajectories", best)
        local, peer, epochs, statuses = [], [], [], []
        subscriptions = [
            node.create_subscription(AgentTrajectory, "/uav_1_agent/planning/trajectory", local.append, best),
            node.create_subscription(AgentTrajectory, "/uav_all_agent/swarm/agent_trajectories", peer.append, best),
            node.create_subscription(FormationEpoch, "/uav_all_agent/swarm/formation_epochs", epochs.append, epoch_qos),
            node.create_subscription(PlannerStatus, "/uav_1_agent/planning/status", statuses.append, 10),
        ]
        state = AgentState()
        state.online = True
        state.px4_state_valid = True
        state.landed = True
        state.position_global = [25.033, 121.565, 490.0]
        grounded_peer = AgentState()
        grounded_peer.online = True
        grounded_peer.px4_state_valid = True
        grounded_peer.landed = True
        grounded_peer.position_global = [25.033, 121.565, 490.0]
        cloud = PointCloud2()
        cloud.header.frame_id = "swarm_ned"
        cloud.height = 1
        cloud.point_step = 12
        cloud.is_dense = True
        cloud.fields = [PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                        for index, name in enumerate(("x", "y", "z"))]

        def tick(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                stamp = node.get_clock().now().to_msg()
                state.stamp = stamp
                cloud.header.stamp = stamp
                state_pub.publish(state)
                grounded_peer.stamp = stamp
                grounded_peer_pub.publish(grounded_peer)
                cloud_pub.publish(cloud)
                rclpy.spin_once(node, timeout_sec=0.05)
                time.sleep(0.03)

        def send(pub, text):
            pub.publish(String(data=text))
            tick(0.3)

        def takeoff():
            state.armed = True
            state.landed = False
            state.position_ned = [0.0, 0.0, -2.0]
            state.position_global = [25.033, 121.565, 492.0]
            tick(2)

        executable = os.environ["TAIPHOON_PLANNER_EXECUTABLE"]
        log_dir = os.environ.get("TAIPHOON_TEST_LOG_DIR", tempfile.gettempdir())
        with open(os.path.join(log_dir, "landing-node.txt"), "w", encoding="utf-8") as log:
            process = subprocess.Popen([executable, "--ros-args", "-p", "agent_index:=1",
                                        "-p", "agent_namespace:=uav_1_agent"], stdout=log, stderr=subprocess.STDOUT)
            try:
                tick(2)
                self.assertIsNone(process.poll())
                self.assertFalse(any(msg.valid for msg in peer), "地面不得發布佔位")
                self.assertFalse(any(msg.valid for msg in local))
                takeoff()
                # 同儕先在空中廣播與本機重疊的佔位，然後落地。
                # 落地後延遲抵達的舊佔位也不得重新阻擋本機規劃。
                grounded_peer.armed = True
                grounded_peer.landed = False
                grounded_peer.position_global = [25.033, 121.565, 492.0]
                tick(0.5)
                old_occupancy = AgentTrajectory()
                old_occupancy.agent_index = 2
                old_occupancy.trajectory_id = 51
                old_occupancy.coordinate_frame = AgentTrajectory.COORDINATE_FRAME_WGS84_GEODETIC
                old_occupancy.frame_revision = 1
                old_occupancy.order = 3
                old_occupancy.knot_interval_s = 1.0
                old_occupancy.duration_s = 1.0
                old_occupancy.terminal_hold = True
                old_occupancy.valid = True
                old_occupancy.control_points = [25.033, 121.565, 492.0] * 4
                old_occupancy.stamp = node.get_clock().now().to_msg()
                old_occupancy.start_time = old_occupancy.stamp
                peer_trajectory_pub.publish(old_occupancy)
                tick(0.3)
                grounded_peer.armed = False
                grounded_peer.landed = True
                grounded_peer.position_global = [25.033, 121.565, 490.0]
                tick(0.3)
                peer_trajectory_pub.publish(old_occupancy)
                send(command_pub, "MOVE_POINT 3 0 -2 0")
                tick(3)
                self.assertTrue(any(msg.valid for msg in local), "空中移動應有本機軌跡")
                self.assertTrue(any(msg.valid and msg.control_points and
                                    max(msg.control_points[0::3]) - min(msg.control_points[0::3]) > 1e-6
                                    for msg in local), "必須有移動軌跡，不能把固定煞車當成成功")
                self.assertTrue(statuses[-1].map_valid, "必須實際提供有效障礙物地圖")
                send(group_pub, "FORMATION_CIRCLE 2 2")
                tick(2)
                self.assertTrue(epochs, "編隊命令應建立 Epoch")
                saved_epoch = epochs[-1]
                self.assertEqual(list(saved_epoch.member_agent_ids), [1],
                                 "已降落且在線的同儕不得留在編隊成員名單")
                state.landed = True
                state.armed = False
                state.position_ned = [0.0, 0.0, 0.0]
                state.position_global = [25.033, 121.565, 490.0]
                tick(0.5)
                self.assertFalse(local[-1].valid, "降落應撤銷舊本機軌跡")
                local_start, peer_start = len(local), len(peer)
                send(command_pub, "MOVE_POINT 3 0 -2 0")
                send(group_pub, "FORMATION_CIRCLE 2 2")
                epoch_pub.publish(saved_epoch)
                tick(1)
                self.assertFalse(any(msg.valid for msg in local[local_start:]))
                self.assertFalse(any(msg.valid for msg in peer[peer_start:]),
                                 "降落後不得發布同儕佔位")
                self.assertTrue(statuses[-1].reason.startswith("Flight mission inactive"))
                takeoff()
                epoch_pub.publish(saved_epoch)
                tick(2)
                self.assertFalse(any(msg.valid for msg in local[local_start:]),
                                 "再次起飛與舊 Epoch 不能恢復舊任務")
                send(group_pub, "FORMATION_CIRCLE 2 2")
                tick(2)
                self.assertTrue(any(msg.valid for msg in local[local_start:]),
                                "新的空中編隊命令應可重新啟動")
            finally:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
                node.destroy_node()
                grounded_peer_node.destroy_node()
                rclpy.shutdown()
                del subscriptions


if __name__ == "__main__":
    unittest.main(verbosity=2)

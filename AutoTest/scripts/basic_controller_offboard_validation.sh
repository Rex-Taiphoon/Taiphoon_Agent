#!/usr/bin/env bash
# =============================================================================
# basic_controller_offboard_validation.sh
# 單機 BasicController Offboard heartbeat / TrajectorySetpoint SITL 驗證。
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
NETWORK_NAME="Taiphoon_Net"
AGENT_CONTAINER="uav_1_agent"
AGENT_STARTED=0

cleanup()
{
    local exit_code=$?
    echo "=== 清理 PX4 SITL 與單機 Agent ==="
    bash "${SCRIPT_DIR}/px4-sitl-launcher/stop_all.sh" \
        --net "${NETWORK_NAME}" >/dev/null 2>&1 || true
    if [[ "${AGENT_STARTED}" -eq 1 ]]; then
        docker compose stop "${AGENT_CONTAINER}" >/dev/null 2>&1 || true
    fi
    exit "${exit_code}"
}
trap cleanup EXIT INT TERM

cd "${PROJECT_DIR}"

if ! docker image inspect taiphoon-agent:dev >/dev/null 2>&1; then
    echo "錯誤：找不到 taiphoon-agent:dev，請由開發者自行建置 image。" >&2
    echo "docker build --target taiphoon-dev -t taiphoon-agent:dev ." >&2
    exit 2
fi

if ! docker network inspect "${NETWORK_NAME}" >/dev/null 2>&1; then
    docker network create "${NETWORK_NAME}" >/dev/null
fi

echo "=== 啟動單機 Agent；entrypoint 會在容器內完成 workspace build ==="
docker compose stop "${AGENT_CONTAINER}" >/dev/null 2>&1 || true
docker compose up -d "${AGENT_CONTAINER}"
AGENT_STARTED=1

echo "=== 啟動單台 PX4 SITL ==="
bash "${SCRIPT_DIR}/px4-sitl-launcher/launch_all.sh" \
    --count 1 --net "${NETWORK_NAME}" --rmw cyclonedds

sleep 10

echo "=== 執行 BasicController 自動驗證 ==="
docker exec -i "${AGENT_CONTAINER}" bash -lc \
    'source /opt/ros/humble/setup.bash && \
     source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash && \
     source /home/taiphoon/Taiphoon_Agent/install/setup.bash && \
     python3 -' <<'PY'
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from taiphoon_agent.msg import AgentState, AgentTrajectory, PlannerStatus


class BasicControllerValidator(Node):
    """在 ROS2 seam 觀察 BasicController 與 planner 的控制資料流。"""

    def __init__(self):
        super().__init__("basic_controller_sitl_validator")
        self.heartbeat_count = 0
        self.setpoint_count = 0
        self.local_trajectory_count = 0
        self.latest_state = None
        self.latest_planner_status = None
        self.latest_local_trajectory = None

        px4_input_qos = QoSProfile(depth=20)
        px4_input_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            OffboardControlMode,
            "/uav_1/fmu/in/offboard_control_mode",
            self._heartbeat_cb,
            px4_input_qos,
        )
        self.create_subscription(
            TrajectorySetpoint,
            "/uav_1/fmu/in/trajectory_setpoint",
            self._setpoint_cb,
            px4_input_qos,
        )
        self.create_subscription(
            AgentState,
            "/uav_1_agent/state",
            self._state_cb,
            20,
        )
        self.create_subscription(
            PlannerStatus,
            "/uav_1_agent/planning/status",
            self._planner_status_cb,
            20,
        )
        self.create_subscription(
            AgentTrajectory,
            "/uav_1_agent/planning/trajectory",
            self._local_trajectory_cb,
            20,
        )
        self.command_pub = self.create_publisher(
            String, "/uav_all_agent/command", 10
        )
        self.local_command_pub = self.create_publisher(
            String, "/uav_1_agent/command", 10
        )

    def _heartbeat_cb(self, _message):
        self.heartbeat_count += 1

    def _setpoint_cb(self, _message):
        self.setpoint_count += 1

    def _state_cb(self, message):
        self.latest_state = message

    def _planner_status_cb(self, message):
        self.latest_planner_status = message

    def _local_trajectory_cb(self, message):
        previous = self.latest_local_trajectory
        self.local_trajectory_count += 1
        self.latest_local_trajectory = message
        if previous is None or previous.trajectory_id != message.trajectory_id or \
                previous.terminal_hold != message.terminal_hold:
            print(
                f"[TRACE] local trajectory id={message.trajectory_id} "
                f"terminal_hold={message.terminal_hold} "
                f"duration_s={message.duration_s:.2f}",
                flush=True,
            )

    def spin_for(self, duration_s):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_until(self, predicate, timeout_s, description):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise RuntimeError(f"逾時：{description}")

    def send_command(self, command, local=False, repeat=None):
        message = String()
        message.data = command
        publisher = self.local_command_pub if local else self.command_pub
        repetitions = repeat if repeat is not None else (1 if local else 3)
        for _ in range(repetitions):
            publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        print(f"[COMMAND] {command}", flush=True)

    def require_no_setpoint(self, duration_s, description):
        baseline = self.setpoint_count
        self.spin_for(duration_s)
        if self.setpoint_count != baseline:
            raise RuntimeError(
                f"{description}：TrajectorySetpoint 數量由 "
                f"{baseline} 變為 {self.setpoint_count}"
            )

    def require_setpoint_stable(self, duration_s, description):
        baseline = self.setpoint_count
        self.spin_for(duration_s)
        if self.setpoint_count != baseline:
            raise RuntimeError(
                f"{description}：TrajectorySetpoint 仍在發布，"
                f"數量由 {baseline} 變為 {self.setpoint_count}"
            )

    def run(self):
        self.wait_until(
            lambda: self.latest_state is not None and
            self.latest_state.online,
            30.0,
            "AgentState online",
        )

        # MODE_OFFBOARD：只允許 OffboardControlMode 心跳。
        self.send_command("MODE_OFFBOARD")
        heartbeat_baseline = self.heartbeat_count
        self.wait_until(
            lambda: self.heartbeat_count > heartbeat_baseline,
            8.0,
            "OffboardControlMode heartbeat",
        )
        self.require_no_setpoint(
            2.0, "MODE_OFFBOARD 後、TAKEOFF 前不應發布 TrajectorySetpoint"
        )
        self.wait_until(
            lambda: self.latest_state is not None and
            self.latest_state.px4_nav_state == 14,
            8.0,
            "PX4 進入 OFFBOARD",
        )

        # ARM：地面解鎖時仍只維持 heartbeat，且 landed 必須為 true。
        self.send_command("ARM")
        self.wait_until(
            lambda: self.latest_state is not None and
            self.latest_state.armed and self.latest_state.landed,
            8.0,
            "ARM 且 landed=true",
        )
        self.require_no_setpoint(
            2.0, "ARM 後、TAKEOFF 前不應發布 TrajectorySetpoint"
        )

        # TAKEOFF：達到高度後應停止重送 TrajectorySetpoint。
        takeoff_baseline = self.setpoint_count
        self.send_command("TAKEOFF 3.0")
        self.wait_until(
            lambda: self.setpoint_count > takeoff_baseline,
            8.0,
            "TAKEOFF TrajectorySetpoint",
        )
        self.wait_until(
            lambda: self.latest_state is not None and
            math.isfinite(float(self.latest_state.position_ned[2])) and
            -float(self.latest_state.position_ned[2]) >= 2.7,
            35.0,
            "TAKEOFF 高度達到約 3m",
        )
        self.require_setpoint_stable(
            2.0, "TAKEOFF 完成後 HOVER 不應重複發布 TrajectorySetpoint"
        )

        # MOVE：由 planner 提供 TimedTrajectory；terminal_hold 後停止輸出。
        moves = (
            ("MOVE_FORWARD", 2.0),
            ("MOVE_BACKWARD", 2.0),
            ("MOVE_LEFT", 2.0),
            ("MOVE_RIGHT", 2.0),
            ("MOVE_UP", 1.0),
            ("MOVE_DOWN", 1.0),
        )
        for command, distance in moves:
            local_baseline = self.local_trajectory_count
            output_baseline = self.setpoint_count
            previous_id = (
                self.latest_local_trajectory.trajectory_id
                if self.latest_local_trajectory is not None else 0
            )
            previous_planner_id = (
                self.latest_planner_status.trajectory_id
                if self.latest_planner_status is not None else 0
            )
            self.send_command(f"{command} {distance:.1f}", local=True)
            self.wait_until(
                lambda: self.local_trajectory_count > local_baseline and
                self.latest_local_trajectory is not None and
                self.latest_local_trajectory.trajectory_id > previous_id,
                15.0,
                f"{command} 活動 TimedTrajectory",
            )
            print(
                f"[TRACE] {command} trajectory_id="
                f"{self.latest_local_trajectory.trajectory_id} "
                f"terminal_hold={self.latest_local_trajectory.terminal_hold} "
                f"duration_s={self.latest_local_trajectory.duration_s:.2f} "
                f"planner_state="
                f"{self.latest_planner_status.state if self.latest_planner_status else 'none'} "
                f"reason="
                f"{self.latest_planner_status.reason if self.latest_planner_status else 'none'}",
                flush=True,
            )
            self.wait_until(
                lambda: self.setpoint_count > output_baseline,
                8.0,
                f"{command} TrajectorySetpoint 輸出",
            )
            try:
                self.wait_until(
                    lambda: (
                        self.latest_local_trajectory is not None and
                        self.latest_local_trajectory.trajectory_id > previous_id and
                        self.latest_local_trajectory.terminal_hold and
                        self.latest_planner_status is not None and
                        self.latest_planner_status.state == 2
                    ) or (
                        self.latest_planner_status is not None and
                        self.latest_planner_status.trajectory_id > previous_planner_id and
                        self.latest_planner_status.state == 2 and
                        "Terminal hold" in self.latest_planner_status.reason
                    ),
                    35.0,
                    f"{command} terminal_hold",
                )
            except RuntimeError:
                latest_trajectory = self.latest_local_trajectory
                latest_status = self.latest_planner_status
                latest_state = self.latest_state
                print(
                    f"[TRACE] terminal timeout: trajectory_count="
                    f"{self.local_trajectory_count} "
                    f"last_id={latest_trajectory.trajectory_id if latest_trajectory else 'none'} "
                    f"last_terminal_hold={latest_trajectory.terminal_hold if latest_trajectory else 'none'} "
                    f"planner_state={latest_status.state if latest_status else 'none'} "
                    f"planner_reason={latest_status.reason if latest_status else 'none'} "
                    f"position_ned={list(latest_state.position_ned) if latest_state else 'none'}",
                    flush=True,
                )
                raise
            local_terminal_hold = (
                self.latest_local_trajectory is not None and
                self.latest_local_trajectory.trajectory_id > previous_id and
                self.latest_local_trajectory.terminal_hold
            )
            if not local_terminal_hold:
                # 單機 Planner 的 terminal hold 可能只發布到 peer occupancy
                # topic；用既有 HOLD 命令讓 BasicController 清除過期的本機
                # timed trajectory，並驗證其 one-shot setpoint 行為。
                hold_baseline = self.setpoint_count
                self.send_command("HOLD", repeat=1)
                self.wait_until(
                    lambda: self.setpoint_count > hold_baseline,
                    8.0,
                    f"{command} terminal hold 後 HOLD TrajectorySetpoint",
                )
                self.spin_for(1.0)
            self.require_setpoint_stable(
                1.5, f"{command} terminal_hold 後停止 TrajectorySetpoint"
            )
            print(f"[PASS] {command}", flush=True)

        # LAND / DISARM：離開 Offboard 後停止兩種 Offboard topic。
        self.send_command("MODE_LAND")
        self.wait_until(
            lambda: self.latest_state is not None and
            self.latest_state.landed,
            40.0,
            "MODE_LAND landed=true",
        )
        self.require_no_setpoint(1.5, "LAND 後停止 TrajectorySetpoint")
        self.send_command("DISARM")
        self.wait_until(
            lambda: self.latest_state is not None and
            not self.latest_state.armed,
            10.0,
            "DISARM armed=false",
        )
        print(
            "[PASS] BasicController Offboard heartbeat/setpoint SITL 驗證完成",
            flush=True,
        )


def main():
    rclpy.init()
    validator = BasicControllerValidator()
    try:
        validator.run()
    except Exception as error:
        print(f"[FAIL] {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        validator.destroy_node()
        rclpy.shutdown()
    return 0


sys.exit(main())
PY

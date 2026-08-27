#!/usr/bin/env python3
"""
@file wait_for_takeoff_altitude.py
@brief 等待所有 SITL Agent 到達指定起飛高度

只讀取既有 AgentState，不發布任何控制命令。每台 Agent 以收到的第一個
有效全球高度作為本次測試的地面基準，確認上升高度達標並連續保持一小段
時間後才結束，避免起飛追蹤誤差被帶入 MOVE 或 Formation 測試。
"""

import argparse
import math
import sys
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from taiphoon_agent.msg import AgentState


class TakeoffAltitudeWaiter(Node):
    """觀察每台 Agent 的全球高度上升量。"""

    def __init__(self, args: argparse.Namespace):
        super().__init__("wait_for_takeoff_altitude")
        self.agent_ids = list(range(1, args.agents + 1))
        self.target_altitude_m = args.target_altitude_m
        self.tolerance_m = args.tolerance_m
        self.hold_s = args.hold_s
        self.baseline_warmup_s = args.baseline_warmup_s
        self.timeout_s = args.timeout_s
        self.started_at = time.monotonic()
        self.baseline_altitude_m: Dict[int, float] = {}
        self.climb_altitude_m: Dict[int, float] = {}
        self.ready_since: Optional[float] = None
        self.completed = False
        self.passed = False

        for agent_id in self.agent_ids:
            self.create_subscription(
                AgentState,
                f"/uav_{agent_id}_agent/state",
                lambda msg, current_id=agent_id: self.state_cb(
                    msg, current_id),
                20,
            )
        self.create_timer(0.1, self.timer_cb)

    def state_cb(self, msg: AgentState, agent_id: int) -> None:
        altitude_m = float(msg.position_global[2])
        if not math.isfinite(altitude_m):
            return
        self.baseline_altitude_m.setdefault(agent_id, altitude_m)
        self.climb_altitude_m[agent_id] = (
            altitude_m - self.baseline_altitude_m[agent_id])

    def timer_cb(self) -> None:
        now = time.monotonic()
        all_ready = (
            set(self.climb_altitude_m) == set(self.agent_ids) and
            all(value >= self.target_altitude_m - self.tolerance_m
                for value in self.climb_altitude_m.values())
        )
        if now - self.started_at < self.baseline_warmup_s:
            all_ready = False
        if all_ready:
            if self.ready_since is None:
                self.ready_since = now
            elif now - self.ready_since >= self.hold_s:
                self.passed = True
                self.completed = True
                return
        else:
            self.ready_since = None

        if now - self.started_at >= self.timeout_s:
            self.completed = True

    def finish(self) -> int:
        print(
            "TAKEOFF_RESULT pass={} target_altitude_m={:.3f} "
            "climb_altitudes_m={}".format(
                self.passed, self.target_altitude_m,
                {agent_id: round(self.climb_altitude_m.get(agent_id, -1.0), 3)
                 for agent_id in self.agent_ids}),
            flush=True,
        )
        self.destroy_node()
        rclpy.shutdown()
        return 0 if self.passed else 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--agents", type=int, required=True)
    parser.add_argument("--target-altitude-m", type=float, default=8.0)
    parser.add_argument("--tolerance-m", type=float, default=0.5)
    parser.add_argument("--hold-s", type=float, default=1.0)
    parser.add_argument("--baseline-warmup-s", type=float, default=2.0)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if (args.agents <= 0 or not math.isfinite(args.target_altitude_m) or
            args.target_altitude_m <= 0.0 or
            not math.isfinite(args.tolerance_m) or args.tolerance_m < 0.0 or
            not math.isfinite(args.hold_s) or args.hold_s < 0.0 or
            not math.isfinite(args.baseline_warmup_s) or
            args.baseline_warmup_s < 0.0 or
            not math.isfinite(args.timeout_s) or args.timeout_s <= 0.0):
        return 2
    rclpy.init()
    waiter = TakeoffAltitudeWaiter(args)
    try:
        while rclpy.ok() and not waiter.completed:
            rclpy.spin_once(waiter, timeout_sec=0.1)
        return waiter.finish()
    except KeyboardInterrupt:
        waiter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 130


if __name__ == "__main__":
    sys.exit(main())

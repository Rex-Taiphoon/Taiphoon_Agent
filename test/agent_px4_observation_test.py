#!/usr/bin/env python3
"""
@file agent_px4_observation_test.py
@brief 在 Docker 開發容器訂閱已連接 PX4 的 Agent 狀態，不發布飛行命令
@author Taiphoon Agent Team
@date 2026-10-02
"""
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from taiphoon_agent.msg import AgentAttitude, AgentState


def main():
    """觀測單台地面未解鎖的 PX4 SIH；至少十秒連續有效狀態與姿態。"""
    rclpy.init()
    node = rclpy.create_node('px4_deployment_observer')
    states = []
    attitudes = []
    subscriptions = [node.create_subscription(AgentState, '/uav_1_agent/state',
                                             lambda value: states.append((time.monotonic(), value)),
                                             qos_profile_sensor_data),
                     node.create_subscription(AgentAttitude, '/uav_1_agent/attitude',
                                              lambda value: attitudes.append((time.monotonic(), value)),
                                              qos_profile_sensor_data)]
    try:
        deadline = time.monotonic() + 40
        observation_start = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if observation_start is None and states and attitudes:
                observation_start = time.monotonic()
            if observation_start is not None and time.monotonic() - observation_start >= 10:
                break
        assert len(states) >= 20 and len(attitudes) >= 20, '狀態或姿態資料缺失／更新不足'
        for _, state in states:
            assert state.online and state.px4_state_valid, 'PX4 狀態無效或 Agent 離線'
            assert not state.armed and state.landed, '飛控不是預期的未解鎖地面狀態'
            assert all(math.isfinite(value) for value in (*state.position_ned, *state.position_global)), '位置無效'
        for _, attitude in attitudes:
            assert all(math.isfinite(value) for value in attitude.q), '姿態含非法值'
        stamps = [value.stamp.sec * 1000000000 + value.stamp.nanosec for _, value in states]
        assert all(right > left for left, right in zip(stamps, stamps[1:])), '狀態時間戳沒有持續前進'
        state_rate = (len(states) - 1) / (states[-1][0] - states[0][0])
        attitude_rate = (len(attitudes) - 1) / (attitudes[-1][0] - attitudes[0][0])
        assert state_rate >= 5 and attitude_rate >= 5, '狀態／姿態更新率低於檢查門檻 5Hz'
        last = states[-1][1]
        result = {'result': 'PASS', 'state_samples': len(states), 'attitude_samples': len(attitudes),
                  'state_hz': round(state_rate, 2), 'attitude_hz': round(attitude_rate, 2),
                  'px4_state_valid': last.px4_state_valid, 'online': last.online, 'armed': last.armed,
                  'landed': last.landed, 'position_ned': [float(value) for value in last.position_ned],
                  'position_global': [float(value) for value in last.position_global],
                  'gcs_peer': '192.168.0.108', 'domain_id': 0}
        if len(sys.argv) > 1:
            Path(sys.argv[1]).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

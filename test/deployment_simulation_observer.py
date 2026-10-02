#!/usr/bin/env python3
"""
@file deployment_simulation_observer.py
@brief Docker 內唯讀驗證一至四組 PX4／Agent 資料，不發布飛行命令
@author Taiphoon Agent Team
@date 2026-10-02
"""
import json
import math
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from taiphoon_agent.msg import AgentState


def main():
    """等待每台都有持續更新且有效的未解鎖狀態；不把容器健康當作飛控連線。"""
    count = int(sys.argv[1])
    assert 1 <= count <= 4
    rclpy.init()
    node = rclpy.create_node('deployment_simulation_observer')
    samples = {identifier: [] for identifier in range(1, count + 1)}
    subscriptions = []
    for identifier in samples:
        subscriptions.append(node.create_subscription(
            AgentState, f'/uav_{identifier}_agent/state',
            lambda value, identifier=identifier: samples[identifier].append((time.monotonic(), value)),
            qos_profile_sensor_data))
    try:
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(len(values) >= 3 and values[-1][1].px4_state_valid
                   and values[-1][1].online and values[-1][1].landed
                   and time.monotonic() - values[-1][0] < 3 for values in samples.values()):
                break
        report = {}
        for identifier, values in samples.items():
            assert len(values) >= 3, f'uav_{identifier}: 缺少持續更新的 Agent 狀態'
            state = values[-1][1]
            assert time.monotonic() - values[-1][0] < 3, f'uav_{identifier}: 狀態沒有繼續更新'
            assert state.px4_state_valid and state.online, f'uav_{identifier}: 未收到有效 PX4 狀態'
            assert all(not value.armed for _, value in values), f'uav_{identifier}: 非預期解鎖'
            assert state.landed, f'uav_{identifier}: 非預期離地'
            assert all(math.isfinite(value) for value in (*state.position_ned, *state.position_global))
            first_stamp = values[0][1].stamp
            assert (state.stamp.sec, state.stamp.nanosec) > (first_stamp.sec, first_stamp.nanosec)
            report[f'uav_{identifier}'] = {'samples': len(values), 'px4_state_valid': True,
                                          'armed': False, 'landed': True}
        print(json.dumps({'result': 'PASS', 'agents': report}, ensure_ascii=False))
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

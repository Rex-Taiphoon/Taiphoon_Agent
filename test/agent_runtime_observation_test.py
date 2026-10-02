#!/usr/bin/env python3
"""
@file agent_runtime_observation_test.py
@brief 在 Docker 開發容器唯讀觀測已啟動 Agent 的 ROS2 節點與介面，不發布控制命令
@author Taiphoon Agent Team
@date 2026-10-02
"""
import json
from pathlib import Path
import sys
import time

import rclpy
from taiphoon_agent.msg import AgentState


def main():
    """預期隔離環境無飛控資料，必有節點／介面存在且不能宣稱 PX4 就緒。"""
    rclpy.init()
    node = rclpy.create_node('deployment_runtime_observer')
    states = []
    subscription = node.create_subscription(AgentState, '/uav_1_agent/state', states.append, 10)
    required_nodes = {'/uav_1/basic_controller_node', '/uav_1/ego_swarm_planner',
                      '/uav_1_agent/agent_supervisor'}
    required_topics = {'/uav_1_agent/state': 'taiphoon_agent/msg/AgentState',
                       '/uav_1_agent/attitude': 'taiphoon_agent/msg/AgentAttitude',
                       '/uav_1_agent/planning/trajectory': 'taiphoon_agent/msg/AgentTrajectory',
                       '/uav_1_agent/command': 'std_msgs/msg/String',
                       '/uav_all_agent/command': 'std_msgs/msg/String'}
    try:
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            names = {namespace.rstrip('/') + '/' + name
                     for name, namespace in node.get_node_names_and_namespaces()}
            topics = dict(node.get_topic_names_and_types())
            if required_nodes <= names and all(type_name in topics.get(topic, [])
                                               for topic, type_name in required_topics.items()):
                break
        assert required_nodes <= names, f'必要節點缺失：{sorted(required_nodes - names)}'
        for topic, type_name in required_topics.items():
            assert type_name in topics.get(topic, []), f'Topic 缺失或型別不符：{topic}'
        assert '/uav_1_agent/obstacle_cloud_simulator' not in names, '部署模式意外啟動模擬感測器'
        state_publishers = node.count_publishers('/uav_1_agent/state')
        assert state_publishers == 1, f'狀態 Publisher 數量異常：{state_publishers}'
        assert node.count_subscribers('/uav_1_agent/command') >= 1, '缺少命令訂閱端'
        observation_end = time.monotonic() + 5
        while time.monotonic() < observation_end:
            rclpy.spin_once(node, timeout_sec=0.2)
        assert not any(state.px4_state_valid or state.armed for state in states), '無飛控時誤報就緒或 Arm'
        result = {'nodes': sorted(required_nodes), 'topics': required_topics,
                  'state_publishers': state_publishers, 'state_messages_received': len(states),
                  'px4_ready': False, 'observation': 'no-flight-controller', 'result': 'PASS'}
        if len(sys.argv) > 1:
            Path(sys.argv[1]).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

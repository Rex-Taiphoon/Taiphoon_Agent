#!/usr/bin/env python3
"""
@file basic_controller.launch.py
@brief Launch BasicController + MissionPlanner with PX4 SITL instances
@author Taiphoon Agent Team
@date 2026-07-28

Usage:
  ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=4

Each node subscribes to:
  - /<agent_namespace>/command     (individual agent commands)
  - /fmu_all_agent/command         (broadcast commands to all agents)
  - /<agent_namespace>/ned_target  (NED target from mission_planner)

MissionPlanner is launched for each drone when drone_count > 1.
It dynamically discovers peers via topic scanning.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context, *args, **kwargs):
    """Create Node actions based on drone_count parameter."""
    drone_count_str = context.perform_substitution(
        LaunchConfiguration("drone_count"))
    try:
        drone_count = int(drone_count_str)
    except (ValueError, TypeError):
        drone_count = 4

    nodes = []
    for i in range(1, drone_count + 1):
        ns = "fmu{}".format(i)
        agent_ns = "fmu{}_agent".format(i)

        # BasicController 節點
        bc_node = Node(
            package="taiphoon_agent",
            executable="basic_controller",
            namespace=ns,
            name="basic_controller_node",
            parameters=[
                {"target_altitude_m": 3.0},
                {"hover_duration_s": 3.0},
                {"startup_delay_s": (i - 1) * 2.0},
                {"target_system": i},
                {"auto_start_sequence": True},
                {"agent_namespace": agent_ns},
                {"collision_safety_margin_m": 3.0},
                {"collision_peer_timeout_s": 2.0},
            ],
            output="screen",
            emulate_tty=True,
        )
        nodes.append(bc_node)

        # MissionPlanner 節點 (僅在多機時啟動)
        # 動態同儕發現，不需要 drone_count 參數
        if drone_count > 1:
            mp_node = Node(
                package="taiphoon_agent",
                executable="mission_planner",
                namespace=ns,
                name="mission_planner",
                parameters=[
                    {"agent_namespace": agent_ns},
                    {"formation_update_rate_hz": 5.0},
                    {"peer_scan_interval_s": 3.0},
                ],
                output="screen",
                emulate_tty=True,
            )
            nodes.append(mp_node)

    return nodes


def generate_launch_description():
    drone_count_arg = DeclareLaunchArgument(
        "drone_count",
        default_value="4",
        description="number of UAV instances to launch"
    )

    return LaunchDescription([
        drone_count_arg,
        OpaqueFunction(function=launch_nodes),
    ])

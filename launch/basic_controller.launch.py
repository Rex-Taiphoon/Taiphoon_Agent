#!/usr/bin/env python3
"""
@file basic_controller.launch.py
@brief Launch multiple BasicController nodes with PX4 SITL instances
@author Taiphoon Agent Team
@date 2026-07-22

Usage:
  ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=3

Each node subscribes to:
  - /<agent_namespace>/command  (individual agent commands)
  - /fmu_all_agent/command      (broadcast commands to all agents)
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
        drone_count = 3

    nodes = []
    for i in range(1, drone_count + 1):
        ns = "fmu{}".format(i)
        agent_ns = "fmu{}_agent".format(i)

        node = Node(
            package="taiphoon_agent",
            executable="basic_controller",
            namespace=ns,
            name="basic_controller_node",
            parameters=[
                {"target_altitude_m": 3.0},
                {"hover_duration_s": 3.0},
                {"startup_delay_s": (i - 1) * 2.0},
                {"target_system": i},
                {"agent_namespace": agent_ns},
            ],
            output="screen",
            emulate_tty=True,
        )
        nodes.append(node)

    return nodes


def generate_launch_description():
    drone_count_arg = DeclareLaunchArgument(
        "drone_count",
        default_value="3",
        description="number of UAV instances to launch"
    )

    return LaunchDescription([
        drone_count_arg,
        OpaqueFunction(function=launch_nodes),
    ])

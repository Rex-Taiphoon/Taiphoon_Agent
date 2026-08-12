#!/usr/bin/env python3
"""
@file basic_controller.launch.py
@brief Launch BasicController、EGO-Swarm planner 與可選 PointCloud2 障礙物模擬器
@author Taiphoon Agent Team
@date 2026-08-10

Usage:
  ros2 launch taiphoon_agent basic_controller.launch.py drone_count:=4
  ros2 launch taiphoon_agent basic_controller.launch.py \
      enable_obstacle_cloud_simulator:=true \
      obstacle_config:=/path/to/obstacles.yaml

每台 Agent 的部署資料流為：
  GCS command -> ego_swarm_planner -> AgentTrajectory -> basic_controller -> PX4

`mission_planner` 與 `trajectory_controller` 不再由本 Launch 啟動。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import fcntl
import os


_AGENT_LAUNCH_LOCKS = {}


def _as_bool(value, default=False):
    """將 LaunchConfiguration 內容轉為明確布林值。"""
    if value is None:
        return default
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    return default


def _acquire_agent_launch_lock(namespace):
    """確保同一容器內同一 Agent namespace 只有一個 launch manager。"""
    lock_path = os.path.join(
        "/tmp", "taiphoon_agent_{}_launch.lock".format(namespace))
    try:
        lock_file = open(lock_path, "a+")
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except (BlockingIOError, OSError):
        try:
            lock_file.close()
        except UnboundLocalError:
            pass
        return False

    _AGENT_LAUNCH_LOCKS[namespace] = lock_file
    return True


def launch_nodes(context, *args, **kwargs):
    """依部署模式建立每個 Agent namespace 的唯一節點集合。"""
    drone_count_str = context.perform_substitution(
        LaunchConfiguration("drone_count"))
    try:
        drone_count = int(drone_count_str)
    except (ValueError, TypeError):
        drone_count = 4

    if drone_count < 1:
        drone_count = 1

    enable_obstacle_cloud = _as_bool(
        context.perform_substitution(
            LaunchConfiguration("enable_obstacle_cloud_simulator")))
    obstacle_config = context.perform_substitution(
        LaunchConfiguration("obstacle_config"))
    startup_delay_base_str = context.perform_substitution(
        LaunchConfiguration("startup_delay_s"))
    try:
        startup_delay_base = float(startup_delay_base_str)
    except (ValueError, TypeError):
        startup_delay_base = 0.0

    single_agent_mode = _as_bool(
        context.perform_substitution(
            LaunchConfiguration("single_agent_mode")))
    target_system_str = context.perform_substitution(
        LaunchConfiguration("target_system"))
    try:
        target_system = int(target_system_str)
    except (ValueError, TypeError):
        target_system = 1
    if target_system < 1:
        target_system = 1

    agent_namespace_override = context.perform_substitution(
        LaunchConfiguration("agent_namespace")).strip()
    agent_indices = ([target_system] if single_agent_mode else
                     list(range(1, drone_count + 1)))

    nodes = []
    for i in agent_indices:
        ns = "fmu{}".format(i)
        agent_ns = agent_namespace_override if (
            single_agent_mode and agent_namespace_override) else \
            "fmu{}_agent".format(i)

        if not _acquire_agent_launch_lock(ns):
            print(
                "[basic_controller.launch] 已略過 /{}："
                "已有 launch manager 持有 Agent lock".format(ns))
            continue

        # BasicController 是唯一發布 PX4 TrajectorySetpoint 的節點。
        bc_node = Node(
            package="taiphoon_agent",
            executable="basic_controller",
            namespace=ns,
            name="basic_controller_node",
            parameters=[
                {"target_altitude_m": LaunchConfiguration(
                    "target_altitude_m")},
                {"hover_duration_s": LaunchConfiguration(
                    "hover_duration_s")},
                {"startup_delay_s": startup_delay_base + (i - 1) * 2.0},
                {"target_system": i},
                {"auto_start_sequence": ParameterValue(
                    LaunchConfiguration("auto_start_sequence"),
                    value_type=bool)},
                {"agent_namespace": agent_ns},
            ],
            output="screen",
            emulate_tty=True,
        )
        nodes.append(bc_node)

        # EGO-Swarm planner：單機也啟動，避免單機與多機使用不同控制鏈。
        planner_node = Node(
            package="taiphoon_agent",
            executable="ego_swarm_planner",
            namespace=ns,
            name="ego_swarm_planner",
            parameters=[
                {"agent_namespace": agent_ns},
                {"agent_index": i},
                {"swarm_origin_agent_index": LaunchConfiguration(
                    "swarm_origin_agent_index")},
                {"peer_trajectory_timeout_s": LaunchConfiguration(
                    "peer_trajectory_timeout_s")},
                {"max_peer_time_skew_s": LaunchConfiguration(
                    "max_peer_time_skew_s")},
                {"planning_rate_hz": LaunchConfiguration(
                    "planning_rate_hz")},
                {"formation_min_distance_m": LaunchConfiguration(
                    "formation_min_distance_m")},
                {"peer_collision_min_distance_m": LaunchConfiguration(
                    "peer_collision_min_distance_m")},
                {"obstacle_clearance_m": LaunchConfiguration(
                    "obstacle_clearance_m")},
                {"knot_interval_s": LaunchConfiguration(
                    "knot_interval_s")},
                {"max_velocity_m_s": LaunchConfiguration(
                    "max_velocity_m_s")},
                {"max_acceleration_m_s2": LaunchConfiguration(
                    "max_acceleration_m_s2")},
                {"candidate_offset_m": LaunchConfiguration(
                    "candidate_offset_m")},
                {"local_arrival_tolerance_m": LaunchConfiguration(
                    "local_arrival_tolerance_m")},
                {"global_arrival_tolerance_m": LaunchConfiguration(
                    "global_arrival_tolerance_m")},
            ],
            output="screen",
            emulate_tty=True,
        )
        nodes.append(planner_node)

        if enable_obstacle_cloud:
            obstacle_node = Node(
                package="taiphoon_agent",
                executable="obstacle_cloud_simulator",
                namespace=agent_ns,
                name="obstacle_cloud_simulator",
                parameters=[
                    {"agent_namespace": agent_ns},
                    {"config_file": obstacle_config},
                    {"publish_rate_hz": LaunchConfiguration(
                        "obstacle_publish_rate_hz")},
                    {"mode": LaunchConfiguration("obstacle_mode")},
                    {"range_m": LaunchConfiguration("obstacle_range_m")},
                    {"horizontal_fov_deg": LaunchConfiguration(
                        "obstacle_horizontal_fov_deg")},
                    {"vertical_fov_deg": LaunchConfiguration(
                        "obstacle_vertical_fov_deg")},
                    {"noise_std_m": LaunchConfiguration(
                        "obstacle_noise_std_m")},
                    {"dropout_ratio": LaunchConfiguration(
                        "obstacle_dropout_ratio")},
                    {"sample_spacing_m": LaunchConfiguration(
                        "obstacle_sample_spacing_m")},
                ],
                output="screen",
                emulate_tty=True,
            )
            nodes.append(obstacle_node)

    return nodes


def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument(
            "drone_count",
            default_value="4",
            description="要啟動的 UAV 數量"),
        DeclareLaunchArgument(
            "single_agent_mode",
            default_value="false",
            description="只啟動指定 target_system 的單一 Agent"),
        DeclareLaunchArgument(
            "target_system",
            default_value="1",
            description="單機模式的 PX4 system id"),
        DeclareLaunchArgument(
            "agent_namespace",
            default_value="",
            description="單機模式的 Agent namespace；空值則依 target_system 推導"),
        DeclareLaunchArgument(
            "target_altitude_m",
            default_value="3.0",
            description="BasicController 預設起飛高度（公尺）"),
        DeclareLaunchArgument(
            "hover_duration_s",
            default_value="3.0",
            description="BasicController 預設懸停時間（秒）"),
        DeclareLaunchArgument(
            "startup_delay_s",
            default_value="0.0",
            description="每台 BasicController 啟動延遲（秒）"),
        DeclareLaunchArgument(
            "auto_start_sequence",
            default_value="true",
            description="是否自動啟動 Offboard sequence"),
        DeclareLaunchArgument(
            "planning_rate_hz",
            default_value="10.0",
            description="EGO-Swarm 規劃頻率（Hz）"),
        DeclareLaunchArgument(
            "swarm_origin_agent_index",
            default_value="1",
            description="固定 swarm_ned 原點的 Agent 索引"),
        DeclareLaunchArgument(
            "peer_trajectory_timeout_s",
            default_value="1.0",
            description="同儕未來軌跡 freshness timeout（秒）"),
        DeclareLaunchArgument(
            "max_peer_time_skew_s",
            default_value="0.5",
            description="同儕軌跡時間戳允許的最大時鐘偏差（秒）"),
        DeclareLaunchArgument(
            "formation_min_distance_m",
            default_value="3.0",
            description="編隊槽位中心到中心最小距離（公尺），可降至 1.5 或 2"),
        DeclareLaunchArgument(
            "knot_interval_s",
            default_value="1.0",
            description="B-spline 最小基準 knot 間隔（秒）"),
        DeclareLaunchArgument(
            "max_velocity_m_s",
            default_value="2.0",
            description="EGO-Swarm 軌跡最大速度（公尺/秒）"),
        DeclareLaunchArgument(
            "max_acceleration_m_s2",
            default_value="1.0",
            description="EGO-Swarm 軌跡最大加速度（公尺/秒^2）"),
        DeclareLaunchArgument(
            "candidate_offset_m",
            default_value="3.0",
            description="局部候選軌跡側向／垂直偏移（公尺）"),
        DeclareLaunchArgument(
            "local_arrival_tolerance_m",
            default_value="0.5",
            description="PX4 local NED 到達容許誤差（公尺）"),
        DeclareLaunchArgument(
            "global_arrival_tolerance_m",
            default_value="1.0",
            description="GPS 共同 NED 到達容許誤差（公尺）"),
        DeclareLaunchArgument(
            "obstacle_clearance_m",
            default_value="1.0",
            description="障礙物安全間距（公尺）"),
        DeclareLaunchArgument(
            "peer_collision_min_distance_m",
            default_value="3.0",
            description="同儕軌跡中心到中心最小距離（公尺），不因資料年齡擴張"),
        DeclareLaunchArgument(
            "enable_obstacle_cloud_simulator",
            default_value="false",
            description="是否啟動 PointCloud2 障礙物模擬器"),
        DeclareLaunchArgument(
            "obstacle_config",
            default_value="config/obstacles/default.yaml",
            description="障礙物 YAML 設定檔路徑"),
        DeclareLaunchArgument(
            "obstacle_publish_rate_hz",
            default_value="15.0",
            description="障礙物 PointCloud2 發布頻率（Hz）"),
        DeclareLaunchArgument(
            "obstacle_mode",
            default_value="sensor_emulation",
            description="障礙物模式：ground_truth_map 或 sensor_emulation"),
        DeclareLaunchArgument(
            "obstacle_range_m",
            default_value="20.0",
            description="障礙物感測範圍（公尺）"),
        DeclareLaunchArgument(
            "obstacle_horizontal_fov_deg",
            default_value="270.0",
            description="障礙物水平視野（度）"),
        DeclareLaunchArgument(
            "obstacle_vertical_fov_deg",
            default_value="90.0",
            description="障礙物垂直視野（度）"),
        DeclareLaunchArgument(
            "obstacle_noise_std_m",
            default_value="0.02",
            description="障礙物點雲位置雜訊標準差（公尺）"),
        DeclareLaunchArgument(
            "obstacle_dropout_ratio",
            default_value="0.05",
            description="障礙物點雲 dropout 比例"),
        DeclareLaunchArgument(
            "obstacle_sample_spacing_m",
            default_value="0.5",
            description="障礙物表面採樣間距（公尺）"),
    ]

    return LaunchDescription(
        launch_arguments + [OpaqueFunction(function=launch_nodes)])

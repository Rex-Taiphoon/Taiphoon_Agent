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
        ns = "uav_{}".format(i)
        agent_ns = agent_namespace_override if (
            single_agent_mode and agent_namespace_override) else \
            "uav_{}_agent".format(i)

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
                {"yaw_rate_limit_deg_s": ParameterValue(
                    LaunchConfiguration("yaw_rate_limit_deg_s"),
                    value_type=float)},
                {"target_system": i},
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
                {"enable_local_goal_planning": ParameterValue(
                    LaunchConfiguration("enable_local_goal_planning"),
                    value_type=bool)},
                {"enable_generic_trajectory_reservation": ParameterValue(
                    LaunchConfiguration("enable_generic_trajectory_reservation"),
                    value_type=bool)},
                {"local_goal_horizon_m": LaunchConfiguration(
                    "local_goal_horizon_m")},
                {"local_goal_detour_margin_m": LaunchConfiguration(
                    "local_goal_detour_margin_m")},
                {"local_goal_replan_ratio": LaunchConfiguration(
                    "local_goal_replan_ratio")},
                {"minimum_replan_interval_s": LaunchConfiguration(
                    "minimum_replan_interval_s")},
                {"peer_trajectory_timeout_s": LaunchConfiguration(
                    "peer_trajectory_timeout_s")},
                {"peer_planning_margin_m": LaunchConfiguration(
                    "peer_planning_margin_m")},
                {"max_peer_time_skew_s": LaunchConfiguration(
                    "max_peer_time_skew_s")},
                {"obstacle_cloud_timeout_s": LaunchConfiguration(
                    "obstacle_cloud_timeout_s")},
                {"require_obstacle_cloud": ParameterValue(
                    LaunchConfiguration("require_obstacle_cloud"),
                    value_type=bool)},
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
                {"max_velocity_mps": LaunchConfiguration(
                    "max_velocity_mps")},
                {"max_acceleration_mps2": LaunchConfiguration(
                    "max_acceleration_mps2")},
                {"max_jerk_mps3": LaunchConfiguration(
                    "max_jerk_mps3")},
                {"candidate_offset_m": LaunchConfiguration(
                    "candidate_offset_m")},
                {"local_arrival_tolerance_m": LaunchConfiguration(
                    "local_arrival_tolerance_m")},
                {"global_arrival_tolerance_m": LaunchConfiguration(
                    "global_arrival_tolerance_m")},
                {"move_attitude_timeout_s": LaunchConfiguration(
                    "move_attitude_timeout_s")},
                {"formation_slot_switch_penalty_m": LaunchConfiguration(
                    "formation_slot_switch_penalty_m")},
                {"formation_member_stable_scan_count": LaunchConfiguration(
                    "formation_member_stable_scan_count")},
                {"planner_backend": LaunchConfiguration("planner_backend")},
                {"optimizer_max_iterations": LaunchConfiguration(
                    "optimizer_max_iterations")},
                {"optimizer_time_budget_ms": LaunchConfiguration(
                    "optimizer_time_budget_ms")},
                {"formation_crossing_penalty_m": LaunchConfiguration(
                    "formation_crossing_penalty_m")},
                {"reservation_lead_time_s": LaunchConfiguration(
                    "reservation_lead_time_s")},
                {"reservation_step_s": LaunchConfiguration(
                    "reservation_step_s")},
                {"reservation_horizon_s": LaunchConfiguration(
                    "reservation_horizon_s")},
                {"enable_peer_deconfliction": ParameterValue(
                    LaunchConfiguration("enable_peer_deconfliction"),
                    value_type=bool)},
                {"peer_deconfliction_release_margin_m": LaunchConfiguration(
                    "peer_deconfliction_release_margin_m")},
                {"peer_deconfliction_max_escape_distance_m": LaunchConfiguration(
                    "peer_deconfliction_max_escape_distance_m")},
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
                    {"agent_index": i},
                    {"swarm_origin_agent_index": LaunchConfiguration(
                        "swarm_origin_agent_index")},
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
            "yaw_rate_limit_deg_s",
            default_value="30.0",
            description=(
                "BasicController 偏航角速率上限（度/秒，1～180）；"
                "PX4 Offboard 不套用 MPC_YAWRAUTO_MAX，需小於等於 MC_YAWRATE_MAX")),
        # 舊版 entrypoint 仍會傳入此參數；保留 launch 介面相容性，但
        # 不讀取、不傳給 BasicController，也不再提供自動啟動流程。
        DeclareLaunchArgument(
            "auto_start_sequence",
            default_value="false",
            description="相容參數；已停用且不再觸發任何自動流程"),
        DeclareLaunchArgument(
            "planning_rate_hz",
            default_value="10.0",
            description="EGO-Swarm 規劃頻率（Hz）"),
        DeclareLaunchArgument(
            "swarm_origin_agent_index",
            default_value="1",
            description="已保留的相容參數；全球 WGS84 軌跡改用各 Agent 自身 Home"),
        DeclareLaunchArgument(
            "enable_local_goal_planning",
            default_value="true",
            description=(
                "是否啟用 Local Goal 與 FIXED_Z 時空路徑引導；"
                "FREE_3D／LINEAR_Z 沿用既有 planner")),
        DeclareLaunchArgument(
            "enable_generic_trajectory_reservation",
            default_value="false",
            description="是否啟用所有 moving trajectory 共用 reservation"),
        DeclareLaunchArgument(
            "local_goal_horizon_m",
            default_value="10.0",
            description="Local Goal 單段距離 horizon（公尺）"),
        DeclareLaunchArgument(
            "local_goal_detour_margin_m",
            default_value="3.0",
            description="Local Goal 側向繞行額外追蹤裕度（公尺）"),
        DeclareLaunchArgument(
            "local_goal_replan_ratio",
            default_value="0.30",
            description="Local Goal progress-based replan 比例"),
        DeclareLaunchArgument(
            "minimum_replan_interval_s",
            default_value="0.5",
            description="正常 progress-based replan 最短間隔（秒）"),
        DeclareLaunchArgument(
            "peer_trajectory_timeout_s",
            default_value="2.0",
            description="同儕未來軌跡 freshness timeout（秒）"),
        DeclareLaunchArgument(
            "peer_planning_margin_m",
            default_value="1.0",
            description="planner 額外保留的機間追蹤裕度（公尺）"),
        DeclareLaunchArgument(
            "max_peer_time_skew_s",
            default_value="1.0",
            description="同儕軌跡時間戳允許的最大時鐘偏差（秒）"),
        DeclareLaunchArgument(
            "obstacle_cloud_timeout_s",
            default_value="1.0",
            description="障礙物地圖 freshness timeout（秒）"),
        DeclareLaunchArgument(
            "require_obstacle_cloud",
            default_value="true",
            description="是否要求 fresh obstacle map 才能規劃"),
        DeclareLaunchArgument(
            "formation_min_distance_m",
            default_value="2.0",
            description="編隊槽位中心到中心最小距離（公尺），可降至 1.5 或 2"),
        DeclareLaunchArgument(
            "knot_interval_s",
            default_value="1.0",
            description="B-spline 最小基準 knot 間隔（秒）"),
        DeclareLaunchArgument(
            "max_velocity_m_s",
            default_value="2.5",
            description="EGO-Swarm 軌跡最大速度（公尺/秒）"),
        DeclareLaunchArgument(
            "max_acceleration_m_s2",
            default_value="1.5",
            description="EGO-Swarm 軌跡最大加速度（公尺/秒^2）"),
        DeclareLaunchArgument(
            "max_velocity_mps",
            default_value="2.5",
            description="動力學驗證最大速度（公尺/秒）"),
        DeclareLaunchArgument(
            "max_acceleration_mps2",
            default_value="1.5",
            description="動力學驗證最大加速度（公尺/秒^2）"),
        DeclareLaunchArgument(
            "max_jerk_mps3",
            default_value="3.0",
            description="動力學驗證最大 jerk（公尺/秒^3）"),
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
            default_value="3.0",
            description=(
                "相容名稱；GPS 投影至本機 planner-local NED 的"
                "到達容許誤差（公尺）")),
        DeclareLaunchArgument(
            "move_attitude_timeout_s",
            default_value="1.0",
            description="水平移動命令可使用的 attitude freshness timeout（秒）"),
        DeclareLaunchArgument(
            "formation_slot_switch_penalty_m",
            default_value="2.0",
            description="編隊 minimum-cost assignment 換槽位成本（公尺）"),
        DeclareLaunchArgument(
            "formation_member_stable_scan_count",
            default_value="2",
            description="成員集合連續穩定 discovery 掃描次數"),
        DeclareLaunchArgument(
            "planner_backend",
            default_value="optimized",
            description="Planner backend：optimized only"),
        DeclareLaunchArgument(
            "optimizer_max_iterations",
            default_value="40",
            description="Optimized backend 最大 solver iteration 次數"),
        DeclareLaunchArgument(
            "optimizer_time_budget_ms",
            default_value="70.0",
            description="單次 optimized backend wall-time budget（毫秒）"),
        DeclareLaunchArgument(
            "formation_crossing_penalty_m",
            default_value="10.0",
            description="編隊 assignment 水平 crossing 等價成本（公尺）"),
        DeclareLaunchArgument(
            "reservation_lead_time_s",
            default_value="0.0",
            description="Future trajectory reservation 最短提前時間（秒）"),
        DeclareLaunchArgument(
            "reservation_step_s",
            default_value="0.5",
            description="Reservation start time 搜尋步進（秒）"),
        DeclareLaunchArgument(
            "reservation_horizon_s",
            default_value="5.0",
            description="Reservation start time 最遠搜尋範圍（秒）"),
        DeclareLaunchArgument(
            "enable_peer_deconfliction",
            default_value="true",
            description="是否啟用已進入 peer hard clearance 的 deterministic recovery"),
        DeclareLaunchArgument(
            "peer_deconfliction_release_margin_m",
            default_value="1.0",
            description="近距離 recovery 結束前需超過 hard clearance 的裕度（公尺）"),
        DeclareLaunchArgument(
            "peer_deconfliction_max_escape_distance_m",
            default_value="12.0",
            description="近距離 recovery 的最大水平脫困距離（公尺）"),
        DeclareLaunchArgument(
            "obstacle_clearance_m",
            default_value="1.0",
            description="障礙物安全間距（公尺）"),
        DeclareLaunchArgument(
            "peer_collision_min_distance_m",
            default_value="2.0",
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
            default_value="explicit_empty",
            description=(
                "障礙物模式：explicit_empty、ground_truth 或 "
                "synthetic_sensor")),
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

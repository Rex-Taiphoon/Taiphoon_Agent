cmake_minimum_required(VERSION 3.8)
project(taiphoon_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 測試依賴 (僅在 BUILD_TESTING 啟用時)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  option(TAIPHOON_BUILD_LEGACY_TESTS "建置未接入現行 runtime 的舊模型相容測試" OFF)
endif()

# --- 自訂訊息 ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NedTarget.msg"
  "msg/AgentState.msg"
  "msg/AgentAttitude.msg"
  "msg/Waypoint.msg"
  "msg/WaypointSequence.msg"
  "msg/AgentTrajectory.msg"
  "msg/PlannerStatus.msg"
  "msg/FormationEpoch.msg"
  "msg/RelativeAgentObservation.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

# 取得自訂訊息 typesupport target (取代已棄用的 rosidl_target_interfaces)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  src
)

# --- BasicController 執行檔 ---
add_executable(basic_controller
  src/main.cpp
  src/controllers/BasicController.cpp
  src/controllers/AgentCommandHandler.cpp
  src/nodes/basic_controller_node.cpp
)

target_include_directories(basic_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(basic_controller
  rclcpp
  px4_msgs
  std_msgs
  builtin_interfaces
  sensor_msgs
)

target_link_libraries(basic_controller ${cpp_typesupport_target})

# --- AgentSupervisor 執行檔 ---
add_executable(agent_supervisor
  src/main_agent_supervisor.cpp
  src/nodes/AgentSupervisor.cpp
)

target_include_directories(agent_supervisor PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(agent_supervisor
  rclcpp
  std_srvs
)

# --- EGO-Swarm Planner 執行檔 ---
add_executable(ego_swarm_planner
  src/main_ego_swarm_planner.cpp
  src/nodes/ego_swarm_planner_node.cpp
  src/ego_swarm/FormationConvergence.cpp
  src/ego_swarm/FormationReservation.cpp
  src/ego_swarm/OptimizedEgoPlanner.cpp
  src/ego_swarm/ProjectedGradientSolver.cpp
  src/ego_swarm/SpatiotemporalRouteInitializer.cpp
  src/perception/RawPointObstacleMap.cpp
)

target_include_directories(ego_swarm_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(ego_swarm_planner
  rclcpp
  std_msgs
  builtin_interfaces
  sensor_msgs
)

target_link_libraries(ego_swarm_planner ${cpp_typesupport_target})

# --- PointCloud2 障礙物模擬器 ---
add_executable(obstacle_cloud_simulator
  src/main_obstacle_cloud_simulator.cpp
  src/nodes/obstacle_cloud_simulator_node.cpp
  src/perception/ObstacleCloudGenerator.cpp
  src/perception/PointCloud2Adapter.cpp
)

target_include_directories(obstacle_cloud_simulator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(obstacle_cloud_simulator
  rclcpp
  sensor_msgs
)

target_link_libraries(obstacle_cloud_simulator ${cpp_typesupport_target})

# --- Docker Desktop 模擬 DDS 同儕單播轉送（部署工具，不連結 ROS2）---
add_executable(dds_peer_relay
  src/main_dds_peer_relay.cpp
)

target_include_directories(dds_peer_relay PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# --- 安裝 ---
install(TARGETS basic_controller agent_supervisor ego_swarm_planner
  obstacle_cloud_simulator dds_peer_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


# --- 單元測試 ---
if(BUILD_TESTING)
  # StatePublisher 測試
  ament_add_gtest(test_state_publisher
    test/test_state_publisher.cpp
  )

  target_include_directories(test_state_publisher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  ament_target_dependencies(test_state_publisher
    rclcpp
    px4_msgs
    builtin_interfaces
  )

  target_link_libraries(test_state_publisher ${cpp_typesupport_target})

  # BasicController 固定位置設定點測試
  ament_add_gtest(test_basic_controller_setpoint
    test/basic_controller_setpoint_test.cpp
    src/controllers/BasicController.cpp
  )

  target_include_directories(test_basic_controller_setpoint PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  ament_target_dependencies(test_basic_controller_setpoint
    rclcpp
    px4_msgs
  )

  # FlightStateResolver 測試（純邏輯，獨立編譯）
  ament_add_gtest(test_flight_state_resolver
    test/flight_state_resolver_test.cpp
  )

  target_include_directories(test_flight_state_resolver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # AgentSupervisor service 測試
  ament_add_gtest(test_agent_supervisor
    test/agent_supervisor_test.cpp
    src/nodes/AgentSupervisor.cpp
  )

  target_include_directories(test_agent_supervisor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  ament_target_dependencies(test_agent_supervisor
    rclcpp
    std_srvs
  )

  # PositionSafetyMonitor 測試 (純邏輯，獨立編譯)
  ament_add_gtest(test_position_safety_monitor
    test/position_safety_monitor_test.cpp
  )

  target_include_directories(test_position_safety_monitor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  if(TAIPHOON_BUILD_LEGACY_TESTS)
    # 舊觀測式防撞與編隊模型不再接入現行 runtime。
    ament_add_gtest(test_collision_safety_monitor
      test/collision_safety_monitor_test.cpp
      src/safety/CollisionSafetyMonitor.cpp
    )

    target_include_directories(test_collision_safety_monitor PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    ament_add_gtest(test_formation_calculator
      test/formation_calculator_test.cpp
    )

    target_include_directories(test_formation_calculator PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
  endif()

  # PeerMatcher 測試 (純邏輯，獨立編譯)
  ament_add_gtest(test_peer_matcher
    test/peer_matcher_test.cpp
  )

  ament_add_gtest(test_agent_naming
    test/agent_naming_test.cpp
  )

  target_include_directories(test_agent_naming PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Docker Desktop 模擬 DDS 同儕轉送埠段純邏輯測試
  ament_add_gtest(test_dds_peer_relay_plan
    test/dds_peer_relay_plan_test.cpp
  )

  target_include_directories(test_dds_peer_relay_plan PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  target_include_directories(test_peer_matcher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  if(TAIPHOON_BUILD_LEGACY_TESTS)
    # 舊 waypoint planner 只在相容驗證時建置。
    ament_add_gtest(test_trajectory_planner
      test/trajectory_planner_test.cpp
    )

    target_include_directories(test_trajectory_planner PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
  endif()

  # EGO-Swarm 核心測試
  ament_add_gtest(test_ego_swarm_core
    test/ego_swarm_core_test.cpp
    src/perception/RawPointObstacleMap.cpp
  )

  target_include_directories(test_ego_swarm_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Local Goal Selector 純邏輯測試
  ament_add_gtest(test_local_goal_selector
    test/local_goal_selector_test.cpp
  )

  target_include_directories(test_local_goal_selector PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Planning Target 與 Local Goal feature gate 純邏輯測試
  ament_add_gtest(test_planning_target_selector
    test/planning_target_selector_test.cpp
  )

  target_include_directories(test_planning_target_selector PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  ament_add_gtest(test_flight_mission_lifecycle
    test/flight_mission_lifecycle_test.cpp
  )
  target_include_directories(test_flight_mission_lifecycle PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # 初次編隊中心高度純邏輯測試
  ament_add_gtest(test_formation_center_policy
    test/formation_center_policy_test.cpp
  )
  target_include_directories(test_formation_center_policy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # FormationEpoch 成員重組與重新加入純邏輯測試
  ament_add_gtest(test_formation_epoch_policy
    test/formation_epoch_policy_test.cpp
  )
  target_include_directories(test_formation_epoch_policy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Planner Feature Flags 純邏輯測試
  ament_add_gtest(test_planner_feature_flags
    test/planner_feature_flags_test.cpp
  )

  target_include_directories(test_planner_feature_flags PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Generic Trajectory Reservation policy 純邏輯測試
  ament_add_gtest(test_trajectory_reservation_policy
    test/trajectory_reservation_policy_test.cpp
  )

  target_include_directories(test_trajectory_reservation_policy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Final／Planning／Active Plan target 語意測試
  ament_add_gtest(test_planner_target_semantics
    test/planner_target_semantics_test.cpp
  )

  target_include_directories(test_planner_target_semantics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Local Goal segment 重規劃 trigger 純邏輯測試
  ament_add_gtest(test_replan_trigger
    test/replan_trigger_test.cpp
  )

  target_include_directories(test_replan_trigger PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Planner candidate acceptance chain 純邏輯測試
  ament_add_gtest(test_planner_acceptance_policy
    test/planner_acceptance_policy_test.cpp
  )

  target_include_directories(test_planner_acceptance_policy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # 即時同儕距離碰撞判定測試
  ament_add_gtest(test_live_peer_distance_policy
    test/live_peer_distance_policy_test.cpp
  )

  target_include_directories(test_live_peer_distance_policy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # 同儕主機時鐘差診斷估計測試
  ament_add_gtest(test_peer_clock_skew_monitor
    test/peer_clock_skew_monitor_test.cpp
  )

  target_include_directories(test_peer_clock_skew_monitor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # 近距離 peer recovery planner 純 C++ 測試
  ament_add_gtest(test_peer_deconfliction_planner
    test/peer_deconfliction_planner_test.cpp
    src/perception/RawPointObstacleMap.cpp
  )

  target_include_directories(test_peer_deconfliction_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Dynamic safety 與 emergency brake 測試（純 C++）
  ament_add_gtest(test_dynamic_safety
    test/dynamic_safety_test.cpp
    src/perception/RawPointObstacleMap.cpp
  )

  target_include_directories(test_dynamic_safety PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Planner seam、optimizer、reservation 與 convergence 測試
  ament_add_gtest(test_optimized_planner
    test/optimized_planner_test.cpp
    src/ego_swarm/FormationConvergence.cpp
    src/ego_swarm/FormationReservation.cpp
    src/ego_swarm/OptimizedEgoPlanner.cpp
    src/ego_swarm/ProjectedGradientSolver.cpp
    src/ego_swarm/SpatiotemporalRouteInitializer.cpp
    src/perception/RawPointObstacleMap.cpp
  )

  target_include_directories(test_optimized_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # FIXED_Z 時空 route initializer 純邏輯測試
  ament_add_gtest(test_spatiotemporal_route_initializer
    test/spatiotemporal_route_initializer_test.cpp
    src/ego_swarm/SpatiotemporalRouteInitializer.cpp
    src/perception/RawPointObstacleMap.cpp
  )

  target_include_directories(test_spatiotemporal_route_initializer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # PointCloud2 幾何資料測試
  ament_add_gtest(test_obstacle_cloud
    test/obstacle_cloud_test.cpp
    src/perception/ObstacleCloudGenerator.cpp
  )

  # LocalObstacleMap freshness 與幾何查詢測試
  ament_add_gtest(test_obstacle_map
    test/obstacle_map_test.cpp
    src/perception/RawPointObstacleMap.cpp
  )

  target_include_directories(test_obstacle_map PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  target_include_directories(test_obstacle_cloud PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # BasicController 定時軌跡測試
  ament_add_gtest(test_basic_controller_trajectory
    test/basic_controller_trajectory_test.cpp
    src/controllers/BasicController.cpp
  )

  target_include_directories(test_basic_controller_trajectory PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  ament_target_dependencies(test_basic_controller_trajectory
    rclcpp
    px4_msgs
  )
endif()

ament_package()

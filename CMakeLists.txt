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

# --- 安裝 ---
install(TARGETS basic_controller agent_supervisor ego_swarm_planner
  obstacle_cloud_simulator
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

  # CollisionSafetyMonitor 測試（純邏輯，獨立編譯）
  ament_add_gtest(test_collision_safety_monitor
    test/collision_safety_monitor_test.cpp
    src/safety/CollisionSafetyMonitor.cpp
  )

  target_include_directories(test_collision_safety_monitor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # FormationCalculator 測試 (純邏輯，獨立編譯)
  ament_add_gtest(test_formation_calculator
    test/formation_calculator_test.cpp
  )

  target_include_directories(test_formation_calculator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # PeerMatcher 測試 (純邏輯，獨立編譯)
  ament_add_gtest(test_peer_matcher
    test/peer_matcher_test.cpp
  )

  target_include_directories(test_peer_matcher PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # TrajectoryPlanner 測試 (純邏輯，獨立編譯)
  ament_add_gtest(test_trajectory_planner
    test/trajectory_planner_test.cpp
  )

  target_include_directories(test_trajectory_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # EGO-Swarm 核心測試
  ament_add_gtest(test_ego_swarm_core
    test/ego_swarm_core_test.cpp
    src/perception/RawPointObstacleMap.cpp
  )

  target_include_directories(test_ego_swarm_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )

  # Phase 4 dynamic safety 與 emergency brake 測試（純 C++）
  ament_add_gtest(test_dynamic_safety
    test/dynamic_safety_test.cpp
    src/perception/RawPointObstacleMap.cpp
  )

  target_include_directories(test_dynamic_safety PRIVATE
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

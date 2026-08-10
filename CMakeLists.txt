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
  src/safety/CollisionSafetyMonitor.cpp
)

target_include_directories(basic_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(basic_controller
  rclcpp
  px4_msgs
  std_msgs
  builtin_interfaces
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

# --- MissionPlanner 執行檔 ---
add_executable(mission_planner
  src/formation/main_mission_planner.cpp
  src/formation/MissionPlanner.cpp
)

target_include_directories(mission_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(mission_planner
  rclcpp
  std_msgs
  builtin_interfaces
)

target_link_libraries(mission_planner ${cpp_typesupport_target})

# --- TrajectoryController 執行檔 ---
add_executable(trajectory_controller
  src/trajectory/main_trajectory_controller.cpp
  src/trajectory/TrajectoryController.cpp
)

target_include_directories(trajectory_controller PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(trajectory_controller
  rclcpp
  std_msgs
  builtin_interfaces
)

target_link_libraries(trajectory_controller ${cpp_typesupport_target})

# --- 安裝 ---
install(TARGETS basic_controller agent_supervisor mission_planner trajectory_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
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
endif()

ament_package()

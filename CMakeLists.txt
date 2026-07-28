cmake_minimum_required(VERSION 3.8)
project(taiphoon_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- 自訂訊息 ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NedTarget.msg"
  DEPENDENCIES std_msgs
)

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
)

rosidl_target_interfaces(basic_controller
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- FormationNode 執行檔 ---
add_executable(formation_node
  src/main_formation.cpp
  src/nodes/formation_node.cpp
)

target_include_directories(formation_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(formation_node
  rclcpp
  px4_msgs
  std_msgs
)

rosidl_target_interfaces(formation_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# --- 安裝 ---
install(TARGETS basic_controller formation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

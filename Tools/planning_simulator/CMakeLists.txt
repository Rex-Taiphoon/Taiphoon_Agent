cmake_minimum_required(VERSION 3.16)
project(taiphoon_planning_simulator LANGUAGES CXX)

if(NOT DEFINED TAIPHOON_AGENT_SOURCE_DIR)
  set(TAIPHOON_AGENT_SOURCE_DIR
      "${CMAKE_CURRENT_LIST_DIR}/../.."
      CACHE PATH "Taiphoon_Agent source directory")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(planning_simulator
  src/planning_simulator.cpp
  "${TAIPHOON_AGENT_SOURCE_DIR}/src/ego_swarm/OptimizedEgoPlanner.cpp"
  "${TAIPHOON_AGENT_SOURCE_DIR}/src/ego_swarm/ProjectedGradientSolver.cpp"
  "${TAIPHOON_AGENT_SOURCE_DIR}/src/perception/RawPointObstacleMap.cpp"
  "${TAIPHOON_AGENT_SOURCE_DIR}/src/perception/ObstacleCloudGenerator.cpp"
)

target_include_directories(planning_simulator PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/src"
  "${TAIPHOON_AGENT_SOURCE_DIR}/src"
)

install(TARGETS planning_simulator DESTINATION bin)

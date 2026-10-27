# @file CMakeLists.txt
# @brief 不依賴 ROS2 的跨平台原生部署入口
# @author Taiphoon Agent Team
# @date 2026-10-02
cmake_minimum_required(VERSION 3.16)
project(taiphoon_agent_launcher LANGUAGES CXX)
set(TAIPHOON_RELEASE_VERSION "v0.0.0-development" CACHE STRING "Release 版本")
add_library(agent_launcher launcher.cpp process.cpp)
target_compile_features(agent_launcher PUBLIC cxx_std_17)
target_include_directories(agent_launcher PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
if(MSVC)
  target_compile_options(agent_launcher PRIVATE /W4 /utf-8)
  set_property(TARGET agent_launcher PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
  target_compile_options(agent_launcher PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(WIN32)
  target_link_libraries(agent_launcher PUBLIC ws2_32)
endif()
add_executable(taiphoon_agent main.cpp)
target_link_libraries(taiphoon_agent PRIVATE agent_launcher)
target_compile_definitions(taiphoon_agent PRIVATE TAIPHOON_RELEASE_VERSION="${TAIPHOON_RELEASE_VERSION}")
if(MSVC)
  target_compile_options(taiphoon_agent PRIVATE /utf-8)
  set_property(TARGET taiphoon_agent PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()
option(BUILD_TESTING "在 Docker 開發容器內建置測試" OFF)
if(BUILD_TESTING)
  enable_testing()
  add_executable(agent_launcher_test ../../test/agent_launcher_test.cpp)
  target_link_libraries(agent_launcher_test PRIVATE agent_launcher)
  add_test(NAME agent_launcher_test COMMAND agent_launcher_test)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME agent_launcher_integration_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../test/agent_launcher_integration_test.py
      $<TARGET_FILE:taiphoon_agent>)
  add_test(NAME deployment_package_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../test/deployment_package_test.py)
  add_test(NAME agent_installer_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../test/agent_installer_test.py)
endif()
install(TARGETS taiphoon_agent RUNTIME DESTINATION .)

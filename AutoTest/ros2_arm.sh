#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash

echo "=== Sending VehicleCommand ARM via ROS2 ==="
ros2 topic pub --once /fmu1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 400, param1: 1.0, param2: 0.0, param7: 0.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}" 2>&1

sleep 2

echo "=== Check arm status ==="
timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>/dev/null | grep flag_armed

echo ""
echo "=== Check command ack ==="
timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_command_ack_v1 --once 2>/dev/null | head -10

echo ""
echo "=== Console ==="
tmux capture-pane -t sitl_0 -p 2>/dev/null | tail -5
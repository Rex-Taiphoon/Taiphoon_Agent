#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash

# Step 1: Start offboard heartbeat in background
ros2 topic pub -r 20 /fmu1/fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode '{timestamp: 0, position: true, velocity: false, acceleration: false, attitude: false, body_rate: false}' > /dev/null 2>&1 &
HB_PID=$!
echo "Offboard heartbeat started (PID: $HB_PID)"
sleep 3

# Step 2: Check if offboard mode is active
echo '=== Control mode ==='
timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>&1 | grep flag_
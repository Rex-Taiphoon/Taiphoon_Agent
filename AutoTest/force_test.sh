#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash

echo "=== Setting SYS_MC_EST_GROUP ==="
tmux send-keys -t sitl_0 "param set SYS_MC_EST_GROUP 2" Enter
sleep 1

echo "=== Force ARM ==="
tmux send-keys -t sitl_0 "commander arm -f" Enter
sleep 3

echo "=== Check status ==="
timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>/dev/null | grep flag_armed

echo ""
echo "=== Console tail ==="
tmux capture-pane -t sitl_0 -p 2>/dev/null | grep -i 'arm\|disarm\|takeoff\|preflight' | tail -5

echo ""
echo "=== Takeoff ==="
tmux send-keys -t sitl_0 "commander takeoff" Enter
sleep 8

echo "=== Position ==="
timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>/dev/null | grep -A3 'position:'
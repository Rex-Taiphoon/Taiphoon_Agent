#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash

LOG_DIR="/home/taiphoon/Taiphoon_Agent/AutoTest/Log"
mkdir -p "$LOG_DIR"
LOGFILE="$LOG_DIR/takeoff_land_test_$(date +%Y%m%d_%H%M%S).log"

exec > >(tee -a "$LOGFILE") 2>&1

echo "============================================"
echo "  Basic Takeoff-Land Test (SIH + ROS2)"
echo "  $(date)"
echo "============================================"

# Step 1: Record initial state
echo ""
echo "--- Initial State ---"
ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>/dev/null | grep -A3 'position:'
ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>/dev/null | grep 'flag_armed'

# Step 2: Arm via commander
echo ""
echo "--- Arming ---"
tmux send-keys -t sitl_0 "commander arm" Enter
sleep 3

echo "Arm status:"
ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>/dev/null | grep 'flag_armed'

# Step 3: Takeoff to 3m
echo ""
echo "--- Takeoff to 3m ---"
tmux send-keys -t sitl_0 "commander takeoff" Enter
sleep 5

echo "Position during takeoff:"
ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>/dev/null | grep -A3 'position:'

# Step 4: Hover 3 seconds (via position hold)
echo ""
echo "--- Hovering (3 seconds) ---"
sleep 3

echo "Position during hover:"
ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>/dev/null | grep -A3 'position:'

# Step 5: Land
echo ""
echo "--- Landing ---"
tmux send-keys -t sitl_0 "commander land" Enter
sleep 5

echo "Position after landing:"
ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>/dev/null | grep -A3 'position:'

echo ""
echo "============================================"
echo "  Test Complete"
echo "============================================"
echo "Log saved to: $LOGFILE"
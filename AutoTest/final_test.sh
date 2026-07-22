#!/bin/bash
set -e

source /opt/ros/humble/setup.bash  
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash

echo "============================================"
echo "  SIH Offboard Mode Verification Test"
echo "============================================"

# 1. Set params needed for offboard without RC
echo ""
echo "[1/5] Setting parameters..."
tmux send-keys -t sitl_0 "param set COM_RCL_EXCEPT 4" Enter
tmux send-keys -t sitl_0 "param set COM_ARM_WO_GPS 1" Enter
sleep 2

# 2. Start offboard heartbeat + position setpoint
echo "[2/5] Starting offboard heartbeat + setpoint..."
ros2 topic pub -r 20 /fmu1/fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode \
  "{timestamp: 0, position: true, velocity: false, acceleration: false, attitude: false, body_rate: false}" &
HB_PID=$!

ros2 topic pub -r 20 /fmu1/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
  "{timestamp: 0, position: [0.0, 0.0, 0.0], velocity: [NaN, NaN, NaN], yaw: NaN}" &
SP_PID=$!

sleep 4

# 3. Check offboard mode
echo "[3/5] Checking offboard mode status..."
MODE=$(timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>&1)
echo "$MODE" | grep -E 'flag_armed|flag_offboard' || echo "(mode info not available)"

IS_OFFBOARD=$(echo "$MODE" | grep 'flag_control_offboard_enabled: true')
if [ -n "$IS_OFFBOARD" ]; then
    echo ">>> OFFBOARD MODE ACTIVE <<<"
else
    echo ">>> Offboard not activating via heartbeat alone - trying commander..."
    tmux send-keys -t sitl_0 "commander mode offboard" Enter
    sleep 3
    MODE2=$(timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>&1)
    echo "$MODE2" | grep -E 'flag_armed|flag_offboard'
fi

# 4. Arm
echo "[4/5] Attempting to arm..."
tmux send-keys -t sitl_0 "commander arm" Enter
sleep 3
MODE3=$(timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>&1)
echo "$MODE3" | grep -E 'flag_armed|flag_offboard'

IS_ARMED=$(echo "$MODE3" | grep 'flag_armed: true')
if [ -n "$IS_ARMED" ]; then
    echo ">>> DRONE ARMED <<<"
else
    echo ">>> Arm failed - checking preflight..."
    tmux capture-pane -t sitl_0 -p 2>/dev/null | grep -i 'arm\|disarm\|preflight\|reject' | tail -5
fi

# 5. Check current state
echo "[5/5] Current vehicle state:"
timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>&1 | grep -A3 'position:'

# Cleanup
kill $HB_PID $SP_PID 2>/dev/null || true
echo ""
echo "============================================"
echo "  Test Complete"
echo "============================================"
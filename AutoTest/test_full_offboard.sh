#!/bin/bash
# Kill previous heartbeat
pkill -f 'ros2 topic pub.*offboard' 2>/dev/null
sleep 1

source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash

# Verify params
echo "=== Checking PX4 params ==="
tmux capture-pane -t sitl_0 -p 2>/dev/null | grep 'param set\|COM_ARM\|COM_RCL\|NAV_RCL' | tail -5

# Publish BOTH offboard heartbeat AND trajectory setpoint
echo ""
echo "=== Starting offboard heartbeat + setpoint ==="

# Heartbeat publisher
ros2 topic pub -r 20 /fmu1/fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode \
  '{timestamp: 0, position: true, velocity: false, acceleration: false, attitude: false, body_rate: false}' > /dev/null 2>&1 &
HB=$!
sleep 0.5

# Setpoint publisher (send current position = stay on ground)
ros2 topic pub -r 20 /fmu1/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
  '{timestamp: 0, position: [0.0, 0.0, 0.0], velocity: [NaN, NaN, NaN], yaw: NaN}' > /dev/null 2>&1 &
SP=$!
echo "Heartbeat PID: $HB, Setpoint PID: $SP"

sleep 3

echo ""
echo "=== Control mode after heartbeat + setpoint ==="
timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>&1 | grep flag_

# Try commander arm
echo ""
echo "=== Arming via commander ==="
tmux send-keys -t sitl_0 "commander arm" Enter
sleep 2

echo ""
echo "=== Control mode after arm ==="
timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>&1 | grep flag_

# Cleanup
kill $HB $SP 2>/dev/null
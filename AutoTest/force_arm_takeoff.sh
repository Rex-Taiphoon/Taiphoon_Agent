#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash

# Step 1: Start offboard heartbeat
ros2 topic pub -r 20 /fmu1/fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode \
  '{timestamp: 0, position: true, velocity: false, acceleration: false, attitude: false, body_rate: false}' > /dev/null 2>&1 &
HB=$!
sleep 1

# Step 2: Force arm via commander
echo "=== Force arming ==="
tmux send-keys -t sitl_0 "commander arm -f" Enter
sleep 3

# Step 3: Check arm status
echo ""
echo "=== Arm status ==="
timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>&1 | grep -E 'flag_armed|flag_offboard'
tmux capture-pane -t sitl_0 -p 2>/dev/null | grep -i 'arm\|disarm\|preflight' | tail -5

# Step 4: Send takeoff setpoint (Z=-3 in NED = 3m up)
echo ""
echo "=== Sending takeoff setpoint ==="
for i in $(seq 1 60); do
    ros2 topic pub --once /fmu1/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
      '{timestamp: 0, position: [0.0, 0.0, -3.0], velocity: [NaN, NaN, NaN], yaw: NaN}' > /dev/null 2>&1
    sleep 0.1
done

# Step 5: Check odometry
echo "=== Odometry ==="
timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>&1 | head -10

# Cleanup
kill $HB 2>/dev/null
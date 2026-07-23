#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash

echo "=== Real-time Takeoff Test ==="

# Start publishers
ros2 topic pub -r 20 /fmu1/fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode \
  "{timestamp: 0, position: true, velocity: false, acceleration: false, attitude: false, body_rate: false}" > /dev/null 2>&1 &
HB=$!

ros2 topic pub -r 20 /fmu1/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
  "{timestamp: 0, position: [0.0, 0.0, 0.0], velocity: [NaN, NaN, NaN], yaw: NaN}" > /dev/null 2>&1 &
SP=$!
sleep 3

# ARM
echo "Arming..."
ros2 topic pub --once /fmu1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 400, param1: 1.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}" > /dev/null 2>&1
sleep 2

echo "Arm status:"
timeout 4 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>/dev/null | grep flag_armed

# TAKEOFF to 3m
echo "Takeoff to 3m..."
kill $SP 2>/dev/null
ros2 topic pub -r 20 /fmu1/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
  "{timestamp: 0, position: [0.0, 0.0, -3.0], velocity: [NaN, NaN, NaN], yaw: NaN}" > /dev/null 2>&1 &
SP2=$!

# Monitor for 8 seconds
for i in 1 2 3 4 5 6 7 8; do
    sleep 1
    Z=$(timeout 3 ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>/dev/null | grep 'position:' -A3 | grep 'z:' || echo "")
    echo "t=${i}s  $Z"
done

# Check altitude
echo ""
echo "Final altitude:"
timeout 4 ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>/dev/null | head -10

# Disarm
echo "Disarming..."
ros2 topic pub --once /fmu1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 400, param1: 0.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}" > /dev/null 2>&1

kill $HB $SP2 2>/dev/null
echo "Done"
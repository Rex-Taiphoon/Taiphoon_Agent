#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
LOG_DIR="/home/taiphoon/Taiphoon_Agent/AutoTest/Log"
mkdir -p "$LOG_DIR"
LOGFILE="$LOG_DIR/full_takeoff_land_$(date +%Y%m%d_%H%M%S).log"

exec > >(tee -a "$LOGFILE") 2>&1

echo "============================================"
echo "  Full Takeoff-Land Test (SIH + ROS2)"
echo "  Target: 3m altitude, 3s hover"
echo "  $(date)"
echo "============================================"

# Start offboard heartbeat
echo ""
echo "[1] Starting offboard heartbeat..."
ros2 topic pub -r 20 /fmu1/fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode \
  "{timestamp: 0, position: true, velocity: false, acceleration: false, attitude: false, body_rate: false}" &
HB_PID=$!
sleep 1

# Start trajectory setpoint (stay at current position first)
echo "[2] Starting trajectory setpoint..."
ros2 topic pub -r 20 /fmu1/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
  "{timestamp: 0, position: [0.0, 0.0, 0.0], velocity: [NaN, NaN, NaN], yaw: NaN}" &
SP_PID=$!
sleep 3

# Switch to offboard mode via VehicleCommand
echo "[3] Switching to offboard mode..."
ros2 topic pub --once /fmu1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 176, param1: 1.0, param2: 6.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}" 2>&1 | tail -1
sleep 2

echo "Mode check:"
timeout 3 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>/dev/null | grep -E 'flag_armed|flag_offboard'

# Arm via VehicleCommand
echo ""
echo "[4] Arming..."
ros2 topic pub --once /fmu1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 400, param1: 1.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}" 2>&1 | tail -1
sleep 2

echo "Arm check:"
timeout 3 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>/dev/null | grep flag_armed

# Takeoff! Set Z=-3.0 (NED: 3m up)
echo ""
echo "[5] TAKEOFF to 3m..."
# Stop old SP, start new one with takeoff position
kill $SP_PID 2>/dev/null
ros2 topic pub -r 20 /fmu1/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
  "{timestamp: 0, position: [0.0, 0.0, -3.0], velocity: [NaN, NaN, NaN], yaw: NaN}" &
SP2_PID=$!

# Monitor altitude
echo "Monitoring altitude..."
for i in $(seq 1 10); do
    sleep 1
    ALT=$(timeout 3 ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>/dev/null | grep 'position:' -A3 | grep '  -' | tail -1 | awk '{print $2}')
    echo "  t=${i}s  Z=${ALT} (NED, -Z = altitude)"
done

# Hover for 3 seconds
echo ""
echo "[6] HOVERING for 3 seconds..."
sleep 3
echo "Hover altitude:"
timeout 3 ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>/dev/null | grep -A3 'position:'

# Land
echo ""
echo "[7] LANDING..."
kill $SP2_PID 2>/dev/null
ros2 topic pub -r 20 /fmu1/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
  "{timestamp: 0, position: [0.0, 0.0, 0.0], velocity: [NaN, NaN, NaN], yaw: NaN}" &
SP3_PID=$!
sleep 5

echo "Landing altitude:"
timeout 3 ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>/dev/null | grep -A3 'position:'

# Disarm
echo ""
echo "[8] Disarming..."
ros2 topic pub --once /fmu1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{timestamp: 0, command: 400, param1: 0.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}" 2>&1 | tail -1

# Cleanup
kill $HB_PID $SP3_PID 2>/dev/null

echo ""
echo "============================================"
echo "  Test Complete!"
echo "  Log: $LOGFILE"
echo "============================================"
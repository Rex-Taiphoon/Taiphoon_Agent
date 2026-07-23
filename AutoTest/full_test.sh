#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash

echo "===== Step 1: Verify params ====="
tmux send-keys -t sitl_0 "param show COM_RCL_EXCEPT" Enter
sleep 1
tmux capture-pane -t sitl_0 -p 2>/dev/null | tail -5

echo ""
echo "===== Step 2: Start offboard heartbeat + setpoint ====="
# Kill any existing publishers
pkill -f 'ros2 topic pub' 2>/dev/null || true
sleep 1

# Start heartbeat publisher
ros2 topic pub -r 20 /fmu1/fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode \
  "{timestamp: 0, position: true, velocity: false, acceleration: false, attitude: false, body_rate: false}" &
PID1=$!

# Start setpoint publisher (stay at current position)
ros2 topic pub -r 20 /fmu1/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
  "{timestamp: 0, position: [0.0, 0.0, 0.0], velocity: [NaN, NaN, NaN], yaw: NaN}" &
PID2=$!

echo "Publishers started: HB=$PID1 SP=$PID2"
echo "Waiting 5 seconds for PX4 to detect offboard..."

sleep 5

echo ""
echo "===== Step 3: Check offboard mode ====="
CTL=$(timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>&1)
echo "$CTL" | grep flag_

OFFBOARD=$(echo "$CTL" | grep 'flag_control_offboard_enabled: true')
ARMED=$(echo "$CTL" | grep 'flag_armed: true')

if [ -n "$OFFBOARD" ]; then
    echo ">>> Offboard mode ACTIVE"
else
    echo ">>> Offboard mode NOT active - checking PX4 console..."
    tmux capture-pane -t sitl_0 -p 2>/dev/null | tail -10
fi

echo ""
echo "===== Step 4: Try arming ====="
tmux send-keys -t sitl_0 "commander arm" Enter
sleep 3

CTL2=$(timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>&1)
echo "$CTL2" | grep flag_

if echo "$CTL2" | grep -q 'flag_armed: true'; then
    echo ">>> DRONE ARMED!"
    
    echo ""
    echo "===== Step 5: Takeoff to 3m ====="
    # Send takeoff setpoint (Z=-3 in NED)
    ros2 topic pub -r 20 /fmu1/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
      "{timestamp: 0, position: [0.0, 0.0, -3.0], velocity: [NaN, NaN, NaN], yaw: NaN}" &
    PID3=$!
    
    sleep 6
    echo "Odometry after takeoff:"
    timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_odometry --once 2>&1 | grep position
    
    # Step 6: Hover 3s then land
    echo ""
    echo "===== Step 6: Hover then Land ====="
    sleep 3
    
    ros2 topic pub -r 20 /fmu1/fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
      "{timestamp: 0, position: [0.0, 0.0, 0.0], velocity: [NaN, NaN, NaN], yaw: NaN}" &
    PID4=$!
    kill $PID3 2>/dev/null
    
    sleep 5
    
    # Disarm
    tmux send-keys -t sitl_0 "commander disarm" Enter
    
    kill $PID4 2>/dev/null
else
    echo ">>> ARM FAILED - check console:"
    tmux capture-pane -t sitl_0 -p 2>/dev/null | grep -i 'arm\|disarm\|reject\|preflight' | tail -5
fi

kill $PID1 $PID2 2>/dev/null
echo "===== Test Complete ====="
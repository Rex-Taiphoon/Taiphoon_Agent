#!/bin/bash
sleep 10  # Wait for PX4 to fully initialize

echo "=== Setting COM_RCL_EXCEPT === "
tmux send-keys -t sitl_0 "param set COM_RCL_EXCEPT 4" Enter
sleep 1

echo "=== Verify param ==="
tmux send-keys -t sitl_0 "param show COM_RCL_EXCEPT" Enter
sleep 1

echo "=== Switch to offboard mode ==="
tmux send-keys -t sitl_0 "commander mode offboard" Enter
sleep 2

echo "=== Check mode ==="
source /opt/ros/humble/setup.bash
source /home/taiphoon/px4_ros_uxrce_dds_ws/install/setup.bash
timeout 5 ros2 topic echo /fmu1/fmu/out/vehicle_control_mode --once 2>&1 | grep flag_offboard

echo ""
echo "=== Console tail ==="
tmux capture-pane -t sitl_0 -p 2>/dev/null | grep -E 'commander|offboard|Offboard|mode|reject|denied|ERROR' | tail -10
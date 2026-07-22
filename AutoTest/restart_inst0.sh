#!/bin/bash
pkill -9 -f "px4 -d -i 0" 2>/dev/null; true
pkill -9 -f "MicroXRCEAgent udp4 -p 8888" 2>/dev/null; true
tmux kill-session -t sitl_0 2>/dev/null; true
tmux kill-session -t agent_0 2>/dev/null; true
sleep 1

source /opt/ros/humble/setup.bash
PX4_BIN="$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4"
PX4_DIR="$HOME/PX4-Autopilot/build/px4_sitl_default"

tmux new-session -d -s "agent_0"
tmux send-keys -t "agent_0" "source /opt/ros/humble/setup.bash && exec MicroXRCEAgent udp4 -p 8888" Enter

tmux new-session -d -s "sitl_0"
tmux send-keys -t "sitl_0" "cd ${PX4_DIR} && PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim PX4_UXRCE_DDS_PORT=8888 PX4_UXRCE_DDS_NS=fmu1 PX4_NET_INTERFACE=eth0 PX4_PARAM_MAV_0_BROADCAST=1 PX4_PARAM_COM_ARM_WO_GPS=1 PX4_PARAM_COM_RCL_EXCEPT=4 PX4_PARAM_NAV_RCL_ACT=0 PX4_HOME_LAT=25.03996592258936 PX4_HOME_LON=121.51195163336611 exec ${PX4_BIN} -d -i 0" Enter

echo "Instance 0 restarted with all params"
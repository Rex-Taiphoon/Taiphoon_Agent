#!/bin/bash
set -e

# Kill everything
pkill -9 -f 'ros2 topic pub' 2>/dev/null || true
pkill -9 -f px4 2>/dev/null || true
pkill -9 -f MicroXRCEAgent 2>/dev/null || true
tmux kill-server 2>/dev/null || true
sleep 2

# Start PX4 with all necessary SIH params
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

for i in 0 1 2; do
    INSTANCE=$i
    PORT=$((8888 + INSTANCE))
    
    read HOME_LAT HOME_LON <<< $(python3 -c "
import math, sys
base_lat = 25.03996592258936
base_lon = 121.51195163336611
instance = int(sys.argv[1])
lon_per_m = 1.0 / (111320.0 * math.cos(math.radians(base_lat)))
offset = instance * 3.0 * lon_per_m
print(f'{base_lat} {base_lon + offset}')
" $INSTANCE)
    
    PX4_BIN="$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4"
    PX4_DIR="$HOME/PX4-Autopilot/build/px4_sitl_default"
    NS_INDEX=$((INSTANCE + 1))
    NAMESPACE="fmu${NS_INDEX}"
    
    mkdir -p /tmp/px4_instance_${INSTANCE}
    
    source /opt/ros/humble/setup.bash
    
    # Start MicroXRCEAgent
    tmux new-session -d -s "agent_${INSTANCE}" 2>/dev/null
    tmux send-keys -t "agent_${INSTANCE}" "source /opt/ros/humble/setup.bash && exec MicroXRCEAgent udp4 -p ${PORT}" Enter
    
    # Start PX4 SITL with ALL needed params for SIH
    tmux new-session -d -s "sitl_${INSTANCE}" 2>/dev/null
    tmux send-keys -t "sitl_${INSTANCE}" \
        "cd ${PX4_DIR} && \
         PX4_SIM_MODEL=sihsim_quadx \
         PX4_SIMULATOR=sihsim \
         PX4_UXRCE_DDS_PORT=${PORT} \
         PX4_UXRCE_DDS_NS=${NAMESPACE} \
         PX4_NET_INTERFACE=eth0 \
         PX4_PARAM_MAV_0_BROADCAST=1 \
         PX4_PARAM_COM_ARM_WO_GPS=1 \
         PX4_PARAM_COM_RCL_EXCEPT=4 \
         PX4_PARAM_NAV_RCL_ACT=0 \
         PX4_PARAM_SYS_MC_EST_GROUP=2 \
         PX4_HOME_LAT=${HOME_LAT} \
         PX4_HOME_LON=${HOME_LON} \
         exec ${PX4_BIN} -d -i ${INSTANCE}" Enter
    
    echo "Instance ${INSTANCE}: port=${PORT}, ns=${NAMESPACE}"
    sleep 2
done

echo "All 3 instances started"
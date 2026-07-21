#!/bin/bash
# start_single_instance.sh - Start one PX4 SITL instance with MicroXRCEAgent
# Usage: ./start_single_instance.sh <instance_id>
#   instance_id: 0, 1, 2 (port 8888, 8889, 8890)

set -e

if [ $# -lt 1 ]; then
    echo "Error: Provide instance ID (0, 1, 2)"
    echo "Usage: $0 <instance_id>"
    exit 1
fi

INSTANCE=$1
PORT=$((8888 + INSTANCE))

# Position: ~3m spacing left to right (lon offset)
# Instance 0 (fmu1) at -3m, Instance 1 (fmu2) at 0m, Instance 2 (fmu3) at +3m
BASE_LAT=25.03996592258936
BASE_LON=121.51195163336611
read HOME_LAT HOME_LON <<< $(python3 -c "
lon_per_m = 1.0 / (111320.0 * 0.9063)
offset = ($INSTANCE - 1) * 3.0 * lon_per_m
print(f\"{25.03996592258936} {121.51195163336611 + offset}\")
")

PX4_BIN=/home/taiphoon/PX4-Autopilot/build/px4_sitl_default/bin/px4
PX4_DIR=/home/taiphoon/PX4-Autopilot/build/px4_sitl_default

# Map instance ID to ROS2 namespace: fmu1, fmu2, fmu3
NS_INDEX=$((INSTANCE + 1))
NAMESPACE="fmu${NS_INDEX}"

mkdir -p /tmp/px4_instance_${INSTANCE}

source /opt/ros/humble/setup.bash

tmux new-session -d -s "agent_${INSTANCE}" 2>/dev/null
tmux send-keys -t "agent_${INSTANCE}" "source /opt/ros/humble/setup.bash && exec MicroXRCEAgent udp4 -p ${PORT}" Enter

tmux new-session -d -s "sitl_${INSTANCE}" 2>/dev/null
tmux send-keys -t "sitl_${INSTANCE}" \
    "cd ${PX4_DIR} && PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim PX4_UXRCE_DDS_PORT=${PORT} PX4_UXRCE_DDS_NS=${NAMESPACE} PX4_NET_INTERFACE=eth0 PX4_PARAM_MAV_0_BROADCAST=1 PX4_HOME_LAT=${HOME_LAT} PX4_HOME_LON=${HOME_LON} exec ${PX4_BIN} -d -i ${INSTANCE}" Enter

echo "Instance ${INSTANCE}: port=${PORT}, ns=${NAMESPACE}, pos=(${HOME_LAT}, ${HOME_LON})"

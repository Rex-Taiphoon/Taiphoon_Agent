#!/bin/bash
# start_single_instance.sh - 啟動單一 PX4 SITL 實例與對應的 MicroXRCEAgent
# 使用方式：bash start_single_instance.sh <實例編號>
#   實例編號：0, 1, 2 ...（埠號依序為 8888, 8889, 8890 ...）
#   位置由左往右依序排列，間距 3 公尺

set -e

if [ $# -lt 1 ]; then
    echo "錯誤：請提供實例編號（0, 1, 2...）"
    echo "使用方式：$0 <實例編號>"
    exit 1
fi

INSTANCE=$1
PORT=$((8888 + INSTANCE))

# 位置計算：由左往右依序排列，間距 3 公尺
# 實例 0 在 (25.03996592258936, 121.51195163336611)，實例 1 往右 3m，依此類推
read HOME_LAT HOME_LON <<< $(python3 -c "
import math, sys
base_lat = 25.03996592258936
base_lon = 121.51195163336611
instance = int(sys.argv[1])
lon_per_m = 1.0 / (111320.0 * math.cos(math.radians(base_lat)))
offset = instance * 3.0 * lon_per_m
print(f'{base_lat} {base_lon + offset}')
" $INSTANCE)

# 使用 $HOME 參考 PX4 路徑，適應不同使用者環境
PX4_BIN="${HOME}/PX4-Autopilot/build/px4_sitl_default/bin/px4"
PX4_DIR="${HOME}/PX4-Autopilot/build/px4_sitl_default"

# 對應 ROS2 命名空間：fmu1, fmu2, fmu3 ...
NS_INDEX=$((INSTANCE + 1))
NAMESPACE="fmu${NS_INDEX}"

mkdir -p /tmp/px4_instance_${INSTANCE}

source /opt/ros/humble/setup.bash

# 啟動 MicroXRCEAgent（在 tmux 工作階段中）
tmux new-session -d -s "agent_${INSTANCE}" 2>/dev/null
tmux send-keys -t "agent_${INSTANCE}" "source /opt/ros/humble/setup.bash && exec MicroXRCEAgent udp4 -p ${PORT}" Enter

# 啟動 PX4 SITL（在 tmux 工作階段中）
tmux new-session -d -s "sitl_${INSTANCE}" 2>/dev/null
tmux send-keys -t "sitl_${INSTANCE}" \
    "cd ${PX4_DIR} && PX4_SIM_MODEL=sihsim_quadx PX4_SIMULATOR=sihsim PX4_UXRCE_DDS_PORT=${PORT} PX4_UXRCE_DDS_NS=${NAMESPACE} PX4_NET_INTERFACE=eth0 PX4_PARAM_MAV_0_BROADCAST=1 PX4_HOME_LAT=${HOME_LAT} PX4_HOME_LON=${HOME_LON} exec ${PX4_BIN} -d -i ${INSTANCE}" Enter

echo "Instance ${INSTANCE}: port=${PORT}, ns=${NAMESPACE}, pos=(${HOME_LAT}, ${HOME_LON})"
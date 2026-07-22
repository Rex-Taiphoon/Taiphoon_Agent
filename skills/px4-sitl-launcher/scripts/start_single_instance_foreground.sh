#!/bin/bash
# start_single_instance_foreground.sh - 在前景啟動單一 PX4 SITL 實例（終端機視窗不會關閉）
# 使用方式：bash start_single_instance_foreground.sh <實例編號>
#   實例編號：0, 1, 2 ...（埠號依序為 8888, 8889, 8890 ...）
#   位置由左往右依序排列，間距 3 公尺
# 此腳本不使用 tmux 啟動 PX4，前景執行，適合獨立終端機視窗

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

PX4_BIN="${HOME}/PX4-Autopilot/build/px4_sitl_default/bin/px4"
PX4_DIR="${HOME}/PX4-Autopilot/build/px4_sitl_default"
NS_INDEX=$((INSTANCE + 1))
NAMESPACE="fmu${NS_INDEX}"

# 檢查 PX4 執行檔是否存在
if [ ! -f "$PX4_BIN" ]; then
    echo "錯誤：找不到 PX4 執行檔"
    echo "路徑：$PX4_BIN"
    echo "請先使用 px4-builder 技能建構 PX4"
    echo ""
    echo "按 Enter 鍵關閉此視窗..."
    read
    exit 1
fi

# 啟動 MicroXRCEAgent（在 tmux 背景）
echo "=== MicroXRCEAgent（tmux agent_${INSTANCE}）==="
source /opt/ros/humble/setup.bash 2>/dev/null || echo "警告：ROS2 環境未安裝"
tmux new-session -d -s "agent_${INSTANCE}" 2>/dev/null
tmux send-keys -t "agent_${INSTANCE}" "source /opt/ros/humble/setup.bash && exec MicroXRCEAgent udp4 -p ${PORT}" Enter
echo "MicroXRCEAgent 已啟動（tmux attach -t agent_${INSTANCE}）"

# 在前景啟動 PX4 SITL
echo ""
echo "=== PX4 SITL Instance ${INSTANCE} ==="
echo "port=${PORT}, ns=${NAMESPACE}, pos=(${HOME_LAT}, ${HOME_LON})"
echo "按 Ctrl+C 可停止此實例"
echo ""

cd ${PX4_DIR}
PX4_SIM_MODEL=sihsim_quadx \
PX4_SIMULATOR=sihsim \
PX4_UXRCE_DDS_PORT=${PORT} \
PX4_UXRCE_DDS_NS=${NAMESPACE} \
PX4_NET_INTERFACE=eth0 \
PX4_PARAM_MAV_0_BROADCAST=1 \
PX4_HOME_LAT=${HOME_LAT} \
PX4_HOME_LON=${HOME_LON} \
exec ${PX4_BIN} -d -i ${INSTANCE}
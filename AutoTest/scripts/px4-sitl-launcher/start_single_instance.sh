#!/bin/bash
# =============================================================================
# start_single_instance.sh - 以 Docker 啟動單一 PX4 SITL 實例
# =============================================================================
# uXRCE-DDS 橋接已內嵌於 Taiphoon_Agent 容器，本腳本僅啟動 PX4 模擬器。
# PX4 的 uXRCE-DDS client 會自動解析 Agent 容器 IP 進行連線。
#
# 相依：需先啟動 Taiphoon_Agent 容器（docker compose up -d）
# =============================================================================
set -e

INSTANCE=""
NETWORK="Taiphoon_Net"
RMW="fastrtps"
FOREGROUND=false
AGENT_CONTAINER="uav_1_agent"
INITIAL_SPACING_M="8.0"
HOME_LAT="25.03996592258936"
HOME_LON="121.51195163336611"
HAS_HOME_LAT=false
HAS_HOME_LON=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --index) INSTANCE="$2"; shift 2 ;;
        --net) NETWORK="$2"; shift 2 ;;
        --rmw) RMW="$2"; shift 2 ;;
        --fg) FOREGROUND=true; shift ;;
        --agent) AGENT_CONTAINER="$2"; shift 2 ;;
        --initial-spacing-m) INITIAL_SPACING_M="$2"; shift 2 ;;
        --home-lat) HOME_LAT="$2"; HAS_HOME_LAT=true; shift 2 ;;
        --home-lon) HOME_LON="$2"; HAS_HOME_LON=true; shift 2 ;;
        *) echo "未知參數：$1"; exit 1 ;;
    esac
done

[ -z "$INSTANCE" ] && { echo "錯誤：請提供實例編號（--index 0）"; exit 1; }
if [ "$HAS_HOME_LAT" != "$HAS_HOME_LON" ]; then
    echo "錯誤：--home-lat 與 --home-lon 必須一起使用"
    exit 1
fi

NUMBER_PATTERN='^-?([0-9]+([.][0-9]*)?|[.][0-9]+)$'
if ! [[ "$HOME_LAT" =~ $NUMBER_PATTERN ]] ||
    ! awk -v value="$HOME_LAT" 'BEGIN { exit !(value > -90.0 && value < 90.0) }'; then
    echo "錯誤：--home-lat 必須是大於 -90 且小於 90 的數值"
    exit 1
fi
if ! [[ "$HOME_LON" =~ $NUMBER_PATTERN ]] ||
    ! awk -v value="$HOME_LON" 'BEGIN { exit !(value >= -180.0 && value <= 180.0) }'; then
    echo "錯誤：--home-lon 必須是 -180 到 180 之間的數值"
    exit 1
fi
if ! [[ "$INITIAL_SPACING_M" =~ ^[0-9]+([.][0-9]+)?$ ]] ||
    [ "$(printf '%s\n' "$INITIAL_SPACING_M" | awk '$1 <= 3.0 {print 1}')" = "1" ]; then
    echo "錯誤：--initial-spacing-m 必須大於 3.0 公尺"
    exit 1
fi

case "$RMW" in
    fastrtps) RMW_ENV="rmw_fastrtps_cpp" ;;
    cyclonedds) RMW_ENV="rmw_cyclonedds_cpp" ;;
    *) echo "錯誤：不支援的 RMW 類型 '$RMW'"; exit 1 ;;
esac

PORT=$((8888 + INSTANCE))
NS_INDEX=$((INSTANCE + 1))
NAMESPACE="uav_${NS_INDEX}"
MAVLINK_PORT=$((18570 + INSTANCE))
SYS_ID=$((INSTANCE + 1))

read -r INSTANCE_HOME_LAT INSTANCE_HOME_LON < <(python3 -c "
import math, sys
base_lat = float(sys.argv[3])
base_lon = float(sys.argv[4])
instance = int(sys.argv[1])
spacing_m = float(sys.argv[2])
lon_per_m = 1.0 / (111320.0 * math.cos(math.radians(base_lat)))
offset = instance * spacing_m * lon_per_m
longitude = (base_lon + offset + 180.0) % 360.0 - 180.0
print(f'{base_lat} {longitude}')
" "$INSTANCE" "$INITIAL_SPACING_M" "$HOME_LAT" "$HOME_LON")

DRONE_NAME="drone${NS_INDEX}"

# 清理舊容器
docker rm -f "$DRONE_NAME" 2>/dev/null || true

# ---- 驗證 Agent 容器是否在線上 ----
if ! docker ps --filter "name=${AGENT_CONTAINER}" --format "{{.Names}}" | grep -q .; then
    echo "警告：Agent 容器 '${AGENT_CONTAINER}' 未執行"
    echo "請先執行：docker compose up -d"
    echo "腳本將繼續啟動 PX4，但 uXRCE-DDS 連線將在 Agent 上線後自動建立"
fi

# ---- 啟動 PX4 SITL ----
echo "  [${DRONE_NAME}] 啟動 PX4 SITL（instance=${INSTANCE}, sys_id=${SYS_ID}, ns=${NAMESPACE}, port=${PORT}）"
echo "  [${DRONE_NAME}] uXRCE-DDS client → ${AGENT_CONTAINER}:${PORT}"

PX4_DOCKER_OPTS=(
    --net="$NETWORK"
    --name="$DRONE_NAME"
    -p "${MAVLINK_PORT}:${MAVLINK_PORT}/udp"
    -e PX4_UXRCE_DDS_PORT="$PORT"
    # 新版 PX4 以 UXRCE_DDS_NS_IDX 決定 namespace；舊版 SITL image 尚未
    # 內建該參數時，使用同值的直接 namespace fallback，確保模擬器輸出一致。
    -e PX4_PARAM_UXRCE_DDS_NS_IDX="$NS_INDEX"
    -e PX4_UXRCE_DDS_NS="$NAMESPACE"
    -e PX4_SIM_MODEL=sihsim_quadx
    -e PX4_NET_INTERFACE=eth0
    -e PX4_SIMULATOR=sihsim
    -e PX4_PARAM_COM_RC_IN_MODE=4
    -e PX4_PARAM_COM_ARM_MIS_REQ=0
    -e PX4_PARAM_MAV_0_BROADCAST=1
    -e PX4_HOME_LAT="$INSTANCE_HOME_LAT"
    -e PX4_HOME_LON="$INSTANCE_HOME_LON"
)

# PX4 entrypoint: 將 uXRCE-DDS client 指向 Agent 容器
PX4_ENTRYPOINT_CMD="
    AGENT_IP=\$(getent ahostsv4 ${AGENT_CONTAINER} | awk '/STREAM/ {print \$1; exit}')
    if [ -n \"\$AGENT_IP\" ]; then
        echo \"[entrypoint] uXRCE-DDS Agent: \$AGENT_IP:${PORT}\"
        sed -i \"s|uxrce_dds_client start -t udp|uxrce_dds_client start -t udp -h \$AGENT_IP -p ${PORT}|\" /opt/px4/etc/init.d-posix/rcS
    else
        echo \"[entrypoint] 警告：無法解析 ${AGENT_CONTAINER}，uXRCE-DDS 將使用預設設定\"
    fi
    HOST_IP=\$(getent ahostsv4 host.docker.internal | awk '/STREAM/ {print \$1; exit}')
    if [ -n \"\$HOST_IP\" ]; then
        sed -i \"s/mavlink start -x -u/mavlink start -x -t \$HOST_IP -u/g\" /opt/px4/etc/init.d-posix/px4-rc.mavlink
    fi
    exec /opt/px4/bin/px4 -d -i ${INSTANCE}
"

if $FOREGROUND; then
    docker run --rm -it \
        "${PX4_DOCKER_OPTS[@]}" \
        --entrypoint /bin/sh \
        px4io/px4-sitl:latest \
        -c "$PX4_ENTRYPOINT_CMD"
else
    docker run -d --rm \
        "${PX4_DOCKER_OPTS[@]}" \
        --entrypoint /bin/sh \
        px4io/px4-sitl:latest \
        -c "$PX4_ENTRYPOINT_CMD"
fi

echo "  [${DRONE_NAME}] uxrce_port=${PORT}, mavlink_port=${MAVLINK_PORT}, sys_id=${SYS_ID}, ns=${NAMESPACE}"
echo "  [${DRONE_NAME}] QGC 連線：udp://localhost:${MAVLINK_PORT}"

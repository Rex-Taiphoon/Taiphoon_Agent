#!/bin/bash
# =============================================================================
# start_single_instance.sh - 以 Docker 啟動單一 PX4 SITL 實例與 uXRCE-DDS Agent
# =============================================================================
set -e

INSTANCE=""
NETWORK="Taiphoon_Net"
RMW="fastrtps"
FOREGROUND=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --index) INSTANCE="$2"; shift 2 ;;
        --net) NETWORK="$2"; shift 2 ;;
        --rmw) RMW="$2"; shift 2 ;;
        --fg) FOREGROUND=true; shift ;;
        *) echo "未知參數：$1"; exit 1 ;;
    esac
done

[ -z "$INSTANCE" ] && { echo "錯誤：請提供實例編號（--index 0）"; exit 1; }

case "$RMW" in
    fastrtps) RMW_ENV="rmw_fastrtps_cpp" ;;
    cyclonedds) RMW_ENV="rmw_cyclonedds_cpp" ;;
    *) echo "錯誤：不支援的 RMW 類型 '$RMW'"; exit 1 ;;
esac

PORT=$((8888 + INSTANCE))
NS_INDEX=$((INSTANCE + 1))
NAMESPACE="fmu${NS_INDEX}"
MAVLINK_PORT=$((18570 + INSTANCE))
SYS_ID=$((INSTANCE + 1))

read HOME_LAT HOME_LON <<< $(python3 -c "
import math, sys
base_lat = 25.03996592258936
base_lon = 121.51195163336611
instance = int(sys.argv[1])
lon_per_m = 1.0 / (111320.0 * math.cos(math.radians(base_lat)))
offset = instance * 3.0 * lon_per_m
print(f'{base_lat} {base_lon + offset}')
" $INSTANCE)

AGENT_NAME="drone${NS_INDEX}-uxrce-dds-agent"
DRONE_NAME="drone${NS_INDEX}"

docker rm -f "$AGENT_NAME" 2>/dev/null || true
docker rm -f "$DRONE_NAME" 2>/dev/null || true

# ---- 步驟 1：啟動 uXRCE-DDS Agent ----
echo "  [${DRONE_NAME}] 啟動 uXRCE-DDS Agent（埠號 ${PORT}，RMW=${RMW}）"
docker run -d --rm \
    --net="$NETWORK" \
    --name="$AGENT_NAME" \
    -e RMW_IMPLEMENTATION="$RMW_ENV" \
    taiphoon/uxrce-agent:latest \
    udp4 -p "$PORT"

# ---- 步驟 2：啟動 PX4 SITL ----
echo "  [${DRONE_NAME}] 啟動 PX4 SITL（instance=${INSTANCE}, sys_id=${SYS_ID}, ns=${NAMESPACE}, mavlink=${MAVLINK_PORT}）"

PX4_DOCKER_OPTS=(
    --net="$NETWORK"
    --name="$DRONE_NAME"
    -p "${MAVLINK_PORT}:${MAVLINK_PORT}/udp"
    -e PX4_UXRCE_DDS_PORT="$PORT"
    -e PX4_UXRCE_DDS_NS="$NAMESPACE"
    -e PX4_SIM_MODEL=sihsim_quadx
    -e PX4_NET_INTERFACE=eth0
    -e PX4_SIMULATOR=sihsim
    -e PX4_PARAM_COM_RC_IN_MODE=4
    -e PX4_PARAM_COM_ARM_MIS_REQ=0
    -e PX4_PARAM_MAV_0_BROADCAST=1
    -e PX4_HOME_LAT="$HOME_LAT"
    -e PX4_HOME_LON="$HOME_LON"
)

if $FOREGROUND; then
    docker run --rm -it \
        "${PX4_DOCKER_OPTS[@]}" \
        --entrypoint /bin/sh \
        px4io/px4-sitl:latest \
        -c "
            AGENT_IP=\$(getent ahostsv4 ${AGENT_NAME} | awk '/STREAM/ {print \$1; exit}')
            HOST_IP=\$(getent ahostsv4 host.docker.internal | awk '/STREAM/ {print \$1; exit}')
            echo \"[entrypoint] uXRCE-DDS Agent: \$AGENT_IP, Host: \$HOST_IP, Instance: ${INSTANCE}\"
            sed -i \"s|uxrce_dds_client start -t udp|uxrce_dds_client start -t udp -h \$AGENT_IP|\" /opt/px4/etc/init.d-posix/rcS
            sed -i \"s/mavlink start -x -u/mavlink start -x -t \$HOST_IP -u/g\" /opt/px4/etc/init.d-posix/px4-rc.mavlink
            exec /opt/px4/bin/px4 -d -i ${INSTANCE}
        "
else
    docker run -d --rm \
        "${PX4_DOCKER_OPTS[@]}" \
        --entrypoint /bin/sh \
        px4io/px4-sitl:latest \
        -c "
            AGENT_IP=\$(getent ahostsv4 ${AGENT_NAME} | awk '/STREAM/ {print \$1; exit}')
            HOST_IP=\$(getent ahostsv4 host.docker.internal | awk '/STREAM/ {print \$1; exit}')
            echo \"[entrypoint] uXRCE-DDS Agent: \$AGENT_IP, Host: \$HOST_IP, Instance: ${INSTANCE}\"
            sed -i \"s|uxrce_dds_client start -t udp|uxrce_dds_client start -t udp -h \$AGENT_IP|\" /opt/px4/etc/init.d-posix/rcS
            sed -i \"s/mavlink start -x -u/mavlink start -x -t \$HOST_IP -u/g\" /opt/px4/etc/init.d-posix/px4-rc.mavlink
            exec /opt/px4/bin/px4 -d -i ${INSTANCE}
        "
fi

echo "  [${DRONE_NAME}] uxrce_port=${PORT}, mavlink_port=${MAVLINK_PORT}, sys_id=${SYS_ID}, ns=${NAMESPACE}"

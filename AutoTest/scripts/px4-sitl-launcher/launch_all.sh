#!/bin/bash
# =============================================================================
# launch_all.sh - 以 Docker 啟動多台 PX4 SITL 模擬器
# =============================================================================
# uXRCE-DDS 橋接已內嵌於 Taiphoon_Agent 容器，本腳本僅管理 PX4 模擬器。
# 使用前請先啟動 Agent 容器：docker compose up -d
#
# 使用方式：
#   bash launch_all.sh --count 4 --net Taiphoon_Net [--rmw fastrtps|cyclonedds] \
#       [--home-lat LAT --home-lon LON]
#
# 參數：
#   --count N      無人機數量（預設 4）
#   --net NETWORK  Docker 網路名稱（預設 Taiphoon_Net）
#   --rmw RMW      RMW 實作（fastrtps 或 cyclonedds，預設 fastrtps）
#   --agent NAME   Agent 容器名稱（預設 uav_1_agent）
#   --initial-spacing-m M  PX4 起始水平間距（預設 8.0，必須大於 3.0）
#   --home-lat LAT          第一台 PX4 的起始緯度，需與 --home-lon 一起使用
#   --home-lon LON          第一台 PX4 的起始經度，需與 --home-lat 一起使用
#   --gcs-host IPV4         QGC 目的 IPv4；省略時沿用 host.docker.internal
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# ---- 預設值 ----
COUNT=4
NETWORK="Taiphoon_Net"
RMW="fastrtps"
AGENT_CONTAINER="uav_1_agent"
INITIAL_SPACING_M="8.0"
HOME_LAT=""
HOME_LON=""
GCS_HOST=""

# ---- 解析參數 ----
while [[ $# -gt 0 ]]; do
    case "$1" in
        --count)
            COUNT="$2"
            shift 2
            ;;
        --net)
            NETWORK="$2"
            shift 2
            ;;
        --rmw)
            RMW="$2"
            shift 2
            ;;
        --agent)
            AGENT_CONTAINER="$2"
            shift 2
            ;;
        --initial-spacing-m)
            INITIAL_SPACING_M="$2"
            shift 2
            ;;
        --home-lat)
            HOME_LAT="$2"
            shift 2
            ;;
        --home-lon)
            HOME_LON="$2"
            shift 2
            ;;
        --gcs-host)
            GCS_HOST="$2"
            shift 2
            ;;
        *)
            echo "未知參數：$1"
            echo "使用方式：$0 --count 4 --net Taiphoon_Net [--rmw fastrtps|cyclonedds]"
            echo "          [--initial-spacing-m 8.0] [--home-lat LAT --home-lon LON]"
            echo "          [--gcs-host IPV4]"
            exit 1
            ;;
    esac
done

if [ -n "$GCS_HOST" ] && ! python3 -c 'import ipaddress, sys; ipaddress.IPv4Address(sys.argv[1])' "$GCS_HOST" >/dev/null 2>&1; then
    echo "錯誤：--gcs-host 必須是合法的 IPv4 位址"
    exit 1
fi

if { [ -n "$HOME_LAT" ] && [ -z "$HOME_LON" ]; } ||
    { [ -z "$HOME_LAT" ] && [ -n "$HOME_LON" ]; }; then
    echo "錯誤：--home-lat 與 --home-lon 必須一起使用"
    exit 1
fi

NUMBER_PATTERN='^-?([0-9]+([.][0-9]*)?|[.][0-9]+)$'
if [ -n "$HOME_LAT" ]; then
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
fi
POSITION_ARGS=()
if [ -n "$HOME_LAT" ]; then
    POSITION_ARGS=(--home-lat "$HOME_LAT" --home-lon "$HOME_LON")
fi
GCS_ARGS=()
if [ -n "$GCS_HOST" ]; then
    GCS_ARGS=(--gcs-host "$GCS_HOST")
fi

# ---- 轉換 RMW 簡稱 ----
case "$RMW" in
    fastrtps)
        RMW_ENV="rmw_fastrtps_cpp"
        ;;
    cyclonedds)
        RMW_ENV="rmw_cyclonedds_cpp"
        ;;
    *)
        echo "錯誤：不支援的 RMW 類型 '$RMW'，請使用 fastrtps 或 cyclonedds"
        exit 1
        ;;
esac

# ---- 確保 Docker 網路存在 ----
if ! docker network inspect "$NETWORK" > /dev/null 2>&1; then
    echo "=== 建立 Docker 網路：${NETWORK} ==="
    docker network create "$NETWORK"
fi

# ---- 啟動前清理舊容器 ----
echo "=== 清理舊有模擬器容器 ==="
"${SCRIPT_DIR}/stop_all.sh" --net "$NETWORK" 2>/dev/null || true
sleep 1
echo ""

echo "=== 啟動 ${COUNT} 台 PX4 SITL 模擬器（網路：${NETWORK}，RMW：${RMW}）==="

for ((i=0; i<COUNT; i++)); do
    echo ""
    echo "--- 啟動 Instance ${i} ---"
    bash "${SCRIPT_DIR}/start_single_instance.sh" \
        --index "$i" \
        --net "$NETWORK" \
        --rmw "$RMW" \
        --agent "uav_$((i+1))_agent" \
        --initial-spacing-m "$INITIAL_SPACING_M" \
        "${GCS_ARGS[@]}" \
        "${POSITION_ARGS[@]}"
    sleep 2
done

echo ""
echo "============================================"
echo " 全部 ${COUNT} 台 PX4 SITL 已啟動完成"
echo "============================================"
echo ""
echo "執行中容器："
docker ps --filter "name=drone" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" 2>/dev/null || echo "(無執行中的 drone 容器)"
echo ""
echo "QGC 連線 port："
for ((i=0; i<COUNT; i++)); do
    MAVLINK_PORT=$((18570 + i))
    echo "  drone$((i+1)): local UDP ${MAVLINK_PORT} -> QGC ${GCS_HOST:-host.docker.internal}:14550"
done
echo ""
echo "檢查 ROS2 topics (在 Agent 容器內)："
echo "  docker exec ${AGENT_CONTAINER} bash -c \"source /opt/ros/humble/setup.bash && ros2 topic list | grep uav\""

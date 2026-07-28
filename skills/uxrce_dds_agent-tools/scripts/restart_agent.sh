#!/bin/bash
# =============================================================================
# restart_agent.sh - 重新啟動 Docker uXRCE-DDS Agent 容器
# =============================================================================
# 使用方式：
#   bash restart_agent.sh --name drone1-uxrce-dds-agent --rmw fastrtps --port 8888 --net Taiphoon_Net
#
# 參數：
#   --name NAME    容器名稱（預設 drone1-uxrce-dds-agent）
#   --rmw RMW      RMW 實作（fastrtps 或 cyclonedds，預設 fastrtps）
#   --port PORT    uXRCE-DDS 監聽埠號（預設 8888）
#   --net NETWORK  Docker 網路名稱（預設 Taiphoon_Net）
# =============================================================================

set -e

# ---- 預設值 ----
AGENT_NAME="drone1-uxrce-dds-agent"
RMW="fastrtps"
PORT=8888
NETWORK="Taiphoon_Net"

# ---- 解析參數 ----
while [[ $# -gt 0 ]]; do
    case "$1" in
        --name)
            AGENT_NAME="$2"
            shift 2
            ;;
        --rmw)
            RMW="$2"
            shift 2
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        --net)
            NETWORK="$2"
            shift 2
            ;;
        *)
            echo "未知參數：$1"
            echo "使用方式：$0 --name drone1-uxrce-dds-agent --rmw fastrtps --port 8888 --net Taiphoon_Net"
            exit 1
            ;;
    esac
done

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

echo "=== 重新啟動 uXRCE-DDS Agent：${AGENT_NAME}（埠號 ${PORT}，RMW=${RMW}）==="

# ---- 停止並移除舊容器 ----
docker stop "$AGENT_NAME" 2>/dev/null || true
docker rm -f "$AGENT_NAME" 2>/dev/null || true

# ---- 重新啟動 ----
docker run -d --rm \
    --net="$NETWORK" \
    --name="$AGENT_NAME" \
    -e RMW_IMPLEMENTATION="$RMW_ENV" \
    taiphoon/uxrce-agent:latest \
    udp4 -p "$PORT"

sleep 1
echo "uXRCE-DDS Agent ${AGENT_NAME} 已重新啟動（埠號 ${PORT}，RMW=${RMW}）"

# ---- 顯示狀態 ----
docker ps --filter "name=${AGENT_NAME}" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"

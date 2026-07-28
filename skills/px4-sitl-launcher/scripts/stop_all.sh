#!/bin/bash
# =============================================================================
# stop_all.sh - 停止所有 Docker PX4 SITL 模擬器與 uXRCE-DDS Agent 容器
# =============================================================================
# 使用方式：
#   bash stop_all.sh [--net Taiphoon_Net]
#
# 參數：
#   --net NETWORK  僅清理指定 Docker 網路的容器（預設清理所有 drone/drone-uxrce-dds-agent）
# =============================================================================

set -e

NETWORK=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --net)
            NETWORK="$2"
            shift 2
            ;;
        *)
            shift
            ;;
    esac
done

echo "=== 停止所有 PX4 SITL 與 uXRCE-DDS Agent 容器 ==="

# ---- 收集要停止的容器名稱 ----
if [ -n "$NETWORK" ]; then
    CONTAINERS=$(docker ps -a --filter "network=$NETWORK" --format '{{.Names}}' 2>/dev/null | grep -E '^drone[0-9]+(-uxrce-dds-agent)?$' || true)
else
    CONTAINERS=$(docker ps -a --format '{{.Names}}' 2>/dev/null | grep -E '^drone[0-9]+(-uxrce-dds-agent)?$' || true)
fi

if [ -z "$CONTAINERS" ]; then
    echo "(無執行中的 drone 容器)"
    exit 0
fi

# ---- 先停止 PX4 容器，再停止 Agent 容器 ----
for name in $CONTAINERS; do
    if [[ "$name" =~ ^drone[0-9]+$ ]]; then
        echo "停止 PX4 容器：$name"
        docker stop "$name" 2>/dev/null || true
    fi
done

for name in $CONTAINERS; do
    if [[ "$name" =~ ^drone[0-9]+-uxrce-dds-agent$ ]]; then
        echo "停止 Agent 容器：$name"
        docker stop "$name" 2>/dev/null || true
    fi
done

sleep 1

# ---- 檢查殘留 ----
REMAINING=$(docker ps --format '{{.Names}}' 2>/dev/null | grep -E '^drone[0-9]+(-uxrce-dds-agent)?$' || true)
if [ -z "$REMAINING" ]; then
    echo "所有 drone 容器已停止。"
else
    echo "仍有殘留容器，強制移除："
    echo "$REMAINING"
    for name in $REMAINING; do
        docker rm -f "$name" 2>/dev/null || true
    done
fi

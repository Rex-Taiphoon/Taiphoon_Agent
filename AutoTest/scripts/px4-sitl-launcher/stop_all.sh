#!/bin/bash
# =============================================================================
# stop_all.sh - 停止所有 Docker PX4 SITL 模擬器容器
# =============================================================================
# 僅清理 PX4 SITL 容器（drone{N}），不影響 Taiphoon_Agent 容器。
#
# 使用方式：
#   bash stop_all.sh [--net Taiphoon_Net]
# =============================================================================

NETWORK="Taiphoon_Net"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --net) NETWORK="$2"; shift 2 ;;
        *) shift ;;
    esac
done

echo "=== 停止所有 PX4 SITL 模擬器容器 ==="

# 停止所有 drone 前綴的容器
DRONE_CONTAINERS=$(docker ps -a --filter "name=drone" --format "{{.Names}}" 2>/dev/null)

if [ -z "$DRONE_CONTAINERS" ]; then
    echo "沒有執行中的 PX4 SITL 容器"
else
    for CONTAINER in $DRONE_CONTAINERS; do
        echo "停止並移除：${CONTAINER}"
        docker rm -f "$CONTAINER" 2>/dev/null || true
    done
fi

echo ""
echo "=== PX4 SITL 容器已全部停止 ==="
echo ""
echo "Taiphoon_Agent 容器不受影響，仍可透過以下指令操作："
echo "  docker compose ps"
echo "  docker compose logs -f"
echo "  docker compose down"

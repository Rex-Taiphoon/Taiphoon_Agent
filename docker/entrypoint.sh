#!/bin/bash
# =============================================================================
# @file entrypoint.sh
# @brief Taiphoon_Agent Docker 容器啟動腳本
# @author Taiphoon Agent Team
# @date 2026-07-23
#
# 啟動時自動 source ROS2、px4_msgs 與 Taiphoon_Agent 的工作空間環境
# =============================================================================

set -e

# ---- 預設環境變數 ----
PX4_UROS_WS="${PX4_UROS_WS:-/home/taiphoon/px4_ros_uxrce_dds_ws}"
TAIPHOON_WS="${TAIPHOON_WS:-/home/taiphoon/Taiphoon_Agent}"

echo "============================================"
echo " Taiphoon_Agent Docker 開發環境"
echo "============================================"

# ---- Source ROS2 Humble ----
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[Entrypoint] ROS2 Humble 環境已載入"
else
    echo "[Entrypoint] 警告：找不到 ROS2 Humble 環境"
fi

# ---- Source px4_msgs 工作空間 ----
if [ -f "${PX4_UROS_WS}/install/setup.bash" ]; then
    source "${PX4_UROS_WS}/install/setup.bash"
    echo "[Entrypoint] px4_msgs 工作空間已載入（${PX4_UROS_WS}）"
else
    echo "[Entrypoint] 提示：px4_msgs 工作空間尚未構建（${PX4_UROS_WS}）"
fi

# ---- Source Taiphoon_Agent 工作空間 ----
if [ -f "${TAIPHOON_WS}/install/setup.bash" ]; then
    source "${TAIPHOON_WS}/install/setup.bash"
    echo "[Entrypoint] Taiphoon_Agent 工作空間已載入（${TAIPHOON_WS}）"
else
    echo "[Entrypoint] 提示：Taiphoon_Agent 尚未構建，請執行 colcon build"
fi

echo "============================================"
echo ""

exec "$@"
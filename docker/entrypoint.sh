#!/bin/bash
# =============================================================================
# @file entrypoint.sh
# @brief Taiphoon_Agent Docker 容器啟動腳本（含 uXRCE-DDS 橋接 + 節點啟動）
# @author Taiphoon Agent Team
# @date 2026-07-29
#
# 啟動流程:
#   1. Source ROS2、px4_msgs 與 Taiphoon_Agent 工作空間
#   2. 背景啟動 MicroXRCEAgent (uXRCE-DDS 橋接)
#   3. 等待 PX4 odometry topic 出現
#   4. 若 topic 就緒 → 啟動 Taiphoon_Agent 節點（BasicController、MissionPlanner、TrajectoryController）
#   5. 若 timeout → 保留 shell 供手動除錯（不 exit）
# =============================================================================

# ---- 預設環境變數 ----
PX4_UROS_WS="${PX4_UROS_WS:-/home/taiphoon/px4_ros_uxrce_dds_ws}"
TAIPHOON_WS="${TAIPHOON_WS:-/home/taiphoon/Taiphoon_Agent}"
UXRCE_PORT="${UXRCE_PORT:-8888}"
TARGET_SYSTEM="${TARGET_SYSTEM:-1}"
AGENT_NAMESPACE="${AGENT_NAMESPACE:-fmu1_agent}"
ODOMETRY_TOPIC="${ODOMETRY_TOPIC:-/fmu1/fmu/out/vehicle_odometry}"
ODOMETRY_TIMEOUT_S="${ODOMETRY_TIMEOUT_S:-30}"

# 從 namespace 提取 fmu{N} 部分作為 ROS2 namespace
FMU_NS="${AGENT_NAMESPACE%_agent}"

echo "============================================"
echo " Taiphoon_Agent Docker 容器啟動"
echo "   UXRCE_PORT=${UXRCE_PORT}"
echo "   TARGET_SYSTEM=${TARGET_SYSTEM}"
echo "   AGENT_NAMESPACE=${AGENT_NAMESPACE}"
echo "============================================"

# ---- Step 1: Source ROS2 Humble ----
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[Entrypoint] ROS2 Humble 環境已載入"
else
    echo "[Entrypoint] 錯誤：找不到 ROS2 Humble 環境"
    exit 1
fi

# ---- Step 2: Source px4_msgs 工作空間 ----
if [ -f "${PX4_UROS_WS}/install/setup.bash" ]; then
    source "${PX4_UROS_WS}/install/setup.bash"
    echo "[Entrypoint] px4_msgs 工作空間已載入"
else
    echo "[Entrypoint] 提示：px4_msgs 工作空間尚未建構"
fi

# ---- Step 3: Source Taiphoon_Agent 工作空間 ----
if [ -f "${TAIPHOON_WS}/install/setup.bash" ]; then
    source "${TAIPHOON_WS}/install/setup.bash"
    echo "[Entrypoint] Taiphoon_Agent 工作空間已載入"
else
    echo "[Entrypoint] 提示：Taiphoon_Agent 尚未建構"
fi

# ---- Step 4: 啟動 MicroXRCEAgent (uXRCE-DDS 橋接) ----
echo "[Entrypoint] 啟動 uXRCE-DDS 橋接 (port=${UXRCE_PORT})..."

MicroXRCEAgent udp4 -p "${UXRCE_PORT}" &
UXRCE_PID=$!

sleep 1
if ! kill -0 ${UXRCE_PID} 2>/dev/null; then
    echo "[Entrypoint] 錯誤：MicroXRCEAgent 啟動失敗，容器仍會保留 shell"
else
    echo "[Entrypoint] MicroXRCEAgent 已啟動 (PID=${UXRCE_PID})"
fi

# ---- Step 5: 等待 odometry topic ----
ODOMETRY_READY=false

if kill -0 ${UXRCE_PID} 2>/dev/null; then
    echo "[Entrypoint] 等待 odometry topic（含 vehicle_odometry）..."
    ELAPSED=0
    while [ ${ELAPSED} -lt ${ODOMETRY_TIMEOUT_S} ]; do
        if ros2 topic list 2>/dev/null | grep -q "vehicle_odometry"; then
            echo "[Entrypoint] Odometry topic 已就緒 (${ELAPSED}s)"
            ODOMETRY_READY=true
            break
        fi
        if ! kill -0 ${UXRCE_PID} 2>/dev/null; then
            echo "[Entrypoint] 警告：MicroXRCEAgent 已終止，可能是 PX4 未啟動或 port 衝突"
            break
        fi
        sleep 1
        ELAPSED=$((ELAPSED + 1))
    done

    if [ "${ODOMETRY_READY}" = false ]; then
        echo "[Entrypoint] 提示：odometry topic 等待逾時 (${ODOMETRY_TIMEOUT_S}s)"
        echo "[Entrypoint] Taiphoon_Agent 節點不會自動啟動，請手動操作"
        echo "[Entrypoint] 若 PX4 尚未啟動，請先啟動模擬器再重新啟動本容器"
    fi
fi

# ---- 確保 binary 與掛載原始碼同步 ----
echo "[Entrypoint] 同步編譯 Taiphoon_Agent..."
cd ${TAIPHOON_WS} && /bin/bash -c "source /opt/ros/humble/setup.bash && source ${PX4_UROS_WS}/install/setup.bash && colcon build --symlink-install 2>&1" | tail -3
echo "[Entrypoint] 編譯完成"

# ---- Step 6: 啟動 Taiphoon_Agent 節點 ----
if [ "${ODOMETRY_READY}" = true ]; then
    echo "[Entrypoint] 啟動 Taiphoon_Agent 節點（namespace=${AGENT_NAMESPACE}）..."

    # BasicController: 飛控橋接 + 命令處理
    ros2 run taiphoon_agent basic_controller \
        --ros-args -r __ns:=/${FMU_NS} \
        -p target_system:=${TARGET_SYSTEM} \
        -p agent_namespace:=${AGENT_NAMESPACE} \
        -p auto_start_sequence:=false \
        &
    BC_PID=$!
    echo "[Entrypoint] BasicController 已啟動 (PID=${BC_PID})"

    # MissionPlanner: 同儕發現 + 編隊計算
    ros2 run taiphoon_agent mission_planner \
        --ros-args -r __ns:=/${AGENT_NAMESPACE} \
        -p agent_namespace:=${AGENT_NAMESPACE} \
        &
    MP_PID=$!
    echo "[Entrypoint] MissionPlanner 已啟動 (PID=${MP_PID})"

    # TrajectoryController: 路徑規劃 + 碰撞檢查
    ros2 run taiphoon_agent trajectory_controller \
        --ros-args -r __ns:=/${AGENT_NAMESPACE} \
        -p agent_namespace:=${AGENT_NAMESPACE} \
        &
    TC_PID=$!
    echo "[Entrypoint] TrajectoryController 已啟動 (PID=${TC_PID})"

    echo "[Entrypoint] 全部節點已啟動 (BC=${BC_PID}, MP=${MP_PID}, TC=${TC_PID})"
else
    echo "[Entrypoint] 略過節點啟動（odometry 未就緒）"
fi

echo "============================================"

# ---- 始終保留互動 shell ----
if [ $# -eq 0 ]; then
    exec bash
else
    exec "$@"
fi

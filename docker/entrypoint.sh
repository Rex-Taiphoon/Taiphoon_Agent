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
#   4. 若 topic 就緒 → 啟動 Taiphoon_Agent 節點
#   5. 若 MicroXRCEAgent crash 則自動重啟，持續等待 topic
#   6. 持續維持 PID 1，直到收到外部 shutdown 訊號
# =============================================================================

# ---- 預設環境變數 ----
PX4_UROS_WS="${PX4_UROS_WS:-/home/taiphoon/px4_ros_uxrce_dds_ws}"
TAIPHOON_WS="${TAIPHOON_WS:-/home/taiphoon/Taiphoon_Agent}"
UXRCE_PORT="${UXRCE_PORT:-8888}"
TARGET_SYSTEM="${TARGET_SYSTEM:-1}"
AGENT_NAMESPACE="${AGENT_NAMESPACE:-fmu1_agent}"
ODOMETRY_TOPIC="${ODOMETRY_TOPIC:-/fmu1/fmu/out/vehicle_odometry}"

# 從 namespace 提取 fmu{N} 部分作為 ROS2 namespace
FMU_NS="${AGENT_NAMESPACE%_agent}"

# PID 1 必須自行處理 SIGTERM；否則容器內的 Bash 可能只忽略訊號，
# 導致遠端 shutdown service 回覆成功但容器仍持續執行。
shutdown_agent() {
    echo "[Entrypoint] 收到 shutdown 訊號，立即結束 Agent 容器"
    trap - TERM INT
    exit 0
}

trap shutdown_agent TERM INT

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

# ---- 啟動 AgentSupervisor（必須早於等待 PX4，確保可遠端關閉）----
echo "[Entrypoint] 啟動 AgentSupervisor（service=/${AGENT_NAMESPACE}/shutdown）..."
ros2 run taiphoon_agent agent_supervisor \
    --ros-args -r __ns:=/${AGENT_NAMESPACE} \
    -p agent_namespace:=${AGENT_NAMESPACE} \
    &
SUPERVISOR_PID=$!
sleep 1
if ! kill -0 "${SUPERVISOR_PID}" 2>/dev/null; then
    echo "[Entrypoint] 錯誤：AgentSupervisor 啟動失敗，無法提供遠端關閉服務"
    exit 1
else
    echo "[Entrypoint] AgentSupervisor 已啟動 (PID=${SUPERVISOR_PID})"
fi

# ---- Step 5: 等待 odometry topic（無限期，含 Agent 自動重啟）----
ODOMETRY_READY=false

echo "[Entrypoint] 等待 odometry topic（含 vehicle_odometry），無限期等待..."
ELAPSED=0
while true; do
    if ros2 topic list 2>/dev/null | grep -q "vehicle_odometry"; then
        echo "[Entrypoint] Odometry topic 已就緒 (${ELAPSED}s)"
        ODOMETRY_READY=true
        break
    fi
    # MicroXRCEAgent crash 時自動重啟
    if ! kill -0 ${UXRCE_PID} 2>/dev/null; then
        echo "[Entrypoint] MicroXRCEAgent 已終止，正在重新啟動..."
        MicroXRCEAgent udp4 -p "${UXRCE_PORT}" &
        UXRCE_PID=$!
        sleep 1
        if kill -0 ${UXRCE_PID} 2>/dev/null; then
            echo "[Entrypoint] MicroXRCEAgent 已重新啟動 (PID=${UXRCE_PID})"
        else
            echo "[Entrypoint] 錯誤：MicroXRCEAgent 重新啟動失敗，稍後重試"
        fi
    fi
    # 每 15 秒印一次狀態
    if [ $((ELAPSED % 15)) -eq 0 ] && [ ${ELAPSED} -gt 0 ]; then
        echo "[Entrypoint] 持續等待 odometry topic... (${ELAPSED}s)"
    fi
    sleep 1
    ELAPSED=$((ELAPSED + 1))
done

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

    echo "[Entrypoint] 全部節點已啟動 (SUP=${SUPERVISOR_PID}, BC=${BC_PID}, MP=${MP_PID}, TC=${TC_PID})"
else
    echo "[Entrypoint] 略過節點啟動（odometry 未就緒）"
fi

echo "============================================"

# ---- 持續維持 PID 1，避免 detached Container 因 stdin/EOF 自行退出 ----
#
# 不在此處啟動互動 bash：docker compose up -d 沒有互動 stdin，bash 會因 EOF
# 立即結束，連帶讓 PID 1 結束 Container。地面站或開發者仍可透過
# `docker compose exec` 開啟除錯 shell。持續迴圈也讓 PID 1 保留 shutdown_agent
# 的 SIGTERM handler，只有外部 shutdown Service 才能讓此腳本退出。
if [ $# -gt 0 ]; then
    "$@" &
    COMMAND_PID=$!
    echo "[Entrypoint] 外部命令已啟動 (PID=${COMMAND_PID})，命令結束後 Container 仍保持運行"
fi

while true; do
    sleep 1
done

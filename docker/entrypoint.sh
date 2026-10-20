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
#   3. 建置共享 workspace（以 lock 序列化）
#   4. 啟動 Taiphoon_Agent 節點，不等待 PX4 data 才建立 subscriptions
#   5. 若 MicroXRCEAgent crash 則自動重啟，持續等待 topic
#   6. 持續維持 PID 1，直到收到外部 shutdown 訊號
# =============================================================================

# ---- 預設環境變數 ----
PX4_UROS_WS="${PX4_UROS_WS:-/home/taiphoon/px4_ros_uxrce_dds_ws}"
TAIPHOON_WS="${TAIPHOON_WS:-/home/taiphoon/Taiphoon_Agent}"
UXRCE_PORT="${UXRCE_PORT:-8888}"
TARGET_SYSTEM="${TARGET_SYSTEM:-1}"
AGENT_NAMESPACE="${AGENT_NAMESPACE:-uav_1_agent}"

# ---- BasicController 部署參數 ----
# 偏航角速率上限（度/秒）；需小於等於 PX4 MC_YAWRATE_MAX
BASIC_CONTROLLER_YAW_RATE_LIMIT_DEG_S="${BASIC_CONTROLLER_YAW_RATE_LIMIT_DEG_S:-30.0}"

# ---- EGO-Swarm 部署參數（僅傳入 Node 已宣告參數）----
EGO_SWARM_PLANNING_RATE_HZ="${EGO_SWARM_PLANNING_RATE_HZ:-10.0}"
EGO_SWARM_FORMATION_MIN_DISTANCE_M="${EGO_SWARM_FORMATION_MIN_DISTANCE_M:-2.0}"
EGO_SWARM_KNOT_INTERVAL_S="${EGO_SWARM_KNOT_INTERVAL_S:-1.0}"
EGO_SWARM_CANDIDATE_OFFSET_M="${EGO_SWARM_CANDIDATE_OFFSET_M:-3.0}"
EGO_SWARM_OBSTACLE_CLEARANCE_M="${EGO_SWARM_OBSTACLE_CLEARANCE_M:-1.0}"
EGO_SWARM_PEER_COLLISION_MIN_DISTANCE_M="${EGO_SWARM_PEER_COLLISION_MIN_DISTANCE_M:-2.0}"
EGO_SWARM_PLANNER_BACKEND="${EGO_SWARM_PLANNER_BACKEND:-optimized}"
EGO_SWARM_ENABLE_GENERIC_TRAJECTORY_RESERVATION="${EGO_SWARM_ENABLE_GENERIC_TRAJECTORY_RESERVATION:-false}"
EGO_SWARM_PEER_PLANNING_MARGIN_M="${EGO_SWARM_PEER_PLANNING_MARGIN_M:-1.0}"
EGO_SWARM_OPTIMIZER_MAX_ITERATIONS="${EGO_SWARM_OPTIMIZER_MAX_ITERATIONS:-40}"
EGO_SWARM_OPTIMIZER_TIME_BUDGET_MS="${EGO_SWARM_OPTIMIZER_TIME_BUDGET_MS:-50.0}"
EGO_SWARM_FORMATION_CROSSING_PENALTY_M="${EGO_SWARM_FORMATION_CROSSING_PENALTY_M:-10.0}"
EGO_SWARM_RESERVATION_LEAD_TIME_S="${EGO_SWARM_RESERVATION_LEAD_TIME_S:-0.0}"
EGO_SWARM_RESERVATION_STEP_S="${EGO_SWARM_RESERVATION_STEP_S:-0.5}"
EGO_SWARM_RESERVATION_HORIZON_S="${EGO_SWARM_RESERVATION_HORIZON_S:-5.0}"

# ---- 可選 PointCloud2 障礙物模擬器（僅傳入 Node 已宣告參數）----
ENABLE_OBSTACLE_CLOUD_SIMULATOR="${ENABLE_OBSTACLE_CLOUD_SIMULATOR:-false}"
OBSTACLE_CLOUD_CONFIG="${OBSTACLE_CLOUD_CONFIG:-${TAIPHOON_WS}/config/obstacles/default.yaml}"
OBSTACLE_CLOUD_PUBLISH_RATE_HZ="${OBSTACLE_CLOUD_PUBLISH_RATE_HZ:-15.0}"
OBSTACLE_CLOUD_MODE="${OBSTACLE_CLOUD_MODE:-explicit_empty}"
OBSTACLE_CLOUD_RANGE_M="${OBSTACLE_CLOUD_RANGE_M:-20.0}"
OBSTACLE_CLOUD_HORIZONTAL_FOV_DEG="${OBSTACLE_CLOUD_HORIZONTAL_FOV_DEG:-270.0}"
OBSTACLE_CLOUD_VERTICAL_FOV_DEG="${OBSTACLE_CLOUD_VERTICAL_FOV_DEG:-90.0}"
OBSTACLE_CLOUD_NOISE_STD_M="${OBSTACLE_CLOUD_NOISE_STD_M:-0.02}"
OBSTACLE_CLOUD_DROPOUT_RATIO="${OBSTACLE_CLOUD_DROPOUT_RATIO:-0.05}"
OBSTACLE_CLOUD_SAMPLE_SPACING_M="${OBSTACLE_CLOUD_SAMPLE_SPACING_M:-0.5}"

BUILD_LOCK_FILE="${TAIPHOON_WS}/log/.taiphoon_agent_build.lock"

# PID 1 必須自行處理 SIGTERM；否則容器內的 Bash 可能只忽略訊號，
# 導致遠端 shutdown service 回覆成功但容器仍持續執行。
shutdown_agent() {
    echo "[Entrypoint] 收到 shutdown 訊號，立即結束 Agent 容器"
    trap - TERM INT
    exit 0
}

trap shutdown_agent TERM INT

build_agent_workspace() {
    if ! command -v flock >/dev/null 2>&1; then
        echo "[Entrypoint] 錯誤：找不到 flock，無法序列化共享 workspace build"
        return 1
    fi

    if ! mkdir -p "${TAIPHOON_WS}/log"; then
        echo "[Entrypoint] 錯誤：無法建立 build lock 目錄：${TAIPHOON_WS}/log"
        return 1
    fi

    (
        if ! exec 9>"${BUILD_LOCK_FILE}"; then
            echo "[Entrypoint] 錯誤：無法建立 build lock：${BUILD_LOCK_FILE}"
            exit 1
        fi

        echo "[Entrypoint] 等待共享 workspace build lock..."
        if ! flock 9; then
            echo "[Entrypoint] 錯誤：無法取得共享 workspace build lock"
            exit 1
        fi

        echo "[Entrypoint] 開始建置 taiphoon_agent（已取得共享 lock）..."
        colcon build --symlink-install \
            --packages-select taiphoon_agent \
            --event-handlers console_direct+
    )
}

# ---- Docker Desktop 模擬：DDS 同儕單播轉送 ----
# 所有 Agent 公告同一個主機 IP 時，CycloneDDS 會把同儕 locator 改寫成本機容器 IP，
# Agent 之間便互相看不到。只有設定 TAIPHOON_DDS_PEER_RELAY_FORWARD（主機區網 IPv4）
# 時才啟動轉送程式，把這些封包轉給同儕容器（無法解析時退回主機埠映射）；
# 實機與 Linux Docker Engine 不受影響。
DDS_PEER_RELAY_BIN="${TAIPHOON_WS}/install/taiphoon_agent/lib/taiphoon_agent/dds_peer_relay"
DDS_PEER_RELAY_PID=""
# 預設值含大括號，不能直接寫在 ${VAR:-...} 內；compose 服務名稱為 uav_<N>_agent
DDS_PEER_RELAY_DEFAULT_HOST_FORMAT='uav_{N}_agent'

start_dds_peer_relay() {
    if [ -z "${TAIPHOON_DDS_PEER_RELAY_FORWARD:-}" ] || [ -n "${DDS_PEER_RELAY_PID}" ]; then
        return 0
    fi
    if [ ! -x "${DDS_PEER_RELAY_BIN}" ]; then
        echo "[Entrypoint] 提示：dds_peer_relay 尚未建置，建置完成後再啟動"
        return 1
    fi
    local bind_address
    bind_address="$(hostname -i 2>/dev/null | awk '{print $1}')"
    if [ -z "${bind_address}" ]; then
        echo "[Entrypoint] 錯誤：無法取得容器 IPv4，Agent 之間將無法互相發現"
        return 1
    fi
    "${DDS_PEER_RELAY_BIN}" --bind "${bind_address}" \
        --forward "${TAIPHOON_DDS_PEER_RELAY_FORWARD}" \
        --domain "${ROS_DOMAIN_ID:-0}" \
        --instance "${TAIPHOON_DDS_PEER_RELAY_INSTANCE:-${TARGET_SYSTEM}}" \
        --count "${TAIPHOON_DDS_PEER_RELAY_COUNT:-4}" \
        --peer-host-format "${TAIPHOON_DDS_PEER_RELAY_HOST_FORMAT:-${DDS_PEER_RELAY_DEFAULT_HOST_FORMAT}}" &
    DDS_PEER_RELAY_PID=$!
    echo "[Entrypoint] DDS 同儕轉送已啟動 (PID=${DDS_PEER_RELAY_PID})"
}

echo "============================================"
echo " Taiphoon_Agent Docker 容器啟動"
echo "   UXRCE_PORT=${UXRCE_PORT}"
echo "   TARGET_SYSTEM=${TARGET_SYSTEM}"
echo "   AGENT_NAMESPACE=${AGENT_NAMESPACE}"
echo "   EGO_SWARM_PLANNING_RATE_HZ=${EGO_SWARM_PLANNING_RATE_HZ}"
echo "   EGO_SWARM_FORMATION_MIN_DISTANCE_M=${EGO_SWARM_FORMATION_MIN_DISTANCE_M}"
echo "   EGO_SWARM_KNOT_INTERVAL_S=${EGO_SWARM_KNOT_INTERVAL_S}"
echo "   EGO_SWARM_CANDIDATE_OFFSET_M=${EGO_SWARM_CANDIDATE_OFFSET_M}"
echo "   EGO_SWARM_PEER_COLLISION_MIN_DISTANCE_M=${EGO_SWARM_PEER_COLLISION_MIN_DISTANCE_M}"
echo "   EGO_SWARM_OBSTACLE_CLEARANCE_M=${EGO_SWARM_OBSTACLE_CLEARANCE_M}"
echo "   EGO_SWARM_PLANNER_BACKEND=${EGO_SWARM_PLANNER_BACKEND}"
echo "   EGO_SWARM_ENABLE_GENERIC_TRAJECTORY_RESERVATION=${EGO_SWARM_ENABLE_GENERIC_TRAJECTORY_RESERVATION}"
echo "   EGO_SWARM_PEER_PLANNING_MARGIN_M=${EGO_SWARM_PEER_PLANNING_MARGIN_M}"
echo "   EGO_SWARM_OPTIMIZER_MAX_ITERATIONS=${EGO_SWARM_OPTIMIZER_MAX_ITERATIONS}"
echo "   EGO_SWARM_OPTIMIZER_TIME_BUDGET_MS=${EGO_SWARM_OPTIMIZER_TIME_BUDGET_MS}"
echo "   ENABLE_OBSTACLE_CLOUD_SIMULATOR=${ENABLE_OBSTACLE_CLOUD_SIMULATOR}"
echo "   OBSTACLE_CLOUD_CONFIG=${OBSTACLE_CLOUD_CONFIG}"
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
# 轉送程式需早於 MicroXRCEAgent 綁定同儕埠，避免其 Fast DDS participant 先占用。
start_dds_peer_relay || true
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

# ---- Step 5: 序列化建置，不以 PX4 topic 存在與否決定節點生命週期 ----
if ! cd "${TAIPHOON_WS}"; then
    echo "[Entrypoint] 錯誤：無法進入工作空間：${TAIPHOON_WS}"
    exit 1
fi

if ! build_agent_workspace; then
    echo "[Entrypoint] 錯誤：taiphoon_agent 建置失敗，略過節點啟動"
    exit 1
fi
echo "[Entrypoint] taiphoon_agent 建置完成"
start_dds_peer_relay || true

# ---- Step 6: 啟動 Agent 節點；PX4 data 未到時由 Node 自行報告 not ready ----
echo "[Entrypoint] 啟動 Agent 節點（namespace=${AGENT_NAMESPACE}）；不等待 PX4 odometry..."

# BasicController、EGO-Swarm 與選配 PointCloud2 模擬器統一由單一 launch
# manager 管理；entrypoint 不再另行 ros2 run BasicController。
ros2 launch taiphoon_agent basic_controller.launch.py \
    single_agent_mode:=true \
    target_system:="${TARGET_SYSTEM}" \
    agent_namespace:="${AGENT_NAMESPACE}" \
    auto_start_sequence:=false \
    yaw_rate_limit_deg_s:="${BASIC_CONTROLLER_YAW_RATE_LIMIT_DEG_S}" \
    planning_rate_hz:="${EGO_SWARM_PLANNING_RATE_HZ}" \
    formation_min_distance_m:="${EGO_SWARM_FORMATION_MIN_DISTANCE_M}" \
    knot_interval_s:="${EGO_SWARM_KNOT_INTERVAL_S}" \
    candidate_offset_m:="${EGO_SWARM_CANDIDATE_OFFSET_M}" \
    obstacle_clearance_m:="${EGO_SWARM_OBSTACLE_CLEARANCE_M}" \
    peer_collision_min_distance_m:="${EGO_SWARM_PEER_COLLISION_MIN_DISTANCE_M}" \
    planner_backend:="${EGO_SWARM_PLANNER_BACKEND}" \
    enable_generic_trajectory_reservation:="${EGO_SWARM_ENABLE_GENERIC_TRAJECTORY_RESERVATION}" \
    peer_planning_margin_m:="${EGO_SWARM_PEER_PLANNING_MARGIN_M}" \
    optimizer_max_iterations:="${EGO_SWARM_OPTIMIZER_MAX_ITERATIONS}" \
    optimizer_time_budget_ms:="${EGO_SWARM_OPTIMIZER_TIME_BUDGET_MS}" \
    formation_crossing_penalty_m:="${EGO_SWARM_FORMATION_CROSSING_PENALTY_M}" \
    reservation_lead_time_s:="${EGO_SWARM_RESERVATION_LEAD_TIME_S}" \
    reservation_step_s:="${EGO_SWARM_RESERVATION_STEP_S}" \
    reservation_horizon_s:="${EGO_SWARM_RESERVATION_HORIZON_S}" \
    enable_obstacle_cloud_simulator:="${ENABLE_OBSTACLE_CLOUD_SIMULATOR}" \
    obstacle_config:="${OBSTACLE_CLOUD_CONFIG}" \
    obstacle_publish_rate_hz:="${OBSTACLE_CLOUD_PUBLISH_RATE_HZ}" \
    obstacle_mode:="${OBSTACLE_CLOUD_MODE}" \
    obstacle_range_m:="${OBSTACLE_CLOUD_RANGE_M}" \
    obstacle_horizontal_fov_deg:="${OBSTACLE_CLOUD_HORIZONTAL_FOV_DEG}" \
    obstacle_vertical_fov_deg:="${OBSTACLE_CLOUD_VERTICAL_FOV_DEG}" \
    obstacle_noise_std_m:="${OBSTACLE_CLOUD_NOISE_STD_M}" \
    obstacle_dropout_ratio:="${OBSTACLE_CLOUD_DROPOUT_RATIO}" \
    obstacle_sample_spacing_m:="${OBSTACLE_CLOUD_SAMPLE_SPACING_M}" \
    &
AGENT_LAUNCH_PID=$!
echo "[Entrypoint] 單一 Agent launch manager 已啟動 (PID=${AGENT_LAUNCH_PID})"
echo "[Entrypoint] 全部節點已交由單一 launch manager 管理 (SUP=${SUPERVISOR_PID}, LAUNCH=${AGENT_LAUNCH_PID})"

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

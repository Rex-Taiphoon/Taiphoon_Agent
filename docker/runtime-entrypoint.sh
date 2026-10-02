#!/usr/bin/env bash
# @file runtime-entrypoint.sh
# @brief 預編譯映像專用入口；不編譯、不等待飛控、不自動起飛
# @author Taiphoon Agent Team
# @date 2026-10-02
set -eo pipefail

TAIPHOON_WS="${TAIPHOON_WS:-/home/taiphoon/Taiphoon_Agent}"
PX4_UROS_WS="${PX4_UROS_WS:-/home/taiphoon/px4_ros_uxrce_dds_ws}"
TARGET_SYSTEM="${TARGET_SYSTEM:-1}"
AGENT_NAMESPACE="${AGENT_NAMESPACE:-uav_${TARGET_SYSTEM}_agent}"
UXRCE_PORT="${UXRCE_PORT:-8888}"
XRCE_TRANSPORT="${XRCE_TRANSPORT:-udp}"
declare -a CHILDREN=()

# @brief 正常停止所有自有程序；不刪除資料或自動重啟飛行服務。
shutdown_runtime()
{
    trap - TERM INT EXIT
    if [ "${#CHILDREN[@]}" -gt 0 ]; then
        kill -TERM "${CHILDREN[@]}" 2>/dev/null || true
        wait "${CHILDREN[@]}" 2>/dev/null || true
    fi
}
trap shutdown_runtime EXIT
trap 'exit 0' TERM INT

for setup in /opt/ros/humble/setup.bash \
    "${PX4_UROS_WS}/install/setup.bash" "${TAIPHOON_WS}/install/setup.bash"; do
    if [ ! -r "${setup}" ]; then
        echo "[Runtime] 缺少預編譯工作空間：${setup}；開發模式請先執行 build" >&2
        exit 1
    fi
    source "${setup}"
done

case "${XRCE_TRANSPORT}" in
    udp)
        MicroXRCEAgent udp4 -p "${UXRCE_PORT}" &
        ;;
    serial)
        if [ ! -c "${XRCE_SERIAL_DEVICE:-}" ]; then
            echo "[Runtime] 找不到串列裝置：${XRCE_SERIAL_DEVICE:-未設定}" >&2
            exit 1
        fi
        MicroXRCEAgent serial --dev "${XRCE_SERIAL_DEVICE}" -b "${XRCE_SERIAL_BAUD:-921600}" &
        ;;
    *)
        echo "[Runtime] 不支援 XRCE_TRANSPORT=${XRCE_TRANSPORT}" >&2
        exit 1
        ;;
esac
CHILDREN+=("$!")

ros2 run taiphoon_agent agent_supervisor \
    --ros-args -r "__ns:=/${AGENT_NAMESPACE}" -p "agent_namespace:=${AGENT_NAMESPACE}" &
CHILDREN+=("$!")

# 與既有 launch 相同的參數預設，部署只改變程序啟動與建置責任。
ros2 launch taiphoon_agent basic_controller.launch.py \
    single_agent_mode:=true target_system:="${TARGET_SYSTEM}" agent_namespace:="${AGENT_NAMESPACE}" \
    auto_start_sequence:=false \
    planning_rate_hz:="${EGO_SWARM_PLANNING_RATE_HZ:-10.0}" \
    formation_min_distance_m:="${EGO_SWARM_FORMATION_MIN_DISTANCE_M:-3.0}" \
    knot_interval_s:="${EGO_SWARM_KNOT_INTERVAL_S:-1.0}" \
    candidate_offset_m:="${EGO_SWARM_CANDIDATE_OFFSET_M:-3.0}" \
    obstacle_clearance_m:="${EGO_SWARM_OBSTACLE_CLEARANCE_M:-1.0}" \
    peer_collision_min_distance_m:="${EGO_SWARM_PEER_COLLISION_MIN_DISTANCE_M:-3.0}" \
    planner_backend:="${EGO_SWARM_PLANNER_BACKEND:-optimized}" \
    enable_generic_trajectory_reservation:="${EGO_SWARM_ENABLE_GENERIC_TRAJECTORY_RESERVATION:-false}" \
    peer_planning_margin_m:="${EGO_SWARM_PEER_PLANNING_MARGIN_M:-1.0}" \
    optimizer_max_iterations:="${EGO_SWARM_OPTIMIZER_MAX_ITERATIONS:-40}" \
    optimizer_time_budget_ms:="${EGO_SWARM_OPTIMIZER_TIME_BUDGET_MS:-50.0}" \
    formation_crossing_penalty_m:="${EGO_SWARM_FORMATION_CROSSING_PENALTY_M:-10.0}" \
    reservation_lead_time_s:="${EGO_SWARM_RESERVATION_LEAD_TIME_S:-0.5}" \
    reservation_step_s:="${EGO_SWARM_RESERVATION_STEP_S:-0.5}" \
    reservation_horizon_s:="${EGO_SWARM_RESERVATION_HORIZON_S:-5.0}" \
    enable_obstacle_cloud_simulator:="${ENABLE_OBSTACLE_CLOUD_SIMULATOR:-false}" \
    obstacle_config:="${OBSTACLE_CLOUD_CONFIG:-${TAIPHOON_WS}/config/obstacles/default.yaml}" \
    obstacle_publish_rate_hz:="${OBSTACLE_CLOUD_PUBLISH_RATE_HZ:-15.0}" \
    obstacle_mode:="${OBSTACLE_CLOUD_MODE:-explicit_empty}" \
    obstacle_range_m:="${OBSTACLE_CLOUD_RANGE_M:-20.0}" \
    obstacle_horizontal_fov_deg:="${OBSTACLE_CLOUD_HORIZONTAL_FOV_DEG:-270.0}" \
    obstacle_vertical_fov_deg:="${OBSTACLE_CLOUD_VERTICAL_FOV_DEG:-90.0}" \
    obstacle_noise_std_m:="${OBSTACLE_CLOUD_NOISE_STD_M:-0.02}" \
    obstacle_dropout_ratio:="${OBSTACLE_CLOUD_DROPOUT_RATIO:-0.05}" \
    obstacle_sample_spacing_m:="${OBSTACLE_CLOUD_SAMPLE_SPACING_M:-0.5}" &
CHILDREN+=("$!")
printf '%s\n' "${CHILDREN[@]}" > /tmp/taiphoon-agent.pids
echo "[Runtime] /${AGENT_NAMESPACE} 程序已建立；飛控連線請使用 taiphoon_agent status 檢查"

has_healthy_nodes=false
while sleep 1; do
    for child in "${CHILDREN[@]}"; do
        if ! kill -0 "${child}" 2>/dev/null; then
            echo "[Runtime] 必要程序 PID=${child} 已停止，結束服務" >&2
            exit 1
        fi
    done
    if pgrep -f '/lib/taiphoon_agent/basic_controller' >/dev/null \
        && pgrep -f '/lib/taiphoon_agent/ego_swarm_planner' >/dev/null; then
        has_healthy_nodes=true
    elif [ "${has_healthy_nodes}" = true ]; then
        echo "[Runtime] 必要 ROS2 節點已停止，結束服務" >&2
        exit 1
    fi
done
